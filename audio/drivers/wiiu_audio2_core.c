/*  RetroArch - A frontend for libretro.
 *  Copyright (C) 2023-2024 - vrda
 *
 *  RetroArch is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  RetroArch is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with RetroArch.
 *  If not, see <http://www.gnu.org/licenses/>.
 */
#include "wiiu_audio2.h"


// just couldn't resist using global data, at least separate the cache lines
typedef struct GlobalDataAudioCore
{   AXProAudioCore*         ac;
    uint32_t                unusedu[7];
//---
    uint8_t                 thread_stack[AXPRO_THREAD_STACKSIZE];
    OSThread                thread_handle;

} GlobalDataAudioCore; AXPRO_ALIGNED_DECLARE_CLINE2(GlobalDataAudioCore ga);



// interrupt handler
//----------------------------------------------------

static inline void _axpro_continue_playing(AXProFrameCallbackData* idat)
{   AXSetVoiceState(idat->voice_l, AX_VOICE_STATE_PLAYING);
    AXSetVoiceState(idat->voice_r, AX_VOICE_STATE_PLAYING);
}

static inline void _axpro_stop_playing(AXProFrameCallbackData* idat)
{   AXSetVoiceState(idat->voice_l, AX_VOICE_STATE_STOPPED);
    AXSetVoiceState(idat->voice_r, AX_VOICE_STATE_STOPPED);
}

static inline void _axpro_begin_playing(AXProFrameCallbackData* idat)
{   //print_irq("bufferred %d blocks, starting play\n", idat->prepared_streamblocks);
    idat->prev_loopcount = AXGetVoiceLoopCount(idat->voice_l);
    _axpro_continue_playing(idat);
    // TODO: what about fade in ?
}


// adjust the rate to keep the playback continuous rather than doing short stops
#if AXPRO_RATE_CONTROL_AMORTIZER
static inline void _axpro_adjust_rate_control(AXProFrameCallbackData* idat, uint32_t prepared_streamblocks)
{
    uint16_t rate_change_timeout = idat->rate_change_timeout;
    if(!rate_change_timeout)
    {
        switch(idat->rate_idx_set)
        {   case 0:
                if(prepared_streamblocks < idat->rate_blocks_hi) { idat->rate_idx_set = 1; idat->rate_change_timeout = idat->rate_change_timeout_slow; }
                else return;
                break;

            case 1:
                if(prepared_streamblocks > idat->rate_blocks_hi) { idat->rate_idx_set = 0; idat->rate_change_timeout = idat->rate_change_timeout_normal; }
                else if(prepared_streamblocks < idat->rate_blocks_low) { idat->rate_idx_set = 2; idat->rate_change_timeout = idat->rate_change_timeout_slow; }
                else return;
                break;

            case 2:
                if(prepared_streamblocks >= idat->rate_blocks_low) { idat->rate_idx_set = 1; idat->rate_change_timeout = idat->rate_change_timeout_slow; }
                else return;
                break;
        }
        //uint16_t current_rate_idx = (prepared_streamblocks > idat->rate_blocks_hi) ? 0 : ((prepared_streamblocks < idat->rate_blocks_low) ? 2 : 1);
        //print_irq("rate change to %d (%.3f)\n", idat->rate_idx_set, idat->rate_src_ratios[idat->rate_idx_set]);
        float new_ratio = idat->rate_src_ratios[idat->rate_idx_set];
        AXSetVoiceSrcRatio(idat->voice_l, new_ratio);
        AXSetVoiceSrcRatio(idat->voice_r, new_ratio);
    }
    else idat->rate_change_timeout = rate_change_timeout - 1;
}
#else
#define _axpro_adjust_rate_control(Idat, PreparedStreamblocks)
#endif


void axpro_audioframe_callback()
{
    // just skip the interrupt handler when interlocked and when init is not complete
    AXPRO_ALIGNED_DECLARE_CLINE1(AXProMsgMain2Audio msg);
    AXProAudioCore* _ac = ga.ac;
    if(!_ac || !_ac->interrupt_data.voice_l) return;
    //if(ac->prev_ax_framecallback) ac->prev_ax_framecallback(); // not really needed here
    AXProFrameCallbackData* idat = &_ac->interrupt_data;

    // check control msgs from main
    while(OSReceiveMessage(&_ac->qmain2irq, (OSMessage*) &msg, OS_MESSAGE_FLAGS_NONE))
        //if(!msg.Call)
        switch(msg.ctl.request)
        {   case AXPMCTL_QUIT:
                // stop voices with fade out, maybe even go into stopping state ?
                _axpro_stop_playing(idat);
                // wake up audio thread if needed with quit msg
                AXPro_RunMsg(&_ac->qirq2audio, msg);
                ga.ac = NULL;
                return;
        }

    // see if any new blocks are provided to us
    uint32_t prepared_streamblocks = idat->prepared_streamblocks;
    while(OSReceiveMessage(&_ac->qaudio2irq, (OSMessage*) &msg, OS_MESSAGE_FLAGS_NONE))
        //if(!msg.Call)
        switch(msg.ctl.request)
        {
            case AXPMCTL_BLOCK_READY:
                ++prepared_streamblocks;
                idat->prepared_streamblocks = prepared_streamblocks;
                break;

            case AXPMCTL_PING:
                if(msg.ctl.pingrequest & AXPINGF_SET) idat->flags |= (msg.ctl.pingrequest & ~AXPINGF_SET);
                else if(msg.ctl.pingrequest & AXPINGF_RESET) idat->flags &= ~(msg.ctl.pingrequest & ~AXPINGF_RESET);
                AXPro_PostCtl(&_ac->qirq2audio, AXPMCTL_PING);
                break;
        }

    // help out the polling mode loop with timing
    // notify when not playing for a long time so audio core can do a blocking wait queue from us
    if(idat->state < AXPACS_PLAYING && idat->flags & AXPIHF_POLLMODE) // AXPACS_STOPPED and AXPACS_BUFFERING_BEFORE_PLAY
    {   uint16_t stopped_state_ticks = idat->stopped_state_duration + 1;
        if(stopped_state_ticks < AXPRO_POLL_LOWPOWER_TIMEOUT) idat->stopped_state_duration = stopped_state_ticks;
        else if(stopped_state_ticks > AXPRO_POLL_LOWPOWER_TIMEOUT)
        AXPro_PostCtl(&_ac->qirq2audio, AXPMCTL_PING)
        else
        {   idat->stopped_state_duration = stopped_state_ticks;
            AXPro_RunCtlEx(&_ac->qirq2audio, AXPMCTL_PING, AXPINGF_SET | AXPACF_STOPPED_LONGAGO, OS_MESSAGE_FLAGS_NONE);
        }
    }

    // adjust the AX renderer and our state
    switch(idat->state)
    {
        case AXPACS_STOPPED:
            if(prepared_streamblocks)
            idat->state = AXPACS_BUFFERING_BEFORE_PLAY;
            else break;

        case AXPACS_BUFFERING_BEFORE_PLAY:
            if(prepared_streamblocks >= AXPRO_MINBLOCKS_FOR_PLAY)
            {
                idat->state = AXPACS_PLAYING;
                if(idat->stopped_state_duration)
                {   idat->stopped_state_duration = 0;
                    AXPro_RunCtlEx(&_ac->qirq2audio, AXPMCTL_PING, AXPINGF_RESET | AXPACF_STOPPED_LONGAGO, OS_MESSAGE_FLAGS_NONE);
                }
                _axpro_begin_playing(idat);
            }
            break;

        case AXPACS_PLAYING:
        {   uint32_t loop_count = AXGetVoiceLoopCount(idat->voice_l);

            // our playing blockid changed, one full streamblock has been played, next one is playing now
            if(idat->prev_loopcount != loop_count)
            {
                // one streamblock has been played
                _axpro_adjust_rate_control(idat, prepared_streamblocks);
                uint16_t playing_blockid = idat->playing_blockid;
                idat->prev_loopcount = loop_count;
                if(++playing_blockid == idat->streamblocks)
                playing_blockid = 0;
                idat->prepared_streamblocks = --prepared_streamblocks;
                idat->playing_blockid = playing_blockid;
                //print_irq("  playing_blockid=%d, prepared=%d\n", idat->playing_blockid, prepared_streamblocks);

                // calculate helpers for adjusting loop and end offsets
                uint32_t playing_sampleidx = playing_blockid * idat->streamblock_samples;
                uint16_t next_playing_blockid = playing_blockid + 1;

                // adjust correct loop and end offsets for continuous play case
                uint32_t end_offset = playing_sampleidx + idat->streamblock_frames - 1;
                uint32_t loop_offset = (next_playing_blockid < idat->streamblocks) ?
                        next_playing_blockid * idat->streamblock_samples :
                        0;

                // must go into stopping state - no two blocks ready for just letting it play with loop jump in play state
                if(prepared_streamblocks < 2)
                idat->state = AXPACS_STOPPING;

                // apply the loop and end offsets and either continue in play or stopping state
                AXSetVoiceLoopOffset(idat->voice_l, loop_offset);
                AXSetVoiceEndOffset(idat->voice_l, end_offset);
                AXSetVoiceLoopOffset(idat->voice_r, loop_offset);
                AXSetVoiceEndOffset(idat->voice_r, end_offset);

                // publish the played block to audio core
                AXPro_PostCtl(&_ac->qirq2audio, AXPMCTL_BLOCK_PLAYED);
            } // loop triggered - one streamblock played, next one started
        }
        break;


        case AXPACS_STOPPING:
        {   uint32_t loop_count = AXGetVoiceLoopCount(idat->voice_l);

            // our playing blockid changed, one full streamblock has been played, next one is playing now
            if(idat->prev_loopcount != loop_count)
            {
                // one streamblock has been played
                _axpro_adjust_rate_control(idat, prepared_streamblocks);
                uint16_t playing_blockid = idat->playing_blockid;
                idat->prev_loopcount = loop_count;
                if(++playing_blockid == idat->streamblocks)
                playing_blockid = 0;
                idat->prepared_streamblocks = --prepared_streamblocks;
                idat->playing_blockid = playing_blockid;
                //print_irq("  stopping_blockid=%d, prepared=%d\n", idat->playing_blockid, prepared_streamblocks);

                // calculate helpers for adjusting loop and end offsets
                uint32_t playing_sampleidx = playing_blockid * idat->streamblock_samples;
                uint16_t next_playing_blockid = playing_blockid + 1;

                // adjust correct loop and end offsets for continuous play case
                uint32_t end_offset = playing_sampleidx + idat->streamblock_frames - 1;
                uint32_t loop_offset = (next_playing_blockid < idat->streamblocks) ?
                        next_playing_blockid * idat->streamblock_samples :
                        0;

                if(prepared_streamblocks)
                {
                    if(prepared_streamblocks > 1)
                    idat->state = AXPACS_PLAYING;

                    // apply the loop and end offsets and either continue in play or stopping state
                    AXSetVoiceLoopOffset(idat->voice_l, loop_offset);
                    AXSetVoiceEndOffset(idat->voice_l, end_offset);
                    AXSetVoiceLoopOffset(idat->voice_r, loop_offset);
                    AXSetVoiceEndOffset(idat->voice_r, end_offset);
                }
                else
                {   idat->state = AXPACS_STOPPED;
                    AXVoiceOffsets offsets;
                    offsets.dataType        = AXPRO_VOICE_FORMAT;
                    offsets.loopingEnabled  = 1;
                    offsets.loopOffset      = loop_offset;
                    offsets.endOffset       = end_offset;
                    offsets.currentOffset   = playing_sampleidx;
                    offsets.data            = idat->voicebuff_l;
                    _axpro_stop_playing(idat);
                    AXSetVoiceOffsets(idat->voice_l, &offsets);
                    offsets.data            = &idat->voicebuff_l[idat->streamblock_frames];
                    AXSetVoiceOffsets(idat->voice_r, &offsets);
                }

                // publish the played block to audio core
                AXPro_PostCtl(&_ac->qirq2audio, AXPMCTL_BLOCK_PLAYED);
            } // loop triggered - one streamblock played, next one started

            else
            {
                if(prepared_streamblocks > 1)
                idat->state = AXPACS_PLAYING;
            }
        }
        break;
    };

    // TODO:update mixer for fades to work
}



// audio core thread main
//----------------------------------------------------

void axpro_close_hw(AXProAudioCore* ac);
void axpro_hw_closed(AXProMainCore* mc);
void axpro_polling_loop(AXProAudioCore* ac, void* gen_samples_ctx, AXPRO_GEN_CALLBACK* GenerateSamples);

static void axpro_thread_ready(AXProMainCore* mc)
{   mc->qmain2audio = mc->msg.ready.q2audio;
    mc->qmain2irq = mc->msg.ready.q2irq;
    mc->ac = mc->msg.ready.ac;
}

// audio core thread "main"
int axpro_core_main(int args, OSMessageQueue* q2main)
{
    // prepare audio core model and link with main core
    AXProAudioCore* ac;
    {   uint32_t audio_core_data_sizeof = sizeof(AXProAudioCore); AXPRO_ALIGN_SIZE(audio_core_data_sizeof, AXPRO_CACHEFETCH_SIZE);
        ac = ga.ac = (AXProAudioCore*) AXPro_MemAlloc(audio_core_data_sizeof, AXPRO_CACHEFETCH_SIZE);
    }
    DCZeroRange(ac, sizeof(AXProAudioCore));
    memset(ac, 0, sizeof(AXProAudioCore));
    //axpdebug_loginit(&ga.debug);

    ac->qaudio2main = q2main;
    OSInitMessageQueue(&ac->qmain2audio, ac->msgstore_main2audio, AXPRO_MAX_MSGS_MAIN2AUDIO);
    AXProMsgAudio2Main ready_msg; ready_msg.Call = axpro_thread_ready;
    ready_msg.ready.ac = ac; ready_msg.ready.q2audio = &ac->qmain2audio; ready_msg.ready.q2irq = &ac->qmain2irq;
    AXPro_RunMsg(q2main, ready_msg);

    // init main core --> interrupt handler <---> audio core message queues
    OSInitMessageQueue(&ac->qirq2audio, ac->msgstore_irq2audio, AXPRO_MAX_MSGS_IRQ2AUDIO);
    OSInitMessageQueue(&ac->qaudio2irq, ac->msgstorage_audio2irq, AXPRO_MAX_MSGS_AUDIO2IRQ);
    OSInitMessageQueue(&ac->qmain2irq, ac->msgstorage_main2irq, AXPRO_MAX_MSGS_MAIN2IRQ);

    // blocking drain the message queue until quit signal
    while(OSReceiveMessage(&ac->qmain2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_BLOCKING))
        if(ac->msg.Call) ac->msg.Call(ac);
        else
        {   checkctl:
            if(ac->msg.ctl.request==AXPMCTL_QUIT) break;
            else if(ac->msg.ctl.request==AXPMCTL_CHANGE_SAMPLEGEN)
            {   if(ac->msg.ctl.setgen.GenerateSamples)
                {   axpro_polling_loop(ac, ac->msg.ctl.setgen.samplegen_ctx, ac->msg.ctl.setgen.GenerateSamples);
                    goto checkctl;
                }
            }

            // respond to main core with fresh free blocks when pinging us
            else if(ac->msg.ctl.request==AXPMCTL_PING)
            {
                uint32_t free_blocks = ac->voicebuff_free_blocks;
                while(OSReceiveMessage(&ac->qirq2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_NONE))
                //if(!ac->msg.Call)
                {   if(ac->msg.ctl.request==AXPMCTL_BLOCK_PLAYED) ++free_blocks;
                    else goto checkctl;
                }
                ac->voicebuff_free_blocks = free_blocks;
                AXPro_RunCtlEx(ac->qaudio2main, AXPMCTL_PING, free_blocks, OS_MESSAGE_FLAGS_BLOCKING);
            }
        }

    // cleanup and quit
    ga.ac = NULL;
    axpro_close_hw(ac);
    //axpdebug_logclose(&ga.debug);

    AXPro_MemFree(ac);
    AXPro_Run(q2main, axpro_hw_closed);
    return 0;
}

int axpro_run_core(OSMessageQueue* qaudio2main)
{
    if(OSCreateThread(&ga.thread_handle,
        (OSThreadEntryPointFn) axpro_core_main,
        0, (char*) qaudio2main,
        ga.thread_stack + AXPRO_THREAD_STACKSIZE, AXPRO_THREAD_STACKSIZE,
        AXPRO_THREAD_PRIORITY, AXPRO_THREAD_CORE | OS_THREAD_ATTRIB_DETACHED
    ))
    {   OSResumeThread(&ga.thread_handle);
        // we're running this from main core, thread handle is not needed in cache
        DCFlushRange(&ga.thread_handle, sizeof(ga.thread_handle));
        return 0;
    }
    else return 1;
}


// sound rendering into circular buffers
//----------------------------------------------------

void _axpro_split_stereo_int16_naive(int16_t* dest_left, int16_t* stereo, int32_t channel_byte_offset, uint32_t stereo_frames)
{   int16_t* dest_right = (int16_t*) ((uint8_t*)dest_left + channel_byte_offset);
    do
    {   *dest_left++ = *stereo++;
        *dest_right++ = *stereo++;
    } while(--stereo_frames);
}

#if AXPRO_ACCEPTS_FLOAT_SAMPLES
static void _axpro_split_stereo_float32_naive(int16_t* dest_left, float* stereo, int32_t channel_byte_offset, uint32_t stereo_frames)
{   int16_t* dest_right = (int16_t*) ((uint8_t*)dest_left + channel_byte_offset);
    do
    {   int stereo_l = (int) (*stereo++ * 32768.0f);
        int stereo_r = (int) (*stereo++ * 32768.0f);
        if(stereo_l > 32767) stereo_l = 32767;
        if(stereo_r > 32767) stereo_r = 32767;
        if(stereo_l < -32768) stereo_l = -32768;
        if(stereo_r < -32768) stereo_r = -32768;
        *dest_left++ = (int16_t) stereo_l;
        *dest_right++ = (int16_t) stereo_r;
    } while(--stereo_frames);
}
#endif

void _axpro_upload_and_publish_block(AXProAudioCore* ac, int16_t* block_base)
{
    // this is a blocking call, we can immediately publish block ready
    LCStoreDMABlocks(ac->voicebuff_writeblock, block_base, AXPRO_CLINES4FRAMES(ac->streamblock_frames));
    AXPro_RunCtl(&ac->qaudio2irq, AXPMCTL_BLOCK_READY);

    if(ac->voicebuff_writeblock < ac->voicebuff_last_block)
    ac->voicebuff_writeblock = (int16_t*) ((uint8_t*)ac->voicebuff_writeblock + ac->streamblock_bytes);
    else ac->voicebuff_writeblock = ac->voicebuff;
}


int16_t* _axpro_finish_lc_writeblock_and_start_new(AXProAudioCore* ac)
{
    int16_t* block_base = ac->lc_block_base;
    if(block_base < ac->lc_last_block)
    {   block_base += ac->streamblock_frames * AXPRO_CHANNELS;
        //ac->lc_missing_block_frames = ac->streamblock_frames;
        //ac->lc_block_writepos = block_base;
        ac->lc_block_base = block_base;
        return block_base;
    }
    else
    {   //ac->lc_missing_block_frames = ac->streamblock_frames;
        //ac->lc_block_writepos = ac->lc_base_circular;
        ac->lc_block_base = ac->lc_base_circular;
        return ac->lc_base_circular;
    }
}



// non-polling writing (default RA)
//----------------------------------------------------


void _axpro_write(AXProAudioCore* ac)
{
    uint32_t frames_generated = ac->msg.write.frame_num;
    AXProInSample* frame_src = (AXProInSample*) ac->msg.write.frame_src;
    uint32_t missing_block_frames = ac->lc_missing_block_frames;
    int16_t* writepos = ac->lc_block_writepos;
    uint32_t free_blocks = ac->voicebuff_free_blocks;

    do
    {
        if(free_blocks)
        {
            // split stereo to per channel, we can do this for all stereo frames generated
            if(frames_generated >= missing_block_frames)
            {
                // prepare the full block
                --free_blocks;
                #if AXPRO_ACCEPTS_FLOAT_SAMPLES
                _axpro_split_stereo_float32_naive(writepos, frame_src, ac->streamblock_channel_bytes, missing_block_frames);
                #else
                _axpro_split_stereo_int16_naive(writepos, frame_src, ac->streamblock_channel_bytes, missing_block_frames);
                #endif

                // prepare new block writing
                frames_generated -= missing_block_frames;
                int16_t* completed_block_base = ac->lc_block_base;
                frame_src += (missing_block_frames * AXPRO_CHANNELS);
                missing_block_frames = ac->streamblock_frames;
                writepos = _axpro_finish_lc_writeblock_and_start_new(ac);

                // upload full block and continue...
                if(frames_generated) _axpro_upload_and_publish_block(ac, completed_block_base);

                // ...or notify front, then upload the full block and quit
                else
                {   AXPro_RunCtlEx(ac->qaudio2main, AXPMCTL_CLREAD_FENCEREACH, free_blocks, OS_MESSAGE_FLAGS_BLOCKING);
                    _axpro_upload_and_publish_block(ac, completed_block_base);
                    break;
                }
            }
            else
            {   // just convert frames and notify front, full block is not ready now
                missing_block_frames -= frames_generated;

                #if AXPRO_ACCEPTS_FLOAT_SAMPLES
                _axpro_split_stereo_float32_naive(writepos, frame_src, ac->streamblock_channel_bytes, frames_generated);
                #else
                _axpro_split_stereo_int16_naive(writepos, frame_src, ac->streamblock_channel_bytes, frames_generated);
                #endif
                writepos += frames_generated;
                AXPro_RunCtlEx(ac->qaudio2main, AXPMCTL_CLREAD_FENCEREACH, free_blocks, OS_MESSAGE_FLAGS_BLOCKING);
                break;
            }

        }
        else
        {   if(OSReceiveMessage(&ac->qirq2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_BLOCKING))
            //if(!ac->msg.Call)
            {   if(ac->msg.ctl.request==AXPMCTL_BLOCK_PLAYED) ++free_blocks;
                else if(ac->msg.ctl.request==AXPMCTL_QUIT) break;
            }
        }

    } while(1);

    // save current state for next write
    ac->lc_missing_block_frames = missing_block_frames;
    ac->lc_block_writepos = writepos;

    // drain the interrupt queue and update free blocks
    while(OSReceiveMessage(&ac->qirq2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_NONE))
        //if(!ac->msg.Call)
        if(ac->msg.ctl.request == AXPMCTL_BLOCK_PLAYED) ++free_blocks;
    ac->voicebuff_free_blocks = (uint16_t) free_blocks;
}

