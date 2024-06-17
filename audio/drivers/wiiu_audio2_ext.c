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

void _axpro_split_stereo_int16_naive(int16_t* dest_left, int16_t* stereo, int32_t channel_byte_offset, uint32_t stereo_frames);
void _axpro_upload_and_publish_block(AXProAudioCore* ac, int16_t* block_base);
int16_t* _axpro_finish_lc_writeblock_and_start_new(AXProAudioCore* ac);
void axpro_pollmode_begin(AXProMainCore* mc);
void axpro_pollmode_end(AXProMainCore* mc);


static uint32_t _axpro_gather_interleaved_stereo(AXProAudioCore* ac, uint32_t frames_generated)
{
    // split stereo to per channel, we can do this for all stereo frames generated
    uint32_t missing_block_frames = ac->lc_missing_block_frames;
    if(frames_generated >= missing_block_frames)
    {
        // convert and publish full block immediately
        _axpro_split_stereo_int16_naive(ac->lc_block_writepos, ac->lc_base, ac->streamblock_channel_bytes, missing_block_frames);
        _axpro_upload_and_publish_block(ac, ac->lc_block_base);

        // adjust bookkeeping and convert any remaining frames
        frames_generated -= missing_block_frames;
        int16_t* new_block = _axpro_finish_lc_writeblock_and_start_new(ac);
        if(frames_generated)
        {   _axpro_split_stereo_int16_naive(new_block, ac->lc_base + (missing_block_frames * AXPRO_CHANNELS), ac->streamblock_channel_bytes, frames_generated);
            new_block += frames_generated;
            ac->lc_missing_block_frames = ac->streamblock_frames - frames_generated;
        }
        else ac->lc_missing_block_frames = ac->streamblock_frames;
        ac->lc_block_writepos = new_block;
        return 1;
    }
    else
    {   // just convert frames and adjust bookkeeping, full block still not ready
        int16_t* writepos = ac->lc_block_writepos;
        ac->lc_missing_block_frames -= frames_generated;
        ac->lc_block_writepos = writepos + frames_generated;
        _axpro_split_stereo_int16_naive(writepos, ac->lc_base, ac->streamblock_channel_bytes, frames_generated);
        return 0;
    }
}

// special loop for cores allowing to call sample generator from audio core thread
void axpro_polling_loop(AXProAudioCore* ac, void* gen_samples_ctx, AXPRO_GEN_CALLBACK* GenerateSamples)
{
    AXPro_RunCtlEx(&ac->qaudio2irq, AXPMCTL_PING, AXPINGF_SET | AXPIHF_POLLMODE, OS_MESSAGE_FLAGS_BLOCKING);
    AXPro_Run(ac->qaudio2main, axpro_pollmode_begin);
    uint32_t stopped_longago = 0;
    uint32_t free_blocks = ac->streamblocks;
    uint32_t do_blocking_wait;

    ac->lc_missing_block_frames = ac->streamblock_frames;
    ac->lc_base_circular = ac->lc_base + (ac->streamblock_frames * AXPRO_CHANNELS);
    ac->lc_block_writepos = ac->lc_base_circular;
    ac->lc_block_base = ac->lc_base_circular;

    do
    {
        // call the sample generator immediately as long as we have a free block to write to
        if(free_blocks)
        {
            uint32_t frames_generated = GenerateSamples(gen_samples_ctx, ac->lc_base, ac->streamblock_bytes);
            do_blocking_wait = !frames_generated;

            if(frames_generated)
            {   if(_axpro_gather_interleaved_stereo(ac, frames_generated))
                --free_blocks;
            }

            // provoke a ping signal when needed so we can safely go to blocking wait
            else if(!stopped_longago)
            AXPro_PostCtl(&ac->qaudio2irq, AXPMCTL_PING);
        }
        else do_blocking_wait = 2;

        // do a blocking wait for 'block played' signal from interrupt handler - perfectly timed sleep for calling sample generator with full buffer
        // also, wait on ping signal to avoid busy loop when no data expected
        if(do_blocking_wait && OSReceiveMessage(&ac->qirq2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_BLOCKING))
            //if(!ac->msg.Call)
            switch(ac->msg.ctl.request)
            {   case AXPMCTL_QUIT: goto quit;
                case AXPMCTL_BLOCK_PLAYED: ++free_blocks; if(do_blocking_wait==2) continue; else break;
                case AXPMCTL_PING:
                    if((ac->msg.ctl.pingrequest & AXPACF_STOPPED_LONGAGO))
                    {   if(ac->msg.ctl.pingrequest & AXPINGF_SET) stopped_longago = 1;
                        else stopped_longago = 0;
                    }
                    break;
            }

        // check our system msgs for control signals
        if(OSReceiveMessage(&ac->qmain2audio, (OSMessage*)&ac->msg, OS_MESSAGE_FLAGS_NONE))
        {   if(ac->msg.Call) ac->msg.Call(ac);
            else break;
        }

    } while(1);

    quit:
    ac->lc_base_circular = ac->lc_base;
    AXPro_RunCtlEx(&ac->qaudio2irq, AXPMCTL_PING, AXPINGF_RESET | AXPIHF_POLLMODE, OS_MESSAGE_FLAGS_NONE);
    AXPro_Run(ac->qaudio2main, axpro_pollmode_end);
}
