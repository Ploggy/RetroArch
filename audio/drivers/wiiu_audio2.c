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
typedef struct GlobalDataMainCore
{   AXProMainCore*          mc;
    uint32_t                unusedu[3];
//--
    AXProMsgMain2Audio      msg_setcallback;
//--

} GlobalDataMainCore; AXPRO_ALIGNED_DECLARE_CLINE2(static GlobalDataMainCore gm);




// audio core init
//----------------------------------------------------

static void _axpro_init_streaming(AXProAudioCore* ac, uint32_t streamblocks, uint32_t streamblock_bytes, uint32_t frames_per_interrupt, float req_rate)
{
    uint32_t streamblock_frames = streamblock_bytes / (AXPRO_CHANNELS * AXPRO_SAMPLE_BYTES);
    
    ac->streamblocks = (uint16_t) streamblocks;
    ac->streamblock_frames = (uint16_t) streamblock_frames;
    ac->streamblock_bytes = streamblock_bytes;
    ac->streamblock_channel_bytes = streamblock_bytes / AXPRO_CHANNELS;
    
    ac->interrupt_data.streamblock_frames = (uint16_t) streamblock_frames;
    ac->interrupt_data.streamblock_samples = (uint16_t) streamblock_frames * AXPRO_CHANNELS;
    ac->interrupt_data.streamblocks = streamblocks;
    ac->interrupt_data.frames_per_interrupt = (uint16_t) frames_per_interrupt;
    #if AXPRO_RATE_CONTROL_AMORTIZER
    {   float hw_rate = (float) AXPRO_SAMPLE_RATE;
        float block_duration = (float)streamblock_frames / req_rate;
        uint16_t rate_blocks_half = (uint16_t) streamblocks >> 1;
        uint16_t rate_blocks_quarter = (uint16_t) streamblocks >> 2;
        ac->interrupt_data.rate_blocks_low = rate_blocks_half;
        ac->interrupt_data.rate_blocks_hi = rate_blocks_half + rate_blocks_quarter;
        ac->interrupt_data.rate_src_ratios[0] = req_rate / hw_rate;
        ac->interrupt_data.rate_src_ratios[1] = (req_rate - AXPRO_RATE_LOW) / hw_rate;
        ac->interrupt_data.rate_src_ratios[2] = (req_rate - AXPRO_RATE_LOWER) / hw_rate;
        ac->interrupt_data.rate_change_timeout_normal = (uint16_t) (AXPRO_RATE_DURATION_NORMAL / block_duration);
        ac->interrupt_data.rate_change_timeout_slow = (uint16_t) (AXPRO_RATE_DURATION_LOW / block_duration);
    }
    #endif
    ac->voicebuff_free_blocks = streamblocks;
    ac->lc_missing_block_frames = (uint16_t) streamblock_frames;
    //print_ac("axpro blocks=%d, block_bytes=%d, block_samples=%d\n", streamblocks, streamblock_bytes, streamblock_bytes / (AXPRO_CHANNELS * AXPRO_SAMPLE_BYTES));
}

int axpro_core_main(int args, OSMessageQueue* q2main);
void axpro_audioframe_callback();
void _axpro_write(AXProAudioCore* ac);

static void axpro_hw_ready(AXProMainCore* mc) 
{   mc->flags |= AXPMCF_HW_READY;
    mc->total_streamblocks = mc->msg.hwready.streamblocks;
}

void axpro_hw_closed(AXProMainCore* mc) { mc->flags |= AXPMCF_HW_CLOSED; }


static void _axpro_init_timing(AXProAudioCore* ac, float req_latency_ms, float req_rate)
{
    // buffer size and single stream block size
    uint32_t streamblock_bytes, streamblocks, frames_per_interrupt;
    {   frames_per_interrupt = (uint32_t) ceilf( ((req_rate * AXPRO_INTERRUPT_INTERVAL_MS) / 1000.0f) );
        uint32_t interrupts_for_max_latency = (uint32_t) ceilf(req_latency_ms / (float) AXPRO_INTERRUPT_INTERVAL_MS );
        if(interrupts_for_max_latency < AXPRO_STREAMBLOCK_INTERRUPTS)
        interrupts_for_max_latency = AXPRO_STREAMBLOCK_INTERRUPTS;

        // make every channel size nicely aligned
        streamblock_bytes = frames_per_interrupt * AXPRO_STREAMBLOCK_INTERRUPTS * AXPRO_SAMPLE_BYTES;
        AXPRO_ALIGN_SIZE(streamblock_bytes, AXPRO_CACHEFETCH_SIZE);
        streamblock_bytes *= AXPRO_CHANNELS;

        uint32_t max_latency_bytes = (interrupts_for_max_latency * frames_per_interrupt) * (AXPRO_CHANNELS * AXPRO_SAMPLE_BYTES);
        streamblocks = (max_latency_bytes / streamblock_bytes) + 1;

        // make the streamblock number even and at least 2
        if(streamblocks > 1) AXPRO_ALIGN_SIZE(streamblocks, 2);
        else streamblocks = 2;
    }
    _axpro_init_streaming(ac, streamblocks, streamblock_bytes, frames_per_interrupt, req_rate);
}

static void _axpro_assign_voices_to_devices(AXProAudioCore* ac)
{
    // fcon: what about wii set to surround mode ?
    AXVoiceDeviceMixData channel_mix[6];
    memset(channel_mix, 0, sizeof(channel_mix));
    channel_mix[0].bus[0].volume = 0x8000; // 1.0 in 1:15 fixed point
    AXSetVoiceDeviceMix(ac->interrupt_data.voice_l, AX_DEVICE_TYPE_TV, 0, channel_mix);
    AXSetVoiceDeviceMix(ac->interrupt_data.voice_l, AX_DEVICE_TYPE_DRC, 0, channel_mix);
    channel_mix[0].bus[0].volume = 0;
    channel_mix[1].bus[0].volume = 0x8000; // 1.0 in 1:15 fixed point
    AXSetVoiceDeviceMix(ac->interrupt_data.voice_r, AX_DEVICE_TYPE_TV, 0, channel_mix);
    AXSetVoiceDeviceMix(ac->interrupt_data.voice_r, AX_DEVICE_TYPE_DRC, 0, channel_mix);
}

static AXVoice* _axpro_create_voice(AXProAudioCore* ac, int16_t* sample_buffer, uint32_t voice_samplerate)
{
    // alloc voice and prepare it for looped playing
    AXVoice* voice = AXAcquireVoice(AXPRO_VOICE_PRIORITY, NULL, 0);
    if(voice)
    {
        // setup voice initially
        AXVoiceOffsets offsets;
           offsets.dataType        = AXPRO_VOICE_FORMAT;
           offsets.loopingEnabled  = 1;
           offsets.loopOffset      = ac->streamblock_frames << 1;
           offsets.endOffset       = ac->streamblock_frames - 1;
           offsets.currentOffset   = 0;
           offsets.data            = sample_buffer;
        AXSetVoiceOffsets(voice, &offsets);
        bool needs_resampler = AXPRO_RATE_CONTROL_AMORTIZER || (voice_samplerate != AXPRO_SAMPLE_RATE);
        AXSetVoiceSrcType(voice,  needs_resampler ? AXPRO_INIT_RESAMPLER                                    : AX_VOICE_SRC_TYPE_NONE);
        AXSetVoiceSrcRatio(voice, needs_resampler ? (float) voice_samplerate / (float) AXPRO_SAMPLE_RATE    : 1.0f);
        AXVoiceVeData vol_envelope; vol_envelope.volume = 0x8000; vol_envelope.delta = 0;
        AXSetVoiceVe(voice, &vol_envelope);
        return voice;
    }
    return NULL;
}

// prepare audio hardware
static void _axpro_init_hw(AXProAudioCore* ac)
{
    // hw startup with desired pipeline setup, prepare timing vars
    AXInitParams init_params;memset(&init_params, 0, sizeof(AXInitParams));
        init_params.renderer = AXPRO_INIT_RENDERER;
        init_params.pipeline = AXPRO_INIT_PIPELINE;
    AXInitWithParams(&init_params);
    _axpro_init_timing(ac, (float)ac->msg.init.latency, (float)ac->msg.init.rate);

    uint32_t init_error_code = 0;
    #define fail_with_error_code(Code)  \
    {   init_error_code = Code;         \
        goto init_error;                \
    }

    // prepare locked cache
    if(LCEnableDMA())
    {   uint32_t lc_size = ac->streamblock_bytes * AXPRO_STREAMBLOCKS_LC;
        AXPRO_ALIGN_SIZE(lc_size, AXPRO_LCACHE_ALIGNMENT);
        int16_t* lc_base = (int16_t*) LCAlloc(lc_size);
        if(lc_base) 
        {   ac->lc_base = lc_base;
            ac->lc_base_circular = lc_base;
            ac->lc_block_writepos = lc_base;
            ac->lc_block_base = lc_base;
            ac->lc_last_block = (int16_t*) ((uint8_t*) lc_base + (ac->streamblock_bytes * (uint32_t) (AXPRO_STREAMBLOCKS_LC-1)));
        } else fail_with_error_code(2);
    } else fail_with_error_code(1);

    // alloc sample buffer (interleaved left and right)
    {   uint32_t buffer_size = ac->streamblock_bytes * (uint32_t) ac->streamblocks;
        ac->voicebuff = (int16_t*) AXPro_MemAlloc(buffer_size, AXPRO_SAMPLEBUFF_ALIGNMENT);
        ac->voicebuff_last_block = ac->voicebuff + ((uint32_t)(ac->streamblocks - 1) * ac->streamblock_frames * AXPRO_CHANNELS);
        ac->voicebuff_writeblock = ac->voicebuff;
        ac->interrupt_data.voicebuff_l = &ac->voicebuff[0];
        if(!ac->voicebuff) fail_with_error_code(3);
    }

    // setup AX voices
    ac->interrupt_data.voice_l = _axpro_create_voice(ac, &ac->voicebuff[0], ac->msg.init.rate);
    ac->interrupt_data.voice_r = _axpro_create_voice(ac, &ac->voicebuff[ac->streamblock_frames], ac->msg.init.rate);
    if(!ac->interrupt_data.voice_l || !ac->interrupt_data.voice_r) fail_with_error_code(4);
    _axpro_assign_voices_to_devices(ac);

    // let main core have info on internal buffer layouts
    {   AXPRO_ALIGNED_DECLARE_CLINE1(AXProMsgAudio2Main msg);
        msg.Call = axpro_hw_ready;
        msg.hwready.streamblocks = ac->streamblocks;
        AXPro_RunMsg(ac->qaudio2main, msg);
    }

    ac->interrupt_data.prev_ax_framecallback = AXRegisterFrameCallback(axpro_audioframe_callback);
    return;

init_error:
    AXPro_RunCtlEx(ac->qaudio2main, AXPMCTL_ERROR_INIT, init_error_code, OS_MESSAGE_FLAGS_BLOCKING);
}

void axpro_close_hw(AXProAudioCore* ac)
{
    AXRegisterFrameCallback(ac->interrupt_data.prev_ax_framecallback);
    if(ac->lc_base)
    {   LCDealloc(ac->lc_base);
        //LCDisableDMA(); // fcon:aroma - check how is this handled system wide, not handling bg switch at the moment and we're defaulting on core 0
    }
    if(ac->interrupt_data.voice_r) AXFreeVoice(ac->interrupt_data.voice_r);
    if(ac->interrupt_data.voice_l) AXFreeVoice(ac->interrupt_data.voice_l);
    if(ac->voicebuff) AXPro_MemFree(ac->voicebuff);
    AXQuit();
}





// client core public interface
//----------------------------------------------------

static void axpro_audio_free(AXProMainCore* mc)
{
    if(mc)
    {   if(mc->qmain2audio)
        {   // drain our queue and dispatch quit signals
            while(OSReceiveMessage(&mc->qaudio2main, (OSMessage*) &mc->msg, OS_MESSAGE_FLAGS_NONE)) if(mc->msg.Call) mc->msg.Call(mc);
            AXPro_RunCtl(mc->qmain2irq, AXPMCTL_QUIT);
            AXPro_RunCtl(mc->qmain2audio, AXPMCTL_QUIT);
            AXPro_Wait(mc, mc->flags & AXPMCF_HW_CLOSED, break, break);
        }
        AXPro_MemFree(mc);
        gm.mc = NULL;
    }
}

static void* axpro_audio_init(const char* device, unsigned rate, unsigned latency, unsigned block_frames, unsigned* new_rate)
{
    AXProMainCore* mc;
    {   uint32_t main_core_data_sizeof = sizeof(AXProMainCore); AXPRO_ALIGN_SIZE(main_core_data_sizeof, AXPRO_CACHEFETCH_SIZE);
        mc = (AXProMainCore*) AXPro_MemAlloc(main_core_data_sizeof, AXPRO_CACHEFETCH_SIZE);
    }
    if(mc)
    {   // init model and queue
        DCZeroRange(mc, sizeof(AXProMainCore));
        memset(mc, 0, sizeof(AXProMainCore));
        OSInitMessageQueue(&mc->qaudio2main, mc->msgstore_audio2main, AXPRO_MAX_MSGS_AUDIO2MAIN);

        // fcon: normal core-tied thread can just be created here if default core thread will be used in the future by RetroArch
        OSThread* audio_thread = OSGetDefaultThread(AXPRO_CORE_AUDIO);
        OSRunThread(audio_thread, (OSThreadEntryPointFn) axpro_core_main, 1, (const char**)&mc->qaudio2main);

        // wait for audio thread ready then init driver with prepared msg
        AXProMsgMain2Audio init_msg;
            init_msg.Call = _axpro_init_hw;
            init_msg.init.rate = rate;
            init_msg.init.latency = latency;
        AXPro_Wait(mc, mc->qmain2audio, goto error_init, goto error_init);
        AXPro_RunMsg(mc->qmain2audio, init_msg);
        AXPro_Wait(mc, mc->flags & AXPMCF_HW_READY, goto error_init, goto error_init);

        // prepare write msg - it will point to empy write call when using poll-mode extension
        mc->writemsg.Call = _axpro_write;

        // also install callback when set before driver was actually created
        if(gm.msg_setcallback.ctl.setgen.GenerateSamples)
        AXPro_RunMsg(mc->qmain2audio, gm.msg_setcallback);


        // let the system continue on main core
        *new_rate = rate;
        gm.mc = mc;
        return mc;

        error_init:
        //print_mc("axpro error %d\n", mc->msg.general_args[0]);
        axpro_audio_free(mc);
    }

    gm.mc = NULL;
    return NULL;
}

static bool axpro_audio_alive(AXProMainCore* mc) { return (mc->flags & (AXPMCF_HW_READY | AXPMCF_HW_CLOSED)) == AXPMCF_HW_READY; }
static bool axpro_audio_use_float(AXProMainCore* mc) { return (bool) (AXPRO_ACCEPTS_FLOAT_SAMPLES); }
static void axpro_audio_set_nonblock_state(AXProMainCore* mc, bool toggle) 
{   if(toggle) mc->flags |= AXPMCF_NONBLOCK;
    else mc->flags &= ~AXPMCF_NONBLOCK;
}
static size_t axpro_audio_write_avail(AXProMainCore* mc) { return mc->free_streamblocks; }
static size_t axpro_audio_buffer_size(AXProMainCore* mc) { return mc->total_streamblocks; }
static bool axpro_audio_stop(AXProMainCore* mc);
static bool axpro_audio_start(AXProMainCore* mc, bool is_shutdown);
static ssize_t axpro_audio_write_empty(AXProMainCore* mc, AXProInSample* samples, size_t size_bytes);

static ssize_t axpro_audio_write(AXProMainCore* mc, AXProInSample* samples, size_t size_bytes)
{
    // normally wait for last write request to at least stop using the client data
    if(!(mc->flags & AXPMCF_NONBLOCK))
    {   if(mc->reading_client_buffer)
        axpro_audio_wait_fence(mc, mc->reading_client_buffer);
    }

    // drop the data if job is still using the client buffer or we have no free blocks reported
    else
    {   axpro_audio_write_empty(mc, NULL, 0); // drain the queue from audio core without blocking
        if(mc->reading_client_buffer)
        return 0;
        // when last reported free blocks was 0, ping the audio core for more recent data
        if(!mc->free_streamblocks)
        {   AXPro_RunCtl(mc->qmain2audio, AXPMCTL_PING);
            return 0;
        }
    }
    
    if(size_bytes)
    {   uint32_t frames = (uint32_t) (size_bytes / (AXPRO_CHANNELS * sizeof(AXProInSample)));
        if(frames)
        {   mc->writemsg.write.frame_src = samples;
            mc->writemsg.write.frame_num = frames;
            mc->reading_client_buffer = samples;
            AXPro_RunMsg(mc->qmain2audio, mc->writemsg);

            // when optimization is on we'll let the client wait on fence before writing its buffer again
            // allowing for more parallel processing of driver and client core
            #if !AXPRO_IMPLICIT_NONBLOCK_OPTIM
            axpro_audio_wait_fence(mc, mc->reading_client_buffer);
            #endif

            return frames;
        }
    }
    return 0;
}


// main driver abstract interface as global data
audio_driver_t audio_axpro =
{
                            axpro_audio_init,
   (ssize_t (*) (void*, const void*, size_t)) axpro_audio_write,
   (bool (*) (void*))       axpro_audio_stop,
   (bool (*) (void*, bool)) axpro_audio_start,
   (bool (*) (void*))       axpro_audio_alive,
   (void (*) (void*, bool)) axpro_audio_set_nonblock_state,
   (void (*) (void*))       axpro_audio_free,
   (bool (*) (void*))       axpro_audio_use_float,
                            "AX Pro",
                            NULL, // optional device_list_new
                            NULL, // optional device_list_free
   (size_t (*) (void*))     axpro_audio_write_avail,
   (size_t (*) (void*))     axpro_audio_buffer_size
};




// starting and stopping will just install an empty write callback for direct writing mode
// when using polling mode, direct write must be disabled without slowing it down otherwise
//----------------------------------------------------------------------------------------------------

static void _axpro_write_empty(AXProAudioCore* ac)  { };
static ssize_t axpro_audio_write_empty(AXProMainCore* mc, AXProInSample* samples, size_t size_bytes)
{   // drain the queue so ending poll mode and fence msgs can get through
    while(OSReceiveMessage(&mc->qaudio2main, (OSMessage*) &mc->msg, OS_MESSAGE_FLAGS_NONE)) 
        if(mc->msg.Call) mc->msg.Call(mc);
        else if(mc->msg.ctl.request == AXPMCTL_CLREAD_FENCEREACH)
        {   mc->free_streamblocks = (uint16_t) mc->msg.ctl.ctlgeneral[0];
            mc->reading_client_buffer = NULL;
        }
        else if(mc->msg.ctl.request == AXPMCTL_PING) 
            mc->free_streamblocks = (uint16_t) mc->msg.ctl.ctlgeneral[0];
    return 0;
}

void axpro_pollmode_begin(AXProMainCore* mc) 
{   mc->flags |= AXPMCF_POLLMODE; 
    mc->writemsg.Call = _axpro_write_empty;
    audio_axpro.write = (ssize_t (*) (void*, const void*, size_t)) axpro_audio_write_empty;
    mc->reading_client_buffer = NULL;
}

void axpro_pollmode_end(AXProMainCore* mc) 
{   mc->flags &= ~AXPMCF_POLLMODE; 
    mc->writemsg.Call = _axpro_write;
    audio_axpro.write = (ssize_t (*) (void*, const void*, size_t)) axpro_audio_write;
}

static bool axpro_audio_stop(AXProMainCore* mc) 
{   if(!(mc->flags & (AXPMCF_POLLMODE | AXPMCF_STOPPED)))
    {   mc->flags |= AXPMCF_STOPPED;
        audio_axpro.write = (ssize_t (*) (void*, const void*, size_t)) axpro_audio_write_empty;
    }
    return true;
}

static bool axpro_audio_start(AXProMainCore* mc, bool is_shutdown)
{   if((mc->flags & AXPMCF_STOPPED) && !(mc->flags & AXPMCF_POLLMODE))
    {   mc->flags &= ~AXPMCF_STOPPED;
        audio_axpro.write = (ssize_t (*) (void*, const void*, size_t)) axpro_audio_write;
    }
    return true;
}


// WiiU specific extensions
//----------------------------------------------------

// in order to parallelize audio processing (float conversion, stereo splitting, audio interfacing) with the main core (which does the emulation)
// fence should be waited on before writing into buffer last passed to driver write call
// this needs to be done in RA for WiiU before writing into audio_st->output_samples_buf passed to audio_st->resampler->process(...)
void axpro_audio_wait_fence(void* context_audio_data, void* client_buffer)
{
    AXProMainCore* mc = (AXProMainCore*) context_audio_data;
    if(mc && client_buffer && mc == gm.mc && mc->reading_client_buffer)
    {
        // normally just wait on audio core thread to report it's done with buffer passed to it
        if(!(mc->flags & AXPMCF_NONBLOCK))
        {
            // wait on reading to complete before letting client write into (this) buffer again
            AXPro_Wait(mc, 0, 
                if(mc->msg.ctl.request == AXPMCTL_CLREAD_FENCEREACH) 
                {   mc->free_streamblocks = (uint16_t) mc->msg.ctl.ctlgeneral[0];
                    goto fence_reached;
                }
                else if(mc->msg.ctl.request == AXPMCTL_PING) 
                    mc->free_streamblocks = (uint16_t) mc->msg.ctl.ctlgeneral[0];
                ,
                break);
            // timeout / error / pollmode begin case
            fence_reached:
            mc->reading_client_buffer = NULL;
        }

        else
        {   // skip blocking when asked, check our message queue and adjust the internal state
            // skipping potential explicit fence wait by the core/ra will cause audible noise
            // data is also dropped by write when in this mode, however it might be acceptable (fast forwarding)
            axpro_audio_write_empty(mc, NULL, 0);
        }
    }
}

void axpro_audio_wait_fence_core(void* client_buffer) { axpro_audio_wait_fence(gm.mc, client_buffer); }


// extension for cores that provide callback for generating audio into given buffer, called from audio core thread
// when non NULL SampleGenCallback is provided it will get called from driver thread and RA driver write should not be used
// RA resampler is avoided and emulation core is called directly from audio driver thread
// scummvm could benefit from this approach
void axpro_audio_set_thread_callback(AXPRO_GEN_CALLBACK* SampleGenCallback, void* ctx)
{
    gm.msg_setcallback.Call = NULL;
    gm.msg_setcallback.ctl.request = AXPMCTL_CHANGE_SAMPLEGEN;
    gm.msg_setcallback.ctl.setgen.samplegen_ctx = ctx;
    gm.msg_setcallback.ctl.setgen.GenerateSamples = SampleGenCallback;

    if(gm.mc && gm.mc->qmain2audio)
    AXPro_RunMsg(gm.mc->qmain2audio, gm.msg_setcallback);

    audio_axpro.write =  (ssize_t (*) (void*, const void*, size_t)) (SampleGenCallback ? axpro_audio_write_empty : axpro_audio_write);
}
