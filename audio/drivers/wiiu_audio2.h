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
#ifndef WIIU_AUDIO2_H_INCLUDED
#define WIIU_AUDIO2_H_INCLUDED

// standard includes
//--------------------------------------
#include <string.h>
#include <stdint.h>
#include <math.h>


// build system specifics
//--------------------------------------

// fcon:change this to real wut build define or just completely remove the Tiramisu RA includes below
#define WUT_BUILD (!defined(HAVE_IOSUHAX))
#if WUT_BUILD
#include <coreinit/thread.h>
#include <coreinit/messagequeue.h>
#include <coreinit/time.h>
#include <coreinit/memdefaultheap.h>
#include <coreinit/cache.h>
#include <sndcore2/core.h>
#include <sndcore2/voice.h>

// wut specific common wrappers
#define AXPro_MemAlloc(Size, Align)     MEMAllocFromDefaultHeapEx(Size, Align)
#define AXPro_MemFree(Ptr)              MEMFreeToDefaultHeap(Ptr)

#else
// build without WUT (current RetroArch build is using devkitPPC r29)
#include <wiiu/os/thread.h>
#include <wiiu/os/systeminfo.h>
#include <wiiu/os/messagequeue.h>
#include <wiiu/os/cache.h>
#include <wiiu/ax.h>
#define OS_MESSAGE_FLAGS_BLOCKING       OS_MESSAGE_QUEUE_BLOCKING
#define OS_MESSAGE_FLAGS_NONE           ((OSMessageFlags)0)
#define OSMillisecondsToTicks           OSMilliseconds
typedef void(*AXFrameCallback) (void);
AXFrameCallback AXRegisterFrameCallback(AXFrameCallback callback);

// devkitPPC r29 specific common wrappers
#define AXPro_MemAlloc(Size, Align)     MEM2_alloc(Size, Align)
#define AXPro_MemFree(Ptr)              MEM2_free(Ptr)
#endif // non WUT build


// common (wut also) missing definitions  // fcon:aroma - check where should this go
extern uint32_t LCEnableDMA();
//extern void LCDisableDMA();
extern void* LCAlloc(uint32_t bytes);
extern void LCDealloc(void* p);
extern void LCStoreDMABlocks(void* destMem, const void* srcLC, uint32_t cache_lines);

// RA includes
#include "../../wiiu/system/memory.h"
#include "../audio_driver.h"


// compile time configuration
//--------------------------------------
#define AXPRO_CORE_AUDIO                0   // coreid we will use for audio processing  (1 is by default the "main" core, 0 and 2 are available)

#define AXPRO_CACHELINE_SIZE            32  // these are pretty much set in stone
#define AXPRO_CACHEFETCH_SIZE           64
#define AXPRO_SAMPLEBUFF_ALIGNMENT      64
#define AXPRO_LCACHE_ALIGNMENT          512
#define AXPRO_INTERRUPT_INTERVAL_MS     3
#define AXPRO_SAMPLE_BYTES              2
#define AXPRO_CHANNELS                  2
#define AXPRO_VOICE_FORMAT              AX_VOICE_FORMAT_LPCM16
#define AXPRO_CLINES4FRAMES(Frames)     ((Frames) >> 3)  // (frames * AXPRO_CHANNELS * AXPRO_SAMPLE_BYTES) / AXPRO_CACHELINE_SIZE
#define AXPRO_MINBLOCKS_FOR_PLAY        2

#define AXPRO_MAX_MSGS_AUDIO2MAIN       16  // message queue sizes, irq ones might need to be adjusted dynamically to support max number of blocks and some more msgs for control msgs
#define AXPRO_MAX_MSGS_MAIN2AUDIO       32
#define AXPRO_MAX_MSGS_MAIN2IRQ         16
#define AXPRO_MAX_MSGS_AUDIO2IRQ        64
#define AXPRO_MAX_MSGS_IRQ2AUDIO        AXPRO_MAX_MSGS_AUDIO2IRQ

#define AXPRO_INIT_RENDERER             AX_INIT_RENDERER_48KHZ      // AX_INIT_RENDERER_48KHZ or AX_INIT_RENDERER_32KHZ
#define AXPRO_SAMPLE_RATE               48000                       // 48000 or 32000
#define AXPRO_INIT_PIPELINE             AX_INIT_PIPELINE_FOUR_STAGE // AX_INIT_PIPELINE_SINGLE or AX_INIT_PIPELINE_FOUR_STAGE
#define AXPRO_INIT_RESAMPLER            AX_VOICE_SRC_TYPE_LINEAR    // 0=none, 1=linear, 2=AX_VOICE_SRC_TYPE_UNK0, 3=unk1, 4=unk2
#define AXPRO_VOICE_PRIORITY            31                          // 31=max priority
#define AXPRO_POLL_LOWPOWER_TIMEOUT     1000                        // in AXPRO_INTERRUPT_INTERVAL_MS units (1000 = 3 seconds)


// streamblock size in interrupt time durations
// minimum granularity and latency, we need two streamblocks filled before playing
// so minimum latency is 1=1*2*3ms=6ms (unstable)  2=2*2*3ms=12ms  3=18ms and so on
#define AXPRO_STREAMBLOCK_INTERRUPTS    2

// streamblocks in locked cache
// must not be smaller than 2 for polling path to work (axpro_audio_set_thread_callback)
// for normal RA it must hold blocks as defined below (round up to next integer)
// (AUDIO_CHUNK_SIZE_NONBLOCKING * 500 / sample_rate) / (AXPRO_STREAMBLOCK_INTERRUPTS * AXPRO_INTERRUPT_INTERVAL_MS)
#define AXPRO_STREAMBLOCKS_LC           3

// very simple rate control
// can help smooth out underruns and maintain continuous playback when input is tiny bit slower than output
#define AXPRO_RATE_CONTROL_AMORTIZER    1
#define AXPRO_RATE_LOW                  100.0f  // Hz lower than requested rate
#define AXPRO_RATE_LOWER                200.0f  // Hz lower than requested rate
#define AXPRO_RATE_DURATION_NORMAL      0.5f    // seconds at least in normal rate state
#define AXPRO_RATE_DURATION_LOW         0.05f   // seconds at least in lower rate states


// RA/core integration optimization setup
#ifndef WIIU_AUDIO_OPTIMIZATION_LEVEL
#define WIIU_AUDIO_OPTIMIZATION_LEVEL 0     // optimization level should come defined as RA build switch
#endif

#if WIIU_AUDIO_OPTIMIZATION_LEVEL == 0      // default - full RA options supported, no changes needed in RA or core code
#define AXPRO_ACCEPTS_FLOAT_SAMPLES     1   // take the float conversion off the main thread 
#define AXPRO_IMPLICIT_NONBLOCK_OPTIM   0   // wait in write until we read all the client data

#elif WIIU_AUDIO_OPTIMIZATION_LEVEL == 1    // lite optimization - full RA options supported, lite changes needed in RA code
#define AXPRO_ACCEPTS_FLOAT_SAMPLES     1   // take the float conversion off the main thread 
#define AXPRO_IMPLICIT_NONBLOCK_OPTIM   1   // RA will call axpro_audio_wait_fence before writing into memory passed to previous write call

#elif WIIU_AUDIO_OPTIMIZATION_LEVEL == 2    // fast direct int16 - no RA options like FF, record, rate control, etc. Changes RA to avoid parts of audio pipeline
#define AXPRO_ACCEPTS_FLOAT_SAMPLES     0   // we're working with int16
#define AXPRO_IMPLICIT_NONBLOCK_OPTIM   0   // wait in write until we read all the client data

#elif WIIU_AUDIO_OPTIMIZATION_LEVEL == 3    // custom core mode - no RA options like FF, record, rate control, etc. Changes RA to avoid parts of audio pipeline. Core changes needed.
#define AXPRO_ACCEPTS_FLOAT_SAMPLES     0   // we're working with int16
#define AXPRO_IMPLICIT_NONBLOCK_OPTIM   1   // core will call axpro_audio_wait_fence before writing into memory passed to previous write call or will not use write call by using axpro_audio_set_thread_callback
#endif


// common utils
//--------------------------------------
#ifdef _MSC_VER
#define AXPRO_ALIGNED_DECLARE(Declaration, AlignBytes) __declspec(align(AlignBytes)) Declaration
#else
#define AXPRO_ALIGNED_DECLARE(Declaration, AlignBytes) Declaration __attribute__((aligned(AlignBytes)))
#endif
#define AXPRO_ALIGNED_DECLARE_CLINE1(Declaration) AXPRO_ALIGNED_DECLARE(Declaration, AXPRO_CACHELINE_SIZE)
#define AXPRO_ALIGNED_DECLARE_CLINE2(Declaration) AXPRO_ALIGNED_DECLARE(Declaration, AXPRO_CACHEFETCH_SIZE)
#define AXPRO_ALIGN_SIZE(_v, _s) _v += (_v & (_s-1)) ? _s - (_v & (_s-1)) : 0

#define AXPro_RunMsg(Queue, MsgRef)                                         \
    OSSendMessage(Queue, (OSMessage*)(&MsgRef), OS_MESSAGE_FLAGS_BLOCKING)

#define AXPro_RunCtlEx(Queue, Request, CtlData, MsgFlags)       \
{   AXPRO_ALIGNED_DECLARE_CLINE1(OSMessage msg);                \
    msg.message = NULL;                                         \
    ((AXProMsgCtlPart*)msg.args)->request = Request;            \
    ((AXProMsgCtlPart*)msg.args)->ctlgeneral[0] = CtlData;      \
    OSSendMessage(Queue, &msg, MsgFlags);                       \
}
#define AXPro_RunCtl(Queue, Request) AXPro_RunCtlEx(Queue, Request, AXPINGF_NONE, OS_MESSAGE_FLAGS_BLOCKING)
#define AXPro_PostCtl(Queue, Request) AXPro_RunCtlEx(Queue, Request, AXPINGF_NONE, OS_MESSAGE_FLAGS_NONE)

#define AXPro_Run(Queue, Call)                      \
{   AXPRO_ALIGNED_DECLARE_CLINE1(OSMessage msg);    \
    msg.message = (void*) (Call);                   \
    AXPro_RunMsg(Queue, msg);                       \
}

#define AXPro_Wait(CoreData, WaitCond, OnCtlMsg, OnError)               \
{   while(!(WaitCond))                                                  \
        if(OSReceiveMessage((OSMessageQueue*) (CoreData),               \
                            (OSMessage*)&((CoreData)->msg),             \
                            OS_MESSAGE_FLAGS_BLOCKING))                 \
        {   if((CoreData)->msg.Call) (CoreData)->msg.Call(CoreData);    \
            else { OnCtlMsg; };                                         \
        }                                                               \
        else { OnError; };                                              \
}


// common types and defines
//--------------------------------------
typedef struct AXProMainCore AXProMainCore;
typedef struct AXProAudioCore AXProAudioCore;
typedef void (AXPRO_PROCAUDIO) (AXProAudioCore* ac);
typedef void (AXPRO_PROCMAIN) (AXProMainCore* mc);

// samplegen callback types - returns number of frames (samples for L and R channels) written
typedef int32_t (AXPRO_GEN_CALLBACK) (void* ctx, int16_t* samples, uint32_t len_bytes);
typedef struct AXProSampleGenData
{
    void*               samplegen_ctx;
    AXPRO_GEN_CALLBACK* GenerateSamples;

} AXProSampleGenData;

#if AXPRO_ACCEPTS_FLOAT_SAMPLES
typedef float AXProInSample;
#else
typedef int16_t AXProInSample;
#endif

// control message ids
#define AXPMCTL_QUIT                    1
#define AXPMCTL_CHANGE_SAMPLEGEN        2
#define AXPMCTL_ERROR_INIT              3
#define AXPMCTL_BLOCK_READY             4
#define AXPMCTL_BLOCK_PLAYED            5
#define AXPMCTL_PING                    6
#define AXPMCTL_CLREAD_FENCEREACH       7

// ping piggybacking subcommands
#define AXPINGF_NONE                    0x00
#define AXPINGF_SET                     0x40
#define AXPINGF_RESET                   0x80

// interrupt handler state
#define AXPACS_STOPPED                  0
#define AXPACS_BUFFERING_BEFORE_PLAY    1
#define AXPACS_PLAYING                  2
#define AXPACS_STOPPING                 3

// main core flags
#define AXPMCF_HW_READY                 0x01
#define AXPMCF_HW_CLOSED                0x02
#define AXPMCF_POLLMODE                 0x04
#define AXPMCF_STOPPED                  0x08
#define AXPMCF_NONBLOCK                 0x10

// interrupt handler flags
#define AXPIHF_POLLMODE                 0x01

// audio core flags
#define AXPACF_STOPPED_LONGAGO          0x01


// message types
//--------------------------------------

typedef struct AXProMsgCtlPart
{
    uint8_t             request;            // AXPMCTL_...
    uint8_t             _res1b[3];
    union {
    AXProSampleGenData  setgen;
    uint32_t            pingreqiest;        // AXPIHF_...
    uint32_t            ctlgeneral[2];
    };
} AXProMsgCtlPart;


typedef struct AXProMsgAudio2Main
{
    AXPRO_PROCMAIN*         Call;
    union
    {   AXProMsgCtlPart     ctl;
        struct
        {   AXProAudioCore* ac;
            OSMessageQueue* q2audio;
            OSMessageQueue* q2irq;
        } ready;
        struct
        {   uint16_t        streamblocks;
        } hwready;
        uint32_t            general_args[3];
    }; // union

} AXProMsgAudio2Main;

typedef struct AXProMsgMain2Audio
{
    AXPRO_PROCAUDIO*        Call;
    union
    {   AXProMsgCtlPart     ctl;
        struct
        {   uint32_t        rate;
            uint32_t        latency;
        } init;
        struct
        {   AXProInSample*  frame_src;
            uint32_t        frame_num;
        } write;
        uint32_t            general_args[3];
    }; // union

} AXProMsgMain2Audio;



// interrupt handler data, kept in separate cache lines

typedef struct AXProFrameCallbackData
{
//----  // read/write cache line
    uint8_t                     state;          // AXPACS_...
    uint8_t                     flags;          // AXPIHF_...
    uint16_t                    playing_blockid;
    uint32_t                    prev_loopcount;
    uint32_t                    prepared_streamblocks;

    uint16_t                    stopped_state_duration;
    
    uint16_t                    rate_idx_set;
    uint16_t                    rate_change_timeout;
    uint16_t                    _resw1;
    float                       rate_src_ratios[3];
//---   // read only cache line
    AXVoice*                    voice_l;
    AXVoice*                    voice_r;

    uint16_t                    streamblock_frames;
    uint16_t                    streamblocks;

    uint16_t                    streamblock_samples;
    uint16_t                    frames_per_interrupt;

    int16_t*                    voicebuff_l;
    AXFrameCallback             prev_ax_framecallback;

    uint16_t                    rate_blocks_hi;
    uint16_t                    rate_blocks_low;

    uint16_t                    rate_change_timeout_normal;
    uint16_t                    rate_change_timeout_slow;
//----

} AXProFrameCallbackData;



// front core data
//--------------------------------------
typedef struct AXProMainCore
{
//----
    OSMessageQueue              qaudio2main;
    OSMessageQueue*             qmain2audio;
//----
    OSMessage                   msgstore_audio2main[AXPRO_MAX_MSGS_AUDIO2MAIN];
//----
    AXProMsgAudio2Main          msg;
    AXProMsgMain2Audio          writemsg;
//---
    OSMessageQueue*             qmain2irq;
    uint32_t                    flags;      // AXPMCF_...
    AXProAudioCore*             ac;         // main core should only pass it as callback context to be used from audio core
    void*                       reading_client_buffer;
//--
    uint16_t                    free_streamblocks;
    uint16_t                    total_streamblocks;
    uint32_t                    _resu1[3];
//---

} AXProMainCore;




// audio core data
//--------------------------------------
typedef struct AXProAudioCore
{
//----
    OSMessageQueue              qmain2audio;
    OSMessageQueue*             qaudio2main;
//----
    OSMessage                   msgstore_main2audio[AXPRO_MAX_MSGS_MAIN2AUDIO];
//----
    AXProFrameCallbackData      interrupt_data;
//----  // read/write data
    AXProMsgMain2Audio          msg;
//--
    uint16_t                    lc_missing_block_frames;    // frames missing to fill the current block
    uint16_t                    voicebuff_free_blocks;
    int16_t*                    lc_block_writepos;
    int16_t*                    lc_block_base;
    int16_t*                    voicebuff_writeblock;
//---   // read only data
    uint16_t                    streamblock_frames;
    uint16_t                    streamblocks;
    int16_t*                    voicebuff;
    int16_t*                    lc_base;
    int16_t*                    lc_base_circular;
//--
    int16_t*                    lc_last_block;
    int16_t*                    voicebuff_last_block;
    uint32_t                    streamblock_bytes;
    uint32_t                    streamblock_channel_bytes;
//---
//----  // read/write queue shared data
    AXPRO_ALIGNED_DECLARE_CLINE2(OSMessageQueue qirq2audio);
    void*                       _resp2;
//----
    OSMessage                   msgstore_irq2audio[AXPRO_MAX_MSGS_IRQ2AUDIO];
//----
    OSMessageQueue              qaudio2irq;
    void*                       _resp3;
//----
    OSMessage                   msgstorage_audio2irq[AXPRO_MAX_MSGS_AUDIO2IRQ];
//----
    OSMessageQueue              qmain2irq;
    void*                       _resp4;
//----
    OSMessage                   msgstorage_main2irq[AXPRO_MAX_MSGS_MAIN2IRQ];
//----
} AXProAudioCore;


// WiiU specific extensions

// in order to parallelize audio processing (float conversion, stereo splitting, audio interfacing) with the main core (which does the emulation)
// fence should be waited on before writing into buffer last passed to driver write call
// this can be done for example in RA for WiiU before writing into audio_st->output_samples_buf passed to audio_st->resampler->process(...)
void axpro_audio_wait_fence(void* context_audio_data, void* client_buffer);
void axpro_audio_wait_fence_core(void* client_buffer);


// extension for cores that provide callback for generating audio into given buffer, called from audio core thread
// when non NULL SampleGenCallback is provided it will get called from driver thread and RA driver write should not be used
// RA resampler is avoided and emulation core is called directly from audio driver thread
// scummvm could benefit from this approach
void axpro_audio_set_thread_callback(AXPRO_GEN_CALLBACK* SampleGenCallback, void* ctx);

#endif
