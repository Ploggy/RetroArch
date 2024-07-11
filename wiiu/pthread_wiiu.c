#ifdef HW_WUP
#include "pthread_wiiu.h"
#include <errno.h>
#include <string.h>
#include <malloc.h>
#include <coreinit/thread.h>


// default creation attributes
#define WIIU_PTHREAD_DEFAULT_STACK_SIZE         131072                          // in bytes
#define WIIU_PTHREAD_DEFAULT_AFFINITY           OS_THREAD_ATTRIB_AFFINITY_CPU1  // core 0, 1, 2 or any, core1 is the "main" core
#define WIIU_PTHREAD_FAST_MUTEX_AND_COND        1                               // mutex and cond var with a fast path


#if WIIU_FAST_MUTEX_AND_COND
#include <coreinit/fastmutex.h>
#include <coreinit/fastcondition.h>
typedef OSFastMutex os_mutex_t
typedef OSFastCondition os_cond_t
#define ptwum_mutex_init(Mutex)         OSFastMutex_Init(Mutex, NULL)
#define ptwum_mutex_lock                OSFastMutex_Lock
#define ptwum_mutex_trylock             OSFastMutex_TryLock
#define ptwum_mutex_unlock              OSFastMutex_Unlock
#define ptwum_cond_init(Cond)           OSFastCond_Init(Cond, NULL)
#define ptwum_cond_signal               OSFastCond_Signal
#define ptwum_cond_broadcast            OSFastCond_Signal
#define ptwum_cond_wait                 OSFastCond_Wait
#else
#include <coreinit/mutex.h>
#include <coreinit/condition.h>
typedef OSMutex os_mutex_t;
typedef OSCondition os_cond_t;
#define ptwum_mutex_init                OSInitMutex
#define ptwum_mutex_lock                OSLockMutex
#define ptwum_mutex_trylock             OSTryLockMutex
#define ptwum_mutex_unlock              OSUnlockMutex
#define ptwum_cond_init                 OSInitCond
#define ptwum_cond_signal               OSSignalCond
#define ptwum_cond_broadcast            OSSignalCond
#define ptwum_cond_wait                 OSWaitCond
#endif

#include <coreinit/memdefaultheap.h>
#define ptwum_memalloc(Size, Align)     memalign(Align, Size);
#define ptwum_memfree(Ptr)              free(Ptr)
#define ptwum_align_size(_v, _s)        _v += (_v & ((_s)-1)) ? (_s) - (_v & ((_s)-1)) : 0

// abusing contentionscope slot for affinity and sched inheriting
#define ptwum_attr_affinity_set(Atr, A) (Atr)->contentionscope = ((((Atr)->contentionscope >> 3) << 3) | ((A) & 0x07))
#define ptwum_attr_affinity_get(Atr)    ((Atr)->contentionscope & 0x07)
#define ptwum_attr_inhersch_set(Atr, I) (Atr)->contentionscope = (((Atr)->contentionscope & ~0xF0) | (((I) & 0xF) << 4))
#define ptwum_attr_inhersch_get(Atr)    (((Atr)->contentionscope >> 4) & 0x0F)
#define ptwum_prio_sched2os(SchedPrio)  (31-(SchedPrio))

typedef struct pthread_wiiu_footer
{
    uint8_t*                alloc_base;
    uint32_t                unusedu[7];
//---
    OSThread                thread_handle;

} pthread_wiiu_footer;


static void pthread_wiiu_dealloc(OSThread* t, void* stack)
{
    pthread_wiiu_footer* footer = (pthread_wiiu_footer*) ((uint8_t*) t - (uintptr_t) (&((pthread_wiiu_footer*)NULL)->thread_handle));
    ptwum_memfree(footer->alloc_base);
}

static int pthread_wiiu_start(int arg, const char** start_routine)
{
    // horrible casting just to silence the pedantic compilers
    // we're running on a 32 bit system where passing any pointers as integers is fine (including function pointers)
    return (int) ((intptr_t) ((void* (*)(void*)) ((uintptr_t) start_routine)) ((void*)arg));
}

int pthread_create(pthread_t* thread, const pthread_attr_t* attr, void *(*start_routine)(void*), void *arg)
{
    uint8_t flags = attr ? ( ptwum_attr_affinity_get(attr) | 
                            ((attr->detachstate == PTHREAD_CREATE_DETACHED) ? OS_THREAD_ATTRIB_DETACHED : 0)
                           ) : WIIU_PTHREAD_DEFAULT_AFFINITY;
    uint8_t priority = (attr && (ptwum_attr_inhersch_get(attr) == PTHREAD_EXPLICIT_SCHED)) ? 
                        ptwum_prio_sched2os(attr->schedparam.sched_priority) : 
                        OSGetThreadPriority(OSGetCurrentThread());
    uint32_t stack_size = attr ? attr->stacksize : WIIU_PTHREAD_DEFAULT_STACK_SIZE;
    uint32_t footer_size = sizeof(pthread_wiiu_footer);
    uint8_t* user_stackptr = attr ? (uint8_t*) attr->stackaddr : NULL;
    uint32_t alloc_size;
    uint8_t* alloc_base, *stack_top;
    pthread_wiiu_footer* footer;

    ptwum_align_size(footer_size, 32);
    if(!user_stackptr)
    {   ptwum_align_size(stack_size, 32);
        alloc_size = footer_size + stack_size;
    }
    else alloc_size = footer_size;

    alloc_base = ptwum_memalloc(alloc_size, 64);
    if(alloc_base)
    {   memset(alloc_base, 0, alloc_size);
        footer = (pthread_wiiu_footer*) (alloc_base + (!user_stackptr ? stack_size : 0));
        stack_top = !user_stackptr ? (uint8_t*) footer : (user_stackptr + stack_size);
        footer->alloc_base = alloc_base;

        if(OSCreateThread(&footer->thread_handle,
            pthread_wiiu_start,
            (int32_t) ((intptr_t) arg), (char*) ( (uintptr_t) start_routine),
            stack_top, stack_size,
            priority, flags
        ))
        {
            (*thread) = (pthread_t) ((uintptr_t) &footer->thread_handle);
            OSSetThreadDeallocator(&footer->thread_handle, pthread_wiiu_dealloc);
            OSResumeThread(&footer->thread_handle);
            return 0;
        }
    }

    // error creating something
    if(alloc_base)
    ptwum_memfree(alloc_base);
    *thread = 0;
    return 1;
}

int pthread_join(pthread_t t, void** retval) { return !OSJoinThread((OSThread*) ((uintptr_t) t), (int*) retval); }
int pthread_cancel(pthread_t t) { OSCancelThread((OSThread*) ((uintptr_t) t)); return 0; }
void pthread_exit (void* retval) { OSExitThread((int)((intptr_t)retval)); while(1); }
void pthread_yield() { OSYieldThread(); }
pthread_t pthread_self() { return (pthread_t) ((uintptr_t) OSGetCurrentThread()); }
int pthread_equal(pthread_t t1, pthread_t t2) { return t1 == t2; }

int pthread_detach (pthread_t t)
{   OSDetachThread((OSThread*) ((uintptr_t) t));
    return 0;
}

int pthread_getname_np(pthread_t t, char* name, size_t size)
{   strncpy(name, OSGetThreadName((OSThread*) ((uintptr_t) t)), size);
    return 0;
}

int pthread_setname_np(pthread_t t, const char* name)
{   OSSetThreadName((OSThread*) ((uintptr_t) t), name);
    return 0;
}



// mutex

int pthread_mutex_init(pthread_mutex_t* mutex, const pthread_mutexattr_t* attr)
{
    os_mutex_t* osmutex = (os_mutex_t*) ptwum_memalloc(sizeof(os_mutex_t), 32);
    if(osmutex)
    {
        ptwum_mutex_init(osmutex);
        *((os_mutex_t**) mutex) = osmutex;
        return 0;
    }
    else
    {   *((int*) mutex) = 0;
        return 1;
    }
}

int pthread_mutex_destroy(pthread_mutex_t* mutex)
{
    os_mutex_t* osmutex = *((os_mutex_t**) mutex);
    if(osmutex) ptwum_memfree(osmutex);
    return 0;
}

int pthread_mutex_trylock(pthread_mutex_t* mutex) { return ptwum_mutex_trylock(*((os_mutex_t**)mutex)) ? 0 : EBUSY; }
int pthread_mutex_lock(pthread_mutex_t* mutex) { ptwum_mutex_lock(*((os_mutex_t**)mutex)); return 0; }
int pthread_mutex_unlock(pthread_mutex_t* mutex) { ptwum_mutex_unlock(*((os_mutex_t**)mutex)); return 0; }



// condition variable

int pthread_cond_signal(pthread_cond_t* cond) { ptwum_cond_signal(*((os_cond_t**)cond)); return 0; }
int pthread_cond_wait(pthread_cond_t* cond, pthread_mutex_t* mutex) { ptwum_cond_wait(*((os_cond_t**)cond), *((os_mutex_t**)mutex)); return 0; }
int pthread_cond_broadcast(pthread_cond_t* cond) { ptwum_cond_broadcast(*((os_cond_t**)cond)); return 0; }

int pthread_cond_init(pthread_cond_t* cond, const pthread_condattr_t* attr)
{
    os_cond_t* oscond = (os_cond_t*) ptwum_memalloc(sizeof(os_cond_t), 32);
    if(oscond)
    {
        ptwum_cond_init(oscond);
        *((os_cond_t**) cond) = oscond;
        return 0;
    }
    else
    {   *((int*) cond) = 0;
        return 1;
    }
}

int pthread_cond_destroy(pthread_cond_t* cond)
{
    os_cond_t* oscond = *((os_cond_t**) cond);
    if(oscond) ptwum_memfree(oscond);
    return 0;
}



// semaphore

#include "semaphore.h"

int sem_init(sem_t* sem, int pshared, unsigned int value)
{   OSInitSemaphore(sem, value);
    return 0;
}
int sem_post(sem_t* sem) { OSSignalSemaphore(sem); return 0; }
int sem_wait(sem_t* sem) { OSWaitSemaphore(sem); return 0; }
int sem_trywait(sem_t* sem) { OSTryWaitSemaphore(sem); return 0; }
int sem_destroy(sem_t* sem) { return 0; }

int sem_getvalue(sem_t* sem, int* sval)
{   *sval = OSGetSemaphoreCount(sem);
    return 0;
}



// thread creation attributes

int pthread_attr_init(pthread_attr_t* attr)
{
    memset(attr, 0, sizeof(pthread_attr_t));
    attr->is_initialized = true;
    attr->stacksize = WIIU_PTHREAD_DEFAULT_STACK_SIZE;
    ptwum_attr_affinity_set(attr, WIIU_PTHREAD_DEFAULT_AFFINITY);
    ptwum_attr_inhersch_set(attr, PTHREAD_INHERIT_SCHED);
    attr->schedparam.sched_priority = -1;
    attr->detachstate = PTHREAD_CREATE_JOINABLE;
    return 0;
}

int pthread_attr_destroy(pthread_attr_t* attr) { return 0; }

int pthread_attr_setstack(pthread_attr_t* attr, void* stackaddr, size_t stacksize)
{   attr->stackaddr = stackaddr;
    attr->stacksize = stacksize;
    return 0;
}

int pthread_attr_getstack(const pthread_attr_t* attr, void** stackaddr, size_t* stacksize)
{   *stackaddr = attr->stackaddr;
    *stacksize = attr->stacksize;
    return 0;
}

int pthread_attr_getstacksize(const pthread_attr_t* attr, size_t* stacksize)
{   *stacksize = attr->stacksize;
    return 0;
}

int pthread_attr_setstacksize(pthread_attr_t* attr, size_t stacksize)
{   attr->stacksize = stacksize;
    return 0;
}

int pthread_attr_getstackaddr(const pthread_attr_t* attr, void** stackaddr)
{   *stackaddr = attr->stackaddr;
    return 0;
}

int pthread_attr_setstackaddr(pthread_attr_t* attr, void* stackaddr)
{   attr->stackaddr = stackaddr;
    return 0;
}

int pthread_attr_getdetachstate(const pthread_attr_t* attr, int* detachstate)
{   *detachstate = attr->detachstate;
    return 0;
}

int pthread_attr_setdetachstate(pthread_attr_t* attr, int detachstate)
{   attr->detachstate = detachstate;
    return 0;
}

int pthread_attr_setaffinity_np(pthread_attr_t* attr, size_t cpusetsize, const cpu_set_t* cpuset)
{   ptwum_attr_affinity_set(attr, cpuset->__bits[0]);
    return 0;
}

int pthread_attr_getaffinity_np(const pthread_attr_t* attr, size_t cpusetsize, cpu_set_t* cpuset) 
{   cpuset->__bits[0] = ptwum_attr_affinity_get(attr);
    return 0;
}

int pthread_setaffinity_np(pthread_t t, size_t cpusetsize, const cpu_set_t* cpuset)
{ return !OSSetThreadAffinity((OSThread*) ((uintptr_t) t), cpuset->__bits[0] & 7); }

int pthread_getaffinity_np(const pthread_t t, size_t cpusetsize, cpu_set_t* cpuset)
{   cpuset->__bits[0] = OSGetThreadAffinity((OSThread*) ((uintptr_t) t));
    return 0;
}

int sched_get_priority_max(int policy) { return 31; }
int sched_get_priority_min(int policy) { return 0; }

int pthread_attr_setschedparam(pthread_attr_t* attr, const struct sched_param* param)
{
    if(param->sched_priority >= 0 && param->sched_priority <= 31)
    {   attr->schedparam.sched_priority = param->sched_priority;
        // make sure prio will be used even if client code forgets about pthread_attr_setinheritsched
        ptwum_attr_inhersch_set(attr, PTHREAD_EXPLICIT_SCHED);
        return 0;
    }
    return EINVAL;
}

int pthread_attr_getschedparam(const pthread_attr_t* attr, struct sched_param* param)
{   param->sched_priority = attr->schedparam.sched_priority;
    return 0;
}

int	pthread_attr_setinheritsched(pthread_attr_t* attr, int inheritsched)
{   
    ptwum_attr_inhersch_set(attr, inheritsched);
    // disable going into create with invalid prio
    // this should not normally happen as PTHREAD_EXPLICIT_SCHED goes with pthread_attr_setschedparam
    if((inheritsched == PTHREAD_EXPLICIT_SCHED) && (attr->schedparam.sched_priority==-1))
    attr->schedparam.sched_priority = 16; 
    return 0;
}

int	pthread_attr_getinheritsched(const pthread_attr_t* attr, int* inheritsched)
{   *inheritsched = ptwum_attr_inhersch_get(attr);
    return 0;
}

int pthread_setschedprio(pthread_t t, int prio)
{   if(prio >= 0 && prio <= 31)
    return !OSSetThreadPriority((OSThread*) ((uintptr_t) t), ptwum_prio_sched2os(prio)); 
    return EINVAL;
}



#endif // HW_WUP
