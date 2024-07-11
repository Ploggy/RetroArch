#pragma once
#ifdef HW_WUP

// our compiler actually provides <pthread.h> but not the implementation afaik
#ifndef _POSIX_THREADS
#define _POSIX_THREADS
#endif
#ifndef _POSIX_THREAD_PRIORITY_SCHEDULING
#define _POSIX_THREAD_PRIORITY_SCHEDULING
#endif
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// POSIX thread APIs beyond the POSIX standard
// WiiU has 3 CPUs and we allow setting affinity as thread creation attribute using pthread_attr_setaffinity_np
// It is also possible to provide thread priority during creation using pthread_attr_setschedparam
//  with sched_param sched_priority set to integer between sched_get_priority_min and sched_get_priority_max
// Affinity can be changed using pthread_setaffinity_np
// Priority can be changed using pthread_setschedprio


// cpu_set_t should normally be defined through sched.h or indirect inclusion of sys/cpuset.h
// For WiiU this is not available and we're defining it here
#if !defined(CPU_SETSIZE) && !defined(__CPU_BITTYPE) && !defined(__CPU_BITS) && !defined(CPU_ZERO) && !defined(CPU_SET)

// only working with a single 32 bit integer bitmask - we're dealing with 3 CPUs here
#define CPU_SETSIZE    32
#define __CPU_BITTYPE  unsigned int
#define __CPU_BITS     32

typedef struct {
   __CPU_BITTYPE  __bits[ CPU_SETSIZE / __CPU_BITS ];
} cpu_set_t;

#define CPU_ZERO(_Set)          (_Set)->__bits[0] = 0
#define CPU_SET(_Cpu, _Set)     (_Set)->__bits[0] |= (1 << (_Cpu))
#define CPU_CLR(_Cpu, _Set)     (_Set)->__bits[0] &= ~(1 << (_Cpu))
#define CPU_ISSET(_Cpu, _Set)   ((_Set)->__bits[0] & (1 << (_Cpu)))
#define CPU_COUNT(_Set)         ((_Set)->__bits[0] & 7) == 0 ? 0 : (                                                                    \
                                ((_Set)->__bits[0] & 7) == 7 ? 3 : (                                                                    \
                                ((((_Set)->__bits[0] & 7) == 1) || (((_Set)->__bits[0] & 7) == 2) || (((_Set)->__bits[0] & 7)) == 4)) ? 1 : 2)

#define CPU_AND(_DestSet, _SrcSet1, _SrcSet2)   (_DestSet)->__bits[0] = (_SrcSet1)->__bits[0] & (_SrcSet2)->__bits[0]
#define CPU_OR(_DestSet, _SrcSet1, _SrcSet2)    (_DestSet)->__bits[0] = (_SrcSet1)->__bits[0] | (_SrcSet2)->__bits[0]
#define CPU_XOR(_DestSet, _SrcSet1, _SrcSet2)   (_DestSet)->__bits[0] = (_SrcSet1)->__bits[0] ^ (_SrcSet2)->__bits[0]
#define CPU_EQUAL(_Set1, _Set2)                 ((_Set1)->__bits[0] == (_Set2)->__bits[0])
#define CPU_ALLOC(_NumCpus)                     ((cpu_set_t*) malloc(sizeof(__CPU_BITTYPE)))
#define CPU_FREE(_Set)                          free(_Set)
#define CPU_ALLOC_SIZE(_NumCpus)                sizeof(__CPU_BITTYPE)

#define CPU_ZERO_S(_Size, _Set)                         CPU_ZERO(_Set)
#define CPU_SET_S(_Cpu, _Size, _Set)                    CPU_SET(_Cpu, _Set)
#define CPU_CLR_S(_Cpu, _Size, _Set)                    CPU_CLR(_Cpu, _Set)
#define CPU_ISSET_S(_Cpu, _Size, _Set)                  CPU_ISSET(_Cpu, _Set)
#define CPU_COUNT_S(_Size, _Set)                        CPU_COUNT(_Cpu, _Set)
#define CPU_AND_S(_Size, _DestSet, _SrcSet1, _SrcSet2)  CPU_AND(_DestSet, _SrcSet1, _SrcSet2)
#define CPU_OR_S(_Size, _DestSet, _SrcSet1, _SrcSet2)   CPU_OR(_DestSet, _SrcSet1, _SrcSet2)
#define CPU_XOR_S(_Size, _DestSet, _SrcSet1, _SrcSet2)  CPU_XOR(_DestSet, _SrcSet1, _SrcSet2)
#define CPU_EQUAL_S(_Size, _Set1, _Set2)                CPU_EQUAL(_Set1, _Set2)

#endif // cpu set should not come defined from anywhere on WiiU, but if it does make sure not to redefine it here


int pthread_attr_setaffinity_np (pthread_attr_t* attr, size_t cpusetsize, const cpu_set_t* cpuset);
int pthread_attr_getaffinity_np (const pthread_attr_t* attr, size_t cpusetsize, cpu_set_t* cpuset);
int pthread_setaffinity_np (pthread_t t, size_t cpusetsize, const cpu_set_t* cpuset);
int pthread_getaffinity_np (const pthread_t t, size_t cpusetsize, cpu_set_t* cpuset);
void pthread_yield();


#ifdef __cplusplus
}
#endif

#endif // HW_WUP
