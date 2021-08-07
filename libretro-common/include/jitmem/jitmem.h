/* Copyright  (C) 2010-2021 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (jitmem.h).
 * ---------------------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __LIBRETRO_SDK_JITMEM_H
#define __LIBRETRO_SDK_JITMEM_H

#include <stddef.h>
#include <boolean.h>

#include <retro_common_api.h>

RETRO_BEGIN_DECLS

typedef struct jitmem_intf
{
   bool (*available)();

   void (*max_allocation)(int* num_allocs, size_t* max_alloc_size);

   void *(*alloc)(size_t size);

   bool (*free)(void *ptr);

   bool (*lock)(void *ptr);

   bool (*unlock)(void *ptr);

   /* Human readable string. */
   const char *ident;
} jitmem_intf_t;

/* Checks if jitmem is available in the current environment.
 * Returns true if jitmem is implemented and enabled in the OS.
 */
bool jitmem_available();

/* Returns the maximum number of possible allocations, and the largest possible
 * allocation for this platform. num_allocs may be -1 on OSes with no limit on
 * jitmem allocations, or as low as 1 on other platforms.
 */
void jitmem_max_allocation(int* num_allocs, size_t* max_alloc_size);

/* Allocates a block of jit-able memory. Will start in the locked (writable)
 * state.
 */
void *jitmem_alloc(size_t size);

/* Frees a block of jit-able memory.
 */
bool jitmem_free(void *ptr);

/* Locks a block of jitmem for writing. On OSes where this is relevant, this
 * function switches the block to rw- mode, allowing writing but not executing.
 * Do not jump into this block before calling jitmem_unlock.
 */
bool jitmem_lock(void *ptr);

/* Unlocks a block of jitmem, allowing it to be executed. On OSes where relevant,
 * this function switches the block to r-x mode, allowing execution but not
 * writing.
 */
bool jitmem_unlock(void *ptr);

RETRO_END_DECLS

#endif
