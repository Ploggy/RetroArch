/* Copyright  (C) 2010-2021 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (jitmem_intf.c).
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

#include <boolean.h>

bool jitmem_available() {

}

void jitmem_max_allocation(int* num_allocs, size_t* max_alloc_size) {

}

//most OSes probably won't need this
typedef struct {
   bool writeable;
   int os_specific_whatever;
   char mem[];
} jitmem_alloc;

void* jitmem_alloc(size_t size) {
   jitmem_alloc* allocation = malloc(sizeof(jitmem_alloc) + size);
   allocation->writeable = false;
   return &allocation->mem;
}

void jitmem_free(void* ptr) {
   free(ptr - sizeof(jitmem_alloc));
}

bool jitmem_lock(void* ptr) {
   if (mprotect(ptr, M_WHATEVER)) return true;
}

bool jitmem_unlock(void* ptr) {
   if (mprotect(ptr, M_WHATEVER)) return true;
}
