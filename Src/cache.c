/*

  cache.c - Cache maintenance helper code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2024 Jon Escombe

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"
#include "cache.h"

/*
 * DMA buffers should start and end on data cache line boundaries (32 bytes for Cortex-M7),
 * in order to avoid cache maintenance side effects on any surrounding data.
 *
 * Note that memory allocated with cache_aligned_malloc() **must** be freed with cache_aligned_free().
 *
 * Code based on alligned_malloc() example from https://embeddedartistry.com/
 */

typedef uint16_t offset_t;

#ifndef align_up
    #define align_up(num, align) (((num) + ((align)-1)) & ~((align)-1))
#endif

/**
 * This function returns a suitablely aligned pointer, and internally pads the allocation
 * to end with the correct alignment.
 */
void* cache_aligned_calloc(size_t num, size_t size)
{
    void* ptr = NULL;

    if(num && size)
    {
        size_t align = __SCB_DCACHE_LINE_SIZE;
        size_t aligned_size = align_up(num * size, align);

        /*
         * We know we have to fit an offset value
         * We also allocate extra bytes to ensure we can meet the alignment
         */
        uint32_t hdr_size = sizeof(offset_t) + (align - 1);
        void* p = calloc(aligned_size + hdr_size, sizeof(uint8_t));

        if(p)
        {
            memset (p, 0, aligned_size + hdr_size);

            /*
             * Add the offset size to malloc's pointer (we will always store that)
             * Then align the resulting value to the target alignment
             */
            ptr = (void*)align_up(((uintptr_t)p + sizeof(offset_t)), align);

            // Calculate the offset and store it behind our aligned pointer
            *((offset_t*)ptr - 1) = (offset_t)((uintptr_t)ptr - (uintptr_t)p);

            debug_print("cache_aligned_calloc(): size (orig):%u, size (align):%u\n", size * num, aligned_size + hdr_size);
            debug_print("cache_aligned_calloc(): addr (orig):0x%08lx, addr (align):0x%08lx \n", p, ptr);

        } // else NULL, could not malloc
    } // else NULL, invalid arguments

    return ptr;
}

/**
 * Work backwards from the returned pointer to find the correct
 * offset and pointer location to return to free().
 */
void cache_aligned_free(void* ptr)
{
    if (ptr) {
    /*
     * Walk backwards from the passed-in pointer to get the pointer offset
     * We convert to an offset_t pointer and rely on pointer math to get the data
     */
    offset_t offset = *((offset_t*)ptr - 1);

    /*
     * Once we have the offset, we can get our original pointer and call free
     */
    void* p = (void*)((uint8_t*)ptr - offset);
    free(p);
    }
}
