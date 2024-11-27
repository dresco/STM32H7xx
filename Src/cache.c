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

typedef uint16_t offset_t;

/*
 * Notes for future self..
 *
 * H7 requires more cache management than F7, as DMA cannot access the tightly coupled memory.
 * Therefore need to use SDRAM and deal with cache coherence when using DMA.
 *
 * Options are;
 *
 * 1) Disable data cache overall.
 * 2) Place DMA buffers in non-cacheable memory (configured by MPU).
 * 3) Use cache management APIs to clean (flush), and invalidate as needed.
 *
 * Assumption is that disabling cache would have a negative impact on performance.
 * Can be disabled for debugging/testing by defining L1_CACHE_ENABLE to 0.
 *
 * The ST derived Ethernet code does use various MPU configured memory regions for buffers
 * and DMA descriptors.
 *
 * When using cache management APIs, DMA buffers should be aligned to start and end on cache
 * line boundaries (32 bytes for Cortex M7). If this is not done, there is potential for
 * corruption of surrounding data.
 *
 * Cache management API usage
 * ==========================
 *
 * DMA Tx
 * ------
 * Use SCB_CleanDCache_by_Addr() before the DMA transfer.
 *
 * Reasonably safe to use with an unaligned buffer.(Exception would be if there was an
 * unrelated DMA Rx buffer sharing the same cache line, where memory written by DMA could
 * be overwritten with stale data from the cache).
 *
 * DMA Rx
 * ------
 * Current thinking is to use SCB_InvalidateDCache_by_Addr() before and after the DMA transfer.
 * This ensures that cache eviction will not write back to buffer memory during the transfer,
 * and also discards any stale buffer data that could be cached by speculative reads during the transfer.
 * See https://community.st.com/t5/stm32-mcus-products/maintaining-cpu-data-cache-coherence-for-dma-buffers/m-p/95746
 *
 * Not safe to use with an unaligned buffer.
 * Can use SCB_CleanInvalidateDCache_by_Addr() before the DMA transfer to ensure any cached data
 * surrounding the buffer is written back to memory. However the second invalidate following the
 * transfer would discard any further cached writes made to this surrounding data.
 *
 * In this instance, is safer to use a correctly aligned scratch buffer for DMA, at the cost
 * of an additional memory copy from the buffer.
 *
 * DMA alignment
 * =============
 *
 * Note also that it may be necessary to use a scratch buffer for DMA, even without cache considerations.
 * Buffers need to be correctly aligned for DMA (1, 2, or 4 byte alignment, depending on the configured width).
 * This is seen in the SDMMC code, as the pointers passed from FATFS are not always (4 byte) aligned.
 *
 */

/*
 * Check both start & end addresses are on cache line boundaries (32 bytes for Cortex-M7).
 */
bool is_cache_aligned(void* ptr, uint32_t len)
{
    bool start_aligned = !((uint32_t)ptr       & (__SCB_DCACHE_LINE_SIZE-1));
    bool end_aligned   = !(((uint32_t)ptr+len) & (__SCB_DCACHE_LINE_SIZE-1));

    return (start_aligned && end_aligned);
}

/*
 * Cache aligned calloc and free routines.
 *
 * Note that memory allocated with cache_aligned_malloc() **must** be freed with cache_aligned_free().
 *
 * Code based on alligned_malloc() example from https://embeddedartistry.com/
 */

/*
 * This function returns a suitably aligned pointer, and internally pads the allocation
 * to end on the correct alignment.
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

        } // else NULL, could not malloc
    } // else NULL, invalid arguments

    return ptr;
}

/*
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
