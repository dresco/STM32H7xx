/*

  cache.h - Cache maintenance helper code for STM32H7xx ARM processors

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

#ifndef __CACHE_H__
#define __CACHE_H__

#ifndef align_up
#define align_up(num, align) (((num) + ((align)-1)) & ~((align)-1))
#endif

bool  is_cache_aligned(void *ptr, uint32_t len);
void* cache_aligned_calloc(size_t num, size_t size);
void  cache_aligned_free(void* ptr);

#endif
