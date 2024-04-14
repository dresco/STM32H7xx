/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

// Code derived from the RP2040 & Teensy grblHAL littlefs implementations.
//
// This STM32H7 implementation supports Winbond WS25Qxx SPI flash memory.
//  - H743 parts use QuadSPI peripheral.
//  - H723 parts use OctoSPI peripheral in Quad mode.

#include "driver.h"

#if LITTLEFS_ENABLE

#include "littlefs_hal.h"
#include "spiflash.h"

// STM32 Quad/Octo SPI hardware abstraction functions

static int stm32_hal_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size)
{
    assert(block < c->block_count);
    assert(off + size <= c->block_size);

    if (spiflash_read((block * c->block_size + off), buffer, size) != SPIFLASH_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

static int stm32_hal_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size)
{
    assert(block < c->block_count);

    if (spiflash_program((block * c->block_size + off), buffer, size) != SPIFLASH_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

static int stm32_hal_erase (const struct lfs_config *c, lfs_block_t block)
{
    assert(block < c->block_count);

    if (spiflash_sector_erase(block * c->block_size) != SPIFLASH_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

static int stm32_hal_sync (const struct lfs_config *c)
{
    (void)c;

    return LFS_ERR_OK;
}

struct lfs_config *stm32_littlefs_hal (void)
{
    static struct lfs_config stm32_cfg = {
        // block device operations
        .read = stm32_hal_read,
        .prog = stm32_hal_prog,
        .erase = stm32_hal_erase,
        .sync = stm32_hal_sync,

        // block device configuration
        .read_size = 1,
        .prog_size = SPIFLASH_PAGE_SIZE,
        .block_size = SPIFLASH_SECTOR_SIZE,
        .block_count = (SPIFLASH_SIZE * 128 * 1024) / SPIFLASH_SECTOR_SIZE,
        .cache_size = SPIFLASH_SECTOR_SIZE / 4,
        .lookahead_size = SPIFLASH_PAGE_SIZE / 2,
        .block_cycles = 500
    };

    return &stm32_cfg;
}

#endif // LITTLEFS_ENABLE
