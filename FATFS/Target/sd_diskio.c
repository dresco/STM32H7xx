/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_template_bspv1.c v2.1.4
   as "Use dma template" is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

 /*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/*
 * when using cacheable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
#if L1_CACHE_ENABLE
#define ENABLE_SD_DMA_CACHE_MAINTENANCE  1
#endif
/* USER CODE END enableSDDmaCacheMaintenance */

/*
* Some DMA requires 4-Byte aligned address buffer to correctly read/write data,
* in FatFs some accesses aren't thus we need a 4-byte aligned scratch buffer to correctly
* transfer data
*/
/* USER CODE BEGIN enableScratchBuffer */
#define ENABLE_SCRATCH_BUFFER
/* USER CODE END enableScratchBuffer */

/* Private variables ---------------------------------------------------------*/
#if defined(ENABLE_SCRATCH_BUFFER)
/* Scratch buffer is 32-Byte aligned for cache maintenance */
ALIGN_32BYTES(static uint8_t scratch[BLOCKSIZE]);
#endif
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static volatile  UINT  WriteStatus = 0, ReadStatus = 0;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if FF_FS_READONLY == 0
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* FF_FS_READONLY == 0 */
DRESULT SD_ioctl (BYTE, BYTE, void*);

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  FF_FS_READONLY == 0
  SD_write,
#endif /* FF_FS_READONLY == 0 */
  SD_ioctl,
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_CheckStatusWithTimeout(uint32_t timeout)
{
  uint32_t timer = HAL_GetTick();
  /* block until SDIO IP is ready again or a timeout occur */
  while(HAL_GetTick() - timer < timeout)
  {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
    {
      return 0;
    }
  }

  return -1;
}

static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{

#if !defined(DISABLE_SD_INIT)

  if(BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

#else
  Stat = SD_CheckStatus(lun);
#endif

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timeout;
#if defined(ENABLE_SCRATCH_BUFFER)
  uint8_t ret = MSD_OK;
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr = (uint32_t)buff & ~0x1F;
  uint32_t alignedLength = count*BLOCKSIZE + ((uint32_t)buff - alignedAddr);
#endif

  /*
  * ensure the SDCard is ready for a new operation
  */

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    return res;
  }

  /*
   * Sanity checks on passed buffer
   *  - 4 byte alignment for DMA transfers
   *  - 32 byte alignment for safe L1 cache operations
   *  - within suitable RAM region for SDMMC1 access
   */
  uint8_t buffAlignedDMA = !((uint32_t)buff & 0x03);
  uint8_t buffAlignedCache = !((uint32_t)buff & 0x1F);
  uint8_t buffRegionDMA = ((uint32_t)buff > D1_AXISRAM_BASE) & ((uint32_t)buff < D2_AHBSRAM_BASE);

#if defined(ENABLE_SCRATCH_BUFFER)
  /*
   * Check that passed in buffer is suitable for SDMMC DMA access, else use a scratch buffer.
   *
   * Buffer needs to be 4 byte aligned for DMA, and within the AXI SRAM in the D1 domain - as
   * SDMMC1 has no DMA access to AHB SRAM in the D2 domain.
   *
   */
  if (buffAlignedDMA && buffRegionDMA)
  {
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)

    if (!buffAlignedCache) {
      /*
       * Buffer is not 32byte aligned for L1 cache operations, therefore may be subject to
       * cache boundary corruption. Clean the aligned buffer before DMA read (will flush any
       * cached data to RAM, this should take care of any cached data surrounding the actual
       * buffer).
       */

      SCB_CleanDCache_by_Addr((uint32_t*)alignedAddr, alignedLength);
    }
#endif

    if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
                             (uint32_t) (sector),
                             count) == MSD_OK)
    {
      ReadStatus = 0;
      /* Wait that the reading process is completed or a timeout occurs */
      timeout = HAL_GetTick();
      while((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
      {
      }
      /* in case of a timeout return error */
      if (ReadStatus == 0)
      {
        res = RES_ERROR;
      }
      else
      {
        ReadStatus = 0;
        timeout = HAL_GetTick();

        while((HAL_GetTick() - timeout) < SD_TIMEOUT)
        {
          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
          {
            res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
            /*
             * If buffer is 32 byte aligned, can safely invalidate the buffer region, clearing the cached data. This
             * ensures subsequent reads access the real RAM (as written by DMA), instead of any cached data.
             *
             * If the buffer is not 32 byte aligned, clean before invalidating. This ensures that any cached data
             * outside of the buffer gets written to RAM before the cache is cleared.
             */
            if (buffAlignedCache) {
              SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, alignedLength);
            } else {
              SCB_CleanInvalidateDCache_by_Addr((uint32_t*)alignedAddr, alignedLength);
            }

#endif
            break;
          }
        }
      }
    }
#if defined(ENABLE_SCRATCH_BUFFER)
  }
    else
    {
      /* Slow path, fetch each sector a part and memcpy to destination buffer */
      int i;

      for (i = 0; i < count; i++) {
        ret = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);

        if (ret == MSD_OK) {
          /* wait until the read is successful or a timeout occurs */

          timeout = HAL_GetTick();
          while((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
          {
          }
          if (ReadStatus == 0)
          {
            res = RES_ERROR;
            break;
          }
          ReadStatus = 0;

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
          /*
          *
          * SCratch buffer is always 32 byte aligned, so safe to invalidate without cleaning first
          */
          SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif
          memcpy(buff, scratch, BLOCKSIZE);
          buff += BLOCKSIZE;
        }
        else
        {
          break;
        }
      }

      if ((i == count) && (ret == MSD_OK))
        res = RES_OK;
    }
#endif

  return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if FF_FS_READONLY == 0

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timeout;
#if defined(ENABLE_SCRATCH_BUFFER)
  uint8_t ret = MSD_OK;
  int i;
#endif

  WriteStatus = 0;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr = (uint32_t)buff & ~0x1F;
  uint32_t alignedLength = count*BLOCKSIZE + ((uint32_t)buff - alignedAddr);
#endif

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
  {
    return res;
  }

  /*
   * Sanity checks on passed buffer
   *  - 4 byte alignment for DMA transfers
   *  - within suitable RAM region for SDMMC1 access
   */
  uint8_t buffAlignedDMA = !((uint32_t)buff & 0x03);
  uint8_t buffRegionDMA = ((uint32_t)buff > D1_AXISRAM_BASE) & ((uint32_t)buff < D2_AHBSRAM_BASE);

#if defined(ENABLE_SCRATCH_BUFFER)
  /*
   * Check that passed in buffer is suitable for SDMMC DMA access, else use scratch buffer.
   */
  if (buffAlignedDMA && buffRegionDMA)
  {
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
    /* Clean the aligned buffer before DMA write (will flush any cached data to RAM) */
    SCB_CleanDCache_by_Addr((uint32_t*)alignedAddr, alignedLength);
#endif

    if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
                              (uint32_t)(sector),
                              count) == MSD_OK)
    {
      /* Wait that writing process is completed or a timeout occurs */

      timeout = HAL_GetTick();
      while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
      {
      }
      /* in case of a timeout return error */
      if (WriteStatus == 0)
      {
        res = RES_ERROR;
      }
      else
      {
        WriteStatus = 0;
        timeout = HAL_GetTick();

        while((HAL_GetTick() - timeout) < SD_TIMEOUT)
        {
          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }
#if defined(ENABLE_SCRATCH_BUFFER)
  }
    else
    {
      /* Slow path, fetch each sector a part and memcpy to destination buffer */

      for (i = 0; i < count; i++)
      {
        WriteStatus = 0;

        memcpy((void *)scratch, (void *)buff, BLOCKSIZE);
        buff += BLOCKSIZE;

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
        /* Clean the scratch buffer before DMA write (will flush any cached data to RAM) */
        SCB_CleanDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif
        ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
        if (ret == MSD_OK) {
          /* wait for a message from the queue or a timeout */
          timeout = HAL_GetTick();
          while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
          {
          }
          if (WriteStatus == 0)
          {
            break;
          }

        }
        else
        {
          break;
        }
      }
      if ((i == count) && (ret == MSD_OK))
        res = RES_OK;
    }
#endif
  return res;
}
#endif /* FF_FS_READONLY == 0 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{

  WriteStatus = 1;
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
  ReadStatus = 1;
}

/* USER CODE BEGIN ErrorAbortCallbacks */
/*
==============================================================================================
  depending on the SD_HAL_Driver version, either the HAL_SD_ErrorCallback() or HAL_SD_AbortCallback()
  or both could be defined, activate the callbacks below when suitable and needed
==============================================================================================
void BSP_SD_AbortCallback(void)
{
}

void BSP_SD_ErrorCallback(void)
{
}
*/
/* USER CODE END ErrorAbortCallbacks */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */
