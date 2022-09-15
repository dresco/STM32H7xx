/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"
#include "driver.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
    struct tm time;
    DWORD dt = ((2007UL-1980) << 25) | // Year = 2007
                (6UL << 21) |          // Month = June
                (5UL << 16) |          // Day = 5
                (11U << 11) |          // Hour = 11
                (38U << 5) |           // Min = 38
                (0U >> 1);             // Sec = 0

    if(hal.rtc.get_datetime && hal.rtc.get_datetime(&time))
        dt = ((time.tm_year - 80) << 25) |
             ((time.tm_mon + 1) << 21) |
              (time.tm_mday << 16) |
              (time.tm_hour << 11) |
              (time.tm_min << 5) |
              (time.tm_sec >> 1);

    return dt;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
