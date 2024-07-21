/*

  debug.c - debug output for STM32H7xx ARM processors

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

#if DEBUGOUT == -1

// Default to Serial Wire Viewer (SWV)
#ifndef DEBUGOUT_BACKEND
#define DEBUGOUT_BACKEND 1
#endif

#if DEBUGOUT_BACKEND == 1 // SWV

/*
 * Using SWV on SWO/PB3 pin for debug output.
 *
 * Expects ITM trace to be enabled by the debugger.
 *
 */
#include "stm32h7xx_hal.h"

// Override default printf() output
int _write(int file, char *ptr, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// debug_write() is called with a null terminated string
void debug_write (const char *s)
{
    char c;

    while((c = *s++))
        ITM_SendChar(c);
}

#elif DEBUGOUT_BACKEND == 2 // RTT

/*
 * Using Segger RTT for debug output.
 *
 * Expects the following Segger files to be added to the project sources;
 * SEGGER_RTT.h
 * SEGGER_RTT_Conf.h
 * SEGGER_RTT.c
 *
 */
#include "SEGGER_RTT.h"

// Override default printf() output
int _write(int file, char *ptr, int len)
{
      SEGGER_RTT_Write(0, ptr, len);
      return len;
}

// debug_write() is called with a null terminated string
void debug_write (const char *s)
{
    SEGGER_RTT_WriteString(0, s);
}

#endif // DEBUGOUT_BACKEND

#endif // DEBUGOUT
