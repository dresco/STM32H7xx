/*
  swo.c - redirect printf to SWO on debug builds

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe

  Some software serial code is ported from Arduino.  Credit belongs to the many
  authors that contributed to that project.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32h7xx_hal.h"

#ifdef DEBUG
int _write(int file, char *ptr, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
#endif
