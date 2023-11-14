/*
  protoneer_3.xx_map.h - driver code for STM32H7xx ARM processor on a Nucleo-H743ZI board

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

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

#if ETHERNET_ENABLE
#error "Not compatible with ethernet!"
#endif

#if N_AUTO_SQUARED || N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#define BOARD_NAME "Protoneer v3"
#define BOARD_URL "https://blog.protoneer.co.nz/arduino-cnc-shield/"

#define SERIAL_PORT             32   // GPIOD: TX = 8, RX = 9

// Define step pulse output pins.
#define X_STEP_PORT             GPIOF // D2
#define X_STEP_PIN              15
#define Y_STEP_PORT             GPIOE // D3
#define Y_STEP_PIN              13
#define Z_STEP_PORT             GPIOF // D4
#define Z_STEP_PIN              14
#define STEP_OUTMODE            GPIO_SINGLE

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOE // D5
#define X_DIRECTION_PIN         11
#define Y_DIRECTION_PORT        GPIOE // D6
#define Y_DIRECTION_PIN         9
#define Z_DIRECTION_PORT        GPIOF // D7
#define Z_DIRECTION_PIN         13
#define DIRECTION_OUTMODE       GPIO_SINGLE

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOF // D8
#define STEPPERS_ENABLE_PIN     12

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOD // D9
#define X_LIMIT_PIN             15
#define Y_LIMIT_PORT            GPIOD // D10
#define Y_LIMIT_PIN             14
#define Z_LIMIT_PORT            GPIOA // D11 - NOTE: remove JP6 on Nucleo-144 board
#define Z_LIMIT_PIN             7
#define LIMIT_INMODE            GPIO_SINGLE

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA // D12
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA // D13
#define M3_DIRECTION_PIN        5
#endif

// Define spindle enable and spindle direction output pins.
#if N_ABC_MOTORS == 0
#define SPINDLE_ENABLE_PORT     GPIOA // D12
#define SPINDLE_ENABLE_PIN      6
#define SPINDLE_DIRECTION_PORT  GPIOA // D13
#define SPINDLE_DIRECTION_PIN   5
#else
// use A4 & A5?
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOF // A3
#define COOLANT_FLOOD_PIN       3

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOA // A0
#define RESET_PIN               3
#define FEED_HOLD_PORT          GPIOC
#define FEED_HOLD_PIN           0 // A1
#define CYCLE_START_PORT        GPIOC
#define CYCLE_START_PIN         1
#define CONTROL_INMODE          GPIO_SINGLE

// I2C -> A4 & A5 (PF5 & PF10) cannot be used

/**/
