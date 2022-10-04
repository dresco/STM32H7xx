/*
  reference_map.h - driver code for STM32H7xx ARM processor on a Nucleo-H743ZI board

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io

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


#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

//#error "Reference map is work in progress - pin assignments are subject to changes, do not use!"

#define BOARD_NAME "grblHAL reference map"
#define HAS_IOPORTS
#define I2C_PORT 1
#define SPI_PORT 3 // SCK - C10, MISO - C11, MOSI - C12
#define IS_NUCLEO_BOB
#define VARIABLE_SPINDLE // Comment out to disable variable spindle

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT   GPIOF
#define STEPPERS_ENABLE_PIN    12

// Define step pulse output pins.
#define X_STEP_PORT         GPIOE
#define X_STEP_PIN          10
#define Y_STEP_PORT         GPIOE
#define Y_STEP_PIN          12
#define Z_STEP_PORT         GPIOE
#define Z_STEP_PIN          14
#define STEP_OUTMODE        GPIO_SINGLE
#define STEP_MASK 0

// Define step direction output pins.
#define X_DIRECTION_PORT    GPIOE
#define X_DIRECTION_PIN     11
#define Y_DIRECTION_PORT    GPIOE
#define Y_DIRECTION_PIN     9
#define Z_DIRECTION_PORT    GPIOE
#define Z_DIRECTION_PIN     13
#define DIRECTION_OUTMODE   GPIO_SINGLE

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        GPIOD
#define X_LIMIT_PIN         15
#define Y_LIMIT_PORT        GPIOD
#define Y_LIMIT_PIN         14
#ifdef VARIABLE_SPINDLE
  #define Z_LIMIT_PORT      GPIOA
  #define Z_LIMIT_PIN       6
#else
  #define Z_LIMIT_PORT      GPIOA
  #define Z_LIMIT_PIN       7
#endif
#define LIMIT_INMODE        GPIO_SINGLE

#if N_ABC_MOTORS
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOE
#define M3_STEP_PIN             0
#define M3_DIRECTION_PORT       GPIOF
#define M3_DIRECTION_PIN        14
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            10
#ifndef STEPPERS_ENABLE_PORT
#define M3_ENABLE_PORT          GPIOE
#define M3_ENABLE_PIN           14
#endif
#endif

#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOE
#define M4_STEP_PIN             2
#define M4_DIRECTION_PORT       GPIOE
#define M4_DIRECTION_PIN        4
#define M4_LIMIT_PORT           GPIOE
#define M4_LIMIT_PIN            5
#ifndef STEPPERS_ENABLE_PORT
#define M4_ENABLE_PORT          GPIOE
#define M4_ENABLE_PIN           6
#endif
#endif

#if N_ABC_MOTORS > 2
#define M5_AVAILABLE
#define M5_STEP_PORT            GPIOE
#define M5_STEP_PIN             3
#define M5_DIRECTION_PORT       GPIOE
#define M5_DIRECTION_PIN        8
#define M5_LIMIT_PORT           GPIOF
#define M5_LIMIT_PIN            7
#ifndef STEPPERS_ENABLE_PORT
#define M5_ENABLE_PORT          GPIOF
#define M5_ENABLE_PIN           9
#endif
#endif

#if N_ABC_MOTORS > 3
#define M6_AVAILABLE
#define M6_STEP_PORT            GPIOG
#define M6_STEP_PIN             1
#define M6_DIRECTION_PORT       GPIOD
#define M6_DIRECTION_PIN        0
#define M6_LIMIT_PORT           GPIOD
#define M6_LIMIT_PIN            1
#ifndef STEPPERS_ENABLE_PORT
#define M6_ENABLE_PORT          GPIOG
#define M6_ENABLE_PIN           0
#endif
#endif

#if N_ABC_MOTORS == 5
#define M7_AVAILABLE
#define M7_STEP_PORT            GPIOA
#define M7_STEP_PIN             7
#define M7_DIRECTION_PORT       GPIOF
#define M7_DIRECTION_PIN        2
#define M7_LIMIT_PORT           GPIOF
#define M7_LIMIT_PIN            1
#ifndef STEPPERS_ENABLE_PORT
#define M7_ENABLE_PORT          GPIOF
#define M7_ENABLE_PIN           0
#endif
#endif

// Define spindle enable and spindle direction output pins.
#ifdef VARIABLE_SPINDLE
  #define SPINDLE_ENABLE_PORT   GPIOA // on morpho header
  #define SPINDLE_ENABLE_PIN    15
#else
  #define SPINDLE_ENABLE_PORT   GPIOA
  #define SPINDLE_ENABLE_PIN    6
#endif
#define SPINDLE_DIRECTION_PORT  GPIOA
#define SPINDLE_DIRECTION_PIN   5

// Define spindle PWM output pin.
#ifdef VARIABLE_SPINDLE
#define SPINDLE_PWM_PORT_BASE   GPIOB_BASE
#define SPINDLE_PWM_PIN         0
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOE
#define COOLANT_FLOOD_PIN       0
#define COOLANT_MIST_PORT       GPIOF
#define COOLANT_MIST_PIN        5

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOA
#define RESET_PIN               3
#define FEED_HOLD_PORT          GPIOC
#define FEED_HOLD_PIN           0
#define CYCLE_START_PORT        GPIOF
#define CYCLE_START_PIN         4
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        GPIOB
#define SAFETY_DOOR_PIN         10
#endif

#define CONTROL_INMODE          GPIO_SINGLE

// Define probe switch input pin.
#define PROBE_PORT              GPIOF
#define PROBE_PIN               10

//#if SDCARD_ENABLE
//#define SD_CS_PORT              GPIOC
//#define SD_CS_PIN               8
//#endif

#define AUXINPUT0_PORT          GPIOE
#define AUXINPUT0_PIN           15
#define AUXINPUT1_PORT          GPIOD
#define AUXINPUT1_PIN           1
#define AUXINPUT2_PORT          GPIOF
#define AUXINPUT2_PIN           2

#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          11
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          10
#define AUXOUTPUT2_PORT         GPIOE
#define AUXOUTPUT2_PIN          2
#define AUXOUTPUT3_PORT         GPIOF
#define AUXOUTPUT3_PIN          0

/**/
