/*
  reference_map.h - driver code for STM32H7xx ARM processor on Nucleo-H743ZI or Nucleo H723ZG board

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

//#error "Reference map is work in progress - pin assignments are subject to changes, do not use!"

#define BOARD_NAME "grblHAL reference map"


#define SERIAL_PORT            32   // GPIOD: TX = 8, RX = 9
#if USB_SERIAL_CDC
#define SERIAL1_PORT           11   // GPIOB: TX = 6, RX = 7
#define COPROC_STREAM           1
#else
#define SERIAL1_PORT            1   // GPIOA: TX = 9, RX = 10
#endif
#define I2C_PORT                1   // GPIOB: SCL = 8, SDA = 9
#ifdef SDCARD_SDIO
#define SPI_PORT                2   // GOPIB: SCK = 13, MISO - 14, MOSI - 15
#else
#define SPI_PORT                3   // GPIOC: SCK = 10, MISO - 11, MOSI - 12
#endif
#define IS_NUCLEO_BOB

#define ETH_PINOUT      1   // PA1, PA2, PA7, PB13, PC1, PC4, PC5, PG11, PG13

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOF
#define STEPPERS_ENABLE_PIN     12

// Define step pulse output pins.
#define X_STEP_PORT             GPIOE
#define X_STEP_PIN              10
#define Y_STEP_PORT             GPIOE
#define Y_STEP_PIN              12
#define Z_STEP_PORT             GPIOE
#define Z_STEP_PIN              14
#define STEP_OUTMODE            GPIO_SINGLE
#define STEP_MASK 0

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOE
#define X_DIRECTION_PIN         11
#define Y_DIRECTION_PORT        GPIOE
#define Y_DIRECTION_PIN         9
#define Z_DIRECTION_PORT        GPIOE
#define Z_DIRECTION_PIN         13
#define DIRECTION_OUTMODE       GPIO_SINGLE

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOD
#define X_LIMIT_PIN             15
#define Y_LIMIT_PORT            GPIOD
#define Y_LIMIT_PIN             14
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #define Z_LIMIT_PORT          GPIOA
  #define Z_LIMIT_PIN           6
#else
  #define Z_LIMIT_PORT          GPIOA
  #define Z_LIMIT_PIN           7
#endif
#define LIMIT_INMODE            GPIO_SINGLE

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
#define M5_LIMIT_PIN            8
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
#define M6_LIMIT_PIN            11
#ifndef STEPPERS_ENABLE_PORT
#define M6_ENABLE_PORT          GPIOG
#define M6_ENABLE_PIN           2
#endif
#endif

#if N_ABC_MOTORS == 5
#define M7_AVAILABLE
#define M7_STEP_PORT            GPIOA
#define M7_STEP_PIN             7
#define M7_DIRECTION_PORT       GPIOF
#define M7_DIRECTION_PIN        2
#define M7_LIMIT_PORT           GPIOF
#define M7_LIMIT_PIN            9
#ifndef STEPPERS_ENABLE_PORT
#define M7_ENABLE_PORT          GPIOF
#define M7_ENABLE_PIN           0
#endif
#endif

// Define driver spindle pins

#define AUXOUTPUT0_PORT         GPIOB // Spindle1 PWM
#define AUXOUTPUT0_PIN          11
#define AUXOUTPUT1_PORT         GPIOE // Spindle1 enable
#define AUXOUTPUT1_PIN          2
#define AUXOUTPUT2_PORT         GPIOF // Spindle1 direction
#define AUXOUTPUT2_PIN          0
#define AUXOUTPUT3_PORT         GPIOA // Spindle direction
#define AUXOUTPUT3_PIN          5
#define AUXOUTPUT4_PORT         GPIOB // Spindle PWM
#define AUXOUTPUT4_PIN          0
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define AUXOUTPUT5_PORT         GPIOA // Spindle enable
#define AUXOUTPUT5_PIN          15
#else
#define AUXOUTPUT5_PORT         GPIOA // Spindle enable
#define AUXOUTPUT5_PIN          6
#endif
#define AUXOUTPUT6_PORT         GPIOE // Coolant flood
#define AUXOUTPUT6_PIN          0
#define AUXOUTPUT7_PORT         GPIOF // Coolant mist
#define AUXOUTPUT7_PIN          5

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT5_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
#define SPINDLE1_ENABLE_PORT    AUXOUTPUT1_PORT
#define SPINDLE1_ENABLE_PIN     AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
#define SPINDLE1_PWM_PORT       AUXOUTPUT0_PORT
#define SPINDLE1_PWM_PIN        AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
#define SPINDLE1_DIRECTION_PORT AUXOUTPUT2_PORT
#define SPINDLE1_DIRECTION_PIN  AUXOUTPUT2_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT7_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT7_PIN
#endif

#if ESP_AT_ENABLE && !(DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA)
#define COPROC_RESET_PORT       AUXOUTPUT1_PORT
#define COPROC_RESET_PIN        AUXOUTPUT1_PIN
#define COPROC_BOOT0_PORT       AUXOUTPUT2_PORT
#define COPROC_BOOT0_PIN        AUXOUTPUT2_PIN
#endif

/*
#define AUXINPUT0_ANALOG_PORT   GPIOF
#define AUXINPUT0_ANALOG_PIN    7
#define AUXINPUT1_ANALOG_PORT   GPIOF
#define AUXINPUT1_ANALOG_PIN    8
*/

#define AUXINPUT0_PORT          GPIOE
#define AUXINPUT0_PIN           15
#define AUXINPUT1_PORT          GPIOD // Safety door
#define AUXINPUT1_PIN           1
#define AUXINPUT2_PORT          GPIOF
#define AUXINPUT2_PIN           2
#if N_ABC_MOTORS == 0
#define AUXINPUT3_PORT          GPIOB
#define AUXINPUT3_PIN           10
#endif
#define AUXINPUT4_PORT          GPIOF // Probe
#define AUXINPUT4_PIN           10
#define AUXINPUT5_PORT          GPIOA // Reset/EStop
#define AUXINPUT5_PIN           3
#define AUXINPUT6_PORT          GPIOC // Feed hold
#define AUXINPUT6_PIN           0
#define AUXINPUT7_PORT          GPIOF // Cycle start
#define AUXINPUT7_PIN           4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT5_PORT
#define RESET_PIN               AUXINPUT5_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT6_PORT
#define FEED_HOLD_PIN           AUXINPUT6_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT7_PORT
#define CYCLE_START_PIN         AUXINPUT7_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT1_PORT
#define SAFETY_DOOR_PIN         AUXINPUT1_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT4_PORT
#define PROBE_PIN               AUXINPUT4_PIN
#endif

#if SAFETY_DOOR_ENABLE
#ifndef AUXINPUT3_PIN
#error "Safety door cannot be enabled when > 3 motors are defined."
#endif
#define SAFETY_DOOR_PORT        AUXINPUT3_PORT
#define SAFETY_DOOR_PIN         AUXINPUT3_PIN
#endif

#ifndef M4_AVAILABLE
#define AUXOUTPUT0_PWM_PORT     GPIOE
#define AUXOUTPUT0_PWM_PIN      5
#define AUXOUTPUT1_PWM_PORT     GPIOE
#define AUXOUTPUT1_PWM_PIN      6
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT1_PORT
#define I2C_STROBE_PIN          AUXINPUT1_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT1_PORT
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

#if ETHERNET_ENABLE && defined(_WIZCHIP_)
#undef SPI_ENABLE
#define SPI_ENABLE              1
#define SPI_CS_PORT             GPIOA // CS_JOG_SW
#define SPI_CS_PIN              4
#define SPI_IRQ_PORT            GPIOB // RXD_INT
#define SPI_IRQ_PIN             12
#define SPI_RST_PORT            GPIOB // TXD_INT
#define SPI_RST_PIN             3
#endif

#if MPG_ENABLE == 1
#define MPG_MODE_PORT           AUXINPUT2_PORT
#define MPG_MODE_PIN            AUXINPUT2_PIN
#elif MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_PORT      AUXINPUT2_PORT
#define MOTOR_WARNING_PIN       AUXINPUT2_PIN
#endif

#if 1
#define NEOPIXEL_SPI            1 // GPIOA: 7
#endif

#if 0
#define CAN_PORT                GPIOD
#define CAN_RX_PIN              0
#define CAN_TX_PIN              1
#endif

/**/
