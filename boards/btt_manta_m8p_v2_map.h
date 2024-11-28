/*
  btt_manta_m8p_v2_map.h - Board map for BIGTREETECH MANTA M8P V2.0

  Part of grblHAL

  Copyright (c) 2024 Jon Escombe

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.
*/

#if N_ABC_MOTORS > 1
#error "BTT Manta M8P v2 board is configuted for 4 motors max."
#endif

#if !(defined(STM32H723xx)) || HSE_VALUE != 25000000
#error "This board has a STM32H723 processor with 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT Manta M8P V2"
#define BOARD_URL "https://github.com/bigtreetech/Manta-M8P"

#define SERIAL_PORT                 1       // GPIOA: TX =  9, RX = 10
#define SPI_PORT                    3       // GPIOC: SCK = 10, MISO - 11, MOSI - 12

// todo: I2C3 is routed to a board header, however the clock & data pins
//       are split across two ports - which is not currently supported..
//#define I2C_PORT                    3       // GPIOx:

// Motor Reference:
// Motor-1: STEP:PE6  DIR:PE5  ENABLE:PC14 LIMIT:PF4  *AXIS:X
// Motor-2: STEP:PE2  DIR:PE1  ENABLE:PE4  LIMIT:PF3  *AXIS:Y
// Motor-3: STEP:PB8  DIR:PB7  ENABLE:PE0  LIMIT:PF2
// Motor-4: STEP:PB4  DIR:PB3  ENABLE:PB6  LIMIT:PF1
// Motor-5: STEP:PG13 DIR:PG12 ENABLE:PG15 LIMIT:PF0  *AXIS:Z
// Motor-6: STEP:PG9  DIR:PD7  ENABLE:PG11 LIMIT:PC15 *AXIS:M3
// Motor-7: STEP:PD4  DIR:PD3  ENABLE:PD6  LIMIT:
// Motor-8: STEP:PC7  DIR:PC8  ENABLE:PD2  LIMIT:

// Note: Motor-3 and Motor-4 limit switches clash with opto-isolated control inputs
//       Motor-7 and Motor-8 do not have associated limit switch inputs

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  6
#define Y_STEP_PORT                 GPIOE
#define Y_STEP_PIN                  2
#define Z_STEP_PORT                 GPIOG
#define Z_STEP_PIN                  13
#define STEP_OUTMODE                GPIO_SINGLE

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOE
#define X_DIRECTION_PIN             5
#define Y_DIRECTION_PORT            GPIOE
#define Y_DIRECTION_PIN             1
#define Z_DIRECTION_PORT            GPIOG
#define Z_DIRECTION_PIN             12
#define DIRECTION_OUTMODE           GPIO_SINGLE

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOC
#define X_ENABLE_PIN                14
#define Y_ENABLE_PORT               GPIOE
#define Y_ENABLE_PIN                4
#define Z_ENABLE_PORT               GPIOG
#define Z_ENABLE_PIN                15

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOF
#define X_LIMIT_PIN                 4
#define Y_LIMIT_PORT                GPIOF
#define Y_LIMIT_PIN                 3
#define Z_LIMIT_PORT                GPIOF
#define Z_LIMIT_PIN                 0
#define LIMIT_INMODE                GPIO_SINGLE

// Define M3 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 0
#define M5_AVAILABLE                // Motor-6
#define M5_STEP_PORT                GPIOG
#define M5_STEP_PIN                 9
#define M5_DIRECTION_PORT           GPIOD
#define M5_DIRECTION_PIN            7
#define M5_LIMIT_PORT               GPIOC
#define M5_LIMIT_PIN                15
#define M5_ENABLE_PORT              GPIOG
#define M5_ENABLE_PIN               11
#endif

#define AUXOUTPUT0_PORT             GPIOF   // FAN0
#define AUXOUTPUT0_PIN              7
#define AUXOUTPUT1_PORT             GPIOF   // FAN1
#define AUXOUTPUT1_PIN              9
#define AUXOUTPUT2_PORT             GPIOF   // Spindle enable - FAN2
#define AUXOUTPUT2_PIN              6
#define AUXOUTPUT3_PORT             GPIOF   // Spindle direction - FAN3
#define AUXOUTPUT3_PIN              8
#define AUXOUTPUT4_PORT             GPIOE   // Spindle PWM - MOTOR
#define AUXOUTPUT4_PIN              9

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT         AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT2_PIN
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT            AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT3_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOA
#define COOLANT_FLOOD_PIN           6       // FAN5
#define COOLANT_MIST_PORT           GPIOA
#define COOLANT_MIST_PIN            2       // FAN6/cool

// Define user-control controls (cycle start, reset, feed hold) input pins.
// Only 3 optos on the board, ind-det, fan5-det, fan6/cool-det...
#define RESET_PORT                  GPIOC
#define RESET_PIN                   2       // FAN5 tacho
#define FEED_HOLD_PORT              GPIOC
#define FEED_HOLD_PIN               1       // FAN6 tacho
#define CYCLE_START_PORT            GPIOD
#define CYCLE_START_PIN             8       // IND probe

#define AUXINPUT0_PORT              GPIOD   // BLTOUCH sensor
#define AUXINPUT0_PIN               13
// #define AUXINPUT1_PORT              GPIOx
// #define AUXINPUT1_PIN               x

// CAUTION: No protection on this input, tied directly to MCU
#if PROBE_ENABLE
 #define PROBE_PORT                  AUXINPUT0_PORT
 #define PROBE_PIN                   AUXINPUT0_PIN
#endif

#if SAFETY_DOOR_ENABLE
// #define SAFETY_DOOR_PORT            AUXINPUT1_PORT
// #define SAFETY_DOOR_PIN             AUXINPUT1_PIN
#endif

#define CONTROL_INMODE              GPIO_SINGLE

#if TRINAMIC_UART_ENABLE

#define MOTOR_UARTX_PORT            GPIOC   // Motor-1
#define MOTOR_UARTX_PIN             13
#define MOTOR_UARTY_PORT            GPIOE   // Motor-2
#define MOTOR_UARTY_PIN             3
#define MOTOR_UARTZ_PORT            GPIOG   // Motor-5
#define MOTOR_UARTZ_PIN             14

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOG   // Motor-6
#define MOTOR_UARTM3_PIN            10
#endif

#elif TRINAMIC_SPI_ENABLE

// The Manta M8P uses software SPI
#define TRINAMIC_SOFT_SPI

#define TRINAMIC_MOSI_PORT          GPIOG
#define TRINAMIC_MOSI_PIN           6
#define TRINAMIC_SCK_PORT           GPIOG
#define TRINAMIC_SCK_PIN            8
#define TRINAMIC_MISO_PORT          GPIOG
#define TRINAMIC_MISO_PIN           7

#define MOTOR_CSX_PORT              GPIOC   // Motor-1
#define MOTOR_CSX_PIN               13
#define MOTOR_CSY_PORT              GPIOE   // Motor-2
#define MOTOR_CSY_PIN               3
#define MOTOR_CSZ_PORT              GPIOG   // Motor-5
#define MOTOR_CSZ_PIN               14

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOG   // Motor-6
#define MOTOR_CSM3_PIN              10
#endif

#endif

// EOF
