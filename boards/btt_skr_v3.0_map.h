/*
  btt_skr_v3.0_map.h - Board map for BIGTREETECH SKR 3 / SKR 3 EZ

  Part of grblHAL

  Copyright (c) 2022-2023 Jon Escombe

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

#if N_ABC_MOTORS > 2
#error "BTT SKR-3 supports 5 motors max."
#endif

#if !(defined(STM32H743xx)|| defined(STM32H723xx)) || HSE_VALUE != 25000000
#error "This board has STM32H7xx processor with a 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT SKR-3"
#define BOARD_URL "https://github.com/bigtreetech/SKR-3"

#define SERIAL_PORT                 1   // TFT header, GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT                32  // ESP-32,     GPIOD: TX = 8, RX = 9

#define COPROC_STREAM               1   // Use SERIAL1_PORT definition

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOD
#define X_STEP_PIN                  4                   // X
#define Y_STEP_PORT                 GPIOA
#define Y_STEP_PIN                  15                  // Y
#define Z_STEP_PORT                 GPIOE
#define Z_STEP_PIN                  2                   // Z
#define STEP_OUTMODE                GPIO_SINGLE
//#define STEP_PINMODE                PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOD
#define X_DIRECTION_PIN             3
#define Y_DIRECTION_PORT            GPIOA
#define Y_DIRECTION_PIN             8
#define Z_DIRECTION_PORT            GPIOE
#define Z_DIRECTION_PIN             3
#define DIRECTION_OUTMODE           GPIO_SINGLE
//#define DIRECTION_PINMODE           PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOD
#define X_ENABLE_PIN                6
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                1
#define Z_ENABLE_PORT               GPIOE
#define Z_ENABLE_PIN                0
//#define STEPPERS_ENABLE_PINMODE   PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOC
#define X_LIMIT_PIN                 1                           // X- Limit
#define Y_LIMIT_PORT                GPIOC
#define Y_LIMIT_PIN                 3                           // Y- Limit
#define Z_LIMIT_PORT                GPIOC
#define Z_LIMIT_PIN                 0                           // Z- Limit
#define LIMIT_INMODE                GPIO_SINGLE

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                // E0
#define M3_STEP_PORT                GPIOD
#define M3_STEP_PIN                 15
#define M3_DIRECTION_PORT           GPIOD
#define M3_DIRECTION_PIN            14
#define M3_LIMIT_PORT               GPIOC
#define M3_LIMIT_PIN                2
#define M3_ENABLE_PORT              GPIOC
#define M3_ENABLE_PIN               7
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE                // E1
#define M4_STEP_PORT                GPIOD
#define M4_STEP_PIN                 11
#define M4_DIRECTION_PORT           GPIOD
#define M4_DIRECTION_PIN            10
// The normal limit pin for E1 is PA0, but bit 0 already has an interrupt (Z_LIMIT_PIN).
// PC15 is normally used for PWRDET but is used for M4_LIMIT_PIN instead.
// If using TMC drivers, jumper from PWRDET connector pin 3 to DIAG pin on driver.
#define M4_LIMIT_PORT               GPIOC                       // orig GPIOA
#define M4_LIMIT_PIN                15                          // orig 0
#define M4_ENABLE_PORT              GPIOD
#define M4_ENABLE_PIN               13
#endif

#define AUXOUTPUT0_PORT             GPIOB                       // Spindle PWM - EXP1 - PB0, pin 9
#define AUXOUTPUT0_PIN              0
#define AUXOUTPUT1_PORT             GPIOB                       // Spindle direction - FAN2
#define AUXOUTPUT1_PIN              5
#define AUXOUTPUT2_PORT             GPIOB                       // Spindle enable - FAN1
#define AUXOUTPUT2_PIN              6
#define AUXOUTPUT3_PORT             GPIOB                       // Coolant flood - HEAT0
#define AUXOUTPUT3_PIN              3
#define AUXOUTPUT4_PORT             GPIOB                       // Coolant mist - HEAT1
#define AUXOUTPUT4_PIN              4
#define AUXOUTPUT5_PORT             GPIOB                       // ESP32 IO0
#define AUXOUTPUT5_PIN              10
#define AUXOUTPUT6_PORT             GPIOC                       // ESP32 RST
#define AUXOUTPUT6_PIN              14

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT4_PIN
#endif

#if ESP_AT_ENABLE
#define COPROC_RESET_PORT           AUXOUTPUT6_PORT
#define COPROC_RESET_PIN            AUXOUTPUT6_PIN
#define COPROC_BOOT0_PORT           AUXOUTPUT5_PORT
#define COPROC_BOOT0_PIN            AUXOUTPUT5_PIN
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
// These are all available on EXP2 along with electrical RESET* (EXP2, pin 3)
#define CONTROL_PORT                GPIOA
#define RESET_PIN                   4                           // EXP2 - PA4, pin 7
#define FEED_HOLD_PIN               5                           // EXP2 - PA5, pin 9
#define CYCLE_START_PIN             6                           // EXP2 - PA6, pin 10

#define AUXINPUT0_PORT              GPIOA
#define AUXINPUT0_PIN               7                           // EXP2 - PA7, pin 5

#define AUXINPUT1_PORT              GPIOC
#define AUXINPUT1_PIN               13                          // Z probe

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT0_PORT
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT1_PORT
#define PROBE_PIN                   AUXINPUT1_PIN
#endif

#define CONTROL_INMODE              GPIO_SINGLE

#if TRINAMIC_UART_ENABLE

#define MOTOR_UARTX_PORT            GPIOD
#define MOTOR_UARTX_PIN             5
#define MOTOR_UARTY_PORT            GPIOD
#define MOTOR_UARTY_PIN             0
#define MOTOR_UARTZ_PORT            GPIOE
#define MOTOR_UARTZ_PIN             1

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOC
#define MOTOR_UARTM3_PIN            6
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOD
#define MOTOR_UARTM4_PIN            12
#endif

#elif TRINAMIC_SPI_ENABLE

// The BTT SKR-3 uses software SPI
#define TRINAMIC_SOFT_SPI

#define TRINAMIC_MOSI_PORT          GPIOE
#define TRINAMIC_MOSI_PIN           13
#define TRINAMIC_SCK_PORT           GPIOE
#define TRINAMIC_SCK_PIN            14
#define TRINAMIC_MISO_PORT          GPIOE
#define TRINAMIC_MISO_PIN           15

#define MOTOR_CSX_PORT              GPIOD
#define MOTOR_CSX_PIN               5
#define MOTOR_CSY_PORT              GPIOD
#define MOTOR_CSY_PIN               0
#define MOTOR_CSZ_PORT              GPIOE
#define MOTOR_CSZ_PIN               1

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOC
#define MOTOR_CSM3_PIN              6
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOD
#define MOTOR_CSM4_PIN              12
#endif

#endif

// EOF
