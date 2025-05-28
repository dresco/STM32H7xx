/*
  dresco_octave_map.h - Board map for 8-axis dresco Octave controller (RevB prototype)

  Part of grblHAL

  Copyright (c) 2024 Jon Escombe

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

#if N_ABC_MOTORS > 5
#error "H7 Octave board supports 8 motors max."
#endif

#if !(defined(STM32H743xx)|| defined(STM32H723xx)) || HSE_VALUE != 25000000
#error "This board has a STM32H7xx processor with 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "Octave CNC Controller"
#define BOARD_URL "https://github.com/dresco/STM32H7xx_CNC_Controller"

#define SERIAL_PORT                 12  // GPIOB: TX = 14, RX = 15
#define SERIAL1_PORT                32  // GPIOD: TX =  8, RX =  9
#define I2C_PORT                    2   // GPIOB: SCL = 10, SDA = 11

#define ETH_PINOUT                  2   // PA1, PA2, PA7, PC1, PC4, PC5, PG11, PG12, PG13

#define SPIFLASH_PINOUT             2   // PB6, PF10, PF8, PF9, PF7, PF6
#define SPIFLASH_SIZE               128 // 128 Mb = 16 MB

#define MODBUS_RTU_STREAM           1   // Use SERIAL1_PORT definition

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  0
#define Y_STEP_PORT                 GPIOE
#define Y_STEP_PIN                  2
#define Z_STEP_PORT                 GPIOE
#define Z_STEP_PIN                  5
#define STEP_OUTMODE                GPIO_SINGLE

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOB
#define X_DIRECTION_PIN             9
#define Y_DIRECTION_PORT            GPIOE
#define Y_DIRECTION_PIN             3
#define Z_DIRECTION_PORT            GPIOE
#define Z_DIRECTION_PIN             6
#define DIRECTION_OUTMODE           GPIO_SINGLE

// Define stepper driver enable/disable output pins.
#define X_ENABLE_PORT               GPIOB
#define X_ENABLE_PIN                8
#define Y_ENABLE_PORT               GPIOE
#define Y_ENABLE_PIN                4
#define Z_ENABLE_PORT               GPIOC
#define Z_ENABLE_PIN                13

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOG
#define X_LIMIT_PIN                 3
#define Y_LIMIT_PORT                GPIOG
#define Y_LIMIT_PIN                 4
#define Z_LIMIT_PORT                GPIOG
#define Z_LIMIT_PIN                 5
#define LIMIT_INMODE                GPIO_SINGLE

// Define M3 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT                GPIOF
#define M3_STEP_PIN                 11
#define M3_DIRECTION_PORT           GPIOB
#define M3_DIRECTION_PIN            2
#define M3_ENABLE_PORT              GPIOB
#define M3_ENABLE_PIN               1
#define M3_LIMIT_PORT               GPIOG
#define M3_LIMIT_PIN                6
#endif

// Define M4 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PORT                GPIOG
#define M4_STEP_PIN                 0
#define M4_DIRECTION_PORT           GPIOF
#define M4_DIRECTION_PIN            15
#define M4_ENABLE_PORT              GPIOF
#define M4_ENABLE_PIN               14
#define M4_LIMIT_PORT               GPIOG
#define M4_LIMIT_PIN                7
#endif

// Define M5 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 2
#define M5_AVAILABLE
#define M5_STEP_PORT                GPIOE
#define M5_STEP_PIN                 8
#define M5_DIRECTION_PORT           GPIOE
#define M5_DIRECTION_PIN            7
#define M5_ENABLE_PORT              GPIOG
#define M5_ENABLE_PIN               1
#define M5_LIMIT_PORT               GPIOA
#define M5_LIMIT_PIN                8
#endif

// Define M6 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 3
#define M6_AVAILABLE
#define M6_STEP_PORT                GPIOE
#define M6_STEP_PIN                 11
#define M6_DIRECTION_PORT           GPIOE
#define M6_DIRECTION_PIN            10
#define M6_ENABLE_PORT              GPIOE
#define M6_ENABLE_PIN               9
#define M6_LIMIT_PORT               GPIOA
#define M6_LIMIT_PIN                9
#endif

// Define M7 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 4
#define M7_AVAILABLE
#define M7_STEP_PORT                GPIOE
#define M7_STEP_PIN                 15
#define M7_DIRECTION_PORT           GPIOE
#define M7_DIRECTION_PIN            12
#define M7_ENABLE_PORT              GPIOE
#define M7_ENABLE_PIN               14
#define M7_LIMIT_PORT               GPIOA
#define M7_LIMIT_PIN                10
#endif

// Define auxiliary output pins (incl spindle PWM).
#define AUXOUTPUT0_PORT             GPIOF
#define AUXOUTPUT0_PIN              5
#define AUXOUTPUT1_PORT             GPIOC
#define AUXOUTPUT1_PIN              0
#define AUXOUTPUT2_PORT             GPIOA
#define AUXOUTPUT2_PIN              0
#define AUXOUTPUT3_PORT             GPIOB       // Spindle PWM
#define AUXOUTPUT3_PIN              0
#define AUXOUTPUT4_PORT             GPIOF       // Spindle direction
#define AUXOUTPUT4_PIN              2
#define AUXOUTPUT5_PORT             GPIOF       // Spindle enable
#define AUXOUTPUT5_PIN              1
#define AUXOUTPUT6_PORT             GPIOF       // Coolant flood
#define AUXOUTPUT6_PIN              4
#define AUXOUTPUT7_PORT             GPIOF       // Coolant mist
#define AUXOUTPUT7_PIN              3

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT5_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT4_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT4_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT7_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT7_PIN
#endif

// Define auxiliary input pins.
#define AUXINPUT0_PORT              GPIOD       // Safety door
#define AUXINPUT0_PIN               11
#define AUXINPUT1_PORT              GPIOB
#define AUXINPUT1_PIN               12
#define AUXINPUT2_PORT              GPIOB
#define AUXINPUT2_PIN               13
#define AUXINPUT3_PORT              GPIOD
#define AUXINPUT3_PIN               10
#define AUXINPUT4_PORT              GPIOD       // Probe
#define AUXINPUT4_PIN               12
#define AUXINPUT5_PORT              GPIOF       // Motor fault
#define AUXINPUT5_PIN               0
#define AUXINPUT6_PORT              GPIOD       // Reset
#define AUXINPUT6_PIN               13
#define AUXINPUT7_PORT              GPIOG       // Feed hold
#define AUXINPUT7_PIN               2
#define AUXINPUT8_PORT              GPIOE       // Cycle start
#define AUXINPUT8_PIN               1

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT                  AUXINPUT6_PORT
#define RESET_PIN                   AUXINPUT6_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT              AUXINPUT7_PORT
#define FEED_HOLD_PIN               AUXINPUT7_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT            AUXINPUT8_PORT
#define CYCLE_START_PIN             AUXINPUT8_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT0_PORT
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT4_PORT
#define PROBE_PIN                   AUXINPUT4_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT            AUXINPUT5_PORT
#define MOTOR_FAULT_PIN             AUXINPUT5_PIN
#endif

// Spindle encoder pins (index pin that must be interrupt capable).
#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_INDEX_PORT          GPIOG
#define SPINDLE_INDEX_PIN           14
#define SPINDLE_PULSE_PORT          GPIOB
#define SPINDLE_PULSE_PIN           4
#define SPINDLE_ENCODER_CLK         1
#endif

#define CAN_PORT                    GPIOD
#define CAN_RX_PIN                  0
#define CAN_TX_PIN                  1

// EOF
