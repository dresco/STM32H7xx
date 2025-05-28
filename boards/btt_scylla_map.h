/*
  btt_scylla_map.h - Board map for BIGTREETECH Scylla

  Part of grblHAL

  Copyright (c) 2024 Joe Corelli
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

#if N_ABC_MOTORS > 1
#error "Scylla board map is only configured for 4 motors max."
#endif

#if !(defined(STM32H723xx)) || HSE_VALUE != 25000000
#error "This board has a STM32H723 processor with 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT Scylla"
#define BOARD_URL "https://github.com/bigtreetech/"

#define SERIAL_PORT                 2   // ESP-32,   GPIOA: TX = 2, RX = 3
#define SERIAL1_PORT                1   // RS-485,   GPIOA: TX = 9, RX = 10
#define I2C_PORT                    2   // I2C conn, GPIOB: SCL = 10, SDA = 11
#define SPI_PORT                    2   // Pi conn,  GPIOB: SCK = 13, MISO = 14, MOSI = 15

#define TRINAMIC_SOFT_SPI
#define MODBUS_RTU_STREAM           1   // Use SERIAL1_PORT definition
#define COPROC_STREAM               0   // Use SERIAL_PORT definition

// Motor Reference:
// Motor-1: X-axis
// Motor-2: Y-axis
// Motor-3: Z-axis
// Motor-4: M3-axis

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOA // X-OUT STEP
#define X_STEP_PIN                  0
#define Y_STEP_PORT                 GPIOC // Y-OUT STEP
#define Y_STEP_PIN                  13
#define Z_STEP_PORT                 GPIOB // Z-OUT STEP
#define Z_STEP_PIN                  8
#define STEP_OUTMODE                GPIO_SINGLE

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOA // X-OUT DIR
#define X_DIRECTION_PIN             1
#define Y_DIRECTION_PORT            GPIOE // Y-OUT DIR
#define Y_DIRECTION_PIN             6
#define Z_DIRECTION_PORT            GPIOB // Z-OUT DIR
#define Z_DIRECTION_PIN             9
#define DIRECTION_OUTMODE           GPIO_SINGLE

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOC // X-OUT EN
#define X_ENABLE_PIN                0
#define Y_ENABLE_PORT               GPIOC // Y-OUT EN
#define Y_ENABLE_PIN                2
#define Z_ENABLE_PORT               GPIOE // Z-OUT EN
#define Z_ENABLE_PIN                5

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOD
#define X_LIMIT_PIN                 11      // X-MIN
#define Y_LIMIT_PORT                GPIOA
#define Y_LIMIT_PIN                 8       // Y-MIN
#define Z_LIMIT_PORT                GPIOC
#define Z_LIMIT_PIN                 7       // Z-MIN
#define LIMIT_INMODE                GPIO_SINGLE

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                        // Motor-4
#define M3_STEP_PORT                GPIOD   // A-OUT Step
#define M3_STEP_PIN                 3
#define M3_DIRECTION_PORT           GPIOD   // A-OUT Dir
#define M3_DIRECTION_PIN            4
#define M3_LIMIT_PORT               GPIOD
#define M3_LIMIT_PIN                15      // A-MIN
#define M3_ENABLE_PORT              GPIOE
#define M3_ENABLE_PIN               1       // A-OUT En
#endif

#define AUXOUTPUT0_PORT             GPIOA   // AUX0
#define AUXOUTPUT0_PIN              4
#define AUXOUTPUT1_PORT             GPIOA   // AUX1
#define AUXOUTPUT1_PIN              5
#define AUXOUTPUT2_PORT             GPIOA   // AUX2
#define AUXOUTPUT2_PIN              6

#define AUXOUTPUT3_PORT             GPIOC   // SPIN EN
#define AUXOUTPUT3_PIN              5
#define AUXOUTPUT4_PORT             GPIOB   // SPIN SPD
#define AUXOUTPUT4_PIN              1
#define AUXOUTPUT5_PORT             GPIOB   // SPIN DIR
#define AUXOUTPUT5_PIN              0

#define AUXOUTPUT6_PORT             GPIOC   // Cool
#define AUXOUTPUT6_PIN              4
#define AUXOUTPUT7_PORT             GPIOA   // Mist
#define AUXOUTPUT7_PIN              7

#define AUXOUTPUT8_PORT             GPIOE   // ESP32 IO0
#define AUXOUTPUT8_PIN              9
#define AUXOUTPUT9_PORT             GPIOE   // ESP32 RST
#define AUXOUTPUT9_PIN              10

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT3_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT5_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT5_PIN
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

#if ESP_AT_ENABLE
#define COPROC_RESET_PORT           AUXOUTPUT9_PORT
#define COPROC_RESET_PIN            AUXOUTPUT9_PIN
#define COPROC_BOOT0_PORT           AUXOUTPUT8_PORT
#define COPROC_BOOT0_PIN            AUXOUTPUT8_PIN
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT                  GPIOC
#define RESET_PIN                   6       // X-MAX
#define FEED_HOLD_PORT              GPIOD
#define FEED_HOLD_PIN               14      // Y-MAX
#define CYCLE_START_PORT            GPIOD
#define CYCLE_START_PIN             12      // Z-MAX

#define AUXINPUT0_PORT              GPIOE
#define AUXINPUT0_PIN               7      // Tool

#define AUXINPUT1_PORT              GPIOE
#define AUXINPUT1_PIN               15      // Probe

#define AUXINPUT2_PORT              GPIOB
#define AUXINPUT2_PIN               7      // IO-IN

#define AUXINPUT3_PORT              GPIOD
#define AUXINPUT3_PIN               13      // A-MAX

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT3_PORT
#define SAFETY_DOOR_PIN             AUXINPUT3_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT1_PORT
#define PROBE_PIN                   AUXINPUT1_PIN
#endif

#define CONTROL_INMODE              GPIO_SINGLE

#if TRINAMIC_SPI_ENABLE

#define TRINAMIC_R_SENSE            50

#ifdef TRINAMIC_SOFT_SPI // Software SPI implementation

#define TRINAMIC_MOSI_PORT          GPIOB
#define TRINAMIC_MOSI_PIN           5
#define TRINAMIC_MISO_PORT          GPIOB
#define TRINAMIC_MISO_PIN           4
#define TRINAMIC_SCK_PORT           GPIOB
#define TRINAMIC_SCK_PIN            3

#else // Hardware SPI implementation

#define SPI_PORT                    12      // GPIOB: SCK = 3, MISO = 4 MOSI = 5

#endif // TRINAMIC_SOFT_SPI

#define MOTOR_CSX_PORT              GPIOC // X-CS
#define MOTOR_CSX_PIN               15
#define MOTOR_CSY_PORT              GPIOC // Y-CS
#define MOTOR_CSY_PIN               14
#define MOTOR_CSZ_PORT              GPIOE // Z-CS
#define MOTOR_CSZ_PIN               2

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOE // A-CS
#define MOTOR_CSM3_PIN              4
#endif

#endif // TRINAMIC_SPI_ENABLE

#define CAN_PORT                    GPIOD
#define CAN_RX_PIN                  0 // CAN-L
#define CAN_TX_PIN                  1 // CAN-H

// EOF
