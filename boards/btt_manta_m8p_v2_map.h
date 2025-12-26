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


// USB_OTG_ID 0 cm5 is host, 1 is device
// USB_OTG_ID 0 - hsd1(+-) is host, 1 - hsd2(+-) 
//USB_OTG_ID 0 - hsd1(+-),USBH_P/N is host to switch FE1.1s; 1 - hsd2(+-),  USBD_P/N to type-c device (if sw2 in upper position 1-2,4-5)
// switch FE1.1s ports:
//DM4/DP4 -ub_n/p USB_p/n pa11-12 
//DM3/DP3 -ub3_n/p USB3_p/n switch 2.54mm 4P  -> GND DP DM VUSB
//DM2/DP2 -ub_2_n/p USB2_p/n DIP USB A #2
//DM1/DP1 -ub1_n/p USB1_p/n DIP USB A #1

//GPIO14 Pi-TX 8
//GPIO15 Pi-RX 10
//GPIO12 UART4_TX
//GPIO13 UART4_RX

*/

#if N_ABC_MOTORS > 3
#error "BTT Manta M8P v2 board is configuted for 6 motors max."
#endif

#if !(defined(STM32H723xx)) || HSE_VALUE != 25000000
#error "This board has a STM32H723 processor with 25MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT Manta M8P V2"
#define BOARD_URL "https://github.com/bigtreetech/Manta-M8P"

#define SERIAL_PORT                 32      // GPIOD: TX = 8, RX = 9
#define SERIAL1_PORT                 1       // GPIOA: TX =  9, RX = 10


//#define I2C_PORT                    3       // GPIOx:

#define SPI_PORT                    3       // GPIOC: SCK = 10, MISO - 11, MOSI - 12 PA15 CS 

// Motor Reference:
// Motor-1:  DIR:PE5   STEP:PE6  ENABLE:PC14 LIMIT:PF4  *AXIS:X
// Motor-2:  DIR:PE1   STEP:PE2  ENABLE:PE4  LIMIT:PF3  *AXIS:Y
// Motor-3:  DIR:PB7   STEP:PB8  ENABLE:PE0  LIMIT:PF2  *AXIS:Z
// Motor-4:  DIR:PB3   STEP:PB4  ENABLE:PB6  LIMIT:PF1  *AXIS:M4
// Motor-5:  DIR:PG12  STEP:PG13 ENABLE:PG15 LIMIT:PF0  *AXIS:M5
// Motor-6:  DIR:PD7   STEP:PG9  ENABLE:PG11 LIMIT:PC15 *AXIS:M6
// Motor-7:  DIR:PD3   STEP:PD4  ENABLE:PD6  LIMIT:
// Motor-8:  DIR:PC8   STEP:PC7  ENABLE:PD2  LIMIT:




// Note: Motor-3 and Motor-4 limit switches clash with opto-isolated control inputs
//       Motor-7 and Motor-8 do not have associated limit switch inputs




// Define step direction output pins. Motor-1 .. Motor-3
#define X_DIRECTION_PORT            GPIOE
#define X_DIRECTION_PIN             5 // PE5 Motor-1
#define Y_DIRECTION_PORT            GPIOE
#define Y_DIRECTION_PIN             1 // PE1 Motor-2
#define Z_DIRECTION_PORT            GPIOB
#define Z_DIRECTION_PIN             7 // PB7 Motor-3
#define DIRECTION_OUTMODE           GPIO_SINGLE


// Define step pulse output pins. Motor-1 .. Motor-3
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  6  // PE6 Motor-1
#define Y_STEP_PORT                 GPIOE
#define Y_STEP_PIN                  2 // PE2 Motor-2
#define Z_STEP_PORT                 GPIOB
#define Z_STEP_PIN                  8 // PB8 Motor-3
#define STEP_OUTMODE                GPIO_SINGLE


// Define stepper driver enable/disable output pin. Motor-1 .. Motor-3
#define X_ENABLE_PORT               GPIOC
#define X_ENABLE_PIN                14 // PC14 Motor-1
#define Y_ENABLE_PORT               GPIOE
#define Y_ENABLE_PIN                4  // PE4 Motor-2
#define Z_ENABLE_PORT               GPIOE
#define Z_ENABLE_PIN                0 // PE0 Motor-3

// Define homing/hard limit switch input pins. M1-Stop, M2-Stop, M3-Stop, M4-Stop
#define X_LIMIT_PORT                GPIOF
#define X_LIMIT_PIN                 4 // PF4 M1-Stop
#define Y_LIMIT_PORT                GPIOF
#define Y_LIMIT_PIN                 3 // PF3 M2-Stop
#define Z_LIMIT_PORT                GPIOF
#define Z_LIMIT_PIN                 2 // PF2 M3-Stop
#define LIMIT_INMODE                GPIO_SINGLE


// Define M3 step, direction, enable, and limit pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                // Motor-4
#define M3_DIRECTION_PORT           GPIOB
#define M3_DIRECTION_PIN            3 // PB3 Motor-4
#define M3_STEP_PORT                GPIOB
#define M3_STEP_PIN                 4 // PB4 Motor-4
#define M3_ENABLE_PORT              GPIOB
#define M3_ENABLE_PIN               6 // PB6 Motor-4
#define M3_LIMIT_PORT               GPIOF
#define M3_LIMIT_PIN                1 // PF1 M4-Stop
#endif


// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                        // Motor-5
#define M4_DIRECTION_PORT           GPIOG
#define M4_DIRECTION_PIN            12 // PG12 Motor-5
#define M4_STEP_PORT                GPIOG
#define M4_STEP_PIN                 13 // PG13 Motor-5
#define M4_ENABLE_PORT              GPIOG
#define M4_ENABLE_PIN               15  // PG15 Motor-5
#define M4_LIMIT_PORT               GPIOF
#define M4_LIMIT_PIN                0 // PF0 M5-Stop
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 2                        
#define M5_AVAILABLE                        // Motor-6
#define M5_DIRECTION_PORT           GPIOD
#define M5_DIRECTION_PIN            7 // PD7 Motor-6
#define M5_STEP_PORT                GPIOG
#define M5_STEP_PIN                 9 // PG9 Motor-6
#define M5_ENABLE_PORT              GPIOG
#define M5_ENABLE_PIN               11       // PG11 Motor-6
#define M5_LIMIT_PORT               GPIOC
#define M5_LIMIT_PIN                15      // PC15 M6-Stop
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
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          GPIOA
#define COOLANT_FLOOD_PIN           6       // FAN5
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           GPIOA
#define COOLANT_MIST_PIN            2       // FAN6/cool
#endif

#define AUXINPUT0_PORT              GPIOD   // BLTOUCH sensor
#define AUXINPUT0_PIN               13
//#define AUXINPUT1_PORT              GPIOB   // Probe - Z probe
//#define AUXINPUT1_PIN               15
//#define AUXINPUT2_PORT              GPIOF   // Reset - PS-ON
//#define AUXINPUT2_PIN               13
//#define AUXINPUT3_PORT              GPIOF   // Feed hold - IND-PROBE
//#define AUXINPUT3_PIN               11
//#define AUXINPUT4_PORT              GPIOF   // Cycle start - FWS
//#define AUXINPUT4_PIN               10

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT                  GPIOC
#define RESET_PIN                   2       // FAN5 tacho
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT              GPIOC
#define FEED_HOLD_PIN               1       // FAN6 tacho
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT            GPIOD
#define CYCLE_START_PIN             8       // IND probe
#endif

#if SAFETY_DOOR_ENABLE
// #define SAFETY_DOOR_PORT            AUXINPUT1_PORT
// #define SAFETY_DOOR_PIN             AUXINPUT1_PIN
#endif

#if PROBE_ENABLE
 #define PROBE_PORT                  AUXINPUT0_PORT
 #define PROBE_PIN                   AUXINPUT0_PIN
#endif

#define CONTROL_INMODE              GPIO_SINGLE

#if TRINAMIC_UART_ENABLE

#define MOTOR_UARTX_PORT            GPIOC   
#define MOTOR_UARTX_PIN             13 // PC13 Motor-1
#define MOTOR_UARTY_PORT            GPIOE    
#define MOTOR_UARTY_PIN             3 // PE3 Motor-2
#define MOTOR_UARTZ_PORT            GPIOB    
#define MOTOR_UARTZ_PIN             9 // PB9 Motor-3

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOB    
#define MOTOR_UARTM3_PIN            5 // PB5 Motor-4
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOG    
#define MOTOR_UARTM4_PIN            14 // PG14 Motor-5
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT           GPIOG    
#define MOTOR_UARTM5_PIN            10 // PG10 Motor-6
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

#define MOTOR_CSX_PORT              GPIOC   
#define MOTOR_CSX_PIN               13 // PC13 Motor-1
#define MOTOR_CSY_PORT              GPIOE    
#define MOTOR_CSY_PIN               3 // PE3 Motor-2
#define MOTOR_CSZ_PORT              GPIOB    
#define MOTOR_CSZ_PIN               9 // PB9 Motor-3


#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOB    
#define MOTOR_CSM3_PIN              5 // PB5 Motor-4
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOG   
#define MOTOR_CSM4_PIN              14 // PG14 Motor-5
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT             GPIOG   
#define MOTOR_CSM5_PIN              10 // PG10 Motor-6
#endif


#endif

// EOF
