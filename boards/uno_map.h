/*
  uno_map.h - driver code for STM32H7xx ARM processor on a Nucleo-H743ZI board

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

/* PIN NOTES:

Cycle start is on A5, not A2 due to pin interrupt clash with A0.
If Ethernet is enabled D11 cannot be used and JP6 on Nucleo-144 boards must be removed!

Default:
Spindle On   D12
Spindle Dir  D1
Spindle PWM  D13
Z-Limit      D11

Modbus enabled:
Spindle On   D12
Spindle Dir  -
Spindle PWM  D13
Z-Limit      D11

Ethernet enabled:
Spindle On   D0
Spindle Dir  D1
Spindle PWM  D13
Z-Limit      D12

Ethernet and Modbus enabled:
Spindle On   D13
Spindle Dir  -
Spindle PWM  -
Z-Limit      D12

*/

#if N_AUTO_SQUARED || N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#define BOARD_NAME "Generic Uno"
#define BOARD_URL "https://www.makerfabs.com/arduino-cnc-shield-v3.html"
#define IS_NUCLEO_BOB

#define SERIAL_PORT             32   // GPIOD: TX =  8, RX = 9
#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define SERIAL1_PORT            61   // GPIOG: TX = 14, RX = 9
#endif

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
#define STEPPERS_ENABLE_PORT   GPIOF // D8
#define STEPPERS_ENABLE_PIN    12

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOD // D9
#define X_LIMIT_PIN             15
#define Y_LIMIT_PORT            GPIOD // D10
#define Y_LIMIT_PIN             14
#if ETHERNET_ENABLE
#define Z_LIMIT_PORT            GPIOA // D12
#define Z_LIMIT_PIN             6
#else
#define Z_LIMIT_PORT            GPIOA // D11
#define Z_LIMIT_PIN             7
#endif
#define LIMIT_INMODE            GPIO_SINGLE

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOF // A4
#define M3_STEP_PIN             5
#define M3_DIRECTION_PORT       GPIOF // A3
#define M3_DIRECTION_PIN        3
#endif

// Define spindle enable, direction and PWM output pins.

#if ETHERNET_ENABLE && (MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define AUXOUTPUT2_PORT         GPIOA // D12 - spindle on
#define AUXOUTPUT2_PIN          6
#else
#if ETHERNET_ENABLE
#define AUXOUTPUT2_PORT         GPIOG // D0 - spindle on
#define AUXOUTPUT2_PIN          9
#else
#define AUXOUTPUT2_PORT         GPIOA // D12 - spindle on
#define AUXOUTPUT2_PIN          6
#endif
#define AUXOUTPUT1_PORT         GPIOA // D13 - spindle PWM
#define AUXOUTPUT1_PIN          5
#if !(MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define AUXOUTPUT0_PORT         GPIOG // D1 - spindle dir
#define AUXOUTPUT0_PIN          14
#define AUXOUTPUT3_PORT         GPIOG // D0
#define AUXOUTPUT3_PIN          9
#endif
#endif
#if N_ABC_MOTORS
#define AUXOUTPUT4_PORT         GPIOF // A4 - coolant flood
#define AUXOUTPUT4_PIN          5
#else
#define AUXOUTPUT4_PORT         GPIOF // A3 - coolant flood
#define AUXOUTPUT4_PIN          3
#define AUXOUTPUT5_PORT         GPIOF // A4 - coolant mist
#define AUXOUTPUT5_PIN          5
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM && defined(AUXOUTPUT1_PIN)
#define SPINDLE_PWM_PORT        AUXOUTPUT1_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE && defined(AUXOUTPUT0_PIN)
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT0_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT0_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT4_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT4_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#ifdef AUXOUTPUT5_PIN
#define COOLANT_MIST_PORT      AUXOUTPUT5_PORT
#define COOLANT_MIST_PIN       AUXOUTPUT5_PIN
#else
#undef COOLANT_ENABLE
#ifdef COOLANT_FLOOD_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif
#endif

#define AUXINPUT0_PORT          GPIOC // A2 - probe
#define AUXINPUT0_PIN           3
#define AUXINPUT1_PORT          GPIOA // A0 - reset/EStop
#define AUXINPUT1_PIN           3
#define AUXINPUT2_PORT          GPIOC // A1 - feed hold
#define AUXINPUT2_PIN           0
#define AUXINPUT3_PORT          GPIOF // A5 - cycle start
#define AUXINPUT3_PIN           10

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT1_PORT
#define RESET_PIN               AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT2_PORT
#define FEED_HOLD_PIN           AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT3_PORT
#define CYCLE_START_PIN         AUXINPUT3_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT0_PORT
#define PROBE_PIN               AUXINPUT0_PIN
#endif

/**/
