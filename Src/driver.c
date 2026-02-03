/*

  driver.c - driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io
  Copyright (c) 2023-2026 Jon Escombe

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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "main.h"
#include "driver.h"
#include "serial.h"

#include "grbl/task.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"
#include "grbl/machine_limits.h"

#if I2C_ENABLE
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#include "sdmmc.h"
#endif

#if SPIFLASH_ENABLE
#include "spiflash.h"
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if ETHERNET_ENABLE
#include "enet.h"
#endif

#if (LIMIT_MASK|CONTROL_MASK|DEVICES_IRQ_MASK) != (LIMIT_MASK_SUM+CONTROL_MASK_SUM+DEVICES_IRQ_MASK_SUM)
#error Interrupt enabled input pins must have unique pin numbers!
#endif

#define STEPPER_TIMER_DIV 4
#define DRIVER_IRQMASK (LIMIT_MASK|DEVICES_IRQ_MASK)

#if SPINDLE_ENCODER_ENABLE

#include "grbl/spindle_sync.h"

#define RPM_TIMER_RESOLUTION 1

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

#if RPM_TIMER_N != 2
static volatile uint32_t rpm_timer_ovf = 0;
#define RPM_TIMER_RESOLUTION 1
#define RPM_TIMER_COUNT (RPM_TIMER->CNT | (rpm_timer_ovf << 16))
#else
#define RPM_TIMER_RESOLUTION 1
#define RPM_TIMER_COUNT RPM_TIMER->CNT
#endif

#endif // SPINDLE_ENCODER_ENABLE

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,       .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,       .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,       .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = X2_LIMIT_PORT,      .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = Y2_LIMIT_PORT,      .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,       .port = Z2_LIMIT_PORT,      .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,         .port = A_LIMIT_PORT,       .pin = A_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = B_LIMIT_PORT,       .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = C_LIMIT_PORT,       .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef U_LIMIT_PIN
    { .id = Input_LimitU,         .port = U_LIMIT_PORT,       .pin = U_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef V_LIMIT_PIN
    { .id = Input_LimitV,         .port = V_LIMIT_PORT,       .pin = V_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef W_LIMIT_PIN
    { .id = Input_LimitW,         .port = W_LIMIT_PORT,       .pin = W_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,     .port = X_LIMIT_PORT_MAX,   .pin = X_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,     .port = Y_LIMIT_PORT_MAX,   .pin = Y_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,     .port = Z_LIMIT_PORT_MAX,   .pin = Z_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,     .port = A_LIMIT_PORT_MAX,   .pin = A_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,     .port = B_LIMIT_PORT_MAX,   .pin = B_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,     .port = C_LIMIT_PORT_MAX,   .pin = C_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef U_LIMIT_PIN_MAX
    { .id = Input_LimitU_Max,     .port = U_LIMIT_PORT_MAX,   .pin = U_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef V_LIMIT_PIN_MAX
    { .id = Input_LimitV_Max,     .port = V_LIMIT_PORT_MAX,   .pin = V_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
#ifdef W_LIMIT_PIN_MAX
    { .id = Input_LimitW_Max,     .port = W_LIMIT_PORT_MAX,   .pin = W_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
#if SPINDLE_SYNC_ENABLE
    { .id = Input_SpindleIndex,   .port = SPINDLE_INDEX_PORT, .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_SpindleIndex },
#endif
#ifdef SPI_IRQ_PORT
    { .id = Input_SPIIRQ,         .port = SPI_IRQ_PORT,       .pin = SPI_IRQ_PIN,         .group = PinGroup_SPI },
#endif
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    { .id = Input_SdCardDetect,   .port = SD_DETECT_PORT,     .pin = SD_DETECT_PIN,       .group = PinGroup_SdCard },
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,           .port = AUXINPUT0_PORT,     .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,           .port = AUXINPUT1_PORT,     .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,           .port = AUXINPUT2_PORT,     .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,           .port = AUXINPUT3_PORT,     .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,           .port = AUXINPUT4_PORT,     .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,           .port = AUXINPUT5_PORT,     .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,           .port = AUXINPUT6_PORT,     .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7,           .port = AUXINPUT7_PORT,     .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT8_PIN
    { .id = Input_Aux8,           .port = AUXINPUT8_PORT,     .pin = AUXINPUT8_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT9_PIN
    { .id = Input_Aux9,           .port = AUXINPUT9_PORT,     .pin = AUXINPUT9_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT10_PIN
    { .id = Input_Aux10,          .port = AUXINPUT10_PORT,    .pin = AUXINPUT10_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT11_PIN
    { .id = Input_Aux11,          .port = AUXINPUT11_PORT,    .pin = AUXINPUT11_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT0_ANALOG_PIN
    { .id = Input_Analog_Aux0,    .port = AUXINPUT0_ANALOG_PORT, .pin = AUXINPUT0_ANALOG_PIN, .group = PinGroup_AuxInputAnalog },
#endif
#ifdef AUXINPUT1_ANALOG_PIN
    { .id = Input_Analog_Aux1,    .port = AUXINPUT1_ANALOG_PORT, .pin = AUXINPUT1_ANALOG_PIN, .group = PinGroup_AuxInputAnalog },
#endif
#ifdef AUXINPUT2_ANALOG_PIN
    { .id = Input_Analog_Aux2,    .port = AUXINPUT2_ANALOG_PORT, .pin = AUXINPUT2_ANALOG_PIN, .group = PinGroup_AuxInputAnalog },
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,              .port = X_STEP_PORT,            .pin = X_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepY,              .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepZ,              .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,            .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,           .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,            .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,           .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,            .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,           .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef A_AXIS
    { .id = Output_StepA,              .port = A_STEP_PORT,            .pin = A_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,              .port = B_STEP_PORT,            .pin = B_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,              .port = C_STEP_PORT,            .pin = C_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef U_AXIS
    { .id = Output_StepU,              .port = U_STEP_PORT,            .pin = U_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef V_AXIS
    { .id = Output_StepV,              .port = V_STEP_PORT,            .pin = V_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef W_AXIS
    { .id = Output_StepW,              .port = W_STEP_PORT,            .pin = W_STEP_PIN,            .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
    { .id = Output_DirX,               .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirY,               .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirZ,               .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,             .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,      .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,             .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,      .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,             .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,      .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef A_AXIS
    { .id = Output_DirA,               .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,               .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,               .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef U_AXIS
    { .id = Output_DirU,               .port = U_DIRECTION_PORT,       .pin = U_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef V_AXIS
    { .id = Output_DirV,               .port = V_DIRECTION_PORT,       .pin = V_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef W_AXIS
    { .id = Output_DirW,               .port = W_DIRECTION_PORT,       .pin = W_DIRECTION_PIN,       .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef STEPPERS_POWER_PORT
    { .id = Output_StepperPower,       .port = STEPPERS_POWER_PORT,    .pin = STEPPERS_POWER_PIN,    .group = PinGroup_StepperPower },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,      .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,   .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,     .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,     .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,     .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,     .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,     .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,     .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef U_ENABLE_PORT
    { .id = Output_StepperEnableU,     .port = U_ENABLE_PORT,          .pin = U_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef V_ENABLE_PORT
    { .id = Output_StepperEnableU,     .port = V_ENABLE_PORT,          .pin = V_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef W_ENABLE_PORT
    { .id = Output_StepperEnableW,     .port = W_ENABLE_PORT,          .pin = W_ENABLE_PIN,          .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,     .port = X2_ENABLE_PORT,         .pin = X2_ENABLE_PIN,         .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,     .port = Y2_ENABLE_PORT,         .pin = Y2_ENABLE_PIN,         .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,     .port = Z2_ENABLE_PORT,         .pin = Z2_ENABLE_PIN,         .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#endif // !TRINAMIC_MOTOR_ENABLE
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,    .port = MOTOR_CS_PORT,          .pin = MOTOR_CS_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSX_PIN
    { .id = Output_MotorChipSelectX,   .port = MOTOR_CSX_PORT,         .pin = MOTOR_CSX_PIN,         .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSY_PIN
    { .id = Output_MotorChipSelectY,   .port = MOTOR_CSY_PORT,         .pin = MOTOR_CSY_PIN,         .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSZ_PIN
    { .id = Output_MotorChipSelectZ,   .port = MOTOR_CSZ_PORT,         .pin = MOTOR_CSZ_PIN,         .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM3_PIN
    { .id = Output_MotorChipSelectM3,  .port = MOTOR_CSM3_PORT,        .pin = MOTOR_CSM3_PIN,        .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM4_PIN
    { .id = Output_MotorChipSelectM4,  .port = MOTOR_CSM4_PORT,        .pin = MOTOR_CSM4_PIN,        .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM5_PIN
    { .id = Output_MotorChipSelectM5,  .port = MOTOR_CSM5_PORT,        .pin = MOTOR_CSM5_PIN,        .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,    .port = MOTOR_CS_PORT,          .pin = MOTOR_CS_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_UARTX_PIN
    { .id = Bidirectional_MotorUARTX,  .port = MOTOR_UARTX_PORT,       .pin = MOTOR_UARTX_PIN,       .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTY_PIN
    { .id = Bidirectional_MotorUARTY,  .port = MOTOR_UARTY_PORT,       .pin = MOTOR_UARTY_PIN,       .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTZ_PIN
    { .id = Bidirectional_MotorUARTZ,  .port = MOTOR_UARTZ_PORT,       .pin = MOTOR_UARTZ_PIN,       .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM3_PIN
    { .id = Bidirectional_MotorUARTM3, .port = MOTOR_UARTM3_PORT,      .pin = MOTOR_UARTM3_PIN,      .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM4_PIN
    { .id = Bidirectional_MotorUARTM4, .port = MOTOR_UARTM4_PORT,      .pin = MOTOR_UARTM4_PIN,      .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM5_PIN
    { .id = Bidirectional_MotorUARTM5, .port = MOTOR_UARTM5_PORT,      .pin = MOTOR_UARTM5_PIN,      .group = PinGroup_MotorUART },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,           .port = SD_CS_PORT,             .pin = SD_CS_PIN,             .group = PinGroup_SdCard },
#endif
#ifdef SPI_CS_PORT
    { .id = Output_SPICS,              .port = SPI_CS_PORT,            .pin = SPI_CS_PIN,            .group = PinGroup_SPI },
#endif
#ifdef SPI_RST_PORT
    { .id = Output_SPIRST,             .port = SPI_RST_PORT,           .pin = SPI_RST_PIN,           .group = PinGroup_SPI },
#endif
#if defined(MODBUS_RTU_STREAM) && defined(RS485_DIR_PORT)
    { .id = Output_RS485_Direction,    .port = RS485_DIR_PORT,         .pin = RS485_DIR_PIN,         .group = PinGroup_UART + MODBUS_RTU_STREAM },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,               .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,               .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,               .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,               .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,               .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,               .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,               .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,               .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT8_PORT
    { .id = Output_Aux8,               .port = AUXOUTPUT8_PORT,        .pin = AUXOUTPUT8_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT9_PORT
    { .id = Output_Aux9,               .port = AUXOUTPUT9_PORT,        .pin = AUXOUTPUT9_PIN,        .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT10_PORT
    { .id = Output_Aux10,              .port = AUXOUTPUT10_PORT,       .pin = AUXOUTPUT10_PIN,       .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT0_ANALOG_PORT
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_ANALOG_PORT, .pin = AUXOUTPUT0_ANALOG_PIN,    .group = PinGroup_AuxOutputAnalog },
#elif defined(AUXOUTPUT0_PWM_PORT)
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_PWM_PORT,    .pin = AUXOUTPUT0_PWM_PIN,       .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT1_ANALOG_PORT
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_ANALOG_PORT, .pin = AUXOUTPUT1_ANALOG_PIN,    .group = PinGroup_AuxOutputAnalog },
#elif defined(AUXOUTPUT1_PWM_PORT)
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_PWM_PORT,    .pin = AUXOUTPUT1_PWM_PIN,       .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } }
#endif
};

extern volatile uint32_t _bootflag __attribute__((section(".dtcmdata")));

extern __IO uint32_t uwTick, cycle_count;
static uint32_t systick_safe_read = 0, cycles2us_factor = 0;
static uint32_t aux_irq = 0;
static bool IOInitDone = false;
static pin_group_pins_t limit_inputs = {0};
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static input_signal_t *pin_irq[16] = {0};
static struct {
    // t_* parameters are timer ticks
    uint32_t t_min_period;
    uint32_t t_on; // delayed pulse
    uint32_t t_off;
    uint32_t t_on_off_min;
    uint32_t t_off_min;
    uint32_t t_dly_off_min;
    axes_signals_t out;
#if STEP_INJECT_ENABLE
    uint32_t length;
    uint32_t delay;
    struct {
        hal_timer_t timer;
        axes_signals_t claimed;
        volatile axes_signals_t axes;
        volatile axes_signals_t out;
    } inject;
#endif
} step_pulse = {};

#if defined(SAFETY_DOOR_PIN) || defined(QEI_SELECT_PIN)
static pin_debounce_t debounce;
#endif
static void aux_irq_handler (uint8_t port, bool state);

#if defined(I2C_STROBE_PIN) || SPI_IRQ_BIT

#if defined(I2C_STROBE_PIN)
static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };
#endif

#if SPI_IRQ_BIT
static driver_irq_handler_t spi_irq = { .type = IRQ_SPI };
#endif

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok = false;

    switch(irq) {

#if defined(I2C_STROBE_PIN)
        case IRQ_I2C_Strobe:
            if((ok = i2c_strobe.callback == NULL))
                i2c_strobe.callback = handler;
            break;
#endif

#if SPI_IRQ_BIT
        case IRQ_SPI:
            if((ok = spi_irq.callback == NULL))
                spi_irq.callback = handler;
            break;
#endif

        default:
            break;
    }

    return ok;
}

#endif // defined(I2C_STROBE_PIN) || SPI_IRQ_BIT

#include "grbl/stepdir_map.h"

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else {
        delay.callback = NULL;
        if(callback)
            callback();
    }
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_MOTOR_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
#else
 #ifdef STEPPERS_ENABLE_PORT
    DIGITAL_OUT(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_BIT, enable.x);
 #else
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_BIT, enable.x);
  #ifdef X2_ENABLE_PORT
    DIGITAL_OUT(X2_ENABLE_PORT, X2_ENABLE_BIT, enable.x);
  #endif
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_BIT, enable.y);
  #ifdef Y2_ENABLE_PORT
    DIGITAL_OUT(Y2_ENABLE_PORT, Y2_ENABLE_BIT, enable.y);
  #endif
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_BIT, enable.z);
  #ifdef Z2_ENABLE_PORT
    DIGITAL_OUT(Z2_ENABLE_PORT, Z2_ENABLE_BIT, enable.z);
  #endif
  #ifdef A_ENABLE_PORT
    DIGITAL_OUT(A_ENABLE_PORT, A_ENABLE_BIT, enable.a);
  #endif
  #ifdef B_ENABLE_PORT
    DIGITAL_OUT(B_ENABLE_PORT, B_ENABLE_BIT, enable.b);
  #endif
  #ifdef C_ENABLE_PORT
    DIGITAL_OUT(C_ENABLE_PORT, C_ENABLE_BIT, enable.c);
  #endif
  #ifdef U_ENABLE_PORT
    DIGITAL_OUT(U_ENABLE_PORT, U_ENABLE_BIT, enable.u);
  #endif
  #ifdef V_ENABLE_PORT
    DIGITAL_OUT(V_ENABLE_PORT, V_ENABLE_BIT, enable.u);
  #endif
  #ifdef W_ENABLE_PORT
    DIGITAL_OUT(W_ENABLE_PORT, W_ENABLE_PIN, enable.w);
  #endif
 #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);

    STEPPER_TIMER->ARR = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up
    STEPPER_TIMER->EGR = TIM_EGR_UG;
    STEPPER_TIMER->SR = 0;
    STEPPER_TIMER->DIER = TIM_DIER_UIE;
    STEPPER_TIMER->CR1 |= TIM_CR1_CEN;
}

// Sets up stepper driver interrupt timeout, "Normal" version
ISR_CODE static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->ARR = cycles_per_tick < (1UL << 20) ? max(cycles_per_tick, step_pulse.t_min_period) : 0x000FFFFFUL;
}

#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void stepper_step_out (axes_signals_t step_out1)
{
    axes_signals_t step_out2;

#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        step_out2.bits = step_out1.bits & motors_2.bits;
        step_out1.bits = step_out1.bits & motors_1.bits;

        uint_fast8_t idx, mask = 1;
        axes_signals_t step1 = { .bits = step_out1.bits },
                       step2 = { .bits = step_out2.bits };

        step_out1.bits ^= settings.steppers.step_invert.bits;
        step_out2.bits ^= settings.steppers.step_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) {

                if(step2.bits & mask) switch(idx) {
#ifdef X2_STEP_PIN
                    case X_AXIS:
                        DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out2.x);
                        break;
#endif
#ifdef Y2_STEP_PIN
                    case Y_AXIS:
                        DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out2.y);
                        break;
#endif
#ifdef Z2_STEP_PIN
                    case Z_AXIS:
                        DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out2.z);
                        break;
#endif
                }

                if(step1.bits & mask) switch(idx) {

                    case X_AXIS:
                        DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_out1.x);
                        break;

                    case Y_AXIS:
                        DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_out1.y);
                        break;

                    case Z_AXIS:
                        DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_out1.z);
                        break;
#ifdef A_AXIS
                    case A_AXIS:
                        DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_out1.a);
                        break;
#endif
#ifdef B_AXIS
                    case B_AXIS:
                        DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_out1.b);
                        break;
#endif
#ifdef C_AXIS
                    case C_AXIS:
                        DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_out1.c);
                        break;
#endif
#ifdef U_AXIS
                    case U_AXIS:
                        DIGITAL_OUT(U_STEP_PORT, U_STEP_BIT, step_out1.u);
                        break;
#endif
#ifdef V_AXIS
                    case V_AXIS:
                        DIGITAL_OUT(V_STEP_PORT, V_STEP_BIT, step_out1.v);
                        break;
#endif
#ifdef W_AXIS
                    case W_AXIS:
                        DIGITAL_OUT(W_STEP_PORT, W_STEP_BIT, step_out1.w);
                        break;
#endif
                }
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    step_out2.bits = (step_out1.bits & motors_2.bits) ^ settings.steppers.step_invert.bits;

#if STEP_OUTMODE == GPIO_SINGLE
    step_out1.bits = (step_out1.bits & motors_1.bits) ^ settings.steppers.step_invert.bits;

    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_out1.x);
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_out1.y);
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_out1.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_out1.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_out1.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_out1.c);
  #endif
  #ifdef U_AXIS
    DIGITAL_OUT(U_STEP_PORT, U_STEP_BIT, step_out1.u);
  #endif
  #ifdef V_AXIS
    DIGITAL_OUT(V_STEP_PORT, V_STEP_BIT, step_out1.v);
  #endif
  #ifdef W_AXIS
    DIGITAL_OUT(W_STEP_PORT, W_STEP_BIT, step_out1.w);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_out1.value & motors_1.value];
#else
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (((step_out1.bits & motors_1.bits) ^ settings.steppers.step_invert.bits) << STEP_OUTMODE);
#endif

#ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out2.x);
#endif
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out2.y);
#endif
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out2.z);
#endif
#if STEP_INJECT_ENABLE
    }
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // SQUARING DISABLED

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepper_step_out (axes_signals_t step_out)
{
#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        step_out.bits ^= settings.steppers.step_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_out.x);
                    break;

                case Y_AXIS:
                    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_out.y);
                    break;

                case Z_AXIS:
                    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_out.z);
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_out.a);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_out.b);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_out.c);
                    break;
#endif
#ifdef U_AXIS
                case U_AXIS:
                    DIGITAL_OUT(U_STEP_PORT, U_STEP_BIT, step_out.u);
                    break;
#endif
#ifdef V_AXIS
                case V_AXIS:
                    DIGITAL_OUT(V_STEP_PORT, V_STEP_BIT, step_out.v);
                    break;
#endif
#ifdef W_AXIS
                case W_AXIS:
                    DIGITAL_OUT(W_STEP_PORT, W_STEP_BIT, step_out.w);
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

#if STEP_OUTMODE == GPIO_SINGLE
    step_out.bits ^= settings.steppers.step_invert.bits;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_out.x);
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out.x);
  #endif
   DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_out.y);
  #ifdef Y2_STEP_PIN
   DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out.y);
  #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_out.z);
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out.z);
  #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_out.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_out.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_out.c);
  #endif
  #ifdef U_AXIS
    DIGITAL_OUT(U_STEP_PORT, U_STEP_BIT, step_out.u);
  #endif
  #ifdef V_AXIS
    DIGITAL_OUT(V_STEP_PORT, V_STEP_BIT, step_out.v);
  #endif
  #ifdef W_AXIS
    DIGITAL_OUT(W_STEP_PORT, W_STEP_BIT, step_out.w);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_out.value];
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out.x ^ settings.steppers.step_invert.x);
  #endif
  #ifdef Y2_STEP_PIN
      DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out.y ^ settings.steppers.step_invert.y);
  #endif
  #ifdef Z2_STEP_PIN
      DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out.z ^ settings.steppers.step_invert.z);
  #endif
#else // STEP_OUTMODE == GPIO_SHIFTx
    step_out.bits ^= settings.steppers.step_invert.bits;
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_out.bits << STEP_OUTMODE);
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out.x);
  #endif
  #ifdef Y2_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out.y);
  #endif
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out.z);
  #endif
#endif
#if STEP_INJECT_ENABLE
    }
#endif
}

#endif // SQUARING_ENABLED

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif
        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif
        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif
        #if Y_GANGED
            ganged.y = On;
        #endif
        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
}

#endif // GANGING_ENABLED

// Set stepper direction output pins
// NOTE: see note for stepper_step_out()
inline static __attribute__((always_inline)) void stepper_dir_out (axes_signals_t dir_out)
{
#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        dir_out.bits ^= settings.steppers.dir_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_BIT, dir_out.x);
#ifdef X2_DIRECTION_PIN
                    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
                    break;

                case Y_AXIS:
                    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_BIT, dir_out.y);
#ifdef Y2_DIRECTION_PIN
                    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
                    break;

                case Z_AXIS:
                    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_BIT, dir_out.z);
#ifdef Z2_DIRECTION_PIN
                    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
#endif
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_BIT, dir_out.a);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_BIT, dir_out.b);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_BIT, dir_out.c);
                    break;
#endif
#ifdef U_AXIS
                case U_AXIS:
                    DIGITAL_OUT(U_DIRECTION_PORT, U_DIRECTION_BIT, dir_out.u);
                    break;
#endif
#ifdef V_AXIS
                case V_AXIS:
                    DIGITAL_OUT(V_DIRECTION_PORT, V_DIRECTION_BIT, dir_out.v);
                    break;
#endif
#ifdef W_AXIS
                case W_AXIS:
                    DIGITAL_OUT(W_DIRECTION_PORT, W_DIRECTION_BIT, dir_out.w);
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

#if DIRECTION_OUTMODE == GPIO_SINGLE
    dir_out.mask ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_BIT, dir_out.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_BIT, dir_out.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_BIT, dir_out.z);
 #ifdef GANGING_ENABLED
    dir_out.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_BIT, dir_out.x);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_BIT, dir_out.z);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_BIT, dir_out.y);
  #endif
 #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_BIT, dir_out.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_BIT, dir_out.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_BIT, dir_out.c);
  #endif
  #ifdef U_AXIS
    DIGITAL_OUT(U_DIRECTION_PORT, U_DIRECTION_BIT, dir_out.u);
  #endif
  #ifdef V_AXIS
    DIGITAL_OUT(V_DIRECTION_PORT, V_DIRECTION_BIT, dir_out.v);
  #endif
  #ifdef W_AXIS
    DIGITAL_OUT(W_DIRECTION_PORT, W_DIRECTION_BIT, dir_out.w);
  #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | dir_outmap[dir_out.value];
 #ifdef GANGING_ENABLED
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, (dir_out.x ^ settings.steppers.dir_invert.x) ^ settings.steppers.ganged_dir_invert.x;
  #endif
  #ifdef Y2_DIRECTION_PIN
      DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, (dir_out.y ^ settings.steppers.dir_invert.y) ^ settings.steppers.ganged_dir_invert.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
      DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, (dir_out.z ^ settings.steppers.dir_invert.z) ^ settings.steppers.ganged_dir_invert.z;
  #endif
 #endif
#else
  #ifdef GANGING_ENABLED
    dir_out.mask ^= settings.steppers.dir_invert.mask;
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (dir_out.mask << DIRECTION_OUTMODE);
    dir_out.mask ^= settings.steppers.ganged_dir_invert.mask;
   #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT->ODR, X2_DIRECTION_BIT, dir_out.x);
   #endif
   #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT->ODR, Y2_DIRECTION_BIT, dir_out.y);
   #endif
   #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT->ODR, Z2_DIRECTION_BIT, dir_out.z);
   #endif
  #else
   DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | ((dir_out.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
  #endif
#endif
#if STEP_INJECT_ENABLE
    }
#endif
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->DIER &= ~TIM_DIER_UIE;

    if(clear_signals) {
        stepper_dir_out((axes_signals_t){0});
        stepper_step_out((axes_signals_t){0});
    }
}

static inline __attribute__((always_inline)) void _stepper_step_out (axes_signals_t step_out)
{
    stepper_step_out(step_out);

    if((STEPPER_TIMER->SR & TIM_SR_UIF) || STEPPER_TIMER->CNT < step_pulse.t_on_off_min) {
        STEPPER_TIMER->CNT = step_pulse.t_on_off_min;
        NVIC_ClearPendingIRQ(STEPPER_TIMER_IRQn);
    }

    STEPPER_TIMER->CCR1 = STEPPER_TIMER->CNT - step_pulse.t_off;
    STEPPER_TIMER->SR = 0;
    STEPPER_TIMER->DIER |= TIM_DIER_CC1IE;
}

// Sets stepper direction and pulse pins and starts a step pulse.
ISR_CODE static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        stepper_dir_out(stepper->dir_out);
    }

    if(stepper->step_out.bits)
        _stepper_step_out(stepper->step_out);
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
ISR_CODE static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {

        stepper_dir_out(stepper->dir_out);

        if(stepper->step_out.bits) {

            if(stepper->step_out.bits & stepper->dir_changed.bits) {

                step_pulse.out = stepper->step_out; // Store out_bits

                if(STEPPER_TIMER->CNT < step_pulse.t_dly_off_min) {
                    STEPPER_TIMER->CNT = step_pulse.t_dly_off_min;
                    NVIC_ClearPendingIRQ(STEPPER_TIMER_IRQn);
                }

                STEPPER_TIMER->CCR2 = STEPPER_TIMER->CNT - step_pulse.t_on;

                STEPPER_TIMER->SR = 0;
                STEPPER_TIMER->DIER |= TIM_DIER_CC2IE;

            } else
                _stepper_step_out(stepper->step_out);
        }

        stepper->dir_changed.bits = 0;

        return;
    }

    if(stepper->step_out.bits)
        _stepper_step_out(stepper->step_out);
}

#if STEP_INJECT_ENABLE

static inline __attribute__((always_inline)) void inject_step (axes_signals_t step_out, axes_signals_t axes)
{
    uint_fast8_t idx = N_AXIS - 1;

    if(!step_out.bits)
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;

    step_out.bits ^= settings.steppers.step_invert.bits;

    do {
        if(axes.bits & (1 << (N_AXIS - 1))) {

            switch(idx) {

                case X_AXIS:
                    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_out.x);
#ifdef X2_STEP_PIN
                    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_out.x);
#endif
                    break;

                case Y_AXIS:
                    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_out.y);
#ifdef Y2_STEP_PIN
                    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_out.y);
#endif
                    break;

                case Z_AXIS:
                    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_out.z);
#ifdef Z2_STEP_PIN
                    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_out.z);
#endif
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_out.a);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_out.b);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_out.c);
                    break;
#endif
#ifdef U_AXIS
                case U_AXIS:
                    DIGITAL_OUT(U_STEP_PORT, U_STEP_BIT, step_out.u);
                    break;
#endif
#ifdef V_AXIS
                case V_AXIS:
                    DIGITAL_OUT(V_STEP_PORT, V_STEP_BIT, step_out.v);
                    break;
#endif
#ifdef W_AXIS
                case W_AXIS:
                    DIGITAL_OUT(W_STEP_PORT, W_STEP_BIT, step_out.w);
                    break;
#endif
            }
        }
        idx--;
        axes.bits <<= 1;
    } while(axes.bits & AXES_BITMASK);
}

static void stepperClaimMotor (uint_fast8_t axis_id, bool claim)
{
    if(claim)
        step_pulse.inject.claimed.mask |= ((1 << axis_id) & AXES_BITMASK);
    else {
        step_pulse.inject.claimed.mask &= ~(1 << axis_id);
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;
    }
}

void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out)
{
    if(step_out.bits) {

        uint_fast8_t idx = N_AXIS - 1;
        axes_signals_t axes = { .bits = (step_out.bits & AXES_BITMASK) };

        step_pulse.inject.out = step_out;
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits | step_out.bits;
        dir_out.bits ^= settings.steppers.dir_invert.bits;

        do {
            if(axes.bits & (1 << (N_AXIS - 1))) {

                switch(idx) {

                    case X_AXIS:
                        DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_BIT, dir_out.x);
#ifdef X2_DIRECTION_PIN
                        DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
                        break;

                    case Y_AXIS:
                        DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_BIT, dir_out.y);
#ifdef Y2_DIRECTION_PIN
                        DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
                        break;

                    case Z_AXIS:
                        DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_BIT, dir_out.z);
#ifdef Z2_DIRECTION_PIN
                        DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
#endif
                        break;
#ifdef A_AXIS
                    case A_AXIS:
                        DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_BIT, dir_out.a);
                        break;
#endif
#ifdef B_AXIS
                    case B_AXIS:
                        DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_BIT, dir_out.b);
                        break;
#endif
#ifdef C_AXIS
                    case C_AXIS:
                        DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_BIT, dir_out.c);
                        break;
#endif
#ifdef U_AXIS
                    case U_AXIS:
                        DIGITAL_OUT(U_DIRECTION_PORT, U_DIRECTION_BIT, dir_out.u);
                        break;
#endif
#ifdef V_AXIS
                    case V_AXIS:
                        DIGITAL_OUT(V_DIRECTION_PORT, V_DIRECTION_BIT, dir_out.v);
                        break;
#endif
#ifdef W_AXIS
                    case W_AXIS:
                        DIGITAL_OUT(W_DIRECTION_PORT, W_DIRECTION_BIT, dir_out.w);
                        break;
#endif
                }
            }
            idx--;
            axes.bits <<= 1;
        } while(axes.bits & AXES_BITMASK);

        if(step_pulse.delay == 0)
            inject_step(step_out, step_out);

        hal.timer.start(step_pulse.inject.timer, step_pulse.length);
    }
}

void step_inject_on (void *context)
{
    inject_step(step_pulse.inject.out, step_pulse.inject.out);
}

void step_inject_off (void *context)
{
    axes_signals_t axes = { .bits = step_pulse.inject.out.bits };

    step_pulse.inject.out.bits = 0;
    step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;

    inject_step((axes_signals_t){0}, axes);
}

#endif // STEP_INJECT_ENABLE

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    axes_signals_t pin;
    input_signal_t *limit;
    uint_fast8_t idx = limit_inputs.n_pins;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        limit = &limit_inputs.pins.inputs[--idx];
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit->id);
            disable = limit->group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        gpio_irq_enable(limit, disable ? IRQ_Mode_None : limit->mode.irq_mode);
    } while(idx);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

#if LIMIT_INMODE == GPIO_SINGLE
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_BIT);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_BIT);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_BIT);
  #ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_BIT);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PORT, C_LIMIT_BIT);
  #endif
  #ifdef U_LIMIT_PIN
    signals.min.u = DIGITAL_IN(U_LIMIT_PORT, U_LIMIT_BIT);
  #endif
  #ifdef V_LIMIT_PIN
    signals.min.v = DIGITAL_IN(V_LIMIT_PORT, V_LIMIT_BIT);
  #endif
  #ifdef W_LIMIT_PIN
    signals.min.w = DIGITAL_IN(W_LIMIT_PORT, W_LIMIT_BIT);
  #endif
#elif LIMIT_INMODE == GPIO_MAP
    uint32_t bits = LIMIT_PORT->IDR;
    signals.min.x = !!(bits & X_LIMIT_BIT);
    signals.min.y = !!(bits & Y_LIMIT_BIT);
    signals.min.z = !!(bits & Z_LIMIT_BIT);
  #ifdef A_LIMIT_PIN
    signals.min.a = !!(bits & A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = !!(bits & B_LIMIT_BIT);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = !!(bits & C_LIMIT_BIT);
  #endif
  #ifdef U_LIMIT_PIN
    signals.min.u = !!(bits & U_LIMIT_BIT);
  #endif
  #ifdef V_LIMIT_PIN
    signals.min.v = !!(bits & V_LIMIT_BIT);
  #endif
  #ifdef W_LIMIT_PIN
    signals.min.w = !!(bits & W_LIMIT_BIT);
  #endif
#else
    signals.min.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PORT, X2_LIMIT_BIT);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PORT, Y2_LIMIT_BIT);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PORT, Z2_LIMIT_BIT);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_BIT_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_BIT_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_BIT_MAX);
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = DIGITAL_IN(A_LIMIT_PORT_MAX, A_LIMIT_BIT_MAX);
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = DIGITAL_IN(B_LIMIT_PORT_MAX, B_LIMIT_BIT_MAX);
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = DIGITAL_IN(C_LIMIT_PORT_MAX, C_LIMIT_BIT_MAX);
#endif
#ifdef U_LIMIT_PIN_MAX
    signals.max.u = DIGITAL_IN(U_LIMIT_PORT_MAX, U_LIMIT_BIT_MAX);
#endif
#ifdef V_LIMIT_PIN_MAX
    signals.max.v = DIGITAL_IN(V_LIMIT_PORT_MAX, V_LIMIT_BIT_MAX);
#endif
#ifdef W_LIMIT_PIN_MAX
    signals.max.w = DIGITAL_IN(W_LIMIT_PORT_MAX, W_LIMIT_BIT_MAX);
#endif

    if(settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals = { settings.control_invert.mask };

#if defined(RESET_PIN) && !ESTOP_ENABLE
    signals.reset = DIGITAL_IN(RESET_PORT, 1<<RESET_PIN);
#endif
#if defined(RESET_PIN) && ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PORT, 1<<RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, 1<<FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, 1<<CYCLE_START_PIN);
#endif
#ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, 1<<SAFETY_DOOR_PIN);;
#endif
#ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, 1<<MOTOR_FAULT_PIN);
#endif
#ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PORT, 1<<MOTOR_WARNING_PIN);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return aux_ctrl_scan_status(signals);
}

#if DRIVER_PROBES

// Returns the probe triggered pin state.
static bool probeGetState (void * input)
{
    return DIGITAL_IN(((input_signal_t *)input)->port, ((input_signal_t *)input)->bit);
}

#endif // DRIVER_PROBES

#if MPG_ENABLE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(MPG_MODE_PORT, 1 << MPG_MODE_PIN) == 0);
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode != (DIGITAL_IN(MPG_MODE_PORT, 1 << MPG_MODE_PIN) == 0))
        stream_mpg_enable(true);
}

#endif // MPG_ENABLE

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *aux_in;
    control_signals_t signals = {0};

    if((aux_in = aux_ctrl_in_get(port))) {
        switch(aux_in->function) {
#ifdef QEI_SELECT_PIN
            case Input_QEI_Select:
                qei_select_handler();
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, (1 << I2C_STROBE_PIN)) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                task_add_immediate(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= aux_in->signal.mask;
        if(aux_in->irq_mode == IRQ_Mode_Change)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, aux_in->port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if(aux_ctrl->input == NULL) {

        uint_fast8_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {
            --i;
            if(inputpin[i].group == PinGroup_AuxInput && inputpin[i].user_port == aux_ctrl->port)
                aux_ctrl->input = &inputpin[i];
        } while(i && aux_ctrl->input == NULL);
    }

    if((pin = aux_ctrl_claim_port(aux_ctrl))) {

        switch(aux_ctrl->function) {
#if PROBE_ENABLE
            case Input_Probe:
                hal.driver_cap.probe = probe_add(Probe_Default, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if PROBE2_ENABLE
            case Input_Probe2:
                hal.driver_cap.probe2 = probe_add(Probe_2, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;

#endif
#if TOOLSETTER_ENABLE
            case Input_Toolsetter:
                hal.driver_cap.toolsetter = probe_add(Probe_Toolsetter, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if SAFETY_DOOR_ENABLE || defined(QEI_SELECT_PIN) || (defined(RESET_PIN) && !ESTOP_ENABLE)
  #if defined(RESET_PIN) && !ESTOP_ENABLE
            case Input_Reset:
  #endif
  #if SAFETY_DOOR_ENABLE
            case Input_SafetyDoor:
  #endif
  #ifdef QEI_SELECT_PIN
            case Input_QEI_Select:
  #endif
                ((input_signal_t *)aux_ctrl->input)->mode.debounce = ((input_signal_t *)aux_ctrl->input)->cap.debounce && hal.driver_cap.software_debounce;
                break;
#endif
            default: break;
        }
    }

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

static void aux_assign_irq (void)
{
    uint32_t i, j, irq = 0;
    input_signal_t *input, *input2;
    aux_ctrl_t *aux;
    pin_group_pins_t aux_digital_in = {};

    const control_signals_t main_signals = { .reset = On, .e_stop = On, .feed_hold = On, .cycle_start = On };

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {

        input = &inputpin[i];

        if(input->group == PinGroup_AuxInput) {

            input->bit = 1 << input->pin;

            if(aux_digital_in.pins.inputs == NULL)
                aux_digital_in.pins.inputs = input;

            input->user_port = aux_digital_in.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->mode.pull_mode = PullMode_Up;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = (DRIVER_IRQMASK & input->bit) ? IRQ_Mode_None : IRQ_Mode_Edges;

            aux = aux_ctrl_get_fn((aux_gpio_t){ .port = input->port, .pin = input->pin });

            if(input->cap.irq_mode == IRQ_Mode_None) {
                if(aux && xbar_is_probe_in(aux->function))
                    input->id = aux->function;
            } else {

                if(aux)
                    input->id = aux->function;

                if(irq & input->bit) { // duplicate IRQ

                    if(aux == NULL)
                        input->cap.irq_mode = IRQ_Mode_None;
                    else for(j = 0; j < aux_digital_in.n_pins - 1; j++) {
                        input2 = &aux_digital_in.pins.inputs[j];
                        if(input->pin == input2->pin) {
                            if(input->id < input2->id || (aux->signal.bits & main_signals.bits)) {
                                input2->cap.irq_mode = IRQ_Mode_None;
                                if(!xbar_is_probe_in(input2->id))
                                    input2->id = (pin_function_t)(Input_Aux0 + input2->user_port);
                            } else {
                                input->cap.irq_mode = IRQ_Mode_None;
                                if(!xbar_is_probe_in(input->id))
                                    input->id = (pin_function_t)(Input_Aux0 + input->user_port);
                            }
                        }
                    }
                } else
                    irq |= input->bit;
            }
        }
    }
}

#if SPINDLE_ENCODER_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;
    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = RPM_TIMER_COUNT - spindle_encoder.timer.last_pulse;

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AtSpeed:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            spindle_data.state_programmed.at_speed = !spindle_data.at_speed_enabled || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindle_encoder.spin_lock);

    uint32_t timeout = uwTick + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && uwTick <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }

#if RPM_TIMER_N != 2
    rpm_timer_ovf = 0;
#endif

    RPM_TIMER->EGR |= TIM_EGR_UG; // Reload RPM timer
    RPM_COUNTER->CR1 &= ~TIM_CR1_CEN;

    spindle_encoder.timer.last_index =
    spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->EGR |= TIM_EGR_UG;
    RPM_COUNTER->CCR1 = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CR1 |= TIM_CR1_CEN;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        if((spindle_data.at_speed_enabled = spindle->at_speed_tolerance > 0.0f)) {
            float tolerance = rpm * spindle->at_speed_tolerance / 100.0f;
            spindle_data.rpm_low_limit = rpm - tolerance;
            spindle_data.rpm_high_limit = rpm + tolerance;
        }
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
        spindle_data.rpm_programmed = spindle_data.rpm = rpm;
    }
}

#endif // SPINDLE_ENCODER_ENABLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant.invert.mask;
#ifdef COOLANT_FLOOD_PIN
    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, mode.flood);
#endif
#ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_BIT, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = { .mask = settings.coolant.invert.mask };

#ifdef COOLANT_FLOOD_PIN
    state.flood = (COOLANT_FLOOD_PORT->IDR & COOLANT_FLOOD_BIT) != 0;
#endif
#ifdef COOLANT_MIST_PIN
    state.mist  = (COOLANT_MIST_PORT->IDR & COOLANT_MIST_BIT) != 0;
#endif
    state.value ^= settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

static uint64_t getElapsedMicros (void)
{
    uint32_t ms, cycles;
    do {
        __LDREXW(&systick_safe_read);
        ms = uwTick;
        cycles = cycle_count;
    } while(__STREXW(1, &systick_safe_read));

    uint32_t cyccnt = DWT->CYCCNT;
    asm volatile("" : : : "memory");
    uint32_t ccdelta = cyccnt - cycles;
    uint32_t frac = ((uint64_t)ccdelta * cycles2us_factor) >> 32;

    return ms * 1000 + (frac > 1000 ? 1000 : frac);
}

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    if(irq_mode == IRQ_Mode_Rising) {
        EXTI->RTSR1 |= input->bit;
        EXTI->FTSR1 &= ~input->bit;
    } else if(irq_mode == IRQ_Mode_Falling) {
        EXTI->RTSR1 &= ~input->bit;
        EXTI->FTSR1 |= input->bit;
    } else if(irq_mode == IRQ_Mode_Change) {
        EXTI->RTSR1 |= input->bit;
        EXTI->FTSR1 |= input->bit;
    } else
        EXTI->IMR1 &= ~input->bit;   // Disable pin interrupt

    if(irq_mode != IRQ_Mode_None)
        EXTI->IMR1 |= input->bit;    // Enable pin interrupt
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{

#if USE_STEPDIR_MAP
    stepdirmap_init(settings);
#endif

    if(IOInitDone) {

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH
        };

        hal.stepper.go_idle(true);

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#if SPINDLE_ENCODER_ENABLE

        static const spindle_data_ptrs_t encoder_data = {
            .get = spindleGetData,
            .reset = spindleDataReset
        };

        static bool event_claimed = false;

        if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL)) {
            if(spindle_encoder.ppr != settings->spindle.ppr) {

                spindle_ptrs_t *spindle;

                hal.spindle_data.reset = spindleDataReset;
                if((spindle = spindle_get(0)))
                    spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

                if(!event_claimed) {
                    event_claimed = true;
                    on_spindle_programmed = grbl.on_spindle_programmed;
                    grbl.on_spindle_programmed = onSpindleProgrammed;
                }

                spindle_encoder.ppr = settings->spindle.ppr;
                spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
                spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
                spindle_encoder.maximum_tt = 250000UL / RPM_TIMER_RESOLUTION; // 250ms
                spindle_encoder.rpm_factor = (60.0f * 1000000.0f / RPM_TIMER_RESOLUTION) / (float)spindle_encoder.ppr;
                spindleDataReset();
            }
        } else {
            spindle_encoder.ppr = 0;
            hal.spindle_data.reset = NULL;
        }

        spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);

#endif // SPINDLE_ENCODER_ENABLE

        float sl = (float)hal.f_step_timer / 1000000.0f;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            step_pulse.t_on = (uint32_t)ceilf(sl * (max(STEP_PULSE_TOFF_MIN, settings->steppers.pulse_delay_microseconds) - STEP_PULSE_TOFF_LATENCY));
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else {
            step_pulse.t_on = 0;
            hal.stepper.pulse_start = stepperPulseStart;
        }

        step_pulse.t_min_period = (uint32_t)ceilf(sl * (settings->steppers.pulse_microseconds + STEP_PULSE_TOFF_MIN));
        step_pulse.t_off = (uint32_t)ceilf(sl * (settings->steppers.pulse_microseconds - STEP_PULSE_TOFF_LATENCY));
        step_pulse.t_off_min = (uint32_t)ceilf(sl * (STEP_PULSE_TOFF_MIN - STEP_PULSE_TON_LATENCY));
        step_pulse.t_on_off_min = step_pulse.t_off + step_pulse.t_off_min;
        step_pulse.t_dly_off_min = step_pulse.t_on + step_pulse.t_on_off_min;

#if STEP_INJECT_ENABLE

        timer_cfg_t step_inject_cfg = {
            .single_shot = On,
            .timeout_callback = step_inject_off
        };

        step_inject_cfg.irq0_callback = step_pulse.delay ? step_inject_on : NULL;
        step_inject_cfg.irq0 = step_pulse.delay;

        step_pulse.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            step_pulse.delay = (uint32_t)(10.0f * settings->steppers.pulse_delay_microseconds) - 1;
            if(step_pulse.delay > (uint32_t)(10.0f * STEP_PULSE_LATENCY))
                step_pulse.delay = max(10, step_pulse.delay - (uint32_t)(10.0f * STEP_PULSE_LATENCY));
        } else
            step_pulse.delay = 0;

        hal.timer.configure(step_pulse.inject.timer, &step_inject_cfg);

#endif // STEP_INJECT_ENABLE

        /*************************
         *  Control pins config  *
         *************************/

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0xFC00
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            input = &inputpin[--i];

            if(input->group == PinGroup_AuxInputAnalog)
                continue;

            if(!(input->group == PinGroup_AuxInput || input->group == PinGroup_MPG))
                input->mode.irq_mode = IRQ_Mode_None;

            switch(input->id) {

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#ifdef A_AXIS
                case Input_LimitA:
                case Input_LimitA_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef B_AXIS
                case Input_LimitB:
                case Input_LimitB_Max:
                    input->mode.pull_mode = !settings->limits.disable_pullup.b ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef C_AXIS
                case Input_LimitC:
                case Input_LimitC_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef U_AXIS
                case Input_LimitU:
                case Input_LimitU_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.u ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.u ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef V_AXIS
                case Input_LimitV:
                case Input_LimitV_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.v ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.v ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef W_AXIS
                case Input_LimitW:
                case Input_LimitW_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.w ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = limit_fei.w ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif

                case Input_SPIIRQ:
                    input->mode.pull_mode = true;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

                case Input_SpindleIndex:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
                case Input_SdCardDetect:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    input->mode.debounce = On;
                    break;
#endif

                default:
                    break;
            }

            if(input->group == PinGroup_AuxInput) {
                if(input->cap.irq_mode != IRQ_Mode_None) {
                    // Map interrupt to pin
                    uint32_t extireg = SYSCFG->EXTICR[input->pin >> 2] & ~(0b1111 << ((input->pin & 0b11) << 2));
                    extireg |= ((uint32_t)(GPIO_GET_INDEX(input->port)) << ((input->pin & 0b11) << 2));
                    SYSCFG->EXTICR[input->pin >> 2] = extireg;
                }
            }

            GPIO_Init.Pin = input->bit;
            GPIO_Init.Pull = input->mode.pull_mode == PullMode_Up ? GPIO_PULLUP : GPIO_NOPULL;

            switch(input->mode.irq_mode) {
                case IRQ_Mode_Rising:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING;
                    break;
                case IRQ_Mode_Falling:
                    GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
                    break;
                case IRQ_Mode_Change:
                    GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
                    break;
                default:
                    GPIO_Init.Mode = GPIO_MODE_INPUT;
                    break;
            }
            HAL_GPIO_Init(input->port, &GPIO_Init);

        } while(i);

        uint32_t irq_mask = DRIVER_IRQMASK|aux_irq;

        __HAL_GPIO_EXTI_CLEAR_IT(irq_mask);

        if(irq_mask & (1<<0)) {
            HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        }
        if(irq_mask & (1<<1)) {
            HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        }
        if(irq_mask & (1<<2)) {
            HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        }
        if(irq_mask & (1<<3)) {
            HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
        }
        if(irq_mask & (1<<4)) {
            HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        }
        if(irq_mask & 0x03E0) {
            HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
        if(irq_mask & 0xFC00) {
            HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 2);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        }

        hal.limits.enable(settings->limits.flags.hard_enabled, (axes_signals_t){0});
        aux_ctrl_irq_enable(settings, aux_irq_handler);
    }
}

static char *port2char (GPIO_TypeDef *port)
{
    static char name[3] = "P?";

    name[1] = 'A' + GPIO_GET_INDEX(port);

    return name;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description == NULL ? xbar_group_to_description(ppin->pin.group) : ppin->pin.description;

        pin_info(&pin, data);

        ppin = ppin->next;
    } while(ppin);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}


#if ETHERNET_ENABLE

uint32_t crc32_bitwise(const void* data, size_t length)
{
  const uint32_t polynomial = 0xEDB88320;
  uint32_t crc = 0xFFFFFFFF;
  unsigned char* current = (unsigned char*) data;
  while (length--)
  {
    crc ^= *current++;
    for (unsigned int j = 0; j < 8; j++)
      if (crc & 1)
        crc = (crc >> 1) ^ polynomial;
      else
        crc =  crc >> 1;
  }
  return ~crc;
}

bool bmac_eth_get (uint8_t mac[6])
{
#if defined(_WIZCHIP_)
    // WizNet OUI prefix
    mac[0] = 0x00;
    mac[1] = 0x08;
    mac[2] = 0xDC;
#else
    // ST OUI prefix
    mac[0] = 0x00;
    mac[1] = 0x80;
    mac[2] = 0xE1;
#endif

    // Set LAA bit
    mac[0] |= 0x02;

    // Get 96 bit unique device identifier
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();

    // Generate 32bit CRC from 96 bit UID
    uint32_t crc = crc32_bitwise(uid, 12);

    // Copy first 24bits of the CRC into the MAC address
    memcpy(&mac[3], &crc, 3);

    return true;
}

#endif // ETHERNET_ENABLE

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    // Interrupt_disableSleepOnIsrExit();

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    axes_signals_t st_enable = st_get_enable_out();

    // Switch on stepper driver power before enabling other output pins
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].group == PinGroup_StepperPower) {

            GPIO_Init.Pin = outputpin[i].bit = 1 << outputpin[i].pin;
            GPIO_Init.Mode = outputpin[i].mode.open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;

            if(outputpin[i].group == PinGroup_MotorChipSelect ||
                outputpin[i].group == PinGroup_MotorUART ||
                 outputpin[i].id == Output_SPICS ||
                  outputpin[i].id == Output_FlashCS ||
                   outputpin[i].id == Output_SdCardCS ||
                    (outputpin[i].group == PinGroup_StepperEnable && (st_enable.mask & xbar_fn_to_axismask(outputpin[i].id).mask)))
                outputpin[i].port->BSRR = GPIO_Init.Pin;

            HAL_GPIO_Init(outputpin[i].port, &GPIO_Init);
        }
    }

    hal.delay_ms(100, NULL);

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(!(outputpin[i].group == PinGroup_StepperPower ||
              outputpin[i].group == PinGroup_AuxOutputAnalog ||
               outputpin[i].id == Output_SpindlePWM ||
                outputpin[i].id == Output_Spindle1PWM)) {

            GPIO_Init.Pin = outputpin[i].bit = 1 << outputpin[i].pin;
            GPIO_Init.Mode = outputpin[i].mode.open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;

            if(outputpin[i].group == PinGroup_MotorChipSelect ||
                outputpin[i].group == PinGroup_MotorUART ||
                 outputpin[i].id == Output_SPICS ||
                  outputpin[i].group == PinGroup_StepperEnable)
                outputpin[i].port->ODR |= outputpin[i].bit;

            HAL_GPIO_Init(outputpin[i].port, &GPIO_Init);
        }
    }

    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

 // Stepper init

    STEPPER_TIMER_CLKEN();
    STEPPER_TIMER->CR1 = 0;
    STEPPER_TIMER->PSC = STEPPER_TIMER_DIV - 1;
    STEPPER_TIMER->CR1 = TIM_CR1_DIR|TIM_CR1_ARPE;

    HAL_NVIC_SetPriority(STEPPER_TIMER_IRQn, 0, 0);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQn);

#if SDCARD_ENABLE

    sdmmc_init();
    sdcard_init();

#endif

#if SPIFLASH_ENABLE
    spiflash_init();
#endif

#if LITTLEFS_ENABLE
    fs_littlefs_mount("/littlefs", stm32_littlefs_hal());
#endif

#if SPINDLE_ENCODER_ENABLE

    RPM_TIMER_CLKEN();
#if timerAPB2(RPM_TIMER_N)
    RPM_TIMER->PSC = HAL_RCC_GetPCLK2Freq() * 2 / 1000000UL * RPM_TIMER_RESOLUTION - 1;
#else
    RPM_TIMER->PSC = HAL_RCC_GetPCLK1Freq() * 2 / 1000000UL * RPM_TIMER_RESOLUTION - 1;
#endif
#if RPM_TIMER_N == 2
    RPM_TIMER->CR1 = TIM_CR1_CKD_1;
#else
    RPM_TIMER->CR1 = TIM_CR1_CKD_1|TIM_CR1_URS;
    RPM_TIMER->DIER |= TIM_DIER_UIE;
    HAL_NVIC_EnableIRQ(RPM_TIMER_IRQn);
    HAL_NVIC_SetPriority(RPM_TIMER_IRQn, 0, 0);
#endif
    RPM_TIMER->CR1 |= TIM_CR1_CEN;

    RPM_COUNTER_CLKEN();

#if SPINDLE_ENCODER_CLK == 1 // External clock mode 1 (TI1FP1 pin)
    RPM_COUNTER->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2;
#elif SPINDLE_ENCODER_CLK == 2 // External clock mode 2 (ETR pin)
    RPM_COUNTER->SMCR = TIM_SMCR_ECE;
#else
    #error Spindle encoder clock mode not defined
#endif


    RPM_COUNTER->PSC = 0;
    RPM_COUNTER->ARR = 65535;
    RPM_COUNTER->DIER = TIM_DIER_CC1IE;

    HAL_NVIC_EnableIRQ(RPM_COUNTER_IRQn);

    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pin = SPINDLE_PULSE_BIT;
    GPIO_Init.Pull = GPIO_PULLUP;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(SPINDLE_PULSE_PORT, &GPIO_Init);

    static const periph_pin_t ssp = {
        .function = Input_SpindlePulse,
        .group = PinGroup_SpindlePulse,
        .port = SPINDLE_PULSE_PORT,
        .pin = SPINDLE_PULSE_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&ssp);

#endif // SPINDLE_ENCODER_ENABLE

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

#if ETHERNET_ENABLE
    enet_start();
#endif

#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    if(!DIGITAL_IN(SD_DETECT_PORT, SD_DETECT_BIT))
        sdcard_detect(true);
#endif

    return IOInitDone;
}

#if RTC_ENABLE

static RTC_HandleTypeDef hrtc = {
    .Instance = RTC,
    .Init.HourFormat = RTC_HOURFORMAT_24,
    .Init.AsynchPrediv = 127,
    .Init.SynchPrediv = 255,
    .Init.OutPut = RTC_OUTPUT_DISABLE,
    .Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH,
    .Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN
};

static bool set_rtc_time (struct tm *time)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = time->tm_hour;
    sTime.Minutes = time->tm_min;
    sTime.Seconds = time->tm_sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
        sDate.WeekDay = RTC_WEEKDAY_MONDAY;
        sDate.Month = time->tm_mon + 1;
        sDate.Date = time->tm_mday;
        sDate.Year = time->tm_year - 100;
        hal.driver_cap.rtc_set = (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK);
    }

    return hal.driver_cap.rtc_set;
}

static bool get_rtc_time (struct tm *time)
{
    bool ok = false;

    if(hal.driver_cap.rtc_set) {

        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        if((ok = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK &&
                  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK)) {

            time->tm_hour = sTime.Hours;
            time->tm_min = sTime.Minutes;
            time->tm_sec = sTime.Seconds;
            time->tm_mon = sDate.Month - 1;
            time->tm_mday = sDate.Date;
            time->tm_year = sDate.Year + 100;
        }
    }

    return ok;
}

#endif

#if USB_SERIAL_CDC

static status_code_t enter_dfu (sys_state_t state, char *args)
{
    report_message("Entering DFU Bootloader", Message_Warning);
    hal.delay_ms(100, NULL);

    __disable_irq();
    _bootflag = 0xDEADBEEF;
    _bootflag = _bootflag; // Read back data to flush ECC before system reset
    __disable_irq();
    NVIC_SystemReset();

    return Status_OK;
}

static void onReportOptions (bool newopt)
{
    if(!newopt)
        report_plugin("Bootloader Entry", "0.02");
}

#if ESP_AT_ENABLE

#include "grbl/stream_passthru.h"

void stream_passthru_enter (void)
{
    __disable_irq();
    _bootflag = 0xFEEDC0DE;
    _bootflag = _bootflag; // Read back data to flush ECC before system reset
    NVIC_SystemReset();
}

#endif

#endif // USB_SERIAL_CDC

uint32_t get_free_mem (void)
{
    extern uint8_t _end; /* Symbol defined in the linker script */
    extern uint8_t _estack; /* Symbol defined in the linker script */
    extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;

    return stack_limit - (uint32_t)&_end - mallinfo().uordblks;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: grblHAL is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

#ifdef MPG_MODE_PIN

 // Drive MPG mode input pin low until setup complete

    GPIO_InitTypeDef GPIO_Init = {
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = 1 << MPG_MODE_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP
    };

    DIGITAL_OUT(MPG_MODE_PORT, 1 << MPG_MODE_PIN, 0);

    HAL_GPIO_Init(MPG_MODE_PORT, &GPIO_Init);

#endif

    uint32_t latency;
    RCC_ClkInitTypeDef clock_cfg;

    HAL_RCC_GetClockConfig(&clock_cfg, &latency);

#if defined(STM32H723xx)
    hal.info = "STM32H723";
#else
    hal.info = "STM32H743";
#endif
    hal.driver_version = "260127";
    hal.driver_url = "https://github.com/dresco/STM32H7xx";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif
    hal.driver_setup = driver_setup;
    hal.f_mcu = HAL_RCC_GetHCLKFreq() / 1000000UL * (clock_cfg.AHBCLKDivider == 0 ? 1 : 2);
    hal.f_step_timer = HAL_RCC_GetPCLK1Freq() * 2 / STEPPER_TIMER_DIV;
    hal.step_us_min = 1.5f;

    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.get_free_mem = get_free_mem;
    hal.delay_ms = &driver_delay;

    hal.timer.claim = timerClaim;
    hal.timer.configure = timerCfg;
    hal.timer.start = timerStart;
    hal.timer.stop = timerStop;

    hal.settings_changed = settings_changed;

    cycles2us_factor = 0xFFFFFFFFU / hal.f_mcu;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif
    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if I2C_STROBE_ENABLE || defined(SPI_IRQ_PIN)
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_micros = getElapsedMicros;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if RTC_ENABLE

    RCC_OscInitTypeDef OscInitStruct;

    HAL_RCC_GetOscConfig(&OscInitStruct);

    if(OscInitStruct.LSIState || OscInitStruct.LSEState) {

        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {
            .PeriphClockSelection = RCC_PERIPHCLK_RTC,
            .RTCClockSelection = OscInitStruct.LSEState ? RCC_RTCCLKSOURCE_LSE : RCC_RTCCLKSOURCE_LSI
        };

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) == HAL_OK) {

            __HAL_RCC_RTC_ENABLE();

            if((hal.driver_cap.rtc = HAL_RTC_Init(&hrtc) == HAL_OK)) {

                struct tm time;

                hal.driver_cap.rtc_set = On;
                hal.rtc.get_datetime = get_rtc_time;
                hal.rtc.set_datetime = set_rtc_time;

                if((hal.driver_cap.rtc_set = get_rtc_time(&time)))
                    hal.driver_cap.rtc_set = ((time.tm_year + 1900) * 10000 + (time.tm_mon + 1) * 100 + time.tm_mday) >= GRBL_BUILD;
            }
        }
    }

#endif

    serialRegisterStreams();

#if USB_SERIAL_CDC

    static const sys_command_t boot_command_list[] = {
        {"DFU", enter_dfu, { .allow_blocking = On, .noargs = On }, { .str = "enter DFU bootloader" } }
    };

    static sys_commands_t boot_commands = {
        .n_commands = sizeof(boot_command_list) / sizeof(sys_command_t),
        .commands = boot_command_list
    };

    grbl.on_report_options = onReportOptions;

    stream_connect(usbInit());
    system_register_commands(&boot_commands);

#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#if EEPROM_ENABLE
    if(!i2c_eeprom_init())
        task_run_on_startup(task_raise_alarm, (void *)Alarm_NVS_Failed);
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.size_max = 1024 * 32,
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

// driver capabilities

    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
#if SPINDLE_ENCODER_ENABLE
    hal.driver_cap.spindle_encoder = On;
#endif
    hal.coolant_cap.bits = COOLANT_ENABLE;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0}, aux_analog_in = {0}, aux_analog_out = {0};

    uint32_t i;
    input_signal_t *input;

    aux_assign_irq();

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        input->bit = 1 << input->pin;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;

            aux_inputs.n_pins++;

            if(!(input->id >= Input_Aux0 && input->id <= Input_AuxMax)) {
                input->id = Input_Aux0 + input->user_port;
                aux_ctrl_remap_explicit((aux_gpio_t){ .port = input->port, .pin = input->pin }, input->user_port, input);
            }

            if((input->cap.debounce = input->cap.irq_mode != IRQ_Mode_None)) {
                aux_irq |= input->bit;
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
            }
        } else if(input->group == PinGroup_AuxInputAnalog) {
            if(aux_analog_in.pins.inputs == NULL)
                aux_analog_in.pins.inputs = input;
            input->id = (pin_function_t)(Input_Analog_Aux0 + aux_analog_in.n_pins++);
            input->mode.analog = input->cap.analog = On;
        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            // input->mode.debounce = On;
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            if(LIMIT_MASK & input->bit)
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
            limit_inputs.n_pins++;
        } else if(input->group & PinGroup_SdCard) {
            if(input->bit & DEVICES_IRQ_MASK)
                pin_irq[__builtin_ffs(input->bit) - 1] = input;
        }
    }

    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins);
            aux_out_remap_explicit((aux_gpio_t){ .port = output->port, .pin = output->pin }, aux_outputs.n_pins, output);
            aux_outputs.n_pins++;
        } else if(output->group == PinGroup_AuxOutputAnalog) {
            if(aux_analog_out.pins.outputs == NULL)
                aux_analog_out.pins.outputs = output;
            output->mode.analog = On;
            output->id = (pin_function_t)(Output_Analog_Aux0 + aux_analog_out.n_pins++);
        }
    }

    if(aux_inputs.n_pins || aux_outputs.n_pins)
        ioports_init(&aux_inputs, &aux_outputs);

    if(aux_analog_in.n_pins || aux_analog_out.n_pins)
        ioports_init_analog(&aux_analog_in, &aux_analog_out);

    io_expanders_init();
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);

    extern bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl);
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);

#if DRIVER_SPINDLE_ENABLE ||  DRIVER_SPINDLE1_ENABLE
    extern void driver_spindles_init (void);
    driver_spindles_init();
#endif

#if STEP_INJECT_ENABLE
    if((step_pulse.inject.timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 100))) {
        hal.stepper.output_step = stepperOutputStep;
        hal.stepper.claim_motor = stepperClaimMotor;
    }
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if TRINAMIC_SPI_ENABLE
  extern void if_init (uint8_t motors, axes_signals_t enabled);
  trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};
  trinamic_if_init(&driver_if);
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif

#ifdef NEOPIXEL_SPI
    extern void neopixel_init (void);
    neopixel_init();
#endif

#if USB_SERIAL_CDC && ESP_AT_ENABLE

    #include "grbl/stream_passthru.h"

    bool enterpt;

    if((enterpt = _bootflag == 0xFEEDC0DE)) {
        _bootflag = 0x0;    // Reset trigger

        // Reduce USB IRQ priority to lower than the UART port!
        HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    }

    stream_passthru_init(COPROC_STREAM, 115200, enterpt);

#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
ISR_CODE void STEPPER_TIMER_IRQHandler (void)
{
//    DIGITAL_OUT(AUXOUTPUT0_PORT, 1<<AUXOUTPUT0_PIN, 1);

    // Delayed step pulse handler
    if((STEPPER_TIMER->SR & STEPPER_TIMER->DIER) & TIM_SR_CC2IF) {

        STEPPER_TIMER->DIER &= ~TIM_DIER_CC2IE;

        _stepper_step_out(step_pulse.out);
    }
    // Step pulse off handler
    else if((STEPPER_TIMER->SR & STEPPER_TIMER->DIER) & TIM_SR_CC1IF) {

        STEPPER_TIMER->DIER &= ~TIM_DIER_CC1IE;

        stepper_step_out((axes_signals_t){0});

        if((STEPPER_TIMER->SR & TIM_SR_UIF) || STEPPER_TIMER->CNT < step_pulse.t_off_min) {
            STEPPER_TIMER->CNT = step_pulse.t_off_min;
            NVIC_ClearPendingIRQ(STEPPER_TIMER_IRQn);
        }

        STEPPER_TIMER->SR = 0;
    }
    // Stepper timeout handler
    else if(STEPPER_TIMER->SR & TIM_SR_UIF) {
        STEPPER_TIMER->SR = 0;
        hal.stepper.interrupt_callback();
    }

//    DIGITAL_OUT(AUXOUTPUT0_PORT, 1<<AUXOUTPUT0_PIN, 0);
}

#if SPINDLE_ENCODER_ENABLE

ISR_CODE void RPM_COUNTER_IRQHandler (void)
{
    spindle_encoder.spin_lock = true;

    __disable_irq();
    uint32_t tval = RPM_TIMER_COUNT;
    uint16_t cval = RPM_COUNTER->CNT;
    __enable_irq();

    RPM_COUNTER->SR = ~TIM_SR_CC1IF;
    RPM_COUNTER->CCR1 = (uint16_t)(RPM_COUNTER->CCR1 + spindle_encoder.tics_per_irq);

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

#if RPM_TIMER_N != 2

ISR_CODE void RPM_TIMER_IRQHandler (void)
{
    RPM_TIMER->SR &= ~TIM_SR_UIF;

    rpm_timer_ovf++;
}

#endif

#endif // SPINDLE_ENCODER_ENABLE

void core_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

    if(input->mode.irq_mode == IRQ_Mode_Change ||
         DIGITAL_IN(input->port, input->bit) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1)) {

        if(input->group & PinGroup_Control) {
            hal.control.interrupt_callback(systemGetState());
        }
        if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            limit_signals_t state = limitsGetState();
            if(limit_signals_merge(state).value) // TODO: add check for limit switches having same state as when limit_isr were invoked?
                hal.limits.interrupt_callback(state);
        }
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
        if(input->group & PinGroup_SdCard)
            sdcard_detect(!DIGITAL_IN(SD_DETECT_PORT, SD_DETECT_BIT)); // TODO: add check for having same state as when isr were invoked?
#endif
    }

    EXTI->IMR1 |= input->bit; // Reenable pin interrupt
}

static inline void core_pin_irq (uint32_t bit)
{
    input_signal_t *input;

    if((input = pin_irq[__builtin_ffs(bit) - 1])) {
        if(input->mode.debounce && task_add_delayed(core_pin_debounce, input, 40)) {
            EXTI->IMR1 &= ~input->bit; // Disable pin interrupt
        } else
            core_pin_debounce(input);
    }
}

void aux_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
          DIGITAL_IN(input->port, input->bit) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
        ioports_event(input);

    EXTI->IMR1 |= input->bit; // Reenable pin interrupt
}

static inline void aux_pin_irq (uint32_t bit)
{
    input_signal_t *input;

    if((input = pin_irq[__builtin_ffs(bit) - 1]) && input->group == PinGroup_AuxInput) {
        if(input->mode.debounce && task_add_delayed(aux_pin_debounce, input, 40)) {
            EXTI->IMR1 &= ~input->bit; // Disable pin interrupt
#if SAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                debounce.safety_door = input->mode.debounce;
#endif
        } else
            ioports_event(input);
    }
}

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)

void EXTI0_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<0)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<0)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#elif AUXINPUT_MASK & (1<<0)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<0)
        uint32_t rpm_count = RPM_COUNTER->CNT;
        spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

        if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.counter.last_index = rpm_count;
        spindle_encoder.counter.index_count++;
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)

void EXTI1_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<1);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<1)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<1)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#elif AUXINPUT_MASK & (1<<1)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<1)
        uint32_t rpm_count = RPM_COUNTER->CNT;
        spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

        if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.counter.last_index = rpm_count;
        spindle_encoder.counter.index_count++;
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)

void EXTI2_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<2);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<2)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<2)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#elif AUXINPUT_MASK & (1<<2)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<2)
        uint32_t rpm_count = RPM_COUNTER->CNT;
        spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

        if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.counter.last_index = rpm_count;
        spindle_encoder.counter.index_count++;
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)

void EXTI3_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<3);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<3)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<3)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#elif AUXINPUT_MASK & (1<<3)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<3)
        uint32_t rpm_count = RPM_COUNTER->CNT;
        spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

        if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.counter.last_index = rpm_count;
        spindle_encoder.counter.index_count++;
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)

void EXTI4_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<4);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<4)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<4)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#elif AUXINPUT_MASK & (1<<4)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<4)
        uint32_t rpm_count = RPM_COUNTER->CNT;
        spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

        if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.counter.last_index = rpm_count;
        spindle_encoder.counter.index_count++;
#endif
    }
}

#endif

#if ((DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0)

void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0x03E0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if SPI_IRQ_BIT & 0x03E0
        if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#endif
#if SPINDLE_INDEX_BIT & 0x03E0
        if(ifg & SPINDLE_INDEX_BIT) {
            uint32_t rpm_count = RPM_COUNTER->CNT;
            spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

            if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                spindle_encoder.error_count++;

            spindle_encoder.counter.last_index = rpm_count;
            spindle_encoder.counter.index_count++;
        }
#endif
#if (LIMIT_MASK|SD_DETECT_BIT) & 0x03E0
        if(ifg & (LIMIT_MASK|SD_DETECT_BIT))
            core_pin_irq(ifg);
#endif
#if AUXINPUT_MASK & 0x03E0
        if(ifg & aux_irq)
            aux_pin_irq(ifg & aux_irq);
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (0xFC00)

void EXTI15_10_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0xFC00);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if SPI_IRQ_BIT & 0xFC00
        if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_BIT) == 0);
#endif
#if SPINDLE_INDEX_BIT & 0xFC00
        if(ifg & SPINDLE_INDEX_BIT) {
            uint32_t rpm_count = RPM_COUNTER->CNT;
            spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

            if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                spindle_encoder.error_count++;

            spindle_encoder.counter.last_index = rpm_count;
            spindle_encoder.counter.index_count++;
        }
#endif
#if QEI_ENABLE && ((QEI_A_BIT|QEI_B_BIT) & 0xFC00)
        if(ifg & (QEI_A_BIT|QEI_B_BIT))
            qei_update();
#endif
#if (LIMIT_MASK|SD_DETECT_BIT) & 0xFC00
        if(ifg & (LIMIT_MASK|SD_DETECT_BIT))
            core_pin_irq(ifg);
#endif
#if AUXINPUT_MASK & 0xFC00
        if(ifg & aux_irq)
            aux_pin_irq(ifg & aux_irq);
#endif
    }
}

#endif

// Interrupt handler for 1 ms interval timer
ISR_CODE void Driver_IncTick (void)
{
    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
