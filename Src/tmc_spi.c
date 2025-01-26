/*
  tmc_spi.c - SPI driver code for STM32H7xx processors

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io
  Copyright (c) 2021 fitch22
  Copyright (c) 2022-2023 Jon Escombe

  Some software serial code is ported from Arduino.  Credit belongs to the many
  authors that contributed to that project.

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

#include "driver.h"
#include "spi.h"

#if TRINAMIC_SPI_ENABLE

#include "trinamic/common.h"

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

// aproximates 1.5MHz SPI clock @ 480MHz processor speed
#ifndef TRIMANIC_SPI_DELAY
#define TRINAMIC_SPI_DELAY 45
#endif

// XXXXX replace with something better...
inline static void delay (uint32_t delay)
{
    volatile uint32_t dly = delay;

    while(--dly)
        __ASM volatile ("nop");
}

#ifdef TRINAMIC_SOFT_SPI // Software SPI implementation

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static uint8_t sw_spi_xfer (uint8_t byte)
{
  uint_fast8_t msk = 0x80, res = 0;

  DIGITAL_OUT(TRINAMIC_SCK_PORT, 1 << TRINAMIC_SCK_PIN, 0);

  do {
    DIGITAL_OUT(TRINAMIC_MOSI_PORT, 1 << TRINAMIC_MOSI_PIN, (byte & msk) != 0);
    msk >>= 1;
    delay(TRINAMIC_SPI_DELAY);
    res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, 1 << TRINAMIC_MISO_PIN);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, 1 << TRINAMIC_SCK_PIN, 1);
    delay(TRINAMIC_SPI_DELAY);
    if (msk)
      DIGITAL_OUT(TRINAMIC_SCK_PORT, 1 << TRINAMIC_SCK_PIN, 0);
  } while (msk);

  return (uint8_t)res;
}

#endif // TRINAMIC_SOFT_SPI

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if (gpio->group == PinGroup_MotorChipSelect)
      switch (gpio->function) {

        case Output_MotorChipSelectX:
            cs[X_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[X_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectY:
            cs[Y_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Y_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectZ:
            cs[Z_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Z_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM3:
            cs[3].port = (GPIO_TypeDef *)gpio->port;
            cs[3].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM4:
            cs[4].port = (GPIO_TypeDef *)gpio->port;
            cs[4].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM5:
            cs[5].port = (GPIO_TypeDef *)gpio->port;
            cs[5].pin = gpio->pin;
            break;

        default:
            break;
    }
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 0);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 1);
    delay(TRINAMIC_SPI_DELAY*2);
    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 0);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 1);
    delay(TRINAMIC_SPI_DELAY*2);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 0);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    DIGITAL_OUT(cs[driver.id].port, 1 << cs[driver.id].pin, 1);
    delay(TRINAMIC_SPI_DELAY*2);

    return status;
}

void if_init(uint8_t motors, axes_signals_t enabled)
{
  static bool init_ok = false;

  UNUSED(motors);

  if (!init_ok) {

#ifdef TRINAMIC_SOFT_SPI
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set all output pins: push-pull, no pull-up, slow
    GPIO_InitStruct.Pin = 1 << TRINAMIC_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRINAMIC_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 1 << TRINAMIC_SCK_PIN;
    HAL_GPIO_Init(TRINAMIC_SCK_PORT, &GPIO_InitStruct);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, 1 << TRINAMIC_SCK_PIN, 1);

    // Set the input pin: input with pull-up
    GPIO_InitStruct.Pin = 1 << TRINAMIC_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TRINAMIC_MISO_PORT, &GPIO_InitStruct);

    static const periph_pin_t sdi = {
        .function = Output_MOSI,
        .group = PinGroup_SPI,
        .port = TRINAMIC_MOSI_PORT,
        .pin = TRINAMIC_MOSI_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Motor"
    };

    static const periph_pin_t sdo = {
        .function = Input_MISO,
        .group = PinGroup_SPI,
        .port = TRINAMIC_MISO_PORT,
        .pin = TRINAMIC_MISO_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Motor"
    };

    static const periph_pin_t sck = {
        .function = Output_SCK,
        .group = PinGroup_SPI,
        .port = TRINAMIC_SCK_PORT,
        .pin = TRINAMIC_SCK_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Motor"
    };

    hal.periph_port.register_pin(&sdi);
    hal.periph_port.register_pin(&sdo);
    hal.periph_port.register_pin(&sck);

#else
    spi_init();
    spi_set_speed(SPI_BAUDRATEPRESCALER_32); // 48 MHz SPI clock / 32 = 1.5MHz
#endif //TRINAMIC_SOFT_SPI

    hal.enumerate_pins(true, add_cs_pin, NULL);
  }
}

#endif // TRINAMIC_SPI_ENABLE
