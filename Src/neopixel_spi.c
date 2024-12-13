/*
  neopixel_spi.c - SPI support for Neopixels, non-blocking

  TODO: use I2S interface for more precise timing?

  Part of grblHAL driver for STM32H7xx

  Copyright (c) 2024 Terje Io, Jon Escombe

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "driver.h"
#include "cache.h"

#ifdef NEOPIXEL_SPI

#ifndef NEOPIXELS_NUM
#define NEOPIXELS_NUM 1
#endif

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

#define DMAirq(d, p) DMAirqI(d, p)
#define DMAirqI(d, p) DMA ## d ## _Stream ## p ## _IRQn

#define DMAhandler(d, p) DMAhandlerI(d, p)
#define DMAhandlerI(d, p) DMA ## d ## _Stream ## p ## _IRQHandler

#define DMAStream(d, p) DMAStreamI(d, p)
#define DMAStreamI(d, p) DMA ## d ## _Stream ## p

#define DMARequestTX(p) DMARequestTXI(p)
#define DMARequestTXI(p) DMA_REQUEST_SPI ## p ## _TX

#define SPIirq(d) SPIirqI(d)
#define SPIirqI(d) SPI ## d ## _IRQn

#define SPIHandler(d) SPIHandlerI(d)
#define SPIHandlerI(d) SPI ## d ## _IRQHandler

#if NEOPIXEL_SPI == 11 || NEOPIXEL_SPI == 12
#define SPIPORT SPIport(1)
#define SPI_IRQ SPIirq(1)
#define SPI_IRQ_HANDLER SPIHandler(1)
#define DMA_REQUEST_TX DMARequestTX(1)
#else
#define SPIPORT SPIport(NEOPIXEL_SPI)
#define SPI_IRQ SPIirq(NEOPIXEL_SPI)
#define SPI_IRQ_HANDLER SPIHandler(NEOPIXEL_SPI)
#define DMA_REQUEST_TX DMARequestTX(NEOPIXEL_SPI)
#endif

// H7 has a DMAMUX, so using the same DMA stream (DMA1_Stream7) regardless of SPI port
#define DMA_STREAM DMAStream(1,7)
#define DMA_TX_IRQ DMAirq(1, 7)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 7)

static SPI_HandleTypeDef spi_port = {
    .Instance               = SPIPORT,
    .Init.Mode              = SPI_MODE_MASTER,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
    .Init.Direction         = SPI_DIRECTION_2LINES_TXONLY,
    .Init.CLKPhase          = SPI_PHASE_1EDGE,
    .Init.CLKPolarity       = SPI_POLARITY_LOW,
    .Init.DataSize          = SPI_DATASIZE_8BIT,
    .Init.FirstBit          = SPI_FIRSTBIT_MSB,
    .Init.TIMode            = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial     = 7,
    .Init.CRCLength         = SPI_CRC_LENGTH_8BIT,
    .Init.NSS               = SPI_NSS_SOFT,
    .Init.NSSPMode          = SPI_NSS_PULSE_DISABLE,
    .Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE,  /* Recommended setting to avoid glitches */
};

static DMA_HandleTypeDef spi_dma_tx = {

    .Instance                 = DMA_STREAM,
    .Init.Request             = DMA_REQUEST_TX,
    .Init.FIFOMode            = DMA_FIFOMODE_DISABLE,
    .Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
    .Init.MemBurst            = DMA_MBURST_INC4,
    .Init.PeriphBurst         = DMA_PBURST_INC4,
    .Init.Direction           = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc           = DMA_PINC_DISABLE,
    .Init.MemInc              = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
    .Init.Mode                = DMA_NORMAL,
    .Init.Priority            = DMA_PRIORITY_LOW,
};

neopixel_cfg_t neopixel = { .intensity = 255 };
static settings_changed_ptr settings_changed;

static inline void _write (void)
{
    while(spi_port.State == HAL_SPI_STATE_BUSY_TX);

#if USE_SPI_DMA

#if L1_CACHE_ENABLE
    SCB_CleanDCache_by_Addr((uint32_t *)neopixel.leds, neopixel.num_bytes);
#endif
    __DSB(); // needed?
    HAL_SPI_Transmit_DMA(&spi_port, neopixel.leds, neopixel.num_bytes);
#else
    HAL_SPI_Transmit(&spi_port, neopixel.leds, neopixel.num_bytes, 1000);
#endif
}

void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_3bpp_pack(&neopixel.leds[device * 9], color, mask, neopixel.intensity);

        if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_3bpp_unpack(& neopixel.leds[device * 9], prev);
                neopixel_out(device, color);
            } while(device);

            if(neopixel.num_leds != 1)
                _write();
        }
    }

    return prev;
}

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0) {

        if(settings->rgb_strip.length0 == 0)
            settings->rgb_strip.length0 = hal.rgb0.num_devices;
        else
            hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(neopixel.leds) {
            cache_aligned_free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 9 + 24;
            if((neopixel.leds = cache_aligned_calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb0.num_devices;
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

void neopixel_init (void)
{
    static bool init = false;

    // Note: SCK pin is not used for neopixels, but the GPIO must be configured
    // for EOT interrupt to fire after DMA transmission.

    if(!init) {

#if NEOPIXEL_SPI == 1

        __HAL_RCC_SPI1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN5|GPIO_PIN_7,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 5,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 7,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 11

        __HAL_RCC_SPI1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // SCK
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // MOSI

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 5,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 12

        __HAL_RCC_SPI1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin =  GPIO_PIN_3|GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 3,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 2

        __HAL_RCC_SPI2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_13|GPIO_PIN_15,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI2,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 15,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 3

        __HAL_RCC_SPI3_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_10|GPIO_PIN_12,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF6_SPI3
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 10,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 4
        __HAL_RCC_SPI4_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_12|GPIO_PIN_14,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI4
        };
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 12,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 14,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };
#endif

#ifdef USE_SPI_DMA
       __HAL_RCC_DMA1_CLK_ENABLE();
        HAL_DMA_Init(&spi_dma_tx);
        __HAL_LINKDMA(&spi_port, hdmatx, spi_dma_tx);

        HAL_NVIC_SetPriority(DMA_TX_IRQ, 1, 1);
        HAL_NVIC_EnableIRQ(DMA_TX_IRQ);
#endif

        HAL_NVIC_SetPriority(SPI_IRQ, 1, 0);
        HAL_NVIC_EnableIRQ(SPI_IRQ);

        HAL_SPI_Init(&spi_port);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);

        hal.rgb0.out = neopixel_out;
        hal.rgb0.out_masked = neopixel_out_masked;
        hal.rgb0.set_intensity = neopixels_set_intensity;
        hal.rgb0.write = neopixels_write;
        hal.rgb0.flags = (rgb_properties_t){ .is_strip = On };
        hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        init = true;
    }
}

void DMA_TX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_tx);
}

void SPI_IRQ_HANDLER(void)
{
    HAL_SPI_IRQHandler(&spi_port);
}

#endif // NEOPIXEL_SPI
