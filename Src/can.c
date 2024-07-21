/*

  can.c - CAN bus driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2022-2024 Jon Escombe
  Copyright (c) 2024 Terje Io

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

#include "driver.h"
#include <stdio.h>

#ifdef CAN_PORT

#define CAN_MAX_FILTERS 32

#include "grbl/task.h"
#include "grbl/canbus.h"

static FDCAN_HandleTypeDef hfdcan1 = {
    .Instance = FDCAN1,
    .Init.FrameFormat = FDCAN_FRAME_CLASSIC,
    .Init.Mode = FDCAN_MODE_NORMAL,
    .Init.AutoRetransmission = DISABLE,
    .Init.TransmitPause = DISABLE,
    .Init.ProtocolException = DISABLE,
    .Init.MessageRAMOffset = 0,
    .Init.StdFiltersNbr = CAN_MAX_FILTERS,
    .Init.ExtFiltersNbr = CAN_MAX_FILTERS,
    .Init.RxFifo0ElmtsNbr = 4,
    .Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
    .Init.RxFifo1ElmtsNbr = 0,
    .Init.RxBuffersNbr = 0,
    .Init.TxEventsNbr = 0,
    .Init.TxBuffersNbr = 0,
    .Init.TxFifoQueueElmtsNbr = 4,
    .Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
    .Init.TxElmtSize = FDCAN_DATA_BYTES_8
};
static can_rx_enqueue_fn rx_enqueue;
static can_rx_ptr rx_callbacks[CAN_MAX_FILTERS] = {0};

bool can_put (canbus_message_t message, bool ext_id)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};

    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxHeader.IdType = ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    TxHeader.Identifier = message.id;

    // DLC has to be left shifted by 16bits for the FDCAN driver
    TxHeader.DataLength = message.len << 16;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, message.data) == HAL_OK;
}

bool can_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback)
{
    static uint8_t index = 0;

    static FDCAN_FilterTypeDef sFilterConfig = {
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    };

    if (index == CAN_MAX_FILTERS)
        return false;

    sFilterConfig.IdType = ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    sFilterConfig.FilterID1 = id;
    sFilterConfig.FilterID2 = mask;

    sFilterConfig.FilterIndex = index;

    //debug_printf("can_add_filter(), adding new filter - id:%lu, idx:%u\n", id, index);

    rx_callbacks[index++] = callback;

    return HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) == HAL_OK;
}

bool can_stop(void)
{
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);

    return true;
}

bool can_set_baud (uint32_t baud)
{
    // unable to stop/start the peripheral to change the baud rate, as filters/callbacks would be lost

    report_message("A hard reset of the controller is required after changing the CAN baud rate", Message_Info);

    return true;
}

bool can_start (uint32_t baud, can_rx_enqueue_fn callback)
{
    uint8_t unknown_rate = 0;

    rx_enqueue = callback;
    /*
     * Can bit time calculations taken from http://www.bittiming.can-wiki.info/ - pre-calculated
     * for supported CAN peripheral clock speeds and baud rates.
     *
     * Unable to query the FDCAN peripheral clock speed until it's initialised, so getting clock speed
     * directly from the RCC_PLL1_DIVQ clock source. This is 48MHz for all currently supported boards..
     */
    PLL1_ClocksTypeDef pll1_clocks;
    HAL_RCCEx_GetPLL1ClockFreq(&pll1_clocks);
    uint32_t pll1_q_freq = pll1_clocks.PLL1_Q_Frequency;
    debug_printf("can_start(), PLL1_Q frequency: %lu\n", pll1_q_freq);

    switch (pll1_q_freq) {

        case 48000000:
            /* FDCAN peripheral running with 48MHz clock, calculated bit timings for all supported baud rates */
            switch (baud) {

                case 125000:
                    hfdcan1.Init.NominalPrescaler = 24;
                    hfdcan1.Init.NominalTimeSeg1 = 13;
                    hfdcan1.Init.NominalTimeSeg2 = 2;
                    hfdcan1.Init.NominalSyncJumpWidth = 1;
                    break;

                case 250000:
                    hfdcan1.Init.NominalPrescaler = 12;
                    hfdcan1.Init.NominalTimeSeg1 = 13;
                    hfdcan1.Init.NominalTimeSeg2 = 2;
                    hfdcan1.Init.NominalSyncJumpWidth = 1;
                    break;

                case 500000:
                    hfdcan1.Init.NominalPrescaler = 6;
                    hfdcan1.Init.NominalTimeSeg1 = 13;
                    hfdcan1.Init.NominalTimeSeg2 = 2;
                    hfdcan1.Init.NominalSyncJumpWidth = 1;
                    break;

                case 1000000:
                    hfdcan1.Init.NominalPrescaler = 3;
                    hfdcan1.Init.NominalTimeSeg1 = 13;
                    hfdcan1.Init.NominalTimeSeg2 = 2;
                    hfdcan1.Init.NominalSyncJumpWidth = 1;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        default:
            /* unsupported CAN peripheral clock frequency */
            unknown_rate = 1;
            break;
    }

    if (unknown_rate) {
        debug_printf("can_start(), error - unable to calculate bit timings\n");
        return(0);
    }

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        return(0);
    }

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* Start the CAN peripheral (calls the MspInit function) */
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return(0);
    }

    /* Add the callback for received data */
    return HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK;
}

/**
* @brief FDCAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    if(hfdcan->Instance==FDCAN1)
    {
        __HAL_RCC_FDCAN_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF9_FDCAN1
        };

        HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

        static const periph_pin_t rx = {
            .function = Input_RX,
            .group = PinGroup_CAN,
            .port = CAN_PORT,
            .pin = CAN_RX_PIN,
            .mode = { .mask = PINMODE_NONE },
            .description = "CAN"
        };

        static const periph_pin_t tx = {
            .function = Output_TX,
            .group = PinGroup_CAN,
            .port = CAN_PORT,
            .pin = CAN_TX_PIN,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "CAN"
        };

        hal.periph_port.register_pin(&rx);
        hal.periph_port.register_pin(&tx);
    }
}

/**
* @brief FDCAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
    if(hfdcan->Instance==FDCAN1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_FDCAN_CLK_DISABLE();

        HAL_GPIO_DeInit(CAN_PORT, (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN));

        /* FDCAN1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    canbus_message_t message;
    FDCAN_RxHeaderTypeDef RxHeader;

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {

            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, message.data) == HAL_OK) {

                /* Sanity check the filter index.
                 *
                 * Unmatched messages can arrive via the global filter if not configured correctly.
                 * In this case FilterIndex is set to the maximum value(?), and rx_callbacks would
                 * point outside of the valid array..
                 */
                if (RxHeader.FilterIndex > CAN_MAX_FILTERS) {
                    debug_printf("HAL_FDCAN_RxFifo0Callback(), unexpected message received - id:%lu, idx:%lu\n",
                            RxHeader.Identifier, RxHeader.FilterIndex);
                    return;
                }

                /* Attempt to add incoming message to the RX queue.
                 *
                 * Note: not currently checking for success, if there is no space available we
                 * would just end up dropping messages somewhere else (i.e. in the CAN RX fifo)..
                 */

                //debug_printf("HAL_FDCAN_RxFifo0Callback(), adding new msg to RX queue - id:%lu, idx:%lu\n",
                //        RxHeader.Identifier, RxHeader.FilterIndex);

                message.id = RxHeader.Identifier;
                // DLC has to be right shifted by 16bits for the FDCAN driver
                message.len = RxHeader.DataLength >> 16;

                rx_enqueue(message, rx_callbacks[RxHeader.FilterIndex]);
            }
        }
    }
}

#endif // CAN_PORT
