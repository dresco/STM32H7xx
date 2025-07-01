#pragma once

#include <stdint.h>

#define MODBUS_FRAME_MAX 256
#define MODBUS_LOG_FRAMES 16

typedef struct {
    uint16_t length;
    uint8_t data[MODBUS_FRAME_MAX];
} modbus_frame_t;

void modbus_log_init(void);
void modbus_log_tx(const uint8_t *data, uint16_t length);
int  modbus_log_get(modbus_frame_t *frame);

