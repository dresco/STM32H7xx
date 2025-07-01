#include "modbus_log.h"
#include <string.h>

static struct {
    uint8_t head;
    uint8_t tail;
    modbus_frame_t frames[MODBUS_LOG_FRAMES];
} logbuf;

void modbus_log_init(void)
{
    logbuf.head = logbuf.tail = 0;
}

void modbus_log_tx(const uint8_t *data, uint16_t length)
{
    if(!data || length == 0)
        return;

    modbus_frame_t *frame = &logbuf.frames[logbuf.head];
    frame->length = length > MODBUS_FRAME_MAX ? MODBUS_FRAME_MAX : length;
    memcpy(frame->data, data, frame->length);
    logbuf.head = (logbuf.head + 1) % MODBUS_LOG_FRAMES;
    if(logbuf.head == logbuf.tail)
        logbuf.tail = (logbuf.tail + 1) % MODBUS_LOG_FRAMES;
}

int modbus_log_get(modbus_frame_t *frame)
{
    if(logbuf.tail == logbuf.head || !frame)
        return 0;

    *frame = logbuf.frames[logbuf.tail];
    logbuf.tail = (logbuf.tail + 1) % MODBUS_LOG_FRAMES;
    return frame->length;
}
