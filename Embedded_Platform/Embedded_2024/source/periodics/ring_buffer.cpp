// ring_buffer.cpp
#include "ring_buffer.hpp"

// actual storage and head/tail indices
static TelemetryMsg         rb_buf[RB_SIZE];
static volatile uint16_t    rb_head = 0;
static volatile uint16_t    rb_tail = 0;

bool rb_push(const TelemetryMsg &msg) {
    uint16_t next = (rb_head + 1) & (RB_SIZE - 1);
    if (next == rb_tail) {
        // buffer is full
        return false;
    }
    rb_buf[rb_head] = msg;
    rb_head = next;
    return true;
}

bool rb_pop(TelemetryMsg &msg) {
    if (rb_head == rb_tail) {
        // buffer is empty
        return false;
    }
    msg = rb_buf[rb_tail];
    rb_tail = (rb_tail + 1) & (RB_SIZE - 1);
    return true;
}
