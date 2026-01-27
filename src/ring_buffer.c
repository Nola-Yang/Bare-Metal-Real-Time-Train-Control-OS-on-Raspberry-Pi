#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ring_buffer.h"

static void init_ring_buffer(RingBuffer_t* buf, void *buffer, uint32_t element_size) {
    buf->buffer = buffer;
    buf->head = 0;
    buf->tail = 0;
    buf->size = RING_BUFFER_SIZE;
    buf->element_size = element_size;
    buf->count = 0;
}

void init_custom_ring_buffer(RingBuffer_t* buf, void *buffer, uint32_t size, uint32_t element_size) {
    init_ring_buffer(buf, buffer, element_size);
    buf->size = size;
}

static bool is_ring_buf_full(RingBuffer_t* buf) {
    return ((buf->head + 1) % buf->size) == buf->tail;
}

bool is_ring_buf_empty(RingBuffer_t* buf) {
    return buf->head == buf->tail;
}

bool ring_buf_append(RingBuffer_t* buf, void *item) {
    if (is_ring_buf_full(buf)) return false;

    void *target = (char *)buf->buffer + (buf->head * buf->element_size);
    memcpy(target, item, buf->element_size);

    buf->head = (buf->head + 1) % buf->size;
    buf->count += 1;
    return true;
}

bool ring_buf_pop_left(RingBuffer_t* buf, void* result) {
    if (is_ring_buf_empty(buf)) return false;

    void *found = (char *)buf->buffer + (buf->tail * buf->element_size);
    memcpy(result, found, buf->element_size);

    buf->tail = (buf->tail + 1) % buf->size;
    buf->count -= 1;
    return true;
}

bool ring_buf_from_tail(RingBuffer_t *buf, void *result, uint32_t ind, bool increasing) {
    if (ind >= buf->count) return false;

    int32_t newInd = (increasing) ? buf->tail + ind : buf->tail - ind - 1;
    newInd = ((newInd % (int32_t)buf->size) + buf->size) % buf->size;

    void *found = (char *)buf->buffer + (newInd * buf->element_size);
    memcpy(result, found, buf->element_size);

    return true;
}