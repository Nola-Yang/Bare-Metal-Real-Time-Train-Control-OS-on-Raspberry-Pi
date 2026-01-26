#ifndef _ring_buffer_h_
#define _ring_buffer_h_ 1

#include <stdint.h>
#include <stdbool.h>

// Default size of the ring buffer
#define RING_BUFFER_SIZE 256


// RingBuffer: Structure for a Ring Buffer
typedef struct {
    uint32_t head;
    uint32_t tail;
    void *buffer;

    uint32_t size;
    uint32_t element_size;
    uint32_t count;
} RingBuffer_t;


// init_custom_ring_buffer(buf, buffer, size, element_size): Constructs a new Ring Buffer with a custom size
void init_custom_ring_buffer(RingBuffer_t* buf, void *buffer, uint32_t size, uint32_t element_size);

// is_ring_buf_empty(buf): Determines whether the ring buffer is empty
bool is_ring_buf_empty(RingBuffer_t* buf);

// ring_buf_append(buf, item): Pushes an element to the right of the ring buffer
bool ring_buf_append(RingBuffer_t* buf, void *item);

// ring_buf_pop_left(buf, result): Pops an element from the left of the ring buffer
bool ring_buf_pop_left(RingBuffer_t* buf, void* result);

// ring_buf_from_tail(buf, result, ind, increasing): Retrieves a particular element from the
//  ring buffer starting from the left of the ring buffer
bool ring_buf_from_tail(RingBuffer_t *buf, void *result, uint32_t ind, bool increasing);

#endif
