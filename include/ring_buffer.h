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


// init_ring_buffer(buf, buffer, element_size): Constructs a new Ring Buffer with the default size
void init_ring_buffer(RingBuffer_t* buf, void *buffer, uint32_t element_size);

// init_custom_ring_buffer(buf, buffer, size, element_size): Constructs a new Ring Buffer with a custom size
void init_custom_ring_buffer(RingBuffer_t* buf, void *buffer, uint32_t size, uint32_t element_size);

// ring_buf_clear(buf): Clears the ring buffer
void ring_buf_clear(RingBuffer_t *buf);

// is_ring_buf_full(buf): Determines whether the ring buffer is full
bool is_ring_buf_full(RingBuffer_t* buf);

// is_ring_buf_empty(buf): Determines whether the ring buffer is empty
bool is_ring_buf_empty(RingBuffer_t* buf);

// ring_buf_append(buf, item): Pushes an element to the right of the ring buffer
bool ring_buf_append(RingBuffer_t* buf, void *item) ;

// ring_buf_append_left(buf, item): Pushes an element to the left of the ring buffer
bool ring_buf_append_left(RingBuffer_t* buf, void *item);

// ring_buf_pop(buf, result): Pops an element from the right of the ring buffer
bool ring_buf_pop(RingBuffer_t* buf, void* result);

// ring_buf_pop_left(buf, result): Pops an element from the left of the ring buffer
bool ring_buf_pop_left(RingBuffer_t* buf, void* result);

// ring_buf_from_head(buf, result, ind, decreasing): Retrieves a particular element from the
//  ring buffer starting from the right of the ring buffer
bool ring_buf_from_head(RingBuffer_t *buf, void *result, uint32_t ind, bool decreasing);

// ring_buf_from_tail(buf, result, ind, decreasing): Retrieves a particular element from the
//  ring buffer starting from the left of the ring buffer
bool ring_buf_from_tail(RingBuffer_t *buf, void*result, uint32_t ind, bool increasing);

#endif
