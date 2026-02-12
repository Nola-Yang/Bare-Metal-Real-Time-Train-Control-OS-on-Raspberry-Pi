#ifndef _ring_buffer_h_
#define _ring_buffer_h_ 1

#include <stdint.h>

#define RING_BUFFER_SIZE 256

// Todo: reuse this ring buffer for other modules

typedef struct {
    char data[RING_BUFFER_SIZE];
    int head;  
    int tail;  
    int count; 
} RingBuffer_t;

static inline void ring_buffer_init(RingBuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

static inline int ring_buffer_is_empty(RingBuffer_t *rb) {
    return rb->count == 0;
}

static inline int ring_buffer_is_full(RingBuffer_t *rb) {
    return rb->count == RING_BUFFER_SIZE;
}

static inline int ring_buffer_count(RingBuffer_t *rb) {
    return rb->count;
}

static inline int ring_buffer_free_space(RingBuffer_t *rb) {
    return RING_BUFFER_SIZE - rb->count;
}

// Put a character into the buffer. Returns 0 on success, -1 if full.
static inline int ring_buffer_put(RingBuffer_t *rb, char c) {
    if (rb->count >= RING_BUFFER_SIZE) {
        return -1;
    }
    rb->data[rb->head] = c;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    rb->count++;
    return 0;
}

// Get a character from the buffer. Returns the char on success, -1 if empty.
static inline int ring_buffer_get(RingBuffer_t *rb) {
    if (rb->count == 0) {
        return -1;
    }
    char c = rb->data[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    rb->count--;
    return (unsigned char)c;
}

// Peek at the next character without removing it. Returns -1 if empty.
static inline int ring_buffer_peek(RingBuffer_t *rb) {
    if (rb->count == 0) {
        return -1;
    }
    return (unsigned char)rb->data[rb->tail];
}

#endif /* _ring_buffer_h_ */
