#ifndef _ring_buffer_h_
#define _ring_buffer_h_ 1

#include <stddef.h>

// Generic ring buffer helpers for structs with:
//   data[N], head, tail, count
// notice: consider data race for kernel usage

#define RING_BUFFER_DECLARE(name, type, size) \
    typedef struct {                           \
        type data[size];                       \
        int head;                              \
        int tail;                              \
        int count;                             \
    } name

#define RING_BUFFER_CAPACITY(rb) \
    ((int)(sizeof((rb)->data) / sizeof((rb)->data[0])))

#define RING_BUFFER_INIT(rb)               \
    do {                                   \
        (rb)->head = 0;                    \
        (rb)->tail = 0;                    \
        (rb)->count = 0;                   \
    } while (0)

#define RING_BUFFER_IS_EMPTY(rb) ((rb)->count == 0)
#define RING_BUFFER_IS_FULL(rb)  ((rb)->count >= RING_BUFFER_CAPACITY(rb))
#define RING_BUFFER_COUNT(rb)    ((rb)->count)
#define RING_BUFFER_FREE_SPACE(rb) (RING_BUFFER_CAPACITY(rb) - (rb)->count)

// Returns 0 on success, -1 if full.
#define RING_BUFFER_PUT(rb, value)                                       \
    (RING_BUFFER_IS_FULL(rb)                                             \
         ? -1                                                            \
         : ((rb)->data[(rb)->head] = (value),                            \
            (rb)->head = ((rb)->head + 1) % RING_BUFFER_CAPACITY(rb),    \
            (rb)->count++,                                               \
            0))

// Returns 0 on success, -1 if empty. Writes to out_ptr on success.
#define RING_BUFFER_GET(rb, out_ptr)                                     \
    (RING_BUFFER_IS_EMPTY(rb)                                            \
         ? -1                                                            \
         : (*(out_ptr) = (rb)->data[(rb)->tail],                         \
            (rb)->tail = ((rb)->tail + 1) % RING_BUFFER_CAPACITY(rb),    \
            (rb)->count--,                                               \
            0))

// Returns 0 on success, -1 if empty. Writes to out_ptr on success.
#define RING_BUFFER_PEEK(rb, out_ptr)                                    \
    (RING_BUFFER_IS_EMPTY(rb)                                            \
         ? -1                                                            \
         : (*(out_ptr) = (rb)->data[(rb)->tail], 0))

// Default char ring buffer 
#define RING_BUFFER_SIZE 256
RING_BUFFER_DECLARE(RingBuffer_t, char, RING_BUFFER_SIZE);

#define ring_buffer_init(rb) RING_BUFFER_INIT(rb)
#define ring_buffer_is_empty(rb) RING_BUFFER_IS_EMPTY(rb)
#define ring_buffer_is_full(rb) RING_BUFFER_IS_FULL(rb)
#define ring_buffer_count(rb) RING_BUFFER_COUNT(rb)
#define ring_buffer_free_space(rb) RING_BUFFER_FREE_SPACE(rb)
#define ring_buffer_put(rb, value) RING_BUFFER_PUT(rb, value)
#define ring_buffer_get(rb, out_ptr) RING_BUFFER_GET(rb, out_ptr)
#define ring_buffer_peek(rb, out_ptr) RING_BUFFER_PEEK(rb, out_ptr)

#endif /* _ring_buffer_h_ */
