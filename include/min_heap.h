#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <stdint.h>
#include <stdbool.h>


// MinHeap: Strcture for a min heap
typedef struct {
    uint32_t *keys;
    void *vals;

    uint32_t size;
    uint32_t element_size;
    uint32_t max_size;    
} MinHeap_t;


// init_min_heap: Constructs a new heap
void init_min_heap(MinHeap_t *heap, uint32_t *keys, void *vals, uint32_t element_size, uint32_t max_size);

// min_heap_is_empty: Determines whether the heap is empty
bool min_heap_is_empty(MinHeap_t *heap);

// min_heap_is_full: Determines whether the heap is full
bool min_heap_is_full(MinHeap_t *heap);

// min_heap_insert: Inserts a KVP into the heap
bool min_heap_insert(MinHeap_t *heap, uint32_t key, void *item);

// min_heap_pop: Pops the smallest item from the heap
bool min_heap_pop(MinHeap_t *heap, uint32_t *key, void *item);

// min_heap_get_top_key: Retrives the smallest key
uint32_t min_heap_get_top_key(MinHeap_t *heap);


#endif