#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    int32_t key;
    uint32_t val_ind;
} HeapItem_t;


typedef struct {
    HeapItem_t *keys;
    void *vals;

    uint32_t size;
    uint32_t element_size;
    uint32_t max_size;    
} MinHeap_t;


void init_min_heap(MinHeap_t *heap, HeapItem_t *keys, void *vals, uint32_t element_size, uint32_t max_size);

bool min_heap_is_empty(MinHeap_t *heap);

bool min_heap_is_full(MinHeap_t *heap);

bool min_heap_insert(MinHeap_t *heap, int32_t key, void *item);

bool min_heap_pop(MinHeap_t *heap, int32_t *key, void *item);

void min_heap_print(MinHeap_t *heap);


#endif