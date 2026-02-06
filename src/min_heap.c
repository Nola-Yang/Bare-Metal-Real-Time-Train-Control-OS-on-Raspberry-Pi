#include "min_heap.h"
#include "util.h"
#include "uart.h"


void init_min_heap(MinHeap_t *heap, uint32_t *keys, void *vals, uint32_t element_size, uint32_t max_size) {
    heap->keys = keys;
    heap->vals = vals;
    heap->size = 0;
    heap->element_size = element_size;
    heap->max_size = max_size;
}

bool min_heap_is_empty(MinHeap_t *heap) {
    return heap->size <= 0;
}

bool min_heap_is_full(MinHeap_t *heap) {
    return heap->size >= heap->max_size;
}

// get_parent_id: Retrieves the index of the parent in the heap
static int32_t get_parent_ind(uint32_t node_ind) {
    return (node_ind - 1) / 2;
}

// get_left_child_ind: Retrieves the index of the left child in a heap
static uint32_t get_left_child_ind(uint32_t node_ind) {
    return 2 * node_ind + 1;
}

// get_left_child_ind: Retrieves the index of the right child in a heap
static uint32_t get_right_child_ind(uint32_t node_ind) {
    return 2 * node_ind + 2;
}

// is_leaf: Determines whether a node in the heap is a leaf
static bool is_leaf(uint32_t node_ind, uint32_t heap_size) {
    return (get_left_child_ind(node_ind) >= heap_size && get_right_child_ind(node_ind) >= heap_size);
}

// heap_fixup: Fixes the heap after an insertion
static void heap_fixup(MinHeap_t *heap, uint32_t node_ind) {
    uint32_t parent_ind = get_parent_ind(node_ind);
    void *parent_val;
    void *node_val;

    while (node_ind > 0 && heap->keys[parent_ind] > heap->keys[node_ind]) {
        node_val = (char *)heap->vals + (node_ind * heap->element_size);
        parent_val = (char *)heap->vals + (parent_ind * heap->element_size);

        swap(heap->keys + parent_ind, heap->keys + node_ind, sizeof(uint32_t));
        swap(parent_val, node_val, heap->element_size);

        node_ind = parent_ind;
        if (node_ind) {
            parent_ind = get_parent_ind(node_ind);
        }
    }
}

// heap_fixdown: Fixes the heap after a deletion
static void heap_fixdown(MinHeap_t *heap, uint32_t node_ind) {
    uint32_t heap_size = heap->size;
    uint32_t smaller_child_ind;
    uint32_t other_child_ind;

    uint32_t *keys = heap->keys;

    uint32_t node_key;
    uint32_t child_node_key;

    void *node_val;
    void *child_val;

    while (!is_leaf(node_ind, heap_size)) {
        smaller_child_ind = get_left_child_ind(node_ind);
        other_child_ind = get_right_child_ind(node_ind);

        if (other_child_ind < heap_size && keys[other_child_ind] < keys[smaller_child_ind]) {
            smaller_child_ind = other_child_ind;
        }

        node_key = keys[node_ind];
        child_node_key = keys[smaller_child_ind];

        if (node_key <= child_node_key) break;

        node_val = (char *)heap->vals + (node_ind * heap->element_size);
        child_val = (char *)heap->vals + (smaller_child_ind * heap->element_size);

        swap(keys + node_ind, keys + smaller_child_ind, sizeof(uint32_t));
        swap(node_val, child_val, heap->element_size);

        node_ind = smaller_child_ind;
    }
}

bool min_heap_insert(MinHeap_t *heap, uint32_t key, void *item) {
    if (min_heap_is_full(heap)) return false;

    uint32_t insert_ind = heap->size;
    void *val_target = (char *)heap->vals + (insert_ind * heap->element_size);

    heap->keys[insert_ind] = key;
    memcpy(val_target, item, heap->element_size);

    heap->size++;
    heap_fixup(heap, insert_ind);
    return true;
}

bool min_heap_pop(MinHeap_t *heap, uint32_t *key, void *item) {
    if (min_heap_is_empty(heap)) return false;

    uint32_t pop_ind = 0;
    *key = heap->keys[pop_ind];
    memcpy(item, heap->vals, heap->element_size);

    uint32_t last_ind = heap->size - 1;
    heap->keys[pop_ind] = heap->keys[last_ind];
    void *last_val = (char *)heap->vals + (last_ind * heap->element_size);
    memcpy(heap->vals, last_val, heap->element_size);

    heap->size--;
    heap_fixdown(heap, 0);
    return true;
}

uint32_t min_heap_get_top_key(MinHeap_t *heap) {
    return heap->keys[0];
}