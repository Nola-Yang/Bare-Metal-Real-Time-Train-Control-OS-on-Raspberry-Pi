#include "min_heap.h"
#include "util.h"
#include "uart.h"


void init_min_heap(MinHeap_t *heap, HeapItem_t *keys, void *vals, uint32_t element_size, uint32_t max_size) {
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

static int32_t get_parent_ind(uint32_t node_ind) {
    return (node_ind - 1) / 2;
}

static uint32_t get_left_child_ind(uint32_t node_ind) {
    return 2 * node_ind + 1;
}

static uint32_t get_right_child_ind(uint32_t node_ind) {
    return 2 * node_ind + 2;
}

static bool is_leaf(uint32_t node_ind, uint32_t heap_size) {
    return (get_left_child_ind(node_ind) >= heap_size && get_right_child_ind(node_ind) >= heap_size);
}

static void heap_fixup(MinHeap_t *heap, uint32_t node_ind) {
    uint32_t parent_ind = get_parent_ind(node_ind);

    while (node_ind > 0 && heap->keys[parent_ind].key > heap->keys[node_ind].key) {
        swap(heap->keys + parent_ind, heap->keys + node_ind, sizeof(HeapItem_t));

        node_ind = parent_ind;
        if (node_ind) {
            parent_ind = get_parent_ind(node_ind);
        }
    }
}

static void heap_fixdown(MinHeap_t *heap, uint32_t node_ind) {
    uint32_t heap_size = heap->size;
    uint32_t smaller_child_ind;
    uint32_t other_child_ind;
    HeapItem_t *keys = heap->keys;
    HeapItem_t *node;
    HeapItem_t *child_node;

    while (!is_leaf(node_ind, heap_size)) {
        smaller_child_ind = get_left_child_ind(node_ind);
        other_child_ind = get_right_child_ind(node_ind);

        node = &(keys[node_ind]);

        if (other_child_ind < heap_size && keys[other_child_ind].key < keys[smaller_child_ind].key) {
            smaller_child_ind = other_child_ind;
        }

        child_node = &(keys[smaller_child_ind]);
        if (node->key <= child_node->key) break;

        swap(node, child_node, sizeof(HeapItem_t));
        node_ind = smaller_child_ind;
    }
}

bool min_heap_insert(MinHeap_t *heap, int32_t key, void *item) {
    if (min_heap_is_full(heap)) return false;

    uint32_t insert_ind = heap->size;
    void *val_target = (char *)heap->vals + (insert_ind * heap->element_size);
    HeapItem_t heap_key = {key, insert_ind};

    memcpy(val_target, item, heap->element_size);
    memcpy(&(heap->keys[insert_ind]), &heap_key, sizeof(HeapItem_t));

    heap->size++;
    heap_fixup(heap, insert_ind);
    return true;
}

bool min_heap_pop(MinHeap_t *heap, int32_t *key, void *item) {
    if (min_heap_is_empty(heap)) return false;

    HeapItem_t *root_key = heap->keys;
    void *val_target = (char *)heap->vals + (root_key->val_ind * heap->element_size);

    memcpy(item, val_target, heap->element_size);
    *key = root_key->key;

    HeapItem_t *last_key = heap->keys + (heap->size - 1);
    memcpy(root_key, last_key, sizeof(HeapItem_t));
    
    heap->size--;
    heap_fixdown(heap, 0);
    return true;
}