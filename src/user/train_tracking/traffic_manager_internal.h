#ifndef _traffic_manager_internal_h_
#define _traffic_manager_internal_h_

#include "track.h"
#include <stddef.h>

void traffic_reservation_init(void);
void traffic_attr_init(void);

static inline int traffic_node_index(track_node *n) {
    if (!n) return -1;
    int idx = (int)(n - g_track);
    return (idx >= 0 && idx < TRACK_MAX) ? idx : -1;
}

static inline int traffic_reverse_index(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return -1;
    track_node *r = g_track[idx].reverse;
    return r ? traffic_node_index(r) : -1;
}

static inline track_edge *traffic_tm_get_next_edge(track_node *n) {
    if (!n) return NULL;
    switch (n->type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
        return &n->edge[DIR_AHEAD];
    case NODE_BRANCH: {
        int sw_idx = track_switch_to_index(n->num);
        if (sw_idx < 0) return NULL;
        char st = track_get_switch_state()[sw_idx].state;
        if (st != 'S' && st != 'C') return NULL;
        return &n->edge[(st == 'C') ? DIR_CURVED : DIR_STRAIGHT];
    }
    default:
        return NULL;
    }
}

#endif /* _traffic_manager_internal_h_ */
