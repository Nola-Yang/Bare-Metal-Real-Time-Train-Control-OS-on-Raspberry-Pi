#ifndef _route_search_internal_h_
#define _route_search_internal_h_

#include "train_tracking/route_priv.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include <stddef.h>
#include <stdint.h>

#define DIJK_INF 0x7FFFFFFF
#define ROUTE_BLOCKED_WORDS ((TRACK_MAX + 31) / 32)
#define ROUTE_HEAP_CAPACITY (TRACK_MAX * TRACK_MAX)
#define ROUTE_HEAP_TIE_BITS 8

typedef struct {
    uint16_t nodes[TRACK_MAX];
    int32_t  prefix_dist[TRACK_MAX];
    uint32_t node_bits[ROUTE_BLOCKED_WORDS];
    int16_t  first_sw_num;
    char     first_sw_dir;
    int      node_count;
    int      valid;
} route_segment_t;

extern route_segment_t g_route_segments[TRACK_MAX][2];

void route_pack_blocked_bits(const uint8_t *blocked,
                             uint32_t out_bits[ROUTE_BLOCKED_WORDS]);

int32_t route_free_track_dist_idx(int from_idx, int to_idx);

int route_search_leg_astar(int32_t *dist, int8_t *done, int16_t *prev,
                           int16_t *sw_num, char *sw_dir,
                           track_node *start, track_node *goal,
                           const uint8_t *blocked,
                           const uint32_t *blocked_bits,
                           const char *fixed_sw_dirs,
                           int32_t upper_bound_mm);

#endif /* _route_search_internal_h_ */
