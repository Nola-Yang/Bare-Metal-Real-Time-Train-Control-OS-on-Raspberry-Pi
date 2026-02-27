#ifndef _track_node_h_
#define _track_node_h_ 1

#include <stdint.h>

typedef struct track_node track_node;
typedef struct track_edge track_edge;

typedef enum {
    NODE_NONE   = 0,
    NODE_SENSOR = 1,
    NODE_BRANCH = 2,
    NODE_MERGE  = 3,
    NODE_ENTER  = 4,
    NODE_EXIT   = 5,
} node_type_t;

/* Sensor/merge/enter nodes use edge[DIR_AHEAD].
 * Branch nodes use edge[DIR_STRAIGHT] and edge[DIR_CURVED].
 * DIR_AHEAD == DIR_STRAIGHT == 0.
 */
enum {
    DIR_AHEAD    = 0,
    DIR_STRAIGHT = 0,
    DIR_CURVED   = 1,
};

struct track_edge {
    struct track_edge *reverse;
    struct track_node *src;
    struct track_node *dest;
    int dist;             
    int16_t time_factor_q8; /* 256=1.0x; 0=uninit.
                             * Learned per-edge: actual_dt/pred_dt ratio, EMA alpha=1/8.
                             * Compensates dead zones, curve friction, bad switches. */
};

struct track_node {
    const char *name;
    node_type_t type;
    int         num;        /* for branches: user switch number (1-18, 153-156) */
    track_node *reverse;    /* reverse-direction node at same physical location */
    track_edge  edge[2];    /* [0]=ahead/straight, [1]=curved */
};

#endif /* _track_node_h_ */
