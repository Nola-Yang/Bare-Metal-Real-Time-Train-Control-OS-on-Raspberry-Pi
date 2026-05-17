#include "train_tracking/position_priv.h"
#include <stddef.h>
#include <stdint.h>

#define POS_GAME_EVENT_CAP 64

static pos_game_event_t g_game_events[POS_GAME_EVENT_CAP];
static uint32_t g_game_event_next_seq = 1;
static uint32_t g_game_event_oldest_seq = 1;

static uint16_t pos_game_sensor_num(track_node *node) {
    if (!node || node->type != NODE_SENSOR) return 0;
    if (node->num < 0) return 0;
    return (uint16_t)(node->num + 1);
}

static void pos_publish_game_event(pos_game_event_type_t type,
                                   int train_num,
                                   track_node *sensor,
                                   uint64_t time_us) {
    uint32_t seq;
    int slot;

    if (type == POS_GAME_EVENT_NONE) return;
    if (train_num < 0) return;

    seq = g_game_event_next_seq++;
    slot = (int)(seq % POS_GAME_EVENT_CAP);
    g_game_events[slot].seq = seq;
    g_game_events[slot].type = type;
    g_game_events[slot].train_num = train_num;
    g_game_events[slot].sensor_num = pos_game_sensor_num(sensor);
    g_game_events[slot].time_us = time_us;

    if (g_game_event_next_seq - g_game_event_oldest_seq > POS_GAME_EVENT_CAP) {
        g_game_event_oldest_seq = g_game_event_next_seq - POS_GAME_EVENT_CAP;
    }
}

void pos_reset_game_events(void) {
    for (int i = 0; i < POS_GAME_EVENT_CAP; i++) {
        g_game_events[i].seq = 0;
        g_game_events[i].type = POS_GAME_EVENT_NONE;
        g_game_events[i].train_num = -1;
        g_game_events[i].sensor_num = 0;
        g_game_events[i].time_us = 0;
    }
    g_game_event_next_seq = 1;
    g_game_event_oldest_seq = 1;
}

void pos_publish_game_sensor_hit(train_pos_t *pos, track_node *hit, uint64_t time_us) {
    if (!pos) return;
    pos_publish_game_event(POS_GAME_EVENT_SENSOR_HIT, pos->train_num, hit, time_us);
}

void pos_publish_game_goal_stop(train_pos_t *pos, track_node *target, uint64_t time_us) {
    if (!pos) return;
    pos_publish_game_event(POS_GAME_EVENT_GOAL_STOP, pos->train_num, target, time_us);
}

int pos_read_game_events(uint32_t *io_seq, pos_game_event_t *out, int max_events) {
    uint32_t seq;
    int count = 0;

    if (!io_seq || !out || max_events <= 0) return 0;

    seq = *io_seq;
    if (seq + 1 < g_game_event_oldest_seq) {
        seq = g_game_event_oldest_seq - 1;
    }

    while (seq + 1 < g_game_event_next_seq && count < max_events) {
        uint32_t next_seq = seq + 1;
        pos_game_event_t event = g_game_events[next_seq % POS_GAME_EVENT_CAP];
        if (event.seq != next_seq) {
            seq = next_seq;
            continue;
        }
        out[count++] = event;
        seq = next_seq;
    }

    *io_seq = seq;
    return count;
}
