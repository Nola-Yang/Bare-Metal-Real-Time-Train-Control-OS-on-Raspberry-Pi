#include "train_tracking/position_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "train_tracking/track_data.h"
#include "ui.h"
#include <stddef.h>
#include <stdint.h>

void pos_on_sensor_trigger(uint16_t sensor_id, uint64_t time_us) {
    int track_idx = (int)sensor_id - 1;
    if (track_idx < 0 || track_idx >= TRACK_MAX) return;

    track_node *hit = &g_track[track_idx];
    if (hit->type != NODE_SENSOR) return;

    traffic_attr_result_t attr = traffic_attribute_sensor(hit, time_us);
    if (!attr.owner) return;

    train_pos_t *owner = attr.owner;
    if (attr.revive_dead_track) {
        pos_revive_dead_track_for_current_hit(owner);
    }

    /* If the train took the alt-direction branch, correct the stored switch state. */
    if (pos_hit_matches_alt_branch(owner, hit) && owner->pred.branch_node != NULL) {
        int sw_idx = track_switch_to_index(owner->pred.branch_node->num);
        if (sw_idx >= 0) {
            char stored = track_get_switch_state()[sw_idx].state;
            char actual = (stored == 'S') ? 'C' : 'S';
            track_update_switch(owner->pred.branch_node->num, actual);
            ui_mark_switches_dirty();
        }
    }

    pos_handle_sensor_hit(owner, hit, time_us);
}
