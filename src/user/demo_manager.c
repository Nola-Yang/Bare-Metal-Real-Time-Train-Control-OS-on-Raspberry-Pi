#include "demo_manager.h"
#include "train_tracking/position.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "track.h"
#include "ui.h"
#include "util.h"
#include "timer.h"
#include <stdint.h>

#define DEMO_MAX_TRAINS 4
#define GOLD_WAIT_RETARGET_US 8000000ULL

typedef enum {
    DEMO_MODE_OFF = 0,
    DEMO_MODE_GOLD = 1,
} demo_mode_t;

typedef enum {
    DEMO_RUN_IDLE = 0,
    DEMO_RUN_STARTING = 1,
    DEMO_RUN_RUNNING = 2,
    DEMO_RUN_STOPPING = 3,
    DEMO_RUN_FAILED = 4,
} demo_run_state_t;

typedef struct {
    int enabled;
    int train_num;
    int started;
    int last_target_idx;
    uint32_t missions_completed;
    uint32_t wait_resource_count;
    uint32_t dead_track_count;
    train_route_state_t last_seen_state;
    uint64_t wait_enter_us;
} demo_train_slot_t;

static demo_mode_t g_demo_mode = DEMO_MODE_OFF;
static demo_run_state_t g_demo_state = DEMO_RUN_IDLE;
static uint64_t g_demo_start_us = 0;
static uint64_t g_demo_stop_request_us = 0;
static uint32_t g_demo_seed = 1;
static uint32_t g_demo_rng_state = 1;

static int g_gold_min_trip_mm = 1400;

static demo_train_slot_t g_slots[DEMO_MAX_TRAINS];

static track_node *g_sensor_pool[TRACK_MAX];
static int g_sensor_pool_count = 0;

static int parse_int_token_local(const char *tok, int *out) {
    if (!tok || !tok[0] || !out) return 0;
    const char *p = tok;
    if (*p == '+' || *p == '-') p++;
    if (!*p) return 0;
    while (*p) {
        if (*p < '0' || *p > '9') return 0;
        p++;
    }
    *out = str2int(tok);
    return 1;
}

static void ui_put_u32(uint32_t v) {
    char b[16];
    ui2a(v, 10, b);
    ui_puts(b);
}

static void ui_put_i32(int v) {
    char b[16];
    i2a(v, b);
    ui_puts(b);
}

static const char *demo_mode_str(demo_mode_t m) {
    switch (m) {
    case DEMO_MODE_OFF:   return "OFF";
    case DEMO_MODE_GOLD:  return "GOLD";
    default:              return "UNK";
    }
}

static const char *demo_state_str(demo_run_state_t s) {
    switch (s) {
    case DEMO_RUN_IDLE:     return "IDLE";
    case DEMO_RUN_STARTING: return "STARTING";
    case DEMO_RUN_RUNNING:  return "RUNNING";
    case DEMO_RUN_STOPPING: return "STOPPING";
    case DEMO_RUN_FAILED:   return "FAILED";
    default:                return "UNK";
    }
}

static const char *route_state_str(train_route_state_t s) {
    switch (s) {
    case TRAIN_STATE_UNKNOWN:           return "UNK";
    case TRAIN_STATE_KNOWN:             return "KNW";
    case TRAIN_STATE_STOPPING_TR:       return "STR";
    case TRAIN_STATE_ON_ROUTE:          return "RTE";
    case TRAIN_STATE_STOPPING:          return "STP";
    case TRAIN_STATE_STOPPED:           return "SPD";
    case TRAIN_STATE_LOOP_FIND_DIR:     return "DIR";
    case TRAIN_STATE_RECOVERY_STOPPING: return "REC";
    case TRAIN_STATE_STOPPING_GOTO:     return "SGT";
    case TRAIN_STATE_DEAD_TRACK:        return "DED";
    case TRAIN_STATE_WAIT_RESOURCE:     return "WAI";
    default:                            return "???";
    }
}

static void demo_reset_slots(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        g_slots[i].enabled = 0;
        g_slots[i].train_num = -1;
        g_slots[i].started = 0;
        g_slots[i].last_target_idx = -1;
        g_slots[i].missions_completed = 0;
        g_slots[i].wait_resource_count = 0;
        g_slots[i].dead_track_count = 0;
        g_slots[i].last_seen_state = TRAIN_STATE_UNKNOWN;
        g_slots[i].wait_enter_us = 0;
    }
}

static demo_train_slot_t *demo_find_slot(int train_num) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (g_slots[i].enabled && g_slots[i].train_num == train_num) return &g_slots[i];
    }
    return NULL;
}

static demo_train_slot_t *demo_alloc_slot(int train_num) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) {
            g_slots[i].enabled = 1;
            g_slots[i].train_num = train_num;
            g_slots[i].started = 0;
            g_slots[i].last_target_idx = -1;
            g_slots[i].missions_completed = 0;
            g_slots[i].wait_resource_count = 0;
            g_slots[i].dead_track_count = 0;
            g_slots[i].last_seen_state = TRAIN_STATE_UNKNOWN;
            g_slots[i].wait_enter_us = 0;
            return &g_slots[i];
        }
    }
    return NULL;
}

static int demo_any_active_goto(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        if (pos_is_train_goto_active(g_slots[i].train_num)) return 1;
    }
    return 0;
}

static void demo_seed_rng(uint32_t seed) {
    g_demo_seed = (seed == 0) ? 1U : seed;
    g_demo_rng_state = g_demo_seed;
}

static uint32_t demo_rand_u32(void) {
    uint32_t x = g_demo_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_demo_rng_state = (x == 0) ? 1U : x;
    return g_demo_rng_state;
}

static int demo_dispatch_to_target(demo_train_slot_t *slot, track_node *target, int32_t offset_mm) {
    if (!slot || !target) return 0;
    if (!track_is_valid_train(slot->train_num)) return 0;
    if (!pos_goto(slot->train_num, target, offset_mm)) return 0;
    slot->started = 1;
    slot->last_target_idx = (int)(target - g_track);
    return 1;
}

static void demo_build_sensor_pool(void) {
    g_sensor_pool_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type != NODE_SENSOR) continue;
        if (!g_track[i].name) continue;
        if (g_sensor_pool_count < TRACK_MAX) {
            g_sensor_pool[g_sensor_pool_count++] = &g_track[i];
        }
    }
}

static track_node *gold_pick_target(int train_num, int min_trip_mm, int *out_idx) {
    if (g_sensor_pool_count <= 0) return NULL;
    train_pos_t *pos = pos_get(train_num);
    track_node *start = (pos && pos->cur_sensor) ? pos->cur_sensor : NULL;

    int tries = g_sensor_pool_count * 3;
    for (int t = 0; t < tries; t++) {
        int idx = (int)(demo_rand_u32() % (uint32_t)g_sensor_pool_count);
        track_node *cand = g_sensor_pool[idx];
        if (!cand) continue;
        if (start && cand == start) continue;
        if (start) {
            route_plan_t rp;
            if (!bfs_find_route_optimal(start, cand, 0, &rp)) continue;
            if (rp.total_dist_mm < min_trip_mm) continue;
        }
        if (out_idx) *out_idx = (int)(cand - g_track);
        return cand;
    }

    for (int idx = 0; idx < g_sensor_pool_count; idx++) {
        track_node *cand = g_sensor_pool[idx];
        if (!cand) continue;
        if (start && cand == start) continue;
        if (start) {
            route_plan_t rp;
            if (!bfs_find_route_optimal(start, cand, 0, &rp)) continue;
            if (rp.total_dist_mm < min_trip_mm) continue;
        }
        if (out_idx) *out_idx = (int)(cand - g_track);
        return cand;
    }

    return NULL;
}

static int gold_dispatch_next(demo_train_slot_t *slot) {
    int target_idx = -1;
    track_node *target = gold_pick_target(slot->train_num, g_gold_min_trip_mm, &target_idx);
    if (!target) return 0;
    if (!demo_dispatch_to_target(slot, target, 0)) return 0;
    slot->last_target_idx = target_idx;
    return 1;
}

static void demo_update_state_counters(uint64_t now_us) {
    (void)now_us;
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled) continue;
        train_pos_t *pos = pos_get(slot->train_num);
        train_route_state_t st = pos ? pos->route_state : TRAIN_STATE_UNKNOWN;
        if (st != slot->last_seen_state) {
            if (st == TRAIN_STATE_WAIT_RESOURCE) {
                slot->wait_resource_count++;
                slot->wait_enter_us = now_us;
            } else if (slot->last_seen_state == TRAIN_STATE_WAIT_RESOURCE) {
                slot->wait_enter_us = 0;
            }
            if (st == TRAIN_STATE_DEAD_TRACK) {
                slot->dead_track_count++;
            }
            slot->last_seen_state = st;
        }
    }
}

static void demo_try_finish_stop(uint64_t now_us) {
    (void)now_us;
    if (g_demo_state != DEMO_RUN_STOPPING) return;
    if (demo_any_active_goto()) return;

    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    demo_reset_slots();
    ui_puts("demo: stopped\r\n");
}

static int demo_count_bootstrap_inflight(void) {
    int count = 0;
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled || !slot->started) continue;
        if (!pos_is_train_goto_active(slot->train_num)) continue;
        train_pos_t *pos = pos_get(slot->train_num);
        if (!pos || !pos->cur_sensor) count++;
    }
    return count;
}

static int demo_start_next_unstarted_gold(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled || slot->started) continue;
        if (gold_dispatch_next(slot)) return 1;
    }
    return 0;
}

static void demo_force_stop(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        traffic_release_train(g_slots[i].train_num);
    }
    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    demo_reset_slots();
}

static int tok_eq(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b && *a == *b) {
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static void demo_maybe_retarget_waiting_gold(uint64_t now_us) {
    if (g_demo_mode != DEMO_MODE_GOLD || g_demo_state != DEMO_RUN_RUNNING) return;
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled) continue;
        if (slot->wait_enter_us == 0) continue;
        if (now_us < slot->wait_enter_us + GOLD_WAIT_RETARGET_US) continue;

        train_pos_t *pos = pos_get(slot->train_num);
        if (!pos) continue;

        int idx = -1;
        track_node *target = gold_pick_target(slot->train_num, g_gold_min_trip_mm, &idx);
        if (!target) {
            slot->wait_enter_us = now_us;
            continue;
        }

        /* Replace current pending target directly so WAIT_RESOURCE replans can use it. */
        pos->pending_target = target;
        pos->pending_offset_mm = 0;
        pos->orig_user_target = target;
        pos->orig_target_offset = 0;
        pos->target_sensor = target;
        pos->target_offset_mm = 0;
        pos->dist_to_target_mm = 0;
        pos->queued_target = NULL;
        pos->queued_offset_mm = 0;
        pos->queued_valid = 0;
        slot->last_target_idx = idx;
        slot->wait_enter_us = now_us;
        ui_mark_position_dirty();
    }
}

static void demo_status_print(void) {
    uint64_t now = read_timer();
    traffic_sensor_stats_t st;
    traffic_get_sensor_stats_ex(&st);

    ui_puts("demo status: mode=");
    ui_puts(demo_mode_str(g_demo_mode));
    ui_puts(" state=");
    ui_puts(demo_state_str(g_demo_state));
    ui_puts(" uptime=");
    if (g_demo_start_us > 0 && now >= g_demo_start_us) ui_put_i32((int)((now - g_demo_start_us) / 1000000ULL));
    else ui_puts("0");
    ui_puts("s seed=");
    ui_put_u32(g_demo_seed);
    ui_puts("\r\n");

    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled) continue;
        train_pos_t *pos = pos_get(slot->train_num);
        ui_puts("  tr");
        ui_put_i32(slot->train_num);
        ui_puts(" missions=");
        ui_put_u32(slot->missions_completed);
        ui_puts(" wait=");
        ui_put_u32(slot->wait_resource_count);
        ui_puts(" dead=");
        ui_put_u32(slot->dead_track_count);
        ui_puts(" st=");
        ui_puts(pos ? route_state_str(pos->route_state) : "N/A");
        ui_puts(" target=");
        if (pos && pos->target_sensor && pos->target_sensor->name) ui_puts(pos->target_sensor->name);
        else ui_puts("-");
        ui_puts("\r\n");
    }

    ui_puts("  sensor_stats: spurious=");
    ui_put_i32(st.spurious_count);
    ui_puts(" ambiguous=");
    ui_put_i32(st.ambiguous_count);
    if (st.last_spurious_sensor_id > 0) {
        ui_puts(" last_spur=");
        ui_put_i32((int)st.last_spurious_sensor_id);
    }
    if (st.last_ambiguous_sensor_id > 0) {
        ui_puts(" last_amb=");
        ui_put_i32((int)st.last_ambiguous_sensor_id);
    }
    ui_puts("\r\n");
}

static void demo_reservations_print(void) {
    int owners[8];
    int owner_count = traffic_get_reserved_train_list(owners, 8);
    if (owner_count <= 0) {
        ui_puts("demo reservations: none\r\n");
        return;
    }

    ui_puts("demo reservations:\r\n");
    for (int i = 0; i < owner_count; i++) {
        int train = owners[i];
        int total = traffic_get_reserved_nodes(train, NULL, 0);
        uint16_t nodes[TRACK_MAX];
        int fill = (total < TRACK_MAX) ? total : TRACK_MAX;
        (void)traffic_get_reserved_nodes(train, nodes, fill);

        ui_puts("  tr");
        ui_put_i32(train);
        ui_puts(" count=");
        ui_put_i32(total);
        ui_puts(": ");

        for (int j = 0; j < fill; j++) {
            int idx = (int)nodes[j];
            const char *name = (idx >= 0 && idx < TRACK_MAX && g_track[idx].name)
                               ? g_track[idx].name : "?";
            ui_puts(name);
            ui_puts("(");
            ui_put_i32(idx);
            ui_puts(")");
            if (j + 1 < fill) {
                ui_puts(",");
                if (((j + 1) % 16) == 0) ui_puts("\r\n      ");
            }
        }
        if (total > fill) {
            ui_puts(" ...");
        }
        ui_puts("\r\n");
    }
}

void demo_init(void) {
    demo_reset_slots();
    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    g_demo_start_us = 0;
    g_demo_stop_request_us = 0;
    g_gold_min_trip_mm = 1400;
    demo_seed_rng((uint32_t)read_timer());
    demo_build_sensor_pool();
}

static int train_seen(const int *arr, int n, int v) {
    for (int i = 0; i < n; i++) if (arr[i] == v) return 1;
    return 0;
}

static int demo_start_gold(int argc, char *argv[]) {
    if (g_demo_mode != DEMO_MODE_OFF) {
        ui_puts("demo: already running, use `demo stop`\r\n");
        return 2;
    }

    int trains[DEMO_MAX_TRAINS];
    int train_count = 0;
    int seed_override = -1;

    for (int i = 2; i < argc; i++) {
        int v = 0;
        if (!parse_int_token_local(argv[i], &v)) {
            ui_puts("demo start: numeric args required\r\n");
            return 2;
        }
        if (track_is_valid_train(v) && train_count < DEMO_MAX_TRAINS) {
            if (train_seen(trains, train_count, v)) {
                ui_puts("demo start: duplicate train\r\n");
                return 2;
            }
            trains[train_count++] = v;
            continue;
        }
        if (i == argc - 1) {
            seed_override = v;
            continue;
        }
        ui_puts("demo start: invalid argument order\r\n");
        return 2;
    }

    if (train_count < 2) {
        ui_puts("Usage: demo start <t1> <t2> [t3] [t4] [seed]\r\n");
        return 2;
    }

    for (int i = 0; i < train_count; i++) {
        if (pos_is_train_goto_active(trains[i])) {
            ui_puts("demo gold: train busy with active goto\r\n");
            return 2;
        }
    }

    if (seed_override >= 0) demo_seed_rng((uint32_t)seed_override);

    demo_reset_slots();
    g_demo_mode = DEMO_MODE_GOLD;
    g_demo_state = DEMO_RUN_STARTING;
    g_demo_start_us = read_timer();
    demo_build_sensor_pool();

    demo_train_slot_t *first_slot = NULL;
    for (int i = 0; i < train_count; i++) {
        demo_train_slot_t *slot = demo_alloc_slot(trains[i]);
        if (!slot) {
            g_demo_mode = DEMO_MODE_OFF;
            g_demo_state = DEMO_RUN_FAILED;
            demo_reset_slots();
            ui_puts("demo gold: failed to allocate slots\r\n");
            return 2;
        }
        if (i == 0) first_slot = slot;
    }

    if (!first_slot || !gold_dispatch_next(first_slot)) {
        g_demo_mode = DEMO_MODE_OFF;
        g_demo_state = DEMO_RUN_FAILED;
        demo_reset_slots();
        ui_puts("demo gold: failed initial dispatch\r\n");
        return 2;
    }

    g_demo_state = DEMO_RUN_RUNNING;
    ui_puts("demo gold: started (staged bootstrap)\r\n");
    return 2;
}

int demo_handle_command(int argc, char *argv[]) {
    if (argc <= 0 || !argv || !argv[0]) return 2;
    if (!(argv[0][0] == 'd' && argv[0][1] == 'e' && argv[0][2] == 'm' &&
          argv[0][3] == 'o' && argv[0][4] == '\0')) {
        return 2;
    }

    if (argc == 1) {
        ui_puts("Usage: demo <start|stop|status|reservations|tune|seed> ...\r\n");
        return 2;
    }

    if (argv[1][0] == 's' && argv[1][1] == 't' && argv[1][2] == 'o' &&
        argv[1][3] == 'p' && argv[1][4] == '\0') {
        if (g_demo_mode == DEMO_MODE_OFF) {
            ui_puts("demo: already stopped\r\n");
            return 2;
        }
        if (argc >= 3 && tok_eq(argv[2], "force")) {
            demo_force_stop();
            ui_puts("demo: force-stopped\r\n");
            return 2;
        }
        if (g_demo_state == DEMO_RUN_STOPPING) {
            ui_puts("demo: still stopping; use `demo stop force` to reset immediately\r\n");
            return 2;
        }
        g_demo_state = DEMO_RUN_STOPPING;
        g_demo_stop_request_us = read_timer();
        ui_puts("demo: stopping (no new missions)\r\n");
        return 2;
    }

    if (argv[1][0] == 's' && argv[1][1] == 't' && argv[1][2] == 'a' &&
        argv[1][3] == 't' && argv[1][4] == 'u' && argv[1][5] == 's' && argv[1][6] == '\0') {
        demo_status_print();
        return 2;
    }

    if (argv[1][0] == 'r' && argv[1][1] == 'e' && argv[1][2] == 's' &&
        argv[1][3] == 'e' && argv[1][4] == 'r' && argv[1][5] == 'v' &&
        argv[1][6] == 'a' && argv[1][7] == 't' && argv[1][8] == 'i' &&
        argv[1][9] == 'o' && argv[1][10] == 'n' && argv[1][11] == 's' &&
        argv[1][12] == '\0') {
        demo_reservations_print();
        return 2;
    }

    if (argv[1][0] == 's' && argv[1][1] == 'e' && argv[1][2] == 'e' &&
        argv[1][3] == 'd' && argv[1][4] == '\0') {
        if (argc != 3) {
            ui_puts("Usage: demo seed <u32>\r\n");
            return 2;
        }
        int sv = 0;
        if (!parse_int_token_local(argv[2], &sv)) {
            ui_puts("demo seed: numeric value required\r\n");
            return 2;
        }
        demo_seed_rng((uint32_t)sv);
        ui_puts("demo seed: updated\r\n");
        return 2;
    }

    if (argv[1][0] == 't' && argv[1][1] == 'u' && argv[1][2] == 'n' &&
        argv[1][3] == 'e' && argv[1][4] == '\0') {
        if (argc != 4) {
            ui_puts("Usage: demo tune <trip> <mm>\r\n");
            return 2;
        }
        int mm = 0;
        if (!parse_int_token_local(argv[3], &mm) || mm <= 0) {
            ui_puts("demo tune: positive mm required\r\n");
            return 2;
        }
        if (argv[2][0] == 't' && argv[2][1] == 'r' && argv[2][2] == 'i' &&
            argv[2][3] == 'p' && argv[2][4] == '\0') {
            g_gold_min_trip_mm = mm;
            ui_puts("demo tune trip: updated\r\n");
            return 2;
        }
        ui_puts("demo tune: unknown key\r\n");
        return 2;
    }

    if (tok_eq(argv[1], "start")) {
        if (argc < 4 || argc > 7) {
            ui_puts("Usage: demo start <t1> <t2> [t3] [t4] [seed]\r\n");
            return 2;
        }
        return demo_start_gold(argc, argv);
    }

    ui_puts("demo: unknown subcommand\r\n");
    return 2;
}

void demo_on_tick(uint64_t now_us) {
    if (g_demo_mode == DEMO_MODE_OFF) return;

    demo_update_state_counters(now_us);
    demo_try_finish_stop(now_us);
    if (g_demo_state != DEMO_RUN_RUNNING) return;

    if (g_demo_mode != DEMO_MODE_GOLD) return;

    demo_maybe_retarget_waiting_gold(now_us);

    if (demo_count_bootstrap_inflight() == 0) {
        (void)demo_start_next_unstarted_gold();
    }

    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled) continue;
        train_pos_t *pos = pos_get(slot->train_num);
        if (!pos) continue;
        if (!pos_is_train_goto_active(slot->train_num) &&
            pos->route_state == TRAIN_STATE_STOPPED &&
            !(pos->queued_valid && pos->queued_target)) {
            (void)gold_dispatch_next(slot);
        }
    }
}

void demo_on_train_stopped(int train_num, uint64_t now_us) {
    (void)now_us;
    if (g_demo_mode == DEMO_MODE_OFF) return;

    demo_train_slot_t *slot = demo_find_slot(train_num);
    if (!slot) return;
    slot->missions_completed++;

    train_pos_t *pos = pos_get(train_num);
    if (pos && pos->queued_valid && pos->queued_target) return;
    if (pos_is_train_goto_active(train_num)) return;

    if (g_demo_state != DEMO_RUN_RUNNING) return;
    if (g_demo_mode != DEMO_MODE_GOLD) return;
    (void)gold_dispatch_next(slot);
}


