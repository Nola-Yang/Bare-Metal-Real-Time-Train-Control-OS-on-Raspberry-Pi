#include "demo_manager.h"
#include "server/position_server.h"
#include "train_tracking/position.h"
#include "train_tracking/route_priv.h"
#include "train_tracking/traffic_manager.h"
#include "server/nameserver.h"
#include "track.h"
#include "ui.h"
#include "util.h"
#include "timer.h"
#include "command.h"
#include <stdint.h>

#define DEMO_MAX_TRAINS 4
#define GOLD_DISPATCH_DELAY_US 1000000ULL
#define DEMO_DEFAULT_SEED 03302002U

typedef enum {
    DEMO_MODE_OFF = 0,
    DEMO_MODE_GOLD = 1,
    DEMO_MODE_LOCATE = 2,
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
    int speed_level;
    int started;
    int last_target_idx;
    uint64_t next_dispatch_us;
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
static int g_position_server_tid = -1;

static int g_gold_min_trip_mm = 1400;
static uint32_t g_demo_last_ui_uptime_sec = UINT32_MAX;

static demo_train_slot_t g_slots[DEMO_MAX_TRAINS];

static track_node *g_sensor_pool[TRACK_MAX];
static int g_sensor_pool_count = 0;

static int demo_sensor_name_is(const track_node *node, const char *name) {
    return node != NULL &&
           node->type == NODE_SENSOR &&
           node->name != NULL &&
           str_eq(node->name, name);
}

static int demo_target_is_excluded(const track_node *node) {
    const track_node *rev;

    if (!node || node->type != NODE_SENSOR) return 0;
    rev = (node->reverse && node->reverse->type == NODE_SENSOR)
              ? node->reverse
              : NULL;

    return demo_sensor_name_is(node, "C12") ||
           demo_sensor_name_is(node, "E5") ||
           demo_sensor_name_is(node, "E14") ||
           demo_sensor_name_is(node, "C9") ||
           demo_sensor_name_is(rev, "C12") ||
           demo_sensor_name_is(rev, "E5") ||
           demo_sensor_name_is(rev, "E14") ||
           demo_sensor_name_is(rev, "C9");
}

static int demo_position_server_tid(void) {
    if (g_position_server_tid < 0) {
        g_position_server_tid = WhoIs(POSITION_SERVER_NAME);
    }
    return g_position_server_tid;
}

int get_demo_train_ind(int train_num) {
    for (int i = 0; i < DEMO_MAX_TRAINS; ++i) {
        if (g_slots[i].train_num == train_num) {
            return i;
        }
    }

    return -1;
}

static const char *demo_mode_str(demo_mode_t m) {
    switch (m) {
    case DEMO_MODE_OFF:   return "OFF";
    case DEMO_MODE_GOLD:   return "GOLD";
    case DEMO_MODE_LOCATE: return "LOCATE";
    default:               return "UNK";
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

static void demo_clear_slot(demo_train_slot_t *slot) {
    slot->started = 0;
    slot->last_target_idx = -1;
    slot->next_dispatch_us = 0;
    slot->missions_completed = 0;
    slot->wait_resource_count = 0;
    slot->dead_track_count = 0;
    slot->last_seen_state = TRAIN_STATE_UNKNOWN;
    slot->wait_enter_us = 0;
}

static void demo_reset_slots(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        g_slots[i].enabled = 0;
        g_slots[i].train_num = -1;
        g_slots[i].speed_level = 0;
        demo_clear_slot(&g_slots[i]);
    }
}

static demo_train_slot_t *demo_alloc_slot(int train_num, int speed_level) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) {
            g_slots[i].enabled = 1;
            g_slots[i].train_num = train_num;
            g_slots[i].speed_level = speed_level;
            demo_clear_slot(&g_slots[i]);
            return &g_slots[i];
        }
    }
    return NULL;
}

static int demo_slot_is_dead_track(const demo_train_slot_t *slot) {
    if (!slot || !slot->enabled) return 0;
    train_pos_t *pos = pos_get(slot->train_num);
    return pos && pos->route_state == TRAIN_STATE_DEAD_TRACK;
}

static int demo_any_active_goto(void) {
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        if (demo_slot_is_dead_track(&g_slots[i])) continue;
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
    int pos_tid;
    if (!slot || !target) return 0;
    if (!track_is_valid_train(slot->train_num)) return 0;
    pos_tid = demo_position_server_tid();
    if (pos_tid < 0) return 0;
    if (!PositionServerGoto(pos_tid, slot->train_num,
                            (int)(target - g_track), slot->speed_level, offset_mm)) return 0;
    slot->started = 1;
    slot->last_target_idx = (int)(target - g_track);
    slot->next_dispatch_us = 0;
    return 1;
}

static void demo_build_sensor_pool(void) {
    g_sensor_pool_count = 0;
    for (int i = 0; i < TRACK_MAX; i++) {
        if (g_track[i].type != NODE_SENSOR) continue;
        if (!g_track[i].name) continue;
        if (demo_target_is_excluded(&g_track[i])) continue;
        if (g_sensor_pool_count < TRACK_MAX) {
            g_sensor_pool[g_sensor_pool_count++] = &g_track[i];
        }
    }
}

static int is_valid_target(track_node *cand, track_node *start, int min_trip_mm) {
    if (!cand) return 0;
    if (start && cand == start) return 0;
    if (start) {
        route_plan_t rp;
        if (!bfs_find_route_optimal(start, cand, 0, &rp)) return 0;
        if (rp.total_dist_mm < min_trip_mm) return 0;
    }
    return 1;
}

static track_node *gold_pick_target(int train_num, int min_trip_mm, int *out_idx) {
    if (g_sensor_pool_count <= 0) return NULL;
    train_pos_t *pos = pos_get(train_num);
    track_node *start = (pos && pos->cur_sensor) ? pos->cur_sensor : NULL;

    int tries = g_sensor_pool_count * 3;
    for (int t = 0; t < tries; t++) {
        int idx = (int)(demo_rand_u32() % (uint32_t)g_sensor_pool_count);
        track_node *cand = g_sensor_pool[idx];
        if (!is_valid_target(cand, start, min_trip_mm)) continue;
        if (out_idx) *out_idx = (int)(cand - g_track);
        return cand;
    }

    for (int idx = 0; idx < g_sensor_pool_count; idx++) {
        track_node *cand = g_sensor_pool[idx];
        if (!is_valid_target(cand, start, min_trip_mm)) continue;
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

int gold_dispatch_next_by_ind(int demo_train_ind) {
    if (demo_train_ind < 0 || demo_train_ind >= DEMO_MAX_TRAINS) return 0;
    return gold_dispatch_next(&g_slots[demo_train_ind]);
}

int demo_retry_train_by_ind(int demo_train_ind) {
    if (demo_train_ind < 0 || demo_train_ind >= DEMO_MAX_TRAINS) return 0;
    if (!g_slots[demo_train_ind].enabled) return 0;
    if (g_demo_mode != DEMO_MODE_GOLD) return 0;
    return gold_dispatch_next(&g_slots[demo_train_ind]);
}

static void demo_update_state_counters(uint64_t now_us) {
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
                slot->wait_enter_us = 0;
            }
            if (g_demo_state == DEMO_RUN_RUNNING &&
                st == TRAIN_STATE_STOPPED &&
                slot->last_seen_state == TRAIN_STATE_STOPPING) {
                slot->missions_completed++;
            }
            if (st == TRAIN_STATE_STOPPED) {
                slot->next_dispatch_us = now_us + GOLD_DISPATCH_DELAY_US;
            } else if (slot->last_seen_state == TRAIN_STATE_STOPPED) {
                slot->next_dispatch_us = 0;
            }
            slot->last_seen_state = st;
            ui_mark_position_dirty();
        }
    }
}

static void demo_try_finish_stop(uint64_t now_us) {
    (void)now_us;
    if (g_demo_state != DEMO_RUN_STOPPING) return;
    if (demo_any_active_goto()) return;

    int pos_tid = demo_position_server_tid();
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        if (!track_is_valid_train(g_slots[i].train_num)) continue;
        if (pos_tid >= 0) {
            (void)PositionServerSpeedChange(pos_tid, g_slots[i].train_num, 0);
        }
        track_set_speed(g_slots[i].train_num, 0);
        pos_reset_dead_train(g_slots[i].train_num);
    }

    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    g_demo_start_us = 0;
    g_demo_stop_request_us = 0;
    demo_reset_slots();
    g_demo_last_ui_uptime_sec = UINT32_MAX;
    ui_mark_position_dirty();
    ui_cmd_puts("demo: stopped\r\n");
}

static void demo_finish_locate(void) {
    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    g_demo_start_us = 0;
    g_demo_stop_request_us = 0;
    demo_reset_slots();
    g_demo_last_ui_uptime_sec = UINT32_MAX;
    ui_mark_position_dirty();
}

static int demo_start_next_unstarted(void) {
    int pos_tid = demo_position_server_tid();

    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled || slot->started) continue;
        if (!track_is_valid_train(slot->train_num)) continue;
        if (pos_is_train_position_known(slot->train_num)) {
            if (g_demo_mode == DEMO_MODE_GOLD) {
                return gold_dispatch_next(slot);
            }
            slot->started = 1;
            slot->next_dispatch_us = 0;
            return 1;
        }
        if (pos_tid < 0) return 0;
        if (!PositionServerStartFindPos(pos_tid, slot->train_num)) return 0;
        slot->started = 1;
        return 1;
    }
    return 0;
}

static void demo_force_stop(void) {
    int pos_tid = demo_position_server_tid();

    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        if (!track_is_valid_train(g_slots[i].train_num)) continue;
        if (pos_tid >= 0) {
            (void)PositionServerSpeedChange(pos_tid, g_slots[i].train_num, 0);
        }
        track_set_speed(g_slots[i].train_num, 0);
        pos_reset_dead_train(g_slots[i].train_num);
    }
    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    g_demo_start_us = 0;
    g_demo_stop_request_us = 0;
    demo_reset_slots();
    g_demo_last_ui_uptime_sec = UINT32_MAX;
    ui_mark_position_dirty();
}

void demo_get_ui_summary(demo_ui_summary_t *out, uint64_t now_us) {
    uint32_t completed = 0;

    if (!out) return;
    out->mode_name = demo_mode_str(g_demo_mode);
    out->state_name = demo_state_str(g_demo_state);
    out->seed = g_demo_seed;
    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        if (!g_slots[i].enabled) continue;
        completed += g_slots[i].missions_completed;
    }
    out->missions_completed = completed;
    out->gold_min_trip_mm = g_gold_min_trip_mm;
    if (g_demo_mode != DEMO_MODE_OFF &&
        g_demo_start_us > 0 &&
        now_us >= g_demo_start_us) {
        out->uptime_sec = (uint32_t)((now_us - g_demo_start_us) / 1000000ULL);
    } else {
        out->uptime_sec = 0;
    }
    traffic_get_sensor_stats_ex(&out->sensor_stats);
}

int demo_is_auto_dispatching_targets(void) {
    return g_demo_mode == DEMO_MODE_GOLD &&
           (g_demo_state == DEMO_RUN_STARTING ||
            g_demo_state == DEMO_RUN_RUNNING);
}

int demo_is_active(void) {
    return g_demo_mode != DEMO_MODE_OFF;
}

void demo_init(void) {
    g_position_server_tid = -1;
    demo_reset_slots();
    g_demo_mode = DEMO_MODE_OFF;
    g_demo_state = DEMO_RUN_IDLE;
    g_demo_start_us = 0;
    g_demo_stop_request_us = 0;
    g_gold_min_trip_mm = 1400;
    g_demo_last_ui_uptime_sec = UINT32_MAX;
    demo_seed_rng(DEMO_DEFAULT_SEED);
    demo_build_sensor_pool();
}

static int train_seen(const int *arr, int n, int v) {
    for (int i = 0; i < n; i++) if (arr[i] == v) return 1;
    return 0;
}

static void demo_puts_gold_usage(void) {
    ui_cmd_puts("Usage: demo <speed> <t1> [t2] [t3] [t4] [seed]\r\n");
}

static void demo_puts_overview_usage(void) {
    ui_cmd_puts("Usage: demo <speed> <t1> [t2] [t3] [t4] [seed] | demo <stop|tune|seed> ...\r\n");
}

static int demo_start_session(demo_mode_t mode,
                              int train_count,
                              int speed_level,
                              const int *trains,
                              int seed_override) {
    const char *mode_prefix = (mode == DEMO_MODE_GOLD) ? "demo gold" : "findpos";

    if (g_demo_mode != DEMO_MODE_OFF) {
        ui_cmd_puts("demo: already running, use `demo stop`\r\n");
        return 2;
    }

    for (int i = 0; i < train_count; i++) {
        if (pos_is_train_goto_active(trains[i])) {
            ui_cmd_puts(mode_prefix);
            ui_cmd_puts(": train busy with active goto\r\n");
            return 2;
        }
    }

    if (mode == DEMO_MODE_GOLD && seed_override >= 0) demo_seed_rng((uint32_t)seed_override);

    demo_reset_slots();
    g_demo_mode = mode;
    g_demo_state = DEMO_RUN_STARTING;
    g_demo_start_us = read_timer();
    g_demo_stop_request_us = 0;
    g_demo_last_ui_uptime_sec = UINT32_MAX;
    demo_build_sensor_pool();

    for (int i = 0; i < train_count; i++) {
        demo_train_slot_t *slot = demo_alloc_slot(trains[i], speed_level);
        if (!slot) {
            g_demo_mode = DEMO_MODE_OFF;
            g_demo_state = DEMO_RUN_FAILED;
            demo_reset_slots();
            ui_cmd_puts(mode_prefix);
            ui_cmd_puts(": failed to allocate slots\r\n");
            return 2;
        }
    }

    /* Start only the first train; demo_on_tick will chain the rest. */
    if (!demo_start_next_unstarted()) {
        g_demo_mode = DEMO_MODE_OFF;
        g_demo_state = DEMO_RUN_FAILED;
        demo_reset_slots();
        if (mode == DEMO_MODE_GOLD) {
            ui_cmd_puts("demo gold: failed initial dispatch\r\n");
        } else {
            ui_cmd_puts("findpos: failed initial bootstrap\r\n");
        }
        return 2;
    }

    if (mode == DEMO_MODE_GOLD) {
        ui_cmd_puts("demo gold: bootstrapping trains one by one\r\n");
    } else {
        ui_cmd_puts("findpos: bootstrapping trains one by one\r\n");
    }
    ui_mark_position_dirty();
    return 2;
}

static int demo_start_gold(int argc, char *argv[]) {
    int trains[DEMO_MAX_TRAINS];
    int train_count = 0;
    int seed_override = -1;
    int speed_level = 0;

    if (argc < 2 || !str_parse_int(argv[1], &speed_level)) {
        demo_puts_gold_usage();
        return 2;
    }

    if (!is_valid_speed_level(speed_level)) {
        ui_cmd_puts("Invalid speed level\r\n");
        return 2;
    }

    if (!is_valid_goto_speed(speed_level)) {
        ui_cmd_puts("Speed level not supported\r\n");
        return 2;
    }

    for (int i = 2; i < argc; i++) {
        int v = 0;
        if (!str_parse_int(argv[i], &v)) {
            ui_cmd_puts("demo: numeric args required\r\n");
            return 2;
        }
        if (track_is_valid_train(v) && train_count < DEMO_MAX_TRAINS) {
            if (train_seen(trains, train_count, v)) {
                ui_cmd_puts("demo: duplicate train\r\n");
                return 2;
            }
            trains[train_count++] = v;
            continue;
        }
        if (i == argc - 1) {
            seed_override = v;
            continue;
        }
        ui_cmd_puts("demo: invalid argument order\r\n");
        return 2;
    }

    if (train_count < 1) {
        demo_puts_gold_usage();
        return 2;
    }

    return demo_start_session(DEMO_MODE_GOLD, train_count, speed_level, trains, seed_override);
}


int demo_start_locate(int train_count, const int *trains) {
    if (train_count < 1 || train_count > DEMO_MAX_TRAINS) {
        ui_cmd_puts("findpos: invalid train count\r\n");
        return 2;
    }
    return demo_start_session(DEMO_MODE_LOCATE, train_count, 0, trains, -1);
}

int demo_handle_command(int argc, char *argv[]) {
    int speed_level = 0;

    if (argc <= 0 || !argv || !argv[0]) return 2;
    if (!str_eq(argv[0], "demo")) {
        return 2;
    }

    if (argc == 1) {
        demo_puts_overview_usage();
        return 2;
    }

    if (str_eq(argv[1], "stop")) {
        if (g_demo_mode == DEMO_MODE_OFF) {
            ui_cmd_puts("demo: already stopped\r\n");
            return 2;
        }
        if (argc >= 3 && str_eq(argv[2], "force")) {
            demo_force_stop();
            ui_cmd_puts("demo: force-stopped\r\n");
            return 2;
        }
        if (g_demo_state == DEMO_RUN_STOPPING) {
            ui_cmd_puts("demo: still stopping; use `demo stop force` to reset immediately\r\n");
            return 2;
        }
        g_demo_state = DEMO_RUN_STOPPING;
        g_demo_stop_request_us = read_timer();
        ui_mark_position_dirty();
        ui_cmd_puts("demo: stopping (no new missions)\r\n");
        return 2;
    }

    if (str_eq(argv[1], "seed")) {
        if (argc != 3) {
            ui_cmd_puts("Usage: demo seed <u32>\r\n");
            return 2;
        }
        int sv = 0;
        if (!str_parse_int(argv[2], &sv)) {
            ui_cmd_puts("demo seed: numeric value required\r\n");
            return 2;
        }
        demo_seed_rng((uint32_t)sv);
        ui_mark_position_dirty();
        ui_cmd_puts("demo seed: updated\r\n");
        return 2;
    }

    if (str_eq(argv[1], "tune")) {
        if (argc != 4) {
            ui_cmd_puts("Usage: demo tune <trip> <mm>\r\n");
            return 2;
        }
        int mm = 0;
        if (!str_parse_int(argv[3], &mm) || mm <= 0) {
            ui_cmd_puts("demo tune: positive mm required\r\n");
            return 2;
        }
        if (str_eq(argv[2], "trip")) {
            g_gold_min_trip_mm = mm;
            ui_cmd_puts("demo tune trip: updated\r\n");
            return 2;
        }
        ui_cmd_puts("demo tune: unknown key\r\n");
        return 2;
    }


    if (str_parse_int(argv[1], &speed_level)) {
        if (argc < 3 || argc > 7) {
            demo_puts_gold_usage();
            return 2;
        }
        return demo_start_gold(argc, argv);
    }

    ui_cmd_puts("demo: unknown subcommand\r\n");
    return 2;
}

void demo_on_tick(uint64_t now_us) {
    if (g_demo_mode == DEMO_MODE_OFF) return;

    uint32_t uptime_sec = 0;
    if (g_demo_start_us > 0 && now_us >= g_demo_start_us) {
        uptime_sec = (uint32_t)((now_us - g_demo_start_us) / 1000000ULL);
    }
    if (uptime_sec != g_demo_last_ui_uptime_sec) {
        g_demo_last_ui_uptime_sec = uptime_sec;
        ui_mark_position_dirty();
    }

    demo_update_state_counters(now_us);
    demo_try_finish_stop(now_us);

    /* STARTING: bootstrap trains one at a time.
     *
     * Phase A — position finding (sequential to avoid attribution ambiguity):
     *   Unknown-position trains acquire position one by one; known-position
     *   trains are accepted immediately (LOCATE) or skip to their first mission
     *   (GOLD).
     *   Wait for every started train to confirm its position,
     *   As soon as the last started train has confirmed position, kick off the
     *   next unstarted train.
     *
     * Phase B — wait for all trains to reach STOPPED at their first destination:
     *   Once every train has started, wait for all to be STOPPED before
     *   transitioning to RUNNING.
     */
    if (g_demo_state == DEMO_RUN_STARTING) {
        int any_unstarted = 0;
        int all_stopped   = 1;
        for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
            demo_train_slot_t *slot = &g_slots[i];
            if (!slot->enabled) continue;
            if (demo_slot_is_dead_track(slot)) continue;
            if (!slot->started) { any_unstarted = 1; all_stopped = 0; continue; }
            train_pos_t *pos = pos_get(slot->train_num);
            if (!pos) { all_stopped = 0; continue; }

            int pos_confirmed = pos->cur_sensor != NULL &&
                                pos->route_state != TRAIN_STATE_UNKNOWN      &&
                                pos->route_state != TRAIN_STATE_FIND_POS &&
                                pos->route_state != TRAIN_STATE_STOPPING_GOTO;
            if (!pos_confirmed) return;  /* still acquiring — wait */

            if (pos->route_state != TRAIN_STATE_STOPPED) all_stopped = 0;
        }

        if (any_unstarted) {
            /* Kick off the next train's bootstrap. */
            (void)demo_start_next_unstarted();
            return;
        }

        if (!all_stopped) return;

        if (g_demo_mode == DEMO_MODE_LOCATE) {
            demo_finish_locate();
            return;
        }

        g_demo_state = DEMO_RUN_RUNNING;
        ui_mark_position_dirty();
        return;
    }

    if (g_demo_mode != DEMO_MODE_GOLD) return;
    if (g_demo_state != DEMO_RUN_RUNNING) return;

    for (int i = 0; i < DEMO_MAX_TRAINS; i++) {
        demo_train_slot_t *slot = &g_slots[i];
        if (!slot->enabled) continue;
        if (demo_slot_is_dead_track(slot)) continue;
        train_pos_t *pos = pos_get(slot->train_num);
        if (!pos) continue;
        if (pos->deadlock_recover.valid) continue;
        if (!pos_is_train_goto_active(slot->train_num) &&
            pos->route_state == TRAIN_STATE_STOPPED &&
            !(pos->queued_valid && pos->queued_target)) {
            if (slot->next_dispatch_us > 0 && now_us < slot->next_dispatch_us) {
                continue;
            }
            (void)gold_dispatch_next(slot);
        }
    }
}
