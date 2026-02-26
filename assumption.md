# assumptions for TC1
- at the start of the control system: we assume the train stop at the loop, exact position unknown, direction unknown
- will use level 8 speed for goto command (for stopping distance and actual velosity measuremnts)
- the bigger loop will be used to gain stable speed (for track A: `A3 → C13 → E7 → D7 → D9 → E12 → D11 → C16 → C6 → B15 → A3 ...`)
- no 2 consecutive sensors are broken
- loop switch will be set at init process
- from any sensor node, the loop is reachable in at least one direction (forward or reverse); KASSERT fires if neither works

---

## Train Position FSM

### State Table

| State | Characteristics | Transition Trigger | Next State |
|---|---|---|---|
| UNKNOWN | Position, direction, and speed all unknown; initial state at system startup | 1. `tr` command to gain position/speed/direction  2. `goto` command issued directly | 1. → KNOWN  2. → LOOP_FIND_DIR |
| KNOWN | Position, direction, and speed known; train is running | 1. `tr` to stop the train  2. `goto` issued directly | 1. → STOPPING_TR  2. → STOPPING_GOTO |
| LOOP_FIND_DIR | Train is stationary on the loop, loop switches are set. Position, direction, and speed unknown | Automatically launches at speed 8 to collect position and direction | → LOOP_STABILIZE (once info is collected) |
| LOOP_STABILIZE | Train is running on the loop | Wait for speed to stabilize | → ON_ROUTE |
| STOPPING_TR | `tr` stop command has been sent | Predict when train has fully stopped | → STOPPED |
| STOPPED | Train is completely stationary, position known | 1. `tr` changes speed  2. `goto` issued | 1. → KNOWN (position updated automatically)  2. → ENTER_LOOP |
| ENTER_LOOP | Train position and direction known, stationary | Set loop switches. If already on loop: launch directly. If not: plan route to loop, launch, then reset loop switches once first loop sensor is detected | → LOOP_STABILIZE |
| ON_ROUTE | Running on loop at stable speed | Plan route to destination, set switches. When brake command is sent → next state | → STOPPING |
| STOPPING | Brake command issued during `goto` | Predict stopping time | → STOPPED |
| RECOVERY_STOPPING | Off-route deviation detected during `goto` | Automatically send brake command, update switch records and UI. Predict when train fully stops | → ENTER_LOOP |
| STOPPING_GOTO | `goto` received while train speed is non-zero | Automatically send brake command, predict when train fully stops | → ENTER_LOOP |

### Notes

1. `rv` must complete before entering any state.
2. All user commands for that train are rejected while a `goto` is in progress.
3. Manual physical repositioning of the train (unknown to the system) is not considered.
4. Route planning start point should be `prediction->pos`, not the last sensor already passed.
5. RECOVERY_STOPPING condition: during `goto`, the train did not follow the planned route AND it is not a sensor-skip situation.
6. `observe_path_and_correct_switches` only checks adjacent sensors; skip cases are ignored.
7. `consec_missed` has no actual use.
8. STOPPING → STOPPED transition timing is approximate.

### Full State Transition Diagram (goto perspective)

```
User issues goto
    │
    ▼ command.c:execute_it()
    ├─ UNKNOWN ──────────────────────────────► LOOP_FIND_DIR
    │   pos_goto(): set loop switches, launch speed 8        │
    │                                                        │ two consecutive loop sensors
    ├─ KNOWN ──► STOPPING_GOTO                               ▼
    │   pos_goto(): brake                             LOOP_STABILIZE
    │                                                        │
    ├─ STOPPING_TR ──► STOPPING_GOTO                  stable_count >= 3
    │   pos_goto(): update target state                      │
    │                                                        ▼
    └─ STOPPED ──────────────────────────► ENTER_LOOP ──► LOOP_STABILIZE
        pos_goto(): directly call              first loop sensor
        transition_to_enter_loop()
                                             ▼ execute_pending_route()
    STOPPING_GOTO                         ON_ROUTE
    pos_on_tick():                            │
    wait for braking ──► transition_to_enter_loop  │ rem <= braking distance
                                              ▼
                                           STOPPING
                                              │
                                              │ after brake_us
                                              ▼
                                           STOPPED ◄─── RECOVERY_STOPPING
                                                          (off-route → back to ENTER_LOOP)
```
