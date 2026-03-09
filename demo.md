# TC1 Demo Runbook (20 Minutes)

## 1) Scope and Roles

- Duration: 20 minutes total.
- Part 1 (scripted): 10-12 minutes.
- Part 2 (unscripted Q/A): 6-8 minutes.
- Part 3 (offline discussion/code review): optional.

Team roles:
- Presenter: explains intent, expected behavior, and design tradeoffs.
- Operator: types commands and controls train setup.
- switch at off-route check


## 2) Ground Truth from Current Code

What is implemented and demo-safe:
- Commands: `tr`, `sw`, `rv`, `li`, `goto`, `q`.
- Valid trains: `13, 14, 15, 17, 18`.
- `goto` runs at fixed user speed 8.
- UI evidence is on rows:
  - Row 22: `Pos` + state (`UNK/KNOW/STR/DIR?/STAB/ROUTE/STOP/STPD/REC/ENT/SGT`).
  - Row 23: prediction error (`err_t`, `err_d`).
  - Row 24: off-route / dead-track status.
- Off-route recovery exists:
  - Unexpected sensor and target no longer reachable -> auto stop -> `RECOVERY_STOPPING` -> `ENTER_LOOP` -> resume.
- Dead-track timeout exists:
  - No sensor within timeout window in active motion states -> `DEAD TRACK`, waits for manual push-triggered sensor to recover.
- Switch reliability helper resends commands for `SW1`, `SW153`, `SW156` during route execution.

Important limitations to state clearly:
- Position tracking path assumes single active tracked train for sensor attribution.
- No CLI command to inject fake sensor events.

## 3) Scripted Demo (Part 1)

## Step 0: Intro + Safety 

Before:
- Presenter: "We will show command control, tracking+stopping accuracy, and automatic recovery paths."
- Presenter: "If something drifts, we will explicitly halt and recover."

During:
- Operator confirms train is clear and track is safe.

After:
- Presenter points at UI rows 22/23/24 and explains what to watch.

## Step 1: Basic Control + Observability 

Before:
- Presenter: "First we show low-level control and visible state."

During (operator commands):
```bash
li 13 1
tr 13 8
```
Wait for at least 2 sensor hits.

Then:
```bash
tr 13 0
```

What audience should watch:
- Headlight on (direction/debug visibility).
- Recent sensors list updates.
- Row 22 state transitions from unknown toward known, then stop path (`STR` then `STPD`).

After:
- Presenter: "This confirms command path, CAN feedback, and UI synchronization are alive."

## Step 2: `goto` 

- sensor branch enter
- positive/negtive offset
- on/off loop
- diff speed
  
**track D**
**tr 18**

During:
```bash
li 18
tr 18 10
goto 18 C5 
goto 18 C3
goto 18 BR2
goto 18 A3 +100
goto 18 C13  -100
goto 18 E1 (sw8)
goto 18 E1
```



  
**track C**
**tr 14**
During:
```bash
li 14
tr 14 10
goto 14 C5 
goto 14 B6
goto 14 BR7
goto 14 C13 +100
goto 14  C13  -100
```

while route is active (to show command gating):
```bash
goto 13 A3
```
Expected: rejected because a `goto` is already active.

What audience should watch:
- Row 22 state flow through `DIR?` / `STAB` / `ROUTE` / `STOP` / `STPD` (exact path depends on starting state).
- Row 23 prediction error converging.
- Row 24 remains clean when no mismatch.

After:
- Presenter: "This shows automated state-machine control, not manual switch babysitting."

Switch presenter/operator roles now.


## Step 3: Robustness Injection 

Primary plan (off-route auto-recovery):
- Start another route requiring at least one switch.
- Right before expected branch, intentionally set one key switch opposite to plan.

During:
```bash
goto 13 NODE_ROUTE
# then sabotage one planned switch, example only:
sw <planned_switch> <opposite_dir>
```

What audience should watch:
- Row 24 shows mismatch (`exp=... act=...`).
- State goes to `REC`, then re-enters loop (`ENT`/`STAB`) and resumes.

Backup plan (if timing miss, no mismatch triggered):
- Show controlled halt/recover path instead:
```bash
tr 13 0
```
- Explain dead-track timeout and manual-push recovery mechanism using Row 24 `DEAD TRACK` message if reproduced.



## 4) Unscripted Q/A Playbook (Part 2)

If asked: "Can you do this with another train?"
- Answer: command/control supports trains `13,14,15,17,18`.
- Clarify: current tracking attribution is intentionally single-train focused for robustness in this milestone.
- Offer: quick control-only action on another train (`li`, `tr`) without claiming multi-train `goto` tracking.

If asked: "Can you do this on the other track?"
- Answer: yes via compile-time `TRACK=C|D`; loop/stabilization and FSM logic are shared.

If asked: "Can we fake sensor triggers?"
- Answer: will trigger off route check

If asked: "Can the train go to this location?"





