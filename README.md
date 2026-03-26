# TC2

[![C](https://img.shields.io/badge/C-005697?style=for-the-badge)](https://en.cppreference.com/w/c/language.html)
[![ARMv8 Assembly](https://img.shields.io/badge/ARMv8%20Assembly-00BEDB?style=for-the-badge)](https://developer.arm.com/documentation/ddi0602/latest/)

<br>


## Authors
|     |     |     |
| --- | --- | --- |
| Alex Au | 20887300 | a32au@uwaterloo.ca |
| Jiayue Yang | 21231362 | j782yang@uwaterloo.ca |

<br>

## How to Build

### STEP 1:
Clone the repository from Gitlab by running the following command:

```bash
git clone ist-git@git.uwaterloo.ca:cs-452-nolex/k2.git
```

<br>

### STEP 2:
Checkout the specific commit hash for the submission.
The commit hash to checkout is: 

```bash
git checkout <commit_hash>
```

<br>

### STEP 3:
```bash
make clean
make TRACK=C
```

An image file should be created at `build/kernel.img`

You can also pass compile-time options:

```bash
make TRACK=<C|D> CACHE=<n|i|d|b> OPT=<0|1> VERBOSE=<0|1> IDLE_WINDOW_TICKS=<ticks>
```

| Option | Default | Valid values | Description |
| --- | --- | --- | --- |
| `TRACK` | `C` | `C`, `D` | Track layout to compile for. |
| `CACHE` | `b` | `n`, `i`, `d`, `b` | Cache mode: none / I-cache / D-cache / both. |
| `OPT` | `1` | `0`, `1` | Enable (`1`) or disable (`0`) `-O3` optimization. |
| `VERBOSE` | `0` | `0`, `1` | Enable extra verbose debug compile flag (`-DVERBOSE`). |
| `IDLE_WINDOW_TICKS` | `50` | positive integer | Rolling window size for idle % display. |


<br>
<br>


## How to Run

### STEP 1.
Upload the image at `build/kernel.img` into a specific Raspberry Pi using the web interface at: https://cs452.student.cs.private.uwaterloo.ca

<br>

### STEP 2.
Restart the selected Raspberry Pi that you have uploaded the image into.

<br>

### STEP 3.
Enter commands at the prompt:

| Command | Description |
| ------- | ----------- |
| `tr <train> <speed>` | Set train speed (from 0 to 14) |
| `sw <switch> <S|C>`  | Set switch direction |
| `rv <train>` | Reverse train (stop -> reverse -> restore speed) |
| `q/Q` | quit/reboot |
| `init` | init all switch to straight |
| `li <train> <1|0>` | Turn on/off the train light |
| `goto <train> <node id> [offset+-mm]` | Make the train pick up a constant speed (fixed at speed step 8) and stops at a specific track node |
| `demo start <train1> <train2> [train3] [train4]` | Controls multiple trains on the track | 

## Four-Round Competitive Game Rules

### Overview

- The match lasts for `4` rounds.
- There are `3` trains on the track during the game:
  - `2` scoring trains: one controlled by the human player and one controlled by the AI.
  - `1` neutral train: it creates traffic pressure but never scores.
- In every round, all three trains receive exactly one destination sensor and depart in the same round.
- The player only chooses a destination sensor. The system handles route planning, reservation, switches, launch timing, deadlock handling, and yielding automatically.
- A train is considered to have reached its round target when the system determines that it has arrived and stopped there. The final target sensor does not need to physically trigger.

### Round Priority

- Priority alternates in a fixed order across the four rounds.
- Before Round 1, the system performs one random draw to decide which scoring player has priority first.
- Round 2 gives priority to the other scoring player.
- Round 3 returns priority to the Round 1 player.
- Round 4 gives priority to the other scoring player again.
- Each scoring player therefore receives priority in exactly `2` rounds.

### Round Flow

1. The human player secretly chooses one destination sensor.
2. The AI secretly chooses one destination sensor.
3. The neutral train's destination sensor is drawn publicly for that round.
4. All three destinations are revealed at the same time.
5. The system plans and reserves routes in this order:
   - the scoring player with round priority
   - the other scoring player
   - the neutral train
6. All three trains execute their movements within the same round.
7. The round ends when:
   - both scoring trains have completed their round targets, and
   - the neutral train has either completed its own round target or has been redirected to a safe standby location and marked as resolved.

There is no round time limit.

### Neutral Train Rules

- The neutral train receives one official destination sensor per round.
- Its round destination is chosen by a public draw without replacement, so the same neutral destination is not repeated within the same 4-round match.
- The neutral destination pool contains only physical sensors.
- At draw time, the neutral destination must not be the current sensor occupied by any train.
- The system should prefer reachable neutral destinations whose shortest path from the neutral train is at least `1400 mm`.
- If no such candidate exists in the current round, the draw may fall back to any reachable physical sensor.
- The neutral train starts the next round from wherever it stopped at the end of the previous round.
- The neutral train never scores and never consumes any sensor reward that could otherwise be earned by a scoring player.

### Scoring

- Only the two scoring trains can earn points.
- Points are awarded only when a scoring train physically triggers a new physical sensor.
- Logical arrival at a round target does not award points by itself.
- If a sensor was used as a final destination but did not physically trigger on arrival, it remains available for future scoring if it is physically triggered later.
- Each physical sensor can award points at most twice during the whole match:
  - The first scoring player to physically trigger that sensor earns `1.5` points.
  - The other scoring player earns `1.0` point the first time they physically trigger that same sensor later.
- A player earns `0` additional points for re-triggering a sensor they have already scored from.
- After both scoring players have already earned from the same physical sensor, that sensor awards no further points.
- Scoring is tracked by physical sensor, not by directional node.
- The neutral train does not affect first-touch or second-touch scoring rewards.
- Any newly triggered scoring sensor during a yielding movement still counts normally.

### Yielding and Conflict Resolution

- Between the two scoring players, the original rule still applies:
  - if the later-planned scoring player still cannot complete their target after the earlier-planned scoring player has already arrived and stopped, and the earlier player's stopped train is the blocking cause, then the earlier player must yield.
- The yielding location is computed automatically by the system.
- A scoring player who yields after already completing their own target does not need to return to that target afterward.
- If a scoring player is blocked by the neutral train's stopped position, the neutral train must yield first.
- If the neutral train is the only unfinished train at the end of the round and its remaining block comes only from already-completed scoring trains, the scoring trains are not forced to yield for it.
- In that case, the system redirects the neutral train to a safe standby location and considers the neutral train resolved for the round.

### Winning the Match

- After 4 rounds, the scoring player with the higher total score wins.
- If both scoring players have the same total score, the match ends in a draw.
