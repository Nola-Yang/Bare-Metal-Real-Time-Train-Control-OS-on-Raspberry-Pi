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
| `findpos <train1> [train2] [train3] [train4]` | Find positions for one or more trains |
| `demo <speed> <train1> [train2] [train3] [train4] [seed]` | Controls multiple trains on the track |
