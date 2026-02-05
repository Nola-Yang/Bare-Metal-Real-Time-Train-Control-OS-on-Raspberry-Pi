# K3

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
<br>


```bash
make clean
make
```

An image file should be created at `build/kernel.img`

<br>
<br>


## How to Run

### STEP 1.
Upload the image at `build/kernel.img` into a specific Raspberry Pi using the web interface at: https://cs452.student.cs.private.uwaterloo.ca

<br>

### STEP 2.
Restart the selected Raspberry Pi that you have uploaded the image into.

<br>

Once the Raspberry Pi has restarted, output will be printed onto the screen as the kernel runs the specific tasks. 


<br>

### Output Format
<details>
<summary>Click to Expand Output</summary>

```text
========================================
K3 Clock Server Test
========================================
Created NameServer, tid=1
Created ClockServer, tid=2
Created IdleTask, tid=4

----- Created Clients -----
tid: 5, priority: 3, delay interval: 10, no. of delays: 20
tid: 6, priority: 4, delay interval: 23, no. of delays: 9
tid: 7, priority: 5, delay interval: 33, no. of delays: 6
tid: 8, priority: 6, delay interval: 71, no. of delays: 3

FirstUserTask: All clients started, waiting for completion
Client tid=5: interval=10, completed=1/20, tick=13
Client tid=5: interval=10, completed=2/20, tick=23
Client tid=6: interval=23, completed=1/9, tick=26
Client tid=5: interval=10, completed=3/20, tick=33
Client tid=7: interval=33, completed=1/6, tick=36
Client tid=5: interval=10, completed=4/20, tick=43
Client tid=6: interval=23, completed=2/9, tick=49
Client tid=5: interval=10, completed=5/20, tick=53
Client tid=5: interval=10, completed=6/20, tick=63
Client tid=7: interval=33, completed=2/6, tick=69
Client tid=6: interval=23, completed=3/9, tick=72
Client tid=5: interval=10, completed=7/20, tick=73
Client tid=8: interval=71, completed=1/3, tick=74
Client tid=5: interval=10, completed=8/20, tick=83
Client tid=5: interval=10, completed=9/20, tick=93
Client tid=6: interval=23, completed=4/9, tick=95
Client tid=7: interval=33, completed=3/6, tick=102
Client tid=5: interval=10, completed=10/20, tick=103
Idle: 96%
Client tid=5: interval=10, completed=11/20, tick=113
Client tid=6: interval=23, completed=5/9, tick=118
Client tid=5: interval=10, completed=12/20, tick=123
Client tid=5: interval=10, completed=13/20, tick=133
Client tid=7: interval=33, completed=4/6, tick=135
Client tid=6: interval=23, completed=6/9, tick=141
Client tid=5: interval=10, completed=14/20, tick=143
Client tid=8: interval=71, completed=2/3, tick=145
Client tid=5: interval=10, completed=15/20, tick=153
Client tid=5: interval=10, completed=16/20, tick=163
Client tid=6: interval=23, completed=7/9, tick=164
Client tid=7: interval=33, completed=5/6, tick=168
Client tid=5: interval=10, completed=17/20, tick=173
Client tid=5: interval=10, completed=18/20, tick=183
Client tid=6: interval=23, completed=8/9, tick=187
Client tid=5: interval=10, completed=19/20, tick=193
Client tid=7: interval=33, completed=6/6, tick=201
Client tid=7: Finished all delays
Client tid=5: interval=10, completed=20/20, tick=203
Client tid=5: Finished all delays
Idle: 95%
Client tid=6: interval=23, completed=9/9, tick=210
Client tid=6: Finished all delays
Client tid=8: interval=71, completed=3/3, tick=216
Client tid=8: Finished all delays
Idle: 98%
Idle: 99%
Idle: 99%
```

</details>

<br>
<br>


### QEMU

> [!WARNING]
> BCM system timer interrupt **does not work on the QEMU**, but arch generic timer does work.
>
> So it is recommended to **test on the actual Raspberry Pi** instead of using QEMU

<br>


- `make sim` to start QEMU simulation
- Press `Ctrl+A` then `X` to exit the QEMU simulator.