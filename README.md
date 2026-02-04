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



## How to Run

### STEP 1.
Upload the image at `build/kernel.img` into a specific Raspberry Pi using the web interface at: https://cs452.student.cs.private.uwaterloo.ca

<br>

### STEP 2.
Restart the selected Raspberry Pi that you have uploaded the image into.

<br>

Once the Raspberry Pi has restarted, output will be printed onto the screen as the kernel runs the specific tasks. 

### Output Format
```text
========================================
K3 Clock Server Test
========================================
Created NameServer, tid=1
Created ClockServer, tid=2
Created IdleTask, tid=4
Created clients: 5, 6, 7, 8
Sent params to client 5: interval=10, num=20
Sent params to client 6: interval=23, num=9
Sent params to client 7: interval=33, num=6
Sent params to client 8: interval=71, num=3
FirstUserTask: All clients started, waiting for completion
Client tid=5, interval=10, completed=1/20, tick=10
Client tid=5, interval=10, completed=2/20, tick=20
Client tid=6, interval=23, completed=1/9, tick=23
Client tid=5, interval=10, completed=3/20, tick=30
Client tid=7, interval=33, completed=1/6, tick=33
Client tid=5, interval=10, completed=4/20, tick=40
Client tid=6, interval=23, completed=2/9, tick=46
Client tid=5, interval=10, completed=5/20, tick=50
Client tid=5, interval=10, completed=6/20, tick=60
Client tid=7, interval=33, completed=2/6, tick=66
Client tid=6, interval=23, completed=3/9, tick=69
Client tid=5, interval=10, completed=7/20, tick=70
Client tid=8, interval=71, completed=1/3, tick=71
Client tid=5, interval=10, completed=8/20, tick=80
Idle: 95%
Client tid=5, interval=10, completed=9/20, tick=90
Client tid=6, interval=23, completed=4/9, tick=92
Client tid=7, interval=33, completed=3/6, tick=99
Client tid=5, interval=10, completed=10/20, tick=100
Client tid=5, interval=10, completed=11/20, tick=110
Client tid=6, interval=23, completed=5/9, tick=115
Client tid=5, interval=10, completed=12/20, tick=120
Client tid=5, interval=10, completed=13/20, tick=130
Client tid=7, interval=33, completed=4/6, tick=132
Client tid=6, interval=23, completed=6/9, tick=138
Client tid=5, interval=10, completed=14/20, tick=140
Client tid=8, interval=71, completed=2/3, tick=142
Client tid=5, interval=10, completed=15/20, tick=150
Client tid=5, interval=10, completed=16/20, tick=160
Client tid=6, interval=23, completed=7/9, tick=161
Idle: 94%
Client tid=7, interval=33, completed=5/6, tick=165
Client tid=5, interval=10, completed=17/20, tick=170
Client tid=5, interval=10, completed=18/20, tick=180
Client tid=6, interval=23, completed=8/9, tick=184
Client tid=5, interval=10, completed=19/20, tick=190
Client tid=7, interval=33, completed=6/6, tick=198
Client 7: Finished all delays
Client tid=5, interval=10, completed=20/20, tick=200
Client 5: Finished all delays
Client tid=6, interval=23, completed=9/9, tick=207
Client 6: Finished all delays
Client tid=8, interval=71, completed=3/3, tick=213
Client 8: Finished all delays
Idle: 97%
Idle: 98%
Idle: 98%
```


### QEMU
- `make sim` to start QEMU simulation
- `Important` BCM system Timer Interrupt not work on the qemu, but arch generic timer works
- Press `Ctrl+A` then `X` to exit the QEMU simulator.


