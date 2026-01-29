# K2

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

An image file should be created at `build/kernel.img`

<br>

#### (a) RPS Game Test

Build and run the Rock-Paper-Scissors game test:

```bash
make clean
make
```

This compiles with default settings (MEASURE=0) and runs the RPS test suite, which includes:
- Play without signup
- Quit without signup
- Immediate quitters
- Double signup
- Player plays again
- Early quit scenarios
- Multi-round games

To run the full test suite including all RPS move combinations:

```bash
make clean
make FULLTEST=1
```

#### (b) Performance Measurements

Build and run performance measurements with different configurations:

```bash
make clean
make MEASURE=1
```

##### Configuration Options

The following options can be combined when building:

| Option | Values | Description |
|--------|--------|-------------|
| MEASURE | 0, 1 | Enable performance measurement mode (default: 0) |
| OPT | 0, 1 | Enable O3 optimization (default: 1) |
| CACHE | n, i, d, b | Cache configuration (default: b) |

Cache options:
- `n` = no caches
- `i` = instruction cache only
- `d` = data cache only
- `b` = both caches (default)

##### Example Configurations

Performance test with optimization and both caches (default):
```bash
make clean
make MEASURE=1
```

Performance test without optimization:
```bash
make clean
make MEASURE=1 OPT=0
```

Performance test with no caches:
```bash
make clean
make MEASURE=1 CACHE=n
```

Performance test with instruction cache only:
```bash
make clean
make MEASURE=1 CACHE=i
```

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

Performance measurements output CSV-formatted rows:
```
<opt>,<cache>,<first>,<msg_len>,<avg_time>
```

Where:
- `opt`: "opt" or "noopt"
- `cache`: "bcache", "icache", "dcache", or "nocache"
- `first`: "S" (sender first) or "R" (receiver first)
- `msg_len`: Message length in bytes (4, 64, or 256)
- `avg_time`: Average time in timer ticks

### QEMU
- `make sim` to start QEMU simulation

- Press `Ctrl+A` then `X` to exit the QEMU simulator.



