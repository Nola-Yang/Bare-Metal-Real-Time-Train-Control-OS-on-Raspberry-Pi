# K4

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

### STEP 3.
Enter commands at the prompt:

   - tr <train> <speed> : set train speed (from 0 to 14)
   - sw <switch> <S|C>  : set switch direction
   - rv <train>         : reverse train (stop -> reverse -> restore speed)
   - q/Q                  : quit/reboot
   - init               : init all switch to straight
   - li <train> <1|0>   :turn on/off the train light



### QEMU

> [!WARNING]
> BCM system timer interrupt **does not work on the QEMU**, but arch generic timer does work.
>
> So it is recommended to **test on the actual Raspberry Pi** instead of using QEMU

<br>


- `make sim` to start QEMU simulation
- Press `Ctrl+A` then `X` to exit the QEMU simulator.