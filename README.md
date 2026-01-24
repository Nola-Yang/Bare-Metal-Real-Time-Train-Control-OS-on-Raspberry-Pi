# K1

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
git clone https://git.uwaterloo.ca/cs-452-nolex/k1.git
```

<br>

### STEP 2:
Checkout the specific commit hash for the submission.
The commit hash to checkout is: `69a750144650f4bf2928072201993eb1a0306170`

```bash
git checkout <commit_hash>
```

<br>

### STEP 3:
Compile the program by running:

```bash
make
```

<br>

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
