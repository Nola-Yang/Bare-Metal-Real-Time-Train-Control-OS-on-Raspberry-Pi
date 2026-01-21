#ifndef TRAPFRAME_H
#define TRAPFRAME_H

#include <stdint.h>


typedef struct trapframe {
    uint64_t x[31];     /* x0-x30 */
    uint64_t sp_el0;    /* user stack */
    uint64_t elr_el1;   /* exception link */
    uint64_t spsr_el1;  /* program status */
} trapframe_t;

_Static_assert(sizeof(trapframe_t) == 272, "trapframe size mismatch");

/*
 * macros for accessing trapframe members
 */
#define TF_X0       0
#define TF_X1       8
#define TF_X2       16
#define TF_X3       24
#define TF_X4       32
#define TF_X5       40
#define TF_X6       48
#define TF_X7       56
#define TF_X8       64
#define TF_X9       72
#define TF_X10      80
#define TF_X11      88
#define TF_X12      96
#define TF_X13      104
#define TF_X14      112
#define TF_X15      120
#define TF_X16      128
#define TF_X17      136
#define TF_X18      144
#define TF_X19      152
#define TF_X20      160
#define TF_X21      168
#define TF_X22      176
#define TF_X23      184
#define TF_X24      192
#define TF_X25      200
#define TF_X26      208
#define TF_X27      216
#define TF_X28      224
#define TF_X29      232   
#define TF_X30      240   
#define TF_SP_EL0   248
#define TF_ELR_EL1  256
#define TF_SPSR_EL1 264
#define TF_SIZE     272

#endif /* TRAPFRAME_H */