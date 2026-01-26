/*
 * Design:
 *   - TPIDR_EL1 holds pointer to current task's TD (= trapframe address)
 *   - Single kernel stack at __kernel_stack_top (add definition in linker script, in original boot.S, at 0x80000)
 *   - Per-task trapframe stored in TD at offset 0
 */

.section ".text"

/*
 * Trapframe offsets 
 */
.equ TF_X0,       0
.equ TF_X1,       8
.equ TF_X2,       16
.equ TF_X3,       24
.equ TF_X4,       32
.equ TF_X5,       40
.equ TF_X6,       48
.equ TF_X7,       56
.equ TF_X8,       64
.equ TF_X9,       72
.equ TF_X10,      80
.equ TF_X11,      88
.equ TF_X12,      96
.equ TF_X13,      104
.equ TF_X14,      112
.equ TF_X15,      120
.equ TF_X16,      128
.equ TF_X17,      136
.equ TF_X18,      144
.equ TF_X19,      152
.equ TF_X20,      160
.equ TF_X21,      168
.equ TF_X22,      176
.equ TF_X23,      184
.equ TF_X24,      192
.equ TF_X25,      200
.equ TF_X26,      208
.equ TF_X27,      216
.equ TF_X28,      224
.equ TF_X29,      232
.equ TF_X30,      240
.equ TF_SP_EL0,   248
.equ TF_ELR_EL1,  256
.equ TF_SPSR_EL1, 264
.equ TF_SIZE,     272


.macro VECTOR_ENTRY label
    .balign 128
\label:
.endm


.balign 2048
.global exception_vector_table
exception_vector_table:


VECTOR_ENTRY curr_el_sp0_sync
    b unhandled_exception

VECTOR_ENTRY curr_el_sp0_irq
    b unhandled_exception

VECTOR_ENTRY curr_el_sp0_fiq
    b unhandled_exception

VECTOR_ENTRY curr_el_sp0_serror
    b unhandled_exception

VECTOR_ENTRY curr_el_spx_sync
    b unhandled_exception

VECTOR_ENTRY curr_el_spx_irq
    b unhandled_exception

VECTOR_ENTRY curr_el_spx_fiq
    b unhandled_exception

VECTOR_ENTRY curr_el_spx_serror
    b unhandled_exception

VECTOR_ENTRY lower_el_a64_sync
    b svc_entry

VECTOR_ENTRY lower_el_a64_irq
    b unhandled_exception

VECTOR_ENTRY lower_el_a64_fiq
    b unhandled_exception

VECTOR_ENTRY lower_el_a64_serror
    b unhandled_exception

VECTOR_ENTRY lower_el_a32_sync
    b unhandled_exception

VECTOR_ENTRY lower_el_a32_irq
    b unhandled_exception

VECTOR_ENTRY lower_el_a32_fiq
    b unhandled_exception

VECTOR_ENTRY lower_el_a32_serror
    b unhandled_exception


unhandled_exception:
    b .


/*
 * Kernel stack overflow handler
 */
kernel_stack_overflow:
    b .


/*
 * Entry conditions:
 *   - Running in EL1 using SP_EL1
 *   - ELR_EL1 = user return address
 *   - SP_EL0 = user stack pointer
 *   - TPIDR_EL1 = pointer to current task's TD (= trapframe)
 */
.global svc_entry
svc_entry:
    
    stp x0, x1, [sp, #-16]!

    // Check kernel stack bounds
    ldr x0, =__kernel_stack_bottom
    cmp sp, x0
    b.lt kernel_stack_overflow  // SP < stack_bottom means overflow

    mrs x0, tpidr_el1           // x0 = current task = trapframe pointer

    // Save x2-x30 
    stp x2, x3, [x0, #TF_X2]
    stp x4, x5, [x0, #TF_X4]
    stp x6, x7, [x0, #TF_X6]
    stp x8, x9, [x0, #TF_X8]
    stp x10, x11, [x0, #TF_X10]
    stp x12, x13, [x0, #TF_X12]
    stp x14, x15, [x0, #TF_X14]
    stp x16, x17, [x0, #TF_X16]
    stp x18, x19, [x0, #TF_X18]
    stp x20, x21, [x0, #TF_X20]
    stp x22, x23, [x0, #TF_X22]
    stp x24, x25, [x0, #TF_X24]
    stp x26, x27, [x0, #TF_X26]
    stp x28, x29, [x0, #TF_X28]
    str x30, [x0, #TF_X30]     

    
    ldp x2, x3, [sp], #16       // x2 = original x0, x3 = original x1
    stp x2, x3, [x0, #TF_X0]    

    mrs x1, sp_el0
    str x1, [x0, #TF_SP_EL0]    

    mrs x1, elr_el1
    str x1, [x0, #TF_ELR_EL1]   

    mrs x1, spsr_el1
    str x1, [x0, #TF_SPSR_EL1] 

    // Switch to kernel stack
    ldr x1, =__kernel_stack_top
    mov sp, x1

    bl syscall_dispatch



/*
 * restores the CPU state from the current task's
 * trapframe and returns to user mode via ERET.
 */
.global return_to_task
return_to_task:
    mrs x0, tpidr_el1

    ldr x1, [x0, #TF_SPSR_EL1]
    msr spsr_el1, x1            
    ldr x1, [x0, #TF_ELR_EL1]
    msr elr_el1, x1             
    ldr x1, [x0, #TF_SP_EL0]
    msr sp_el0, x1              


    ldp x2, x3, [x0, #TF_X2]
    ldp x4, x5, [x0, #TF_X4]
    ldp x6, x7, [x0, #TF_X6]
    ldp x8, x9, [x0, #TF_X8]
    ldp x10, x11, [x0, #TF_X10]
    ldp x12, x13, [x0, #TF_X12]
    ldp x14, x15, [x0, #TF_X14]
    ldp x16, x17, [x0, #TF_X16]
    ldp x18, x19, [x0, #TF_X18]
    ldp x20, x21, [x0, #TF_X20]
    ldp x22, x23, [x0, #TF_X22]
    ldp x24, x25, [x0, #TF_X24]
    ldp x26, x27, [x0, #TF_X26]
    ldp x28, x29, [x0, #TF_X28]
    ldr x30, [x0, #TF_X30]      

    // Reset SP_EL1 to kernel stack top
    ldr x1, =__kernel_stack_top
    mov sp, x1

    ldr x1, [x0, #TF_X1]
    ldr x0, [x0, #TF_X0]        // x0 = return value

    eret


.global activate_first_task
activate_first_task:
    b return_to_task
