
/******************************************************************************
*******************************************************************************
  References:

  [5] Real-Time Operating Systems for
  ARM Cortex-M Microcontrollers

*******************************************************************************
******************************************************************************/
#include "Kernel HAL_asm.h"

  .syntax unified                   // Required to execute Thumb-2 code

  .global OS_Start                  // Required to expose function to .c files
  .global TIM6_DAC_IRQHandler
  .extern OS_TaskRUNNING            // Defined in Kernel.c
  .extern OS_TaskNEW
  .extern OSp_ScheduleTask
  .extern OS_SetLEDs
  .extern OS_ClearLEDs

  .text                    // Where to locate this code
  .align 0

  .thumb_func
OS_Start:
    //   /
    // \/ pt 06:
    //    Implement the StartOS assembly code from the Valvano text, but 
    //  replace RunPt with OS_TaskRUNNING.  Place a breakpoint on the bx
    //  instruction and verify that the registers are getting loaded with 
    //  the data you manually placed in the stack for the task.  If it isn't,
    //  go back and fix OS_TaskCreate.  Then, place a breakpoint in the 
    //  Idle task.  After the bx command executes, you should be running
    //  inside the Idle task.  When the code is free-running, the Red LED
    //  should continue to flash as before while program execution remains in
    //  the Idle task.  Note that pure assembly functions don't require any
    //  () or {} around them- code just begins and ends.
    //
    // [5] p.177 (3rd Ed.)

    LDR R0, =OS_TaskRUNNING // currently running thread
    LDR R1, [R0]            // R1 = value of OS_TaskRUNNING
    LDR SP, [R1]            // new thread SP// SP = RunPt->sp//
    POP {R4-R11}            // restore regs r4-11
    POP {R0-R3}             // restore regs r0-3
    POP {R12}
    ADD SP, SP, #4          // discard LR from initial stack
    POP {LR}                // start location
    ADD SP, SP, #4          // discard PSR
    CPSIE I                 // Enable interrupts at processor level
    BX LR                   // start first thread

  .bss

  .align 2
microsecondsElapsed: .word 0
redLedEnabled: .word 0

  .text

  .weak TIM6_DAC_IRQHandler
  .thumb_func
TIM6_DAC_IRQHandler:
  CPSID I
  PUSH {R4-R11}
  LDR R4, =0x40001010 // load address of timer 6 SR
  LDR R5, [R4]
.TIM6_DAC_IRQHandler_ClearInterrupt:
  BIC R5, R5, #1 // clear the needed bit
  STR R5, [R4]
  LDR R5, [R4] // load again to check
  TST R5, #1
  BNE .TIM6_DAC_IRQHandler_ClearInterrupt
  
  LDR R0, =OS_TaskRUNNING
  LDR R1, [R0]
  STR SP, [R1]
  PUSH {R0, LR}
  BL OSp_ScheduleTask
  POP {R0, LR}
  LDR R1, [R0]
  LDR SP, [R1]

  LDR R0, =OS_TaskRUNNING // R0: ptr(ptr(TCB)) = &OS_TaskRunning
  LDR R2, [R0]            // R2: ptr(TCB) = *R0
  // OS_TaskRUNNING→taskState = READY;
  MOV R1, #OS_READY       // R1: word = OS_READY
  STR R1, [R2, #8]        // R2->taskState = R1
  // OS_TaskRUNNING→sp = sp;
  STR SP, [R2, #0]        // R2->sp = SP
  // OS_TaskRUNNING = OS_TaskNEW;
  LDR R1, =OS_TaskNEW     // R1: ptr(ptr(TCB)) = &OS_TaskNEW
  LDR R3, [R1]            // R3: ptr(TCB) = *R1
  STR R3, [R0]            // *R0 = R3
  LDR R2, [R0]            // R2: ptr(TCB) = *R0
  // OS_TaskRUNNING→taskState = RUNNING
  MOV R1, #OS_RUNNING     // R1: word = OS_RUNNING
  STR R1, [R2, #8]        // R2->taskState = R1
  // sp = OS_TaskRUNNING→sp;
  LDR SP, [R2, #0]        // SP = R2->sp

.TIM6_DAC_IRQHandler_End:
  CPSIE I
  POP {R4-R11}                 // return
  BX LR

 //EOF    Kernel HAL_asm.S
 //Note: Some IDEs generate warnings if a file doesn't end in whitespace,
  //but Embedded Studio doesn't seem to be one of them.
