
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
  PUSH {R4-R7, IP, LR}
  LDR R4, =0x40001010 // load address of timer 6 SR
  LDR R5, [R4]
.TIM6_DAC_IRQHandler_ClearInterrupt:
  BIC R5, R5, #1 // clear the needed bit
  STR R5, [R4]
  LDR R5, [R4] // load again to check
  TST R5, #1
  BNE .TIM6_DAC_IRQHandler_ClearInterrupt
  
  LDR R6, =microsecondsElapsed// R6 := address(microsecondsElapsed)
  LDR R5, [R6]                // R5 := deref(R6)
  ADD R5, R5, #1              // R5 += 1
  CMP R5, #1000               // if (R5 < 1000) 
  BCS .TIM6_DAC_IRQHandler_MillisecondPassed
  STR R5, [R6]                // deref(R6) = R5
  B .TIM6_DAC_IRQHandler_End  // return

.TIM6_DAC_IRQHandler_MillisecondPassed:
  LDR R5, [R6, #4]      // R5 := deref(address(redLedEnabled))
  MOV R7, #0            // R7 = 0
  MOV R0, #1            // R0 := RED
  CMP R7, R5
  ITEE EQ
  MOVEQ R4, #1          // if (!R5) R4 = 1
  MOVNE R4, #0          // if (R5)  R4 = 0
  BLNE OS_SetLEDs       // if (R5)  OS_SetLEDs(R0)
  CMP R7, R5
  IT EQ
  BLEQ OS_ClearLEDs     // if (!R5) OS_ClearLEDs(R0)
  STR R4, [R6, #4]      // deref(address(redLedEnabled)) = R4
  STR R7, [R6]          // deref(address(microsecondsElapsed)) = R7
.TIM6_DAC_IRQHandler_End:
  CPSIE I
  POP {R4-R7, IP, PC}                 // return

 //EOF    Kernel HAL_asm.S
 //Note: Some IDEs generate warnings if a file doesn't end in whitespace,
  //but Embedded Studio doesn't seem to be one of them.
