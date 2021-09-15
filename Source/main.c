/******************************************************************************
*******************************************************************************
  References:

  [1] RM0090 Reference manual 
  STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 
    advanced ARM®-based 32-bit MCUs

  [2] UM1670 User manual
  Discovery kit with STM32F429ZI MCU
*******************************************************************************
******************************************************************************/

#include <stdio.h>      // Note:not used, but left in as a reminder
#include <stdlib.h>     //    these libraries are available for use
#include "Kernel HAL.h"

extern void
OSp_InitGPIOG (void);

extern void
OSp_InitGPIOA (void);

/******************************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
******************************************************************************/
void 
main(void) {
    unsigned int i;

    //   /
    // \/ pt:
    //    Complete these two functions in Kernel HAL.c before proceeding.
    //    Place a breakpoint in the do-while loop; the Green LED should be ON
    //    when program execution pauses at the breakpoint.
    OSp_InitGPIOG();
    OS_SetLEDs(0x01);

    //   /
    // \/ pt:
    //    When the Green LED successfully turns on, uncomment this function
    //    and implement it in Kernel HAL.c.
    OSp_InitGPIOA();

    do {
        ++i;
    //   /
    // \/ pt:
    //    When OSp_InitGPIOA is implemented, uncomment this line and implement
    //    OS_GetButton in Kernel HAL.c.
        OS_SetLEDs(OS_GetButton());
    } while (0x01);
} // end main

// EOF
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
