
/******************************************************************************
*******************************************************************************
  References:

  [1] RM0090 Reference manual 
  STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 
    advanced ARM®-based 32-bit MCUs

  [2] UM1670 User manual
  Discovery kit with STM32F429ZI MCU

  [3] STM32F427xx/STM32F429xx Datasheet
  ARM Cortex-M4 32b MCU+FPU, 225DMIPS, up to 2MB Flash/256+4KB RAM, USB
    OTG HS/FS, Ethernet, 17 TIMs, 3 ADCs, 20 comm. interfaces, camera & LCD-TFT

  [4] PM0214 Programming manual
  STM32F3, STM32F4 and STM32L4 Series
    Cortex®-M4 programming manual

*******************************************************************************
******************************************************************************/

#include <stdio.h>      // Note:not used, but left in as a reminder
#include <stdlib.h>     //    these libraries are available for use
#include "Kernel HAL.h"

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
    // \/ pt 01:
    //    Complete implementation of function OS_InitKernelHAL.
    OS_InitKernelHAL();

    do {
        ++i;
    //   /
    // \/ pt 12:
    //    When OS_InitKernelHAL executes correctly, uncomment out these
    //    function calls and allow the program to run.  The Red LED should 
    //    flash at a 1-second interval while the Green & Blue LEDs stay ON 
	//    unless the Blue pushbutton is depressed.
        if (OS_GetButton()) {
            OS_ClearLEDs(GREEN | BLUE);
        } // end if
        else {
            OS_SetLEDs(GREEN | BLUE);
        } // end else
    } while (0x01);
} // end main

// EOF    main.c
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
