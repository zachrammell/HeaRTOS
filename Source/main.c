/******************************************************************************
*******************************************************************************
  ECE 270 Fall 2017

  main.c
	
  Christopher Theriault
  Coding Assignment #:4

  Copyright DigiPen (USA) Corporation
        All Rights Reserved
	
*******************************************************************************
******************************************************************************/

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

#include "Kernel.h"

#define   ONE_SECOND          0x00081B32

/******************************************************************************
*******************************************************************************
    Helper Functions
*******************************************************************************
******************************************************************************/

void
TaskGreen (void) {
    while (0x01) {
        if (OS_GetButton()) {
            OS_ClearLEDs(GREEN | BLUE);
        } // end if
        else {
            OS_SetLEDs(GREEN);
        } // end else
    } // end while

} // end TaskGreen

void
TaskRed (void) {
    static unsigned int i = 0x00;
    static unsigned int j = 0x00;

    while (0x01) {
        if (j >= ONE_SECOND) {
            if (i != 0x00) {
                i = 0x00;
                OS_ClearLEDs(RED | BLUE);
            } // end if
            else {
                i = 0x01;
                OS_SetLEDs(RED);
            } // end else
        
            j = 0x00;
        } // end if
        else {
            ++j;
        } // end else
    } // end while

} // end TaskRed

/******************************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
******************************************************************************/

void 
main(void) {
    OS_InitKernel();
    OS_CreateTask(&TaskGreen);
    OS_CreateTask(&TaskRed);
    OS_Start();

    while (0x01) {
        ;
    } // end while

} // end main

// EOF    main.c
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
