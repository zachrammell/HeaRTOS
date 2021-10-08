
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

#ifndef		__KERNEL_HAL__H__
#define		__KERNEL_HAL__H__

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include 	"stm32f429xx.h"

/******************************************************************************
*******************************************************************************
    Public Prototypes
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Declarations & Types
*******************************************************************************
******************************************************************************/

//   /
// \/ pt:
//    Left up to the student to figure out what values these might be
//  
//  [4] p.58, core_cmFunc.h in (Proj. Dir.)/CMSIS_4/CMSIS/Include
#define   OS_CRITICAL_BEGIN   __disable_irq() // This needs to disable global interrupts
#define   OS_CRITICAL_END     __enable_irq() // This needs to enable global interrupts
// The RED LED on the Discovery Adapter Board
#define   RED (1 << 0)                
// The GREED LED on the Discovery Adapter Board
#define   GREEN (1 << 1)
// The BLUE LED on the Discovery Adapter Board
#define   BLUE (1 << 2)                     

/******************************************************************************
    OS_SetLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to ON.  This does not turn any LEDs OFF (requires OS_ClearLEDs).
******************************************************************************/
void
OS_SetLEDs (unsigned int);

/******************************************************************************
    OS_ClearLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to OFF.  This does not turn any LEDs ON (requires OS_SetLEDs).
******************************************************************************/
void
OS_ClearLEDs (unsigned int);

/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned int
OS_GetButton (void);

/******************************************************************************
    OS_InitKernelHAL
		
      Prepares the system hardware for use.
******************************************************************************/
void
OS_InitKernelHAL (void);

#endif	//	__KERNEL_HAL__H__

// EOF    Kernel HAL.h
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
