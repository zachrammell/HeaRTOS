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
    OS_SetLEDs
		
      Parameter is the logical OR of all the colors that should be set
    to ON.  If no colors should be on, pass zero. 
******************************************************************************/
void
OS_SetLEDs (unsigned int);

/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned int
OS_GetButton (void);

#endif	//	__KERNEL_HAL__H__

// EOF
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
