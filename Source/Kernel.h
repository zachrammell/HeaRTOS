
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

#ifndef		__KERNEL__H__
#define		__KERNEL__H__

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include 	"Kernel HAL.h"

/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/
#define   OS_STACK_SIZE       0x40
#define   OS_MAX_TASKS        0x04
#define   OS_STACK_MARKER     0xDEADBEEF

/******************************************************************************
*******************************************************************************
    Public Prototypes
*******************************************************************************
******************************************************************************/

/******************************************************************************
    OS_InitKernel
		
      Prepares the Kernel for use, but does not start any services.  No OS_
    function should be called until after this one has executed.
******************************************************************************/    
void    
OS_InitKernel(void);

/******************************************************************************
    OS_CreateTask
		
      Takes the assigned function pointer and uses it to create a kernel task
    that is ready for execution.
******************************************************************************/
void
OS_CreateTask(void (*)(void));

/******************************************************************************
    OS_Start
		
      Begins kernel services and starts execution of the highest priority task.
******************************************************************************/
extern void
OS_Start(void);

#endif	//	__KERNEL__H__

// EOF    Kernel HAL.h
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
