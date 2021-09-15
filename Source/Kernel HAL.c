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
#define   UINT32_ALL_ON       0xFFFFFFFF
#define   POS_13              0x0D
#define   POS_26              0x1A
#define   BIT_00              0x01
#define   BIT_01              0x02
#define   BIT_13              (0x01<<POS_13)
#define   BIT_26              (0x01<<POS_26) 

// Optional definitions not required to produce working code
#ifndef			MAX_WAIT
	#define		BON(X)			|=(X)
	#define		BOFF(X)			&=~(X)
	#define		BTOG(X)			^=(X)
	#define		MAX_WAIT		0xFFFF
#endif		//	MAX_WAIT

/******************************************************************************
*******************************************************************************
    Prototypes
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Declarations & Types
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Helper Functions
*******************************************************************************
******************************************************************************/

/******************************************************************************
    OSp_InitGPIOG
		
      The clock to the PORTG module is enabled and the two pins 
    attached to the red/green LEDs have their digital outputs enabled. 
******************************************************************************/
void
OSp_InitGPIOG (void) {
    volatile unsigned int wait = MAX_WAIT;	
	
    // Identify which pin/port connects to the Green user LED
    // [2] (Note: no code here, just something you need to do)
    // port G, pin 3

    // Enable the system clock for the PORTG peripheral
    // [1] pp.180-1
    RCC->AHB1ENR BON(RCC_AHB1ENR_GPIOGEN);

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Configure the desired I/O as output or input in the GPIOx_MODER register
    // [1] p.273, [1] p.283, [1] table 35 (p. 270)
    //GPIOG->MODER BON(6);
    GPIOG->MODER BON(GPIO_MODER_MODE3_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE9_0);

    // Set the output type.  Note: remember that `open drain` requires a 
    //    pull-up or pull-down resistor attached to the I/O port
    // [1] p.283, [1] table 35
    GPIOG->OTYPER BON(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT9);

    // Set the speed
    // [1] p.284, [1] table 35
    GPIOG->OSPEEDR BOFF(GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED9);

    // Disable pull up/down
    // [1] pp.284-5, [1] table 35
    GPIOG->PUPDR BOFF(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD9);

} // end OSp_InitGPIOG

/******************************************************************************
    OSp_InitGPIOA
		
      The clock to the PORTA module is enabled and the pin
    attached to the Blue pushbutton has its digital input enabled. 
******************************************************************************/
void
OSp_InitGPIOA (void) {
    volatile unsigned int wait = MAX_WAIT;	
	
    // Identify which pin/port connects to the Blue pushbutton
    // [2] (Note: no code here, just something you need to do)
    // pin A, port 0

    // Enable the system clock for the PORTA peripheral
    // [1] pp.180-1
    RCC->AHB1ENR BON(RCC_AHB1ENR_GPIOAEN);

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Configure the desired I/O as output or input in the GPIOx_MODER register
    // [1] p.273, [1] p.283, [1] table 35
    GPIOA->MODER BOFF(GPIO_MODER_MODE0);

    // Set the speed
    // [1] p.284, [1] table 35
    GPIOA->OSPEEDR BOFF(GPIO_OSPEEDR_OSPEED0);

    // Disable pull up/down
    // [1] pp.284-5, [1] table 35
    GPIOA->PUPDR BOFF(GPIO_PUPDR_PUPD0);

} // end OSp_InitGPIOA

/******************************************************************************
    OS_SetLEDs
		
      Parameter is the logical OR of all the colors that should be set
    to ON.  If no colors should be on, pass zero. 
******************************************************************************/
void
OS_SetLEDs (unsigned int LEDs) {
    // Note:In this assignment, turn the Green LED on if passed a nonzero 
    //    value.  In future assignments, this functionality will be expanded. 

    if (LEDs) {
        // Set the LED value to ON - WARNING: Active-LOW Circuit!!!
        // [1] p.285
        GPIOG->ODR BON(GPIO_ODR_OD2);
        GPIOG->ODR BON(GPIO_ODR_OD3);
        GPIOG->ODR BON(GPIO_ODR_OD9);
    } // end if
    else {
        // Set the LED value to OFF - WARNING: Active-LOW Circuit!!!
        // [1] p.285
        GPIOG->ODR BOFF(GPIO_ODR_OD2);
        GPIOG->ODR BOFF(GPIO_ODR_OD3);
        GPIOG->ODR BOFF(GPIO_ODR_OD9);
    } // end else

} // end OS_SetLEDs

/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned int
OS_GetButton (void) {
    // Send back the value of the bit in the input register assigned to the 
    //    Blue pushbutton on the STM32 board
    return !(GPIOA->IDR & GPIO_IDR_ID0);

} // end OS_GetButton


// EOF
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
