
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

/*
        This code provides the Hardware Abstraction Layer (HAL) for the
    kernel.  This HAL only supports the STM32F429ZI microcontroller. 
*/

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include 	"Kernel HAL.h"
#include 	"stm32f429xx.h"

/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/

#define   UINT32_ALL_ON       0xFFFFFFFF
#define   POS_07              0x07
#define   POS_13              0x0D
#define   POS_14              0x0E
#define   POS_16              0x10
#define   POS_17              0x11
#define   POS_18              0x12
#define   POS_19              0x13
#define   POS_26              0x1A
#define   POS_27              0x1B
#define   POS_28              0x1C
#define   POS_29              0x1D
#define   BIT_00              0x01
#define   BIT_01              0x02
#define   BIT_02              0x04
#define   BIT_03              0x08
#define   BIT_04              0x10
#define   BIT_07              (0x01<<POS_07)
#define   BIT_13              (0x01<<POS_13)
#define   BIT_14              (0x01<<POS_14)
#define   BIT_16              (0x01<<POS_16)
#define   BIT_17              (0x01<<POS_17)
#define   BIT_18              (0x01<<POS_18)
#define   BIT_19              (0x01<<POS_19)
#define   BIT_26              (0x01<<POS_26)
#define   BIT_27              (0x01<<POS_27)
#define   BIT_28              (0x01<<POS_28)
#define   BIT_29              (0x01<<POS_29)

#define   ONE_MS        1
#define   ONE_SEC       1000
#define   TIM6_IRQ      54 // [1] table 62 (STM32F42xxx version)            
#define   TIMER_CYCLES_PER_MS 16000

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

void
TIM6_DAC_IRQHandler (void);

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
    attached to the Red/Green LEDs have their digital outputs enabled. 
******************************************************************************/
void
OSp_InitGPIOG (void) {
    // Enable the system clock for the PORTG peripheral
    // [1] pp.180-1
    RCC->AHB1ENR BON(RCC_AHB1ENR_GPIOGEN);

    // Wait for the system clock to stabilize
    for (int wait = 0x00; wait < MAX_WAIT; ) {
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
    // Enable the system clock for the PORTA peripheral
    // [1] pp.180-1
    RCC->AHB1ENR BON(RCC_AHB1ENR_GPIOAEN);

    // Wait for the system clock to stabilize
    for (int wait = 0x00; wait < MAX_WAIT; ) {
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
    OSp_InitTIM6
		
      The clock to the TIM6 module is enabled and configured to run at a 1ms
    cycle, causing an interrupt if global interrupts are enabled. 
******************************************************************************/
void
OSp_InitTIM6 (void) {
    volatile unsigned int wait = MAX_WAIT;	

    // Enable the system clock for the TIM6 peripheral
    // [1] p.183
    RCC->APB1ENR BON(RCC_APB1ENR_TIM6EN);

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Enable auto-reload
    // [1] p.704
    TIM6->CR1 BON(TIM_CR1_ARPE);

    // Ensure counter is free-running (does not stop counting)
    // [1] p.705
    TIM6->CR1 BOFF(TIM_CR1_OPM);

    // Only over/underflow causes interrupts
    // [1] p.705
    TIM6->CR1 BON(TIM_CR1_URS);
    
    // Interrupt on over/underflow enabled
    // [1] p.705
    TIM6->DIER BON(TIM_DIER_UDE);
    // [1] p.706
    TIM6->DIER BON(TIM_DIER_UIE);

    // No prescaler used
    // [1] pp.699, 708
    TIM6->PSC = 0x00;

    // Value for auto-reload register (sets the timer duration)
    // !!! BE SURE YOU USE A VALUE THAT CAUSES THE TIMER TO ROLLOVER
    //     AT 1 millisecond INTERVALS !!!
    // Note that the peripheral clock is running at 16MHz
    // [1] pp.699, 701, 708
    TIM6->ARR = TIMER_CYCLES_PER_MS;

    // Set the timer to the lowest-numbered (best-possible) priority
    // [4] pp.208, 214, core_cm4.h in (Proj. Dir.)/CMSIS_4/CMSIS/Include
    NVIC_SetPriority(TIM6_IRQ, 0);

    // Timer is ON
    // [1] p.705
    TIM6->CR1 BON(TIM_CR1_CEN);

    // Enable IRQ for TIM6 in the NVIC
    // [4] p.208, 214, core_cm4.h in (Proj. Dir.)/CMSIS_4/CMSIS/Include
    NVIC_SetVector(TIM6_IRQ, (uint32_t)TIM6_DAC_IRQHandler);
    NVIC_EnableIRQ(TIM6_IRQ);
} // end OSp_InitTIM6

/******************************************************************************
    OS_SetLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to ON.  This does not turn any LEDs OFF (requires OS_ClearLEDs).
******************************************************************************/
void
OS_SetLEDs (unsigned int LEDs) {
    if (LEDs & RED)
    {
        GPIOG->ODR BON(GPIO_ODR_OD2);
    }
    if (LEDs & GREEN)
    {
        GPIOG->ODR BON(GPIO_ODR_OD3);
    }
    if (LEDs & BLUE)
    {
        GPIOG->ODR BON(GPIO_ODR_OD9);
    }
} // end OS_SetLEDs

/******************************************************************************
    OS_ClearLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to OFF.  This does not turn any LEDs ON (requires OS_SetLEDs).
******************************************************************************/
void
OS_ClearLEDs (unsigned int LEDs) {
    if (LEDs & RED)
    {
        GPIOG->ODR BOFF(GPIO_ODR_OD2);
    }
    if (LEDs & GREEN)
    {
        GPIOG->ODR BOFF(GPIO_ODR_OD3);
    }
    if (LEDs & BLUE)
    {
        GPIOG->ODR BOFF(GPIO_ODR_OD9);
    }
} // end OS_ClearLEDs

/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned int
OS_GetButton (void) {
    return !(GPIOA->IDR & GPIO_IDR_ID0);
} // end OS_GetButton

/******************************************************************************
    OS_InitKernelHAL
		
      Prepares the system hardware for use.
******************************************************************************/
void
OS_InitKernelHAL (void) {
    OSp_InitGPIOG();
    OSp_InitGPIOA();
    OSp_InitTIM6();

} // end OS_InitKernelHAL

// EOF    Kernel HAL.c
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
