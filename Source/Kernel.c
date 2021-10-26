
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
    
  [5] Real-Time Operating Systems for
  ARM Cortex-M Microcontrollers

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
#include 	"Kernel.h"

/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/

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
OS_IdleTask(void);

/******************************************************************************
*******************************************************************************
    Declarations & Types
*******************************************************************************
******************************************************************************/
typedef	void (* OS_TaskAddress)(void);	

typedef enum
{
  READY = OS_READY,
  RUNNING = OS_RUNNING,
} taskState_t;

typedef struct {
    unsigned int	*sp;
    unsigned int	taskID;
    taskState_t		taskState;
    OS_TaskAddress	taskAddress;
} TCB;	

// Globals used by the system
TCB             Tasks[OS_MAX_TASKS];
TCB             *OS_TaskNEW;
TCB             *OS_TaskRUNNING;
unsigned char 	newTaskID;
unsigned int 	Stacks[OS_MAX_TASKS][OS_STACK_SIZE];

/******************************************************************************
*******************************************************************************
    Helper Functions
*******************************************************************************
******************************************************************************/

void OSp_ScheduleTask(void)
{
  static int toggle = 0;
  OS_TaskNEW = &Tasks[1 + toggle];
  toggle = !toggle;
}

/******************************************************************************
    OS_InitKernel
		
      Prepares the Kernel for use, but does not start any services.  No OS_
    function should be called until after this one has executed.
******************************************************************************/    
void    
OS_InitKernel(void) {
    //   /
    // \/ pt 02:
    //    Complete functions OS_InitKernelHAL and OS_TaskCreate.
    OS_InitKernelHAL();
    OS_CreateTask(OS_IdleTask);
    OS_TaskRUNNING = &Tasks[0];
    
} // end OS_InitKernel

/******************************************************************************
    OS_TaskCreate
		
      Takes the assigned function pointer and uses it to create a kernel task
    that is ready for execution.
******************************************************************************/
void
OS_CreateTask(void (* newTask)(void)) {
    static unsigned int newTaskID = 0;

    //   /
    // \/ pt 04:
    //    This function has been passed a pointer to a function that we wish 
    //  to execute as a thread.  Examine the definition of the TCB structure
    //  contained in this file.  The code in this function must do the  
    //  following things:
    //  1) Store the passed-in function pointer inside the
    //      TCB structure and the TCB's stack.

    TCB* newTCB = &Tasks[newTaskID];
    unsigned int* stack = &Stacks[newTaskID];

    newTCB->taskAddress = newTask;
    stack[OS_STACK_SIZE - 3] = newTask;

    //  2) Assign the correct ID number to the new task.  Note that newTaskID
    //      is both a task ID and an index into the global array of tasks.
    newTCB->taskID = newTaskID;

    //  3) Set the task state to READY.
    newTCB->taskState = READY;

    //  4) Pre-load the task's stack with the correct starting data.  Examine
    //    page 174* of the Valvano text very carefully to see what must be set,
    //    and where.  However, the Valvano code needs to be modified slightly.
    //    The top of the stack will be marked with a special value so that
    //    stack overruns can be detected.  This means that your code should 
    //    write our marker value (0xDEADBEEF) into location `stack size - 0x01`
    //    and all other locations need to have their locations in the stack
    //    incremented by +1 accordingly. 

    
    stack[OS_STACK_SIZE -  1] = OS_STACK_MARKER;
    stack[OS_STACK_SIZE -  2] = 0x01000000; // Thumb bit
    stack[OS_STACK_SIZE -  4] = 0x14141414; // LR
    stack[OS_STACK_SIZE -  5] = 0x12121212; // R12
    stack[OS_STACK_SIZE -  6] = 0x03030303; // R3
    stack[OS_STACK_SIZE -  7] = 0x02020202; // R2
    stack[OS_STACK_SIZE -  8] = 0x01010101; // R1
    stack[OS_STACK_SIZE -  9] = 0x00000000; // R0
    stack[OS_STACK_SIZE - 10] = 0x11111111; // R11
    stack[OS_STACK_SIZE - 11] = 0x10101010; // R10
    stack[OS_STACK_SIZE - 12] = 0x09090909; // R9
    stack[OS_STACK_SIZE - 13] = 0x08080808; // R8
    stack[OS_STACK_SIZE - 14] = 0x07070707; // R7
    stack[OS_STACK_SIZE - 15] = 0x06060606; // R6
    stack[OS_STACK_SIZE - 16] = 0x05050505; // R5
    stack[OS_STACK_SIZE - 17] = 0x04040404; // R4

    //  5) Set the stack pointer value in the TCB to set to the starting 
    //    location of valid stack data (which will be `stack size - 0x11`.
    newTCB->sp = &stack[OS_STACK_SIZE - 17]; // thread stack pointer

    //  6) Lastly, the value of newTaskID should be updated.
    ++newTaskID;

    //  NOTE: To make it easier to follow the book, the rule against 
    //    `magic numbers` in the code is lifted FOR THIS FUNCTION ONLY
    //
	// * 3rd edition



} // end OS_TaskCreate

/******************************************************************************
    OS_IdleTask
		
      This task should always be created, have the lowest possible (worst) 
    priority, and never be prevented from running.
******************************************************************************/
void
OS_IdleTask(void) {
    while (0x01) {
      ;
    } // end while

} // end OS_IdleTask



// EOF    Kernel.c
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.
