/*
 * Auther:          Ahmed Mostafa
 * Date:            14/7/2018
 * File Name:       SysTick.h
 * Description:     This file contains:
 *                   -Prototypes of the APIs of the SysTick.
 *                   -Macros for accessing SysTick registers.
 * Microcontroller: TM4C123GH6PM
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "Std_Types.h"
#include "SysTick_Config.h"

/* SysTick Base Addresse */
#define SYSTICK_BASE_ADD  0xE000E000U

/* SysTick Registers Offset */
#define SYSTICK_STCTRL_R_OFFSET    0x010U
#define SYSTICK_STRELOAD_R_OFFSET  0x014U
#define SYSTICK_STCURRENT_R_OFFSET 0x018U

/* SysTick Registers */
#define SYSTICK_STCTRL_R     *((SysTick_RegAddType)(SYSTICK_BASE_ADD+SYSTICK_STCTRL_R_OFFSET))
#define SYSTICK_STRELOAD_R   *((SysTick_RegAddType)(SYSTICK_BASE_ADD+SYSTICK_STRELOAD_R_OFFSET))
#define SYSTICK_STCURRENT_R  *((SysTick_RegAddType)(SYSTICK_BASE_ADD+SYSTICK_STCURRENT_R_OFFSET))

#define DISABLE 0
#define ENABLE  1

/* Datatype volatile (to avoid compiler optimization) and constant (as the addresses are constants) */
typedef volatile uint32* const SysTick_RegAddType;

/* status of the GPIO */
typedef enum
{
    SysTick_OK=0,
    SysTick_NOK
}SysTick_Status;

/* Prototypes of functions */

/* Initialization function of SysTick */
SysTick_Status SysTick_enInit(void);


/* Function to make accurate delay
 * Inputs : -Delay value by Milliseconds
 */
SysTick_Status SysTick_enDelaymSec(uint32 Copy_u32Delayms);


/* Function to make accurate delay
 * Inputs : -Delay value by seconds
 */
SysTick_Status SysTick_enDelaySec(uint32 Copy_u32DelaySec);

#endif /* SYSTICK_H_ */
