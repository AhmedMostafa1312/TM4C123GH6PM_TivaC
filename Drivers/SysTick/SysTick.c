/*
 * Auther:          Ahmed Mostafa
 * Date:            14/7/2018
 * File Name:       SysTick.c
 * Description:     This file contains:
 *                   -Implementation of the functions that is used to access SysTick register.
 * Microcontroller: TM4C123GH6PM
 */

#include "SysTick.h"
#include "GPIO.h"

/* Initialization function of SysTick */
SysTick_Status SysTick_enInit(void)
{
    /* Variable to indicate the success of SysTick configuration */
    SysTick_Status Local_enReturnValue=SysTick_OK;

    /* Disable SysTick for the initialization */
    SYSTICK_STCTRL_R=0;

    /* Enable system clock */
    SYSTICK_STCTRL_R|=0x04;

#if(SYSTICK_INT_ENABLE==ENABLE)
    /* Enable SysTick interrupt */
    SYSTICK_STCTRL_R|=0x02;

#else
    /* Disable SysTick interrupt */
    SYSTICK_STCTRL_R&=~0x02;

#endif

    /* Clear SysTick current value */
    SYSTICK_STCURRENT_R=0;

    Local_enReturnValue=SysTick_OK;

    return Local_enReturnValue;
}

/* Function to make accurate delay
 * Inputs : -Delay value by msec
 */
SysTick_Status SysTick_enDelaymSec(uint32 Copy_u32Delayms)
{
    uint32 Local_u32SysTickReloadValue;

    /* Variable to indicate the success of SysTick configuration */
    SysTick_Status Local_enReturnValue=SysTick_OK;

    /* Clear SysTick current value to clear count flag */
    SYSTICK_STCURRENT_R=0;

    /* Calculate the reload value */
    Local_u32SysTickReloadValue=(((Copy_u32Delayms)/1000.0)/(1.0/16000000.0))-1;

    /* Assign the reload value to SysTick reload register */
    SYSTICK_STRELOAD_R=Local_u32SysTickReloadValue;

    /* Start SysTick */
    SYSTICK_STCTRL_R|=0x01;

#if(SYSTICK_INT_ENABLE==DISABLE)
    /* Polling on the count flag to be one */
    while(((SYSTICK_STCTRL_R>>16)&0x01)==0);
#endif

    Local_enReturnValue=SysTick_OK;

    return Local_enReturnValue;
}

/* Function to make accurate delay
 * Inputs : -Delay value by sec
 */
SysTick_Status SysTick_enDelaySec(uint32 Copy_u32DelaySec)
{
    uint32 Local_u32LoopIndex=0;

    /* Variable to indicate the success of SysTick configuration */
    SysTick_Status Local_enReturnValue=SysTick_OK;

    for(Local_u32LoopIndex=0;Local_u32LoopIndex<(60*Copy_u32DelaySec);++Local_u32LoopIndex)
    {
        SysTick_enDelaymSec(1000);
    }

    Local_enReturnValue=SysTick_OK;

    return Local_enReturnValue;
}

/* SysTick ISR
 * It must be configured in startup source file
 */
void SysTick_ISR(void)
{

}
