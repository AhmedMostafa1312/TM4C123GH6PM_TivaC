/*
 * Auther:          Ahmed Mostafa
 * Date:            26/6/2018
 * File Name:       GPIO.c
 * Description:     This file contains:
 *                   -Implementation of the functions that is used to access GPIO register.
 * Microcontroller: TM4C123GH6PM
 */

#include "GPIO.h"

/* static array of the GPIO groups that is filled by user 0 -> initialized , 1 -> not initialized */
static uint8 Global_u8GroupStatusArray[GPIO_NUM_OF_GROUPS]={0};

/* Array of the GPIO ports (APB) base addresses */
static const uint32 Global_u32PortxAPBBaseAddArray[GPIO_NUM_OF_PORTS]=
{
    GPIO_PORTA_APB_BASE_ADD,
    GPIO_PORTB_APB_BASE_ADD,
    GPIO_PORTC_APB_BASE_ADD,
    GPIO_PORTD_APB_BASE_ADD,
    GPIO_PORTE_APB_BASE_ADD,
    GPIO_PORTF_APB_BASE_ADD
};

/* Array of the GPIO ports (AHB) base addresses */
static const uint32 Global_u32PortxAHBBaseAddArray[GPIO_NUM_OF_PORTS]=
{
    GPIO_PORTA_AHB_BASE_ADD,
    GPIO_PORTB_AHB_BASE_ADD,
    GPIO_PORTC_AHB_BASE_ADD,
    GPIO_PORTD_AHB_BASE_ADD,
    GPIO_PORTE_AHB_BASE_ADD,
    GPIO_PORTF_AHB_BASE_ADD
};

/* Initialization function of GPIO ports */
GPIO_Status GPIO_enInit(void)
{
    /* For loop Local_u8LoopIndex */
    uint8 Local_u8LoopIndex;

    /* Variable to indicate the success of GPIO configuration */
    GPIO_Status Local_enReturnValue=GPIO_OK;

    /* Pointer to GPIO Configuration structure */
    const GPIO_ConfigStrType* Local_strConfigPtr;

    /* For loop on the number of GPIO groups in the array of structures */
    for(Local_u8LoopIndex=0;(Local_u8LoopIndex<GPIO_NUM_OF_GROUPS)&&(Local_enReturnValue==GPIO_OK);Local_u8LoopIndex++)
    {
        /* Check on the validity of the port ID that is provided by user */
        if(Gobal_strConfigArray[Local_u8LoopIndex].GPIO_PortID<GPIO_NUM_OF_PORTS)
        {
            /* Assign the pointer to the Configuration array of structures */
            Local_strConfigPtr=&Gobal_strConfigArray[Local_u8LoopIndex];

            /* Enable Clock Gating Control for the provided port */
            RCGC_GPIO_R_BASE_ADD|=(HIGH<<(Local_strConfigPtr->GPIO_PortID));

            /* Enable the type of GPIO bus ABP ->low speed or AHB -> high speed*/
            GPIO_HBCTL_R_BASE_ADD|=((Local_strConfigPtr->GPIO_Speed)<<(Local_strConfigPtr->GPIO_PortID));

            /* Switch on the provided mode : input , output , analog , alternative function */
            switch(Local_strConfigPtr->GPIO_Mode)
            {
                case MODE_INPUT:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_APB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_APB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_APB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_APB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable alternative function select on GPIO port pins */
                        GPIO_APB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Assign the direction of the provided pins as output */
                        GPIO_APB_DIR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_AHB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_AHB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_AHB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_AHB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable alternative function select on GPIO port pins */
                        GPIO_AHB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Assign the direction of the provided pins as output */
                        GPIO_AHB_DIR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case MODE_OUTPUT:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_APB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_APB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_APB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_APB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable alternative function select on GPIO port pins */
                        GPIO_APB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Assign the direction of the provided pins as output */
                        GPIO_APB_DIR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_AHB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_AHB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_AHB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_AHB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable alternative function select on GPIO port pins */
                        GPIO_AHB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Assign the direction of the provided pins as output */
                        GPIO_AHB_DIR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case MODE_ANALOG:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_APB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_APB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable Digital signals on GPIO port pins */
                        GPIO_APB_DEN_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable analog signals on GPIO port pins */
                        GPIO_APB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable alternative function select on GPIO port pins */
                        GPIO_APB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_AHB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_AHB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable Digital signals on GPIO port pins */
                        GPIO_AHB_DEN_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable analog signals on GPIO port pins */
                        GPIO_AHB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable alternative function select on GPIO port pins */
                        GPIO_AHB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case MODE_ALTERNATIVE_FUNCTION:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_APB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_APB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_APB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_APB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable alternative function select on GPIO port pins */
                        GPIO_APB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Unlock the provided GPIO port */
                        GPIO_AHB_LOCK_R(Local_strConfigPtr->GPIO_PortID)=GPIO_PORT_UNLOCK;

                        /* Enable committing for the provided GPIO pins */
                        GPIO_AHB_CR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable Digital signals on GPIO port pins */
                        GPIO_AHB_DEN_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                        /* Disable analog signals on GPIO port pins */
                        GPIO_AHB_AMSEL_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                        /* Enable alternative function select on GPIO port pins */
                        GPIO_AHB_AFSEL_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                default:
                break;
            }

            /* Switch on the GPIO pin type pullup , pulldown , opendrain */
            switch(Local_strConfigPtr->GPIO_PUPDOD)
            {
                case NOPUPDOD:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Disable pullup , pulldown , opendrain */
                        GPIO_APB_PUR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                        GPIO_APB_PDR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                        GPIO_APB_ODR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Disable pullup , pulldown , opendrain */
                        GPIO_AHB_PUR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                        GPIO_AHB_PDR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                        GPIO_AHB_ODR_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case PULLUP:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Enable the corresponding bit in the pullup register */
                        GPIO_APB_PUR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Enable the corresponding bit in the pullup register */
                        GPIO_AHB_PUR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case PULLDOWN:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Enable the corresponding bit in the pulldown register */
                        GPIO_APB_PDR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Enable the corresponding bit in the pulldown register */
                        GPIO_AHB_PDR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                case OPENDRAIN:
                {
                    /* Check if the provided bus is APB */
                    if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                    {
                        /* Enable the corresponding bit in the opendrain register */
                        GPIO_APB_ODR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    /* Check if the provided bus is AHB */
                    else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                    {
                        /* Enable the corresponding bit in the opendrain register */
                        GPIO_AHB_ODR_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);
                    }
                    else
                    {
                        /* Nothing (Misra) */
                    }
                }
                break;

                default:
                break;
            }

            /* Change the status of this Group to initialized */
            Global_u8GroupStatusArray[Local_u8LoopIndex]=HIGH;

            /* Return initialization done successfully */
            Local_enReturnValue=GPIO_OK;
        }
        else
        {
            /* Invalid port ID */
            Local_enReturnValue=GPIO_NOK;
        }
    }

    return Local_enReturnValue;
}

/* Write function to GPIO port pins */
GPIO_Status GPIO_enWrite(uint8 Copy_u8GroupNum,uint8 Copy_u8DataType)
{
    /* Variable to indicate the success of GPIO configuration */
    GPIO_Status Local_enReturnValue=GPIO_OK;

    /* Pointer to GPIO Configuration structure */
    const GPIO_ConfigStrType* Local_strConfigPtr;

    /* Check if the provided group number is within the range */
    if(Copy_u8GroupNum<GPIO_NUM_OF_GROUPS)
    {
        /* Assign the pointer to the Configuration array of structures to access the group number */
        Local_strConfigPtr=&Gobal_strConfigArray[Copy_u8GroupNum];

        /* Check if the provided group mode is output and the provided group number is initialized */
        if((Global_u8GroupStatusArray[Copy_u8GroupNum]==HIGH)&&(Local_strConfigPtr->GPIO_Mode==MODE_OUTPUT))
        {
            /* Check if the provided data is high */
            if(Copy_u8DataType==HIGH)
            {
                /* Check if the provided bus is APB */
                if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_APB_DATA_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                /* Check if the provided bus is AHB */
                else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_AHB_DATA_R(Local_strConfigPtr->GPIO_PortID)|=(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                else
                {
                    /* Nothing (Misra) */
                }
            }
            /* Check if the provided data is LOW */
            else if(Copy_u8DataType==LOW)
            {
                /* Check if the provided bus is APB */
                if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_APB_DATA_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                /* Check if the provided bus is AHB */
                else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_AHB_DATA_R(Local_strConfigPtr->GPIO_PortID)&=~(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                else
                {
                    /* Nothing (Misra) */
                }
            }
            /* Check if the provided data is TOGGLE */
            else if(Copy_u8DataType==TOGGLE)
            {
                /* Check if the provided bus is APB */
                if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_APB_DATA_R(Local_strConfigPtr->GPIO_PortID)^=(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                /* Check if the provided bus is AHB */
                else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
                {
                    /* Assign the provided data to the data register */
                    GPIO_AHB_DATA_R(Local_strConfigPtr->GPIO_PortID)^=(Local_strConfigPtr->GPIO_PinNumber);

                    /* Return write done */
                    Local_enReturnValue=GPIO_OK;
                }
                else
                {
                    /* Nothing (Misra) */
                }
            }
            else
            {
                /* Nothing (Misra) */
            }
        }
        else
        {
            /* The provided group is not initialized or the provided mode is not output */
            Local_enReturnValue=GPIO_NOK;
        }
    }
    else
    {
        /* Invalid group number */
        Local_enReturnValue=GPIO_NOK;
    }

    return Local_enReturnValue;
}

/* Read function from GPIO port pins */
GPIO_Status GPIO_enRead(uint8 Copy_u8GroupNum,uint8* Copy_pu8DataType)
{
    /* Variable to indicate the success of GPIO configuration */
    GPIO_Status Local_enReturnValue=GPIO_OK;

    /* Pointer to GPIO Configuration structure */
    const GPIO_ConfigStrType* Local_strConfigPtr;

    /* Check if the provided group number is within the range */
    if(Copy_u8GroupNum<GPIO_NUM_OF_GROUPS)
    {
        /* Assign the pointer to the Configuration array of structures to access the group number */
        Local_strConfigPtr=&Gobal_strConfigArray[Copy_u8GroupNum];

        /* Check if the provided group mode is input and the provided group number is initialized */
        if((Global_u8GroupStatusArray[Copy_u8GroupNum]==HIGH)&&(Local_strConfigPtr->GPIO_Mode==MODE_INPUT))
        {
            /* Check if the provided bus is APB */
            if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
            {
                /* Read the data from the provided port pin */
                *Copy_pu8DataType=GPIO_APB_DATA_R(Local_strConfigPtr->GPIO_PortID)&(Local_strConfigPtr->GPIO_PinNumber);

                /* Return write done */
                Local_enReturnValue=GPIO_OK;
            }
            /* Check if the provided bus is AHB */
            else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
            {
                /* Read the data from the provided port pin */
                *Copy_pu8DataType=GPIO_AHB_DATA_R(Local_strConfigPtr->GPIO_PortID)&(Local_strConfigPtr->GPIO_PinNumber);

                /* Return write done */
                Local_enReturnValue=GPIO_OK;
            }
            else
            {
                /* Nothing (Misra) */
            }
        }
        else
        {
            /* The provided group is not initialized or the provided mode is not input */
            Local_enReturnValue=GPIO_NOK;
        }
    }
    else
    {
        /* Invalid group number */
        Local_enReturnValue=GPIO_NOK;
    }

    return Local_enReturnValue;
}

/* Set alternative function to GPIO port pins */
GPIO_Status GPIO_enSetAlternativeFunction(uint8 Copy_u8GroupNum,uint32 Copy_u32AlternativeFunctionId)
{
    /* Variable to indicate the success of GPIO configuration */
    GPIO_Status Local_enReturnValue=GPIO_OK;

    /* Pointer to GPIO Configuration structure */
    const GPIO_ConfigStrType* Local_strConfigPtr;

    /* Check if the provided group number is within the range */
    if(Copy_u8GroupNum<GPIO_NUM_OF_GROUPS)
    {
        /* Assign the pointer to the Configuration array of structures to access the group number */
        Local_strConfigPtr=&Gobal_strConfigArray[Copy_u8GroupNum];

        /* Check if the provided group mode is alternative function and the provided group number is initialized */
        if((Global_u8GroupStatusArray[Copy_u8GroupNum]==HIGH)&&(Local_strConfigPtr->GPIO_Mode==MODE_ALTERNATIVE_FUNCTION))
        {
            /* Check if the provided bus is APB */
            if(Local_strConfigPtr->GPIO_Speed==LOW_SPEED)
            {
                /* Set the alternative function to the provided port pin */
                GPIO_APB_PCTL_R(Local_strConfigPtr->GPIO_PortID)|=(Copy_u32AlternativeFunctionId);

                /* Return write done */
                Local_enReturnValue=GPIO_OK;
            }
            /* Check if the provided bus is AHB */
            else if(Local_strConfigPtr->GPIO_Speed==HIGH_SPEED)
            {
                /* Set the alternative function to the provided port pin */
                GPIO_AHB_PCTL_R(Local_strConfigPtr->GPIO_PortID)|=(Copy_u32AlternativeFunctionId);

                /* Return write done */
                Local_enReturnValue=GPIO_OK;
            }
            else
            {
                /* Nothing (Misra) */
            }
        }
        else
        {
            /* The provided group is not initialized or the provided mode is not alternative function */
            Local_enReturnValue=GPIO_NOK;
        }
    }
    else
    {
        /* Invalid group number */
        Local_enReturnValue=GPIO_NOK;
    }

    return Local_enReturnValue;
}
