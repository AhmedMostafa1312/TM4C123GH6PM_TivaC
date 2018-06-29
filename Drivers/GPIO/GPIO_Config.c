/*
 * Auther:          Edited by the user.
 * Date:            26/6/2018
 * File Name:       GPIO_Config.c
 * Description:     This file contains:
 *                   -An array of structures that contains the Groups to initialize the GPIO.
 * Microcontroller: TM4C123GH6PM
 */

#include "GPIO.h"
#include "GPIO_Config.h"

/* Extern const array of structures of the GPIO groups initialization that is filled by user
 *
 * Example :
 *  {
 *     PORT_F,             //GPIO port ID (type : PORT_A , PORT_B , ... , PORT_F)
 *     LOW_SPEED,          //GPIO bus speed (type : LOW_SPEED for APB , HIGH_SPEED for AHB)
 *     PIN_1 | PIN_2,      //GPIO pin number (type : PIN_0 , PIN_1 , ... , PIN_7 or PIN_ALL for all pins or PIN_0|PIN_1 for pins combinations)
 *     MODE_OUTPUT,        //GPIO mode (type : MODE_INPUT , MODE_OUTPUT , MODE_ANALOG , MODE_ALTERNATIVE_FUNCTION)
 *     NOPUPDOD            //GPIO nothing , pullup , pulldown and opendrain (type : NOPUPDOD , PULLUP , PULLDOWN , OPENDRAIN)
 *  },
 *
 *  Don't forget to set number of groups you will use , in the file : GPIO_Config.h
 */

const GPIO_ConfigStrType Gobal_strConfigArray[GPIO_NUM_OF_GROUPS]=
{
    {
  
    },

    {

    },

    {
   
    },

    {
 
    },

    {
 
    }
};
