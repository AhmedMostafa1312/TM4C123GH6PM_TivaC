/*
 * Auther:          Edited by the user.
 * Date:            26/6/2018
 * File Name:       TM4C123GH6PM_MemMap.h
 * Description:     This file contains:
 *                   -Macros to access the memory and registers of the microcontroller.
 * Microcontroller: TM4C123GH6PM
 */

#ifndef TM4C123GH6PM_MEMMAP_H_
#define TM4C123GH6PM_MEMMAP_H_

#include "Std_Types.h"

/* Datatype volatile (to avoid compiler optimization) and constant (as the addresses are constants) */
typedef volatile uint32* const TM4C123GH6PM_SysCtlRegAddType;

/* System Control Registers Base Address */
#define SYS_CTL_R_BASE_ADD 0x400FE000U

/* System Control GPIO High-Performance Bus Control Register offset*/
#define GPIO_HBCTL_R_OFFSET 0x06CU

/* System Control GPIO High-Performance Bus Control Register base address*/
#define GPIO_HBCTL_R_BASE_ADD *((TM4C123GH6PM_SysCtlRegAddType)(SYS_CTL_R_BASE_ADD+GPIO_HBCTL_R_OFFSET))

/* System Control General-Purpose Input/Output Run Mode Clock Gating Control Register offset */
#define RCGC_GPIO_R_OFFSET 0x608U

/* System Control General-Purpose Input/Output Run Mode Clock Gating Control Register base address */
#define RCGC_GPIO_R_BASE_ADD *((TM4C123GH6PM_SysCtlRegAddType)(SYS_CTL_R_BASE_ADD+RCGC_GPIO_R_OFFSET))

#endif /* TM4C123GH6PM_MEMMAP_H_ */
