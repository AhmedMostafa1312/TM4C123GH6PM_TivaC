/*
 * Auther:          Ahmed Mostafa
 * Date:            26/6/2018
 * File Name:       GPIO.h
 * Description:     This file contains:
 *                   -Prototypes of the APIs of the GPIO.
 *                   -Macros for accessing GPIO registers.
 * Microcontroller: TM4C123GH6PM
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "Std_Types.h"
#include "GPIO_Config.h"
#include "TM4C123GH6PM_MemMap.h"

/* Number of GPIO ports */
#define GPIO_NUM_OF_PORTS 6U

/* GPIO unlock port value to assign it to lock register */
#define GPIO_PORT_UNLOCK 0x4C4F434B

/* GPIO Base Addresses for the Advanced Peripheral Bus (APB) */
#define GPIO_PORTA_APB_BASE_ADD  0x40004000U
#define GPIO_PORTB_APB_BASE_ADD  0x40005000U
#define GPIO_PORTC_APB_BASE_ADD  0x40006000U
#define GPIO_PORTD_APB_BASE_ADD  0x40007000U
#define GPIO_PORTE_APB_BASE_ADD  0x40024000U
#define GPIO_PORTF_APB_BASE_ADD  0x40025000U

/* GPIO Base Addresses for the Advanced High-Performance Bus (AHB) */
#define GPIO_PORTA_AHB_BASE_ADD  0x40058000U
#define GPIO_PORTB_AHB_BASE_ADD  0x40059000U
#define GPIO_PORTC_AHB_BASE_ADD  0x4005A000U
#define GPIO_PORTD_AHB_BASE_ADD  0x4005B000U
#define GPIO_PORTE_AHB_BASE_ADD  0x4005C000U
#define GPIO_PORTF_AHB_BASE_ADD  0x4005D000U

/* GPIO Registers Offset */
#define GPIO_DATA_R_OFFSET           0x000U
#define GPIO_DATA_R_BIT_BANDING_ALL  0x3FCU
#define GPIO_DIR_R_OFFSET            0x400U
#define GPIO_AFSEL_R_OFFSET          0x420U
#define GPIO_ODR_R_OFFSET            0x50CU
#define GPIO_PUR_R_OFFSET            0x510U
#define GPIO_PDR_R_OFFSET            0x514U
#define GPIO_DEN_R_OFFSET            0x51CU
#define GPIO_LOCK_R_OFFSET           0x520U
#define GPIO_CR_R_OFFSET             0x524U
#define GPIO_AMSEL_R_OFFSET          0x528U
#define GPIO_PCTL_R_OFFSET           0x52CU

/* Generate the base address of the used register located in the GPIO port (APB) */
#define GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_R_OFFSET) (Global_u32PortxAPBBaseAddArray[GPIO_PORT_ID]+GPIO_R_OFFSET)

/* GPIO Registers for any port (APB) */
#define GPIO_APB_DATA_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_DATA_R_OFFSET)+GPIO_DATA_R_BIT_BANDING_ALL))
#define GPIO_APB_DIR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_DIR_R_OFFSET)))
#define GPIO_APB_AFSEL_R(GPIO_PORT_ID)   *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_AFSEL_R_OFFSET)))
#define GPIO_APB_ODR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_ODR_R_OFFSET)))
#define GPIO_APB_PUR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_PUR_R_OFFSET)))
#define GPIO_APB_PDR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_PDR_R_OFFSET)))
#define GPIO_APB_DEN_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_DEN_R_OFFSET)))
#define GPIO_APB_LOCK_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_LOCK_R_OFFSET)))
#define GPIO_APB_CR_R(GPIO_PORT_ID)      *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_CR_R_OFFSET)))
#define GPIO_APB_AMSEL_R(GPIO_PORT_ID)   *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_AMSEL_R_OFFSET)))
#define GPIO_APB_PCTL_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_APB_BASE_ADD(GPIO_PORT_ID,GPIO_PCTL_R_OFFSET)))

/* Generate the base address of the used register located in the GPIO port (AHB) */
#define GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_R_OFFSET) (Global_u32PortxAHBBaseAddArray[GPIO_PORT_ID]+GPIO_R_OFFSET)

/* GPIO Registers for any port (AHB) */
#define GPIO_AHB_DATA_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_DATA_R_OFFSET)+GPIO_DATA_R_BIT_BANDING_ALL))
#define GPIO_AHB_DIR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_DIR_R_OFFSET)))
#define GPIO_AHB_AFSEL_R(GPIO_PORT_ID)   *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_AFSEL_R_OFFSET)))
#define GPIO_AHB_ODR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_ODR_R_OFFSET)))
#define GPIO_AHB_PUR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_PUR_R_OFFSET)))
#define GPIO_AHB_PDR_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_PDR_R_OFFSET)))
#define GPIO_AHB_DEN_R(GPIO_PORT_ID)     *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_DEN_R_OFFSET)))
#define GPIO_AHB_LOCK_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_LOCK_R_OFFSET)))
#define GPIO_AHB_CR_R(GPIO_PORT_ID)      *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_CR_R_OFFSET)))
#define GPIO_AHB_AMSEL_R(GPIO_PORT_ID)   *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_AMSEL_R_OFFSET)))
#define GPIO_AHB_PCTL_R(GPIO_PORT_ID)    *((GPIO_RegAddType)(GPIO_R_AHB_BASE_ADD(GPIO_PORT_ID,GPIO_PCTL_R_OFFSET)))

#define DEFAULT 0U

/* Datatype volatile (to avoid compiler optimization) and constant (as the addresses are constants) */
typedef volatile uint32* const GPIO_RegAddType;

 /* GPIO port ID */
typedef enum
{
    PORT_A=0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_F
}GPIO_PortID;

/* GPIO pin number */
typedef enum
{
    PIN_0=0x01,
    PIN_1=0x02,
    PIN_2=0x04,
    PIN_3=0x08,
    PIN_4=0x10,
    PIN_5=0x20,
    PIN_6=0x40,
    PIN_7=0x80,
    PIN_ALL=0xFF
}GPIO_PinNumber;

/*GPIO mode */
typedef enum
{
    MODE_INPUT=0,
    MODE_OUTPUT,
    MODE_ANALOG,
    MODE_ALTERNATIVE_FUNCTION
}GPIO_Mode;

/* GPIO Alternative Functions ID */
typedef enum
{
    /* No need to set alternative function API (GPIO_PCTL) */

    /* For ADC */
    AIN0=0xA0000000,AIN1=0xA000001,AIN2=0xA0000002,AIN3=0xA0000003,
    AIN4=0xA0000004,AIN5=0xA0000005,AIN6=0xA0000006,AIN7=0xA0000007,
    AIN8=0xA0000008,AIN9=0xA0000009,AIN10=0xA000000A,AIN11=0xA000000B,


    /* Need to set alternative function API (GPIO_PCTL) */

    /* For Timers */
    T0CCP0_PB=0x07000000,T0CCP1_PB=0x70000000,
    T0CCP0_PF=0x00000007,T0CCP1_PF=0x00000070,
    T1CCP0_PB=0x00070000,T1CCP1_PB=0x00700000,
    T1CCP0_PF=0x00000700,T1CCP1_PF=0x00007000,
    T2CCP0_PB=0x00000007,T2CCP0_PF=0x00070000,
    T2CCP1=0x00000070,
    T3CCP0=0x00000700,T3CCP1=0x00007000,
    T4CCP0=0x00000007,T4CCP1=0x00000070,
    T5CCP0=0x00000700,T5CCP1=0x00007000,
    WT0CCP0=0x00070000,WT0CCP1=0x00700000,
    WT1CCP0=0x07000000,WT1CCP1=0x70000000,
    WT2CCP0=0x00000007,WT2CCP1=0x00000070,
    WT3CCP0=0x00000700,WT3CCP1=0x00007000,
    WT4CCP0=0x00070000,WT4CCP1=0x00700000,
    WT5CCP0=0x07000000,WT5CCP1=0x70000000,

    /* For PWM */
    M0PWM0=0x04000000,M0PWM1=0x40000000,
    M0PWM2=0x00040000,M0PWM3=0x00400000,
    M0PWM4=0x00040000,M0PWM5=0x00400000,
    M0PWM6_PC=0x00040000,M0PWM6_PD=0x00000004,
    M0PWM7_PC=0x00400000,M0PWM7_PD=0x00000040,
    M1PWM0=0x00000005,M1PWM1=0x00000050,
    M1PWM2_PA=0x05000000,M1PWM2_PE=0x00050000,
    M1PWM3_PA=0x50000000,M1PWM3_PE=0x00500000,
    M1PWM4=0x00000005,M1PWM5=0x00000050,
    M1PWM6=0x00000500,M1PWM7=0x00005000,

    /* For non-maskable interrupt */
    NMI_PD=0x80000000,NMI_PF=0x00000008,

    /* For UART */
    U0Rx=0x00000001,U0Tx=0x00000010,
    U1Rx_PB=0x00000001,U1Tx_PB=0x00000010,
    U1Rx_PC=0x00020000,U1Tx_PC=0x00200000,
    U2Rx=0x01000000,U2Tx=0x10000000,
    U3Rx=0x01000000,U3Tx=0x10000000,
    U4Rx=0x00010000,U4Tx=0x00100000,
    U5Rx=0x00010000,U5Tx=0x00100000,
    U6Rx=0x00010000,U6Tx=0x00100000,
    U7Rx=0x00000001,U7Tx=0x00000010,
    U1RTS_PC=0x00080000,U1CTS_PC=0x00800000,
    U1RTS_PF=0x00000001,U1CTS_PF=0x00000010,

    /* For SPI */
    SSI0Clk=0x00000200,SSI0Fss=0x00002000,SSI0Rx=0x00020000,SSI0Tx=0x00200000,
    SSI1Clk_PD=0x00000002,SSI1Fss_PD=0x00000020,SSI1Rx_PD=0x00000200,SSI1Tx_PD=0x00002000,
    SSI1Clk_PF=0x00000200,SSI1Fss_PF=0x00002000,SSI1Rx_PF=0x00000002,SSI1Tx_PF=0x00000020,
    SSI2Clk=0x00020000,SSI2Fss=0x00200000,SSI2Rx=0x02000000,SSI2Tx=0x20000000,
    SSI3Clk=0x00000001,SSI3Fss=0x00000010,SSI3Rx=0x00000100,SSI3Tx=0x00001000,

    /* For I2C */
    I2C0SCL=0x00000300,I2C0SDA=0x00003000,
    I2C1SCL=0x03000000,I2C1SDA=0x30000000,
    I2C2SCL=0x00030000,I2C2SDA=0x00300000,
    I2C3SCL=0x00000003,I2C3SDA=0x00000030,

    /* For CAN */
    CAN0Rx_PE=0x00080000,CAN0Tx_PE=0x00800000,
    CAN0Rx_PF=0x00000003,CAN0Tx_PF=0x00003000,
    CAN0Rx_PB=0x00080000,CAN0Tx_PB=0x00800000,
    CAN1Rx=0x00000008,CAN1Tx=0x00000080
}GPIO_AlternativeFunction;

/* Type of GPIO bus ABP ->low speed or AHB -> high speed */
typedef enum
{
    LOW_SPEED=0,
    HIGH_SPEED
}GPIO_Speed;

typedef enum
{
    NOPUPDOD=0,
    PULLUP,
    PULLDOWN,
    OPENDRAIN
}GPIO_PUPDOD;

/* status of the GPIO */
typedef enum
{
    GPIO_OK=0,
    GPIO_NOK
}GPIO_Status;

/* Data type of the GPIO pins */
typedef enum
{
    LOW=0,
    HIGH,
    TOGGLE
}GPIO_DataType;

/* Configuration Structure of GPIO that is filled by user */
typedef struct
{
    uint8  GPIO_PortID;
    uint8  GPIO_Speed;
    uint16 GPIO_PinNumber;
    uint8  GPIO_Mode;
    uint8  GPIO_PUPDOD;
}GPIO_ConfigStrType;

/* Extern const array of structures of the GPIO groups that is filled by user */
extern const GPIO_ConfigStrType Gobal_strConfigArray[GPIO_NUM_OF_GROUPS];


/* Prototypes of functions */

/* Initialization function of GPIO ports
 * Inputs : -the configuration array of structure that should be filled by user
 */
GPIO_Status GPIO_enInit(void);



/* Write function to GPIO port pins
 * Inputs : -Group number in the configuration array of structure : 0 , 1 , 2 , ...
 *          -Data type : HIGH , LOW , TOGGLE
 */
GPIO_Status GPIO_enWrite(uint8 Copy_u8GroupNum,uint8 Copy_u8DataType);



/* Read function from GPIO port pins
 * Inputs : -Group number in the configuration array of structure : 0 , 1 , 2 , ...
 *          -Address to variable that should hold the returned Data type : HIGH , LOW
 */
GPIO_Status GPIO_enRead(uint8 Copy_u8GroupNum,uint8* Copy_pu8DataType);



/* Set alternative function to GPIO port pins
 * Inputs : -Group number in the configuration array of structure : 0 , 1 , 2 , ...
 *          -Alternative function ID : T2CCP1 , M0PWM0 , NMI_PD , U0Rx , SSI0Clk , I2C0SCL , CAN0Rx_PE , ...
 */
GPIO_Status GPIO_enSetAlternativeFunction(uint8 Copy_u8GroupNum,uint32 Copy_u32AlternativeFunctionId);

#endif /* GPIO_H_ */
