/*
 * Auther:      Ahmed Mostafa
 * Date:        26/6/2018
 * File Name:   Utils.h
 */


#ifndef UTILS_H_
#define UTILS_H_

/*----------32-bit registers----------*/

/*-----for single bit-----*/
#define GET_BIT(reg,bit_number) ((reg&(1<<bit_number))>>bit_number)
#define SET_BIT(reg,bit_number) (reg|=(1<<bit_number))
#define CLEAR_BIT(reg,bit_number) (reg&=~(1<<bit_number))
#define TOGGLE_BIT(reg,bit_number) (reg^=(1<<bit_number))
#define ASSIGN_BIT(reg,bit_number,value) do{if(value==0)\
										    {reg&=~(1<<bit_number);}\
										    else\
										    {reg|=(1<<bit_number);}}while(0); /*value can be 0 or 1*/


/*-----for port-----*/
#define GET_PORT(reg) (reg)
#define SET_PORT(reg) (reg|=(0xffffffff))
#define CLEAR_PORT(reg) (reg&=~(0xffffffff))
#define TOGGLE_PORT(reg) (reg^=(0xffffffff))
#define ASSIGN_PORT(reg,value) do{reg&=~(0xffffffff);\
								  reg|=(value);}while(0); /*value can be 0:15*/

/*-----for right rotation-----*/
#define ROTATE_RIGHT(reg,value) reg=(reg>>value)|(reg<<(32-value)) /*value can be 1:31 , if value is 0 or 32 this makes no change in reg*/

/*-----for left rotation-----*/
#define ROTATE_LEFT(reg,value) reg=(reg<<value)|(reg>>(32-value)) /*value can be 1:31 , if value is 0 or 32 this makes no change in reg*/

#endif /* UTILS_H_ */
