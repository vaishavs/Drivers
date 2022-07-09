/*
 * rcc.h
 *
 *  Created on: 01-Jul-2022
 *      Author: Vaishnavi
 */

#ifndef RCC_H_
#define RCC_H_


#include "stm32f411xe.h"

uint32_t GetPLLClock();
uint32_t GetSysClockFreq();
uint32_t GetHClock();
uint32_t GetPClock1();
uint32_t GetPClock2();


#endif /* RCC_H_ */
