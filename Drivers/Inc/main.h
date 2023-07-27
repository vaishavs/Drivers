/*
 * main.h
 *
 *  Created on: 09-Jul-2022
 *      Author: Vaishnavi
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <cstdint>
#include <cstdbool>
#include <bitset>

#define SET_BITS(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)   ((REG) &= ~(BIT))
#define CLEAR_REG(REG)         ((REG) = (0x0))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

using namespace std;

//typedef bitset<64> bit64_t;
typedef bitset<32> bit32_t; //32-bit architecture

uint32_t read_bits(bit32_t reg, uint32_t mask);
uint32_t read_mask_value(bit32_t reg, uint32_t mask, uint8_t pos);
/*
 * Utility Macros
 */
#define HIGH    1
#define LOW     0
#define EN      HIGH
#define DIS     LOW
#define SET     HIGH
#define RESET   LOW
#define SUCCESS HIGH
#define FAILURE LOW
#define YES     HIGH
#define NO      LOW
#define RISE    HIGH
#define FALL    LOW

/*
 * Abstract class definition
 */
template<typename T>
class API
{
protected:
  T *pReg;
  uint8_t Mode, Speed;

public:
  virtual void ClockControl(bool En_Dis) = 0;
  virtual void DeInit() = 0;
  virtual void Init() = 0;
};


#endif /* MAIN_H_ */
