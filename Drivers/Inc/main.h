/*
 * main.h
 *
 *  Created on: 09-Jul-2022
 *      Author: Vaishnavi
 */

#ifndef MAIN_H_
#define MAIN_H_


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

#define SET_BITS(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BITS(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)         ((REG) = (0x0))
#define WRITE_REG(REG, VAL)    ((REG) = (VAL))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/*
 * Class definition
 */
template<typename T>
class API
{
protected:
  T *pReg;

public:
  virtual void ClockControl(bool En_Dis) = 0;
  virtual void DeInit() = 0;
  void Init();
};


#endif /* MAIN_H_ */
