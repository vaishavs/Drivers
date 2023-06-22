/*
 * i2c.h
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */

#ifndef I2C_H_
#define I2C_H_


#include "stm32f411xe.h"

/*
 * Macros
 */
//Device type
#define I2C_MODE_MASTER   1
#define I2C_MODE_SLAVE    0

//Speed
#define I2C_SPEED_SM    100000
#define I2C_SPEED_FM4K  400000
#define I2C_SPEED_FM2K  200000

//Duty cycle (fast mode)
#define I2C_DUTY_2      0
#define I2C_DUTY_16_9   1

/*
 * Class definition
 */
class I2C : API<I2C_t>
{
  uint8_t DeviceAddress, Ack, DutyCycle;
  uint8_t *pBuf, Len;

public:
  I2C(){}
  I2C(I2C_t *reg, uint8_t mode, uint8_t speed, uint8_t devaddr, uint8_t ack, uint8_t dutycycle)
  {
    pReg=reg;
    Mode = mode; Speed = speed;
    DeviceAddress = devaddr;
    Ack = ack;   DutyCycle = dutycycle;
  }
  ~I2C() { DeInit(); }

  //Initialization
  void ClockControl(bool En_Dis);
  void Init();
  void DeInit();

  //Data transmission
  void MasterTransmit(uint16_t DevAddress, uint8_t *pData, uint8_t Size);
  void MasterReceive(uint16_t DevAddress, uint8_t *pData, uint8_t Size);
  void SlaveTransmit(uint8_t *pData, uint8_t Size);
  void SlaveReceive(uint8_t *pData, uint8_t Size);

  //Helper functions
  void ClearAddressFlag();
  void ClearStopFlag();
};


#endif /* I2C_H_ */
