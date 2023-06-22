/*
 * gpio.h
 *
 *  Created on: 16-Jun-2022
 *      Author: Vaishnavi
 */

#ifndef GPIO_H_
#define GPIO_H_


#include "stm32f411xe.h"

/*
 * Register configuration macros
 */
//Pin modes
#define GPIO_MODE_INPUT   0 //Input mode
#define GPIO_MODE_OUTPUT  1 //Output mode
#define GPIO_MODE_ALTFN   2 //Alternate function
#define GPIO_MODE_ANALOG  3 //Analog

//Output type
#define GPIO_TYPE_PP  0 //Push pull
#define GPIO_TYPE_OD  1 //Open drain

//Speed
#define GPIO_SPEED_LOW      0 //Low speed
#define GPIO_SPEED_MEDIUM   1 //Medium speed
#define GPIO_SPEED_FAST     2 //Fast speed
#define GPIO_SPEED_HIGH     3 //High speed

//Pullup or pull-down
#define GPIO_PULL_NONE  0 //No push-pull
#define GPIO_PULL_UP    1 //Pullup
#define GPIO_PULL_DOWN  2 //Pull-down

//Alternate functions: 0 to 15

/*
 * Peripheral handle
 */
class GPIO : API<GPIO_t>
{
  uint8_t Pin, Pull, OutputType, AltFn;

public:
  //Constructor
  GPIO(){}
  GPIO(GPIO_t* reg, uint8_t pin, uint8_t speed, uint8_t pull)
  {
  	pReg=reg; Pin=pin;
  	Speed = speed; Pull = pull;
  }
  ~GPIO() { DeInit(); }

  //Init functions
  void Init();
  void DeInit();
  void ClockControl(bool En_Dis);

  //IO functions
  uint8_t ReadPin();
  bit32_t ReadPort();
  void WritePin(bool value);
  void WritePort(uint32_t value);
  void TogglePin();

  //For output and alternate function modes
  void setMode(uint8_t mode, uint8_t config) {
    Mode = mode;

    if(Mode==GPIO_MODE_OUTPUT)
    {
    	OutputType = config; // Push-pull or open-drain
      pReg->OTYPER[Pin] = (bool)OutputType;
    }
    else if(Mode == GPIO_MODE_ALTFN)
    {
    	AltFn = config; //Alternate function
    	SET_BITS(pReg->AFR[Pin / 8], (AltFn << 4*(Pin % 8)));
    }
  }

  //For input and analog modes
  void setMode(uint8_t mode)
  {
    Mode = mode;
  }

};



#endif /* GPIO_H_ */
