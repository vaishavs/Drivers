/*
 * gpio.cpp
 *
 *  Created on: 16-Jun-2022
 *      Author: Vaishnavi
 */


#include "gpio.h"

/**
  * @name       ClockControl
  * @brief      Enables or disables the clock
  * @param[in]  En_Dis: Enable or Disable
  * @retval     None
  * @note       None
  */
void GPIO::ClockControl(bool En_Dis)
{
  if(pReg == GPIOA) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIOAEN_Pos, En_Dis);
  else if(pReg == GPIOB) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIOBEN_Pos, En_Dis);
  else if(pReg == GPIOC) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIOCEN_Pos, En_Dis);
  else if(pReg == GPIOD) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIODEN_Pos, En_Dis);
  else if(pReg == GPIOE) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIOEEN_Pos, En_Dis);
  else if(pReg == GPIOH) RCC->AHB1ENR.set(RCC_AHB1ENR_GPIOHEN_Pos, En_Dis);
}

/**
  * @name   DeInit
  * @brief  Resets the peripheral.
  * @param  None
  * @retval None
  * @note   None
  */
void GPIO::DeInit()
{

  if(pReg == GPIOA) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIOARST_Pos);
  else if(pReg == GPIOB) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIOBRST_Pos);
  else if(pReg == GPIOC) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIOCRST_Pos);
  else if(pReg == GPIOD) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIODRST_Pos);
  else if(pReg == GPIOE) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIOERST_Pos);
  else if(pReg == GPIOH) RCC->AHB1ENR.set(RCC_AHB1RSTR_GPIOHRST_Pos);
}

/**
  * @name       Init
  * @brief      Initializes the peripheral according to the specified parameters.
  * @param[in]  Speed: Low, medium, fast, or high
  * @param[in]  Pull: Pullup or Pulldown
  * @retval     None
  * @note       None
  */
void GPIO::Init()
{

  //Configure mode
  SET_BITS(pReg->MODER, (Mode << 2*Pin));

  //Configure speed
  SET_BITS(pReg->OSPEEDR, (Speed << 2*Pin));

  //Configure pullup/pulldown
  SET_BITS(pReg->PUPDR, (Pull << 2*Pin));

  //Enable clock
  this->ClockControl(EN);
}

/**
  * @name   ReadPort
  * @brief  Reads the value of the entire port
  * @param  None
  * @retval Input port value.
  * @note   None
  */
bit32_t GPIO::ReadPort()
{
  return pReg->IDR;
}

/**
  * @name   ReadPin
  * @brief  Reads the value of the Pin
  * @param  None
  * @retval Input port Pin value.
  * @note   None
  */
uint8_t GPIO::ReadPin()
{
  return pReg->IDR[Pin];
}

/**
  * @name       WritePort
  * @brief      Writes a value to the entire port
  * @param[in]  value: Value to be written
  * @retval     None
  * @note       None
  */
void GPIO::WritePort(uint32_t value)
{
  WRITE_REG(pReg->ODR, value);
}

/**
  * @name       WritePin
  * @brief      Writes a value to the specified Pin
  * @param[in]  value: Value to be written
  * @retval     None
  * @note       None
  */
void GPIO::WritePin(bool value)
{
  pReg->ODR.set(Pin, value);
}

/**
  * @name        WritePin
  * @brief      Writes a value to the specified Pin
  * @param[in]  value: Value to be written
  * @retval     None
  * @note       None
  */
void GPIO::TogglePin()
{
  pReg->ODR.flip(Pin);
}
