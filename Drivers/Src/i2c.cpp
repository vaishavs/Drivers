/*
 * i2c.cpp
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */


#include "i2c.h"
#include "rcc.h"

/**
  * @name       ClockControl
  * @brief      Enables or disables the clock
  * @param[in]  En_Dis: Enable or Disable
  * @retval     None
  * @note       None
  */
void I2C::ClockControl(bool En_Dis)
{
  if(pReg==I2C1) RCC->APB1ENR.set(RCC_APB1ENR_I2C1EN_Pos, En_Dis);
  else if(pReg==I2C2) RCC->APB1ENR.set(RCC_APB1ENR_I2C2EN_Pos, En_Dis);
  else if(pReg==I2C3) RCC->APB1ENR.set(RCC_APB1ENR_I2C3EN_Pos, En_Dis);
}

/**
  * @name   DeInit
  * @brief  Resets the peripheral.
  * @param  None
  * @retval None
  * @note   None
  */
void I2C::DeInit()
{
  pReg->CR1.reset(I2C_CR1_PE_Pos); //Disable peripheral
  this->ClockControl(DIS);

  if(pReg==I2C1) RCC->APB1RSTR.set(RCC_APB1RSTR_I2C1RST_Pos);
  else if(pReg==I2C2) RCC->APB1RSTR.set(RCC_APB1RSTR_I2C2RST_Pos);
  else if(pReg==I2C3) RCC->APB1RSTR.set(RCC_APB1RSTR_I2C3RST_Pos);

  delete[] pReg; delete[] pBuf;
}

/**
  * @name       Init
  * @brief      Initializes the peripheral.
  * @param[in]  Mode: Master or slave
  * @param[in]  Speed: Clock speed
  * @param[in]  DeviceAddress: Address of the current device
  * @param[in]  Ack: Acknowledgment bit
  * @param[in]  DutyCycle: Duty cycle of the clock
  * @retval     None
  * @note       None
  */
void I2C::Init()
{
  //Acknowledgement
  pReg->CR1.set(I2C_CR1_ACK_Pos, Ack);

  //Clock frequency
  uint32_t temp = GetPClock1() / 1000000;
  SET_BITS(pReg->CR2, temp<<I2C_CR2_FREQ_Pos);

  //Addressing
  pReg->OAR1.reset(I2C_OAR1_ADDMODE_Pos);
  SET_BITS(pReg->OAR1, DeviceAddress<<I2C_OAR1_ADD1_Pos);

  //Clock speed
  uint16_t ccr;
  if(Speed <= I2C_SPEED_SM)
  {
    pReg->CCR.reset(I2C_CCR_FS_Pos);
    ccr = GetPClock1() / (2*Speed);
    SET_BITS(pReg->CCR, ccr<<I2C_CCR_CCR_Pos);
  }
  else
  {
    pReg->CCR.set(I2C_CCR_FS_Pos);
    pReg->CCR.set(I2C_CCR_DUTY_Pos, DutyCycle);

    if(DutyCycle == I2C_DUTY_2)
      ccr = GetPClock1() / (3*Speed);
    else
      ccr = GetPClock1() / (25*Speed);

    SET_BITS(pReg->CCR, ccr<<I2C_CCR_CCR_Pos);
  }

  ClockControl(EN);
  pReg->CR1.set(I2C_CR1_PE_Pos);
}


/**
  * @name       MasterTransmit
  * @brief      Sends data.
  * @param[in]  DevAddress: Device address
  * @param[in]  pData: Data buffer
  * @param[in]  Size: Size of data
  * @retval     None
  * @note       None
  */
void I2C::MasterTransmit(uint16_t DevAddress, uint8_t *pData, uint8_t Size)
{
  pBuf = pData; Len = Size;
  Mode = I2C_MODE_MASTER;

  //Disable Pos
  pReg->CR1.reset(I2C_CR1_POS_Pos);

  //Generate start
  pReg->CR1.set(I2C_CR1_START_Pos);

  //Wait till SB flag is set
  while(!pReg->SR1[I2C_SR1_SB_Pos]);

  //Send the slave address
  pReg->DR = DevAddress;

  //Wait till ADDR flag is set
  while(!pReg->SR1[I2C_SR1_ADDR_Pos]);

  //Clear ADDR flag
  ClearAddressFlag();

  //Wait until TXE flag is set
  while(!pReg->SR1[I2C_SR1_TXE_Pos]);

  //Write data to DR
  while(Len>0)
  {
    pReg->DR = *pBuf;
    pBuf++; Len--;
  }

  //Wait until BTF flag is set
  while(!pReg->SR1[I2C_SR1_BTF_Pos]);

  //Generate stop
  pReg->CR1.set(I2C_CR1_STOP_Pos);
}

/**
  * @name       MasterReceive
  * @brief      Receives data.
  * @param[in]  DevAddress: Device address
  * @param[in]  pData: Data buffer
  * @param[in]  Size: Size of data
  * @retval     None
  * @note       None
  */
void I2C::MasterReceive(uint16_t DevAddress, uint8_t *pData, uint8_t Size)
{
  pBuf = pData; Len = Size;
  Mode = I2C_MODE_MASTER;

  //Wait until BUSY flag is reset
  while(pReg->SR2[I2C_SR2_BUSY_Pos]);

   //Disable Pos
  pReg->CR1.reset(I2C_CR1_POS_Pos);

  //Send the slave address
  pReg->DR = DevAddress;

  if(Len==0)
  {
    //Clear ADDR flag
    ClearAddressFlag();

    //Generate stop
    pReg->CR1.set(I2C_CR1_STOP_Pos);
  }
  else if(Len == 1)
  {
    //Disable ACK
    pReg->CR1.reset(I2C_CR1_ACK_Pos);

    //Clear ADDR flag
    ClearAddressFlag();

    //Generate stop
    pReg->CR1.set(I2C_CR1_STOP_Pos);
  }
  else if(Len==2)
  {
    //Disable ACK
    pReg->CR1.reset(I2C_CR1_ACK_Pos);

    //Enable POS
    pReg->CR1.set(I2C_CR1_POS_Pos);

    //Clear ADDR flag
    ClearAddressFlag();
  }
  else
  {
    //Enable ACK
    pReg->CR1.set(I2C_CR1_ACK_Pos);

    //Clear ADDR flag
    ClearAddressFlag();
  }

  while(Len>0)
  {
    if(Len == 1)
    {
      //Wait until RXNE flag is set
      while(!pReg->SR1[I2C_SR1_RXNE_Pos]);

      //Read data
      *pBuf = pReg->DR.to_ulong();
      pBuf++; Len--;
    }

    else if(Len == 2)
    {
      //Wait until BTF flag is set
      while(!pReg->SR1[I2C_SR1_BTF_Pos]);

      //Generate stop
      pReg->CR1.set(I2C_CR1_STOP_Pos);

      //Read data
      *pBuf = pReg->DR.to_ulong();
      pBuf++; Len--;
    }
    else
    {
      //Wait until BTF flag is set
      while(!pReg->SR1[I2C_SR1_BTF_Pos]);

      //Disable ACK
      pReg->CR1.reset(I2C_CR1_ACK_Pos);

      //Read data
      *pBuf = pReg->DR.to_ulong();
      pBuf++; Len--;

      //Wait until BTF flag is set
      while(!pReg->SR1[I2C_SR1_BTF_Pos]);

      //Generate stop
      pReg->CR1.set(I2C_CR1_STOP_Pos);
    }
  }
}

/**
  * @name       SlaveTransmit
  * @brief      Transmits data.
  * @param[in]  pData: Data buffer
  * @param[in]  Size: Size of data
  * @retval     None
  * @note       None
  */
void I2C::SlaveTransmit(uint8_t *pData, uint8_t Size)
{
  pBuf = pData; Len = Size;
  Mode = I2C_MODE_SLAVE;

   //Disable Pos
  pReg->CR1.reset(I2C_CR1_POS_Pos);

  //Clear ADDR flag
  ClearAddressFlag();

  while(Len>0)
  {
    //Wait until TXE flag is set
    while(!pReg->SR1[I2C_SR1_TXE_Pos]);

    pReg->DR = *pBuf;
    pBuf++; Len--;
  }

  //Wait until AF flag is set
  while(!pReg->SR1[I2C_SR1_AF_Pos]);

  //Clear AF flag
  pReg->SR1.reset(I2C_SR1_AF_Pos);

  //Disable ACK
  pReg->CR1.reset(I2C_CR1_ACK_Pos);
}

/**
  * @name       SlaveReceive
  * @brief      Receives data.
  * @param[in]  pData: Data buffer
  * @param[in]  Size: Size of data
  * @retval     None
  * @note       None
  */
void I2C::SlaveReceive(uint8_t *pData, uint8_t Size)
{
  pBuf = pData; Len = Size;
  Mode = I2C_MODE_SLAVE;

   //Disable Pos
  pReg->CR1.reset(I2C_CR1_POS_Pos);

  //Enable ACK
  pReg->CR1.set(I2C_CR1_ACK_Pos);

  //Wait until ADDR flag is set
  while(!pReg->SR1[I2C_SR1_ADDR_Pos]);

  //Clear ADDR flag
  ClearAddressFlag();

  while(Len>0)
  {
    //Wait until RXNE flag is set
    while(!pReg->SR1[I2C_SR1_RXNE_Pos]);

    //Read data
    *pBuf = pReg->DR.to_ulong();
    pBuf++; Len--;
  }

  //Wait until STOP flag is set
  while(!pReg->SR1[I2C_SR1_STOPF_Pos]);

  //Clear STOP flag
  ClearStopFlag();

  //Disable ACK
  pReg->CR1.reset(I2C_CR1_ACK_Pos);
}

/*
 * Helper functions
 */
void I2C::ClearAddressFlag()
{
  //Clear ADDR flag
  uint32_t temp = pReg->SR1.to_ulong();
  temp = pReg->SR2.to_ulong();
  (void)temp;
}

void I2C::ClearStopFlag()
{
  uint32_t tmpreg = pReg->SR1.to_ulong();
  pReg->CR1.set(I2C_CR1_PE_Pos);
  (void)tmpreg;
}

