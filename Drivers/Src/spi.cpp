/*
 * spi.cpp
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */


#include "spi.h"

/**
  * @name       ClockControl
  * @brief      Enables or disables the clock
  * @param[in]  En_Dis: Enable or Disable
  * @retval     None
  * @note       None
  */
void SPI::ClockControl(bool En_Dis)
{
  if(pReg==SPI1) RCC->APB2ENR.set(RCC_APB2ENR_SPI1EN_Pos, En_Dis);
  else if(pReg==SPI2) RCC->APB1ENR.set(RCC_APB1ENR_SPI2EN_Pos, En_Dis);
  else if(pReg==SPI3) RCC->APB1ENR.set(RCC_APB1ENR_SPI3EN_Pos, En_Dis);
  else if(pReg==SPI4) RCC->APB2ENR.set(RCC_APB2ENR_SPI4EN_Pos, En_Dis);
  else if(pReg==SPI5) RCC->APB2ENR.set(RCC_APB2ENR_SPI5EN_Pos, En_Dis);
}

/**
  * @name    DeInit
  * @brief  Resets the peripheral.
  * @param  None
  * @retval None
  * @note   None
  */
void SPI::DeInit()
{
  pReg->CR1.reset(SPI_CR1_SPE_Pos); //Disable peripheral

  this->ClockControl(DIS);

  if(pReg==SPI1) RCC->APB2RSTR.set(RCC_APB2RSTR_SPI1RST_Pos);
  else if(pReg==SPI2) RCC->APB1RSTR.set(RCC_APB1RSTR_SPI2RST_Pos);
  else if(pReg==SPI3) RCC->APB1RSTR.set(RCC_APB1RSTR_SPI3RST_Pos);
  else if(pReg==SPI4) RCC->APB2RSTR.set(RCC_APB2RSTR_SPI4RST_Pos);
  else if(pReg==SPI5) RCC->APB2RSTR.set(RCC_APB2RSTR_SPI5RST_Pos);

  delete[] pReg; delete[] pRxBuf; delete[] pTxBuf;
}

/**
  * @name       Init
  * @brief      Resets the peripheral.
  * @param[in]  Mode: Master or slave
  * @param[in]  BusCfg: Full duplex or half duplex or simplex
  * @param[in]  DFF: 8-bit or 16-bit data frame
  * @param[in]  CPOL: Clock polarity
  * @param[in]  CPHA: Clock sampling edge
  * @param[in]  SSM: Software Slave Management (disabled by default)
  * @retval     None
  * @note       None
  */
void SPI::Init()
{

  //Device mode
  pReg->CR1.set(SPI_CR1_MSTR_Pos, Mode);

  //Bus configuration
  SET_BITS(pReg->CR1, BusCfg);

  //Speed
  SET_BITS(pReg->CR1, Speed<<SPI_CR1_BR_Pos);

  //DFF
  pReg->CR1.set(SPI_CR1_DFF_Pos, DFF);

  //CPOL
  pReg->CR1.set(SPI_CR1_CPOL_Pos, CPOL);

  //CPHA
  pReg->CR1.set(SPI_CR1_CPHA_Pos, CPHA);

  //SSM
  if(SSM) pReg->CR1.set(SPI_CR1_SSI_Pos);
  else if(Mode==SPI_DEV_MASTER) pReg->CR1.set(SPI_CR2_SSOE_Pos);
  pReg->CR1.set(SPI_CR1_SSM_Pos, SSM);

  //Enable SPI
  ClockControl(EN);
  pReg->CR1.set(SPI_CR1_SPE_Pos);
}

/**
  * @name       Transmit
  * @brief      Sends the data.
  * @param[in]  TxBuf: TX buffer
  * @param[in]  len: No. of bytes to send
  * @retval     None
  * @note       This is a blocking call
  */
void SPI::Transmit(uint8_t *TxBuf, uint8_t len)
{
  pTxBuf = TxBuf; TxCount=len;

  while(TxCount > 0)
  {
    //Wait till TXE flag is set
    while(!pReg->SR[SPI_SR_TXE_Pos]);

    //Check the DFF bit
    if(pReg->CR1[SPI_CR1_DFF_Pos])
    {
      //16 bit data frame
      //Send data to Data Register (DR)
      pReg->DR = *(uint16_t*)pTxBuf;
      TxCount-=2;
      pTxBuf+=sizeof(uint16_t);
    }
    else
    {
      //8 bit data frame
      //Send data to Data Register (DR)
      pReg->DR = *pTxBuf;
      TxCount--;
      pTxBuf+= sizeof(uint8_t);
    }
  }
}

/**
  * @name       Receive
  * @brief      Receives the data.
  * @param[in]  RxBuf: RX buffer
  * @param[in]  len: No. of bytes to read
  * @retval     None
  * @note       This is a blocking call
  */
void SPI::Receive(uint8_t *RxBuf, uint8_t len)
{
  pRxBuf = RxBuf; RxCount=len;

  while(RxCount>0)
  {
    //Wait till RXNE flag is set
    while(!pReg->SR[SPI_SR_RXNE_Pos]);

    //Check DFF bit
    if(pReg->CR1[SPI_CR1_DFF_Pos])
    {
      *(uint16_t*)pRxBuf = pReg->DR.to_ulong(); //Read data into buffer
      RxCount-=2;
      pRxBuf+=sizeof(uint16_t); //Point to next byte
    }
    else
    {
      *pRxBuf = pReg->DR.to_ulong(); //Read data into buffer
      RxCount--;
      pRxBuf+=sizeof(uint8_t); //Point to next location
    }
  }
}

/**
  * @name       TransmitReceive
  * @brief      Transmits and receives the data.
  * @param[in]  pTxData: Tx Data buffer
  * @param[in]  pRxData: Rx Data buffer
  * @param[in]  len: No. of bytes to read
  * @retval     None
  * @note       This is a blocking call
  */
void SPI::TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint8_t len)
{

  pRxBuf  = pRxData;
  pTxBuf  = pTxData;
  TxCount=len, RxCount=len;

  if(pReg->CR1[SPI_CR1_DFF_Pos])
  {
    if(Mode == SPI_DEV_SLAVE)
    {
      pReg->DR = *(uint16_t*)pTxBuf;
      pTxBuf += sizeof(uint16_t);
      TxCount--;
    }
    while(len > 0)
    {
      //Wait till TXE flag is set
      while(!pReg->SR[SPI_SR_TXE_Pos]);

      pReg->DR = *(uint16_t*)pTxBuf;
      TxCount-=2;
      pTxBuf+=sizeof(uint16_t);

      //Wait till RXNE flag is set
      while(!pReg->SR[SPI_SR_RXNE_Pos]);


      *(uint16_t*)pRxBuf = pReg->DR.to_ulong(); //Read data into buffer
      RxCount-=2;
      pRxBuf+=sizeof(uint16_t); //Point to next byte
    }
  }

  else
  {
    if(Mode == SPI_DEV_SLAVE)
    {
      pReg->DR = *pTxBuf;
      pTxBuf += sizeof(uint8_t);
      TxCount--;
    }
    while(len > 0)
    {
      //Wait till TXE flag is set
      while(!pReg->SR[SPI_SR_TXE_Pos]);

      pReg->DR = *pTxBuf;
      TxCount-=2;
      pTxBuf+=sizeof(uint8_t);

      //Wait till RXNE flag is set
      while(!pReg->SR[SPI_SR_RXNE_Pos]);


      *pRxBuf = pReg->DR.to_ulong(); //Read data into buffer
      RxCount-=2;
      pRxBuf+=sizeof(uint8_t); //Point to next byte
    }
  }
}

/**
  * @name   CloseCommunication
  * @brief  Ends the communication.
  * @param  None
  * @retval None
  * @note   None
  */
void SPI::CloseCommunication()
{
  //Wait till BSY flag is reset
  while(pReg->SR[SPI_SR_BSY_Pos]);
  pReg->CR1.reset(SPI_CR1_SPE_Pos);
}
