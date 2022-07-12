/*
 * usart.cpp
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */


#include "usart.h"
#include "rcc.h"

/**
  * @name       ClockControl
  * @brief      Enables or disables the clock
  * @param[in]  En_Dis: Enable or Disable
  * @retval     None
  * @note       None
  */
void USART::ClockControl(bool En_Dis)
{
  if(pReg == USART1) RCC->APB2ENR.set(RCC_APB2ENR_USART1EN_Pos, En_Dis);
  if(pReg == USART2) RCC->APB1ENR.set(RCC_APB1ENR_USART2EN_Pos, En_Dis);
  if(pReg == USART6) RCC->APB2ENR.set(RCC_APB2ENR_USART6EN_Pos, En_Dis);

  pReg->CR1.set(USART_CR1_UE_Pos, En_Dis);
}

/**
  * @name    DeInit
  * @brief  Resets the peripheral.
  * @param  None
  * @retval None
  * @note   None
  */
void USART::DeInit()
{
  ClockControl(DIS);

  if(pReg == USART1) RCC->APB2RSTR.reset(RCC_APB2RSTR_USART1RST_Pos);
  if(pReg == USART2) RCC->APB1RSTR.reset(RCC_APB1RSTR_USART2RST_Pos);
  if(pReg == USART6) RCC->APB2RSTR.reset(RCC_APB2RSTR_USART6RST_Pos);

  delete pReg;
}

/**
  * @name   Init
  * @brief  Initializes the peripheral.
  * @param  None
  * @retval None
  * @note   None
  */
void USART::Init(uint8_t wordlen, uint8_t parity, uint8_t mode, uint8_t stopbits, uint8_t hwflow, uint32_t baudrate)
{
  WordLen = wordlen; Parity = parity;
  Mode = mode;       StopBits = stopbits;
  HWFlow = hwflow;   Baud = baudrate;

  //Mode
  SET_BITS(pReg->CR1, Mode<<USART_CR1_RE_Pos);

  //Parity
  SET_BITS(pReg->CR1, USART_CR1_PS_Pos);

  //Word length
  pReg->CR1.set(USART_CR1_M_Pos, WordLen);

  //Stop bits
  SET_BITS(pReg->CR2, StopBits<<USART_CR2_STOP_Pos);

  //Hardware flow control
  SET_BITS(pReg->CR3, HWFlow<<USART_CR3_RTSE_Pos);

  //Baud rate
  SetBaudRate(Baud);

  ClockControl(EN);
}

/**
  * @name       Transmit
  * @brief      Sends the data.
  * @param[in]  TxBuf: TX buffer
  * @param[in]  Size: No. of bytes to send
  * @retval     None
  * @note       This is a blocking call
  */
void USART::Transmit(uint8_t *pTxData, uint8_t Size)
{
  pTxBuf = pTxData; Len = Size;

  for(uint8_t i=0; i<Len; i++)
  {
    //Wait till TXE flag is set
    while(!pReg->SR[USART_SR_TXE_Pos]);

    //If 9-bit data,
    if(WordLen == USART_WORD_9)
    {
      pReg->DR = (*(uint16_t*)pTxBuf & (uint16_t)0x01FF); //Send only 9 bits

      //If parity is disabled,
      if(Parity == USART_PARITY_NONE) {
        pTxBuf++; pTxBuf++; //Increment pointer
      }
      else pTxBuf++; //9th bit is replaced by parity
    }
    else //8-bit data
    {
      pReg->DR = (*pTxBuf  & (uint8_t)0xFF);
      pTxBuf++;
    }
  }

  //Wait till TC flag is set
  while(!pReg->SR[USART_SR_TC_Pos]);
}

/**
  * @name       Receive
  * @brief      Receives the data.
  * @param[in]  RxBuf: RX buffer
  * @param[in]  Size: No. of bytes left
  * @retval     None
  * @note       This is a blocking call
  */
void USART::Receive(uint8_t *pRxData, uint8_t Size)
{
  pRxBuf = pRxData; Len = Size;

  for(uint8_t i=0; i<Len; i++)
  {
    //Wait till RXNE flag is set
    while(!pReg->SR[USART_SR_RXNE_Pos]);

    //If 9-bit data,
    if(WordLen == USART_WORD_9)
    {
      //If parity is disabled,
      if(Parity == USART_PARITY_NONE)
      {
        //Read first 9 bits of data. so, mask the DR with 0x01FF
        *((uint16_t*) pRxBuf) = (pReg->DR.to_ulong()  & (uint16_t)0x01FF);
        pRxBuf++; pRxBuf++;
      }
      else
      {
        //Parity is used, so 8 bits will be of user data and 1 bit is parity
        *pRxBuf = (pReg->DR.to_ulong()  & (uint8_t)0xFF);
        pRxBuf++;
      }
    }
    else //8-bit data
    {
      if(Parity == USART_PARITY_NONE)
        *pRxBuf = pReg->DR.to_ulong(); //Read 8 bits from DR
      else
        *pRxBuf = (pReg->DR.to_ulong()  & (uint8_t)0x7F); //7 bits will be of user data and 1 bit is parity

      pRxBuf++;
    }
  }
}

/**
  * @name       TransmitReceive
  * @brief      Transmits and receives the data.
  * @param[in]  TxBuf: TX buffer
  * @param[in]  RxBuf: RX buffer
  * @param[in]  Size: No. of bytes left
  * @retval     None
  * @note       This is a blocking call
  */
void USART::TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size)
{
  pTxBuf = pTxData; pRxBuf = pRxData; Len = Size;

  for(uint8_t i=0; i<Len; i++)
  {
    /* Transmit */
    //Wait till TXE flag is set
    while(!pReg->SR[USART_SR_TXE_Pos]);

    //If 9-bit data,
    if(WordLen == USART_WORD_9)
    {
      pReg->DR = (*(uint16_t*)pTxBuf & (uint16_t)0x01FF); //Send only 9 bits

      //If parity is disabled,
      if(Parity == USART_PARITY_NONE) {
        pTxBuf++; pTxBuf++; //Increment pointer
      }
      else pTxBuf++; //9th bit is replaced by parity
    }
    else //8-bit data
    {
      pReg->DR = (*pTxBuf  & (uint8_t)0xFF);
      pTxBuf++;
    }

    /* Receive */
    //Wait till RXNE flag is set
    while(!pReg->SR[USART_SR_RXNE_Pos]);

    //If 9-bit data,
    if(WordLen == USART_WORD_9)
    {
      //If parity is disabled,
      if(Parity == USART_PARITY_NONE)
      {
        //Read first 9 bits of data. so, mask the DR with 0x01FF
        *((uint16_t*) pRxBuf) = (pReg->DR.to_ulong()  & (uint16_t)0x01FF);
        pRxBuf++; pRxBuf++;
      }
      else
      {
        //Parity is used, so 8 bits will be of user data and 1 bit is parity
        *pRxBuf = (pReg->DR.to_ulong()  & (uint8_t)0xFF);
        pRxBuf++;
      }
    }
    else //8-bit data
    {
      if(Parity == USART_PARITY_NONE)
        *pRxBuf = pReg->DR.to_ulong(); //Read 8 bits from DR
      else
        *pRxBuf = (pReg->DR.to_ulong()  & (uint8_t)0x7F); //7 bits will be of user data and 1 bit is parity

      pRxBuf++;
    }
  }
}

/**
  * @name       SetBaudRate
  * @brief      Calculates the baud rate.
  * @param      None
  * @retval     None
  * @note       None
  */
void USART::SetBaudRate()
{
  uint32_t PCLKx, usartdiv;
  uint32_t M_part,F_part; //Mantissa and Fraction values
  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pReg == USART1 || pReg == USART6)
    PCLKx = GetPClock2();
  else
    PCLKx = GetPClock1();

  //Check for OVER8 configuration bit
  if(pReg->CR1[USART_CR1_OVER8_Pos])
    usartdiv = ((25 * PCLKx) / (2 *Baud)); //OVER8 = 1 , over sampling by 8
  else
    usartdiv = ((25 * PCLKx) / (4 *Baud)); //OVER8 = 1 , over sampling by 16

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pReg->CR1[USART_CR1_OVER8_Pos])
    F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07); //Over sampling by 8
  else
    F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F); //Over sampling by 16

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //Copy the value of tempreg in to BRR register
  pReg->BRR = tempreg;
}

