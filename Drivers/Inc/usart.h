/*
 * usart.h
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */

#ifndef USART_H_
#define USART_H_


#include "stm32f411xe.h"
#include "main.h"

/*
 * Macros
 */
//Word length
#define USART_WORD_8  0
#define USART_WORD_9  1

//Parity
#define USART_PARITY_NONE   0
#define USART_PARITY_EVEN   2
#define USART_PARITY_ODD    3

//Mode
#define USART_MODE_RX     1
#define USART_MODE_TX     2
#define USART_MODE_TX_RX  3

//Stop bits: 1,0.5,2,1.5
#define USART_STOP_1    0
#define USART_STOP_0_5  1
#define USART_STOP_2    2
#define USART_STOP_1_5  3

//Baud Rate
#define USART_BAUD_1200     1200
#define USART_BAUD_2400     2400
#define USART_BAUD_4800     4800
#define USART_BAUD_9600     9600
#define USART_BAUD_19200    19200
#define USART_BAUD_38400    38400
#define USART_BAUD_57600    57600
#define USART_BAUD_115200   115200
#define USART_BAUD_230400   230400
#define USART_BAUD_460800   460800
#define USART_BAUD_921600   921600
#define USART_BAUD_2M       2000000
#define USART_BAUD_3M       3000000

//Hardware flow control
#define USART_HW_NONE     0
#define USART_HW_RTS      1
#define USART_HW_CTS      2
#define USART_HW_CTS_RTS  3

/*
 * Class definition
 */
class USART : API<USART_t>
{
  USART_t *pReg;
  uint8_t WordLen, Parity, Mode, StopBits, HWFlow;
  uint32_t Baud;
  uint8_t *pRxBuf, *pTxBuf, Len;

public:
  USART(){}
  USART(USART_t *reg) { pReg = reg; }
  ~USART() { DeInit(); }

  //Initialization
  void ClockControl(bool En_Dis);
  void Init(uint8_t WordLen, uint8_t Parity, uint8_t Mode, uint8_t StopBits, uint8_t HWFlow, uint32_t Baudrate);
  void DeInit();
  void SetBaudRate(uint32_t BaudRate);

  //Data transmission
  void Transmit(uint8_t *pTxData, uint8_t Size);
  void Receive(uint8_t *pRxData, uint8_t Size);
  void TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
};


#endif /* USART_H_ */
