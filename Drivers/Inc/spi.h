/*
 * spi.h
 *
 *  Created on: 21-Jun-2022
 *      Author: Vaishnavi
 */

#ifndef SPI_H_
#define SPI_H_


#include "stm32f411xe.h"

/*
 * Utility macros
 */
//Device mode
#define SPI_DEV_SLAVE 0
#define SPI_DEV_MASTER 1

//Data frame size
#define SPI_DFF_8   0
#define SPI_DFF_16  1

//Data format
#define SPI_MSB 0
#define SPI_LSB 1

//Bus cofiguration
#define SPI_BUS_FD    0 //Full duplex
#define SPI_BUS_HD_TX (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE) //Half duplex for MASTER mode ONLY
#define SPI_BUS_HD_RX (SPI_CR1_BIDIMODE | ~SPI_CR1_BIDIOE) //Half duplex for SLAVE mode ONLY
#define SPI_BUS_S_TX  (~SPI_CR1_BIDIMODE | ~SPI_CR1_RXONLY) //Simplex TX
#define SPI_BUS_S_RX  (~SPI_CR1_BIDIMODE | SPI_CR1_RXONLY) //Simplex RX

//Baud rate
#define FREQ_DIV_2    0
#define FREQ_DIV_4    1
#define FREQ_DIV_8    2
#define FREQ_DIV_16   3
#define FREQ_DIV_32   4
#define FREQ_DIV_64   5
#define FREQ_DIV_128  6
#define FREQ_DIV_256  7

/*
 * Class definition
 */
class SPI : API<SPI_t>
{
  uint8_t *pRxBuf, *pTxBuf, TxCount, RxCount;
  uint8_t BusCfg, DFF, CPOL, CPHA, SSM;

public:
  SPI(){}
  SPI(SPI_t *reg, uint8_t devmode, uint8_t buscfg, uint8_t speed, uint8_t dff, uint8_t cpol, uint8_t cpha, uint8_t ssm)
  {
  	pReg=reg; Mode=devmode;
    Speed = speed;   CPHA = cpha;
    CPOL = cpol;     SSM = ssm;
    DFF = dff;       BusCfg = buscfg;
  }

  ~SPI() { DeInit(); }

  //Initialization
  void ClockControl(bool En_Dis);
  void Init();
  void DeInit();

  //Send and receive data - Blocking mode
  void Transmit(uint8_t *TxBuf, uint8_t len);
  void Receive(uint8_t *RxBuf, uint8_t len);
  void TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint8_t len);
  void CloseCommunication();
};


#endif /* SPI_H_ */
