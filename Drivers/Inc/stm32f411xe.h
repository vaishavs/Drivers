/**
  ******************************************************************************
  * @file    stm32f411xe.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32F411xE Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - peripherals registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32f411xe
  * @{
  */
    
#ifndef __STM32F411xE_H
#define __STM32F411xE_H

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
  */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                   */

#include <cstdint>
#include <stdbool.h>
#include <bitset>

using namespace std;

/*
 * @addtogroup Type definitions
 * @{
 */

typedef bitset<64> bit64_t; /* DO NOT USE */
typedef bitset<32> bit32_t; //32-bit architecture

/*
 * @}
 */

/*
 * @addtogroup Utility Macros
 * @{
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
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
/*
 * @}
 */


/**
  * @}
  */
  
/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85      /*!< SPI5 global Interrupt                                              */
} IRQn_Type;

/**
  * @}
  */


/** @addtogroup Peripheral_registers_structures
  * @{
  */   

/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  bit32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
  bit32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  bit32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  bit32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  bit32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  bit32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  bit32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  bit32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  bit32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  bit32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  bit32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  bit32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  bit32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
  bit32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
  bit32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  bit32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
  bit32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
  bit32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
  bit32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
  bit32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_t;

typedef struct
{
  bit32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  bit32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  bit32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_t;

/** 
  * @brief CRC calculation unit 
  */

typedef struct
{
  bit32_t DR;         /*!< CRC Data register,             Address offset: 0x00 */
  uint8_t  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
  uint8_t       RESERVED0;  /*!< Reserved, 0x05                                      */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                      */
  bit32_t CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_t;

/** 
  * @brief Debug MCU
  */

typedef struct
{
  bit32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  bit32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  bit32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  bit32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_t;


/** 
  * @brief DMA Controller
  */

typedef struct
{
  bit32_t CR;     /*!< DMA stream x configuration register      */
  bit32_t NDTR;   /*!< DMA stream x number of data register     */
  bit32_t PAR;    /*!< DMA stream x peripheral address register */
  bit32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  bit32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  bit32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_t;

typedef struct
{
  bit32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  bit32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  bit32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  bit32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_t;

/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  bit32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  bit32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  bit32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  bit32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  bit32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  bit32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_t;

/** 
  * @brief FLASH Registers
  */

typedef struct
{
  bit32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  bit32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  bit32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  bit32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  bit32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  bit32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  bit32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_t;

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  bit32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  bit32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  bit32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  bit32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  bit32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  bit32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  bit32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  bit32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  bit32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_t;

/** 
  * @brief System configuration controller
  */

typedef struct
{
  bit32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  bit32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  bit32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  bit32_t RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  bit32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_t;

/** 
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  bit32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  bit32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  bit32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  bit32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  bit32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  bit32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  bit32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  bit32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  bit32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  bit32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_t;

/** 
  * @brief Independent WATCHDOG
  */

typedef struct
{
  bit32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  bit32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  bit32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  bit32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_t;


/** 
  * @brief Power Control
  */

typedef struct
{
  bit32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  bit32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_t;

/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  bit32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  bit32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  bit32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  bit32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  bit32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  bit32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  bit32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  bit32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  bit32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  bit32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  bit32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  bit32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  bit32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  bit32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  bit32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  bit32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  bit32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  bit32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  bit32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  bit32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  bit32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  bit32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  bit32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  bit32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  bit32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  bit32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  bit32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  bit32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  bit32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  bit32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  bit32_t      RESERVED7[1];  /*!< Reserved, 0x88                                                                    */
  bit32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_t;

/** 
  * @brief Real-Time Clock
  */

typedef struct
{
  bit32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  bit32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  bit32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  bit32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  bit32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  bit32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  bit32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  bit32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  bit32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  bit32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  bit32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
  bit32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
  bit32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  bit32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  bit32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  bit32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
  bit32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  bit32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  bit32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  bit32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  bit32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  bit32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  bit32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  bit32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  bit32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  bit32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  bit32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  bit32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  bit32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  bit32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  bit32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  bit32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  bit32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  bit32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  bit32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  bit32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  bit32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  bit32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  bit32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  bit32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_t;

/** 
  * @brief SD host Interface
  */

typedef struct
{
  bit32_t POWER;                 /*!< SDIO power control register,    Address offset: 0x00 */
  bit32_t CLKCR;                 /*!< SDI clock control register,     Address offset: 0x04 */
  bit32_t ARG;                   /*!< SDIO argument register,         Address offset: 0x08 */
  bit32_t CMD;                   /*!< SDIO command register,          Address offset: 0x0C */
  const bit32_t  RESPCMD;        /*!< SDIO command response register, Address offset: 0x10 */
  const bit32_t  RESP1;          /*!< SDIO response 1 register,       Address offset: 0x14 */
  const bit32_t  RESP2;          /*!< SDIO response 2 register,       Address offset: 0x18 */
  const bit32_t  RESP3;          /*!< SDIO response 3 register,       Address offset: 0x1C */
  const bit32_t  RESP4;          /*!< SDIO response 4 register,       Address offset: 0x20 */
  bit32_t DTIMER;                /*!< SDIO data timer register,       Address offset: 0x24 */
  bit32_t DLEN;                  /*!< SDIO data length register,      Address offset: 0x28 */
  bit32_t DCTRL;                 /*!< SDIO data control register,     Address offset: 0x2C */
  const bit32_t  DCOUNT;         /*!< SDIO data counter register,     Address offset: 0x30 */
  const bit32_t  STA;            /*!< SDIO status register,           Address offset: 0x34 */
  bit32_t ICR;                   /*!< SDIO interrupt clear register,  Address offset: 0x38 */
  bit32_t MASK;                  /*!< SDIO mask register,             Address offset: 0x3C */
  bit32_t      RESERVED0[2];          /*!< Reserved, 0x40-0x44                                  */
  const bit32_t  FIFOCNT;        /*!< SDIO FIFO counter register,     Address offset: 0x48 */
  bit32_t      RESERVED1[13];         /*!< Reserved, 0x4C-0x7C                                  */
  bit32_t FIFO;                  /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_t;

/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  bit32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  bit32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  bit32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  bit32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  bit32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  bit32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  bit32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  bit32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  bit32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_t;


/** 
  * @brief TIM
  */

typedef struct
{
  bit32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  bit32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  bit32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  bit32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  bit32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  bit32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  bit32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  bit32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  bit32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  bit32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  bit32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  bit32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  bit32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  bit32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  bit32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  bit32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  bit32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  bit32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  bit32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  bit32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  bit32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_t;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  bit32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  bit32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  bit32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  bit32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  bit32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  bit32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  bit32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_t;

/** 
  * @brief Window WATCHDOG
  */

typedef struct
{
  bit32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  bit32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  bit32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_t;
/** 
  * @brief USB_OTG_Core_Registers
  */
typedef struct
{
  bit32_t GOTGCTL;              /*!< USB_OTG Control and Status Register          000h */
  bit32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
  bit32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
  bit32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
  bit32_t GRSTCTL;              /*!< Core Reset Register                          010h */
  bit32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
  bit32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
  bit32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
  bit32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
  bit32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
  bit32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  bit32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  bit32_t Reserved30[2];             /*!< Reserved                                     030h */
  bit32_t GCCFG;                /*!< General Purpose IO Register                  038h */
  bit32_t CID;                  /*!< User ID Register                             03Ch */
  bit32_t  Reserved40[48];           /*!< Reserved                                0x40-0xFF */
  bit32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
  bit32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;

/** 
  * @brief USB_OTG_device_Registers
  */
typedef struct 
{
  bit32_t DCFG;            /*!< dev Configuration Register   800h */
  bit32_t DCTL;            /*!< dev Control Register         804h */
  bit32_t DSTS;            /*!< dev Status Register (RO)     808h */
  bit32_t Reserved0C;           /*!< Reserved                     80Ch */
  bit32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  bit32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  bit32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  bit32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  bit32_t  Reserved20;          /*!< Reserved                     820h */
  bit32_t Reserved9;            /*!< Reserved                     824h */
  bit32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  bit32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  bit32_t DTHRCTL;         /*!< dev threshold                830h */
  bit32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  bit32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  bit32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  bit32_t Reserved40;           /*!< dedicated EP mask            840h */
  bit32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  bit32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  bit32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;

/** 
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct 
{
  bit32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  bit32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  bit32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  bit32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  bit32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  bit32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  bit32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  bit32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

/** 
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct 
{
  bit32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  bit32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  bit32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  bit32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  bit32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  bit32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  bit32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

/** 
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct 
{
  bit32_t HCFG;             /*!< Host Configuration Register          400h */
  bit32_t HFIR;             /*!< Host Frame Interval Register         404h */
  bit32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  bit32_t Reserved40C;           /*!< Reserved                             40Ch */
  bit32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  bit32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  bit32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;

/** 
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  bit32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  bit32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  bit32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  bit32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  bit32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  bit32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  bit32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(128 KB) base address in the alias region                             */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(128 KB) base address in the bit-band region                          */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0807FFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC1_COMMON_BASE      (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC1_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)
#define SPI5_BASE             (APB2PERIPH_BASE + 0x5000UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           0xE0042000UL
/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               0x50000000UL

#define USB_OTG_GLOBAL_BASE                  0x000UL
#define USB_OTG_DEVICE_BASE                  0x800UL
#define USB_OTG_IN_ENDPOINT_BASE             0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00UL
#define USB_OTG_EP_REG_SIZE                  0x20UL
#define USB_OTG_HOST_BASE                    0x400UL
#define USB_OTG_HOST_PORT_BASE               0x440UL
#define USB_OTG_HOST_CHANNEL_BASE            0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE            0x20UL
#define USB_OTG_PCGCCTL_BASE                 0xE00UL
#define USB_OTG_FIFO_BASE                    0x1000UL
#define USB_OTG_FIFO_SIZE                    0x1000UL

#define UID_BASE                     0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0UL           /*!< Package size register base address     */

#define NVIC_PR_BASE 	(volatile uint32_t*)0xE000E400

#define NVIC_ISER0  (volatile uint32_t*)0xE000E100
#define NVIC_ISER1  (volatile uint32_t*)0xE000E104
#define NVIC_ISER2  (volatile uint32_t*)0xE000E108
#define NVIC_ISER3  (volatile uint32_t*)0xE000E10C

#define NVIC_ICER0  (volatile uint32_t*)0XE000E180
#define NVIC_ICER1  (volatile uint32_t*)0XE000E184
#define NVIC_ICER2  (volatile uint32_t*)0XE000E188
#define NVIC_ICER3  (volatile uint32_t*)0XE000E18C
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */  
#define TIM2                ((TIM_t *) TIM2_BASE)
#define TIM3                ((TIM_t *) TIM3_BASE)
#define TIM4                ((TIM_t *) TIM4_BASE)
#define TIM5                ((TIM_t *) TIM5_BASE)
#define RTC                 ((RTC_t *) RTC_BASE)
#define WWDG                ((WWDG_t *) WWDG_BASE)
#define IWDG                ((IWDG_t *) IWDG_BASE)
#define I2S2ext             ((SPI_t *) I2S2ext_BASE)
#define SPI2                ((SPI_t *) SPI2_BASE)
#define SPI3                ((SPI_t *) SPI3_BASE)
#define I2S3ext             ((SPI_t *) I2S3ext_BASE)
#define USART2              ((USART_t *) USART2_BASE)
#define I2C1                ((I2C_t *) I2C1_BASE)
#define I2C2                ((I2C_t *) I2C2_BASE)
#define I2C3                ((I2C_t *) I2C3_BASE)
#define PWR                 ((PWR_t *) PWR_BASE)
#define TIM1                ((TIM_t *) TIM1_BASE)
#define USART1              ((USART_t *) USART1_BASE)
#define USART6              ((USART_t *) USART6_BASE)
#define ADC1                ((ADC_t *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_t *) ADC1_COMMON_BASE)
/* Legacy define */
#define ADC                  ADC1_COMMON
#define SDIO                ((SDIO_t *) SDIO_BASE)
#define SPI1                ((SPI_t *) SPI1_BASE)
#define SPI4                ((SPI_t *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_t *) SYSCFG_BASE)
#define EXTI                ((EXTI_t *) EXTI_BASE)
#define TIM9                ((TIM_t *) TIM9_BASE)
#define TIM10               ((TIM_t *) TIM10_BASE)
#define TIM11               ((TIM_t *) TIM11_BASE)
#define SPI5                ((SPI_t *) SPI5_BASE)
#define GPIOA               ((GPIO_t *) GPIOA_BASE)
#define GPIOB               ((GPIO_t *) GPIOB_BASE)
#define GPIOC               ((GPIO_t *) GPIOC_BASE)
#define GPIOD               ((GPIO_t *) GPIOD_BASE)
#define GPIOE               ((GPIO_t *) GPIOE_BASE)
#define GPIOH               ((GPIO_t *) GPIOH_BASE)
#define CRC                 ((CRC_t *) CRC_BASE)
#define RCC                 ((RCC_t *) RCC_BASE)
#define FLASH               ((FLASH_t *) FLASH_R_BASE)
#define DMA1                ((DMA_t *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_t *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_t *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_t *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_t *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_t *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_t *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_t *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_t *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_t *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_t *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_t *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_t *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_t *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_t *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_t *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_t *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_t *) DMA2_Stream7_BASE)
#define DBGMCU              ((DBGMCU_t *) DBGMCU_BASE)
#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Hardware_Constant_Definition
  * @{
  */
#define LSI_STARTUP_TIME                40U /*!< LSI Maximum startup time in us */
/**
  * @}
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */
    
/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_AWD_Pos            (0U)                                         
#define ADC_SR_AWD                (0x1UL << ADC_SR_AWD_Pos)                     /*!< 0x00000001 */
#define ADC_SR_EOC_Pos            (1U)                                         
#define ADC_SR_EOC                (0x1UL << ADC_SR_EOC_Pos)                     /*!< 0x00000002 */
#define ADC_SR_JEOC_Pos           (2U)                                         
#define ADC_SR_JEOC               (0x1UL << ADC_SR_JEOC_Pos)                    /*!< 0x00000004 */
#define ADC_SR_JSTRT_Pos          (3U)                                         
#define ADC_SR_JSTRT              (0x1UL << ADC_SR_JSTRT_Pos)                   /*!< 0x00000008 */
#define ADC_SR_STRT_Pos           (4U)                                         
#define ADC_SR_STRT               (0x1UL << ADC_SR_STRT_Pos)                    /*!< 0x00000010 */
#define ADC_SR_OVR_Pos            (5U)                                         
#define ADC_SR_OVR                (0x1UL << ADC_SR_OVR_Pos)                     /*!< 0x00000020 */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define ADC_CR1_AWDCH_Pos         (0U)                                         
#define ADC_CR1_AWDCH             (0x1FUL << ADC_CR1_AWDCH_Pos)                 /*!< 0x0000001F */
#define ADC_CR1_AWDCH_0           (0x01UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000001 */
#define ADC_CR1_AWDCH_1           (0x02UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000002 */
#define ADC_CR1_AWDCH_2           (0x04UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000004 */
#define ADC_CR1_AWDCH_3           (0x08UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000008 */
#define ADC_CR1_AWDCH_4           (0x10UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000010 */
#define ADC_CR1_EOCIE_Pos         (5U)                                         
#define ADC_CR1_EOCIE             (0x1UL << ADC_CR1_EOCIE_Pos)                  /*!< 0x00000020 */
#define ADC_CR1_AWDIE_Pos         (6U)                                         
#define ADC_CR1_AWDIE             (0x1UL << ADC_CR1_AWDIE_Pos)                  /*!< 0x00000040 */
#define ADC_CR1_JEOCIE_Pos        (7U)                                         
#define ADC_CR1_JEOCIE            (0x1UL << ADC_CR1_JEOCIE_Pos)                 /*!< 0x00000080 */
#define ADC_CR1_SCAN_Pos          (8U)                                         
#define ADC_CR1_SCAN              (0x1UL << ADC_CR1_SCAN_Pos)                   /*!< 0x00000100 */
#define ADC_CR1_AWDSGL_Pos        (9U)                                         
#define ADC_CR1_AWDSGL            (0x1UL << ADC_CR1_AWDSGL_Pos)                 /*!< 0x00000200 */
#define ADC_CR1_JAUTO_Pos         (10U)                                        
#define ADC_CR1_JAUTO             (0x1UL << ADC_CR1_JAUTO_Pos)                  /*!< 0x00000400 */
#define ADC_CR1_DISCEN_Pos        (11U)                                        
#define ADC_CR1_DISCEN            (0x1UL << ADC_CR1_DISCEN_Pos)                 /*!< 0x00000800 */
#define ADC_CR1_JDISCEN_Pos       (12U)                                        
#define ADC_CR1_JDISCEN           (0x1UL << ADC_CR1_JDISCEN_Pos)                /*!< 0x00001000 */
#define ADC_CR1_DISCNUM_Pos       (13U)                                        
#define ADC_CR1_DISCNUM           (0x7UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x0000E000 */
#define ADC_CR1_DISCNUM_0         (0x1UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00002000 */
#define ADC_CR1_DISCNUM_1         (0x2UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00004000 */
#define ADC_CR1_DISCNUM_2         (0x4UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00008000 */
#define ADC_CR1_JAWDEN_Pos        (22U)                                        
#define ADC_CR1_JAWDEN            ADC_CR1_JAWDEN                               /*!<Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN_Pos         (23U)                                        
#define ADC_CR1_AWDEN             (0x1UL << ADC_CR1_AWDEN_Pos)                  /*!< 0x00800000 */
#define ADC_CR1_RES_Pos           (24U)                                        
#define ADC_CR1_RES               (0x3UL << ADC_CR1_RES_Pos)                    /*!< 0x03000000 */
#define ADC_CR1_RES_0             (0x1UL << ADC_CR1_RES_Pos)                    /*!< 0x01000000 */
#define ADC_CR1_RES_1             (0x2UL << ADC_CR1_RES_Pos)                    /*!< 0x02000000 */
#define ADC_CR1_OVRIE_Pos         (26U)                                        
#define ADC_CR1_OVRIE             (0x1UL << ADC_CR1_OVRIE_Pos)                  /*!< 0x04000000 */
  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON_Pos          (0U)                                         
#define ADC_CR2_ADON              (0x1UL << ADC_CR2_ADON_Pos)                   /*!< 0x00000001 */
#define ADC_CR2_CONT_Pos          (1U)                                         
#define ADC_CR2_CONT              (0x1UL << ADC_CR2_CONT_Pos)                   /*!< 0x00000002 */
#define ADC_CR2_DMA_Pos           (8U)                                         
#define ADC_CR2_DMA               (0x1UL << ADC_CR2_DMA_Pos)                    /*!< 0x00000100 */
#define ADC_CR2_DDS_Pos           (9U)                                         
#define ADC_CR2_DDS               (0x1UL << ADC_CR2_DDS_Pos)                    /*!< 0x00000200 */
#define ADC_CR2_EOCS_Pos          (10U)                                        
#define ADC_CR2_EOCS              (0x1UL << ADC_CR2_EOCS_Pos)                   /*!< 0x00000400 */
#define ADC_CR2_ALIGN_Pos         (11U)                                        
#define ADC_CR2_ALIGN             (0x1UL << ADC_CR2_ALIGN_Pos)                  /*!< 0x00000800 */
#define ADC_CR2_JEXTSEL_Pos       (16U)                                        
#define ADC_CR2_JEXTSEL           (0xFUL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x000F0000 */
#define ADC_CR2_JEXTSEL_0         (0x1UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00010000 */
#define ADC_CR2_JEXTSEL_1         (0x2UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00020000 */
#define ADC_CR2_JEXTSEL_2         (0x4UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00040000 */
#define ADC_CR2_JEXTSEL_3         (0x8UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00080000 */
#define ADC_CR2_JEXTEN_Pos        (20U)                                        
#define ADC_CR2_JEXTEN            (0x3UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00300000 */
#define ADC_CR2_JEXTEN_0          (0x1UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00100000 */
#define ADC_CR2_JEXTEN_1          (0x2UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00200000 */
#define ADC_CR2_JSWSTART_Pos      (22U)                                        
#define ADC_CR2_JSWSTART          (0x1UL << ADC_CR2_JSWSTART_Pos)               /*!< 0x00400000 */
#define ADC_CR2_EXTSEL_Pos        (24U)                                        
#define ADC_CR2_EXTSEL            (0xFUL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x0F000000 */
#define ADC_CR2_EXTSEL_0          (0x1UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x01000000 */
#define ADC_CR2_EXTSEL_1          (0x2UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x02000000 */
#define ADC_CR2_EXTSEL_2          (0x4UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x04000000 */
#define ADC_CR2_EXTSEL_3          (0x8UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x08000000 */
#define ADC_CR2_EXTEN_Pos         (28U)                                        
#define ADC_CR2_EXTEN             (0x3UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x30000000 */
#define ADC_CR2_EXTEN_0           (0x1UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x10000000 */
#define ADC_CR2_EXTEN_1           (0x2UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x20000000 */
#define ADC_CR2_SWSTART_Pos       (30U)                                        
#define ADC_CR2_SWSTART           (0x1UL << ADC_CR2_SWSTART_Pos)                /*!< 0x40000000 */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define ADC_SMPR1_SMP10_Pos       (0U)                                         
#define ADC_SMPR1_SMP10           (0x7UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000007 */
#define ADC_SMPR1_SMP10_0         (0x1UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000001 */
#define ADC_SMPR1_SMP10_1         (0x2UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000002 */
#define ADC_SMPR1_SMP10_2         (0x4UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000004 */
#define ADC_SMPR1_SMP11_Pos       (3U)                                         
#define ADC_SMPR1_SMP11           (0x7UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000038 */
#define ADC_SMPR1_SMP11_0         (0x1UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000008 */
#define ADC_SMPR1_SMP11_1         (0x2UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000010 */
#define ADC_SMPR1_SMP11_2         (0x4UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000020 */
#define ADC_SMPR1_SMP12_Pos       (6U)                                         
#define ADC_SMPR1_SMP12           (0x7UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x000001C0 */
#define ADC_SMPR1_SMP12_0         (0x1UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000040 */
#define ADC_SMPR1_SMP12_1         (0x2UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000080 */
#define ADC_SMPR1_SMP12_2         (0x4UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000100 */
#define ADC_SMPR1_SMP13_Pos       (9U)                                         
#define ADC_SMPR1_SMP13           (0x7UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000E00 */
#define ADC_SMPR1_SMP13_0         (0x1UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000200 */
#define ADC_SMPR1_SMP13_1         (0x2UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000400 */
#define ADC_SMPR1_SMP13_2         (0x4UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000800 */
#define ADC_SMPR1_SMP14_Pos       (12U)                                        
#define ADC_SMPR1_SMP14           (0x7UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00007000 */
#define ADC_SMPR1_SMP14_0         (0x1UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00001000 */
#define ADC_SMPR1_SMP14_1         (0x2UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00002000 */
#define ADC_SMPR1_SMP14_2         (0x4UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00004000 */
#define ADC_SMPR1_SMP15_Pos       (15U)                                        
#define ADC_SMPR1_SMP15           (0x7UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00038000 */
#define ADC_SMPR1_SMP15_0         (0x1UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00008000 */
#define ADC_SMPR1_SMP15_1         (0x2UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00010000 */
#define ADC_SMPR1_SMP15_2         (0x4UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00020000 */
#define ADC_SMPR1_SMP16_Pos       (18U)                                        
#define ADC_SMPR1_SMP16           (0x7UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x001C0000 */
#define ADC_SMPR1_SMP16_0         (0x1UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00040000 */
#define ADC_SMPR1_SMP16_1         (0x2UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00080000 */
#define ADC_SMPR1_SMP16_2         (0x4UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00100000 */
#define ADC_SMPR1_SMP17_Pos       (21U)                                        
#define ADC_SMPR1_SMP17           (0x7UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00E00000 */
#define ADC_SMPR1_SMP17_0         (0x1UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00200000 */
#define ADC_SMPR1_SMP17_1         (0x2UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00400000 */
#define ADC_SMPR1_SMP17_2         (0x4UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00800000 */
#define ADC_SMPR1_SMP18_Pos       (24U)                                        
#define ADC_SMPR1_SMP18           (0x7UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x07000000 */
#define ADC_SMPR1_SMP18_0         (0x1UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x01000000 */
#define ADC_SMPR1_SMP18_1         (0x2UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x02000000 */
#define ADC_SMPR1_SMP18_2         (0x4UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x04000000 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0_Pos        (0U)                                         
#define ADC_SMPR2_SMP0            (0x7UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000007 */
#define ADC_SMPR2_SMP0_0          (0x1UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000001 */
#define ADC_SMPR2_SMP0_1          (0x2UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000002 */
#define ADC_SMPR2_SMP0_2          (0x4UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000004 */
#define ADC_SMPR2_SMP1_Pos        (3U)                                         
#define ADC_SMPR2_SMP1            (0x7UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000038 */
#define ADC_SMPR2_SMP1_0          (0x1UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000008 */
#define ADC_SMPR2_SMP1_1          (0x2UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000010 */
#define ADC_SMPR2_SMP1_2          (0x4UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000020 */
#define ADC_SMPR2_SMP2_Pos        (6U)                                         
#define ADC_SMPR2_SMP2            (0x7UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x000001C0 */
#define ADC_SMPR2_SMP2_0          (0x1UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000040 */
#define ADC_SMPR2_SMP2_1          (0x2UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000080 */
#define ADC_SMPR2_SMP2_2          (0x4UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000100 */
#define ADC_SMPR2_SMP3_Pos        (9U)                                         
#define ADC_SMPR2_SMP3            (0x7UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000E00 */
#define ADC_SMPR2_SMP3_0          (0x1UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000200 */
#define ADC_SMPR2_SMP3_1          (0x2UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000400 */
#define ADC_SMPR2_SMP3_2          (0x4UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000800 */
#define ADC_SMPR2_SMP4_Pos        (12U)                                        
#define ADC_SMPR2_SMP4            (0x7UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00007000 */
#define ADC_SMPR2_SMP4_0          (0x1UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00001000 */
#define ADC_SMPR2_SMP4_1          (0x2UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00002000 */
#define ADC_SMPR2_SMP4_2          (0x4UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00004000 */
#define ADC_SMPR2_SMP5_Pos        (15U)                                        
#define ADC_SMPR2_SMP5            (0x7UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00038000 */
#define ADC_SMPR2_SMP5_0          (0x1UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00008000 */
#define ADC_SMPR2_SMP5_1          (0x2UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00010000 */
#define ADC_SMPR2_SMP5_2          (0x4UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00020000 */
#define ADC_SMPR2_SMP6_Pos        (18U)                                        
#define ADC_SMPR2_SMP6            (0x7UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x001C0000 */
#define ADC_SMPR2_SMP6_0          (0x1UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00040000 */
#define ADC_SMPR2_SMP6_1          (0x2UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00080000 */
#define ADC_SMPR2_SMP6_2          (0x4UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00100000 */
#define ADC_SMPR2_SMP7_Pos        (21U)                                        
#define ADC_SMPR2_SMP7            (0x7UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00E00000 */
#define ADC_SMPR2_SMP7_0          (0x1UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00200000 */
#define ADC_SMPR2_SMP7_1          (0x2UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00400000 */
#define ADC_SMPR2_SMP7_2          (0x4UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00800000 */
#define ADC_SMPR2_SMP8_Pos        (24U)                                        
#define ADC_SMPR2_SMP8            (0x7UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x07000000 */
#define ADC_SMPR2_SMP8_0          (0x1UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x01000000 */
#define ADC_SMPR2_SMP8_1          (0x2UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x02000000 */
#define ADC_SMPR2_SMP8_2          (0x4UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x04000000 */
#define ADC_SMPR2_SMP9_Pos        (27U)                                        
#define ADC_SMPR2_SMP9            (0x7UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x38000000 */
#define ADC_SMPR2_SMP9_0          (0x1UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x08000000 */
#define ADC_SMPR2_SMP9_1          (0x2UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x10000000 */
#define ADC_SMPR2_SMP9_2          (0x4UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x20000000 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define ADC_JOFR1_JOFFSET1_Pos    (0U)                                         
#define ADC_JOFR1_JOFFSET1        (0xFFFUL << ADC_JOFR1_JOFFSET1_Pos)           /*!< 0x00000FFF */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define ADC_JOFR2_JOFFSET2_Pos    (0U)                                         
#define ADC_JOFR2_JOFFSET2        (0xFFFUL << ADC_JOFR2_JOFFSET2_Pos)           /*!< 0x00000FFF */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define ADC_JOFR3_JOFFSET3_Pos    (0U)                                         
#define ADC_JOFR3_JOFFSET3        (0xFFFUL << ADC_JOFR3_JOFFSET3_Pos)           /*!< 0x00000FFF */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define ADC_JOFR4_JOFFSET4_Pos    (0U)                                         
#define ADC_JOFR4_JOFFSET4        (0xFFFUL << ADC_JOFR4_JOFFSET4_Pos)           /*!< 0x00000FFF */

/*******************  Bit definition for ADC_HTR register  ********************/
#define ADC_HTR_HT_Pos            (0U)                                         
#define ADC_HTR_HT                (0xFFFUL << ADC_HTR_HT_Pos)                   /*!< 0x00000FFF */

/*******************  Bit definition for ADC_LTR register  ********************/
#define ADC_LTR_LT_Pos            (0U)                                         
#define ADC_LTR_LT                (0xFFFUL << ADC_LTR_LT_Pos)                   /*!< 0x00000FFF */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define ADC_SQR1_SQ13_Pos         (0U)                                         
#define ADC_SQR1_SQ13             (0x1FUL << ADC_SQR1_SQ13_Pos)                 /*!< 0x0000001F */
#define ADC_SQR1_SQ13_0           (0x01UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000001 */
#define ADC_SQR1_SQ13_1           (0x02UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000002 */
#define ADC_SQR1_SQ13_2           (0x04UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000004 */
#define ADC_SQR1_SQ13_3           (0x08UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000008 */
#define ADC_SQR1_SQ13_4           (0x10UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000010 */
#define ADC_SQR1_SQ14_Pos         (5U)                                         
#define ADC_SQR1_SQ14             (0x1FUL << ADC_SQR1_SQ14_Pos)                 /*!< 0x000003E0 */
#define ADC_SQR1_SQ14_0           (0x01UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000020 */
#define ADC_SQR1_SQ14_1           (0x02UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000040 */
#define ADC_SQR1_SQ14_2           (0x04UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000080 */
#define ADC_SQR1_SQ14_3           (0x08UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000100 */
#define ADC_SQR1_SQ14_4           (0x10UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000200 */
#define ADC_SQR1_SQ15_Pos         (10U)                                        
#define ADC_SQR1_SQ15             (0x1FUL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00007C00 */
#define ADC_SQR1_SQ15_0           (0x01UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000400 */
#define ADC_SQR1_SQ15_1           (0x02UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000800 */
#define ADC_SQR1_SQ15_2           (0x04UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00001000 */
#define ADC_SQR1_SQ15_3           (0x08UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00002000 */
#define ADC_SQR1_SQ15_4           (0x10UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00004000 */
#define ADC_SQR1_SQ16_Pos         (15U)                                        
#define ADC_SQR1_SQ16             (0x1FUL << ADC_SQR1_SQ16_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR1_SQ16_0           (0x01UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00008000 */
#define ADC_SQR1_SQ16_1           (0x02UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00010000 */
#define ADC_SQR1_SQ16_2           (0x04UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00020000 */
#define ADC_SQR1_SQ16_3           (0x08UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00040000 */
#define ADC_SQR1_SQ16_4           (0x10UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00080000 */
#define ADC_SQR1_L_Pos            (20U)                                        
#define ADC_SQR1_L                (0xFUL << ADC_SQR1_L_Pos)                     /*!< 0x00F00000 */
#define ADC_SQR1_L_0              (0x1UL << ADC_SQR1_L_Pos)                     /*!< 0x00100000 */
#define ADC_SQR1_L_1              (0x2UL << ADC_SQR1_L_Pos)                     /*!< 0x00200000 */
#define ADC_SQR1_L_2              (0x4UL << ADC_SQR1_L_Pos)                     /*!< 0x00400000 */
#define ADC_SQR1_L_3              (0x8UL << ADC_SQR1_L_Pos)                     /*!< 0x00800000 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define ADC_SQR2_SQ7_Pos          (0U)                                         
#define ADC_SQR2_SQ7              (0x1FUL << ADC_SQR2_SQ7_Pos)                  /*!< 0x0000001F */
#define ADC_SQR2_SQ7_0            (0x01UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000001 */
#define ADC_SQR2_SQ7_1            (0x02UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000002 */
#define ADC_SQR2_SQ7_2            (0x04UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000004 */
#define ADC_SQR2_SQ7_3            (0x08UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000008 */
#define ADC_SQR2_SQ7_4            (0x10UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000010 */
#define ADC_SQR2_SQ8_Pos          (5U)                                         
#define ADC_SQR2_SQ8              (0x1FUL << ADC_SQR2_SQ8_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR2_SQ8_0            (0x01UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000020 */
#define ADC_SQR2_SQ8_1            (0x02UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000040 */
#define ADC_SQR2_SQ8_2            (0x04UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000080 */
#define ADC_SQR2_SQ8_3            (0x08UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000100 */
#define ADC_SQR2_SQ8_4            (0x10UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000200 */
#define ADC_SQR2_SQ9_Pos          (10U)                                        
#define ADC_SQR2_SQ9              (0x1FUL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR2_SQ9_0            (0x01UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000400 */
#define ADC_SQR2_SQ9_1            (0x02UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000800 */
#define ADC_SQR2_SQ9_2            (0x04UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00001000 */
#define ADC_SQR2_SQ9_3            (0x08UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00002000 */
#define ADC_SQR2_SQ9_4            (0x10UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00004000 */
#define ADC_SQR2_SQ10_Pos         (15U)                                        
#define ADC_SQR2_SQ10             (0x1FUL << ADC_SQR2_SQ10_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR2_SQ10_0           (0x01UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00008000 */
#define ADC_SQR2_SQ10_1           (0x02UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00010000 */
#define ADC_SQR2_SQ10_2           (0x04UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00020000 */
#define ADC_SQR2_SQ10_3           (0x08UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00040000 */
#define ADC_SQR2_SQ10_4           (0x10UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00080000 */
#define ADC_SQR2_SQ11_Pos         (20U)                                        
#define ADC_SQR2_SQ11             (0x1FUL << ADC_SQR2_SQ11_Pos)                 /*!< 0x01F00000 */
#define ADC_SQR2_SQ11_0           (0x01UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00100000 */
#define ADC_SQR2_SQ11_1           (0x02UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00200000 */
#define ADC_SQR2_SQ11_2           (0x04UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00400000 */
#define ADC_SQR2_SQ11_3           (0x08UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00800000 */
#define ADC_SQR2_SQ11_4           (0x10UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x01000000 */
#define ADC_SQR2_SQ12_Pos         (25U)                                        
#define ADC_SQR2_SQ12             (0x1FUL << ADC_SQR2_SQ12_Pos)                 /*!< 0x3E000000 */
#define ADC_SQR2_SQ12_0           (0x01UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x02000000 */
#define ADC_SQR2_SQ12_1           (0x02UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x04000000 */
#define ADC_SQR2_SQ12_2           (0x04UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x08000000 */
#define ADC_SQR2_SQ12_3           (0x08UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x10000000 */
#define ADC_SQR2_SQ12_4           (0x10UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x20000000 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define ADC_SQR3_SQ1_Pos          (0U)                                         
#define ADC_SQR3_SQ1              (0x1FUL << ADC_SQR3_SQ1_Pos)                  /*!< 0x0000001F */
#define ADC_SQR3_SQ1_0            (0x01UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000001 */
#define ADC_SQR3_SQ1_1            (0x02UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000002 */
#define ADC_SQR3_SQ1_2            (0x04UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000004 */
#define ADC_SQR3_SQ1_3            (0x08UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000008 */
#define ADC_SQR3_SQ1_4            (0x10UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000010 */
#define ADC_SQR3_SQ2_Pos          (5U)                                         
#define ADC_SQR3_SQ2              (0x1FUL << ADC_SQR3_SQ2_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR3_SQ2_0            (0x01UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000020 */
#define ADC_SQR3_SQ2_1            (0x02UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000040 */
#define ADC_SQR3_SQ2_2            (0x04UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000080 */
#define ADC_SQR3_SQ2_3            (0x08UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000100 */
#define ADC_SQR3_SQ2_4            (0x10UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000200 */
#define ADC_SQR3_SQ3_Pos          (10U)                                        
#define ADC_SQR3_SQ3              (0x1FUL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR3_SQ3_0            (0x01UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000400 */
#define ADC_SQR3_SQ3_1            (0x02UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000800 */
#define ADC_SQR3_SQ3_2            (0x04UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00001000 */
#define ADC_SQR3_SQ3_3            (0x08UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00002000 */
#define ADC_SQR3_SQ3_4            (0x10UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00004000 */
#define ADC_SQR3_SQ4_Pos          (15U)                                        
#define ADC_SQR3_SQ4              (0x1FUL << ADC_SQR3_SQ4_Pos)                  /*!< 0x000F8000 */
#define ADC_SQR3_SQ4_0            (0x01UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00008000 */
#define ADC_SQR3_SQ4_1            (0x02UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00010000 */
#define ADC_SQR3_SQ4_2            (0x04UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00020000 */
#define ADC_SQR3_SQ4_3            (0x08UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00040000 */
#define ADC_SQR3_SQ4_4            (0x10UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00080000 */
#define ADC_SQR3_SQ5_Pos          (20U)                                        
#define ADC_SQR3_SQ5              (0x1FUL << ADC_SQR3_SQ5_Pos)                  /*!< 0x01F00000 */
#define ADC_SQR3_SQ5_0            (0x01UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00100000 */
#define ADC_SQR3_SQ5_1            (0x02UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00200000 */
#define ADC_SQR3_SQ5_2            (0x04UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00400000 */
#define ADC_SQR3_SQ5_3            (0x08UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00800000 */
#define ADC_SQR3_SQ5_4            (0x10UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x01000000 */
#define ADC_SQR3_SQ6_Pos          (25U)                                        
#define ADC_SQR3_SQ6              (0x1FUL << ADC_SQR3_SQ6_Pos)                  /*!< 0x3E000000 */
#define ADC_SQR3_SQ6_0            (0x01UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x02000000 */
#define ADC_SQR3_SQ6_1            (0x02UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x04000000 */
#define ADC_SQR3_SQ6_2            (0x04UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x08000000 */
#define ADC_SQR3_SQ6_3            (0x08UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x10000000 */
#define ADC_SQR3_SQ6_4            (0x10UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x20000000 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define ADC_JSQR_JSQ1_Pos         (0U)                                         
#define ADC_JSQR_JSQ1             (0x1FUL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x0000001F */
#define ADC_JSQR_JSQ1_0           (0x01UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000001 */
#define ADC_JSQR_JSQ1_1           (0x02UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000002 */
#define ADC_JSQR_JSQ1_2           (0x04UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000004 */
#define ADC_JSQR_JSQ1_3           (0x08UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000008 */
#define ADC_JSQR_JSQ1_4           (0x10UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000010 */
#define ADC_JSQR_JSQ2_Pos         (5U)                                         
#define ADC_JSQR_JSQ2             (0x1FUL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x000003E0 */
#define ADC_JSQR_JSQ2_0           (0x01UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000020 */
#define ADC_JSQR_JSQ2_1           (0x02UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000040 */
#define ADC_JSQR_JSQ2_2           (0x04UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000080 */
#define ADC_JSQR_JSQ2_3           (0x08UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000100 */
#define ADC_JSQR_JSQ2_4           (0x10UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000200 */
#define ADC_JSQR_JSQ3_Pos         (10U)                                        
#define ADC_JSQR_JSQ3             (0x1FUL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00007C00 */
#define ADC_JSQR_JSQ3_0           (0x01UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000400 */
#define ADC_JSQR_JSQ3_1           (0x02UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000800 */
#define ADC_JSQR_JSQ3_2           (0x04UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00001000 */
#define ADC_JSQR_JSQ3_3           (0x08UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00002000 */
#define ADC_JSQR_JSQ3_4           (0x10UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00004000 */
#define ADC_JSQR_JSQ4_Pos         (15U)                                        
#define ADC_JSQR_JSQ4             (0x1FUL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x000F8000 */
#define ADC_JSQR_JSQ4_0           (0x01UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00008000 */
#define ADC_JSQR_JSQ4_1           (0x02UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00010000 */
#define ADC_JSQR_JSQ4_2           (0x04UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00020000 */
#define ADC_JSQR_JSQ4_3           (0x08UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00040000 */
#define ADC_JSQR_JSQ4_4           (0x10UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00080000 */
#define ADC_JSQR_JL_Pos           (20U)                                        
#define ADC_JSQR_JL               (0x3UL << ADC_JSQR_JL_Pos)                    /*!< 0x00300000 */
#define ADC_JSQR_JL_0             (0x1UL << ADC_JSQR_JL_Pos)                    /*!< 0x00100000 */
#define ADC_JSQR_JL_1             (0x2UL << ADC_JSQR_JL_Pos)                    /*!< 0x00200000 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define ADC_JDR1_JDATA_Pos        (0U)                                         
#define ADC_JDR1_JDATA            (0xFFFFUL << ADC_JDR1_JDATA_Pos)              /*!< 0x0000FFFF */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define ADC_JDR2_JDATA_Pos        (0U)                                         
#define ADC_JDR2_JDATA            (0xFFFFUL << ADC_JDR2_JDATA_Pos)              /*!< 0x0000FFFF */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define ADC_JDR3_JDATA_Pos        (0U)                                         
#define ADC_JDR3_JDATA            (0xFFFFUL << ADC_JDR3_JDATA_Pos)              /*!< 0x0000FFFF */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define ADC_JDR4_JDATA_Pos        (0U)                                         
#define ADC_JDR4_JDATA            (0xFFFFUL << ADC_JDR4_JDATA_Pos)              /*!< 0x0000FFFF */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)                                         
#define ADC_DR_DATA               (0xFFFFUL << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_ADC2DATA_Pos       (16U)                                        
#define ADC_DR_ADC2DATA           (0xFFFFUL << ADC_DR_ADC2DATA_Pos)             /*!< 0xFFFF0000 */

/*******************  Bit definition for ADC_CSR register  ********************/
#define ADC_CSR_AWD1_Pos          (0U)                                         
#define ADC_CSR_AWD1              (0x1UL << ADC_CSR_AWD1_Pos)                   /*!< 0x00000001 */
#define ADC_CSR_EOC1_Pos          (1U)                                         
#define ADC_CSR_EOC1              (0x1UL << ADC_CSR_EOC1_Pos)                   /*!< 0x00000002 */
#define ADC_CSR_JEOC1_Pos         (2U)                                         
#define ADC_CSR_JEOC1             (0x1UL << ADC_CSR_JEOC1_Pos)                  /*!< 0x00000004 */
#define ADC_CSR_JSTRT1_Pos        (3U)                                         
#define ADC_CSR_JSTRT1            (0x1UL << ADC_CSR_JSTRT1_Pos)                 /*!< 0x00000008 */
#define ADC_CSR_STRT1_Pos         (4U)                                         
#define ADC_CSR_STRT1             (0x1UL << ADC_CSR_STRT1_Pos)                  /*!< 0x00000010 */
#define ADC_CSR_OVR1_Pos          (5U)                                         
#define ADC_CSR_OVR1              (0x1UL << ADC_CSR_OVR1_Pos)                   /*!< 0x00000020 */

/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_MULTI_Pos         (0U)                                         
#define ADC_CCR_MULTI             (0x1FUL << ADC_CCR_MULTI_Pos)                 /*!< 0x0000001F */
#define ADC_CCR_MULTI_0           (0x01UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000001 */
#define ADC_CCR_MULTI_1           (0x02UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000002 */
#define ADC_CCR_MULTI_2           (0x04UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000004 */
#define ADC_CCR_MULTI_3           (0x08UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000008 */
#define ADC_CCR_MULTI_4           (0x10UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000010 */
#define ADC_CCR_DELAY_Pos         (8U)                                         
#define ADC_CCR_DELAY             (0xFUL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000F00 */
#define ADC_CCR_DELAY_0           (0x1UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000100 */
#define ADC_CCR_DELAY_1           (0x2UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000200 */
#define ADC_CCR_DELAY_2           (0x4UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000400 */
#define ADC_CCR_DELAY_3           (0x8UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000800 */
#define ADC_CCR_DDS_Pos           (13U)                                        
#define ADC_CCR_DDS               (0x1UL << ADC_CCR_DDS_Pos)                    /*!< 0x00002000 */
#define ADC_CCR_DMA_Pos           (14U)                                        
#define ADC_CCR_DMA               (0x3UL << ADC_CCR_DMA_Pos)                    /*!< 0x0000C000 */
#define ADC_CCR_DMA_0             (0x1UL << ADC_CCR_DMA_Pos)                    /*!< 0x00004000 */
#define ADC_CCR_DMA_1             (0x2UL << ADC_CCR_DMA_Pos)                    /*!< 0x00008000 */
#define ADC_CCR_ADCPRE_Pos        (16U)                                        
#define ADC_CCR_ADCPRE            (0x3UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00030000 */
#define ADC_CCR_ADCPRE_0          (0x1UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00010000 */
#define ADC_CCR_ADCPRE_1          (0x2UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00020000 */
#define ADC_CCR_VBATE_Pos         (22U)                                        
#define ADC_CCR_VBATE             (0x1UL << ADC_CCR_VBATE_Pos)                  /*!< 0x00400000 */
#define ADC_CCR_TSVREFE_Pos       (23U)                                        
#define ADC_CCR_TSVREFE           (0x1UL << ADC_CCR_TSVREFE_Pos)                /*!< 0x00800000 */

/*******************  Bit definition for ADC_CDR register  ********************/
#define ADC_CDR_DATA1_Pos         (0U)                                         
#define ADC_CDR_DATA1             (0xFFFFUL << ADC_CDR_DATA1_Pos)               /*!< 0x0000FFFF */
#define ADC_CDR_DATA2_Pos         (16U)                                        
#define ADC_CDR_DATA2             (0xFFFFUL << ADC_CDR_DATA2_Pos)               /*!< 0xFFFF0000 */




/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)                                         
#define EXTI_IMR_MR0              (0x1UL << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR1_Pos          (1U)                                         
#define EXTI_IMR_MR1              (0x1UL << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR2_Pos          (2U)                                         
#define EXTI_IMR_MR2              (0x1UL << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR3_Pos          (3U)                                         
#define EXTI_IMR_MR3              (0x1UL << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR4_Pos          (4U)                                         
#define EXTI_IMR_MR4              (0x1UL << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR5_Pos          (5U)                                         
#define EXTI_IMR_MR5              (0x1UL << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR6_Pos          (6U)                                         
#define EXTI_IMR_MR6              (0x1UL << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR7_Pos          (7U)                                         
#define EXTI_IMR_MR7              (0x1UL << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR8_Pos          (8U)                                         
#define EXTI_IMR_MR8              (0x1UL << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR9_Pos          (9U)                                         
#define EXTI_IMR_MR9              (0x1UL << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR10_Pos         (10U)                                        
#define EXTI_IMR_MR10             (0x1UL << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR11_Pos         (11U)                                        
#define EXTI_IMR_MR11             (0x1UL << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR12_Pos         (12U)                                        
#define EXTI_IMR_MR12             (0x1UL << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR13_Pos         (13U)                                        
#define EXTI_IMR_MR13             (0x1UL << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR14_Pos         (14U)                                        
#define EXTI_IMR_MR14             (0x1UL << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR15_Pos         (15U)                                        
#define EXTI_IMR_MR15             (0x1UL << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR16_Pos         (16U)                                        
#define EXTI_IMR_MR16             (0x1UL << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR17_Pos         (17U)                                        
#define EXTI_IMR_MR17             (0x1UL << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR18_Pos         (18U)                                        
#define EXTI_IMR_MR18             (0x1UL << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR19_Pos         (19U)                                        
#define EXTI_IMR_MR19             (0x1UL << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR20_Pos         (20U)                                        
#define EXTI_IMR_MR20             (0x1UL << EXTI_IMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_IMR_MR21_Pos         (21U)                                        
#define EXTI_IMR_MR21             (0x1UL << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR22_Pos         (22U)                                        
#define EXTI_IMR_MR22             (0x1UL << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */

#define EXTI_IMR_IM_Pos           (0U)                                         
#define EXTI_IMR_IM               (0x7FFFFFUL << EXTI_IMR_IM_Pos)               /*!< 0x007FFFFF */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos          (0U)                                         
#define EXTI_EMR_MR0              (0x1UL << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR1_Pos          (1U)                                         
#define EXTI_EMR_MR1              (0x1UL << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR2_Pos          (2U)                                         
#define EXTI_EMR_MR2              (0x1UL << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR3_Pos          (3U)                                         
#define EXTI_EMR_MR3              (0x1UL << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR4_Pos          (4U)                                         
#define EXTI_EMR_MR4              (0x1UL << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR5_Pos          (5U)                                         
#define EXTI_EMR_MR5              (0x1UL << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR6_Pos          (6U)                                         
#define EXTI_EMR_MR6              (0x1UL << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR7_Pos          (7U)                                         
#define EXTI_EMR_MR7              (0x1UL << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR8_Pos          (8U)                                         
#define EXTI_EMR_MR8              (0x1UL << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR9_Pos          (9U)                                         
#define EXTI_EMR_MR9              (0x1UL << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR10_Pos         (10U)                                        
#define EXTI_EMR_MR10             (0x1UL << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR11_Pos         (11U)                                        
#define EXTI_EMR_MR11             (0x1UL << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR12_Pos         (12U)                                        
#define EXTI_EMR_MR12             (0x1UL << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR13_Pos         (13U)                                        
#define EXTI_EMR_MR13             (0x1UL << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR14_Pos         (14U)                                        
#define EXTI_EMR_MR14             (0x1UL << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR15_Pos         (15U)                                        
#define EXTI_EMR_MR15             (0x1UL << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR16_Pos         (16U)                                        
#define EXTI_EMR_MR16             (0x1UL << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR17_Pos         (17U)                                        
#define EXTI_EMR_MR17             (0x1UL << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR18_Pos         (18U)                                        
#define EXTI_EMR_MR18             (0x1UL << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR19_Pos         (19U)                                        
#define EXTI_EMR_MR19             (0x1UL << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR20_Pos         (20U)                                        
#define EXTI_EMR_MR20             (0x1UL << EXTI_EMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_EMR_MR21_Pos         (21U)                                        
#define EXTI_EMR_MR21             (0x1UL << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR22_Pos         (22U)                                        
#define EXTI_EMR_MR22             (0x1UL << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */


/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos         (0U)                                         
#define EXTI_RTSR_TR0             (0x1UL << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR1_Pos         (1U)                                         
#define EXTI_RTSR_TR1             (0x1UL << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR2_Pos         (2U)                                         
#define EXTI_RTSR_TR2             (0x1UL << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR3_Pos         (3U)                                         
#define EXTI_RTSR_TR3             (0x1UL << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR4_Pos         (4U)                                         
#define EXTI_RTSR_TR4             (0x1UL << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR5_Pos         (5U)                                         
#define EXTI_RTSR_TR5             (0x1UL << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR6_Pos         (6U)                                         
#define EXTI_RTSR_TR6             (0x1UL << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR7_Pos         (7U)                                         
#define EXTI_RTSR_TR7             (0x1UL << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR8_Pos         (8U)                                         
#define EXTI_RTSR_TR8             (0x1UL << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR9_Pos         (9U)                                         
#define EXTI_RTSR_TR9             (0x1UL << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR10_Pos        (10U)                                        
#define EXTI_RTSR_TR10            (0x1UL << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR11_Pos        (11U)                                        
#define EXTI_RTSR_TR11            (0x1UL << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR12_Pos        (12U)                                        
#define EXTI_RTSR_TR12            (0x1UL << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR13_Pos        (13U)                                        
#define EXTI_RTSR_TR13            (0x1UL << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR14_Pos        (14U)                                        
#define EXTI_RTSR_TR14            (0x1UL << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR15_Pos        (15U)                                        
#define EXTI_RTSR_TR15            (0x1UL << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR16_Pos        (16U)                                        
#define EXTI_RTSR_TR16            (0x1UL << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR17_Pos        (17U)                                        
#define EXTI_RTSR_TR17            (0x1UL << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR18_Pos        (18U)                                        
#define EXTI_RTSR_TR18            (0x1UL << EXTI_RTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_RTSR_TR19_Pos        (19U)                                        
#define EXTI_RTSR_TR19            (0x1UL << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR20_Pos        (20U)                                        
#define EXTI_RTSR_TR20            (0x1UL << EXTI_RTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_RTSR_TR21_Pos        (21U)                                        
#define EXTI_RTSR_TR21            (0x1UL << EXTI_RTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_RTSR_TR22_Pos        (22U)                                        
#define EXTI_RTSR_TR22            (0x1UL << EXTI_RTSR_TR22_Pos)                 /*!< 0x00400000 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos         (0U)                                         
#define EXTI_FTSR_TR0             (0x1UL << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR1_Pos         (1U)                                         
#define EXTI_FTSR_TR1             (0x1UL << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR2_Pos         (2U)                                         
#define EXTI_FTSR_TR2             (0x1UL << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR3_Pos         (3U)                                         
#define EXTI_FTSR_TR3             (0x1UL << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR4_Pos         (4U)                                         
#define EXTI_FTSR_TR4             (0x1UL << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR5_Pos         (5U)                                         
#define EXTI_FTSR_TR5             (0x1UL << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR6_Pos         (6U)                                         
#define EXTI_FTSR_TR6             (0x1UL << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR7_Pos         (7U)                                         
#define EXTI_FTSR_TR7             (0x1UL << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR8_Pos         (8U)                                         
#define EXTI_FTSR_TR8             (0x1UL << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR9_Pos         (9U)                                         
#define EXTI_FTSR_TR9             (0x1UL << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR10_Pos        (10U)                                        
#define EXTI_FTSR_TR10            (0x1UL << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR11_Pos        (11U)                                        
#define EXTI_FTSR_TR11            (0x1UL << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR12_Pos        (12U)                                        
#define EXTI_FTSR_TR12            (0x1UL << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR13_Pos        (13U)                                        
#define EXTI_FTSR_TR13            (0x1UL << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR14_Pos        (14U)                                        
#define EXTI_FTSR_TR14            (0x1UL << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR15_Pos        (15U)                                        
#define EXTI_FTSR_TR15            (0x1UL << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR16_Pos        (16U)                                        
#define EXTI_FTSR_TR16            (0x1UL << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR17_Pos        (17U)                                        
#define EXTI_FTSR_TR17            (0x1UL << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR18_Pos        (18U)                                        
#define EXTI_FTSR_TR18            (0x1UL << EXTI_FTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_FTSR_TR19_Pos        (19U)                                        
#define EXTI_FTSR_TR19            (0x1UL << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR20_Pos        (20U)                                        
#define EXTI_FTSR_TR20            (0x1UL << EXTI_FTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_FTSR_TR21_Pos        (21U)                                        
#define EXTI_FTSR_TR21            (0x1UL << EXTI_FTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_FTSR_TR22_Pos        (22U)                                        
#define EXTI_FTSR_TR22            (0x1UL << EXTI_FTSR_TR22_Pos)                 /*!< 0x00400000 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)                                         
#define EXTI_SWIER_SWIER0         (0x1UL << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER1_Pos     (1U)                                         
#define EXTI_SWIER_SWIER1         (0x1UL << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER2_Pos     (2U)                                         
#define EXTI_SWIER_SWIER2         (0x1UL << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER3_Pos     (3U)                                         
#define EXTI_SWIER_SWIER3         (0x1UL << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER4_Pos     (4U)                                         
#define EXTI_SWIER_SWIER4         (0x1UL << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER5_Pos     (5U)                                         
#define EXTI_SWIER_SWIER5         (0x1UL << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER6_Pos     (6U)                                         
#define EXTI_SWIER_SWIER6         (0x1UL << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER7_Pos     (7U)                                         
#define EXTI_SWIER_SWIER7         (0x1UL << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER8_Pos     (8U)                                         
#define EXTI_SWIER_SWIER8         (0x1UL << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER9_Pos     (9U)                                         
#define EXTI_SWIER_SWIER9         (0x1UL << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER10_Pos    (10U)                                        
#define EXTI_SWIER_SWIER10        (0x1UL << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER11_Pos    (11U)                                        
#define EXTI_SWIER_SWIER11        (0x1UL << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER12_Pos    (12U)                                        
#define EXTI_SWIER_SWIER12        (0x1UL << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER13_Pos    (13U)                                        
#define EXTI_SWIER_SWIER13        (0x1UL << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER14_Pos    (14U)                                        
#define EXTI_SWIER_SWIER14        (0x1UL << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER15_Pos    (15U)                                        
#define EXTI_SWIER_SWIER15        (0x1UL << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER16_Pos    (16U)                                        
#define EXTI_SWIER_SWIER16        (0x1UL << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER17_Pos    (17U)                                        
#define EXTI_SWIER_SWIER17        (0x1UL << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER18_Pos    (18U)                                        
#define EXTI_SWIER_SWIER18        (0x1UL << EXTI_SWIER_SWIER18_Pos)             /*!< 0x00040000 */
#define EXTI_SWIER_SWIER19_Pos    (19U)                                        
#define EXTI_SWIER_SWIER19        (0x1UL << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER20_Pos    (20U)                                        
#define EXTI_SWIER_SWIER20        (0x1UL << EXTI_SWIER_SWIER20_Pos)             /*!< 0x00100000 */
#define EXTI_SWIER_SWIER21_Pos    (21U)                                        
#define EXTI_SWIER_SWIER21        (0x1UL << EXTI_SWIER_SWIER21_Pos)             /*!< 0x00200000 */
#define EXTI_SWIER_SWIER22_Pos    (22U)                                        
#define EXTI_SWIER_SWIER22        (0x1UL << EXTI_SWIER_SWIER22_Pos)             /*!< 0x00400000 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos           (0U)                                         
#define EXTI_PR_PR0               (0x1UL << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR1_Pos           (1U)                                         
#define EXTI_PR_PR1               (0x1UL << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR2_Pos           (2U)                                         
#define EXTI_PR_PR2               (0x1UL << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR3_Pos           (3U)                                         
#define EXTI_PR_PR3               (0x1UL << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR4_Pos           (4U)                                         
#define EXTI_PR_PR4               (0x1UL << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR5_Pos           (5U)                                         
#define EXTI_PR_PR5               (0x1UL << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR6_Pos           (6U)                                         
#define EXTI_PR_PR6               (0x1UL << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR7_Pos           (7U)                                         
#define EXTI_PR_PR7               (0x1UL << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR8_Pos           (8U)                                         
#define EXTI_PR_PR8               (0x1UL << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR9_Pos           (9U)                                         
#define EXTI_PR_PR9               (0x1UL << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR10_Pos          (10U)                                        
#define EXTI_PR_PR10              (0x1UL << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR11_Pos          (11U)                                        
#define EXTI_PR_PR11              (0x1UL << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR12_Pos          (12U)                                        
#define EXTI_PR_PR12              (0x1UL << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR13_Pos          (13U)                                        
#define EXTI_PR_PR13              (0x1UL << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR14_Pos          (14U)                                        
#define EXTI_PR_PR14              (0x1UL << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR15_Pos          (15U)                                        
#define EXTI_PR_PR15              (0x1UL << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR16_Pos          (16U)                                        
#define EXTI_PR_PR16              (0x1UL << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR17_Pos          (17U)                                        
#define EXTI_PR_PR17              (0x1UL << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR18_Pos          (18U)                                        
#define EXTI_PR_PR18              (0x1UL << EXTI_PR_PR18_Pos)                   /*!< 0x00040000 */
#define EXTI_PR_PR19_Pos          (19U)                                        
#define EXTI_PR_PR19              (0x1UL << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR20_Pos          (20U)                                        
#define EXTI_PR_PR20              (0x1UL << EXTI_PR_PR20_Pos)                   /*!< 0x00100000 */
#define EXTI_PR_PR21_Pos          (21U)                                        
#define EXTI_PR_PR21              (0x1UL << EXTI_PR_PR21_Pos)                   /*!< 0x00200000 */
#define EXTI_PR_PR22_Pos          (22U)                                        
#define EXTI_PR_PR22              (0x1UL << EXTI_PR_PR22_Pos)                   /*!< 0x00400000 */


/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos            (0U)                                  
#define GPIO_MODER_MODER0                (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0_0              (0x1UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1              (0x2UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos            (2U)                                  
#define GPIO_MODER_MODER1                (0x3UL << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */
#define GPIO_MODER_MODER1_0              (0x1UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1              (0x2UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos            (4U)                                  
#define GPIO_MODER_MODER2                (0x3UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */
#define GPIO_MODER_MODER2_0              (0x1UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1              (0x2UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos            (6U)                                  
#define GPIO_MODER_MODER3                (0x3UL << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */
#define GPIO_MODER_MODER3_0              (0x1UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1              (0x2UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos            (8U)                                  
#define GPIO_MODER_MODER4                (0x3UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */
#define GPIO_MODER_MODER4_0              (0x1UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1              (0x2UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos            (10U)                                 
#define GPIO_MODER_MODER5                (0x3UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */
#define GPIO_MODER_MODER5_0              (0x1UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1              (0x2UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos            (12U)                                 
#define GPIO_MODER_MODER6                (0x3UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */
#define GPIO_MODER_MODER6_0              (0x1UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1              (0x2UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos            (14U)                                 
#define GPIO_MODER_MODER7                (0x3UL << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */
#define GPIO_MODER_MODER7_0              (0x1UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1              (0x2UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos            (16U)                                 
#define GPIO_MODER_MODER8                (0x3UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8_0              (0x1UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1              (0x2UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos            (18U)                                 
#define GPIO_MODER_MODER9                (0x3UL << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9_0              (0x1UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1              (0x2UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos           (20U)                                 
#define GPIO_MODER_MODER10               (0x3UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */
#define GPIO_MODER_MODER10_0             (0x1UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1             (0x2UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos           (22U)                                 
#define GPIO_MODER_MODER11               (0x3UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */
#define GPIO_MODER_MODER11_0             (0x1UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1             (0x2UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos           (24U)                                 
#define GPIO_MODER_MODER12               (0x3UL << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */
#define GPIO_MODER_MODER12_0             (0x1UL << GPIO_MODER_MODER12_Pos)      /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1             (0x2UL << GPIO_MODER_MODER12_Pos)      /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos           (26U)                                 
#define GPIO_MODER_MODER13               (0x3UL << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */
#define GPIO_MODER_MODER13_0             (0x1UL << GPIO_MODER_MODER13_Pos)      /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1             (0x2UL << GPIO_MODER_MODER13_Pos)      /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos           (28U)                                 
#define GPIO_MODER_MODER14               (0x3UL << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */
#define GPIO_MODER_MODER14_0             (0x1UL << GPIO_MODER_MODER14_Pos)      /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1             (0x2UL << GPIO_MODER_MODER14_Pos)      /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos           (30U)                                 
#define GPIO_MODER_MODER15               (0x3UL << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */
#define GPIO_MODER_MODER15_0             (0x1UL << GPIO_MODER_MODER15_Pos)      /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1             (0x2UL << GPIO_MODER_MODER15_Pos)      /*!< 0x80000000 */



/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos              (0U)                                  
#define GPIO_OTYPER_OT0                  (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT1_Pos              (1U)                                  
#define GPIO_OTYPER_OT1                  (0x1UL << GPIO_OTYPER_OT1_Pos)         /*!< 0x00000002 */
#define GPIO_OTYPER_OT2_Pos              (2U)                                  
#define GPIO_OTYPER_OT2                  (0x1UL << GPIO_OTYPER_OT2_Pos)         /*!< 0x00000004 */
#define GPIO_OTYPER_OT3_Pos              (3U)                                  
#define GPIO_OTYPER_OT3                  (0x1UL << GPIO_OTYPER_OT3_Pos)         /*!< 0x00000008 */
#define GPIO_OTYPER_OT4_Pos              (4U)                                  
#define GPIO_OTYPER_OT4                  (0x1UL << GPIO_OTYPER_OT4_Pos)         /*!< 0x00000010 */
#define GPIO_OTYPER_OT5_Pos              (5U)                                  
#define GPIO_OTYPER_OT5                  (0x1UL << GPIO_OTYPER_OT5_Pos)         /*!< 0x00000020 */
#define GPIO_OTYPER_OT6_Pos              (6U)                                  
#define GPIO_OTYPER_OT6                  (0x1UL << GPIO_OTYPER_OT6_Pos)         /*!< 0x00000040 */
#define GPIO_OTYPER_OT7_Pos              (7U)                                  
#define GPIO_OTYPER_OT7                  (0x1UL << GPIO_OTYPER_OT7_Pos)         /*!< 0x00000080 */
#define GPIO_OTYPER_OT8_Pos              (8U)                                  
#define GPIO_OTYPER_OT8                  (0x1UL << GPIO_OTYPER_OT8_Pos)         /*!< 0x00000100 */
#define GPIO_OTYPER_OT9_Pos              (9U)                                  
#define GPIO_OTYPER_OT9                  (0x1UL << GPIO_OTYPER_OT9_Pos)         /*!< 0x00000200 */
#define GPIO_OTYPER_OT10_Pos             (10U)                                 
#define GPIO_OTYPER_OT10                 (0x1UL << GPIO_OTYPER_OT10_Pos)        /*!< 0x00000400 */
#define GPIO_OTYPER_OT11_Pos             (11U)                                 
#define GPIO_OTYPER_OT11                 (0x1UL << GPIO_OTYPER_OT11_Pos)        /*!< 0x00000800 */
#define GPIO_OTYPER_OT12_Pos             (12U)                                 
#define GPIO_OTYPER_OT12                 (0x1UL << GPIO_OTYPER_OT12_Pos)        /*!< 0x00001000 */
#define GPIO_OTYPER_OT13_Pos             (13U)                                 
#define GPIO_OTYPER_OT13                 (0x1UL << GPIO_OTYPER_OT13_Pos)        /*!< 0x00002000 */
#define GPIO_OTYPER_OT14_Pos             (14U)                                 
#define GPIO_OTYPER_OT14                 (0x1UL << GPIO_OTYPER_OT14_Pos)        /*!< 0x00004000 */
#define GPIO_OTYPER_OT15_Pos             (15U)                                 
#define GPIO_OTYPER_OT15                 (0x1UL << GPIO_OTYPER_OT15_Pos)        /*!< 0x00008000 */



/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)                                  
#define GPIO_OSPEEDR_OSPEED0             (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0_0           (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1           (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEED1_Pos         (2U)                                  
#define GPIO_OSPEEDR_OSPEED1             (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1_0           (0x1UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1           (0x2UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEED2_Pos         (4U)                                  
#define GPIO_OSPEEDR_OSPEED2             (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2_0           (0x1UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1           (0x2UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEED3_Pos         (6U)                                  
#define GPIO_OSPEEDR_OSPEED3             (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3_0           (0x1UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1           (0x2UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEED4_Pos         (8U)                                  
#define GPIO_OSPEEDR_OSPEED4             (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4_0           (0x1UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1           (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEED5_Pos         (10U)                                 
#define GPIO_OSPEEDR_OSPEED5             (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5_0           (0x1UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1           (0x2UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEED6_Pos         (12U)                                 
#define GPIO_OSPEEDR_OSPEED6             (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6_0           (0x1UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1           (0x2UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEED7_Pos         (14U)                                 
#define GPIO_OSPEEDR_OSPEED7             (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7_0           (0x1UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1           (0x2UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEED8_Pos         (16U)                                 
#define GPIO_OSPEEDR_OSPEED8             (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8_0           (0x1UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1           (0x2UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEED9_Pos         (18U)                                 
#define GPIO_OSPEEDR_OSPEED9             (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9_0           (0x1UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1           (0x2UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEED10_Pos        (20U)                                 
#define GPIO_OSPEEDR_OSPEED10            (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10_0          (0x1UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1          (0x2UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEED11_Pos        (22U)                                 
#define GPIO_OSPEEDR_OSPEED11            (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11_0          (0x1UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1          (0x2UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEED12_Pos        (24U)                                 
#define GPIO_OSPEEDR_OSPEED12            (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12_0          (0x1UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1          (0x2UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEED13_Pos        (26U)                                 
#define GPIO_OSPEEDR_OSPEED13            (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13_0          (0x1UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1          (0x2UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEED14_Pos        (28U)                                 
#define GPIO_OSPEEDR_OSPEED14            (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14_0          (0x1UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1          (0x2UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEED15_Pos        (30U)                                 
#define GPIO_OSPEEDR_OSPEED15            (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15_0          (0x1UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1          (0x2UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x80000000 */


/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos             (0U)                                  
#define GPIO_PUPDR_PUPD0                 (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0_0               (0x1UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1               (0x2UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPD1_Pos             (2U)                                  
#define GPIO_PUPDR_PUPD1                 (0x3UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1_0               (0x1UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1               (0x2UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPD2_Pos             (4U)                                  
#define GPIO_PUPDR_PUPD2                 (0x3UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2_0               (0x1UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1               (0x2UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPD3_Pos             (6U)                                  
#define GPIO_PUPDR_PUPD3                 (0x3UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3_0               (0x1UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1               (0x2UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPD4_Pos             (8U)                                  
#define GPIO_PUPDR_PUPD4                 (0x3UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4_0               (0x1UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1               (0x2UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPD5_Pos             (10U)                                 
#define GPIO_PUPDR_PUPD5                 (0x3UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5_0               (0x1UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1               (0x2UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPD6_Pos             (12U)                                 
#define GPIO_PUPDR_PUPD6                 (0x3UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6_0               (0x1UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1               (0x2UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPD7_Pos             (14U)                                 
#define GPIO_PUPDR_PUPD7                 (0x3UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7_0               (0x1UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1               (0x2UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPD8_Pos             (16U)                                 
#define GPIO_PUPDR_PUPD8                 (0x3UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8_0               (0x1UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1               (0x2UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPD9_Pos             (18U)                                 
#define GPIO_PUPDR_PUPD9                 (0x3UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9_0               (0x1UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1               (0x2UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPD10_Pos            (20U)                                 
#define GPIO_PUPDR_PUPD10                (0x3UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10_0              (0x1UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1              (0x2UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPD11_Pos            (22U)                                 
#define GPIO_PUPDR_PUPD11                (0x3UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11_0              (0x1UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1              (0x2UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPD12_Pos            (24U)                                 
#define GPIO_PUPDR_PUPD12                (0x3UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12_0              (0x1UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1              (0x2UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPD13_Pos            (26U)                                 
#define GPIO_PUPDR_PUPD13                (0x3UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13_0              (0x1UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1              (0x2UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPD14_Pos            (28U)                                 
#define GPIO_PUPDR_PUPD14                (0x3UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14_0              (0x1UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1              (0x2UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPD15_Pos            (30U)                                 
#define GPIO_PUPDR_PUPD15                (0x3UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15_0              (0x1UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1              (0x2UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x80000000 */



/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)                                  
#define GPIO_IDR_ID0                     (0x1UL << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */
#define GPIO_IDR_ID1_Pos                 (1U)                                  
#define GPIO_IDR_ID1                     (0x1UL << GPIO_IDR_ID1_Pos)            /*!< 0x00000002 */
#define GPIO_IDR_ID2_Pos                 (2U)                                  
#define GPIO_IDR_ID2                     (0x1UL << GPIO_IDR_ID2_Pos)            /*!< 0x00000004 */
#define GPIO_IDR_ID3_Pos                 (3U)                                  
#define GPIO_IDR_ID3                     (0x1UL << GPIO_IDR_ID3_Pos)            /*!< 0x00000008 */
#define GPIO_IDR_ID4_Pos                 (4U)                                  
#define GPIO_IDR_ID4                     (0x1UL << GPIO_IDR_ID4_Pos)            /*!< 0x00000010 */
#define GPIO_IDR_ID5_Pos                 (5U)                                  
#define GPIO_IDR_ID5                     (0x1UL << GPIO_IDR_ID5_Pos)            /*!< 0x00000020 */
#define GPIO_IDR_ID6_Pos                 (6U)                                  
#define GPIO_IDR_ID6                     (0x1UL << GPIO_IDR_ID6_Pos)            /*!< 0x00000040 */
#define GPIO_IDR_ID7_Pos                 (7U)                                  
#define GPIO_IDR_ID7                     (0x1UL << GPIO_IDR_ID7_Pos)            /*!< 0x00000080 */
#define GPIO_IDR_ID8_Pos                 (8U)                                  
#define GPIO_IDR_ID8                     (0x1UL << GPIO_IDR_ID8_Pos)            /*!< 0x00000100 */
#define GPIO_IDR_ID9_Pos                 (9U)                                  
#define GPIO_IDR_ID9                     (0x1UL << GPIO_IDR_ID9_Pos)            /*!< 0x00000200 */
#define GPIO_IDR_ID10_Pos                (10U)                                 
#define GPIO_IDR_ID10                    (0x1UL << GPIO_IDR_ID10_Pos)           /*!< 0x00000400 */
#define GPIO_IDR_ID11_Pos                (11U)                                 
#define GPIO_IDR_ID11                    (0x1UL << GPIO_IDR_ID11_Pos)           /*!< 0x00000800 */
#define GPIO_IDR_ID12_Pos                (12U)                                 
#define GPIO_IDR_ID12                    (0x1UL << GPIO_IDR_ID12_Pos)           /*!< 0x00001000 */
#define GPIO_IDR_ID13_Pos                (13U)                                 
#define GPIO_IDR_ID13                    (0x1UL << GPIO_IDR_ID13_Pos)           /*!< 0x00002000 */
#define GPIO_IDR_ID14_Pos                (14U)                                 
#define GPIO_IDR_ID14                    (0x1UL << GPIO_IDR_ID14_Pos)           /*!< 0x00004000 */
#define GPIO_IDR_ID15_Pos                (15U)                                 
#define GPIO_IDR_ID15                    (0x1UL << GPIO_IDR_ID15_Pos)           /*!< 0x00008000 */


/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos                 (0U)                                  
#define GPIO_ODR_OD0                     (0x1UL << GPIO_ODR_OD0_Pos)            /*!< 0x00000001 */
#define GPIO_ODR_OD1_Pos                 (1U)                                  
#define GPIO_ODR_OD1                     (0x1UL << GPIO_ODR_OD1_Pos)            /*!< 0x00000002 */
#define GPIO_ODR_OD2_Pos                 (2U)                                  
#define GPIO_ODR_OD2                     (0x1UL << GPIO_ODR_OD2_Pos)            /*!< 0x00000004 */
#define GPIO_ODR_OD3_Pos                 (3U)                                  
#define GPIO_ODR_OD3                     (0x1UL << GPIO_ODR_OD3_Pos)            /*!< 0x00000008 */
#define GPIO_ODR_OD4_Pos                 (4U)                                  
#define GPIO_ODR_OD4                     (0x1UL << GPIO_ODR_OD4_Pos)            /*!< 0x00000010 */
#define GPIO_ODR_OD5_Pos                 (5U)                                  
#define GPIO_ODR_OD5                     (0x1UL << GPIO_ODR_OD5_Pos)            /*!< 0x00000020 */
#define GPIO_ODR_OD6_Pos                 (6U)                                  
#define GPIO_ODR_OD6                     (0x1UL << GPIO_ODR_OD6_Pos)            /*!< 0x00000040 */
#define GPIO_ODR_OD7_Pos                 (7U)                                  
#define GPIO_ODR_OD7                     (0x1UL << GPIO_ODR_OD7_Pos)            /*!< 0x00000080 */
#define GPIO_ODR_OD8_Pos                 (8U)                                  
#define GPIO_ODR_OD8                     (0x1UL << GPIO_ODR_OD8_Pos)            /*!< 0x00000100 */
#define GPIO_ODR_OD9_Pos                 (9U)                                  
#define GPIO_ODR_OD9                     (0x1UL << GPIO_ODR_OD9_Pos)            /*!< 0x00000200 */
#define GPIO_ODR_OD10_Pos                (10U)                                 
#define GPIO_ODR_OD10                    (0x1UL << GPIO_ODR_OD10_Pos)           /*!< 0x00000400 */
#define GPIO_ODR_OD11_Pos                (11U)                                 
#define GPIO_ODR_OD11                    (0x1UL << GPIO_ODR_OD11_Pos)           /*!< 0x00000800 */
#define GPIO_ODR_OD12_Pos                (12U)                                 
#define GPIO_ODR_OD12                    (0x1UL << GPIO_ODR_OD12_Pos)           /*!< 0x00001000 */
#define GPIO_ODR_OD13_Pos                (13U)                                 
#define GPIO_ODR_OD13                    (0x1UL << GPIO_ODR_OD13_Pos)           /*!< 0x00002000 */
#define GPIO_ODR_OD14_Pos                (14U)                                 
#define GPIO_ODR_OD14                    (0x1UL << GPIO_ODR_OD14_Pos)           /*!< 0x00004000 */
#define GPIO_ODR_OD15_Pos                (15U)                                 
#define GPIO_ODR_OD15                    (0x1UL << GPIO_ODR_OD15_Pos)           /*!< 0x00008000 */


/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos                (0U)                                  
#define GPIO_BSRR_BS0                    (0x1UL << GPIO_BSRR_BS0_Pos)           /*!< 0x00000001 */
#define GPIO_BSRR_BS1_Pos                (1U)                                  
#define GPIO_BSRR_BS1                    (0x1UL << GPIO_BSRR_BS1_Pos)           /*!< 0x00000002 */
#define GPIO_BSRR_BS2_Pos                (2U)                                  
#define GPIO_BSRR_BS2                    (0x1UL << GPIO_BSRR_BS2_Pos)           /*!< 0x00000004 */
#define GPIO_BSRR_BS3_Pos                (3U)                                  
#define GPIO_BSRR_BS3                    (0x1UL << GPIO_BSRR_BS3_Pos)           /*!< 0x00000008 */
#define GPIO_BSRR_BS4_Pos                (4U)                                  
#define GPIO_BSRR_BS4                    (0x1UL << GPIO_BSRR_BS4_Pos)           /*!< 0x00000010 */
#define GPIO_BSRR_BS5_Pos                (5U)                                  
#define GPIO_BSRR_BS5                    (0x1UL << GPIO_BSRR_BS5_Pos)           /*!< 0x00000020 */
#define GPIO_BSRR_BS6_Pos                (6U)                                  
#define GPIO_BSRR_BS6                    (0x1UL << GPIO_BSRR_BS6_Pos)           /*!< 0x00000040 */
#define GPIO_BSRR_BS7_Pos                (7U)                                  
#define GPIO_BSRR_BS7                    (0x1UL << GPIO_BSRR_BS7_Pos)           /*!< 0x00000080 */
#define GPIO_BSRR_BS8_Pos                (8U)                                  
#define GPIO_BSRR_BS8                    (0x1UL << GPIO_BSRR_BS8_Pos)           /*!< 0x00000100 */
#define GPIO_BSRR_BS9_Pos                (9U)                                  
#define GPIO_BSRR_BS9                    (0x1UL << GPIO_BSRR_BS9_Pos)           /*!< 0x00000200 */
#define GPIO_BSRR_BS10_Pos               (10U)                                 
#define GPIO_BSRR_BS10                   (0x1UL << GPIO_BSRR_BS10_Pos)          /*!< 0x00000400 */
#define GPIO_BSRR_BS11_Pos               (11U)                                 
#define GPIO_BSRR_BS11                   (0x1UL << GPIO_BSRR_BS11_Pos)          /*!< 0x00000800 */
#define GPIO_BSRR_BS12_Pos               (12U)                                 
#define GPIO_BSRR_BS12                   (0x1UL << GPIO_BSRR_BS12_Pos)          /*!< 0x00001000 */
#define GPIO_BSRR_BS13_Pos               (13U)                                 
#define GPIO_BSRR_BS13                   (0x1UL << GPIO_BSRR_BS13_Pos)          /*!< 0x00002000 */
#define GPIO_BSRR_BS14_Pos               (14U)                                 
#define GPIO_BSRR_BS14                   (0x1UL << GPIO_BSRR_BS14_Pos)          /*!< 0x00004000 */
#define GPIO_BSRR_BS15_Pos               (15U)                                 
#define GPIO_BSRR_BS15                   (0x1UL << GPIO_BSRR_BS15_Pos)          /*!< 0x00008000 */
#define GPIO_BSRR_BR0_Pos                (16U)                                 
#define GPIO_BSRR_BR0                    (0x1UL << GPIO_BSRR_BR0_Pos)           /*!< 0x00010000 */
#define GPIO_BSRR_BR1_Pos                (17U)                                 
#define GPIO_BSRR_BR1                    (0x1UL << GPIO_BSRR_BR1_Pos)           /*!< 0x00020000 */
#define GPIO_BSRR_BR2_Pos                (18U)                                 
#define GPIO_BSRR_BR2                    (0x1UL << GPIO_BSRR_BR2_Pos)           /*!< 0x00040000 */
#define GPIO_BSRR_BR3_Pos                (19U)                                 
#define GPIO_BSRR_BR3                    (0x1UL << GPIO_BSRR_BR3_Pos)           /*!< 0x00080000 */
#define GPIO_BSRR_BR4_Pos                (20U)                                 
#define GPIO_BSRR_BR4                    (0x1UL << GPIO_BSRR_BR4_Pos)           /*!< 0x00100000 */
#define GPIO_BSRR_BR5_Pos                (21U)                                 
#define GPIO_BSRR_BR5                    (0x1UL << GPIO_BSRR_BR5_Pos)           /*!< 0x00200000 */
#define GPIO_BSRR_BR6_Pos                (22U)                                 
#define GPIO_BSRR_BR6                    (0x1UL << GPIO_BSRR_BR6_Pos)           /*!< 0x00400000 */
#define GPIO_BSRR_BR7_Pos                (23U)                                 
#define GPIO_BSRR_BR7                    (0x1UL << GPIO_BSRR_BR7_Pos)           /*!< 0x00800000 */
#define GPIO_BSRR_BR8_Pos                (24U)                                 
#define GPIO_BSRR_BR8                    (0x1UL << GPIO_BSRR_BR8_Pos)           /*!< 0x01000000 */
#define GPIO_BSRR_BR9_Pos                (25U)                                 
#define GPIO_BSRR_BR9                    (0x1UL << GPIO_BSRR_BR9_Pos)           /*!< 0x02000000 */
#define GPIO_BSRR_BR10_Pos               (26U)                                 
#define GPIO_BSRR_BR10                   (0x1UL << GPIO_BSRR_BR10_Pos)          /*!< 0x04000000 */
#define GPIO_BSRR_BR11_Pos               (27U)                                 
#define GPIO_BSRR_BR11                   (0x1UL << GPIO_BSRR_BR11_Pos)          /*!< 0x08000000 */
#define GPIO_BSRR_BR12_Pos               (28U)                                 
#define GPIO_BSRR_BR12                   (0x1UL << GPIO_BSRR_BR12_Pos)          /*!< 0x10000000 */
#define GPIO_BSRR_BR13_Pos               (29U)                                 
#define GPIO_BSRR_BR13                   (0x1UL << GPIO_BSRR_BR13_Pos)          /*!< 0x20000000 */
#define GPIO_BSRR_BR14_Pos               (30U)                                 
#define GPIO_BSRR_BR14                   (0x1UL << GPIO_BSRR_BR14_Pos)          /*!< 0x40000000 */
#define GPIO_BSRR_BR15_Pos               (31U)                                 
#define GPIO_BSRR_BR15                   (0x1UL << GPIO_BSRR_BR15_Pos)          /*!< 0x80000000 */


/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)                                  
#define GPIO_LCKR_LCK0                   (0x1UL << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */
#define GPIO_LCKR_LCK1_Pos               (1U)                                  
#define GPIO_LCKR_LCK1                   (0x1UL << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */
#define GPIO_LCKR_LCK2_Pos               (2U)                                  
#define GPIO_LCKR_LCK2                   (0x1UL << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */
#define GPIO_LCKR_LCK3_Pos               (3U)                                  
#define GPIO_LCKR_LCK3                   (0x1UL << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */
#define GPIO_LCKR_LCK4_Pos               (4U)                                  
#define GPIO_LCKR_LCK4                   (0x1UL << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */
#define GPIO_LCKR_LCK5_Pos               (5U)                                  
#define GPIO_LCKR_LCK5                   (0x1UL << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */
#define GPIO_LCKR_LCK6_Pos               (6U)                                  
#define GPIO_LCKR_LCK6                   (0x1UL << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */
#define GPIO_LCKR_LCK7_Pos               (7U)                                  
#define GPIO_LCKR_LCK7                   (0x1UL << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */
#define GPIO_LCKR_LCK8_Pos               (8U)                                  
#define GPIO_LCKR_LCK8                   (0x1UL << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */
#define GPIO_LCKR_LCK9_Pos               (9U)                                  
#define GPIO_LCKR_LCK9                   (0x1UL << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */
#define GPIO_LCKR_LCK10_Pos              (10U)                                 
#define GPIO_LCKR_LCK10                  (0x1UL << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */
#define GPIO_LCKR_LCK11_Pos              (11U)                                 
#define GPIO_LCKR_LCK11                  (0x1UL << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */
#define GPIO_LCKR_LCK12_Pos              (12U)                                 
#define GPIO_LCKR_LCK12                  (0x1UL << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */
#define GPIO_LCKR_LCK13_Pos              (13U)                                 
#define GPIO_LCKR_LCK13                  (0x1UL << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */
#define GPIO_LCKR_LCK14_Pos              (14U)                                 
#define GPIO_LCKR_LCK14                  (0x1UL << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */
#define GPIO_LCKR_LCK15_Pos              (15U)                                 
#define GPIO_LCKR_LCK15                  (0x1UL << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */
#define GPIO_LCKR_LCKK_Pos               (16U)                                 
#define GPIO_LCKR_LCKK                   (0x1UL << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */
/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos             (0U)                                  
#define GPIO_AFRL_AFSEL0                 (0xFUL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0_0               (0x1UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1               (0x2UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2               (0x4UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000004 */
#define GPIO_AFRL_AFSEL0_3               (0x8UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000008 */
#define GPIO_AFRL_AFSEL1_Pos             (4U)                                  
#define GPIO_AFRL_AFSEL1                 (0xFUL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1_0               (0x1UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1               (0x2UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2               (0x4UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000040 */
#define GPIO_AFRL_AFSEL1_3               (0x8UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000080 */
#define GPIO_AFRL_AFSEL2_Pos             (8U)                                  
#define GPIO_AFRL_AFSEL2                 (0xFUL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2_0               (0x1UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1               (0x2UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2               (0x4UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000400 */
#define GPIO_AFRL_AFSEL2_3               (0x8UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000800 */
#define GPIO_AFRL_AFSEL3_Pos             (12U)                                 
#define GPIO_AFRL_AFSEL3                 (0xFUL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3_0               (0x1UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1               (0x2UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2               (0x4UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00004000 */
#define GPIO_AFRL_AFSEL3_3               (0x8UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00008000 */
#define GPIO_AFRL_AFSEL4_Pos             (16U)                                 
#define GPIO_AFRL_AFSEL4                 (0xFUL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4_0               (0x1UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1               (0x2UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2               (0x4UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00040000 */
#define GPIO_AFRL_AFSEL4_3               (0x8UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00080000 */
#define GPIO_AFRL_AFSEL5_Pos             (20U)                                 
#define GPIO_AFRL_AFSEL5                 (0xFUL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5_0               (0x1UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1               (0x2UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2               (0x4UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00400000 */
#define GPIO_AFRL_AFSEL5_3               (0x8UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00800000 */
#define GPIO_AFRL_AFSEL6_Pos             (24U)                                 
#define GPIO_AFRL_AFSEL6                 (0xFUL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6_0               (0x1UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1               (0x2UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2               (0x4UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x04000000 */
#define GPIO_AFRL_AFSEL6_3               (0x8UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x08000000 */
#define GPIO_AFRL_AFSEL7_Pos             (28U)                                 
#define GPIO_AFRL_AFSEL7                 (0xFUL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7_0               (0x1UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1               (0x2UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2               (0x4UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x40000000 */
#define GPIO_AFRL_AFSEL7_3               (0x8UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x80000000 */



/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos             (0U)                                  
#define GPIO_AFRH_AFSEL8                 (0xFUL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8_0               (0x1UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1               (0x2UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2               (0x4UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000004 */
#define GPIO_AFRH_AFSEL8_3               (0x8UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000008 */
#define GPIO_AFRH_AFSEL9_Pos             (4U)                                  
#define GPIO_AFRH_AFSEL9                 (0xFUL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9_0               (0x1UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1               (0x2UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2               (0x4UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000040 */
#define GPIO_AFRH_AFSEL9_3               (0x8UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000080 */
#define GPIO_AFRH_AFSEL10_Pos            (8U)                                  
#define GPIO_AFRH_AFSEL10                (0xFUL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10_0              (0x1UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1              (0x2UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2              (0x4UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000400 */
#define GPIO_AFRH_AFSEL10_3              (0x8UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000800 */
#define GPIO_AFRH_AFSEL11_Pos            (12U)                                 
#define GPIO_AFRH_AFSEL11                (0xFUL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11_0              (0x1UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1              (0x2UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2              (0x4UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00004000 */
#define GPIO_AFRH_AFSEL11_3              (0x8UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00008000 */
#define GPIO_AFRH_AFSEL12_Pos            (16U)                                 
#define GPIO_AFRH_AFSEL12                (0xFUL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12_0              (0x1UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1              (0x2UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2              (0x4UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00040000 */
#define GPIO_AFRH_AFSEL12_3              (0x8UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00080000 */
#define GPIO_AFRH_AFSEL13_Pos            (20U)                                 
#define GPIO_AFRH_AFSEL13                (0xFUL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13_0              (0x1UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1              (0x2UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2              (0x4UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00400000 */
#define GPIO_AFRH_AFSEL13_3              (0x8UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00800000 */
#define GPIO_AFRH_AFSEL14_Pos            (24U)                                 
#define GPIO_AFRH_AFSEL14                (0xFUL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14_0              (0x1UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1              (0x2UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2              (0x4UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x04000000 */
#define GPIO_AFRH_AFSEL14_3              (0x8UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x08000000 */
#define GPIO_AFRH_AFSEL15_Pos            (28U)                                 
#define GPIO_AFRH_AFSEL15                (0xFUL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15_0              (0x1UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1              (0x2UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2              (0x4UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x40000000 */
#define GPIO_AFRH_AFSEL15_3              (0x8UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x80000000 */




/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos            (0U)                                         
#define I2C_CR1_PE                (0x1UL << I2C_CR1_PE_Pos)                     /*!< 0x00000001 */
#define I2C_CR1_SMBUS_Pos         (1U)                                         
#define I2C_CR1_SMBUS             (0x1UL << I2C_CR1_SMBUS_Pos)                  /*!< 0x00000002 */
#define I2C_CR1_SMBTYPE_Pos       (3U)                                         
#define I2C_CR1_SMBTYPE           (0x1UL << I2C_CR1_SMBTYPE_Pos)                /*!< 0x00000008 */
#define I2C_CR1_ENARP_Pos         (4U)                                         
#define I2C_CR1_ENARP             (0x1UL << I2C_CR1_ENARP_Pos)                  /*!< 0x00000010 */
#define I2C_CR1_ENPEC_Pos         (5U)                                         
#define I2C_CR1_ENPEC             (0x1UL << I2C_CR1_ENPEC_Pos)                  /*!< 0x00000020 */
#define I2C_CR1_ENGC_Pos          (6U)                                         
#define I2C_CR1_ENGC              (0x1UL << I2C_CR1_ENGC_Pos)                   /*!< 0x00000040 */
#define I2C_CR1_NOSTRETCH_Pos     (7U)                                         
#define I2C_CR1_NOSTRETCH         (0x1UL << I2C_CR1_NOSTRETCH_Pos)              /*!< 0x00000080 */
#define I2C_CR1_START_Pos         (8U)                                         
#define I2C_CR1_START             (0x1UL << I2C_CR1_START_Pos)                  /*!< 0x00000100 */
#define I2C_CR1_STOP_Pos          (9U)                                         
#define I2C_CR1_STOP              (0x1UL << I2C_CR1_STOP_Pos)                   /*!< 0x00000200 */
#define I2C_CR1_ACK_Pos           (10U)                                        
#define I2C_CR1_ACK               (0x1UL << I2C_CR1_ACK_Pos)                    /*!< 0x00000400 */
#define I2C_CR1_POS_Pos           (11U)                                        
#define I2C_CR1_POS               (0x1UL << I2C_CR1_POS_Pos)                    /*!< 0x00000800 */
#define I2C_CR1_PEC_Pos           (12U)                                        
#define I2C_CR1_PEC               (0x1UL << I2C_CR1_PEC_Pos)                    /*!< 0x00001000 */
#define I2C_CR1_ALERT_Pos         (13U)                                        
#define I2C_CR1_ALERT             (0x1UL << I2C_CR1_ALERT_Pos)                  /*!< 0x00002000 */
#define I2C_CR1_SWRST_Pos         (15U)                                        
#define I2C_CR1_SWRST             (0x1UL << I2C_CR1_SWRST_Pos)                  /*!< 0x00008000 */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos          (0U)                                         
#define I2C_CR2_FREQ              (0x3FUL << I2C_CR2_FREQ_Pos)                  /*!< 0x0000003F */
#define I2C_CR2_FREQ_0            (0x01UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000001 */
#define I2C_CR2_FREQ_1            (0x02UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000002 */
#define I2C_CR2_FREQ_2            (0x04UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000004 */
#define I2C_CR2_FREQ_3            (0x08UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000008 */
#define I2C_CR2_FREQ_4            (0x10UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000010 */
#define I2C_CR2_FREQ_5            (0x20UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000020 */

#define I2C_CR2_ITERREN_Pos       (8U)                                         
#define I2C_CR2_ITERREN           (0x1UL << I2C_CR2_ITERREN_Pos)                /*!< 0x00000100 */
#define I2C_CR2_ITEVTEN_Pos       (9U)                                         
#define I2C_CR2_ITEVTEN           (0x1UL << I2C_CR2_ITEVTEN_Pos)                /*!< 0x00000200 */
#define I2C_CR2_ITBUFEN_Pos       (10U)                                        
#define I2C_CR2_ITBUFEN           (0x1UL << I2C_CR2_ITBUFEN_Pos)                /*!< 0x00000400 */
#define I2C_CR2_DMAEN_Pos         (11U)                                        
#define I2C_CR2_DMAEN             (0x1UL << I2C_CR2_DMAEN_Pos)                  /*!< 0x00000800 */
#define I2C_CR2_LAST_Pos          (12U)                                        
#define I2C_CR2_LAST              (0x1UL << I2C_CR2_LAST_Pos)                   /*!< 0x00001000 */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7           0x000000FEU                                  /*!<Interface Address */
#define I2C_OAR1_ADD8_9           0x00000300U                                  /*!<Interface Address */

#define I2C_OAR1_ADD0_Pos         (0U)                                         
#define I2C_OAR1_ADD0             (0x1UL << I2C_OAR1_ADD0_Pos)                  /*!< 0x00000001 */
#define I2C_OAR1_ADD1_Pos         (1U)                                         
#define I2C_OAR1_ADD1             (0x1UL << I2C_OAR1_ADD1_Pos)                  /*!< 0x00000002 */
#define I2C_OAR1_ADD2_Pos         (2U)                                         
#define I2C_OAR1_ADD2             (0x1UL << I2C_OAR1_ADD2_Pos)                  /*!< 0x00000004 */
#define I2C_OAR1_ADD3_Pos         (3U)                                         
#define I2C_OAR1_ADD3             (0x1UL << I2C_OAR1_ADD3_Pos)                  /*!< 0x00000008 */
#define I2C_OAR1_ADD4_Pos         (4U)                                         
#define I2C_OAR1_ADD4             (0x1UL << I2C_OAR1_ADD4_Pos)                  /*!< 0x00000010 */
#define I2C_OAR1_ADD5_Pos         (5U)                                         
#define I2C_OAR1_ADD5             (0x1UL << I2C_OAR1_ADD5_Pos)                  /*!< 0x00000020 */
#define I2C_OAR1_ADD6_Pos         (6U)                                         
#define I2C_OAR1_ADD6             (0x1UL << I2C_OAR1_ADD6_Pos)                  /*!< 0x00000040 */
#define I2C_OAR1_ADD7_Pos         (7U)                                         
#define I2C_OAR1_ADD7             (0x1UL << I2C_OAR1_ADD7_Pos)                  /*!< 0x00000080 */
#define I2C_OAR1_ADD8_Pos         (8U)                                         
#define I2C_OAR1_ADD8             (0x1UL << I2C_OAR1_ADD8_Pos)                  /*!< 0x00000100 */
#define I2C_OAR1_ADD9_Pos         (9U)                                         
#define I2C_OAR1_ADD9             (0x1UL << I2C_OAR1_ADD9_Pos)                  /*!< 0x00000200 */

#define I2C_OAR1_ADDMODE_Pos      (15U)                                        
#define I2C_OAR1_ADDMODE          (0x1UL << I2C_OAR1_ADDMODE_Pos)               /*!< 0x00008000 */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL_Pos       (0U)                                         
#define I2C_OAR2_ENDUAL           (0x1UL << I2C_OAR2_ENDUAL_Pos)                /*!< 0x00000001 */
#define I2C_OAR2_ADD2_Pos         (1U)                                         
#define I2C_OAR2_ADD2             (0x7FUL << I2C_OAR2_ADD2_Pos)                 /*!< 0x000000FE */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR_Pos             (0U)                                         
#define I2C_DR_DR                 (0xFFUL << I2C_DR_DR_Pos)                     /*!< 0x000000FF */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos            (0U)                                         
#define I2C_SR1_SB                (0x1UL << I2C_SR1_SB_Pos)                     /*!< 0x00000001 */
#define I2C_SR1_ADDR_Pos          (1U)                                         
#define I2C_SR1_ADDR              (0x1UL << I2C_SR1_ADDR_Pos)                   /*!< 0x00000002 */
#define I2C_SR1_BTF_Pos           (2U)                                         
#define I2C_SR1_BTF               (0x1UL << I2C_SR1_BTF_Pos)                    /*!< 0x00000004 */
#define I2C_SR1_ADD10_Pos         (3U)                                         
#define I2C_SR1_ADD10             (0x1UL << I2C_SR1_ADD10_Pos)                  /*!< 0x00000008 */
#define I2C_SR1_STOPF_Pos         (4U)                                         
#define I2C_SR1_STOPF             (0x1UL << I2C_SR1_STOPF_Pos)                  /*!< 0x00000010 */
#define I2C_SR1_RXNE_Pos          (6U)                                         
#define I2C_SR1_RXNE              (0x1UL << I2C_SR1_RXNE_Pos)                   /*!< 0x00000040 */
#define I2C_SR1_TXE_Pos           (7U)                                         
#define I2C_SR1_TXE               (0x1UL << I2C_SR1_TXE_Pos)                    /*!< 0x00000080 */
#define I2C_SR1_BERR_Pos          (8U)                                         
#define I2C_SR1_BERR              (0x1UL << I2C_SR1_BERR_Pos)                   /*!< 0x00000100 */
#define I2C_SR1_ARLO_Pos          (9U)                                         
#define I2C_SR1_ARLO              (0x1UL << I2C_SR1_ARLO_Pos)                   /*!< 0x00000200 */
#define I2C_SR1_AF_Pos            (10U)                                        
#define I2C_SR1_AF                (0x1UL << I2C_SR1_AF_Pos)                     /*!< 0x00000400 */
#define I2C_SR1_OVR_Pos           (11U)                                        
#define I2C_SR1_OVR               (0x1UL << I2C_SR1_OVR_Pos)                    /*!< 0x00000800 */
#define I2C_SR1_PECERR_Pos        (12U)                                        
#define I2C_SR1_PECERR            (0x1UL << I2C_SR1_PECERR_Pos)                 /*!< 0x00001000 */
#define I2C_SR1_TIMEOUT_Pos       (14U)                                        
#define I2C_SR1_TIMEOUT           (0x1UL << I2C_SR1_TIMEOUT_Pos)                /*!< 0x00004000 */
#define I2C_SR1_SMBALERT_Pos      (15U)                                        
#define I2C_SR1_SMBALERT          (0x1UL << I2C_SR1_SMBALERT_Pos)               /*!< 0x00008000 */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos           (0U)                                         
#define I2C_SR2_MSL               (0x1UL << I2C_SR2_MSL_Pos)                    /*!< 0x00000001 */
#define I2C_SR2_BUSY_Pos          (1U)                                         
#define I2C_SR2_BUSY              (0x1UL << I2C_SR2_BUSY_Pos)                   /*!< 0x00000002 */
#define I2C_SR2_TRA_Pos           (2U)                                         
#define I2C_SR2_TRA               (0x1UL << I2C_SR2_TRA_Pos)                    /*!< 0x00000004 */
#define I2C_SR2_GENCALL_Pos       (4U)                                         
#define I2C_SR2_GENCALL           (0x1UL << I2C_SR2_GENCALL_Pos)                /*!< 0x00000010 */
#define I2C_SR2_SMBDEFAULT_Pos    (5U)                                         
#define I2C_SR2_SMBDEFAULT        (0x1UL << I2C_SR2_SMBDEFAULT_Pos)             /*!< 0x00000020 */
#define I2C_SR2_SMBHOST_Pos       (6U)                                         
#define I2C_SR2_SMBHOST           (0x1UL << I2C_SR2_SMBHOST_Pos)                /*!< 0x00000040 */
#define I2C_SR2_DUALF_Pos         (7U)                                         
#define I2C_SR2_DUALF             (0x1UL << I2C_SR2_DUALF_Pos)                  /*!< 0x00000080 */
#define I2C_SR2_PEC_Pos           (8U)                                         
#define I2C_SR2_PEC               (0xFFUL << I2C_SR2_PEC_Pos)                   /*!< 0x0000FF00 */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR_Pos           (0U)                                         
#define I2C_CCR_CCR               (0xFFFUL << I2C_CCR_CCR_Pos)                  /*!< 0x00000FFF */
#define I2C_CCR_DUTY_Pos          (14U)                                        
#define I2C_CCR_DUTY              (0x1UL << I2C_CCR_DUTY_Pos)                   /*!< 0x00004000 */
#define I2C_CCR_FS_Pos            (15U)                                        
#define I2C_CCR_FS                (0x1UL << I2C_CCR_FS_Pos)                     /*!< 0x00008000 */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE_Pos       (0U)                                         
#define I2C_TRISE_TRISE           (0x3FUL << I2C_TRISE_TRISE_Pos)               /*!< 0x0000003F */

/******************  Bit definition for I2C_FLTR register  *******************/
#define I2C_FLTR_DNF_Pos          (0U)                                         
#define I2C_FLTR_DNF              (0xFUL << I2C_FLTR_DNF_Pos)                   /*!< 0x0000000F */
#define I2C_FLTR_ANOFF_Pos        (4U)                                         
#define I2C_FLTR_ANOFF            (0x1UL << I2C_FLTR_ANOFF_Pos)                 /*!< 0x00000010 */


/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                   (0U)                                
#define RCC_CR_HSION                       (0x1UL << RCC_CR_HSION_Pos)          /*!< 0x00000001 */
#define RCC_CR_HSIRDY_Pos                  (1U)                                
#define RCC_CR_HSIRDY                      (0x1UL << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */

#define RCC_CR_HSITRIM_Pos                 (3U)                                
#define RCC_CR_HSITRIM                     (0x1FUL << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */
#define RCC_CR_HSITRIM_0                   (0x01UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                   (0x02UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                   (0x04UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                   (0x08UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                   (0x10UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                  (8U)                                
#define RCC_CR_HSICAL                      (0xFFUL << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */
#define RCC_CR_HSICAL_0                    (0x01UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                    (0x02UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                    (0x04UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                    (0x08UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                    (0x10UL << RCC_CR_HSICAL_Pos)        /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                    (0x20UL << RCC_CR_HSICAL_Pos)        /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                    (0x40UL << RCC_CR_HSICAL_Pos)        /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                    (0x80UL << RCC_CR_HSICAL_Pos)        /*!< 0x00008000 */

#define RCC_CR_HSEON_Pos                   (16U)                               
#define RCC_CR_HSEON                       (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSERDY_Pos                  (17U)                               
#define RCC_CR_HSERDY                      (0x1UL << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSEBYP_Pos                  (18U)                               
#define RCC_CR_HSEBYP                      (0x1UL << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */
#define RCC_CR_CSSON_Pos                   (19U)                               
#define RCC_CR_CSSON                       (0x1UL << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */
#define RCC_CR_PLLON_Pos                   (24U)                               
#define RCC_CR_PLLON                       (0x1UL << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLRDY_Pos                  (25U)                               
#define RCC_CR_PLLRDY                      (0x1UL << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos                (26U)                               
#define RCC_CR_PLLI2SON                    (0x1UL << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */
#define RCC_CR_PLLI2SRDY_Pos               (27U)                               
#define RCC_CR_PLLI2SRDY                   (0x1UL << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos               (0U)                                
#define RCC_PLLCFGR_PLLM                   (0x3FUL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM_0                 (0x01UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1                 (0x02UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2                 (0x04UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3                 (0x08UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4                 (0x10UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5                 (0x20UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000020 */

#define RCC_PLLCFGR_PLLN_Pos               (6U)                                
#define RCC_PLLCFGR_PLLN                   (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN_0                 (0x001UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1                 (0x002UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2                 (0x004UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3                 (0x008UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4                 (0x010UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5                 (0x020UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6                 (0x040UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7                 (0x080UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8                 (0x100UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_Pos               (16U)                               
#define RCC_PLLCFGR_PLLP                   (0x3UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP_0                 (0x1UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1                 (0x2UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)                               
#define RCC_PLLCFGR_PLLSRC                 (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)                               
#define RCC_PLLCFGR_PLLSRC_HSE             (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U                         

#define RCC_PLLCFGR_PLLQ_Pos               (24U)                               
#define RCC_PLLCFGR_PLLQ                   (0xFUL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ_0                 (0x1UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1                 (0x2UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2                 (0x4UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3                 (0x8UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x08000000 */


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)                                
#define RCC_CFGR_SW                        (0x3UL << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW_0                      (0x1UL << RCC_CFGR_SW_Pos)           /*!< 0x00000001 */
#define RCC_CFGR_SW_1                      (0x2UL << RCC_CFGR_SW_Pos)           /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)                                
#define RCC_CFGR_SWS                       (0x3UL << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */
#define RCC_CFGR_SWS_0                     (0x1UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                     (0x2UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)                                
#define RCC_CFGR_HPRE                      (0xFUL << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE_0                    (0x1UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                    (0x2UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                    (0x4UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                    (0x8UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)                               
#define RCC_CFGR_PPRE1                     (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1_0                   (0x1UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)                               
#define RCC_CFGR_PPRE2                     (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2_0                   (0x1UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)                               
#define RCC_CFGR_RTCPRE                    (0x1FUL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE_0                  (0x01UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1                  (0x02UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2                  (0x04UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3                  (0x08UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4                  (0x10UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)                               
#define RCC_CFGR_MCO1                      (0x3UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */
#define RCC_CFGR_MCO1_0                    (0x1UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                    (0x2UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)                               
#define RCC_CFGR_I2SSRC                    (0x1UL << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */

#define RCC_CFGR_MCO1PRE_Pos               (24U)                               
#define RCC_CFGR_MCO1PRE                   (0x7UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE_0                 (0x1UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1                 (0x2UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2                 (0x4UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)                               
#define RCC_CFGR_MCO2PRE                   (0x7UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE_0                 (0x1UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1                 (0x2UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2                 (0x4UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)                               
#define RCC_CFGR_MCO2                      (0x3UL << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */
#define RCC_CFGR_MCO2_0                    (0x1UL << RCC_CFGR_MCO2_Pos)         /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                    (0x2UL << RCC_CFGR_MCO2_Pos)         /*!< 0x80000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)                                
#define RCC_CIR_LSIRDYF                    (0x1UL << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIR_LSERDYF_Pos                (1U)                                
#define RCC_CIR_LSERDYF                    (0x1UL << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIR_HSIRDYF_Pos                (2U)                                
#define RCC_CIR_HSIRDYF                    (0x1UL << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */
#define RCC_CIR_HSERDYF_Pos                (3U)                                
#define RCC_CIR_HSERDYF                    (0x1UL << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIR_PLLRDYF_Pos                (4U)                                
#define RCC_CIR_PLLRDYF                    (0x1UL << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIR_PLLI2SRDYF_Pos             (5U)                                
#define RCC_CIR_PLLI2SRDYF                 (0x1UL << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */

#define RCC_CIR_CSSF_Pos                   (7U)                                
#define RCC_CIR_CSSF                       (0x1UL << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */
#define RCC_CIR_LSIRDYIE_Pos               (8U)                                
#define RCC_CIR_LSIRDYIE                   (0x1UL << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */
#define RCC_CIR_LSERDYIE_Pos               (9U)                                
#define RCC_CIR_LSERDYIE                   (0x1UL << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */
#define RCC_CIR_HSIRDYIE_Pos               (10U)                               
#define RCC_CIR_HSIRDYIE                   (0x1UL << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIR_HSERDYIE_Pos               (11U)                               
#define RCC_CIR_HSERDYIE                   (0x1UL << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */
#define RCC_CIR_PLLRDYIE_Pos               (12U)                               
#define RCC_CIR_PLLRDYIE                   (0x1UL << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */
#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)                               
#define RCC_CIR_PLLI2SRDYIE                (0x1UL << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */

#define RCC_CIR_LSIRDYC_Pos                (16U)                               
#define RCC_CIR_LSIRDYC                    (0x1UL << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */
#define RCC_CIR_LSERDYC_Pos                (17U)                               
#define RCC_CIR_LSERDYC                    (0x1UL << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */
#define RCC_CIR_HSIRDYC_Pos                (18U)                               
#define RCC_CIR_HSIRDYC                    (0x1UL << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */
#define RCC_CIR_HSERDYC_Pos                (19U)                               
#define RCC_CIR_HSERDYC                    (0x1UL << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */
#define RCC_CIR_PLLRDYC_Pos                (20U)                               
#define RCC_CIR_PLLRDYC                    (0x1UL << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */
#define RCC_CIR_PLLI2SRDYC_Pos             (21U)                               
#define RCC_CIR_PLLI2SRDYC                 (0x1UL << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */

#define RCC_CIR_CSSC_Pos                   (23U)                               
#define RCC_CIR_CSSC                       (0x1UL << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos          (0U)                                
#define RCC_AHB1RSTR_GPIOARST              (0x1UL << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */
#define RCC_AHB1RSTR_GPIOBRST_Pos          (1U)                                
#define RCC_AHB1RSTR_GPIOBRST              (0x1UL << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */
#define RCC_AHB1RSTR_GPIOCRST_Pos          (2U)                                
#define RCC_AHB1RSTR_GPIOCRST              (0x1UL << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */
#define RCC_AHB1RSTR_GPIODRST_Pos          (3U)                                
#define RCC_AHB1RSTR_GPIODRST              (0x1UL << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */
#define RCC_AHB1RSTR_GPIOERST_Pos          (4U)                                
#define RCC_AHB1RSTR_GPIOERST              (0x1UL << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */
#define RCC_AHB1RSTR_GPIOHRST_Pos          (7U)                                
#define RCC_AHB1RSTR_GPIOHRST              (0x1UL << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */
#define RCC_AHB1RSTR_CRCRST_Pos            (12U)                               
#define RCC_AHB1RSTR_CRCRST                (0x1UL << RCC_AHB1RSTR_CRCRST_Pos)   /*!< 0x00001000 */
#define RCC_AHB1RSTR_DMA1RST_Pos           (21U)                               
#define RCC_AHB1RSTR_DMA1RST               (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos)  /*!< 0x00200000 */
#define RCC_AHB1RSTR_DMA2RST_Pos           (22U)                               
#define RCC_AHB1RSTR_DMA2RST               (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos)  /*!< 0x00400000 */

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_OTGFSRST_Pos          (7U)                                
#define RCC_AHB2RSTR_OTGFSRST              (0x1UL << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */
/********************  Bit definition for RCC_AHB3RSTR register  **************/


/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos           (0U)                                
#define RCC_APB1RSTR_TIM2RST               (0x1UL << RCC_APB1RSTR_TIM2RST_Pos)  /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM3RST_Pos           (1U)                                
#define RCC_APB1RSTR_TIM3RST               (0x1UL << RCC_APB1RSTR_TIM3RST_Pos)  /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM4RST_Pos           (2U)                                
#define RCC_APB1RSTR_TIM4RST               (0x1UL << RCC_APB1RSTR_TIM4RST_Pos)  /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM5RST_Pos           (3U)                                
#define RCC_APB1RSTR_TIM5RST               (0x1UL << RCC_APB1RSTR_TIM5RST_Pos)  /*!< 0x00000008 */
#define RCC_APB1RSTR_WWDGRST_Pos           (11U)                               
#define RCC_APB1RSTR_WWDGRST               (0x1UL << RCC_APB1RSTR_WWDGRST_Pos)  /*!< 0x00000800 */
#define RCC_APB1RSTR_SPI2RST_Pos           (14U)                               
#define RCC_APB1RSTR_SPI2RST               (0x1UL << RCC_APB1RSTR_SPI2RST_Pos)  /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI3RST_Pos           (15U)                               
#define RCC_APB1RSTR_SPI3RST               (0x1UL << RCC_APB1RSTR_SPI3RST_Pos)  /*!< 0x00008000 */
#define RCC_APB1RSTR_USART2RST_Pos         (17U)                               
#define RCC_APB1RSTR_USART2RST             (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_I2C1RST_Pos           (21U)                               
#define RCC_APB1RSTR_I2C1RST               (0x1UL << RCC_APB1RSTR_I2C1RST_Pos)  /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C2RST_Pos           (22U)                               
#define RCC_APB1RSTR_I2C2RST               (0x1UL << RCC_APB1RSTR_I2C2RST_Pos)  /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C3RST_Pos           (23U)                               
#define RCC_APB1RSTR_I2C3RST               (0x1UL << RCC_APB1RSTR_I2C3RST_Pos)  /*!< 0x00800000 */
#define RCC_APB1RSTR_PWRRST_Pos            (28U)                               
#define RCC_APB1RSTR_PWRRST                (0x1UL << RCC_APB1RSTR_PWRRST_Pos)   /*!< 0x10000000 */

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos           (0U)                                
#define RCC_APB2RSTR_TIM1RST               (0x1UL << RCC_APB2RSTR_TIM1RST_Pos)  /*!< 0x00000001 */
#define RCC_APB2RSTR_USART1RST_Pos         (4U)                                
#define RCC_APB2RSTR_USART1RST             (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_USART6RST_Pos         (5U)                                
#define RCC_APB2RSTR_USART6RST             (0x1UL << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_ADCRST_Pos            (8U)                                
#define RCC_APB2RSTR_ADCRST                (0x1UL << RCC_APB2RSTR_ADCRST_Pos)   /*!< 0x00000100 */
#define RCC_APB2RSTR_SDIORST_Pos           (11U)                               
#define RCC_APB2RSTR_SDIORST               (0x1UL << RCC_APB2RSTR_SDIORST_Pos)  /*!< 0x00000800 */
#define RCC_APB2RSTR_SPI1RST_Pos           (12U)                               
#define RCC_APB2RSTR_SPI1RST               (0x1UL << RCC_APB2RSTR_SPI1RST_Pos)  /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI4RST_Pos           (13U)                               
#define RCC_APB2RSTR_SPI4RST               (0x1UL << RCC_APB2RSTR_SPI4RST_Pos)  /*!< 0x00002000 */
#define RCC_APB2RSTR_SYSCFGRST_Pos         (14U)                               
#define RCC_APB2RSTR_SYSCFGRST             (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_TIM9RST_Pos           (16U)                               
#define RCC_APB2RSTR_TIM9RST               (0x1UL << RCC_APB2RSTR_TIM9RST_Pos)  /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM10RST_Pos          (17U)                               
#define RCC_APB2RSTR_TIM10RST              (0x1UL << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM11RST_Pos          (18U)                               
#define RCC_APB2RSTR_TIM11RST              (0x1UL << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_SPI5RST_Pos           (20U)                               
#define RCC_APB2RSTR_SPI5RST               (0x1UL << RCC_APB2RSTR_SPI5RST_Pos)  /*!< 0x00100000 */

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)                                
#define RCC_AHB1ENR_GPIOAEN                (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)                                
#define RCC_AHB1ENR_GPIOBEN                (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)                                
#define RCC_AHB1ENR_GPIOCEN                (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIODEN_Pos            (3U)                                
#define RCC_AHB1ENR_GPIODEN                (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)                                
#define RCC_AHB1ENR_GPIOEEN                (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)                                
#define RCC_AHB1ENR_GPIOHEN                (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB1ENR_CRCEN_Pos              (12U)                               
#define RCC_AHB1ENR_CRCEN                  (0x1UL << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */
#define RCC_AHB1ENR_DMA1EN_Pos             (21U)                               
#define RCC_AHB1ENR_DMA1EN                 (0x1UL << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA2EN_Pos             (22U)                               
#define RCC_AHB1ENR_DMA2EN                 (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB2_SUPPORT                   /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)                                
#define RCC_AHB2ENR_OTGFSEN                (0x1UL << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)                                
#define RCC_APB1ENR_TIM2EN                 (0x1UL << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */
#define RCC_APB1ENR_TIM3EN_Pos             (1U)                                
#define RCC_APB1ENR_TIM3EN                 (0x1UL << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */
#define RCC_APB1ENR_TIM4EN_Pos             (2U)                                
#define RCC_APB1ENR_TIM4EN                 (0x1UL << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */
#define RCC_APB1ENR_TIM5EN_Pos             (3U)                                
#define RCC_APB1ENR_TIM5EN                 (0x1UL << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */
#define RCC_APB1ENR_WWDGEN_Pos             (11U)                               
#define RCC_APB1ENR_WWDGEN                 (0x1UL << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */
#define RCC_APB1ENR_SPI2EN_Pos             (14U)                               
#define RCC_APB1ENR_SPI2EN                 (0x1UL << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */
#define RCC_APB1ENR_SPI3EN_Pos             (15U)                               
#define RCC_APB1ENR_SPI3EN                 (0x1UL << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */
#define RCC_APB1ENR_USART2EN_Pos           (17U)                               
#define RCC_APB1ENR_USART2EN               (0x1UL << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */
#define RCC_APB1ENR_I2C1EN_Pos             (21U)                               
#define RCC_APB1ENR_I2C1EN                 (0x1UL << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */
#define RCC_APB1ENR_I2C2EN_Pos             (22U)                               
#define RCC_APB1ENR_I2C2EN                 (0x1UL << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */
#define RCC_APB1ENR_I2C3EN_Pos             (23U)                               
#define RCC_APB1ENR_I2C3EN                 (0x1UL << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */
#define RCC_APB1ENR_PWREN_Pos              (28U)                               
#define RCC_APB1ENR_PWREN                  (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)                                
#define RCC_APB2ENR_TIM1EN                 (0x1UL << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */
#define RCC_APB2ENR_USART1EN_Pos           (4U)                                
#define RCC_APB2ENR_USART1EN               (0x1UL << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_USART6EN_Pos           (5U)                                
#define RCC_APB2ENR_USART6EN               (0x1UL << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_ADC1EN_Pos             (8U)                                
#define RCC_APB2ENR_ADC1EN                 (0x1UL << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */
#define RCC_APB2ENR_SDIOEN_Pos             (11U)                               
#define RCC_APB2ENR_SDIOEN                 (0x1UL << RCC_APB2ENR_SDIOEN_Pos)    /*!< 0x00000800 */
#define RCC_APB2ENR_SPI1EN_Pos             (12U)                               
#define RCC_APB2ENR_SPI1EN                 (0x1UL << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI4EN_Pos             (13U)                               
#define RCC_APB2ENR_SPI4EN                 (0x1UL << RCC_APB2ENR_SPI4EN_Pos)    /*!< 0x00002000 */
#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)                               
#define RCC_APB2ENR_SYSCFGEN               (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_TIM9EN_Pos             (16U)                               
#define RCC_APB2ENR_TIM9EN                 (0x1UL << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */
#define RCC_APB2ENR_TIM10EN_Pos            (17U)                               
#define RCC_APB2ENR_TIM10EN                (0x1UL << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */
#define RCC_APB2ENR_TIM11EN_Pos            (18U)                               
#define RCC_APB2ENR_TIM11EN                (0x1UL << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */
#define RCC_APB2ENR_SPI5EN_Pos             (20U)                               
#define RCC_APB2ENR_SPI5EN                 (0x1UL << RCC_APB2ENR_SPI5EN_Pos)    /*!< 0x00100000 */

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)                                
#define RCC_AHB1LPENR_GPIOALPEN            (0x1UL << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)                                
#define RCC_AHB1LPENR_GPIOBLPEN            (0x1UL << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)                                
#define RCC_AHB1LPENR_GPIOCLPEN            (0x1UL << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)                                
#define RCC_AHB1LPENR_GPIODLPEN            (0x1UL << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)                                
#define RCC_AHB1LPENR_GPIOELPEN            (0x1UL << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)                                
#define RCC_AHB1LPENR_GPIOHLPEN            (0x1UL << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)                               
#define RCC_AHB1LPENR_CRCLPEN              (0x1UL << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)                               
#define RCC_AHB1LPENR_FLITFLPEN            (0x1UL << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)                               
#define RCC_AHB1LPENR_SRAM1LPEN            (0x1UL << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)                               
#define RCC_AHB1LPENR_DMA1LPEN             (0x1UL << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)                               
#define RCC_AHB1LPENR_DMA2LPEN             (0x1UL << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */


/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)                                
#define RCC_AHB2LPENR_OTGFSLPEN            (0x1UL << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */

/********************  Bit definition for RCC_AHB3LPENR register  *************/

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)                                
#define RCC_APB1LPENR_TIM2LPEN             (0x1UL << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)                                
#define RCC_APB1LPENR_TIM3LPEN             (0x1UL << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)                                
#define RCC_APB1LPENR_TIM4LPEN             (0x1UL << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)                                
#define RCC_APB1LPENR_TIM5LPEN             (0x1UL << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)                               
#define RCC_APB1LPENR_WWDGLPEN             (0x1UL << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)                               
#define RCC_APB1LPENR_SPI2LPEN             (0x1UL << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)                               
#define RCC_APB1LPENR_SPI3LPEN             (0x1UL << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)                               
#define RCC_APB1LPENR_USART2LPEN           (0x1UL << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)                               
#define RCC_APB1LPENR_I2C1LPEN             (0x1UL << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)
#define RCC_APB1LPENR_I2C2LPEN             (0x1UL << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)                               
#define RCC_APB1LPENR_I2C3LPEN             (0x1UL << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)                               
#define RCC_APB1LPENR_PWRLPEN              (0x1UL << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)                                
#define RCC_APB2LPENR_TIM1LPEN             (0x1UL << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)                                
#define RCC_APB2LPENR_USART1LPEN           (0x1UL << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)                                
#define RCC_APB2LPENR_USART6LPEN           (0x1UL << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)                                
#define RCC_APB2LPENR_ADC1LPEN             (0x1UL << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_SDIOLPEN_Pos         (11U)                               
#define RCC_APB2LPENR_SDIOLPEN             (0x1UL << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)                               
#define RCC_APB2LPENR_SPI1LPEN             (0x1UL << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI4LPEN_Pos         (13U)                               
#define RCC_APB2LPENR_SPI4LPEN             (0x1UL << RCC_APB2LPENR_SPI4LPEN_Pos) /*!< 0x00002000 */
#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)                               
#define RCC_APB2LPENR_SYSCFGLPEN           (0x1UL << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)                               
#define RCC_APB2LPENR_TIM9LPEN             (0x1UL << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)                               
#define RCC_APB2LPENR_TIM10LPEN            (0x1UL << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)                               
#define RCC_APB2LPENR_TIM11LPEN            (0x1UL << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_SPI5LPEN_Pos         (20U)                               
#define RCC_APB2LPENR_SPI5LPEN             (0x1UL << RCC_APB2LPENR_SPI5LPEN_Pos) /*!< 0x00100000 */

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)                                
#define RCC_BDCR_LSEON                     (0x1UL << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */
#define RCC_BDCR_LSERDY_Pos                (1U)                                
#define RCC_BDCR_LSERDY                    (0x1UL << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */
#define RCC_BDCR_LSEBYP_Pos                (2U)                                
#define RCC_BDCR_LSEBYP                    (0x1UL << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */
#define RCC_BDCR_LSEMOD_Pos                (3U)                                
#define RCC_BDCR_LSEMOD                    (0x1UL << RCC_BDCR_LSEMOD_Pos)       /*!< 0x00000008 */

#define RCC_BDCR_RTCSEL_Pos                (8U)                                
#define RCC_BDCR_RTCSEL                    (0x3UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL_0                  (0x1UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                  (0x2UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000200 */

#define RCC_BDCR_RTCEN_Pos                 (15U)                               
#define RCC_BDCR_RTCEN                     (0x1UL << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */
#define RCC_BDCR_BDRST_Pos                 (16U)                               
#define RCC_BDCR_BDRST                     (0x1UL << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)                                
#define RCC_CSR_LSION                      (0x1UL << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */
#define RCC_CSR_LSIRDY_Pos                 (1U)                                
#define RCC_CSR_LSIRDY                     (0x1UL << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */
#define RCC_CSR_RMVF_Pos                   (24U)                               
#define RCC_CSR_RMVF                       (0x1UL << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */
#define RCC_CSR_BORRSTF_Pos                (25U)                               
#define RCC_CSR_BORRSTF                    (0x1UL << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */
#define RCC_CSR_PINRSTF_Pos                (26U)
#define RCC_CSR_PINRSTF                    (0x1UL << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */
#define RCC_CSR_PORRSTF_Pos                (27U)                               
#define RCC_CSR_PORRSTF                    (0x1UL << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */
#define RCC_CSR_SFTRSTF_Pos                (28U)                               
#define RCC_CSR_SFTRSTF                    (0x1UL << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */
#define RCC_CSR_IWDGRSTF_Pos               (29U)
#define RCC_CSR_IWDGRSTF                   (0x1UL << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */
#define RCC_CSR_WWDGRSTF_Pos               (30U)                               
#define RCC_CSR_WWDGRSTF                   (0x1UL << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */
#define RCC_CSR_LPWRRSTF_Pos               (31U)                               
#define RCC_CSR_LPWRRSTF                   (0x1UL << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */


/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)                                
#define RCC_SSCGR_MODPER                   (0x1FFFUL << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */
#define RCC_SSCGR_INCSTEP_Pos              (13U)                               
#define RCC_SSCGR_INCSTEP                  (0x7FFFUL << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */
#define RCC_SSCGR_SPREADSEL_Pos            (30U)                               
#define RCC_SSCGR_SPREADSEL                (0x1UL << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */
#define RCC_SSCGR_SSCGEN_Pos               (31U)                               
#define RCC_SSCGR_SSCGEN                   (0x1UL << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SM_Pos         (0U)                                
#define RCC_PLLI2SCFGR_PLLI2SM             (0x3FUL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x0000003F */
#define RCC_PLLI2SCFGR_PLLI2SM_0           (0x01UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000001 */
#define RCC_PLLI2SCFGR_PLLI2SM_1           (0x02UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000002 */
#define RCC_PLLI2SCFGR_PLLI2SM_2           (0x04UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000004 */
#define RCC_PLLI2SCFGR_PLLI2SM_3           (0x08UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000008 */
#define RCC_PLLI2SCFGR_PLLI2SM_4           (0x10UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000010 */
#define RCC_PLLI2SCFGR_PLLI2SM_5           (0x20UL << RCC_PLLI2SCFGR_PLLI2SM_Pos) /*!< 0x00000020 */

#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)                                
#define RCC_PLLI2SCFGR_PLLI2SN             (0x1FFUL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN_0           (0x001UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1           (0x002UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2           (0x004UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3           (0x008UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4           (0x010UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5           (0x020UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6           (0x040UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7           (0x080UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8           (0x100UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */

#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)                               
#define RCC_PLLI2SCFGR_PLLI2SR             (0x7UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_0           (0x1UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1           (0x2UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2           (0x4UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_DCKCFGR register  ***************/

#define RCC_DCKCFGR_TIMPRE_Pos             (24U)                               
#define RCC_DCKCFGR_TIMPRE                 (0x1UL << RCC_DCKCFGR_TIMPRE_Pos)    /*!< 0x01000000 */



/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
#define SPI_I2S_FULLDUPLEX_SUPPORT                                             /*!< I2S Full-Duplex support */

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA                (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL                (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR                (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */

#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR                  (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE                 (0x1UL << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST            (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI                 (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM                 (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY              (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_DFF_Pos             (11U)                                      
#define SPI_CR1_DFF                 (0x1UL << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT             (0x1UL << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN               (0x1UL << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE              (0x1UL << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE            (0x1UL << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN             (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN             (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE                (0x1UL << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF                 (0x1UL << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE               (0x1UL << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE              (0x1UL << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE               (0x1UL << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE                 (0x1UL << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE                  (0x1UL << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_CHSIDE_Pos           (2U)                                       
#define SPI_SR_CHSIDE               (0x1UL << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_UDR_Pos              (3U)                                       
#define SPI_SR_UDR                  (0x1UL << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR               (0x1UL << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF                 (0x1UL << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR                  (0x1UL << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY                  (0x1UL << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE                  (0x1UL << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR                   (0xFFFFUL << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY           (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC            (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC            (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN_Pos       (0U)                                       
#define SPI_I2SCFGR_CHLEN           (0x1UL << SPI_I2SCFGR_CHLEN_Pos)            /*!< 0x00000001 */

#define SPI_I2SCFGR_DATLEN_Pos      (1U)                                       
#define SPI_I2SCFGR_DATLEN          (0x3UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000006 */
#define SPI_I2SCFGR_DATLEN_0        (0x1UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000002 */
#define SPI_I2SCFGR_DATLEN_1        (0x2UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000004 */

#define SPI_I2SCFGR_CKPOL_Pos       (3U)                                       
#define SPI_I2SCFGR_CKPOL           (0x1UL << SPI_I2SCFGR_CKPOL_Pos)            /*!< 0x00000008 */

#define SPI_I2SCFGR_I2SSTD_Pos      (4U)                                       
#define SPI_I2SCFGR_I2SSTD          (0x3UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000030 */
#define SPI_I2SCFGR_I2SSTD_0        (0x1UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000010 */
#define SPI_I2SCFGR_I2SSTD_1        (0x2UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000020 */

#define SPI_I2SCFGR_PCMSYNC_Pos     (7U)                                       
#define SPI_I2SCFGR_PCMSYNC         (0x1UL << SPI_I2SCFGR_PCMSYNC_Pos)          /*!< 0x00000080 */

#define SPI_I2SCFGR_I2SCFG_Pos      (8U)                                       
#define SPI_I2SCFGR_I2SCFG          (0x3UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000300 */
#define SPI_I2SCFGR_I2SCFG_0        (0x1UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000100 */
#define SPI_I2SCFGR_I2SCFG_1        (0x2UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000200 */

#define SPI_I2SCFGR_I2SE_Pos        (10U)                                      
#define SPI_I2SCFGR_I2SE            (0x1UL << SPI_I2SCFGR_I2SE_Pos)             /*!< 0x00000400 */
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD          (0x1UL << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_I2SDIV_Pos        (0U)                                       
#define SPI_I2SPR_I2SDIV            (0xFFUL << SPI_I2SPR_I2SDIV_Pos)            /*!< 0x000000FF */
#define SPI_I2SPR_ODD_Pos           (8U)                                       
#define SPI_I2SPR_ODD               (0x1UL << SPI_I2SPR_ODD_Pos)                /*!< 0x00000100 */
#define SPI_I2SPR_MCKOE_Pos         (9U)                                       
#define SPI_I2SPR_MCKOE             (0x1UL << SPI_I2SPR_MCKOE_Pos)              /*!< 0x00000200 */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE_Pos           (0U)                              
#define SYSCFG_MEMRMP_MEM_MODE               (0x3UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_MEMRMP_MEM_MODE_0             (0x1UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000001 */
#define SYSCFG_MEMRMP_MEM_MODE_1             (0x2UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000002 */
/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_ADC1DC2_Pos               (16U)                             
#define SYSCFG_PMC_ADC1DC2                   (0x1UL << SYSCFG_PMC_ADC1DC2_Pos)  /*!< 0x00010000 */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos             (0U)                              
#define SYSCFG_EXTICR1_EXTI0                 (0xFUL << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI1_Pos             (4U)                              
#define SYSCFG_EXTICR1_EXTI1                 (0xFUL << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI2_Pos             (8U)                              
#define SYSCFG_EXTICR1_EXTI2                 (0xFUL << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI3_Pos             (12U)                             
#define SYSCFG_EXTICR1_EXTI3                 (0xFUL << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
/**
  * @brief   EXTI0 configuration  
  */
#define SYSCFG_EXTICR1_EXTI0_PA              0x0000U                           /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB              0x0001U                           /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC              0x0002U                           /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD              0x0003U                           /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE              0x0004U                           /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH              0x0007U                           /*!<PH[0] pin */

/**
  * @brief   EXTI1 configuration  
  */
#define SYSCFG_EXTICR1_EXTI1_PA              0x0000U                           /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB              0x0010U                           /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC              0x0020U                           /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD              0x0030U                           /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE              0x0040U                           /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH              0x0070U                           /*!<PH[1] pin */

/**
  * @brief   EXTI2 configuration  
  */
#define SYSCFG_EXTICR1_EXTI2_PA              0x0000U                           /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB              0x0100U                           /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC              0x0200U                           /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD              0x0300U                           /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE              0x0400U                           /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH              0x0700U                           /*!<PH[2] pin */

/**
  * @brief   EXTI3 configuration  
  */
#define SYSCFG_EXTICR1_EXTI3_PA              0x0000U                           /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB              0x1000U                           /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC              0x2000U                           /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD              0x3000U                           /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE              0x4000U                           /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH              0x7000U                           /*!<PH[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4_Pos             (0U)                              
#define SYSCFG_EXTICR2_EXTI4                 (0xFUL << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI5_Pos             (4U)                              
#define SYSCFG_EXTICR2_EXTI5                 (0xFUL << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI6_Pos             (8U)                              
#define SYSCFG_EXTICR2_EXTI6                 (0xFUL << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI7_Pos             (12U)                             
#define SYSCFG_EXTICR2_EXTI7                 (0xFUL << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */

/**
  * @brief   EXTI4 configuration  
  */
#define SYSCFG_EXTICR2_EXTI4_PA              0x0000U                           /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB              0x0001U                           /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC              0x0002U                           /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD              0x0003U                           /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE              0x0004U                           /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH              0x0007U                           /*!<PH[4] pin */

/**
  * @brief   EXTI5 configuration  
  */
#define SYSCFG_EXTICR2_EXTI5_PA              0x0000U                           /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB              0x0010U                           /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC              0x0020U                           /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD              0x0030U                           /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE              0x0040U                           /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH              0x0070U                           /*!<PH[5] pin */

/**
  * @brief   EXTI6 configuration  
  */
#define SYSCFG_EXTICR2_EXTI6_PA              0x0000U                           /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB              0x0100U                           /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC              0x0200U                           /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD              0x0300U                           /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE              0x0400U                           /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH              0x0700U                           /*!<PH[6] pin */

/**
  * @brief   EXTI7 configuration  
  */
#define SYSCFG_EXTICR2_EXTI7_PA              0x0000U                           /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB              0x1000U                           /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC              0x2000U                           /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD              0x3000U                           /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE              0x4000U                           /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH              0x7000U                           /*!<PH[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8_Pos             (0U)                              
#define SYSCFG_EXTICR3_EXTI8                 (0xFUL << SYSCFG_EXTICR3_EXTI8_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI9_Pos             (4U)                              
#define SYSCFG_EXTICR3_EXTI9                 (0xFUL << SYSCFG_EXTICR3_EXTI9_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI10_Pos            (8U)                              
#define SYSCFG_EXTICR3_EXTI10                (0xFUL << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI11_Pos            (12U)                             
#define SYSCFG_EXTICR3_EXTI11                (0xFUL << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */

/**
  * @brief   EXTI8 configuration  
  */
#define SYSCFG_EXTICR3_EXTI8_PA              0x0000U                           /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB              0x0001U                           /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC              0x0002U                           /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD              0x0003U                           /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE              0x0004U                           /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH              0x0007U                           /*!<PH[8] pin */

/**
  * @brief   EXTI9 configuration  
  */
#define SYSCFG_EXTICR3_EXTI9_PA              0x0000U                           /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB              0x0010U                           /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC              0x0020U                           /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD              0x0030U                           /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE              0x0040U                           /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH              0x0070U                           /*!<PH[9] pin */

/**
  * @brief   EXTI10 configuration  
  */
#define SYSCFG_EXTICR3_EXTI10_PA             0x0000U                           /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB             0x0100U                           /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC             0x0200U                           /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD             0x0300U                           /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE             0x0400U                           /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH             0x0700U                           /*!<PH[10] pin */

/**
  * @brief   EXTI11 configuration  
  */
#define SYSCFG_EXTICR3_EXTI11_PA             0x0000U                           /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB             0x1000U                           /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC             0x2000U                           /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD             0x3000U                           /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE             0x4000U                           /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH             0x7000U                           /*!<PH[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12_Pos            (0U)                              
#define SYSCFG_EXTICR4_EXTI12                (0xFUL << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI13_Pos            (4U)                              
#define SYSCFG_EXTICR4_EXTI13                (0xFUL << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI14_Pos            (8U)                              
#define SYSCFG_EXTICR4_EXTI14                (0xFUL << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI15_Pos            (12U)                             
#define SYSCFG_EXTICR4_EXTI15                (0xFUL << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */

/**
  * @brief   EXTI12 configuration  
  */
#define SYSCFG_EXTICR4_EXTI12_PA             0x0000U                           /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB             0x0001U                           /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC             0x0002U                           /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD             0x0003U                           /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE             0x0004U                           /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH             0x0007U                           /*!<PH[12] pin */

/**
  * @brief   EXTI13 configuration  
  */
#define SYSCFG_EXTICR4_EXTI13_PA             0x0000U                           /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB             0x0010U                           /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC             0x0020U                           /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD             0x0030U                           /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE             0x0040U                           /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH             0x0070U                           /*!<PH[13] pin */

/**
  * @brief   EXTI14 configuration  
  */
#define SYSCFG_EXTICR4_EXTI14_PA             0x0000U                           /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB             0x0100U                           /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC             0x0200U                           /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD             0x0300U                           /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE             0x0400U                           /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH             0x0700U                           /*!<PH[14] pin */

/**
  * @brief   EXTI15 configuration  
  */
#define SYSCFG_EXTICR4_EXTI15_PA             0x0000U                           /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB             0x1000U                           /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC             0x2000U                           /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD             0x3000U                           /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE             0x4000U                           /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH             0x7000U                           /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD_Pos              (0U)                              
#define SYSCFG_CMPCR_CMP_PD                  (0x1UL << SYSCFG_CMPCR_CMP_PD_Pos) /*!< 0x00000001 */
#define SYSCFG_CMPCR_READY_Pos               (8U)                              
#define SYSCFG_CMPCR_READY                   (0x1UL << SYSCFG_CMPCR_READY_Pos)  /*!< 0x00000100 */



/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE_Pos               (0U)                                     
#define USART_SR_PE                   (0x1UL << USART_SR_PE_Pos)                /*!< 0x00000001 */
#define USART_SR_FE_Pos               (1U)                                     
#define USART_SR_FE                   (0x1UL << USART_SR_FE_Pos)                /*!< 0x00000002 */
#define USART_SR_NE_Pos               (2U)                                     
#define USART_SR_NE                   (0x1UL << USART_SR_NE_Pos)                /*!< 0x00000004 */
#define USART_SR_ORE_Pos              (3U)                                     
#define USART_SR_ORE                  (0x1UL << USART_SR_ORE_Pos)               /*!< 0x00000008 */
#define USART_SR_IDLE_Pos             (4U)                                     
#define USART_SR_IDLE                 (0x1UL << USART_SR_IDLE_Pos)              /*!< 0x00000010 */
#define USART_SR_RXNE_Pos             (5U)                                     
#define USART_SR_RXNE                 (0x1UL << USART_SR_RXNE_Pos)              /*!< 0x00000020 */
#define USART_SR_TC_Pos               (6U)                                     
#define USART_SR_TC                   (0x1UL << USART_SR_TC_Pos)                /*!< 0x00000040 */
#define USART_SR_TXE_Pos              (7U)                                     
#define USART_SR_TXE                  (0x1UL << USART_SR_TXE_Pos)               /*!< 0x00000080 */
#define USART_SR_LBD_Pos              (8U)                                     
#define USART_SR_LBD                  (0x1UL << USART_SR_LBD_Pos)               /*!< 0x00000100 */
#define USART_SR_CTS_Pos              (9U)                                     
#define USART_SR_CTS                  (0x1UL << USART_SR_CTS_Pos)               /*!< 0x00000200 */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR_Pos               (0U)                                     
#define USART_DR_DR                   (0x1FFUL << USART_DR_DR_Pos)              /*!< 0x000001FF */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction_Pos    (0U)                                     
#define USART_BRR_DIV_Fraction        (0xFUL << USART_BRR_DIV_Fraction_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_Mantissa_Pos    (4U)                                     
#define USART_BRR_DIV_Mantissa        (0xFFFUL << USART_BRR_DIV_Mantissa_Pos)   /*!< 0x0000FFF0 */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK_Pos             (0U)                                     
#define USART_CR1_SBK                 (0x1UL << USART_CR1_SBK_Pos)              /*!< 0x00000001 */
#define USART_CR1_RWU_Pos             (1U)                                     
#define USART_CR1_RWU                 (0x1UL << USART_CR1_RWU_Pos)              /*!< 0x00000002 */
#define USART_CR1_RE_Pos              (2U)                                     
#define USART_CR1_RE                  (0x1UL << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_TE_Pos              (3U)                                     
#define USART_CR1_TE                  (0x1UL << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_IDLEIE_Pos          (4U)                                     
#define USART_CR1_IDLEIE              (0x1UL << USART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define USART_CR1_RXNEIE_Pos          (5U)                                     
#define USART_CR1_RXNEIE              (0x1UL << USART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define USART_CR1_TCIE_Pos            (6U)                                     
#define USART_CR1_TCIE                (0x1UL << USART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define USART_CR1_TXEIE_Pos           (7U)                                     
#define USART_CR1_TXEIE               (0x1UL << USART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define USART_CR1_PEIE_Pos            (8U)                                     
#define USART_CR1_PEIE                (0x1UL << USART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define USART_CR1_PS_Pos              (9U)                                     
#define USART_CR1_PS                  (0x1UL << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PCE_Pos             (10U)                                    
#define USART_CR1_PCE                 (0x1UL << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_WAKE_Pos            (11U)                                    
#define USART_CR1_WAKE                (0x1UL << USART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define USART_CR1_M_Pos               (12U)                                    
#define USART_CR1_M                   (0x1UL << USART_CR1_M_Pos)                /*!< 0x00001000 */
#define USART_CR1_UE_Pos              (13U)                                    
#define USART_CR1_UE                  (0x1UL << USART_CR1_UE_Pos)               /*!< 0x00002000 */
#define USART_CR1_OVER8_Pos           (15U)                                    
#define USART_CR1_OVER8               (0x1UL << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD_Pos             (0U)                                     
#define USART_CR2_ADD                 (0xFUL << USART_CR2_ADD_Pos)              /*!< 0x0000000F */
#define USART_CR2_LBDL_Pos            (5U)                                     
#define USART_CR2_LBDL                (0x1UL << USART_CR2_LBDL_Pos)             /*!< 0x00000020 */
#define USART_CR2_LBDIE_Pos           (6U)                                     
#define USART_CR2_LBDIE               (0x1UL << USART_CR2_LBDIE_Pos)            /*!< 0x00000040 */
#define USART_CR2_LBCL_Pos            (8U)                                     
#define USART_CR2_LBCL                (0x1UL << USART_CR2_LBCL_Pos)             /*!< 0x00000100 */
#define USART_CR2_CPHA_Pos            (9U)                                     
#define USART_CR2_CPHA                (0x1UL << USART_CR2_CPHA_Pos)             /*!< 0x00000200 */
#define USART_CR2_CPOL_Pos            (10U)                                    
#define USART_CR2_CPOL                (0x1UL << USART_CR2_CPOL_Pos)             /*!< 0x00000400 */
#define USART_CR2_CLKEN_Pos           (11U)                                    
#define USART_CR2_CLKEN               (0x1UL << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */

#define USART_CR2_STOP_Pos            (12U)                                    
#define USART_CR2_STOP                (0x3UL << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP_0              (0x1UL << USART_CR2_STOP_Pos)             /*!< 0x1000 */
#define USART_CR2_STOP_1              (0x2UL << USART_CR2_STOP_Pos)             /*!< 0x2000 */

#define USART_CR2_LINEN_Pos           (14U)                                    
#define USART_CR2_LINEN               (0x1UL << USART_CR2_LINEN_Pos)            /*!< 0x00004000 */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos             (0U)                                     
#define USART_CR3_EIE                 (0x1UL << USART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define USART_CR3_IREN_Pos            (1U)                                     
#define USART_CR3_IREN                (0x1UL << USART_CR3_IREN_Pos)             /*!< 0x00000002 */
#define USART_CR3_IRLP_Pos            (2U)                                     
#define USART_CR3_IRLP                (0x1UL << USART_CR3_IRLP_Pos)             /*!< 0x00000004 */
#define USART_CR3_HDSEL_Pos           (3U)                                     
#define USART_CR3_HDSEL               (0x1UL << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_NACK_Pos            (4U)                                     
#define USART_CR3_NACK                (0x1UL << USART_CR3_NACK_Pos)             /*!< 0x00000010 */
#define USART_CR3_SCEN_Pos            (5U)                                     
#define USART_CR3_SCEN                (0x1UL << USART_CR3_SCEN_Pos)             /*!< 0x00000020 */
#define USART_CR3_DMAR_Pos            (6U)                                     
#define USART_CR3_DMAR                (0x1UL << USART_CR3_DMAR_Pos)             /*!< 0x00000040 */
#define USART_CR3_DMAT_Pos            (7U)                                     
#define USART_CR3_DMAT                (0x1UL << USART_CR3_DMAT_Pos)             /*!< 0x00000080 */
#define USART_CR3_RTSE_Pos            (8U)                                     
#define USART_CR3_RTSE                (0x1UL << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_CTSE_Pos            (9U)                                     
#define USART_CR3_CTSE                (0x1UL << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSIE_Pos           (10U)                                    
#define USART_CR3_CTSIE               (0x1UL << USART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define USART_CR3_ONEBIT_Pos          (11U)                                    
#define USART_CR3_ONEBIT              (0x1UL << USART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos            (0U)                                     
#define USART_GTPR_PSC                (0xFFUL << USART_GTPR_PSC_Pos)            /*!< 0x000000FF */
#define USART_GTPR_PSC_0              (0x01UL << USART_GTPR_PSC_Pos)            /*!< 0x0001 */
#define USART_GTPR_PSC_1              (0x02UL << USART_GTPR_PSC_Pos)            /*!< 0x0002 */
#define USART_GTPR_PSC_2              (0x04UL << USART_GTPR_PSC_Pos)            /*!< 0x0004 */
#define USART_GTPR_PSC_3              (0x08UL << USART_GTPR_PSC_Pos)            /*!< 0x0008 */
#define USART_GTPR_PSC_4              (0x10UL << USART_GTPR_PSC_Pos)            /*!< 0x0010 */
#define USART_GTPR_PSC_5              (0x20UL << USART_GTPR_PSC_Pos)            /*!< 0x0020 */
#define USART_GTPR_PSC_6              (0x40UL << USART_GTPR_PSC_Pos)            /*!< 0x0040 */
#define USART_GTPR_PSC_7              (0x80UL << USART_GTPR_PSC_Pos)            /*!< 0x0080 */

#define USART_GTPR_GT_Pos             (8U)                                     
#define USART_GTPR_GT                 (0xFFUL << USART_GTPR_GT_Pos)             /*!< 0x0000FF00 */


/**
  * @}
  */ 

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_COMMON)

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOH))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/******************************* SMBUS Instances ******************************/
#define IS_SMBUS_ALL_INSTANCE         IS_I2C_ALL_INSTANCE

/******************************** I2S Instances *******************************/

#define IS_I2S_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == SPI1) || \
                                        ((INSTANCE) == SPI2) || \
                                        ((INSTANCE) == SPI3) || \
                                        ((INSTANCE) == SPI4) || \
                                        ((INSTANCE) == SPI5))

/*************************** I2S Extended Instances ***************************/
#define IS_I2S_EXT_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2S2ext)|| \
                                           ((INSTANCE) == I2S3ext))


/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)


/******************************** SPI Instances *******************************/

#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3) || \
                                       ((INSTANCE) == SPI4) || \
                                       ((INSTANCE) == SPI5))


/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                               ((INSTANCE) == USART2) || \
                                               ((INSTANCE) == USART6))


/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART6))
/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE          IS_UART_HALFDUPLEX_INSTANCE


/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART6))     

/*
 * @brief Specific devices reset values definitions
 */
#define RCC_PLLCFGR_RST_VALUE              0x24003010U
#define RCC_PLLI2SCFGR_RST_VALUE           0x20003010U

#define RCC_MAX_FREQUENCY           100000000U         /*!< Max frequency of family in Hz*/
#define RCC_MAX_FREQUENCY_SCALE1    RCC_MAX_FREQUENCY  /*!< Maximum frequency for system clock at power scale1, in Hz */
#define RCC_MAX_FREQUENCY_SCALE2     84000000U         /*!< Maximum frequency for system clock at power scale2, in Hz */
#define RCC_MAX_FREQUENCY_SCALE3     64000000U         /*!< Maximum frequency for system clock at power scale3, in Hz */
#define RCC_PLLVCO_OUTPUT_MIN       100000000U       /*!< Frequency min for PLLVCO output, in Hz */
#define RCC_PLLVCO_INPUT_MIN           950000U       /*!< Frequency min for PLLVCO input, in Hz  */
#define RCC_PLLVCO_INPUT_MAX          2100000U       /*!< Frequency max for PLLVCO input, in Hz  */
#define RCC_PLLVCO_OUTPUT_MAX       432000000U       /*!< Frequency max for PLLVCO output, in Hz */

#define RCC_PLLN_MIN_VALUE                 50U
#define RCC_PLLN_MAX_VALUE                432U

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#define IRQInterruptConfig(IRQNumber,En_Dis)  (En_Dis)?((IRQNumber <= 31)? *NVIC_ISER0 |= ( 1 << IRQNumber ): (IRQNumber > 31 && IRQNumber < 64 )?*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) ):*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) )):((IRQNumber <= 31)? *NVIC_ICER0 |= ( 1 << IRQNumber ): (IRQNumber > 31 && IRQNumber < 64 )? *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) ): *NVIC_ICER2 |= ( 1 << (IRQNumber % 64) ))


#endif /* __STM32F411xE_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
