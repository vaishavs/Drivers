/*
 * rcc.cpp
 *
 *  Created on: 01-Jul-2022
 *      Author: Vaishnavi
 */


#include "main.h"
#include "rcc.h"

uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t GetPLLClock()
{
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclockfreq = 0U;

  /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
     SYSCLK = PLL_VCO / PLLP */
  pllm = RCC->PLLCFGR[RCC_PLLCFGR_PLLM_Pos];

  if(RCC->PLLCFGR[RCC_PLLCFGR_PLLSRC_Pos] != RCC_PLLCFGR_PLLSRC_HSI)
  {
     // HSE used as PLL clock source
    uint32_t plln = read_value(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, RCC_PLLCFGR_PLLN_Pos);
     pllvco = (uint32_t) ((uint64_t) 8000000U * ((uint64_t) plln) / (uint64_t)pllm);
  }
  else
  {
     // HSI used as PLL clock source
    uint32_t plln = read_value(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, RCC_PLLCFGR_PLLN_Pos);
     pllvco = (uint32_t) ((uint64_t) 16000000U * ((uint64_t) plln) / (uint64_t)pllm);
  }
  pllp = ((read_value(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);

  sysclockfreq = pllvco/pllp;

  return sysclockfreq;
}

uint32_t GetSysClockFreq()
{
  uint32_t SystemClock;
  uint8_t clksrc;

  clksrc = READ_BITS(RCC->CFGR, RCC_CFGR_SWS); //Read bits 2 and 3

  if(clksrc == RCC_CFGR_SWS_HSI) //HSI
    SystemClock = 16000000;
  else if(clksrc == RCC_CFGR_SWS_HSE) //HSE
    SystemClock = 8000000;
  else if(clksrc == RCC_CFGR_SWS_PLL) //HSE
    SystemClock = GetPLLClock();

  return SystemClock;
}

uint32_t GetHClock()
{
  uint32_t temp = read_value(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_Pos);
  return (GetSysClockFreq() >> AHBPrescTable[temp]);
}

uint32_t GetPClock1()
{
  uint32_t temp = read_value(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_Pos);
  return (GetHClock() >> APBPrescTable[temp]);
}

uint32_t GetPClock2()
{
  uint32_t temp = read_value(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_Pos);
  return (GetHClock() >> APBPrescTable[temp]);
}
