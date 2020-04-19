/*
 * dw_stm32_delay.h
 *
 *  Created on: Mar 11, 2020
 *      Author: suleyman.eskil
 */

#ifndef INC_DW_STM32_DELAY_H_
#define INC_DW_STM32_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdlib.h>
#ifndef __STM32L4xx_HAL_H
#include "stm32l4xx_hal.h"
#endif

/**
 * @brief  Initializes DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: DWT counter Error
 *         0: DWT counter works
 */
uint32_t DWT_Delay_Init(void);



/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}


#ifdef __cplusplus
}
#endif


#endif /* INC_DW_STM32_DELAY_H_ */
