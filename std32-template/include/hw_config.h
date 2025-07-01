/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  link
  * @version V1.0.0
  * @date    01-07-2025
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"


void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);

#endif  /*__HW_CONFIG_H*/
