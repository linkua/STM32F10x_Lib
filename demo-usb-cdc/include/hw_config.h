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
#include "usb_lib.h"

void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USART_Config_Default(void);
bool USART_Config(void);
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes);
void USART_To_USB_Send_Data(void);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void VCP_RX_DMA_Channel_ISR(void);
void VCP_SendRxBufPacketToUsb(void);
void VCP_Data_InISR(void);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);

#endif  /*__HW_CONFIG_H*/
