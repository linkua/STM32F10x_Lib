/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  link
  * @version V1.0.0
  * @date    01-07-2025
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

void RCC_Configuration(void)
{
    /* Reset RCC configuration to default */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    ErrorStatus HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer and set Flash latency */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* PLLCLK = HSE * 9 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait until PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08);
    }
    else
    {
        /* HSE failed to start-up, handle error here */
        while (1);
    }
}

/* 配置PC13为推挽输出 */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* 配置PC13为通用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 默认LED熄灭 (注意：PC13低电平点亮) */
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* 配置TIM3为1秒定时 */
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM3 配置 */
    /* 假设系统时钟为72MHz */
    /* 定时器时钟 = 72MHz / 7200 = 10kHz */
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;  /* 10000个时钟周期 = 1秒 */
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* 使能TIM3中断 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* 配置NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* TIM3中断处理函数 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        /* 清除中断标志 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        /* 翻转PC13状态 */
        GPIOC->ODR ^= GPIO_Pin_13;
    }
}
