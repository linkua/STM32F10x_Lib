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
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"


/* Private typedef -----------------------------------------------------------*/
/* Communication boards USART Interface */
/* Private define ------------------------------------------------------------*/
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

#define RCC_APB2Periph_ALLGPIO              (RCC_APB2Periph_GPIOA \
                                               | RCC_APB2Periph_GPIOB \
                                               | RCC_APB2Periph_GPIOC )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;

EXTI_InitTypeDef EXTI_InitStructure;

#ifdef VCP_RX_BY_DMA
DMA_InitTypeDef  DMA_InitStructure;
#endif

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef VCP_RX_BY_DMA
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
#endif

    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32xxx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32xxx.c file
       */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /********************************************/
    /*  Configure USB DM/DP pins                */
    /********************************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Enable all GPIOs Clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, ENABLE);

    /********************************************/
    /* Enable the USB PULL UP                   */
    /********************************************/
    /* USB_DISCONNECT used as USB pull-up */

#ifdef USB_LOW_PWR_MGMT_SUPPORT

    /**********************************************************************/
  /*  Configure the EXTI line 18 connected internally to the USB IP     */
  /**********************************************************************/

  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

#endif  /* USB_LOW_PWR_MGMT_SUPPORT */

#ifdef VCP_RX_BY_DMA
    /********************************************************************************/
  /*      Configure Timer. It will be used (instead of SOF) to check USART buffer */
  /********************************************************************************/

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 348;/* This will be changed when the COM port is configured */
  TIM_TimeBaseStructure.TIM_Prescaler = 31;// 32 - 1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
#endif
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->Current_Configuration != 0)
    {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
    else
    {
        bDeviceState = ATTACHED;
    }
    /*Enable SystemCoreClock*/
    SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(uint32_t*)ID1;
    Device_Serial1 = *(uint32_t*)ID2;
    Device_Serial2 = *(uint32_t*)ID3;

    Device_Serial0 += Device_Serial2;

    if (Device_Serial0 != 0)
    {
        IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
        IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
    }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
    uint8_t idx = 0;

    for( idx = 0 ; idx < len ; idx ++)
    {
        if( ((value >> 28)) < 0xA )
        {
            pbuf[ 2* idx] = (value >> 28) + '0';
        }
        else
        {
            pbuf[2* idx] = (value >> 28) + 'A' - 10;
        }

        value = value << 4;

        pbuf[ 2* idx + 1] = 0;
    }
}

/*******************************************************************************
 * RCC Configuration
*******************************************************************************/
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
