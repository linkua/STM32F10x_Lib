/**
  ******************************************************************************
  * @file    main.c
  * @author  link
  * @version V1.0.0
  * @date    01-07-2025
  * @brief   std template main file
  ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    RCC_Configuration();
    SystemCoreClockUpdate();

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    GPIO_Configuration();
    TIM_Configuration();

    /* 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);

    while (1)
    {
    }
}
