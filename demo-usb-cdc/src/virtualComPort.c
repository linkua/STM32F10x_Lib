/**
  ******************************************************************************
  * @file    virtualComPort.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Virtual Com Port Configuration
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include <string.h>
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_RX_DATA_SIZE   512

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
extern DMA_InitTypeDef  DMA_InitStructure;

uint8_t  USART_Rx_Buffer[USART_RX_DATA_SIZE];
uint16_t USB_Tx_length;
uint16_t USB_Tx_ptr;

uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;
uint8_t  USB_Tx_State = 0;

/* Extern variables ----------------------------------------------------------*/
extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : VCP_Data_InISR.
* Description    : EPxIN USB transfer complete ISR. Send pending data if any.
* Input          : None.
* Return         : none.
*******************************************************************************/
void VCP_Data_InISR(void)
{
    VCP_SendRxBufPacketToUsb();
}

/*******************************************************************************
* Function Name  : DMA_COMInit.
* Description    : Global initialization of Virtual Com Port.
* Input          : USART_InitStruct: Pointer to the structure defining the USART
*                : configuration for the Virtual Com Port.
* Return         : none.
*******************************************************************************/
void DMA_COMInit(USART_InitTypeDef* USART_InitStruct)
{
    /* Re-initialize global variables */
    USART_Rx_ptr_in = 0;
    USART_Rx_ptr_out = 0;
    USART_Rx_length  = 0;
    USB_Tx_State = 0;

    GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Enable USARTx clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, USART_InitStruct);
  
  /* Reinit the buffer */
  memset(USART_Rx_Buffer, 0, USART_RX_DATA_SIZE);
}

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg COM1
  *     @arg COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(uint8_t COM, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART1, USART_InitStruct);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
}


/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the USART1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
    /* USART1 default configuration */
    /* USART1 configured as follow:
          - BaudRate = 9600 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - Parity Odd
          - Hardware flow control disabled
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Odd;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure and enable the USART */
    STM_EVAL_COMInit(0, &USART_InitStructure);
    /* Enable the USART Receive interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*******************************************************************************
* Function Name  :  USART_Config.
* Description    :  Configure the USART1 according to the line coding structure.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
bool USART_Config(void)
{
    /* set the Stop bit*/
    switch (linecoding.format)
    {
        case 0:
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
            break;
        case 1:
            USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
            break;
        case 2:
            USART_InitStructure.USART_StopBits = USART_StopBits_2;
            break;
        default :
        {
            USART_Config_Default();
            return (FALSE);
        }
    }

    /* set the parity bit*/
    switch (linecoding.paritytype)
    {
        case 0:
            USART_InitStructure.USART_Parity = USART_Parity_No;
            break;
        case 1:
            USART_InitStructure.USART_Parity = USART_Parity_Even;
            break;
        case 2:
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
            break;
        default :
        {
            USART_Config_Default();
            return (FALSE);
        }
    }

    /*set the data type : only 8bits and 9bits is supported */
    switch (linecoding.datatype)
    {
        case 0x07:
            /* With this configuration a parity (Even or Odd) should be set */
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            break;
        case 0x08:
            if (USART_InitStructure.USART_Parity == USART_Parity_No)
            {
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            }
            else
            {
                USART_InitStructure.USART_WordLength = USART_WordLength_9b;
            }

            break;
        default :
        {
            USART_Config_Default();
            return (FALSE);
        }
    }

    USART_InitStructure.USART_BaudRate = linecoding.bitrate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure and enable the USART */
    STM_EVAL_COMInit(0, &USART_InitStructure);

    return (TRUE);
}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes)
{
    uint32_t i;

    for (i = 0; i < Nb_bytes; i++)
    {
        USART_SendData(USART1, *(data_buffer + i));
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

/*******************************************************************************
* Function Name  : VCP_SendRxBufPacketToUsb.
* Description    : send data from USART_Rx_Buffer to the USB. Manage the segmentation
*                  into USB FIFO buffer. Commit one packet to the USB at each call.
* Input          : globals:
*                  - USB_Tx_State: transmit state variable
*                  - USART_Rx_Buffer: buffer of data to be sent
*                  - USART_Rx_length: amount of data (in bytes) ready to be sent
*                  - USART_Rx_ptr_out: index in USART_Rx_Buffer of the first data
*                    to send
* Return         : none.
*******************************************************************************/
void VCP_SendRxBufPacketToUsb(void)
{
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State == 1)
    {
        if (USART_Rx_length == 0)
        {
            USB_Tx_State = 0;
        }
        else
        {
            if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE){
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

                USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
                USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
            }
            else
            {
                USB_Tx_ptr = USART_Rx_ptr_out;
                USB_Tx_length = USART_Rx_length;

                USART_Rx_ptr_out += USART_Rx_length;
                USART_Rx_length = 0;
            }

            UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
            SetEPTxCount(ENDP1, USB_Tx_length);
            SetEPTxValid(ENDP1);
        }
    }
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
    if(USB_Tx_State != 1)
    {
        if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
        {
            USART_Rx_ptr_out = 0;
        }

        if(USART_Rx_ptr_out == USART_Rx_ptr_in)
        {
            USB_Tx_State = 0;
            return;
        }

        if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
        {
            USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
        }
        else
        {
            USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
        }

        if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

            USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
            USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
        }
        else
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = USART_Rx_length;

            USART_Rx_ptr_out += USART_Rx_length;
            USART_Rx_length = 0;
        }
        USB_Tx_State = 1;

        UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
        SetEPTxCount(ENDP1, USB_Tx_length);
        SetEPTxValid(ENDP1);
    }
}

/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{
    if (linecoding.datatype == 7)
    {
        USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(USART1) & 0x7F;
    }
    else if (linecoding.datatype == 8)
    {
        USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(USART1);
    }

    USART_Rx_ptr_in++;

    /* To avoid buffer overflow */
    if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
    {
        USART_Rx_ptr_in = 0;
    }
}
