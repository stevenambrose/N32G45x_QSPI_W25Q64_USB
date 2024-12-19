/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g45x.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_init.h"
#include "w25q64.h"
#include "mass_mal.h"
#include "stdio.h"
#include "string.h"
uint32_t buf1[1024],buf2[1024];
void usart_init(void);
/*******************************************************************************
 * Function Name  : main.
 * Description    : main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
int main(void)
{
    uint32_t i;
    Set_System();
    
    /* MAL configuration */
    MAL_Init(0);
    
    usart_init();
    
//    for(i=0;i<1024;i++) buf1[i]=i+0X10000000;
//    for(i=0;i<1024;i++) buf2[i]=i+0XF0000000;
//    
//    QspiFlashErase(0x20,0x000000);
//    W25Q64_BufferWrite(buf1,0,512);
//    QspiFlashErase(0x20,0x1000);
//    QspiFlashErase(0x20,0x2000);
//    QspiFlashErase(0x20,0x3000);
//    QspiFlashErase(0x20,0x004000); 
//    W25Q64_BufferWrite(buf2,0x4000,512);
//    QspiFlashErase(0x20,0x5000);
//    QspiFlashErase(0x20,0x6000);
//    QspiFlashErase(0x20,0x7000);
//    QspiFlashErase(0x20,0x8000);
//    QspiFlashErase(0x20,0x9000);
//    QspiFlashErase(0x20,0xA000); 
//    QspiFlashErase(0x20,0xB000);
//    
//    memset(buf1,0,sizeof(buf1));
//    memset(buf2,0,sizeof(buf2));
//    QspiFlashRead(0x0000,buf1,1024);
//    QspiFlashRead(0x4000,buf2,1024);
//    printf("read_addr:0x00\n\r");
//    for(i=0;i<512;i++)
//    {
//        printf(" 0x%08x ",buf1[i]);
//    }
//    printf("\n\r");
//    printf("\n\r");
//    printf("read_addr:0x4000\n\r");
//    for(i=0;i<512;i++)
//    {
//        printf(" 0x%08x ",buf2[i]);
//    }
//    printf("\n\r");
//    printf("\n\r");

    USB_Interrupts_Config();

    Set_USBClock();

    USB_Init(); 
    
    

    while (bDeviceState != CONFIGURED);
    
    
    
    
    
    while (1)
    {
            
    }
        
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
#define USARTx          USART1
#define USARTx_GPIO     GPIOA
#define USARTx_CLK      RCC_APB2_PERIPH_USART1
#define USARTx_GPIO_CLK RCC_APB2_PERIPH_GPIOA
#define USARTx_RxPin    GPIO_PIN_10
#define USARTx_TxPin    GPIO_PIN_9

#define GPIO_APBxClkCmd  RCC_EnableAPB2PeriphClk
#define USART_APBxClkCmd RCC_EnableAPB2PeriphClk

void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin        = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);

    /* Configure USARTx Rx as input floating */
    GPIO_InitStructure.Pin       = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);
}

void usart_init(void)
{
    USART_InitType USART_InitStructure;
    /* Enable GPIO clock */
    GPIO_APBxClkCmd(USARTx_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    /* Enable USARTy and USARTz Clock */
    USART_APBxClkCmd(USARTx_CLK, ENABLE);
    GPIO_Configuration();
    /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);
    /* Enable the USARTx */
    USART_Enable(USARTx, ENABLE);
    printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    USART_SendData(USARTx, (uint8_t)ch);
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) == RESET)
        ;

    return (ch);
}

