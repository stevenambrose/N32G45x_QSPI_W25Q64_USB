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
#include "ff.h"
#include "ffconf.h"

FATFS fs;
FIL fnew;
FRESULT res_flash;
UINT fnum;
BYTE ReadBuffer[4096] = {0};
BYTE WriteBuffer[] =            /* 写缓冲区*/
    " this is a file system testing";
void usart_init(void);

void DFU_enter(void)
{
    GPIO_InitType DFU_IO;
    GPIO_InitStruct(&DFU_IO);
    
    DFU_IO.GPIO_Mode = GPIO_Mode_IPU;
    DFU_IO.Pin      =GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOA,&DFU_IO);
    
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_6)==RESET)
    {
    /* MAL configuration */
        MAL_Init(0);

        USB_Interrupts_Config();

        Set_USBClock();

        USB_Init();

        while (bDeviceState != CONFIGURED);
    }
}


void FatFs_Test(void)
{
    uint32_t i;
    BYTE work[FF_MAX_SS];
    MKFS_PARM opt =
    {
        .fmt = FM_FAT | FM_SFD,
        .n_fat = 1,
        .align = 0,
        .n_root = 0,
        .au_size = 0
    };

    res_flash = f_mount(&fs, "0:", 1);

    if (res_flash == FR_NO_FILESYSTEM)
    {
        printf("》FLASH还没有文件系统，即将进行格式化...\r\n");

        res_flash = f_mkfs("0:", &opt, work, sizeof(work));

        if (res_flash == FR_OK)
        {
            printf("》FLASH已成功格式化文件系统。\r\n");
            res_flash = f_mount(NULL, "0:", 1);
            res_flash = f_mount(&fs, "0:", 1);

            if (res_flash == FR_OK)
            {
                printf("格式化后挂载成功\r\n");
            }
            else
            {
                printf("格式化后挂载失败, err = %d\r\n", res_flash);
            }
        }
        else
        {
            printf("《《格式化失败。》》\r\n");
            printf("errcode = %d\r\n", res_flash);

            while (1);
        }
    }
    else if (res_flash != FR_OK)
    {
        printf("！！FatFs mount fail(%d)\r\n", res_flash);
        printf("！！Maybe：SPI Flash init fail。\r\n");
        printf("请下载 SPI—读写串行FLASH 例程测试，如果正常，在该例程f_mount语句下if语句前临时多添加一句 res_flash = FR_NO_FILESYSTEM; 让重新直接执行格式化流程\r\n");

        while (1);
    }
    else
    {
        printf("》FatFs mount successful\r\n");
    }

    //    /*----------------------- 文件系统测试：写测试 -------------------*/
    //  /* 打开文件，每次都以新建的形式打开，属性为可写 */
    //  printf("\r\n****** Write testing... ******\r\n");
    //  res_flash = f_open(&fnew, "0:/test1.txt",FA_CREATE_ALWAYS | FA_WRITE );
    //  printf("/******************************************************/\r\n");
    //  if ( res_flash == FR_OK )
    //  {
    //      printf("》open creat successful,ready to write.\r\n");
    //      /* 将指定存储区内容写入到文件内 */
    //      res_flash=f_write(&fnew,WriteBuffer,sizeof(WriteBuffer),&fnum);
    //      if(res_flash == FR_OK)
    //      {
    //        printf("》fail write successful,len：%d\n",fnum);
    //        printf("》writing data is ：\r\n%s\r\n",WriteBuffer);
    //      }
    //      else
    //      {
    //        printf("！！fail write fail：(%d)\n",res_flash);
    //      }
    //          /* 不再读写，关闭文件 */
    //      f_close(&fnew);
    //  }
    //  else
    //  {
    //      printf("！！open/creat fail.\r\n");
    //  }

    /*----------------------- 文件系统测试：读测试 -------------------*/
    printf("\r\n****** ready for file reading... ******\r\n");
    res_flash = f_open(&fnew, "0:/Oscilloscope.bin", FA_OPEN_EXISTING | FA_READ);
    DWORD fileSize = f_size(&fnew);
    printf("！！file size：(%d)\n", fileSize);

    if (res_flash == FR_OK)
    {
        printf("》open successful.\r\n");

        res_flash = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum);

        if (res_flash == FR_OK)
        {
            //          printf("》file read succesul,reading data len ：%d\r\n",fnum);
            for (i = 0; i < fnum; i++)
            {
                printf(" %02x ", ReadBuffer[i]);
            }

            printf("\n\r");

        }
        else
        {
            printf("！！file reading fail：(%d)\n", res_flash);
        }
    }
    else
    {
        printf("！！open file failure.\r\n");
    }

    /* 不再读写，关闭文件 */
    f_close(&fnew);

    /* 不再使用文件系统，取消挂载文件系统 */

    f_mount(NULL, "0:", 1);

}







/*******************************************************************************
 * Function Name  : main.
 * Description    : main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
int main(void)
{

    Set_System();

    usart_init();
    
    DFU_enter();
    
    
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

