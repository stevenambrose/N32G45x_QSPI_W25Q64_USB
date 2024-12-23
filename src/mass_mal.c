/*******************************************************************************
* @FileName    : mass_mal.c
* @Tool-Chain  : MDK(armcc)
* @Author      : Ambrose
* @Version     : 0.0.1
* @EditionDate : 2024/12/19
* @note        : n32g45x USB Read and Write
*******************************************************************************/

/**
 * @file mass_mal.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
/* Includes ------------------------------------------------------------------*/


#include "mass_mal.h"
#include "n32g45x_flash.h"
#include "w25q64.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE 4096        //4k
#define FLASH_SIZE      4*1024

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];
uint32_t READADDR,WADDR;
/* logic unit count; the first is 0 */
uint32_t Max_Lun = 0;

/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the Nations
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
    W25Q64_Init();
    if(W25Q64_ReadID() != 0X00EF4017)
        return 0;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{
    uint16_t i;
    switch (lun)
    {
    case 0: 
        WADDR=Memory_Offset;
//    printf("\n\r");
//        printf("write_addr:0x%x\n\r",WADDR);
//        printf("\n\r");
        QspiFlashErase(0x20,Memory_Offset+FLASH_OFFSET);
        W25Q64_BufferWrite((uint8_t *)Writebuff,Memory_Offset+FLASH_OFFSET,Transfer_Length);
//        for(i=0;i<32;i++)
//        {
//            printf(" 0x%x ",Writebuff[i]);
//        }
//            printf("\n\r");
     /*  for( i = 0; i < Transfer_Length; i += FLASH_PAGE_SIZE )
       { 
          if( FLASH_WaitForLastOpt(FLASH_WAIT_TIMEOUT) != FLASH_TIMEOUT )
          {
             FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);
          }     
          FLASH_EraseOnePage(FLASH_START_ADDR + Memory_Offset + i); 
       }            
         
       for( i = 0; i < Transfer_Length; i+=4 )
       { 
           if( FLASH_WaitForLastOpt(FLASH_WAIT_TIMEOUT) != FLASH_TIMEOUT )
           {
              FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR); 
           } 
           FLASH_ProgramWord(FLASH_START_ADDR + Memory_Offset + i , Writebuff[i>>2]); 
       }*/

      break;
    case 1:
      break;
    default:
      return MAL_FAIL;
    }
    return MAL_OK;
}
//uint32_t buffer[4096];
/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{
    uint16_t i;
    
    switch (lun)
    {
        case 0:
//            READADDR=Memory_Offset;         
//            QspiFlashRead(READADDR,Readbuff,Transfer_Length);
            QspiFlashRead(Memory_Offset+FLASH_OFFSET,(uint8_t *)Readbuff,Transfer_Length);
//            printf("read_addr:0x%x\n\r",READADDR);
//            printf("read_lenth:%d\n\r",Transfer_Length);
//            for(i=0;i<8;i++)
//            {
//                printf(" 0x%x ",Readbuff[i]);
//            }
//            printf("\n\r");
/*          for( i=0; i < Transfer_Length; i+=4 )
          {
             Readbuff[i>>2] = ((vu32*)(FLASH_START_ADDR + Memory_Offset))[i>>2]; 
          }*/
          break;
        case 1:
          break;
        default:
          return MAL_FAIL;
    }
    return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
    if (lun == 0)
    {
        Mass_Block_Count[0] = 1024; 
        Mass_Block_Size[0] =  4096; 
        Mass_Memory_Size[0] = Mass_Block_Size[0]*Mass_Block_Count[0];
        return MAL_OK;
    }
    return MAL_FAIL;
}

