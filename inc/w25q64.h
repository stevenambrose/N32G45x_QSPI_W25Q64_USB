/*******************************************************************************
* @FileName    : w25q64.h
* @Tool-Chain  : MDK(armcc)
* @Author      : Ambrose
* @Version     : 0.0.1
* @EditionDate : 2024/12/19
* @note        : w25q64+QSPI驱动
*******************************************************************************/
#ifndef __W25Q64_H
#define __W25Q64_H

#include "n32g45x.h"
#include "n32g45x_qspi.h"

#define W25Q64_PAGESIZE        256      //每页256字节
#define W25Q64_ID              0XEF4017    //W25Q64

 /* 复位操作 */
 #define RESET_ENABLE_CMD                     0x66
 #define RESET_MEMORY_CMD                     0x99

 #define ENTER_QPI_MODE_CMD                   0x38
 #define EXIT_QPI_MODE_CMD                    0xFF

 /* 识别操作 */
 #define READ_ID_CMD                          0x90
 #define DUAL_READ_ID_CMD                     0x92
 #define QUAD_READ_ID_CMD                     0x94
 #define READ_JEDEC_ID_CMD                    0x9F

 /* 读操作 */
 #define READ_CMD                             0x03
 #define FAST_READ_CMD                        0x0B
 #define DUAL_OUT_FAST_READ_CMD               0x3B
 #define DUAL_INOUT_FAST_READ_CMD             0xBB
 #define QUAD_OUT_FAST_READ_CMD               0x6B
 #define QUAD_INOUT_FAST_READ_CMD             0xEB

 /* 写操作 */
 #define WRITE_ENABLE_CMD                     0x06
 #define WRITE_DISABLE_CMD                    0x04

 /* 寄存器操作 */
 #define READ_STATUS_REG1_CMD                  0x05
 #define READ_STATUS_REG2_CMD                  0x35
 #define READ_STATUS_REG3_CMD                  0x15

 #define WRITE_STATUS_REG1_CMD                 0x01
 #define WRITE_STATUS_REG2_CMD                 0x31
 #define WRITE_STATUS_REG3_CMD                 0x11


 /* 编程操作 */
 #define PAGE_PROG_CMD                        0x02
 #define QUAD_INPUT_PAGE_PROG_CMD             0x32
 #define EXT_QUAD_IN_FAST_PROG_CMD            0x12

 /* 擦除操作 */
 #define SECTOR_ERASE_CMD                     0x20
 #define CHIP_ERASE_CMD                       0xC7

 #define PROG_ERASE_RESUME_CMD                0x7A
 #define PROG_ERASE_SUSPEND_CMD               0x75

#define QSPI_CLK            RCC_AHB_PERIPH_QSPI
#define QSPI_GPIO_CLK       RCC_APB2_PERIPH_GPIOC|RCC_APB2_PERIPH_GPIOD|RCC_APB2_PERIPH_AFIO
#define QSPI_GPIO_REMAP     GPIO_RMP3_QSPI
#define QSPI_CS_PIN         GPIO_PIN_10
#define QSPI_CLK_PIN        GPIO_PIN_11
#define QSPI_IO0_PIN        GPIO_PIN_12
#define QSPI_IO1_PIN        GPIO_PIN_0
#define QSPI_IO2_PIN        GPIO_PIN_1
#define QSPI_IO3_PIN        GPIO_PIN_2

#define TEST_ADDR                       0x000000


ErrorStatus W25Q64_Init(void);
void QSPI_Test(void);
uint32_t W25Q64_ReadID(void);
void QspiFlashErase(uint32_t EraseCmd, uint32_t ErsAddr);
void QspiFlashRead(uint32_t addr, uint8_t *buf, uint32_t size);
void QspiFlashProgram(uint32_t PrgAddr, uint32_t *pAddr, uint32_t cnt);
void W25Q64_PageWrite(uint32_t PrgAddr, uint8_t *pAddr, uint32_t cnt);
void W25Q64_BufferWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite);
void QspiFlashChipErase(void);
void Qspi_Init(QSPI_FORMAT_SEL qspi_format_sel, QSPI_DATA_DIR data_dir, uint16_t count);



#endif
