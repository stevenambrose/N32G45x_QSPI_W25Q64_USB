/*******************************************************************************
* @FileName    : w25q64.c
* @Tool-Chain  : MDK(armcc)
* @Author      : Ambrose
* @Version     : 0.0.1
* @EditionDate : 2024/12/19
* @note        : w25q64+QSPI驱动
*******************************************************************************/
#include "w25q64.h"
#include "n32g45x_qspi.h"
#include "n32g45x_gpio.h"

static void QspiFlashWriteEnable(void);
static uint32_t WaitQSPI_Busy(void);
static void Delayus(uint32_t count);
static void QspiFlashReadStatus(void);
/**
  * @brief  设置QSPI的格式
  * @param qspi_format_sel Select format of QSPI.
                        STANDARD_SPI_FORMAT_SEL:标准SPI
                        DUAL_SPI_FORMAT_SEL:双输出SPI
                        QUAD_SPI_FORMAT_SEL:四线SPI
                        XIP_SPI_FORMAT_SEL: 内存映射模式
  * @param data_dir The direction of transferring data.
                        TX_AND_RX:发送接收
                        RX_ONLY:只接收.
                        TX_ONLY:只发送.
  * @param count Number of data frames. It is valid only in RX_ONLY mode of DUAL_SPI_FORMAT_SEL or QUAD_SPI_FORMAT_SEL.
  */
void Qspi_Init(QSPI_FORMAT_SEL qspi_format_sel, QSPI_DATA_DIR data_dir, uint16_t count)
{
    QSPI_InitType QSPI_Struct = {0};

    switch (qspi_format_sel)
    {
        case STANDARD_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_NSS_PORTC_SEL, 0, 1);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(QSPI_CLK, ENABLE);
            QSPI_Struct.SPI_FRF = QSPI_CTRL0_SPI_FRF_STANDARD_FORMAT; // 设置为标准模式
            QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_TX_AND_RX;             // 发送接收
            QSPI_Struct.SCPOL = QSPI_CTRL0_SCPOL_HIGH;                // 空闲时钟高
            QSPI_Struct.SCPH = QSPI_CTRL0_SCPH_SECOND_EDGE;           // 采集相位
            QSPI_Struct.DFS = QSPI_CTRL0_DFS_8_BIT;                   // 数据量大小
            QSPI_Struct.CLK_DIV = 4;                                  // 144/4=36M
            QSPI_Struct.TXFT = QSPI_TXFT_TEI_0;                       // 发送fifo阈值
            QSPI_Struct.RXFT = QSPI_RXFT_TFI_0;                       // 接收fifo阈值
            QSPI_Struct.NDF = 1024;                                   // 数据帧长度

            QspiInitConfig(&QSPI_Struct);
            QSPI_Cmd(ENABLE);
            break;

        case DUAL_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_NSS_PORTC_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(QSPI_CLK, ENABLE);           // 启动时钟
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE); // 不使用XIP

            QSPI_Struct.SPI_FRF = QSPI_CTRL0_SPI_FRF_DUAL_FORMAT; // 双线SPI帧模式

            if (data_dir == TX_ONLY)
            {
                QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_TX_ONLY; // 只有标准SPI模式才允许同时接收发送
                QSPI_Struct.NDF = 0;                        // 数据帧长度
                QSPI_Struct.ENHANCED_WAIT_CYCLES = 0;       // 等待周期
            }
            else if (data_dir == RX_ONLY)
            {
                QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_RX_ONLY;                     // 接收模式
                QSPI_Struct.NDF = count;                                        // 接收数据帧
                QSPI_Struct.ENHANCED_WAIT_CYCLES = QSPI_ENH_CTRL0_WAIT_8CYCLES; // 等待周期
            }

            QSPI_Struct.CFS = QSPI_CTRL0_CFS_8_BIT;         // 帧长度
            QSPI_Struct.SCPOL = QSPI_CTRL0_SCPOL_HIGH;      // 空闲时钟相位
            QSPI_Struct.SCPH = QSPI_CTRL0_SCPH_SECOND_EDGE; // 时钟相位
            QSPI_Struct.FRF = QSPI_CTRL0_FRF_MOTOROLA;      // 帧格式
            QSPI_Struct.DFS = QSPI_CTRL0_DFS_8_BIT;         // 数据帧长度
            QSPI_Struct.CLK_DIV = 4;                        // （144/4）*2=72M
            QSPI_Struct.TXFT = QSPI_TXFT_TEI_0;             // 发送fifo阈值
            QSPI_Struct.RXFT = QSPI_RXFT_TFI_0;             // 接收fifo阈值

            QSPI_Struct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; // 时钟拉伸
            QSPI_Struct.ENHANCED_ADDR_LEN = QSPI_ENH_CTRL0_ADDR_LEN_24_BIT;      // 传输的地址长度
            QSPI_Struct.ENHANCED_INST_L = QSPI_ENH_CTRL0_INST_L_8_LINE;          // 指令长度

            QspiInitConfig(&QSPI_Struct);
            QSPI_Cmd(ENABLE);
            break;

        case QUAD_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_NSS_PORTC_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE); // enable clock of qspi
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE);  // disable XIP

            QSPI_Struct.SPI_FRF = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT; // 四线QSPI格式

            if (data_dir == TX_ONLY)
            {
                QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_TX_ONLY; // TX transfer mode
                QSPI_Struct.NDF = 0;                        // number of data frames
                QSPI_Struct.ENHANCED_WAIT_CYCLES = 0;       // wait cycles of dummy
            }
            else if (data_dir == RX_ONLY)
            {
                QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_RX_ONLY;                     // RX transfer mode
                QSPI_Struct.NDF = count;                                        // number of data frames
                QSPI_Struct.ENHANCED_WAIT_CYCLES = QSPI_ENH_CTRL0_WAIT_8CYCLES; // wait cycles of dummy
            }

            QSPI_Struct.CFS = QSPI_CTRL0_CFS_8_BIT;         // control frame size
            QSPI_Struct.SCPOL = QSPI_CTRL0_SCPOL_HIGH;      // serial clock polarity
            QSPI_Struct.SCPH = QSPI_CTRL0_SCPH_SECOND_EDGE; // serial clock phase
            QSPI_Struct.FRF = QSPI_CTRL0_FRF_MOTOROLA;      // frame format
            QSPI_Struct.DFS = QSPI_CTRL0_DFS_8_BIT;         // data frame size
            QSPI_Struct.CLK_DIV = 4;                        // 144/4*4=144
            QSPI_Struct.TXFT = QSPI_TXFT_TEI_0;             // transmit fifo threshold
            QSPI_Struct.RXFT = QSPI_RXFT_TFI_0;             // receive fifo threshold

            QSPI_Struct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; // enable stretch
            QSPI_Struct.ENHANCED_ADDR_LEN = QSPI_ENH_CTRL0_ADDR_LEN_24_BIT;      // length of address to transmit
            QSPI_Struct.ENHANCED_INST_L = QSPI_ENH_CTRL0_INST_L_8_LINE;          // instruction length

            QspiInitConfig(&QSPI_Struct);
            QSPI_Cmd(ENABLE);
            break;

        case XIP_SPI_FORMAT_SEL:
            QSPI_GPIO(QSPI_NSS_PORTC_SEL, 0, 0);

            QSPI_DeInit();
            RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_QSPI, ENABLE); // enable clock of qspi
            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, DISABLE);  // disable XIP

            QSPI_Struct.SPI_FRF = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT; // quad SPI frame format
            QSPI_Struct.TMOD = QSPI_CTRL0_TMOD_RX_ONLY;           // transfer mode
            QSPI_Struct.NDF = count;                              // number of data frames
            QSPI_Struct.CFS = QSPI_CTRL0_CFS_8_BIT;               // control frame size
            QSPI_Struct.SCPOL = QSPI_CTRL0_SCPOL_HIGH;            // serial clock polarity
            QSPI_Struct.SCPH = QSPI_CTRL0_SCPH_SECOND_EDGE;       // serial clock phase
            QSPI_Struct.FRF = QSPI_CTRL0_FRF_MOTOROLA;            // frame format
            QSPI_Struct.DFS = QSPI_CTRL0_DFS_8_BIT;               // data frame size
            QSPI_Struct.CLK_DIV = 4;                              // clock divider
            QSPI_Struct.TXFT = QSPI_TXFT_TEI_0;                   // transmit fifo threshold
            QSPI_Struct.RXFT = QSPI_RXFT_TFI_0;                   // receive fifo threshold

            QSPI_Struct.ENHANCED_CLK_STRETCH_EN = QSPI_ENH_CTRL0_CLK_STRETCH_EN; // enable stretch

            QSPI_Struct.XIP_MBL = QSPI_XIP_CTRL_XIP_MBL_LEN_8_BIT; // XIP mode bits length
            QSPI_Struct.XIP_CT_EN = QSPI_XIP_CTRL_XIP_CT_EN;       // enable continuous transfer in XIP mode
            QSPI_Struct.XIP_INST_EN = QSPI_XIP_CTRL_XIP_INST_EN;   // enable XIP instruction
            // QSPI_InitStruct.XIP_DFS_HC    = QSPI_XIP_CTRL_DFS_HC; //Fix DFS for XIP transfer
            QSPI_Struct.XIP_ADDR_LEN = QSPI_XIP_CTRL_ADDR_24BIT;      // length of address to transmit
            QSPI_Struct.XIP_INST_L = QSPI_XIP_CTRL_INST_L_8_LINE;     // instruction length
            QSPI_Struct.XIP_WAIT_CYCLES = QSPI_XIP_CTRL_WAIT_8CYCLES; // wait cycles of dummy
            QSPI_Struct.XIP_FRF = QSPI_XIP_CTRL_FRF_4_LINE;           // frame format
            QSPI_Struct.XIP_MD_BITS = 0xaabb;                         // content of mode stage
            QSPI_Struct.ITOC = QUAD_OUT_FAST_READ_CMD;                // 0X6B = QUAD Read
            QSPI_Struct.WTOC = 0x6b;                                  // WTOC

            QspiInitConfig(&QSPI_Struct);

            /** enable XIP */
            QSPI_XIP_Cmd(ENABLE);
            QSPI_Cmd(ENABLE);

            GPIO_ConfigPinRemap(GPIO_RMP_QSPI_XIP_EN, ENABLE); // enable memory map of XIP mode
            break;

        default:
            break;
    }
}

/**================================================================================
 *      SPI FLASH initial
 ================================================================================*/

ErrorStatus W25Q64_Init(void)
{
    uint32_t temp[4] = {READ_JEDEC_ID_CMD, 0x00, 0x00, 0x00}; // 用于读产品ID号
    uint32_t ID[4];

    Qspi_Init(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, 1024); // 先设置为标准SPI，即D0输出数据，D1接收
    // write enable
    QspiFlashWriteEnable();           // 开启写允许

    // enable QE
    /*W25Q64手册明确可以连续写状态寄存器器，即传输寄存器1地址后可连续写入第1、2、3传感器值*/
    temp[0] = WRITE_STATUS_REG1_CMD;    // 写入传感器1地址
    temp[1] = 0x02;                     // 寄存器1值
    temp[2] = 0x02;                     // 寄存器2写入值，开启QE
    QspiSendAndGetWords(temp, temp, 3); // QSPI总线传输数据
    WaitQSPI_Busy();                    // 等待QSPI空闲
    QspiFlashReadStatus();              // 等待W25Q64返回空闲指令，实际操作通过读取状态寄存器1标志位

    //  read status register1
    temp[0] = READ_STATUS_REG1_CMD;     // 读状态寄存器1值
    temp[1] = 0xff;                     // dummy指令
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();
    QspiFlashReadStatus();              // wait for operating completely

    // read status register2
    temp[0] = READ_STATUS_REG2_CMD;     // 读状态寄存器2值
    temp[1] = 0xff;                     // dummy指令
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();

    if (W25Q64_ReadID() == 0x00ef4017)
        return SUCCESS;
    
    return -1;
}

uint32_t W25Q64_ReadID(void)
{
    uint32_t ID[4] = {READ_JEDEC_ID_CMD, 0x00, 0x00, 0x00}; // 用于读产品ID号;
    uint32_t TID;
    Qspi_Init(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, 1024); // 先设置为标准SPI，即D0输出数据，D1接收
    QspiFlashWriteEnable();           // 开启写允许
    QspiSendAndGetWords(ID, ID, 4); // 读取ID号，返回结果应是0X00,0XEF,0X40,0X17
    TID = (ID[0] << 24) + (ID[1] << 16) + (ID[2] << 8) + (ID[3] & 0XFF);

    return TID;
}

/**================================================================================
        enable Write flash
 ================================================================================*/
static void QspiFlashWriteEnable(void)
{
    u32 temp;

    QspiSendWordAndGetWords(WRITE_ENABLE_CMD, &temp, 1);
    WaitQSPI_Busy();
}

/**================================================================================
Name:   wait for the busy of QSPI
 ================================================================================*/
static uint32_t WaitQSPI_Busy(void)
{
    uint32_t timeout = 0;

    Delayus(5);

    while (GetQspiBusyStatus())
    {
        if (++timeout >= 20)
        {
            break;
        }

        Delayus(50);
    }

    return 0xff;
}

/**================================================================================
        delay function
 ================================================================================*/
static void Delayus(volatile uint32_t count)
{
    for (; count > 0; count--)
        ;   
}

static void m_delay_ms(uint32_t ms)
{
    for (; ms != 0; ms--)
    {
        Delayus(14400);
    }
}

/**================================================================================
        enable QE
 ================================================================================*/
static void QspiFlashEnableQE(void)
{
    uint32_t temp[3] = {READ_STATUS_REG2_CMD, 0xff, 0xff};

    // read status register
    QspiSendAndGetWords(temp, temp, 2);
    WaitQSPI_Busy();

    // enable QE if disable
    if (!(temp[1] & 0x02))
    {
        temp[0] = WRITE_STATUS_REG1_CMD;
        temp[1] = 0x02;
        temp[1] = 0x02;
        QspiSendAndGetWords(temp, temp, 3);
        WaitQSPI_Busy();
        QspiFlashReadStatus(); // wait for operating completely
    }
}

/**================================================================================
        read status register1 of flash, wait for operating completely
 ================================================================================*/
static void QspiFlashReadStatus(void)
{
    uint32_t tx_buf[2] = {READ_STATUS_REG1_CMD, 0x00};
    uint32_t rx_buf[2];
    uint32_t timeout = 0;

    do
    {
        m_delay_ms(1);

        QspiSendAndGetWords(tx_buf, rx_buf, 2);

        if (++timeout >= 200)
        {
            break;
        }
    } while ((rx_buf[1] & 0x01) != 0x00);
}

/**================================================================================
      full chip erase
 ================================================================================*/
void QspiFlashChipErase(void)
{
    uint32_t temp;

    // write enable
    QspiFlashWriteEnable();

    // transmit the erase command
    QspiSendWordAndGetWords(CHIP_ERASE_CMD, &temp, 1); // 0x60 or 0xc7
    WaitQSPI_Busy();
    ClrFifo();
    QspiFlashReadStatus();
}

/**================================================================================
  * @brief  Erase one region of flash. / Flash erase
  * @param  EraseCmd command to select erase unit.
      0x20:erase 4KB sector.
      0x52 erase 32KB block.
      0xD8 erase 64KB block.
  * @param ErsAddr start address of the erase region.
  ================================================================================*/
void QspiFlashErase(uint32_t EraseCmd, uint32_t ErsAddr)
{
    u32 tx_buf[4], rx_buf[4];

    // switch to standard write mode
    Qspi_Init(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, 1024);
    QspiFlashWriteEnable();

    // transmit the erase command
    tx_buf[0] = EraseCmd;
    tx_buf[1] = (ErsAddr & 0xff0000) >> 16;
    tx_buf[2] = (ErsAddr & 0xff00) >> 8;
    tx_buf[3] = ErsAddr & 0xff;
    QspiSendAndGetWords(tx_buf, rx_buf, 4);
    WaitQSPI_Busy();
    ClrFifo();
    QspiFlashReadStatus();
}

/**================================================================================
        QSPI read flash data in quad mode
    In:     addr
        *buf    the data from third byte is valid
        size    valid data length
================================================================================*/
void QspiFlashRead(uint32_t addr, uint8_t *buf, uint32_t size)
{
//    size/=4;
    uint32_t tbuf[4096],i;
    // quad receive, read data
    Qspi_Init(QUAD_SPI_FORMAT_SEL, RX_ONLY, size);

    QspiSendWord(QUAD_OUT_FAST_READ_CMD); // CMD
    QspiSendWord(addr);                   // ADDR 
    Delayus(0x100);    
    GetFifoData(tbuf, size);
    WaitQSPI_Busy();
    for(i=0;i<size;i++) buf[i]=(tbuf[i]&0xff);
}

/**================================================================================
            QSPI page write in quad wire mode
  ================================================================================*/
void W25Q64_PageWrite(uint32_t PrgAddr, uint8_t *pAddr, uint32_t cnt)
{
    uint32_t    i;  
    uint32_t    tx_buf[256+2] = {0};    
    u32 num = 0;
    uint32_t timeout = 0;

    // page write, don't exceed 256 bytes
    if(cnt > 256)   return;

    tx_buf[0] = QUAD_INPUT_PAGE_PROG_CMD;           // cmd---Instruction
    tx_buf[1] = PrgAddr;                            // addr---Address
    for(i=0; i<cnt; i++)                            //     ---data
    {
        tx_buf[i+2] = pAddr[i];
    }
    
    // switch to standard mode, write enable
    Qspi_Init(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, 1024);
    QspiFlashWriteEnable();         
    QspiFlashEnableQE();                
    
    // switch to quad mode, write data 
    Qspi_Init(QUAD_SPI_FORMAT_SEL, TX_ONLY, cnt+2);  
    for (num = 0; num < cnt+2; num++)
    {
        while (GetQspiTxDataBusyStatus())
        {
            if(++timeout >= 2000)
            {
                break;
            }
        }
        QSPI->DAT0 = *(tx_buf+num);
    }
    
    // switch to standard mode 
    Qspi_Init(STANDARD_SPI_FORMAT_SEL, TX_AND_RX, 1024);
    QspiFlashReadStatus();
}
void W25Q64_BufferWrite(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

//    NumByteToWrite/=4;
    Addr        = WriteAddr % W25Q64_PAGESIZE;                        //求按页写入后剩余数据量，例如需要写入257个数据，即写入1页后还需要再另1页多写1个数据
    count       = W25Q64_PAGESIZE - Addr;                             //页剩余空间，例上即剩余256-1个字节空间
    NumOfPage   = NumByteToWrite / W25Q64_PAGESIZE;                   //计算需要写入的页面
    NumOfSingle = NumByteToWrite % W25Q64_PAGESIZE;                   //计算单页需要写入的数据量
    
    if (Addr == 0) /*!<如果写入地址页对齐  */
    {
        if (NumOfPage == 0) /*!< 单次写入字节数刚好等于256*/
        {
            W25Q64_PageWrite(WriteAddr,pBuffer, NumByteToWrite);
        }
        else /*!< 单次写入字节数大于一页字节数 */
        {
            while (NumOfPage--)     //逐页写入
            {
                W25Q64_PageWrite(WriteAddr,pBuffer, W25Q64_PAGESIZE);
                WriteAddr += W25Q64_PAGESIZE;
                pBuffer += W25Q64_PAGESIZE;
            }

            W25Q64_PageWrite(WriteAddr,pBuffer, NumOfSingle);        //剩余不足一页数据写入
        }
    }
    else /*!< 写入数据量页不对齐  */
    {
        if (NumOfPage == 0) /*!< 单次写入量 小于 页数据量 */
        {
            if (NumOfSingle > count) /*!< 数据写入起始地址+需要写入的数据量 大于 页剩余空间 */
            {
                temp = NumOfSingle - count;             //先将此页存满，计算存满此页后剩下字节数

                W25Q64_PageWrite(WriteAddr,pBuffer, count);      //先写入当前一页数据
                WriteAddr += count;                                         //偏移
                pBuffer += count;       

                W25Q64_PageWrite(WriteAddr,pBuffer, temp);       //在下一页写入剩余数据
            }
            else        //当前页可写入所有数据
            {
                W25Q64_PageWrite(WriteAddr,pBuffer, NumByteToWrite);
            }
        }
        else /*!< 需要写入字节数 大于 一页字节总量 */
        {
            NumByteToWrite -= count;                                //计算当前页可存字节数
            NumOfPage   = NumByteToWrite / W25Q64_PAGESIZE;         //计算剩余待存数据量需要多少页
            NumOfSingle = NumByteToWrite % W25Q64_PAGESIZE;         //剩余多少数据

            W25Q64_PageWrite(WriteAddr,pBuffer, count);  //当前页存
            WriteAddr += count;
            pBuffer += count;

            while (NumOfPage--)                                     //整页存
            {
                W25Q64_PageWrite(WriteAddr,pBuffer, W25Q64_PAGESIZE);
                WriteAddr += W25Q64_PAGESIZE;
                pBuffer += W25Q64_PAGESIZE;
            }

            if (NumOfSingle != 0)                                   //剩余数据存
            {
                W25Q64_PageWrite(WriteAddr,pBuffer, NumOfSingle);
            }
        }
    }
    
}

//uint32_t QSPI_TxBuf[50] = {0}, QSPI_RxBuf[50] = {0};
/**================================================================================
        测试程序
================================================================================*/
//void QSPI_Test(void)
//{
//    uint32_t i, temp;
//    uint32_t buf[4];
//    static uint8_t add;

//    while (1)
//    {
//        /** prepare for testing data */
//        for (i = 0; i < 50; i++)
//        {
//            QSPI_TxBuf[i] = i + add;
//        }

//        add++;

//        //=============================================================
//        /**---------- erase test ----------*/
//        QspiFlashErase(0x20, TEST_ADDR);

//        /**---------- QSPI read and test ----------*/
//        for (i = 0; i < 50; i++)
//            QSPI_RxBuf[i] = 0;

//        QspiFlashRead(TEST_ADDR, QSPI_RxBuf, 50);

//        for (i = 0; i < 50; i++)
//        {
//            // log_info(" Erase = 0x%x \n", QSPI_RxBuf[i]);
//            if (QSPI_RxBuf[i] != 0xff)
//            {
//                // log_info(" Erase Fail\n");
//                return;
//            }
//        }

//        // log_info(" Erase OK\n");

//        //=============================================================
//        /**---------- write test ----------*/
//        QspiFlashProgram(TEST_ADDR, QSPI_TxBuf, 50);

//        /**---------- QSPI read and print ----------*/
//        for (i = 0; i < 50; i++)
//            QSPI_RxBuf[i] = 0;

//        QspiFlashRead(TEST_ADDR, QSPI_RxBuf, 50);

//        for (i = 0; i < 50; i++)
//        {
//            // log_info(" read = 0x%x \n", QSPI_RxBuf[i]);
//            if (QSPI_RxBuf[i] != (QSPI_TxBuf[i] & 0xff))
//            {
//                // log_info(" Write Fail\n");
//                return;
//            }
//        }

//        // log_info(" Write OK\n");

//        //=============================================================
//        /** QUAD SPI read test*/
//        // log_info("================= \r\n");

//        for (i = 0; i < 2; i++)
//        {
//            QspiFlashRead(TEST_ADDR + 4 * i, buf, 4);
//            temp = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]; // word print
//            // log_info(" data = 0x%08x \n", temp);
//        }

//        // log_info("================= \r\n");
//        for (i = 0; i < 8; i++)
//        {
//            QspiFlashRead(TEST_ADDR + i, &temp, 1);
//            // log_info(" data = 0x%08x \n", temp);      // byte print
//        }

//        // QspiInit(QUAD_SPI_FORMAT_SEL, RX_ONLY, CTRL1_NDF_CNT);
//        QspiSendWord(0xff);

//        m_delay_ms(5000);
//    }
//}
