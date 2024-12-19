/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"         /* Obtains integer types */
#include "diskio.h"     /* Declarations of disk functions */

/* Definitions of physical drive number for each drive */
#define DEV_W25Q64      0   /* Example: Map Ramdisk to physical drive 0 */
#include "w25q64.h"


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            if (W25Q64_ReadID() != 0X00EF4017)  //W25Q64_ID
                stat = STA_NOINIT;
            else
                stat = 0;

            break;

        default:
            stat = STA_NOINIT;
            break;
    }

    return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;
    int result;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            result = W25Q64_Init();

            // translate the reslut code here

            return disk_status(DEV_W25Q64);

    }

    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    LBA_t sector,   /* Start sector in LBA */
    UINT count      /* Number of sectors to read */
)
{
    DRESULT res;

    switch (pdrv)
    {
        case DEV_W25Q64 :

            QspiFlashRead(sector * 4096, buff, count * 4096);
            res = RES_OK;

            return res;
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write(
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE *buff,   /* Data to be written */
    LBA_t sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write */
)
{
    DRESULT res;
    int result;
    uint32_t waddr;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            waddr = sector + 1024;                           //扇区偏移4M，头4M用于XIP程序
            waddr = (uint32_t)(waddr << 12);
            QspiFlashErase(0x20, waddr);
            W25Q64_BufferWrite((uint8_t *)buff, waddr, 4096);
            res = RES_OK;
            return res;
    }

    return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT res;
    int result;

    switch (pdrv)
    {
        case DEV_W25Q64 :

            switch (cmd)
            {
                case GET_SECTOR_COUNT:
                    *(DWORD *)buff = 4096;          //W25Q64,8MByte,4096个扇区
                    break;

                case GET_SECTOR_SIZE:
                    *(WORD *)buff = 4096;           //以一个扇区4096字节为读写单位
                    break;

                case GET_BLOCK_SIZE:
                    *(WORD *)buff = 1;              //每次擦除一个扇区
                    break;

            }

            res = RES_OK;
            return res;

    }

    return RES_PARERR;
}

__weak DWORD get_fattime(void) {
	/* Returns current time packed into a DWORD variable */
	return	  ((DWORD)(2024 - 1980) << 25)	/* Year 2013 */
			| ((DWORD)12 << 21)				/* Month 7 */
			| ((DWORD)18 << 16)				/* Mday 28 */
			| ((DWORD)11 << 11)				/* Hour 0 */
			| ((DWORD)59 << 5)				/* Min 0 */
			| ((DWORD)59 >> 1);				/* Sec 0 */
}
