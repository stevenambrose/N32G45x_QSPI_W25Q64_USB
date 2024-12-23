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
#include "w25q64.h"
/* Definitions of physical drive number for each drive */
#define DEV_W25Q64      0   /* Example: Map Ramdisk to physical drive 0 */
//#define DEV_MMC       1   /* Example: Map MMC/SD card to physical drive 1 */
//#define DEV_USB       2   /* Example: Map USB MSD to physical drive 2 */

#define ADDR_SHIFT      0X400000

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat = STA_NOINIT;
    uint32_t TID;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            TID=W25Q64_ReadID();
            if ((0X00EF4017 && TID))
            {
                stat &= ~STA_NOINIT;
            }
            else
            {
                stat = STA_NOINIT;
            }

            break;

        default:
            stat = STA_NOINIT;

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
    DSTATUS stat = STA_NOINIT;
    uint16_t i;

    switch (pdrv)
    {
        case DEV_W25Q64 :

            W25Q64_Init();

            stat = disk_status(DEV_W25Q64);
            break;

        // translate the reslut code here

        default:
            stat = STA_NOINIT;

    }

    return stat;
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
    DRESULT status = RES_PARERR;
    uint32_t addr;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            addr = sector;
            addr = (addr << 12) + ADDR_SHIFT;

            QspiFlashRead(addr, buff, count << 12);
            status = RES_OK;
            break;

        default:
            status = RES_PARERR;
    }

    return status;
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
    DRESULT status = RES_PARERR;
    uint32_t waddr;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            waddr = (sector <<12)+ADDR_SHIFT;
            QspiFlashErase(0x20,waddr);
            W25Q64_BufferWrite((uint8_t *)buff,waddr,count<<12);
            status = RES_OK;
        break;
        
        default:
            status = RES_PARERR;

    }

    return status;
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
    DRESULT status = RES_PARERR;
    int result;

    switch (pdrv)
    {
        case DEV_W25Q64 :
            switch(cmd)
            {
                case GET_SECTOR_COUNT:
                    *(DWORD *)buff = 1024;
                break;
                
                case GET_SECTOR_SIZE:
                    *(WORD *)buff = 4096;
                break;
                
                case GET_BLOCK_SIZE:
                    *(DWORD *)buff = 1;
                break;
            }
        status = RES_OK;
        break;
                
        default:
            status = RES_PARERR;
    }

    return status;
}

__weak DWORD get_fattime(void) {
	/* Returns current time packed into a DWORD variable */
	return	  ((DWORD)(2024 - 1980) << 25)	/* Year 2013 */
			| ((DWORD)12 << 21)				/* Month 7 */
			| ((DWORD)21 << 16)				/* Mday 28 */
			| ((DWORD)11 << 11)				/* Hour 0 */
			| ((DWORD)59 << 5)				/* Min 0 */
			| ((DWORD)59 >> 1);				/* Sec 0 */
}
