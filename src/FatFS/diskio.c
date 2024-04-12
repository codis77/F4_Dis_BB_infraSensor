/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "stm32f4xx.h"
#include "ffconf.h"
#include "stm32f4_discovery_sdio_sd.h"

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */
/*-----------------------------------------------------------------------*/

#define ATA		0
#define MMC		1
#define USB		2

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/* par: physical drive number (0..)                                      */
/*-----------------------------------------------------------------------*/

DSTATUS  disk_initialize (BYTE drv)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    SD_Error          res = SD_OK;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel                   = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    res =  SD_Init();
    if(res == SD_OK)
        res = (SD_Error)0x0;
    return ((DSTATUS)res);
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */
/* par: physical drive number (0..)                                      */
/*-----------------------------------------------------------------------*/

DSTATUS  disk_status (BYTE drv)
{
    if (drv)
        return STA_NOINIT;    /* Supports only single drive */
    return 0;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/* par: physical drive number (0..)                                      */
/*      data buffer to store read data                                   */
/*      sector address (LBA)                                             */
/*      number of sectors to read (1..255)                               */
/*-----------------------------------------------------------------------*/

DRESULT  disk_read (BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
    SD_Error status = SD_OK;

    SD_ReadMultiBlocks(buff, sector << 9, 512, 1);

    /* Check if the Transfer is finished */
    status =  SD_WaitReadOperation();
    while (SD_GetStatus() != SD_TRANSFER_OK);

    if (status == SD_OK)
        return RES_OK;
    else
        return RES_ERROR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/* par: physical drive number (0..)                                      */
/*      data buffer to store read data                                   */
/*      sector address (LBA)                                             */
/*      number of sectors to read (1..255)                               */
/*-----------------------------------------------------------------------*/
/* The FatFs module will issue multiple sector transfer request
/  (count > 1) to the disk I/O layer. The disk function should process
/  the multiple sector transfer properly Do. not translate it into
/  multiple single sector transfers to the media, or the data read/write
/  performance may be drasticaly decreased. */

#if _READONLY == 0
DRESULT  disk_write (BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{
    SD_Error status = SD_OK;

    SD_WriteMultiBlocks ((BYTE *) buff, sector << 9, 512, 1);

    /* Check if the Transfer is finished */
    status = SD_WaitWriteOperation();
    while(SD_GetStatus() != SD_TRANSFER_OK);     
    if (status == SD_OK)
        return RES_OK;
    else
        return RES_ERROR;
}
#endif /* _READONLY */


/*-----------------------------------------------------------------------*/
/* Get current time                                                      */
/*-----------------------------------------------------------------------*/

DWORD  get_fattime ()
{
    return ((2006UL-1980) << 25)  // Year = 2006
            | (2UL << 21)         // Month = Feb
            | (9UL << 16)         // Day = 9
            | (22U << 11)         // Hour = 22
            | (30U << 5)          // Min = 30
            | (0U  >> 1)          // Sec = 0
            ;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/* par: physical drive number (0..)                                      */
/*      control code                                                     */
/*      buffer to send/receive control data                              */
/*-----------------------------------------------------------------------*/

DRESULT  disk_ioctl (BYTE drv, BYTE ctrl, void *buff)
{
    DRESULT  res = RES_OK;

    switch (ctrl)
    {
        case GET_SECTOR_COUNT :	      // Get number of sectors on the disk (DWORD)
            *(DWORD*)buff = 131072;	  // 4*1024*32 = 131072
            res = RES_OK;
            break;

        case GET_SECTOR_SIZE :	      // Get R/W sector size (WORD) 
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE :	      // Get erase block size in unit of sector (DWORD)
            *(DWORD*)buff = 32;
            res = RES_OK;
    }

    return res;
}
