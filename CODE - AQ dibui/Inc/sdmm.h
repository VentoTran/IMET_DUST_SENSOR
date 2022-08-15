/*
 * sdmc.h
 *
 *  Created on: May 23, 2021
 *      Author: manht
 */

#ifndef SDCARD_SDMM_H_
#define SDCARD_SDMM_H_

#include "main.h"
#include "ff.h"		/* Obtains integer types for FatFs */
#include "diskio.h"	/* Common include file for FatFs and disk I/O layer */
//#include "uart/uart.h"
#include "delay.h"

/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/

#include "spi.h"		/* Include device specific declareation file here */
#include "io.h"
#define	CS_H()		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, 1)	/* Set MMC CS "high" */
#define CS_L()		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, 0)	/* Set MMC CS "low" */

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command (SPI mode) */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD13	(13)		/* SEND_STATUS */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */


#define SD_SPI	hspi1
typedef struct {
	bool mount;
	char label[20];
	uint32_t serial;
	FATFS FatFs;
	FIL Fil;
}sdcard_handle_t;
extern sdcard_handle_t sdcard;
FRESULT write2file(char *filename, char *content, int content_len);
DSTATUS disk_status (
	BYTE drv			/* Drive number (always 0) */
);
DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
);
DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..128) */
);
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..128) */
);
DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
);
FRESULT scan_files (
    char* path        /* Start node to be scanned (***also used as work area***) */
);
FRESULT set_timestamp (
    char *obj,     /* Pointer to the file name */
    int year,
    int month,
    int mday,
    int hour,
    int min,
    int sec
);
FRESULT	create_directory(
	char *directory	 	/* Pointer to the directory */
);
#endif /* SDCARD_SDMM_H_ */
