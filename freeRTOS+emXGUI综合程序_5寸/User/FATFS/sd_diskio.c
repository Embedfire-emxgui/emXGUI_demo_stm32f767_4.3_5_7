/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    08-May-2015
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "./sdmmc/bsp_sdmmc_sd.h"
#include "emXGUI.h"
#include "FreeRTOS.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Block Size in Bytes */
#define BLOCK_SIZE                512
static GUI_SEM *sem_sd = NULL;
static GUI_MUTEX *mutex_lock=NULL;
//发送标志位
volatile uint8_t TX_Flag;
//接受标志位
volatile uint8_t RX_Flag; 

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */
  
const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read, 
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */
  
#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  lun : not used 
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
  Stat = STA_NOINIT;
  
  /* Configure the uSD device */
  if(BSP_SD_Init() == MSD_OK)
  {
		sem_sd = GUI_SemCreate(0,1);//同步SD卡读取
    mutex_lock = GUI_MutexCreate();
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetStatus() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }
  
  return Stat;
}

DRESULT SD_ReadBlocks(BYTE *buff,//数据缓存区 
                      DWORD sector, //扇区首地址
                      UINT count)//扇区个数(1..128)
{
  DRESULT res = RES_ERROR;  
  uint32_t alignedAddr;
  taskENTER_CRITICAL();
  if(HAL_SD_ReadBlocks_DMA(&uSdHandle, (uint32_t*)buff,
                           (uint32_t) (sector),
														BLOCK_SIZE,
                           count) == HAL_OK)
  {
    taskEXIT_CRITICAL();
	
    GUI_SemWait(sem_sd,2000);
    {
			res = RES_OK;
			/*
			 *	 the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
			 *       adjust the address and the D-Cache size to invalidate accordingly.
			 */
			alignedAddr = (uint32_t)buff & ~0x1F;
			//使相应的DCache无效
			SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCK_SIZE + ((uint32_t)buff - alignedAddr));
			GUI_DEBUG("SD_ReadBlocks 1 ! \r\n");
		}
	}else
	{
		GUI_DEBUG("SD_ReadBlocks ERROR ! \r\n");
		taskEXIT_CRITICAL();
	}
  return res;  
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
#if 1
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_OK;
    uint32_t alignedAddr = 0;
		BYTE * tempbuf = buff;
			/* 字节不对齐的情况 */
    if ((DWORD)buff & 3) 
    {
        DWORD scratch[BLOCK_SIZE / 4];//以字为单位(4个字节)创建的BUF

        while (count--) 
        {
            memcpy(scratch, buff, BLOCK_SIZE);//把不对齐的buf数据拷贝到对齐的数组中
            res = SD_read(lun,(void *)scratch, sector++, 1);
            
            if (res != RES_OK)
            {
               break;
            }
            buff += BLOCK_SIZE;
        }
//				SCB_InvalidateDCache();
//				SCB_InvalidateDCache_by_Addr((uint32_t*)tempbuf, count*BLOCK_SIZE + ((uint32_t)tempbuf));
//	GUI_DEBUG("xxxxxxxxxxxxxxxxSD_read(BYTE lun %d, BYTE *buff = 0x%p, DWORD sector = %ld, UINT count = %d)",lun,buff,sector,count);

        return(res);
     }
    GUI_MutexLock(mutex_lock,0xffffff);

			/* 字节对齐的情况 */
		if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff, 
												 (uint64_t) (sector * BLOCK_SIZE), 
												 BLOCK_SIZE, 
												 count) != MSD_OK)
		{
			res = RES_ERROR;
		}
		else
		{	
//			SCB_InvalidateDCache();

//			alignedAddr = (uint32_t)buff & ~0x1F;

//			SCB_InvalidateDCache_by_Addr((uint32_t*)buff, count*BLOCK_SIZE + ((uint32_t)buff - alignedAddr));
			SCB_InvalidateDCache_by_Addr((uint32_t*)buff, count*BLOCK_SIZE);
		}
		
		GUI_MutexUnlock(mutex_lock);
//	GUI_DEBUG("oooooooooooooooSD_read(BYTE lun %d, BYTE *buff = 0x%p, DWORD sector = %ld, UINT count = %d)",lun,buff,sector,count);

  return res;
}
#else
DRESULT SD_read(BYTE lun,//物理扇区，多个设备时用到(0...)
                BYTE *buff,//数据缓存区 
                DWORD sector, //扇区首地址
                UINT count)//扇区个数(1..128)
{
  DRESULT res = RES_ERROR;
  uint32_t i;
  DWORD pbuff[512/4];	
  	
	RX_Flag = 0;
	
  if((DWORD)buff&3)
  {
    DRESULT res = RES_OK;
    DWORD scratch[BLOCK_SIZE / 4];

    while (count--) 
    {
      res = disk_read(0,(void *)scratch, sector++, 1);

      if (res != RES_OK) 
      {
        break;
      }
      memcpy(buff, scratch, BLOCK_SIZE);
      buff += BLOCK_SIZE;
    }
    return res;
  }
  
    GUI_MutexLock(mutex_lock,0xffffff);
	 	for(i=0;i<count;i++)
		{
			
		 	res = SD_ReadBlocks((BYTE *)pbuff,sector+i,1);//单个sector的读操作
      taskENTER_CRITICAL();
			memcpy(buff,pbuff,512);
      taskEXIT_CRITICAL();
			buff+=512;
		} 
	
  GUI_MutexUnlock(mutex_lock);
  return RES_OK;
}
#endif

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_OK;
  
    if ((DWORD)buff & 3) 
    {
        DWORD scratch[BLOCK_SIZE / 4];

        while (count--) 
        {
            memcpy(scratch, buff, BLOCK_SIZE);
            res = SD_write(lun,(void *)scratch, sector++, 1);
            
            if (res != RES_OK)
            {
               break;
            }
            buff += BLOCK_SIZE;
        }

        return(res);
     }
    if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff, 
                        (uint64_t)(sector * BLOCK_SIZE), 
                        BLOCK_SIZE, count) != MSD_OK)
    {
        res = RES_ERROR;
    }
  
    return res;
}   
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  SD_CardInfo CardInfo;
  
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;
  
  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.CardCapacity / BLOCK_SIZE;
    res = RES_OK;
    break;
  
  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    *(WORD*)buff = BLOCK_SIZE;
    res = RES_OK;
    break;
  
  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    *(DWORD*)buff = BLOCK_SIZE;
    break;
  
  default:
    res = RES_PARERR;
  }
  
  return res;
}
#endif /* _USE_IOCTL == 1 */
  
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma)
{
  TX_Flag=1; //标记写完成
}

void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
  GUI_SemPostISR(sem_sd);
  RX_Flag=1;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

