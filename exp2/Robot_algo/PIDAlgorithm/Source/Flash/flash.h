/*
***************************************Copyright (c)***************************
**                               	  武汉钛联信息技术有限公司
**                                     		工程事业部
**                                        
**
**                                		 
**
**--------------文件信息-------------------------------------------------------
**文   件   名: Flash.h
**创   建   人: 杨开琦
**最后修改日期: 2018年6月4日
**描        述: 内部Flash操作
**              
**--------------历史版本信息---------------------------------------------------
** 创建人: 杨开琦
** 版  本: v1.0
** 日　期: 2018年6月4日
** 描　述: 初始创建
**
**--------------当前版本修订---------------------------------------------------
** 修改人: 
** 日　期: 
** 描　述: 
**
**-----------------------------------------------------------------------------
******************************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f4xx.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_7   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_7   /* End @ of user Flash area */

#define ADDR_WRONG_SECTOR       ((uint16_t)0x00FF)

//stm32f407V-E(0):512k;F(1):768k;G(2):1024k
#define MCU_FLASH_TYPE 0  /*stm32f407VE*/
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */
#if MCU_FLASH_TYPE>0
  #define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbyte */
  #define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
#endif  //MCU_FLASH_TYPE>0
#if MCU_FLASH_TYPE>1
  #define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
  #define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */
#endif  //MCU_FLASH_TYPE>1

/******************************************************************************
** 函数名称: flash_write
** 功能描述: 将数据写入flash
** 入口参数: 要写入的起始地址；要写入的数据头指针；要写入的数据长度(字节数)
** 返 回 值: 写入结果：0--失败；1--成功
**
** 作　者: 杨开琦
** 日　期: 2018年6月4日
**-----------------------------------------------------------------------------
******************************************************************************/
uint8_t flash_write(uint32_t address, uint8_t *pdata, uint16_t length);

/******************************************************************************
** 函数名称: flash_read
** 功能描述: 读取flash中的数据
** 入口参数: 要读取的起始地址；保存读取出的数据的头指针；要读取的数据长度(字节数)
** 返 回 值: 读取结果：0--失败；1--成功
**
** 作　者: 杨开琦
** 日　期: 2018年6月4日
**-----------------------------------------------------------------------------
******************************************************************************/
uint8_t flash_read(uint32_t address, uint8_t *pdata, uint16_t length);

#endif //_FLASH_H_
