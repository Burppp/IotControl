/*
***************************************Copyright (c)***************************
**                               	  武汉钛联信息技术有限公司
**                                     		工程事业部
**                                        
**
**                                		 
**
**--------------文件信息-------------------------------------------------------
**文   件   名: Flash.c
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
#include  "Flash/flash.h"

/******************************************************************************
** 函数名称: _flash_Address_Sector
** 功能描述: 获取指定地址对应的flash扇区
** 入口参数: 地址
** 返 回 值: 扇区
**
** 作　者: 杨开琦
** 日　期: 2018年8月29日
**-----------------------------------------------------------------------------
******************************************************************************/
static uint16_t _flash_Address_Sector(uint32_t address) {
  uint16_t sector = ADDR_WRONG_SECTOR;
  if(!IS_FLASH_ADDRESS(address) || address<FLASH_USER_START_ADDR || address>FLASH_USER_END_ADDR) {
    sector = ADDR_WRONG_SECTOR;
  }
#if MCU_FLASH_TYPE>1
  else if(address >= ADDR_FLASH_SECTOR_11) {
    sector = FLASH_Sector_11;
  } else if(address >= ADDR_FLASH_SECTOR_10) {
    sector = FLASH_Sector_10;
  }
#endif  //MCU_FLASH_TYPE>1
#if MCU_FLASH_TYPE>0
  else if(address >= ADDR_FLASH_SECTOR_9) {
    sector = FLASH_Sector_9;
  } else if(address >= ADDR_FLASH_SECTOR_8) {
    sector = FLASH_Sector_8;
  }
#endif  //MCU_FLASH_TYPE>0
  else if(address >= ADDR_FLASH_SECTOR_7) {
    sector = FLASH_Sector_7;
  } else if(address >= ADDR_FLASH_SECTOR_6) {
    sector = FLASH_Sector_6;
  } else if(address >= ADDR_FLASH_SECTOR_5) {
    sector = FLASH_Sector_5;
  } else if(address >= ADDR_FLASH_SECTOR_4) {
    sector = FLASH_Sector_4;
  } else if(address >= ADDR_FLASH_SECTOR_3) {
    sector = FLASH_Sector_3;
  } else if(address >= ADDR_FLASH_SECTOR_2) {
    sector = FLASH_Sector_2;
  } else if(address >= ADDR_FLASH_SECTOR_1) {
    sector = FLASH_Sector_1;
  } else {
    sector = FLASH_Sector_0;
  }
  return sector;
}


/******************************************************************************
** 函数名称: _flash_Erase_Page
** 功能描述: 擦除指定区间的Flash；此函数没有Unlock Flash，必须在Unlock Flash之后调用
** 入口参数: 要擦除的区间头尾（含头含尾）
** 返 回 值: 无
**
** 作　者: 杨开琦
** 日　期: 2018年6月4日
**-----------------------------------------------------------------------------
******************************************************************************/
static void _flash_Erase_Sectors(uint16_t sector_start, uint16_t sector_end) {
  if(sector_start > sector_end || !(IS_FLASH_SECTOR(sector_start))) return;
  
  for(uint16_t sector=sector_start; sector<=sector_end && IS_FLASH_SECTOR(sector); sector+=8) {
    FLASH_EraseSector(sector, VoltageRange_3);
  }
}

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
uint8_t flash_write(uint32_t address, uint8_t *pdata, uint16_t length) {
  uint16_t sector_start = _flash_Address_Sector(address);
  uint16_t sector_end   = _flash_Address_Sector(address+length);
  if(sector_start == ADDR_WRONG_SECTOR || sector_end == ADDR_WRONG_SECTOR) return 0;
  
  FLASH_Unlock();
  
  _flash_Erase_Sectors(sector_start, sector_end);
  for(uint16_t i = 0; i < length; i++) {
    FLASH_ProgramByte(address + i, *pdata++);
  }
  
  FLASH_Lock();
  return 1;
}

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
uint8_t flash_read(uint32_t address, uint8_t *pdata, uint16_t length) {
  if(!IS_FLASH_ADDRESS(address) || !IS_FLASH_ADDRESS(address+length)) return 0;
  
  for(uint16_t i = 0; i < length; i++) {
    *pdata++ = *(__IO uint8_t *) (address + i);
  }
  return 1;
}
