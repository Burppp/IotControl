/*
***************************************Copyright (c)***************************
**                               	  �人������Ϣ�������޹�˾
**                                     		������ҵ��
**                                        
**
**                                		 
**
**--------------�ļ���Ϣ-------------------------------------------------------
**��   ��   ��: Flash.c
**��   ��   ��: ���
**����޸�����: 2018��6��4��
**��        ��: �ڲ�Flash����
**              
**--------------��ʷ�汾��Ϣ---------------------------------------------------
** ������: ���
** ��  ��: v1.0
** �ա���: 2018��6��4��
** �衡��: ��ʼ����
**
**--------------��ǰ�汾�޶�---------------------------------------------------
** �޸���: 
** �ա���: 
** �衡��: 
**
**-----------------------------------------------------------------------------
******************************************************************************/
#include  "Flash/flash.h"

/******************************************************************************
** ��������: _flash_Address_Sector
** ��������: ��ȡָ����ַ��Ӧ��flash����
** ��ڲ���: ��ַ
** �� �� ֵ: ����
**
** ������: ���
** �ա���: 2018��8��29��
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
** ��������: _flash_Erase_Page
** ��������: ����ָ�������Flash���˺���û��Unlock Flash��������Unlock Flash֮�����
** ��ڲ���: Ҫ����������ͷβ����ͷ��β��
** �� �� ֵ: ��
**
** ������: ���
** �ա���: 2018��6��4��
**-----------------------------------------------------------------------------
******************************************************************************/
static void _flash_Erase_Sectors(uint16_t sector_start, uint16_t sector_end) {
  if(sector_start > sector_end || !(IS_FLASH_SECTOR(sector_start))) return;
  
  for(uint16_t sector=sector_start; sector<=sector_end && IS_FLASH_SECTOR(sector); sector+=8) {
    FLASH_EraseSector(sector, VoltageRange_3);
  }
}

/******************************************************************************
** ��������: flash_write
** ��������: ������д��flash
** ��ڲ���: Ҫд�����ʼ��ַ��Ҫд�������ͷָ�룻Ҫд������ݳ���(�ֽ���)
** �� �� ֵ: д������0--ʧ�ܣ�1--�ɹ�
**
** ������: ���
** �ա���: 2018��6��4��
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
** ��������: flash_read
** ��������: ��ȡflash�е�����
** ��ڲ���: Ҫ��ȡ����ʼ��ַ�������ȡ�������ݵ�ͷָ�룻Ҫ��ȡ�����ݳ���(�ֽ���)
** �� �� ֵ: ��ȡ�����0--ʧ�ܣ�1--�ɹ�
**
** ������: ���
** �ա���: 2018��6��4��
**-----------------------------------------------------------------------------
******************************************************************************/
uint8_t flash_read(uint32_t address, uint8_t *pdata, uint16_t length) {
  if(!IS_FLASH_ADDRESS(address) || !IS_FLASH_ADDRESS(address+length)) return 0;
  
  for(uint16_t i = 0; i < length; i++) {
    *pdata++ = *(__IO uint8_t *) (address + i);
  }
  return 1;
}
