/*
***************************************Copyright (c)***************************
**                               	  �人������Ϣ�������޹�˾
**                                     		������ҵ��
**                                        
**
**                                		 
**
**--------------�ļ���Ϣ-------------------------------------------------------
**��   ��   ��: Flash.h
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
** ��������: flash_write
** ��������: ������д��flash
** ��ڲ���: Ҫд�����ʼ��ַ��Ҫд�������ͷָ�룻Ҫд������ݳ���(�ֽ���)
** �� �� ֵ: д������0--ʧ�ܣ�1--�ɹ�
**
** ������: ���
** �ա���: 2018��6��4��
**-----------------------------------------------------------------------------
******************************************************************************/
uint8_t flash_write(uint32_t address, uint8_t *pdata, uint16_t length);

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
uint8_t flash_read(uint32_t address, uint8_t *pdata, uint16_t length);

#endif //_FLASH_H_
