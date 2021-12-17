/************************************************************************** 
 * ģ������ : 
 * FilePath: \2.3.0c:\Users\Administrator\Desktop\MICRO\FAtfs\USBCAN\SETUP 4 USB+FILESYSTEM+CAN+FILEWRITE\BSPF103\inc\bsp_led103.h
 * Date: 2021-09-26 22:21:51
 * LastEditTime: 2021-10-11 23:52:15
 * ˵     ��: 
 **************************************************************************/
/*
*********************************************************************************************************
*
*	ģ������ : LEDָʾ������ģ��
*	�ļ����� : bsp_led.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_LED_H
#define __BSP_LED_H

/* ���ⲿ���õĺ������� */
void bsp_InitLed(void);
void bsp_LedOn(uint8_t _no);
void bsp_LedOff(uint8_t _no);
void bsp_LedToggle(uint8_t _no);
uint8_t bsp_IsLedOn(uint8_t _no);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
