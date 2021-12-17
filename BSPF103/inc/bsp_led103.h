/************************************************************************** 
 * 模块名称 : 
 * FilePath: \2.3.0c:\Users\Administrator\Desktop\MICRO\FAtfs\USBCAN\SETUP 4 USB+FILESYSTEM+CAN+FILEWRITE\BSPF103\inc\bsp_led103.h
 * Date: 2021-09-26 22:21:51
 * LastEditTime: 2021-10-11 23:52:15
 * 说     明: 
 **************************************************************************/
/*
*********************************************************************************************************
*
*	模块名称 : LED指示灯驱动模块
*	文件名称 : bsp_led.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_LED_H
#define __BSP_LED_H

/* 供外部调用的函数声明 */
void bsp_InitLed(void);
void bsp_LedOn(uint8_t _no);
void bsp_LedOff(uint8_t _no);
void bsp_LedToggle(uint8_t _no);
uint8_t bsp_IsLedOn(uint8_t _no);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
