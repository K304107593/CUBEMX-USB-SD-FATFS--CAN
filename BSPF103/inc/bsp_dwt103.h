/*
*********************************************************************************************************
*
*	模块名称 : 数据观察点与跟踪(DWT)模块
*	文件名称 : bsp_dwt.h
*	版    本 : V1.0
*	说    明 : 头文件
*	修改记录 :
*		版本号    日期        作者     说明
*		V1.0    2015-08-18   Eric2013 正式发布
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_DWT_H
#define __BSP_DWT_H


void bsp_InitDWT(void);
void bsp_DelayUS(uint32_t _ulDelayTime);
void bsp_DelayMS(uint32_t _ulDelayTime);

uint32_t CPU_TS_GET(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
