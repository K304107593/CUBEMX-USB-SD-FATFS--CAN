/************************************************************************** 
 * 模块名称 : 
 * FilePath: \RTOScore\BSPF103\inc\bsp_timer103.h
 * Date: 2021-09-03 13:10:48
 * LastEditTime: 2021-11-13 00:29:03
 * 说     明: 
 **************************************************************************/
/*
*********************************************************************************************************
*
*	模块名称 : 定时器模块
*	文件名称 : bsp_timer.h
*	版    本 : V1.3
*	说    明 : 头文件
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H


#include "bsp103.h"
//#include "include.h"
//#include "stm32f10x.h"


/*
	在此定义若干个软件定时器全局变量
	注意，必须增加__IO 即 volatile，因为这个变量在中断和主程序中同时被访问，有可能造成编译器错误优化。
*/
#define TMR_COUNT 4 /* 软件定时器的个数 （定时器ID范围 0 - 3) */

/*时间结构体*/
typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t msec;
} clock;
static clock time = {0, 0, 0, 0};
/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
typedef enum
{
	TMR_ONCE_MODE = 0, /* 一次工作模式 */
	TMR_AUTO_MODE = 1  /* 自动定时工作模式 */
} TMR_MODE_E;

/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
typedef struct
{
	volatile uint8_t Mode;	   /* 计数器模式，1次性 */
	volatile uint8_t Flag;	   /* 定时到达标志  */
	volatile uint32_t Count;   /* 计数器 */
	volatile uint32_t PreLoad; /* 计数器预装值 */
} SOFT_TMR;

/* 提供给其他C文件调用的函数 */
void bsp_InitTimer(void); //systick中断,初始化定时器变量,开启RTX时使用TIME1-TIME4,否则使用systick定时器
void bsp_DelayMS(uint32_t n);//MS级延时,RTX=0时由设定的TIME1-TIME4控制,否择使用DWT控制
void bsp_DelayUS(uint32_t n);//us级延时,RTX=0时由设定的TIME1-TIME4控制,否择使用DWT控制
void bsp_StartTimer(uint8_t _id, uint32_t _period);//启动一个单次定时器,控制由sysTick_ISR控制
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);//启动一个自动定时器,控制由sysTick_ISR控制
void bsp_StopTimer(uint8_t _id);//停止一个定时器
uint8_t bsp_CheckTimer(uint8_t _id);//检查定时器是否到时间
int32_t bsp_GetRunTime(void);//获取运行时间
int32_t bsp_CheckRunTime(int32_t _LastTime);//检查运行时间
void Time_GetTime(clock *time1);//获取24小时时间

void bsp_InitHardTimer(void);
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void *_pCallBack);

void SysTick_ISR(void);//bsp_time时间更新函数,要使用bsp_time必须1ms更新
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
