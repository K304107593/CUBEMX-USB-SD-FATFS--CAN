/************************************************************************** 
 * ģ������ : 
 * FilePath: \RTOScore\BSPF103\inc\bsp_timer103.h
 * Date: 2021-09-03 13:10:48
 * LastEditTime: 2021-11-13 00:29:03
 * ˵     ��: 
 **************************************************************************/
/*
*********************************************************************************************************
*
*	ģ������ : ��ʱ��ģ��
*	�ļ����� : bsp_timer.h
*	��    �� : V1.3
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H


#include "bsp103.h"
//#include "include.h"
//#include "stm32f10x.h"


/*
	�ڴ˶������ɸ������ʱ��ȫ�ֱ���
	ע�⣬��������__IO �� volatile����Ϊ����������жϺ���������ͬʱ�����ʣ��п�����ɱ����������Ż���
*/
#define TMR_COUNT 4 /* �����ʱ���ĸ��� ����ʱ��ID��Χ 0 - 3) */

/*ʱ��ṹ��*/
typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t msec;
} clock;
static clock time = {0, 0, 0, 0};
/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */
typedef enum
{
	TMR_ONCE_MODE = 0, /* һ�ι���ģʽ */
	TMR_AUTO_MODE = 1  /* �Զ���ʱ����ģʽ */
} TMR_MODE_E;

/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */
typedef struct
{
	volatile uint8_t Mode;	   /* ������ģʽ��1���� */
	volatile uint8_t Flag;	   /* ��ʱ�����־  */
	volatile uint32_t Count;   /* ������ */
	volatile uint32_t PreLoad; /* ������Ԥװֵ */
} SOFT_TMR;

/* �ṩ������C�ļ����õĺ��� */
void bsp_InitTimer(void); //systick�ж�,��ʼ����ʱ������,����RTXʱʹ��TIME1-TIME4,����ʹ��systick��ʱ��
void bsp_DelayMS(uint32_t n);//MS����ʱ,RTX=0ʱ���趨��TIME1-TIME4����,����ʹ��DWT����
void bsp_DelayUS(uint32_t n);//us����ʱ,RTX=0ʱ���趨��TIME1-TIME4����,����ʹ��DWT����
void bsp_StartTimer(uint8_t _id, uint32_t _period);//����һ�����ζ�ʱ��,������sysTick_ISR����
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);//����һ���Զ���ʱ��,������sysTick_ISR����
void bsp_StopTimer(uint8_t _id);//ֹͣһ����ʱ��
uint8_t bsp_CheckTimer(uint8_t _id);//��鶨ʱ���Ƿ�ʱ��
int32_t bsp_GetRunTime(void);//��ȡ����ʱ��
int32_t bsp_CheckRunTime(int32_t _LastTime);//�������ʱ��
void Time_GetTime(clock *time1);//��ȡ24Сʱʱ��

void bsp_InitHardTimer(void);
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void *_pCallBack);

void SysTick_ISR(void);//bsp_timeʱ����º���,Ҫʹ��bsp_time����1ms����
#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
