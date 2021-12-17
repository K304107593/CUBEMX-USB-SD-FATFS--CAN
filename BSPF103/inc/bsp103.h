/************************************************************************** 
 * ģ������ : BSPģ��(for STM32F1)
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\inc\bsp103.h
 * Date: 2021-08-18 11:22:44
 * LastEditTime: 2021-12-02 19:43:14
 * ˵     ��: 
 **************************************************************************/

#ifndef _BSP_H_
#define _BSP_H_

/* ���� BSP �汾�� */
#define VERSION "1.1"

#define USE_RTX 1

#define DEBUG 1//�Ƿ�������ģʽ

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT() __set_PRIMASK(0)  /* ʹ��ȫ���ж� */
#define DISABLE_INT() __set_PRIMASK(0) /* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */

#if DEBUG
#define debug_printf(...)     USART_Printf(1, ##__VA_ARGS__) //printf(__VA_ARGS__)
#define BSP_Printf(...) debug_printf(__VA_ARGS__)
#define debug_printffreertos() char InfoBuffer[1000];vTaskList(InfoBuffer);debug_printf("��������\t����״̬\t���ȼ�\tʣ���ջ\t�������\r\n");\
debug_printf("%s\r\n",InfoBuffer);debug_printf("B : ����, R : ����, D : ɾ��, S : ��ͣ\r\n")
//#define debug_printf USART_Printf
#else
#define debug_printf(...) 
#define BSP_Printf(...)    
#define debug_printffreertos()
#endif

#define EXTI9_5_ISR_MOVE_OUT /* bsp.h �ж�����У���ʾ�������Ƶ� stam32f4xx_it.c�� �����ظ����� */

#define Error_Handler() Error_Handler103(__FILE__, __LINE__) /*����HAL��*/

/* Ĭ���ǹر�״̬ */
#define Enable_EventRecorder 0

#if Enable_EventRecorder == 1
#include "EventRecorder.h"
#endif

//#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>


#include "stdint.h"







#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* �������ȼ����� */
#define NVIC_PREEMPT_PRIORITY 4

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
//#include "bsp_msg.h"
//#include "bsp_user_lib.h"
//#include "bsp_timer103.h"
#include "bsp_led103.h"
#include "bsp_key103.h" //Ҫʹ��KEY,ҲҪʹ��bsp_timer,keyˢ������10ms�ж�����
#include "bsp_dwt103.h"


//#include "bsp_cpu_rtc.h"
//#include "bsp_cpu_adc.h"
//#include "bsp_cpu_dac.h"
#include "bsp_uart_fifo103.h"
//#include "bsp_uart_gps.h"
//#include "bsp_uart_esp8266.h"
//#include "bsp_uart_sim800.h"

//#include "bsp_spi_bus.h"
//#include "bsp_spi_ad9833.h"
//#include "bsp_spi_ads1256.h"
//#include "bsp_spi_dac8501.h"
//#include "bsp_spi_dac8562.h"
//#include "bsp_spi_flash.h"
//#include "bsp_spi_tm7705.h"
//#include "bsp_spi_vs1053b.h"

//#include "bsp_fmc_sdram.h"
//#include "bsp_fmc_nand_flash.h"
//#include "bsp_fmc_ad7606.h"
//#include "bsp_fmc_oled.h"
//#include "bsp_fmc_io.h"

//#include "bsp_i2c_gpio.h"
//#include "bsp_i2c_bh1750.h"
//#include "bsp_i2c_bmp085.h"
//#include "bsp_i2c_eeprom_24xx.h"
//#include "bsp_i2c_hmc5883l.h"
//#include "bsp_i2c_mpu6050.h"
//#include "bsp_i2c_si4730.h"
//#include "bsp_i2c_wm8978.h"

//#include "bsp_tft_h7.h"
//#include "bsp_tft_lcd.h"
//#include "bsp_ts_touch.h"
//#include "bsp_ts_ft5x06.h"
//#include "bsp_ts_gt811.h"
//#include "bsp_ts_gt911.h"
//#include "bsp_ts_stmpe811.h"

//#include "bsp_beep.h"
//#include "bsp_tim_pwm.h"
//#include "bsp_sdio_sd.h"
//#include "bsp_dht11.h"
//#include "bsp_ds18b20.h"
//#include "bsp_ps2.h"
//#include "bsp_ir_decode.h"
//#include "bsp_camera.h"
//#include "bsp_rs485_led.h"
//#include "bsp_can.h"
#include "bsp_can_fifo103.h"

/* �ṩ������C�ļ����õĺ��� */

void bsp_Init(void);
void bsp_Idle(void);
void System_Init(void);

void bsp_GetCpuID(uint32_t *_id);
void Error_Handler103(char *file, uint32_t line);
extern void USART_Printf(uint8_t UsartPort,const char *fmt, ...);//ʵ�ֵ��Դ�ӡ

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
