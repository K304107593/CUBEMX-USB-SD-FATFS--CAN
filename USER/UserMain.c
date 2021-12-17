/************************************************************************** 
 * 模块名称 : 
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\USER\UserMain.c
 * Date: 2021-12-02 19:04:34
 * LastEditTime: 2021-12-17 23:46:57
 * 说     明: 
 **************************************************************************/

#include "main.h"
#include "cmsis_os.h"
#include "UserMain.h"
#include "bsp_can_fifo103.h"

extern TIM_HandleTypeDef htim2;

/*
*********************************************************************************************************
*	函 数 名: dataProcessing
*	功能说明: 处理缓存中的数据
*	形    参: 任务句柄指针
*	返 回 值: 无
*********************************************************************************************************
*/
void dataProcessing(osThreadId *Thread_P)
{
        FRESULT DP;
        osEvent evt;
        evt = osSignalWait(0x01, 5000);
        if (evt.status == osEventSignal) //接收到信号标志
        {
                DP = canStoreFatfs(0);
                if (DP != FR_OK)
                {
                        debug_printf("%x\n", DP);
                        Error_Handler();
                }
        }
        else if (evt.status == osEventTimeout) //信号标志接收超时
        {
                DP = canStoreFatfs(1);
                if (DP != FR_OK)
                {
                        debug_printf("%x\n", DP);
                        Error_Handler();
                }
        }
}
