/************************************************************************** 
 * ģ������ : 
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\USER\UserMain.c
 * Date: 2021-12-02 19:04:34
 * LastEditTime: 2021-12-17 23:46:57
 * ˵     ��: 
 **************************************************************************/

#include "main.h"
#include "cmsis_os.h"
#include "UserMain.h"
#include "bsp_can_fifo103.h"

extern TIM_HandleTypeDef htim2;

/*
*********************************************************************************************************
*	�� �� ��: dataProcessing
*	����˵��: �������е�����
*	��    ��: ������ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dataProcessing(osThreadId *Thread_P)
{
        FRESULT DP;
        osEvent evt;
        evt = osSignalWait(0x01, 5000);
        if (evt.status == osEventSignal) //���յ��źű�־
        {
                DP = canStoreFatfs(0);
                if (DP != FR_OK)
                {
                        debug_printf("%x\n", DP);
                        Error_Handler();
                }
        }
        else if (evt.status == osEventTimeout) //�źű�־���ճ�ʱ
        {
                DP = canStoreFatfs(1);
                if (DP != FR_OK)
                {
                        debug_printf("%x\n", DP);
                        Error_Handler();
                }
        }
}
