/************************************************************************** 
 * ģ������ : BSPģ��(for STM32F1)
 * FilePath: \RTOScore\BSPF103\bsp103.c
 * Date: 2021-08-18 14:27:53
 * LastEditTime: 2021-11-19 14:18:31
 * ˵     ��: 
 **************************************************************************/
#include "bsp103.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler( void );//��Ҫ��Ϊ�˸���freertos
/*
*********************************************************************************************************
*	                                   ��������
*********************************************************************************************************
*/
#if 0 //��main�����ʼ����
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);


static HAL_StatusTypeDef BSP_InitTick(uint32_t TickPriority);
#endif
/*
*********************************************************************************************************
*	�� �� ��: System_Init
*	����˵��: ϵͳ��ʼ������Ҫ��MPU��Cache��ϵͳʱ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void System_Init(void)
{
#if 0
    /* ����MPU */
    MPU_Config();

    /* ʹ��L1 Cache */
    CPU_CACHE_Enable();

    /* 
       STM32H7xx HAL ���ʼ������ʱϵͳ�õĻ���H7�Դ���64MHz��HSIʱ��:
	   - ���ú���HAL_InitTick����ʼ���δ�ʱ���ж�1ms��
	   - ����NVIV���ȼ�����Ϊ4��
	 */
    HAL_Init();

    /* 
       ����ϵͳʱ�ӵ�72MHz
       - �л�ʹ��HSE��
       - �˺��������ȫ�ֱ���SystemCoreClock������������HAL_InitTick��
    */
    SystemClock_Config();
#endif
    /* 
	   Event Recorder��
	   - �����ڴ���ִ��ʱ�������MDK5.25�������ϰ汾��֧�֣�IAR��֧�֡�
	   - Ĭ�ϲ����������Ҫʹ�ܴ�ѡ���ؿ�V7�������û��ֲ��8��
	*/
#if Enable_EventRecorder == 1
    /* ��ʼ��EventRecorder������ */
    EventRecorderInitialize(EventRecordAll, 1U);
    EventRecorderStart();
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_Init
*	����˵��: ��ʼ�����е�Ӳ���豸���ú�������CPU�Ĵ���������ļĴ�������ʼ��һЩȫ�ֱ�����ֻ��Ҫ����һ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_Init(void)
{
    //bsp_InitDWT();   /* ��ʼ��CM7�ں˵�ʱ�����ڼ����� */
    bsp_InitKey();   /* ������ʼ����Ҫ���ڵδ�ʱ��֮ǰ����Ϊ��ť�����ͨ���δ�ʱ��ɨ�� */
    bsp_InitUart();  /* ��ʼ������ */
    //bsp_InitExtIO(); /* ��ʼ��FMC����74HC574��չIO. ������ bsp_InitLed()ǰִ�� */
    bsp_InitLed();   /* ��ʼ��LED */
	//BSP_InitTick(TICK_INT_PRIORITY);//������ʼ��SYSTICK 1ms����һ��

    //NAND_Init();
}
/*
*********************************************************************************************************
*	�� �� ��: BSP_InitTick
*	����˵��: ��ʼ��systickΪ1ms�����ж�һ��,��Ҫ����freertosʱ��׼
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#if 0
static HAL_StatusTypeDef BSP_InitTick(uint32_t TickPriority) 
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: SystemClock_Config
*	����˵��: ��ʼ��ϵͳʱ��
*            	System Clock source            = PLL (HSE)
*            	SYSCLK(Hz)                     = 72000000 (CPU Clock)
*           	HCLK(Hz)                       = 72000000 (AXI and AHBs Clock)
*            	
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#if 0
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: Error_Handler
*	��    ��: file : Դ�����ļ����ơ��ؼ��� __FILE__ ��ʾԴ�����ļ�����
*			  line �������кš��ؼ��� __LINE__ ��ʾԴ�����к�
*	�� �� ֵ: ��
*		Error_Handler();
*********************************************************************************************************
*/
void Error_Handler103(char *file, uint32_t line)
{
    /* 
		�û���������Լ��Ĵ��뱨��Դ�����ļ����ʹ����кţ����罫�����ļ����кŴ�ӡ������
		printf("Wrong parameters value: file %s on line %d\r\n", file, line) 
	*/

    /* ����һ����ѭ��������ʧ��ʱ������ڴ˴��������Ա����û���� */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    if (line == 0)
    {
        return;
    }

    while (1)
    {
    }
}

/*
*********************************************************************************************************
*	�� �� ��: MPU_Config
*	����˵��: ����MPU
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#ifdef STM32H7
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* ��ֹ MPU */
    HAL_MPU_Disable();

    /* ����AXI SRAM��MPU����ΪWrite through, read allocate��no write allocate */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* ����FMC��չIO��MPU����ΪDevice����Strongly Ordered */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x60000000;
    MPU_InitStruct.Size = ARM_MPU_REGION_SIZE_64KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; /* ������MPU_ACCESS_CACHEABLE�������2��CS��WE�ź� */
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* ����NAND Flash */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x80000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512MB; // MPU_REGION_SIZE_512MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /*ʹ�� MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*
*********************************************************************************************************
*	�� �� ��: CPU_CACHE_Enable
*	����˵��: ʹ��L1 Cache
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void CPU_CACHE_Enable(void)
{
    /* ʹ�� I-Cache */
    SCB_EnableICache();

    /* ʹ�� D-Cache */
    SCB_EnableDCache();
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: bsp_RunPer10ms
*	����˵��: �ú���ÿ��10ms��Systick�жϵ���1�Ρ���� bsp_timer.c�Ķ�ʱ�жϷ������һЩ����ʱ��Ҫ���ϸ��
*			������Է��ڴ˺��������磺����ɨ�衢���������п��Ƶȡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_RunPer10ms(void)
{
    bsp_KeyScan10ms();
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_RunPer1ms
*	����˵��: �ú���ÿ��1ms��Systick�жϵ���1�Ρ���� bsp_timer.c�Ķ�ʱ�жϷ������һЩ��Ҫ�����Դ��������
*			 ���Է��ڴ˺��������磺��������ɨ�衣
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_RunPer1ms(void)
{
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_Idle
*	����˵��: ����ʱִ�еĺ�����һ����������for��whileѭ������������Ҫ���� CPU_IDLE() �������ñ�������
*			 ������ȱʡΪ�ղ������û��������ι��������CPU��������ģʽ�Ĺ��ܡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_Idle(void)
{
    /* --- ι�� */

    /* --- ��CPU�������ߣ���Systick��ʱ�жϻ��ѻ��������жϻ��� */

    /* ���� emWin ͼ�ο⣬���Բ���ͼ�ο���Ҫ����ѯ���� */
    //GUI_Exec();

    /* ���� uIP Э�飬���Բ���uip��ѯ���� */
    //TOUCH_CapScan();
}

/*
*********************************************************************************************************
*	�� �� ��: HAL_Delay
*	����˵��: �ض�������ӳٺ������滻HAL�еĺ�������ΪHAL�е�ȱʡ����������Systick�жϣ������USB��SD��
*             �ж������ӳٺ��������������Ҳ����ͨ������HAL_NVIC_SetPriority����Systick�ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/* ��ǰ����ʹ��stm32h7xx_hal.cĬ�Ϸ�ʽʵ�֣�δʹ�������ض���ĺ��� */
#if 0
void HAL_Delay(uint32_t Delay)
{
	bsp_DelayUS(Delay * 1000);
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: SysTick_Handler
*	����˵��: SysTick_Handler�ж�,��Ҫ����freertosʱ��
*	��    ��: 
*	�� �� ֵ: 
*********************************************************************************************************
*/
void SysTick_Handler(void)
{
  if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
    {
        xPortSysTickHandler();	
    }
	//���û��ʹ��HAL���ж�ʱ��,���԰�HALʱ������systickһ��
	//HAL_IncTick();
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
