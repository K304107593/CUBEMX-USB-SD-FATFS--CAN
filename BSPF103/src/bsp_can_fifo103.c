/**************************************************************************
 * ģ������ :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\src\bsp_can_fifo103.c
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-03 13:47:17
 * ˵     ��:
 **************************************************************************/

#include "bsp103.h"
#include "bsp_can_fifo103.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan;     // CAN���ƽṹ��
extern osThreadId StartTaskHandle; //������,��MIAN�ж���

void Can_Filter_Config(void); //��ʼ��CAN������

//���ļ�ϵͳʹ�ñ���//////////////
FATFS SDFatFs;
extern char SDPath[4];

////////////////////////////////

/*CAN1��GPIO, PB8,PB9*/
#define CAN1_CLK_ENABLE() __HAL_RCC_CAN1_CLK_ENABLE()

#if CAN1_AF == 1 //ʹ����CAN����ӳ�� CAN1 TX/PB9 RX/PB8
#define CAN1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN1_TX_GPIO_PORT GPIOE
#define CAN1_TX_PIN GPIO_PIN_9

#define CAN1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN1_RX_GPIO_PORT GPIOE
#define CAN1_RX_PIN GPIO_PIN_8

#else // CAN TX/PD1 RX/PD0
#define CAN1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define CAN1_TX_GPIO_PORT GPIOD
#define CAN1_TX_PIN GPIO_PIN_1

#define CAN1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define CAN1_RX_GPIO_PORT GPIOD
#define CAN1_RX_PIN GPIO_PIN_0
#endif

/******************������,103ֻ��һ��CAN********************************/
#if CAN2_FIFO_EN == 1
/*CAN1��GPIO, PB8,PB9*/
#define CAN2_CLK_ENABLED __HAL_RCC_CAN2_CLK_ENABLE();

#define CAN_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN2_TX_GPIO_PORT GPIOE
#define CAN2_TX_PIN GPIO_PIN_9

#define CAN2_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN2_RX_GPIO_PORT GPIOE
#define CAN2_RX_PIN GPIO_PIN_8
#endif
/*����CAN1�ṹ�����*/
#if CAN1_FIFO_EN == 1

static CAN_T g_tCAN1;
static CAN_TX_Header g_TxBuf1[CAN1_TX_BUF_SIZE];
static CAN_RX_Header g_RxBuf[CAN1_RX_BUF_SIZE];  //��������CAN1���յ�����1
static CAN_RX_Header g_RxBuf1[CAN1_RX_BUF_SIZE]; //��������CAN1���յ�����2

#endif

/*����CAN2�ṹ�����*/
#if CAN2_FIFO_EN == 1
static CAN_T g_tCAN2;
static CAN_TX_Header g_TxBuf2[CAN2_TX_BUF_SIZE];
static CAN_RX_Header g_RxBuf2[CAN2_RX_BUF_SIZE];

#endif

void CanVarInit(void);
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitCan
*	����˵��: ��ʼ��CANӲ��,���Ծֱ�����ֵ.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitCan(void)
{
    CanVarInit(); //��ʼ��CAN��صı���,����FIFO
    Can_Filter_Config();
}

/*
*********************************************************************************************************
*	�� �� ��: CanVarInit
*	����˵��: ��ʼ��CAN��صı���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void CanVarInit(void)
{
#if CAN1_FIFO_EN == 1
    g_tCAN1.CAN = CAN1;                      /*CAN�豸*/
    g_tCAN1.CanTxBuffer = g_TxBuf1;          /*���ͻ�����ָ��*/
    g_tCAN1.CanRxBuffer = g_RxBuf;           /*���ջ�����ָ��1*/
    g_tCAN1.CanRxBuffer1 = g_RxBuf1;         /*���ջ�����ָ��2*/
    g_tCAN1.usTxBufSize = CAN1_TX_BUF_SIZE;  /*���ͻ�������С*/
    g_tCAN1.usRxBufSize = CAN1_RX_BUF_SIZE;  /*���ջ�������С1*/
    g_tCAN1.usRxBufSize1 = CAN1_RX_BUF_SIZE; /*���ջ�������С2*/
    g_tCAN1.usTxWrite = 0;                   /*����FIFOд����*/
    g_tCAN1.usTxRead = 0;                    /*����FIFO������*/
    g_tCAN1.usRxWrite = 0;                   /*����FIFOд����*/
    g_tCAN1.usRxRead = 0;                    /*����FIFO������1*/
    g_tCAN1.usRxRead1 = 0;                   /*����FIFO������2*/
    g_tCAN1.usRxCount = 0;                   /*���յ������ݸ���1*/
    g_tCAN1.usRxCount1 = 0;                  /*���յ������ݸ���2*/
    g_tCAN1.usTxCount = 0;                   /*�����͵������ݸ���*/
    g_tCAN1.e_RxBufferNum = RxBufferNum;     /*ʹ���ĸ����ջ�����*/
#endif
#if CAN2_FIFO_EN == 1
    g_tCAN2.CAN = CAN2;                     /*CAN�豸*/
    g_tCAN2.CanTxBuffer = g_TxBuf1;         /*���ͻ�����ָ��*/
    g_tCAN2.CanRxBuffer = g_RxBuf1;         /*���ջ�����ָ��*/
    g_tCAN2.usTxBufSize = CAN2_TX_BUF_SIZE; /*���ͻ�������С*/
    g_tCAN2.usRxBufSize = CAN2_RX_BUF_SIZE; /*���ջ�������С*/
    g_tCAN2.usTxWrite = 0;                  /*����FIFOд����*/
    g_tCAN2.usTxRead = 0;                   /*����FIFO������*/
    g_tCAN2.usRxWrite = 0;                  /*����FIFOд����*/
    g_tCAN2.usRxRead = 0;                   /*����FIFO������*/
    g_tCAN2.usRxCount = 0;                  /*���յ������ݸ���*/
    g_tCAN2.usTxCount = 0;                  /*�����͵������ݸ���*/
#endif
}
#if 0
/*
*********************************************************************************************************
*	�� �� ��: InitHardUart
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitHardCan(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
#if CAN1_FIFO_EN == 1 /*CAN1*/
    /*ʹ�� GPIO TX/RX ʱ��*/
    CAN1_TX_GPIO_CLK_ENABLE();
    CAN1_RX_GPIO_CLK_ENABLE();

    /*ʹ�� CAN ʱ��*/
    CAN1_CLK_ENABLE();

    /*����CAN���� */
    GPIO_InitStruct.Pin = CAN1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

#if (CAN1_AF == 1)
    /*��ӳ��CAN1���ŵ�PE8��PE9*/
    __HAL_AFIO_REMAP_CAN1_2();
#endif

    /*����NVIC for CAN1 RX0�ж�*/
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    /*���ò�����,���ù���������*/
    // bsp_SetCanParam
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetCanParam
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32- H7������
*	��    ��: Instance   USART_TypeDef���ͽṹ��
*             BaudRate   ������
*             Parity     У�����ͣ���У�����żУ��
*             Mode       ���ͺͽ���ģʽʹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetCanParam(CAN_HandleTypeDef *hcan, uint32_t BaudRate)
{
}
/*
*********************************************************************************************************
*	�� �� ��: InitCanfilter
*	����˵��: ��ʼ��CAN������
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitCanfilter(CAN_HandleTypeDef *hcan)
{
    static uint8_t Can1FilterNum = 0, Can2FilterNum = 14;
    CAN_FilterTypeDef CANfil;

    if (hcan->Instance == CAN1)
    {
        CANfil.FilterBank = 0;
        CANfil.FilterMode = CAN_FILTERMODE_IDMASK;
        CANfil.FilterScale = CAN_FILTERSCALE_32BIT;
        CANfil.FilterIdHigh = 0;
        CANfil.FilterIdLow = 0;
        CANfil.FilterMaskIdHigh = 0;
        CANfil.FilterMaskIdLow = 0;
        CANfil.FilterFIFOAssignment = CAN_RX_FIFO0;
        CANfil.FilterActivation = ENABLE;
        CANfil.SlaveStartFilterBank = 14;
    }

#if CAN2_FIFO_EN == 1
    else if (hcan->Instance == CAN2)
    {
        CANfil.FilterBank = 15;
        CANfil.FilterMode = CAN_FILTERMODE_IDMASK;
        CANfil.FilterScale = CAN_FILTERSCALE_32BIT;
        CANfil.FilterIdHigh = 0;
        CANfil.FilterIdLow = 0;
        CANfil.FilterMaskIdHigh = 0;
        CANfil.FilterMaskIdLow = 0;
        CANfil.FilterFIFOAssignment = CAN_RX_FIFO0;
        CANfil.FilterActivation = ENABLE;
        CANfil.SlaveStartFilterBank = 14;
    }
#endif
    if (HAL_CAN_ConfigFilter(hcan, &CANfil) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}

/*�������,��׼CAN����չCANͬʱ��Ϲ���*/
 static void CANFilterConfig_Scale32_IdMask_StandardId_ExtendId_Mix(void)
{
    
  CAN_FilterTypeDef  sFilterConfig;
  //����һ���׼CAN ID
uint32_t StdIdArray[10] ={0x711,0x712,0x713,0x714,0x715,
                          0x716,0x717,0x718,0x719,0x71a};
  //��������һ����չCAN ID
uint32_t ExtIdArray[10] ={0x1900fAB1,0x1900fAB2,0x1900fAB3,0x1900fAB4,0x1900fAB5,
                            0x1900fAB6,0x1900fAB7,0x1900fAB8,0x1900fAB9,0x1900fABA};
  uint32_t      mask,num,tmp,i,standard_mask,extend_mask,mix_mask;
  
  sFilterConfig.FilterNumber = 4;				//ʹ�ù�����4
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		//����Ϊ����ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	//��Ϊ32λ��
  sFilterConfig.FilterIdHigh =((ExtIdArray[0]<<3) >>16) &0xffff;	//ʹ�õ�һ����չCAN  ID��Ϊ��֤��
  sFilterConfig.FilterIdLow =((ExtIdArray[0]<<3)&0xffff);
  
  standard_mask =0x7ff;		//�����Ǽ���������
  num =sizeof(StdIdArray)/sizeof(StdIdArray[0]);
  for(i =0; i<num; i++)			//���ȼ�������б�׼CAN ID��������
  {
    tmp =StdIdArray[i] ^ (~StdIdArray[0]);
    standard_mask &=tmp;
  }
  
  extend_mask =0x1fffffff;
  num =sizeof(ExtIdArray)/sizeof(ExtIdArray[0]);
  for(i =0; i<num; i++)			//���ż����������չCAN ID��������
  {
    tmp =ExtIdArray[i] ^ (~ExtIdArray[0]);
    extend_mask &=tmp;
  }
  mix_mask =(StdIdArray[0]<<18)^ (~ExtIdArray[0]);	//�ټ����׼CAN ID����չCAN ID��ϵ�������
  mask =(standard_mask<<18)& extend_mask &mix_mask;	//���������յ�������
  mask <<=3;    						//����Ĵ���
 
  sFilterConfig.FilterMaskIdHigh = (mask>>16)&0xffff;
  sFilterConfig.FilterMaskIdLow = (mask&0xffff);
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: Can_Filter_Config
*	����˵��:  ��
ʼ��CAN������,
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void Can_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //����ģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ����ģʽ
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //����0
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox1CompleteCallback
*	����˵��:  ����1������ɻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
FRESULT canStoreFatfs(uint8_t status)
{
    FRESULT FatfsStatus = FR_INVALID_PARAMETER;
    FIL FIL_file;
    FatfsStatus = f_mount(&SDFatFs, SDPath, 0);
    if (FatfsStatus)
    {
        debug_printf("Fmount����ʧ��:%d\n", FatfsStatus);
        if (FatfsStatus == FR_NO_FILESYSTEM)
        {
            debug_printf("û���ļ�ϵͳ:��ʽ���洢�豸");
            if (f_mkfs((TCHAR const *)SDPath, 0, 0) != FR_OK)
            {
                /* FatFs Format Error */
                Error_Handler();
            }
        }
        
    }
    debug_printf("Fmount�����ɹ�:%d\n", FatfsStatus);


    if (status == 0) //��ʾ��ȷ���յ���־λ
    {
        if (g_tCAN1.e_RxBufferNum == RxBufferNum1) // CANBUFF1������Ҫ����
        {
            //f_open()
        }
        else if (g_tCAN1.e_RxBufferNum == RxBufferNum) // CANBUFF2������Ҫ����
        {
        }
    }
    else if (status == 1) //��ʾ����5Sû�н��յ���־λ
    {
        f_open(&FIL_file, "STM32TE.txt", FA_OPEN_ALWAYS);
        
    }
    return FatfsStatus;
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox0CompleteCallback
*	����˵��:  ����0������ɻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (g_tCAN1.e_RxBufferNum == RxBufferNum)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(g_tCAN1.CanRxBuffer[g_tCAN1.usRxWrite].CAN_RXHeader), g_tCAN1.CanRxBuffer[g_tCAN1.usRxWrite].Rx_aData);
        if (++g_tCAN1.usRxWrite >= g_tCAN1.usRxBufSize)
        {
            g_tCAN1.usRxWrite = 0;
            g_tCAN1.e_RxBufferNum = RxBufferNum1;
            osSignalSet(StartTaskHandle, 0x01); //֪ͨ������һ������������
        }
        if (g_tCAN1.usRxCount < g_tCAN1.usRxBufSize)
        {
            g_tCAN1.usRxCount++;
        }
    }
    else if (g_tCAN1.e_RxBufferNum == RxBufferNum1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(g_tCAN1.CanRxBuffer1[g_tCAN1.usRxWrite1].CAN_RXHeader), g_tCAN1.CanRxBuffer1[g_tCAN1.usRxWrite1].Rx_aData);
        if (++g_tCAN1.usRxWrite1 >= g_tCAN1.usRxBufSize1)
        {
            g_tCAN1.usRxWrite1 = 0;
            g_tCAN1.e_RxBufferNum = RxBufferNum;
            osSignalSet(StartTaskHandle, 0x01); //֪ͨ������һ������������
        }
        if (g_tCAN1.usRxCount1 < g_tCAN1.usRxBufSize1)
        {
            g_tCAN1.usRxCount1++;
        }
    }
    else
    {
        Error_Handler();
    }
}

/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox1CompleteCallback
*	����˵��:  ����1������ɻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox2CompleteCallback
*	����˵��:  ����2������ɻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox0AbortCallback
*	����˵��:  ����0����ʧ�ܻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox1AbortCallback
*	����˵��:  ����1����ʧ�ܻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_TxMailbox2AbortCallback
*	����˵��:  ����2����ʧ�ܻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_RxFifo0MsgPendingCallback
*	����˵��:  ����0���յ���Ϣ�ж�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_RxFifo0FullCallback
*	����˵��:  ����0��������Ϣ�ж�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_RxFifo1MsgPendingCallback
*	����˵��:  ����1���յ���Ϣ�ж�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_RxFifo1MsgPendingCallback
*	����˵��:  ����1��������Ϣ�ж�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_SleepCallback
*	����˵��:  CAN˯���жϻص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_WakeUpFromRxMsgCallback
*	����˵��:  ������Ϣ�����ж�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	�� �� ��: HAL_CAN_ErrorCallback
*	����˵��:  ������Ϣ�ص�
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
}
