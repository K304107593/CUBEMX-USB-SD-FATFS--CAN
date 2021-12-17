/**************************************************************************
 * 模块名称 :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\src\bsp_can_fifo103.c
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-03 13:47:17
 * 说     明:
 **************************************************************************/

#include "bsp103.h"
#include "bsp_can_fifo103.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan;     // CAN控制结构体
extern osThreadId StartTaskHandle; //任务句柄,在MIAN中定义

void Can_Filter_Config(void); //初始化CAN过滤器

//给文件系统使用变量//////////////
FATFS SDFatFs;
extern char SDPath[4];

////////////////////////////////

/*CAN1的GPIO, PB8,PB9*/
#define CAN1_CLK_ENABLE() __HAL_RCC_CAN1_CLK_ENABLE()

#if CAN1_AF == 1 //使用了CAN引脚映射 CAN1 TX/PB9 RX/PB8
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

/******************待补充,103只有一个CAN********************************/
#if CAN2_FIFO_EN == 1
/*CAN1的GPIO, PB8,PB9*/
#define CAN2_CLK_ENABLED __HAL_RCC_CAN2_CLK_ENABLE();

#define CAN_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN2_TX_GPIO_PORT GPIOE
#define CAN2_TX_PIN GPIO_PIN_9

#define CAN2_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define CAN2_RX_GPIO_PORT GPIOE
#define CAN2_RX_PIN GPIO_PIN_8
#endif
/*定义CAN1结构体变量*/
#if CAN1_FIFO_EN == 1

static CAN_T g_tCAN1;
static CAN_TX_Header g_TxBuf1[CAN1_TX_BUF_SIZE];
static CAN_RX_Header g_RxBuf[CAN1_RX_BUF_SIZE];  //用来缓存CAN1接收的数据1
static CAN_RX_Header g_RxBuf1[CAN1_RX_BUF_SIZE]; //用来缓存CAN1接收的数据2

#endif

/*定义CAN2结构体变量*/
#if CAN2_FIFO_EN == 1
static CAN_T g_tCAN2;
static CAN_TX_Header g_TxBuf2[CAN2_TX_BUF_SIZE];
static CAN_RX_Header g_RxBuf2[CAN2_RX_BUF_SIZE];

#endif

void CanVarInit(void);
/*
*********************************************************************************************************
*	函 数 名: bsp_InitCan
*	功能说明: 初始化CAN硬件,并对局变量赋值.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitCan(void)
{
    CanVarInit(); //初始化CAN相关的变量,用来FIFO
    Can_Filter_Config();
}

/*
*********************************************************************************************************
*	函 数 名: CanVarInit
*	功能说明: 初始化CAN相关的变量
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void CanVarInit(void)
{
#if CAN1_FIFO_EN == 1
    g_tCAN1.CAN = CAN1;                      /*CAN设备*/
    g_tCAN1.CanTxBuffer = g_TxBuf1;          /*发送缓冲区指针*/
    g_tCAN1.CanRxBuffer = g_RxBuf;           /*接收缓冲区指针1*/
    g_tCAN1.CanRxBuffer1 = g_RxBuf1;         /*接收缓冲区指针2*/
    g_tCAN1.usTxBufSize = CAN1_TX_BUF_SIZE;  /*发送缓冲区大小*/
    g_tCAN1.usRxBufSize = CAN1_RX_BUF_SIZE;  /*接收缓冲区大小1*/
    g_tCAN1.usRxBufSize1 = CAN1_RX_BUF_SIZE; /*接收缓冲区大小2*/
    g_tCAN1.usTxWrite = 0;                   /*发送FIFO写索引*/
    g_tCAN1.usTxRead = 0;                    /*发送FIFO读索引*/
    g_tCAN1.usRxWrite = 0;                   /*接收FIFO写索引*/
    g_tCAN1.usRxRead = 0;                    /*接收FIFO读索引1*/
    g_tCAN1.usRxRead1 = 0;                   /*接收FIFO读索引2*/
    g_tCAN1.usRxCount = 0;                   /*接收的新数据个数1*/
    g_tCAN1.usRxCount1 = 0;                  /*接收的新数据个数2*/
    g_tCAN1.usTxCount = 0;                   /*待发送的新数据个数*/
    g_tCAN1.e_RxBufferNum = RxBufferNum;     /*使用哪个接收缓冲区*/
#endif
#if CAN2_FIFO_EN == 1
    g_tCAN2.CAN = CAN2;                     /*CAN设备*/
    g_tCAN2.CanTxBuffer = g_TxBuf1;         /*发送缓冲区指针*/
    g_tCAN2.CanRxBuffer = g_RxBuf1;         /*接收缓冲区指针*/
    g_tCAN2.usTxBufSize = CAN2_TX_BUF_SIZE; /*发送缓冲区大小*/
    g_tCAN2.usRxBufSize = CAN2_RX_BUF_SIZE; /*接收缓冲区大小*/
    g_tCAN2.usTxWrite = 0;                  /*发送FIFO写索引*/
    g_tCAN2.usTxRead = 0;                   /*发送FIFO读索引*/
    g_tCAN2.usRxWrite = 0;                  /*接收FIFO写索引*/
    g_tCAN2.usRxRead = 0;                   /*接收FIFO读索引*/
    g_tCAN2.usRxCount = 0;                  /*接收的新数据个数*/
    g_tCAN2.usTxCount = 0;                  /*待发送的新数据个数*/
#endif
}
#if 0
/*
*********************************************************************************************************
*	函 数 名: InitHardUart
*	功能说明: 配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void InitHardCan(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
#if CAN1_FIFO_EN == 1 /*CAN1*/
    /*使能 GPIO TX/RX 时钟*/
    CAN1_TX_GPIO_CLK_ENABLE();
    CAN1_RX_GPIO_CLK_ENABLE();

    /*使能 CAN 时钟*/
    CAN1_CLK_ENABLE();

    /*配置CAN引脚 */
    GPIO_InitStruct.Pin = CAN1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

#if (CAN1_AF == 1)
    /*重映射CAN1引脚到PE8和PE9*/
    __HAL_AFIO_REMAP_CAN1_2();
#endif

    /*配置NVIC for CAN1 RX0中断*/
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    /*配置波特率,设置过滤器配置*/
    // bsp_SetCanParam
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetCanParam
*	功能说明: 配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32- H7开发板
*	形    参: Instance   USART_TypeDef类型结构体
*             BaudRate   波特率
*             Parity     校验类型，奇校验或者偶校验
*             Mode       发送和接收模式使能
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetCanParam(CAN_HandleTypeDef *hcan, uint32_t BaudRate)
{
}
/*
*********************************************************************************************************
*	函 数 名: InitCanfilter
*	功能说明: 初始化CAN过滤器
*	形    参:
*	返 回 值: 无
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

/*掩码过滤,标准CAN和扩展CAN同时混合过滤*/
 static void CANFilterConfig_Scale32_IdMask_StandardId_ExtendId_Mix(void)
{
    
  CAN_FilterTypeDef  sFilterConfig;
  //定义一组标准CAN ID
uint32_t StdIdArray[10] ={0x711,0x712,0x713,0x714,0x715,
                          0x716,0x717,0x718,0x719,0x71a};
  //定义另外一组扩展CAN ID
uint32_t ExtIdArray[10] ={0x1900fAB1,0x1900fAB2,0x1900fAB3,0x1900fAB4,0x1900fAB5,
                            0x1900fAB6,0x1900fAB7,0x1900fAB8,0x1900fAB9,0x1900fABA};
  uint32_t      mask,num,tmp,i,standard_mask,extend_mask,mix_mask;
  
  sFilterConfig.FilterNumber = 4;				//使用过滤器4
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		//配置为掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	//设为32位宽
  sFilterConfig.FilterIdHigh =((ExtIdArray[0]<<3) >>16) &0xffff;	//使用第一个扩展CAN  ID作为验证码
  sFilterConfig.FilterIdLow =((ExtIdArray[0]<<3)&0xffff);
  
  standard_mask =0x7ff;		//下面是计算屏蔽码
  num =sizeof(StdIdArray)/sizeof(StdIdArray[0]);
  for(i =0; i<num; i++)			//首先计算出所有标准CAN ID的屏蔽码
  {
    tmp =StdIdArray[i] ^ (~StdIdArray[0]);
    standard_mask &=tmp;
  }
  
  extend_mask =0x1fffffff;
  num =sizeof(ExtIdArray)/sizeof(ExtIdArray[0]);
  for(i =0; i<num; i++)			//接着计算出所有扩展CAN ID的屏蔽码
  {
    tmp =ExtIdArray[i] ^ (~ExtIdArray[0]);
    extend_mask &=tmp;
  }
  mix_mask =(StdIdArray[0]<<18)^ (~ExtIdArray[0]);	//再计算标准CAN ID与扩展CAN ID混合的屏蔽码
  mask =(standard_mask<<18)& extend_mask &mix_mask;	//最后计算最终的屏蔽码
  mask <<=3;    						//对齐寄存器
 
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
*	函 数 名: Can_Filter_Config
*	功能说明:  初
始化CAN过滤器,
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
static void Can_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位掩码模式
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //邮箱0
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
*	函 数 名: HAL_CAN_TxMailbox1CompleteCallback
*	功能说明:  邮箱1传输完成回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
FRESULT canStoreFatfs(uint8_t status)
{
    FRESULT FatfsStatus = FR_INVALID_PARAMETER;
    FIL FIL_file;
    FatfsStatus = f_mount(&SDFatFs, SDPath, 0);
    if (FatfsStatus)
    {
        debug_printf("Fmount创建失败:%d\n", FatfsStatus);
        if (FatfsStatus == FR_NO_FILESYSTEM)
        {
            debug_printf("没有文件系统:格式化存储设备");
            if (f_mkfs((TCHAR const *)SDPath, 0, 0) != FR_OK)
            {
                /* FatFs Format Error */
                Error_Handler();
            }
        }
        
    }
    debug_printf("Fmount创建成功:%d\n", FatfsStatus);


    if (status == 0) //表示正确接收到标志位
    {
        if (g_tCAN1.e_RxBufferNum == RxBufferNum1) // CANBUFF1数据需要处理
        {
            //f_open()
        }
        else if (g_tCAN1.e_RxBufferNum == RxBufferNum) // CANBUFF2数据需要处理
        {
        }
    }
    else if (status == 1) //表示超过5S没有接收到标志位
    {
        f_open(&FIL_file, "STM32TE.txt", FA_OPEN_ALWAYS);
        
    }
    return FatfsStatus;
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_TxMailbox0CompleteCallback
*	功能说明:  邮箱0传输完成回调
*	形    参:
*	返 回 值: 无
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
            osSignalSet(StartTaskHandle, 0x01); //通知任务有一个缓存器已满
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
            osSignalSet(StartTaskHandle, 0x01); //通知任务有一个缓存器已满
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
*	函 数 名: HAL_CAN_TxMailbox1CompleteCallback
*	功能说明:  邮箱1传输完成回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_TxMailbox2CompleteCallback
*	功能说明:  邮箱2传输完成回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_TxMailbox0AbortCallback
*	功能说明:  邮箱0传输失败回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_TxMailbox1AbortCallback
*	功能说明:  邮箱1传输失败回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_TxMailbox2AbortCallback
*	功能说明:  邮箱2传输失败回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
*	功能说明:  邮箱0接收到消息中断
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_RxFifo0FullCallback
*	功能说明:  邮箱0接收满消息中断
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_RxFifo1MsgPendingCallback
*	功能说明:  邮箱1接收到消息中断
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}

/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_RxFifo1MsgPendingCallback
*	功能说明:  邮箱1接收满消息中断
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_SleepCallback
*	功能说明:  CAN睡眠中断回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_WakeUpFromRxMsgCallback
*	功能说明:  接收消息唤醒中断
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
}
/*
*********************************************************************************************************
*	函 数 名: HAL_CAN_ErrorCallback
*	功能说明:  错误消息回调
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
}
