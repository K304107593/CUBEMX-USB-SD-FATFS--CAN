/************************************************************************** 
 * 模块名称 : 
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\inc\bsp_can_fifo103.h
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-10 23:51:46
 * 说     明: 
 **************************************************************************/
/**************************************************************************
 * 模块名称 :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\inc\bsp_can_fifo103.h
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-06 23:10:51
 * 说     明:
 **************************************************************************/
#ifndef _BSP_CAN_FIFO_H_
#define _BSP_CAN_FIFO_H_

#include "bsp103.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"
//#include "ff.h"

/*
CAN1:
PB9/CAN1_TX
PB8/CAN1_RX

CAN2:待补充

*/
#define CAN1_FIFO_EN 1 //使用CAN1
#define CAN1_AF 1      //使用CAN1引脚映射

#define CAN2_FIFO_EN 0 //使用CAN2
#define CAN2_AF 0      //使用CAN2引脚映射
/* 定义CAN端口号 */
typedef enum
{
    CANCOM1 = 0, /* CAN1 */
    CANCOM2 = 1, /* CAN2 */

} CANCOM_PORT_E;

/* 定义CAN波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if CAN1_FIFO_EN == 1
#define CAN1_BAUD 500000 // 500K
#define CAN1_TX_BUF_SIZE 1
#define CAN1_RX_BUF_SIZE 50
#endif

#if CAN2_FIFO_EN == 1
#define CAN2_BAUD 500000 // 500K
#define CAN2_TX_BUF_SIZE 1 * 1024
#define CAN2_TX_BUF_SIZE 1 * 1024
#endif

/*CAN发送数据结构体*/
typedef struct
{
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t Tx_aData[8];
} CAN_TX_Header;

/*CAN接收结构体*/
typedef struct
{
    CAN_RxHeaderTypeDef CAN_RXHeader;
    uint8_t Rx_aData[8];
} CAN_RX_Header;
typedef enum
{
    RxBufferNum = 0,
    RxBufferNum1
} CanRxBufferNum;
/*CAN设备结构体*/
typedef struct
{
    CAN_TypeDef *CAN;            /*STM32内部CAN设备指针*/
    CAN_TX_Header *CanTxBuffer;  /*发送缓冲区*/
    CAN_RX_Header *CanRxBuffer;  /*接收缓冲区1*/
    CAN_RX_Header *CanRxBuffer1; /*接收缓存区2*/
    uint16_t usTxBufSize;        /*发送缓冲区大小*/
    uint16_t usRxBufSize;        /*接收缓冲区大小1*/
    uint16_t usRxBufSize1;       /*接收缓冲区大小2*/
    __IO uint16_t usTxWrite;     /*发送缓冲区写指针*/
    __IO uint16_t usTxRead;      /*发送缓冲区读指针*/
    __IO uint16_t usTxCount;     /*等待发送的数据个数*/

    __IO uint16_t usRxWrite;      /*接收缓冲区写指针1*/
    __IO uint16_t usRxWrite1;     /*接收缓冲区写指针2*/
    __IO uint16_t usRxRead;       /*接收缓冲区读指针1*/
    __IO uint16_t usRxRead1;      /*接收缓冲区读指针2*/
    __IO uint16_t usRxCount;      /*还未读取的新数据个数1*/
    __IO uint16_t usRxCount1;     /*还未读取的新数据个数2*/
    CanRxBufferNum e_RxBufferNum; /*用来确定接收缓冲区是哪个*/

} CAN_T;

FRESULT canStoreFatfs(uint8_t status);//处理CAN接收的数据
void bsp_InitCan(void);//CANFIFO初始化和过滤器初始化
#endif
