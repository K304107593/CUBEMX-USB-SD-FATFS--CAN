/************************************************************************** 
 * ģ������ : 
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\inc\bsp_can_fifo103.h
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-10 23:51:46
 * ˵     ��: 
 **************************************************************************/
/**************************************************************************
 * ģ������ :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\BSPF103\inc\bsp_can_fifo103.h
 * Date: 2021-12-02 19:04:30
 * LastEditTime: 2021-12-06 23:10:51
 * ˵     ��:
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

CAN2:������

*/
#define CAN1_FIFO_EN 1 //ʹ��CAN1
#define CAN1_AF 1      //ʹ��CAN1����ӳ��

#define CAN2_FIFO_EN 0 //ʹ��CAN2
#define CAN2_AF 0      //ʹ��CAN2����ӳ��
/* ����CAN�˿ں� */
typedef enum
{
    CANCOM1 = 0, /* CAN1 */
    CANCOM2 = 1, /* CAN2 */

} CANCOM_PORT_E;

/* ����CAN�����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
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

/*CAN�������ݽṹ��*/
typedef struct
{
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t Tx_aData[8];
} CAN_TX_Header;

/*CAN���սṹ��*/
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
/*CAN�豸�ṹ��*/
typedef struct
{
    CAN_TypeDef *CAN;            /*STM32�ڲ�CAN�豸ָ��*/
    CAN_TX_Header *CanTxBuffer;  /*���ͻ�����*/
    CAN_RX_Header *CanRxBuffer;  /*���ջ�����1*/
    CAN_RX_Header *CanRxBuffer1; /*���ջ�����2*/
    uint16_t usTxBufSize;        /*���ͻ�������С*/
    uint16_t usRxBufSize;        /*���ջ�������С1*/
    uint16_t usRxBufSize1;       /*���ջ�������С2*/
    __IO uint16_t usTxWrite;     /*���ͻ�����дָ��*/
    __IO uint16_t usTxRead;      /*���ͻ�������ָ��*/
    __IO uint16_t usTxCount;     /*�ȴ����͵����ݸ���*/

    __IO uint16_t usRxWrite;      /*���ջ�����дָ��1*/
    __IO uint16_t usRxWrite1;     /*���ջ�����дָ��2*/
    __IO uint16_t usRxRead;       /*���ջ�������ָ��1*/
    __IO uint16_t usRxRead1;      /*���ջ�������ָ��2*/
    __IO uint16_t usRxCount;      /*��δ��ȡ�������ݸ���1*/
    __IO uint16_t usRxCount1;     /*��δ��ȡ�������ݸ���2*/
    CanRxBufferNum e_RxBufferNum; /*����ȷ�����ջ��������ĸ�*/

} CAN_T;

FRESULT canStoreFatfs(uint8_t status);//����CAN���յ�����
void bsp_InitCan(void);//CANFIFO��ʼ���͹�������ʼ��
#endif
