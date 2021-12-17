/**************************************************************************
 * ģ������ :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\test.c
 * Date: 2021-12-05 22:39:24
 * LastEditTime: 2021-12-05 23:34:32
 * ˵     ��:
 **************************************************************************/
/**************************************************************************
 * ģ������ :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\test.c
 * Date: 2021-12-05 22:39:24
 * LastEditTime: 2021-12-05 23:17:45
 * ˵     ��:
 **************************************************************************/
#include "bsp_can_fifo103.h"

typedef enum
{
    Mask_Id = 0,
    List_Id
} Filter_t;
/*
*********************************************************************************************************
*	�� �� ��: Can_Filter_ConfigMask
*	����˵��: ����ӹ�����Ϊ�б����ʱ,�м���ID,���ü���������,��������������ʱ,����ID��һ�������
*   ����ӵ�ID�Ǳ�׼IDʱ,������ʹ��16λģʽ,����ӵ���չIDʱ,ʹ�õ���32λģʽ
*	��    ��:FilterID:Ҫд��������Ĵ�����ID��ַ,FilterNumber:Ҫд���������ID��Ŀ,F_Type:ȷ�������б�ģʽ����
*   ����ģʽ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint32_t G_Rem_Filter; //ʣ��Ĺ�������Ŀ,���Ϊ16λ�б�ģʽ����Ŀ
static void Can_Filter_ConfigMask(uint32_t *FilterID, uint32_t FilterNumber, Filter_t F_Type)
{
    CAN_FilterTypeDef sFilterConfig;
    if (FilterNumber > 0)
    {
        if (F_Type == List_Id)
        {
            if (*FilterID <= 0X7FF)  //��ʾ�Ǳ�׼ID
            {
                sFilterConfig.FilterBank = 0;
                sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
                sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
                sFilterConfig.FilterIdHigh = 0;
                sFilterConfig.FilterIdLow = 0;
                sFilterConfig.FilterMaskIdHigh = 0;
                sFilterConfig.FilterMaskIdLow = 0;
                sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
                sFilterConfig.FilterActivation = ENABLE;
                sFilterConfig.SlaveStartFilterBank = 14;
            }
            else if(*FilterID > 0x7FF)//��ʾ����չID
            {
                
            }
        }
        else if (F_Type == Mask_Id)
        {
        }
    }
}

/*
*********************************************************************************************************
*	�� �� ��: Can_Filter_ConfigList
*	����˵��: �б�ģʽ��ӹ�����
*	��    ��:
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void Can_Filter_ConfigList(uint32_t *FilterID)
{
}