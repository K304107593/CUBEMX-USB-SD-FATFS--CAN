/**************************************************************************
 * 模块名称 :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\test.c
 * Date: 2021-12-05 22:39:24
 * LastEditTime: 2021-12-05 23:34:32
 * 说     明:
 **************************************************************************/
/**************************************************************************
 * 模块名称 :
 * FilePath: \CUBEMX USB+SD+FATFS +CAN\test.c
 * Date: 2021-12-05 22:39:24
 * LastEditTime: 2021-12-05 23:17:45
 * 说     明:
 **************************************************************************/
#include "bsp_can_fifo103.h"

typedef enum
{
    Mask_Id = 0,
    List_Id
} Filter_t;
/*
*********************************************************************************************************
*	函 数 名: Can_Filter_ConfigMask
*	功能说明: 当添加过滤器为列表过滤时,有几个ID,就用几个过滤器,当用添加掩码过滤时,所有ID用一组过滤器
*   当添加的ID是标准ID时,过滤器使用16位模式,当添加的扩展ID时,使用的是32位模式
*	形    参:FilterID:要写入过滤器寄存器的ID地址,FilterNumber:要写入过滤器的ID数目,F_Type:确定是用列表模式还是
*   掩码模式
*	返 回 值: 无
*********************************************************************************************************
*/
static uint32_t G_Rem_Filter; //剩余的过滤器数目,拆分为16位列表模式的数目
static void Can_Filter_ConfigMask(uint32_t *FilterID, uint32_t FilterNumber, Filter_t F_Type)
{
    CAN_FilterTypeDef sFilterConfig;
    if (FilterNumber > 0)
    {
        if (F_Type == List_Id)
        {
            if (*FilterID <= 0X7FF)  //表示是标准ID
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
            else if(*FilterID > 0x7FF)//表示是扩展ID
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
*	函 数 名: Can_Filter_ConfigList
*	功能说明: 列表模式添加过滤器
*	形    参:
*	返 回 值: 无
*********************************************************************************************************
*/
static void Can_Filter_ConfigList(uint32_t *FilterID)
{
}