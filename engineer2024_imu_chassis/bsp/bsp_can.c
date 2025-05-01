#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;//使能
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式CAN_FILTERMODE_IDLIST或列表模式CAN_FILTERMODE_IDMASK
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32位或16位
    can_filter_st.FilterIdHigh = 0x0000;           //CAN_FiR1寄存器的高16位     存储要筛选的ID
    can_filter_st.FilterIdLow = 0x0000;             //CAN_FiR1寄存器的低16位
    can_filter_st.FilterMaskIdHigh = 0x0000;      //CAN_FiR2寄存器的高16位    掩码模式为掩码
    can_filter_st.FilterMaskIdLow = 0x0000;       //CAN_FiR2寄存器的低16位		列表模式为要筛选的ID
    can_filter_st.FilterBank = 0;                //此次配置用的是哪个筛选器。用单CAN的取值为0-13
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//通过筛选器的报文存在FIFO0还是FIFO1中
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//开启中断


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);


}
