#include "CAN_receive.h"
#include "can.h"

// CAN通信管理对象，分别对应3个CAN总线
Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];  // 超级电容的数据

// 电机数据读取宏
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

// 电机数据结构，7个电机的数据
// static motor_measure_t motor_chassis[7];

static CAN_HandleTypeDef  gimbal_tx_message;  // 云台电机控制的CAN消息
// static uint8_t              gimbal_can_send_data[8];  // 云台控制数据
static CAN_HandleTypeDef  chassis_tx_message;  // 底盘电机控制的CAN消息
// static uint8_t              chassis_can_send_data[8];  // 底盘电机控制数据

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{	
	if(hcan->Instance == CAN1)
	{
		// 配置CAN1
		CAN1_Manage_Object.CAN_Handler = hcan;
		CAN1_Manage_Object.Callback_Function = Callback_Function;
		
		CAN_FilterTypeDef sFilterConfig;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;

		HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  // 配置CAN1的滤波器
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 激活接收FIFO0的中断
		HAL_CAN_Start(hcan);  // 启动CAN1
	}
	else if(hcan->Instance == CAN2)
	{
		// 配置CAN2
		CAN2_Manage_Object.CAN_Handler = hcan;
		CAN2_Manage_Object.Callback_Function = Callback_Function;
		
		CAN_FilterTypeDef sFilterConfig;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;

		HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  // 配置CAN2的滤波器
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 激活接收FIFO0的中断
		HAL_CAN_Start(hcan);  // 启动CAN2
	}
}

/**
  * @brief          CAN通信接收回调函数
  * @param[in]      hcan: CAN句柄
  * @retval         无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief 发送CAN数据
 * @param hcan CAN句柄
 * @param ID 数据ID
 * @param Data 数据内容
 * @param Length 数据长度
 * @retval HAL状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;

    tx_header.DLC = Length;   // 设置数据长度
    tx_header.StdId = ID;     // 设置ID
    tx_header.IDE = CAN_ID_STD;  // 设置ID类型为标准ID
    tx_header.RTR = CAN_RTR_DATA;  // 设置数据帧

    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &send_mail_box);  // 发送数据
}

/**
 * @brief CAN定时器定时中断发送函数
 */
void TIM_CAN_PeriodElapsedCallback()
{
    static int mod10 = 0;

    mod10++;

    // 发送CAN1电机数据
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);

    // 发送CAN2电机数据
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);

    if (mod10 == 10 - 1)
    {
        mod10 = 0;
        // 发送CAN1超级电容数据
        CAN_Send_Data(&hcan1, 0x220, CAN_Supercap_Tx_Data, 8);
    }
}

// /**
//   * @brief          返回yaw 6020电机数据指针
//   * @param[in]      none
//   * @retval         电机数据指针
//   */
// const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
// {
//     return &motor_chassis[4];
// }

// /**
//   * @brief          返回pitch 6020电机数据指针
//   * @param[in]      none
//   * @retval         电机数据指针
//   */
// const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
// {
//     return &motor_chassis[5];
// }


// /**
//   * @brief          返回拨弹电机 2006电机数据指针
//   * @param[in]      none
//   * @retval         电机数据指针
//   */
// const motor_measure_t *get_trigger_motor_measure_point(void)
// {
//     return &motor_chassis[6];
// }


// /**
//   * @brief          返回底盘电机 3508电机数据指针
//   * @param[in]      i: 电机编号,范围[0,3]
//   * @retval         电机数据指针
//   */
// const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
// {
//     return &motor_chassis[(i & 0x03)];
// }
