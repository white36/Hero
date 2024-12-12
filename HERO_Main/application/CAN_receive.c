#include "CAN_receive.h"
#include "can.h"

// CANͨ�Ź�����󣬷ֱ��Ӧ3��CAN����
Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN���ͻ�����
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

uint8_t CAN_Supercap_Tx_Data[8];  // �������ݵ�����

// ������ݶ�ȡ��
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

// ������ݽṹ��7�����������
// static motor_measure_t motor_chassis[7];

static CAN_HandleTypeDef  gimbal_tx_message;  // ��̨������Ƶ�CAN��Ϣ
// static uint8_t              gimbal_can_send_data[8];  // ��̨��������
static CAN_HandleTypeDef  chassis_tx_message;  // ���̵�����Ƶ�CAN��Ϣ
// static uint8_t              chassis_can_send_data[8];  // ���̵����������

/**
 * @brief ��ʼ��CAN����
 *
 * @param hcan CAN���
 * @param Callback_Function ����ص�����
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{	
	if(hcan->Instance == CAN1)
	{
		// ����CAN1
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

		HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  // ����CAN1���˲���
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // �������FIFO0���ж�
		HAL_CAN_Start(hcan);  // ����CAN1
	}
	else if(hcan->Instance == CAN2)
	{
		// ����CAN2
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

		HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  // ����CAN2���˲���
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // �������FIFO0���ж�
		HAL_CAN_Start(hcan);  // ����CAN2
	}
}

/**
  * @brief          CANͨ�Ž��ջص�����
  * @param[in]      hcan: CAN���
  * @retval         ��
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
 * @brief ����CAN����
 * @param hcan CAN���
 * @param ID ����ID
 * @param Data ��������
 * @param Length ���ݳ���
 * @retval HAL״̬
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;

    tx_header.DLC = Length;   // �������ݳ���
    tx_header.StdId = ID;     // ����ID
    tx_header.IDE = CAN_ID_STD;  // ����ID����Ϊ��׼ID
    tx_header.RTR = CAN_RTR_DATA;  // ��������֡

    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &send_mail_box);  // ��������
}

/**
 * @brief CAN��ʱ����ʱ�жϷ��ͺ���
 */
void TIM_CAN_PeriodElapsedCallback()
{
    static int mod10 = 0;

    mod10++;

    // ����CAN1�������
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);

    // ����CAN2�������
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);

    if (mod10 == 10 - 1)
    {
        mod10 = 0;
        // ����CAN1������������
        CAN_Send_Data(&hcan1, 0x220, CAN_Supercap_Tx_Data, 8);
    }
}

// /**
//   * @brief          ����yaw 6020�������ָ��
//   * @param[in]      none
//   * @retval         �������ָ��
//   */
// const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
// {
//     return &motor_chassis[4];
// }

// /**
//   * @brief          ����pitch 6020�������ָ��
//   * @param[in]      none
//   * @retval         �������ָ��
//   */
// const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
// {
//     return &motor_chassis[5];
// }


// /**
//   * @brief          ���ز������ 2006�������ָ��
//   * @param[in]      none
//   * @retval         �������ָ��
//   */
// const motor_measure_t *get_trigger_motor_measure_point(void)
// {
//     return &motor_chassis[6];
// }


// /**
//   * @brief          ���ص��̵�� 3508�������ָ��
//   * @param[in]      i: ������,��Χ[0,3]
//   * @retval         �������ָ��
//   */
// const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
// {
//     return &motor_chassis[(i & 0x03)];
// }
