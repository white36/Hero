#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

// ʹ�ó���CAN
#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define SHOOT_CAN hcan2

// �˲������
#define CAN_FILTER(x) ((x) << 3)

// ���ն���
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// ��׼֡����չ֡
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// ����֡��ң��֡
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* CAN���ͺͽ���ID */
typedef enum
{
    // CAN_CHASSIS_ALL_ID = 0x200,
    // CAN_3508_M1_ID = 0x201,
    // CAN_3508_M2_ID = 0x202,
    // CAN_3508_M3_ID = 0x203,
    // CAN_3508_M4_ID = 0x204,

    // CAN_GIMBAL_ALL_ID = 0x1FF,
    // CAN_YAW_MOTOR_ID = 0x205,
    // CAN_PIT_MOTOR_ID = 0x206,
    // CAN_TRIGGER_MOTOR_ID = 0x207,

    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,//6020
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_PITCH_ALL_ID = 0x2FF,
    CAN_PIT_MOTOR_ID = 0x209,//3508

    // Can2
    CAN_SHOOT_ALL_ID = 0x200,
    CAN_3508_LEFT_ID = 0x202,
    CAN_3508_RIGHT_ID = 0x203,

    CAN_CAP_BUG_ID = 0x220,
    CAN_CAP_CONTROL_ID = 0x219,

} can_msg_id_e;

// ������ݽṹ
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

// /**
//   * @brief          ���͵�����Ƶ��� (0x205, 0x206, 0x207, 0x208)
//   * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000, 30000]
//   * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000, 30000]
//   * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000, 10000]
//   * @param[in]      rev: (0x208) ����������Ƶ���
//   * @retval         none
//   */
// extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

// /**
//   * @brief          ����IDΪ0x700��CAN��������3508���Ϊ����ID����
//   * @param[in]      none
//   * @retval         none
//   */
// extern void CAN_cmd_chassis_reset_ID(void);

// /**
//   * @brief          ���͵�����Ƶ��� (0x201, 0x202, 0x203, 0x204)
//   * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384, 16384]
//   * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384, 16384]
//   * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384, 16384]
//   * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384, 16384]
//   * @retval         none
//   */
// extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          ���� yaw 6020 �������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          ���� pitch 6020 �������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          ���� trigger 2006 �������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          ���ص��̵�� 3508 ����ָ��
  * @param[in]      i: �����ţ���Χ[0, 3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief CAN���յ���Ϣ�ṹ��
 *
 */
extern uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} Struct_CAN_Rx_Buffer;

/**
 * @brief CANͨ�Ž��ջص�������������
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CANͨ�Ź���ṹ��
 *
 */
typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
} Struct_CAN_Manage_Object;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0x3fe_Tx_Data[];
extern uint8_t CAN1_0x4fe_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0x3fe_Tx_Data[];
extern uint8_t CAN2_0x4fe_Tx_Data[];

extern uint8_t CAN_Supercap_Tx_Data[];

/**
 * @brief          CAN��ʼ��
 * @param[in]      hcan: CAN���
 * @param[in]      Callback_Function: ���ջص�����
 * @retval         none
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

/**
 * @brief          CAN�˲�������������
 * @param[in]      hcan: CAN���
 * @param[in]      Object_Para: �������
 * @param[in]      ID: CAN ID
 * @param[in]      Mask_ID: ����ID
 * @retval         none
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief          ��ʱ���ص�����
 * @param[in]      none
 * @retval         none
 */
void TIM_CAN_PeriodElapsedCallback(void);

#endif
