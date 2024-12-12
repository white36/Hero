#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

// 使用常规CAN
#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define SHOOT_CAN hcan2

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* CAN发送和接收ID */
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

// 电机数据结构
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

// /**
//   * @brief          发送电机控制电流 (0x205, 0x206, 0x207, 0x208)
//   * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000, 30000]
//   * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000, 30000]
//   * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000, 10000]
//   * @param[in]      rev: (0x208) 保留电机控制电流
//   * @retval         none
//   */
// extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

// /**
//   * @brief          发送ID为0x700的CAN包，设置3508电机为快速ID设置
//   * @param[in]      none
//   * @retval         none
//   */
// extern void CAN_cmd_chassis_reset_ID(void);

// /**
//   * @brief          发送电机控制电流 (0x201, 0x202, 0x203, 0x204)
//   * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384, 16384]
//   * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384, 16384]
//   * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384, 16384]
//   * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384, 16384]
//   * @retval         none
//   */
// extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          返回 yaw 6020 电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回 pitch 6020 电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回 trigger 2006 电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回底盘电机 3508 数据指针
  * @param[in]      i: 电机编号，范围[0, 3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief CAN接收的信息结构体
 *
 */
extern uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信管理结构体
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
 * @brief          CAN初始化
 * @param[in]      hcan: CAN句柄
 * @param[in]      Callback_Function: 接收回调函数
 * @retval         none
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

/**
 * @brief          CAN滤波器和掩码配置
 * @param[in]      hcan: CAN句柄
 * @param[in]      Object_Para: 对象参数
 * @param[in]      ID: CAN ID
 * @param[in]      Mask_ID: 掩码ID
 * @retval         none
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief          定时器回调函数
 * @param[in]      none
 * @retval         none
 */
void TIM_CAN_PeriodElapsedCallback(void);

#endif
