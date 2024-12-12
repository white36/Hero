/**
 * @file motor_dji.h
 * @author CHR
 * @brief 大疆CAN电机配置与操作
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN_receive.h"
#include "Mathh.h"
#include "arm_math.h"
#include "can.h"
#include "PID_Control.h"

// RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)


/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态
 *
 */
typedef enum {
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
} Enum_CAN_Motor_Status;

/**
 * @brief CAN电机的ID枚举类型
 *
 */
typedef enum {
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
} Enum_CAN_Motor_ID;

/**
 * @brief CAN电机的ID分配情况
 *
 */
typedef enum {
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
} Enum_CAN_Motor_ID_Status;

/**
 * @brief 电机控制方式
 *
 */
typedef enum {
    Control_Method_OPENLOOP = 0,//开环
    Control_Method_TORQUE,      //力矩环
    Control_Method_OMEGA,       //角速度环
	Control_Method_Velocity,    //速度环
    Control_Method_ANGLE,       //角度环
} Enum_Control_Method;

/**
 * @brief GM6020无刷电机, 单片机控制输出电压
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;
    // PID扭矩环控制
    PID_control PID_Torque;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    uint32_t Encoder_Offset; // 编码器偏移
    float Omega_Max; // 最大速度

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出电压

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag
    uint16_t Rx_Encoder; // 接收的编码器位置
    int16_t Rx_Omega; // 接收的速度
    int16_t Rx_Torque; // 接收的扭矩
    uint16_t Rx_Temperature; // 接收的温度

    uint16_t Pre_Encoder; // 之前的编码器位置
    int32_t Total_Encoder; // 总编码器位置
    int32_t Total_Round; // 总圈数

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态
    float Now_Angle; // 当前的角度
    float Now_Omega; // 当前的速度
    float Now_Torque; // 当前的扭矩
    uint8_t Now_Temperature; // 当前的温度

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle; // 目标的角度
    float Target_Omega; // 目标的速度
    float Target_Torque; // 目标的扭矩
    float Out; // 输出量

} Motor_GM6020;

/**
 * @brief C610无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    float Gearbox_Rate; // 减速比
    float Torque_Max; // 最大扭矩

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出扭矩

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag
    uint16_t Rx_Encoder; // 接收的编码器位置
    int16_t Rx_Omega; // 接收的速度
    int16_t Rx_Torque; // 接收的扭矩
    uint16_t Rx_Temperature; // 接收的温度

    uint16_t Pre_Encoder; // 之前的编码器位置
    int32_t Total_Encoder; // 总编码器位置
    int32_t Total_Round; // 总圈数

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态
    float Now_Angle; // 当前的角度
    float Now_Omega; // 当前的速度
    float Now_Torque; // 当前的扭矩
    uint8_t Now_Temperature; // 当前的温度

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle; // 目标的角度
    float Target_Omega; // 目标的速度
    float Target_Torque; // 目标的扭矩
    float Out; // 输出量

} Motor_C610;

/**
 * @brief C620无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;
	// PID线速度环控制
	PID_control PID_Velocity;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    float Gearbox_Rate; // 减速比
    float Torque_Max; // 最大扭矩

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出扭矩

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag
    uint16_t Rx_Encoder; // 接收的编码器位置
    int16_t Rx_Omega; // 接收的速度 单位 rpm
    int16_t Rx_Torque; // 接收的扭矩
    uint16_t Rx_Temperature; // 接收的温度

    uint16_t Pre_Encoder; // 之前的编码器位置
    int32_t Total_Encoder; // 总编码器位置
    int32_t Total_Round; // 总圈数

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态
    float Now_Angle; 	// 当前的角度
    float Now_Omega; 	// 当前的角速度 //单位 rad/s
	float Now_Velocity; // 当前的线速度 //单位 m/s
    float Now_Torque; 	// 当前的扭矩
    uint8_t Now_Temperature; // 当前的温度

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle; 	// 目标的角度
    float Target_Omega; 	// 目标的角速度 //单位 rad/s
	float Target_Velocity;  // 目标的线速度 //单位 m/s
    float Target_Torque; 	// 目标的扭矩
    float Out; 				// 输出量

} Motor_C620;

/* Exported function declarations --------------------------------------------*/

void Motor_GM6020_Init(Motor_GM6020 *motor, CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, float __Omega_Max);
void Motor_GM6020_Output(Motor_GM6020 *motor);
uint16_t Motor_GM6020_Get_Output_Max(Motor_GM6020 *motor);
Enum_CAN_Motor_Status Motor_GM6020_Get_CAN_Motor_Status(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Angle(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Omega(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Torque(Motor_GM6020 *motor);
uint8_t Motor_GM6020_Get_Now_Temperature(Motor_GM6020 *motor);
Enum_Control_Method Motor_GM6020_Get_Control_Method(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Angle(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Omega(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Torque(Motor_GM6020 *motor);
float Motor_GM6020_Get_Out(Motor_GM6020 *motor);
void Motor_GM6020_Set_Control_Method(Motor_GM6020 *motor, Enum_Control_Method __Control_Method);
void Motor_GM6020_Set_Target_Angle(Motor_GM6020 *motor, float __Target_Angle);
void Motor_GM6020_Set_Target_Omega(Motor_GM6020 *motor, float __Target_Omega);
void Motor_GM6020_Set_Target_Torque(Motor_GM6020 *motor, float __Target_Torque);
void Motor_GM6020_Set_Out(Motor_GM6020 *motor, float __Out);
void Motor_GM6020_CAN_RxCpltCallback(Motor_GM6020 *motor, uint8_t *Rx_Data);
void Motor_GM6020_TIM_Alive_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_GM6020_TIM_PID_PeriodElapsedCallback(Motor_GM6020 *motor);

void Motor_C610_Init(Motor_C610 *motor, CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method);
void Motor_C610_Output(Motor_C610 *motor);
uint16_t Motor_C610_Get_Output_Max(Motor_C610 *motor);
Enum_CAN_Motor_Status Motor_C610_Get_CAN_Motor_Status(Motor_C610 *motor);
float Motor_C610_Get_Now_Angle(Motor_C610 *motor);
float Motor_C610_Get_Now_Omega(Motor_C610 *motor);
float Motor_C610_Get_Now_Torque(Motor_C610 *motor);
uint8_t Motor_C610_Get_Now_Temperature(Motor_C610 *motor);
Enum_Control_Method Motor_C610_Get_Control_Method(Motor_C610 *motor);
float Motor_C610_Get_Target_Angle(Motor_C610 *motor);
float Motor_C610_Get_Target_Omega(Motor_C610 *motor);
float Motor_C610_Get_Target_Torque(Motor_C610 *motor);
float Motor_C610_Get_Out(Motor_C610 *motor);
void Motor_C610_Set_Control_Method(Motor_C610 *motor, Enum_Control_Method __Control_Method);
void Motor_C610_Set_Target_Angle(Motor_C610 *motor, float __Target_Angle);
void Motor_C610_Set_Target_Omega(Motor_C610 *motor, float __Target_Omega);
void Motor_C610_Set_Target_Torque(Motor_C610 *motor, float __Target_Torque);
void Motor_C610_Set_Out(Motor_C610 *motor, float __Out);
void Motor_C610_CAN_RxCpltCallback(Motor_C610 *motor, uint8_t *Rx_Data);
void Motor_C610_TIM_Alive_PeriodElapsedCallback(Motor_C610 *motor);
void Motor_C610_TIM_PID_PeriodElapsedCallback(Motor_C610 *motor);

void Motor_C620_Init(Motor_C620 *motor, CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method);
void Motor_C620_Output(Motor_C620 *motor);
uint16_t Motor_C620_Get_Output_Max(Motor_C620 *motor);
Enum_CAN_Motor_Status Motor_C620_Get_CAN_Motor_Status(Motor_C620 *motor);
float Motor_C620_Get_Now_Angle(Motor_C620 *motor);
float Motor_C620_Get_Now_Omega(Motor_C620 *motor);
float Motor_C620_Get_Now_Torque(Motor_C620 *motor);
uint8_t Motor_C620_Get_Now_Temperature(Motor_C620 *motor);
Enum_Control_Method Motor_C620_Get_Control_Method(Motor_C620 *motor);
float Motor_C620_Get_Target_Angle(Motor_C620 *motor);
float Motor_C620_Get_Target_Omega(Motor_C620 *motor);
float Motor_C620_Get_Target_Torque(Motor_C620 *motor);
float Motor_C620_Get_Out(Motor_C620 *motor);
void Motor_C620_Set_Control_Method(Motor_C620 *motor, Enum_Control_Method __Control_Method);
void Motor_C620_Set_Target_Angle(Motor_C620 *motor, float __Target_Angle);
void Motor_C620_Set_Target_Omega(Motor_C620 *motor, float __Target_Omega);
void Motor_C620_Set_Target_Torque(Motor_C620 *motor, float __Target_Torque);
void Motor_C620_Set_Out(Motor_C620 *motor, float __Out);
void Motor_C620_CAN_RxCpltCallback(Motor_C620 *motor, uint8_t *Rx_Data);
void Motor_C620_TIM_Alive_PeriodElapsedCallback(Motor_C620 *motor);
void Motor_C620_TIM_PID_PeriodElapsedCallback(Motor_C620 *motor);

#endif
