/**
 * @file motor_dji.c
 * @author CHR
 * @brief 通过CAN总线控制电机
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "motor_dji.h"
//**
// * @brief 分配CAN上传数据指针
// *
// * @param hcan CAN控制器参数
// * @param __CAN_ID CAN ID
// * @return uint8_t* 数据指针
// */
uint8_t *allocate_tx_data_dji(CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case CAN_Motor_ID_0x201:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x202:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x203:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
            break;
        case CAN_Motor_ID_0x204:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x205:
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x206:
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x207:
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            break;
        case CAN_Motor_ID_0x208:
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x209:
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x20A:
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x20B:
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
            break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (__CAN_ID)
        {
        case CAN_Motor_ID_0x201:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x202:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x203:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
            break;
        case CAN_Motor_ID_0x204:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x205:
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x206:
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x207:
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
            break;
        case CAN_Motor_ID_0x208:
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x209:
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x20A:
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x20B:
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
            break;
        }
    }
    return tmp_tx_data_ptr;
}

//**
// * @brief 初始化GM6020电机参数
// *
// * @param motor 电机对象指针
// * @param hcan CAN控制器句柄
// * @param __CAN_ID CAN ID
// * @param __Control_Method 控制方式
// * @param __Encoder_Offset 中值
// * @param __Omega_Max 最大角速度（5）
// */
void Motor_GM6020_Init(Motor_GM6020 *motor, CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, float __Omega_Max)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }

    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->Encoder_Offset = __Encoder_Offset;
    motor->Omega_Max = __Omega_Max;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID);
    motor->Encoder_Num_Per_Round = 8192;
    motor->Output_Max = 30000.0f;
}

//**
// * @brief 电机CAN接收完成回调函数
// *
// * @param motor 电机对象指针
// * @param Rx_Data 接收的数据指针
// */
void Motor_GM6020_CAN_RxCpltCallback(Motor_GM6020 *motor, uint8_t *Rx_Data)
{
    int16_t delta_encoder;

    motor->Flag += 1;

    motor->Pre_Encoder = motor->Rx_Encoder;

    motor->Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
    motor->Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
    motor->Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
    motor->Rx_Temperature = Rx_Data[6];

    delta_encoder = motor->Rx_Encoder - motor->Pre_Encoder;
    if (delta_encoder < -4096)
    {
        motor->Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        motor->Total_Round--;
    }
    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round + motor->Rx_Encoder + motor->Encoder_Offset;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * PI;
    motor->Now_Omega = (float)motor->Rx_Omega * RPM_TO_RADPS;
    motor->Now_Torque = motor->Rx_Torque;
    motor->Now_Temperature = motor->Rx_Temperature;
}

//**
// * @brief 定时器触发电机存活检测回调函数
// *
// * @param motor 电机对象指针
// */
void Motor_GM6020_TIM_Alive_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Torque, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

//**
// * @brief 电机PID控制回调函数
// *
// * @param motor 电机对象指针
// */
void Motor_GM6020_TIM_PID_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    switch (motor->Control_Method)
    {
        // 开环
    case Control_Method_OPENLOOP:
        Motor_GM6020_Set_Out(motor, motor->Target_Omega / motor->Omega_Max * motor->Output_Max);
        break;
    // 力矩环
    case Control_Method_TORQUE:
        motor->PID_Torque.Target = motor->Target_Torque;
        motor->PID_Torque.Now = motor->Now_Torque;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);

        Motor_GM6020_Set_Out(motor, motor->PID_Torque.Out);
        break;
    // 角速度环
    case Control_Method_OMEGA:

        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Torque = motor->PID_Omega.Out;

        motor->PID_Torque.Target = motor->Target_Torque;
        motor->PID_Torque.Now = motor->Now_Torque;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);

        Motor_GM6020_Set_Out(motor, motor->PID_Torque.Out);
        break;
    // 角度环
    case Control_Method_ANGLE:
        motor->PID_Angle.Target = motor->Target_Angle;
        motor->PID_Angle.Now = motor->Now_Angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);

        motor->Target_Omega = motor->PID_Angle.Out;

        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Torque = motor->PID_Omega.Out;

        motor->PID_Torque.Target = motor->Target_Torque;
        motor->PID_Torque.Now = motor->Now_Torque;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Torque);

        Motor_GM6020_Set_Out(motor, motor->PID_Torque.Out);
        break;
    default:
        Motor_GM6020_Set_Out(motor, 0.0f);
        break;
    }
    Motor_GM6020_Output(motor);
}

//**
// * @brief 输出电机控制命令
// *
// * @param motor 电机对象指针
// */
void Motor_GM6020_Output(Motor_GM6020 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

//**
// * @brief 获取电机最大输出
// *
// * @param motor 电机对象指针
// * @return uint16_t 电机最大输出值
// */
uint16_t Motor_GM6020_Get_Output_Max(Motor_GM6020 *motor)
{
    return motor->Output_Max;
}

//**
// * @brief 获取电机CAN状态
// *
// * @param motor 电机对象指针
// * @return Enum_CAN_Motor_Status 电机CAN状态
// */
Enum_CAN_Motor_Status Motor_GM6020_Get_CAN_Motor_Status(Motor_GM6020 *motor)
{
    return motor->CAN_Motor_Status;
}

//**
// * @brief 获取当前角度
// *
// * @param motor 电机对象指针
// * @return float 当前角度值
// */
float Motor_GM6020_Get_Now_Angle(Motor_GM6020 *motor)
{
    return motor->Now_Angle;
}

//**
// * @brief 获取当前角速度
// *
// * @param motor 电机对象指针
// * @return float 当前角速度值
// */
float Motor_GM6020_Get_Now_Omega(Motor_GM6020 *motor)
{
    return motor->Now_Omega;
}

//**
// * @brief 获取当前扭矩
// *
// * @param motor 电机对象指针
// * @return float 当前扭矩值
// */
float Motor_GM6020_Get_Now_Torque(Motor_GM6020 *motor)
{
    return motor->Now_Torque;
}

//**
// * @brief 获取当前温度
// *
// * @param motor 电机对象指针
// * @return uint8_t 当前温度值
// */
uint8_t Motor_GM6020_Get_Now_Temperature(Motor_GM6020 *motor)
{
    return motor->Now_Temperature;
}

//**
// * @brief 获取电机控制方法
// *
// * @param motor 电机对象指针
// * @return Enum_Control_Method 当前控制方法
// */
Enum_Control_Method Motor_GM6020_Get_Control_Method(Motor_GM6020 *motor)
{
    return motor->Control_Method;
}

//**
// * @brief 获取目标角度
// *
// * @param motor 电机对象指针
// * @return float 目标角度值
// */
float Motor_GM6020_Get_Target_Angle(Motor_GM6020 *motor)
{
    return motor->Target_Angle;
}

//**
// * @brief 获取目标角速度
// *
// * @param motor 电机对象指针
// * @return float 目标角速度值
// */
float Motor_GM6020_Get_Target_Omega(Motor_GM6020 *motor)
{
    return motor->Target_Omega;
}

//**
// * @brief 获取目标扭矩
// *
// * @param motor 电机对象指针
// * @return float 目标扭矩值
// */
float Motor_GM6020_Get_Target_Torque(Motor_GM6020 *motor)
{
    return motor->Target_Torque;
}

//**
// * @brief 获取电机输出值
// *
// * @param motor 电机对象指针
// * @return float 电机输出值
// */
float Motor_GM6020_Get_Out(Motor_GM6020 *motor)
{
    return motor->Out;
}

//**
// * @brief 设置控制方法
// *
// * @param motor 电机对象指针
// * @param __Control_Method 控制方法
// */
void Motor_GM6020_Set_Control_Method(Motor_GM6020 *motor, Enum_Control_Method __Control_Method)
{
    motor->Control_Method = __Control_Method;
}

//**
// * @brief 设置目标角度
// *
// * @param motor 电机对象指针
// * @param __Target_Angle 目标角度
// */
void Motor_GM6020_Set_Target_Angle(Motor_GM6020 *motor, float __Target_Angle)
{
    motor->Target_Angle = __Target_Angle;
}

//**
// * @brief 设置目标角速度
// *
// * @param motor 电机对象指针
// * @param __Target_Omega 目标角速度
// */
void Motor_GM6020_Set_Target_Omega(Motor_GM6020 *motor, float __Target_Omega)
{
    motor->Target_Omega = __Target_Omega;
}

//**
// * @brief 设置目标扭矩
// *
// * @param motor 电机对象指针
// * @param __Target_Torque 目标扭矩
// */
void Motor_GM6020_Set_Target_Torque(Motor_GM6020 *motor, float __Target_Torque)
{
    motor->Target_Torque = __Target_Torque;
}

//**
// * @brief 设置电机输出
// *
// * @param motor 电机对象指针
// * @param __Out 输出值
// */
void Motor_GM6020_Set_Out(Motor_GM6020 *motor, float __Out)
{
    motor->Out = __Out;
}


/* C610 and C620 motor functions using CAN communication */

/**
 * @brief 初始化C610电机参数
 *
 * @param motor 电机对象指针
 * @param hcan CAN控制器句柄
 * @param __CAN_ID CAN ID
 * @param __Control_Method 控制方式
 */
void Motor_C610_Init(Motor_C610 *motor, CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID);
    motor->Gearbox_Rate = 36.0f;
    motor->Torque_Max = 10000.0f;
    motor->Encoder_Num_Per_Round = 8192;
    motor->Output_Max = 10000;
}

/**
 * @brief CAN接收完成回调函数，用于处理接收到的数据
 *
 * @param motor 电机对象指针
 * @param Rx_Data 接收的数据
 */
void Motor_C610_CAN_RxCpltCallback(Motor_C610 *motor, uint8_t *Rx_Data)
{
    int16_t delta_encoder;

    motor->Flag += 1;

    motor->Pre_Encoder = motor->Rx_Encoder;

    motor->Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
    motor->Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
    motor->Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
    motor->Rx_Temperature = Rx_Data[6];

    delta_encoder = motor->Rx_Encoder - motor->Pre_Encoder;
    if (delta_encoder < -4096)
    {
        motor->Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        motor->Total_Round--;
    }
    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round + motor->Rx_Encoder;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * PI / motor->Gearbox_Rate;
    motor->Now_Omega = (float)motor->Rx_Omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Now_Torque = motor->Rx_Torque;
    motor->Now_Temperature = motor->Rx_Temperature;
}

/**
 * @brief 定时器中断回调函数，用于检测电机是否失效
 *
 * @param motor 电机对象指针
 */
void Motor_C610_TIM_Alive_PeriodElapsedCallback(Motor_C610 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

/**
 * @brief 定时器中断回调函数，用于执行PID调节输出
 *
 * @param motor 电机对象指针
 */
void Motor_C610_TIM_PID_PeriodElapsedCallback(Motor_C610 *motor)
{
    switch (motor->Control_Method)
    {
    case Control_Method_OPENLOOP:
        Motor_C610_Set_Out(motor, motor->Target_Torque / motor->Torque_Max * motor->Output_Max);
        break;
    case Control_Method_TORQUE:
        Motor_C610_Set_Out(motor, motor->Target_Torque / motor->Torque_Max * motor->Output_Max);
        break;
    case Control_Method_OMEGA:
        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        Motor_C610_Set_Out(motor, motor->PID_Omega.Out);
        break;
    case Control_Method_ANGLE:
        motor->PID_Angle.Target = motor->Target_Angle;
        motor->PID_Angle.Now = motor->Now_Angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);

        motor->Target_Omega = motor->PID_Angle.Out;

        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        Motor_C610_Set_Out(motor, motor->PID_Omega.Out);
        break;
    default:
        Motor_C610_Set_Out(motor, 0.0f);
        break;
    }
    Motor_C610_Output(motor);
}

/**
 * @brief 设置电机输出指令
 *
 * @param motor 电机对象指针
 */
void Motor_C610_Output(Motor_C610 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

/**
 * @brief 获取电机的最大输出值
 *
 * @param motor 电机对象指针
 * @return uint16_t 电机最大输出值
 */
uint16_t Motor_C610_Get_Output_Max(Motor_C610 *motor)
{
    return motor->Output_Max;
}

/**
 * @brief 获取电机的CAN状态
 *
 * @param motor 电机对象指针
 * @return Enum_CAN_Motor_Status 电机CAN状态
 */
Enum_CAN_Motor_Status Motor_C610_Get_CAN_Motor_Status(Motor_C610 *motor)
{
    return motor->CAN_Motor_Status;
}

/**
 * @brief 获取电机的当前角度
 *
 * @param motor 电机对象指针
 * @return float 当前角度值
 */
float Motor_C610_Get_Now_Angle(Motor_C610 *motor)
{
    return motor->Now_Angle;
}

/**
 * @brief 获取电机的当前转速
 *
 * @param motor 电机对象指针
 * @return float 当前转速值
 */
float Motor_C610_Get_Now_Omega(Motor_C610 *motor)
{
    return motor->Now_Omega;
}

/**
 * @brief 获取电机的当前力矩
 *
 * @param motor 电机对象指针
 * @return float 当前力矩值
 */
float Motor_C610_Get_Now_Torque(Motor_C610 *motor)
{
    return motor->Now_Torque;
}

/**
 * @brief 获取电机的当前温度
 *
 * @param motor 电机对象指针
 * @return uint8_t 当前温度值
 */
uint8_t Motor_C610_Get_Now_Temperature(Motor_C610 *motor)
{
    return motor->Now_Temperature;
}

/**
 * @brief 获取电机的控制方法
 *
 * @param motor 电机对象指针
 * @return Enum_Control_Method 电机控制方法
 */
Enum_Control_Method Motor_C610_Get_Control_Method(Motor_C610 *motor)
{
    return motor->Control_Method;
}

/**
 * @brief 获取电机的目标角度
 *
 * @param motor 电机对象指针
 * @return float 目标角度值
 */
float Motor_C610_Get_Target_Angle(Motor_C610 *motor)
{
    return motor->Target_Angle;
}

/**
 * @brief 获取电机的目标转速
 *
 * @param motor 电机对象指针
 * @return float 目标转速值
 */
float Motor_C610_Get_Target_Omega(Motor_C610 *motor)
{
    return motor->Target_Omega;
}

/**
 * @brief 获取电机的目标力矩
 *
 * @param motor 电机对象指针
 * @return float 目标力矩值
 */
float Motor_C610_Get_Target_Torque(Motor_C610 *motor)
{
    return motor->Target_Torque;
}

/**
 * @brief 获取电机的输出
 *
 * @param motor 电机对象指针
 * @return float 电机输出
 */
float Motor_C610_Get_Out(Motor_C610 *motor)
{
    return motor->Out;
}

/**
 * @brief 设置电机控制方法
 *
 * @param motor 电机对象指针
 * @param __Control_Method 控制方法
 */
void Motor_C610_Set_Control_Method(Motor_C610 *motor, Enum_Control_Method __Control_Method)
{
    motor->Control_Method = __Control_Method;
}

/**
 * @brief 设置电机目标角度
 *
 * @param motor 电机对象指针
 * @param __Target_Angle 目标角度值
 */
void Motor_C610_Set_Target_Angle(Motor_C610 *motor, float __Target_Angle)
{
    motor->Target_Angle = __Target_Angle;
}

/**
 * @brief 设置电机目标转速
 *
 * @param motor 电机对象指针
 * @param __Target_Omega 目标转速值
 */
void Motor_C610_Set_Target_Omega(Motor_C610 *motor, float __Target_Omega)
{
    motor->Target_Omega = __Target_Omega;
}

/**
 * @brief 设置电机目标力矩
 *
 * @param motor 电机对象指针
 * @param __Target_Torque 目标力矩值
 */
void Motor_C610_Set_Target_Torque(Motor_C610 *motor, float __Target_Torque)
{
    motor->Target_Torque = __Target_Torque;
}

/**
 * @brief 设置电机输出
 *
 * @param motor 电机对象指针
 * @param __Out 输出值
 */
void Motor_C610_Set_Out(Motor_C610 *motor, float __Out)
{
    motor->Out = __Out;
}


/* C620 motor functions using CAN communication */

/**
 * @brief 初始化C620电机参数
 *
 * @param motor 电机对象指针
 * @param hcan CAN控制器句柄
 * @param __CAN_ID CAN ID
 * @param __Control_Method 控制方式
 */
void Motor_C620_Init(Motor_C620 *motor, CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID);
    motor->Gearbox_Rate = 3591.0f / 187.0f;
    motor->Torque_Max = 16384.0f;
    motor->Encoder_Num_Per_Round = 8192;
    motor->Output_Max = 16384;
}

/**
 * @brief CAN接收完成回调函数，用于处理接收到的数据
 *
 * @param motor 电机对象指针
 * @param Rx_Data 接收的数据
 */
void Motor_C620_CAN_RxCpltCallback(Motor_C620 *motor, uint8_t *Rx_Data)
{
    int16_t delta_encoder;

    motor->Flag += 1;

    motor->Pre_Encoder = motor->Rx_Encoder;

    motor->Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
    motor->Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
    motor->Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
    motor->Rx_Temperature = Rx_Data[6];

    delta_encoder = motor->Rx_Encoder - motor->Pre_Encoder;
    if (delta_encoder < -4096)
    {
        motor->Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        motor->Total_Round--;
    }
    motor->Total_Encoder = motor->Total_Round * motor->Encoder_Num_Per_Round + motor->Rx_Encoder;

    motor->Now_Angle = (float)motor->Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * PI / motor->Gearbox_Rate;
    motor->Now_Omega = (float)motor->Rx_Omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Now_Velocity = (float)motor->Rx_Omega * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
    motor->Now_Torque = motor->Rx_Torque;
    motor->Now_Temperature = motor->Rx_Temperature;
}

/**
 * @brief 定时器中断回调函数，用于检测电机是否失效
 *
 * @param motor 电机对象指针
 */
void Motor_C620_TIM_Alive_PeriodElapsedCallback(Motor_C620 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

/**
 * @brief 定时器中断回调函数，用于执行PID调节输出
 *
 * @param motor 电机对象指针
 */
void Motor_C620_TIM_PID_PeriodElapsedCallback(Motor_C620 *motor)
{
    switch (motor->Control_Method)
    {
    // 开环
    case Control_Method_OPENLOOP:
        Motor_C620_Set_Out(motor, motor->Target_Torque / motor->Torque_Max * motor->Output_Max);
        break;
    // 力矩环
    case Control_Method_TORQUE:
        Motor_C620_Set_Out(motor, motor->Target_Torque / motor->Torque_Max * motor->Output_Max);
        break;
    // 角速度环
    case Control_Method_OMEGA:
        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        Motor_C620_Set_Out(motor, motor->PID_Omega.Out);
        break;
    // 速度环
    case Control_Method_Velocity:
        motor->PID_Velocity.Target = motor->Target_Velocity;
        motor->PID_Velocity.Now = motor->Now_Velocity;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Velocity);

        Motor_C620_Set_Out(motor, motor->PID_Velocity.Out);
        break;
    // 角度环
    case Control_Method_ANGLE:
        motor->PID_Angle.Target = motor->Target_Angle;
        motor->PID_Angle.Now = motor->Now_Angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);

        motor->Target_Omega = motor->PID_Angle.Out;

        motor->PID_Omega.Target = motor->Target_Omega;
        motor->PID_Omega.Now = motor->Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        Motor_C620_Set_Out(motor, motor->PID_Omega.Out);
        break;
    default:
        Motor_C620_Set_Out(motor, 0.0f);
        break;
    }
    Motor_C620_Output(motor);
}

/**
 * @brief 设置电机输出指令
 *
 * @param motor 电机对象指针
 */
void Motor_C620_Output(Motor_C620 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

/**
 * @brief 获取电机的最大输出值
 *
 * @param motor 电机对象指针
 * @return uint16_t 电机最大输出值
 */
uint16_t Motor_C620_Get_Output_Max(Motor_C620 *motor)
{
    return motor->Output_Max;
}

/**
 * @brief 获取电机的CAN状态
 *
 * @param motor 电机对象指针
 * @return Enum_CAN_Motor_Status 电机CAN状态
 */
Enum_CAN_Motor_Status Motor_C620_Get_CAN_Motor_Status(Motor_C620 *motor)
{
    return motor->CAN_Motor_Status;
}

/**
 * @brief 获取电机的当前角度
 *
 * @param motor 电机对象指针
 * @return float 当前角度值
 */
float Motor_C620_Get_Now_Angle(Motor_C620 *motor)
{
    return motor->Now_Angle;
}

/**
 * @brief 获取电机的当前转速
 *
 * @param motor 电机对象指针
 * @return float 当前转速值
 */
float Motor_C620_Get_Now_Omega(Motor_C620 *motor)
{
    return motor->Now_Omega;
}

/**
 * @brief 获取电机的当前力矩
 *
 * @param motor 电机对象指针
 * @return float 当前力矩值
 */
float Motor_C620_Get_Now_Torque(Motor_C620 *motor)
{
    return motor->Now_Torque;
}

/**
 * @brief 获取电机的当前温度
 *
 * @param motor 电机对象指针
 * @return uint8_t 当前温度值
 */
uint8_t Motor_C620_Get_Now_Temperature(Motor_C620 *motor)
{
    return motor->Now_Temperature;
}

/**
 * @brief 获取电机的控制方法
 *
 * @param motor 电机对象指针
 * @return Enum_Control_Method 电机控制方法
 */
Enum_Control_Method Motor_C620_Get_Control_Method(Motor_C620 *motor)
{
    return motor->Control_Method;
}

/**
 * @brief 获取电机的目标角度
 *
 * @param motor 电机对象指针
 * @return float 目标角度值
 */
float Motor_C620_Get_Target_Angle(Motor_C620 *motor)
{
    return motor->Target_Angle;
}

/**
 * @brief 获取电机的目标转速
 *
 * @param motor 电机对象指针
 * @return float 目标转速值
 */
float Motor_C620_Get_Target_Omega(Motor_C620 *motor)
{
    return motor->Target_Omega;
}

/**
 * @brief 获取电机的目标力矩
 *
 * @param motor 电机对象指针
 * @return float 目标力矩值
 */
float Motor_C620_Get_Target_Torque(Motor_C620 *motor)
{
    return motor->Target_Torque;
}

/**
 * @brief 获取电机的输出
 *
 * @param motor 电机对象指针
 * @return float 电机输出
 */
float Motor_C620_Get_Out(Motor_C620 *motor)
{
    return motor->Out;
}

/**
 * @brief 设置电机控制方法
 *
 * @param motor 电机对象指针
 * @param __Control_Method 控制方法
 */
void Motor_C620_Set_Control_Method(Motor_C620 *motor, Enum_Control_Method __Control_Method)
{
    motor->Control_Method = __Control_Method;
}

/**
 * @brief 设置电机目标角度
 *
 * @param motor 电机对象指针
 * @param __Target_Angle 目标角度值
 */
void Motor_C620_Set_Target_Angle(Motor_C620 *motor, float __Target_Angle)
{
    motor->Target_Angle = __Target_Angle;
}

/**
 * @brief 设置电机目标转速
 *
 * @param motor 电机对象指针
 * @param __Target_Omega 目标转速值
 */
void Motor_C620_Set_Target_Omega(Motor_C620 *motor, float __Target_Omega)
{
    motor->Target_Omega = __Target_Omega;
}

/**
 * @brief 设置电机目标力矩
 *
 * @param motor 电机对象指针
 * @param __Target_Torque 目标力矩值
 */
void Motor_C620_Set_Target_Torque(Motor_C620 *motor, float __Target_Torque)
{
    motor->Target_Torque = __Target_Torque;
}

/**
 * @brief 设置电机输出
 *
 * @param motor 电机对象指针
 * @param __Out 输出值
 */
void Motor_C620_Set_Out(Motor_C620 *motor, float __Out)
{
    motor->Out = __Out;
}
