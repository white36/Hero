/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"
#include "can_comm_task.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot_task.h"
// #include "pid.h"
#include "bsp_usart.h"
#include "referee.h"
#include <stdio.h>
#include "vision.h"
#include "vision_task.h"
#include "math.h"
//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
int16_t angle_sin,angle_cos;
float	angle_radto;
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
  * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
  * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
/**
  * @brief          云台校准计算
  * @param[in]      gimbal_cali: 校准数据
  * @param[out]     yaw_offset:yaw电机云台中值
  * @param[out]     pitch_offset:pitch 电机云台中值
  * @param[out]     max_yaw:yaw 电机最大机械角度
  * @param[out]     min_yaw: yaw 电机最小机械角度
  * @param[out]     max_pitch: pitch 电机最大机械角度
  * @param[out]     min_pitch: pitch 电机最小机械角度
  * @retval         none
  */
	
/**
 * @brief 云台二阶线性控制器初始化
 * 
 * @param controller 云台二阶线性控制器结构体
 * @param k_feed_forward 前馈系数
 * @param k_angle_error 角度误差系数
 * @param k_angle_speed 角速度系数
 * @param max_out 最大输出值
 * @param min_out 最小输出值
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t* controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out);

/**
 * @brief 云台二阶线性控制器计算 
 * 
 * @param controller 云台二阶线性控制器结构体
 * @param set_angle 角度设置值
 * @param cur_angle 当前角度
 * @param cur_angle_speed 当前角速度 
 * @param cur_current 当前电流
 * @return 返回系统输入 即电机电流值 
 */
// static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t* controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);

// static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
//云台控制所有相关数据
gimbal_control_t gimbal_control;
extern int16_t trigger_can_set_current;
extern shoot_control_t shoot_control;
extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_pitch;
//视觉任务结构体
extern vision_control_t vision_control;
////视觉发送任务结构体
//extern vision_send_t vision_send;
////视觉数据
vision_rxfifo_t *vision_rx;
//UI
// extern can_feedback_a_typedef get_capA;
extern int16_t R;
extern int16_t turn_flags;
extern int16_t anglesr;
extern int16_t angle_sin,angle_cos;
extern float angle_radto;
int16_t pitch_dian=2000;

/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back_GimbalCAN2(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
    case (0x205)://yaw
    {
        Motor_GM6020_CAN_RxCpltCallback(&gimbal_control.gimbal_yaw_motor.Motor_GM6020, Rx_Buffer->Data);
        break;
    }
    case (0x209)://PITCH
    {
        Motor_C620_CAN_RxCpltCallback(&gimbal_control.gimbal_pitch_motor.Motor_C620, Rx_Buffer->Data);
        break;
    }
    }
}

    /**
     * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
     * @param[in]      pvParameters: null
     * @retval         none
     */
    /**
     * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
     * @param[in]      pvParameters: 空
     * @retval         none
     */
    extern gimbal_behaviour_e gimbal_behaviour;
void gimbal_task(void const *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    gimbal_init(&gimbal_control);
    //判断电机是否都上线
//    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
//    {   
//			  CAN_cmd_gimbal(0, 0, 0, 0);
//        vTaskDelay(GIMBAL_CONTROL_TIME);
//        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
//			 
//    }

    while (1)
    {		//can_comm_UIO((int16_t)R,(int16_t)gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z,(int16_t)(get_capB.output_voltage/10),(int16_t)gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd);
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
        gimbal_set_control(&gimbal_control);                 //设置云台控制量
        gimbal_control_loop(&gimbal_control);                //云台控制PID计算
		


    //    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
   //     {
          if (toe_is_error(DBUS_TOE)||gimbal_behaviour == GIMBAL_ZERO_FORCE)
           {  
            //   CAN_cmd_gimbal(0, 0, 0, 0);
			// 			 CAN_cmd_pitch(0,0,0,0);
          }
           else
          {
//					CAN_cmd_pitch(gimbal_control.gimbal_pitch_motor.given_current,0,0,0);
//						CAN_cmd_pitch(2000,0,0,0);
					// CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.given_current,0,0, 0);
                    
                    

                    //CAN_cmd_gimbal(0,0,trigger_can_set_current, 0);
            }
    //    }

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
	}

/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
    CAN_Init(&GIMBAL_CAN, CAN_Motor_Call_Back_GimbalCAN2);
    init->gimbal_pitch_motor.absolute_angle_set=10;
	// const static fp32 yaw_vision_pid[3] = {YAW_VISION_PID_KP, YAW_VISION_PID_KI, YAW_VISION_PID_KD};
	// const static fp32 pitch_vision_pid[3] = {PITCH_VISION_PID_KP, PITCH_VISION_PID_KI, PITCH_VISION_PID_KD};
    //电机数据指针获取  (无需获取了)
    // init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    // init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    vision_rx = get_vision_fifo();
    // 陀螺仪数据指针获取（变为底盘姿态）
    init->gimbal_INS_point = get_INS_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl =get_remote_control_point();
	    // 获取上位机视觉数据指针
    init->gimbal_vision_point = get_vision_gimbal_point();
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化电机pi
   	stm32_pid_pitch_init();
	  stm32_pid_yaw_init();
    //初始化云台电机二阶控制器
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_yaw_motor.YAW_SOLC, YAW_FEED_FORWARD, K_YAW_ANGLE_ERROR, K_YAW_ANGLE_SPEED, YAW_MAX_OUT, YAW_MIX_OUT);
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_pitch_motor.PITCH_SOLC, PITCH_FEED_FORWARD, K_PITCH_ANGLE_ERROR, K_PITCH_ANGLE_SPEED, PITCH_MAX_OUT, PITCH_MIX_OUT);

    init->gimbal_pitch_motor.offset_ecd = 1700;
    init->gimbal_yaw_motor.offset_ecd = 3987;
    gimbal_feedback_update(init);
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    //yaw
    //pid初始化
    Motor_GM6020_Init(&init->gimbal_yaw_motor.Motor_GM6020, &hcan1, CAN_Motor_ID_0x205, Control_Method_ANGLE,0,5);
    /*角度环*/
    PID_Init(&init->gimbal_yaw_motor.Motor_GM6020.PID_Angle,
             YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD,
             YAW_GYRO_ABSOLUTE_PID_KF, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, PID_D_T,
             YAW_GYRO_ABSOLUTE_PID_DEAD_ZONE, YAW_GYRO_ABSOLUTE_I_Variable_Speed_A, YAW_GYRO_ABSOLUTE_I_Variable_Speed_B, YAW_GYRO_ABSOLUTE_I_Separate_Threshold, PID_D_First_ENABLE);
    /*角速度环*/
    PID_Init(&init->gimbal_yaw_motor.Motor_GM6020.PID_Omega,
             YAW_Omega_PID_KP, YAW_Omega_PID_KI, YAW_Omega_PID_KD,
             YAW_Omega_PID_KF, YAW_Omega_PID_MAX_IOUT, YAW_Omega_PID_MAX_OUT, PID_D_T,
             YAW_Omega_PID_DEAD_ZONE, YAW_Omega_I_Variable_Speed_A, YAW_Omega_I_Variable_Speed_B, YAW_Omega_I_Separate_Threshold, PID_D_First_ENABLE);
    /*力矩环*/
    PID_Init(&init->gimbal_yaw_motor.Motor_GM6020.PID_Torque,
             YAW_Torque_PID_KP, YAW_Torque_PID_KI, YAW_Torque_PID_KD,
             YAW_Torque_PID_KF, YAW_Torque_PID_MAX_IOUT, YAW_Torque_PID_MAX_OUT, PID_D_T,
             YAW_Torque_PID_DEAD_ZONE, YAW_Torque_I_Variable_Speed_A, YAW_Torque_I_Variable_Speed_B, YAW_Torque_I_Separate_Threshold, PID_D_First_ENABLE);

    //PITCH
    //pid初始化
    Motor_C620_Init(&init->gimbal_pitch_motor.Motor_C620, &hcan1, CAN_Motor_ID_0x209, Control_Method_ANGLE);
    /*角度环*/
    PID_Init(&init->gimbal_pitch_motor.Motor_C620.PID_Angle,
             PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD,
             PITCH_GYRO_ABSOLUTE_PID_KF, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PID_D_T,
             PITCH_GYRO_ABSOLUTE_PID_DEAD_ZONE, PITCH_GYRO_ABSOLUTE_I_Variable_Speed_A, PITCH_GYRO_ABSOLUTE_I_Variable_Speed_B, PITCH_GYRO_ABSOLUTE_I_Separate_Threshold, PID_D_First_ENABLE);
    /*角速度环*/
    PID_Init(&init->gimbal_pitch_motor.Motor_C620.PID_Omega,
             PITCH_Omega_PID_KP, PITCH_Omega_PID_KI, PITCH_Omega_PID_KD,
             PITCH_Omega_PID_KF, PITCH_Omega_PID_MAX_IOUT, PITCH_Omega_PID_MAX_OUT, PID_D_T,
             PITCH_Omega_PID_DEAD_ZONE, PITCH_Omega_I_Variable_Speed_A, PITCH_Omega_I_Variable_Speed_B, PITCH_Omega_I_Separate_Threshold, PID_D_First_ENABLE);

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    // init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

		init->gimbal_yaw_motor.max_relative_angle =2.10f;
    init->gimbal_yaw_motor.min_relative_angle = -2.60f;  
/*************************************************************** 做新的角度限制 *********************************************************************/
	// 	init->gimbal_pitch_motor.min_relative_angle =-0.55f;
    // init->gimbal_pitch_motor.max_relative_angle =0.30f;
		angle_radto = 0;
		angle_sin =0;
		angle_cos =0;
		
		vision_rx=get_vision_fifo();
}

/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    /****************************************************************待更改****************************************************************************************************/
		angle_radto=((float)(fabs(0.48f-gimbal_control.gimbal_pitch_motor.relative_angle)*180/3.1415926f));
		angle_sin=(int16_t)(100*sin(0.48f-gimbal_control.gimbal_pitch_motor.relative_angle));
		angle_cos=(int16_t)(fabs(100*cos(0.48f-gimbal_control.gimbal_pitch_motor.relative_angle)));
    //设置云台行为状态机、电机状态机
    gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          云台测量数据更新，包括绝对角度，相对角度，（角速度，电机速度）
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    // 云台数据更新
    // Pitch绝对角度
    feedback_update->gimbal_pitch_motor.absolute_angle = feedback_update->gimbal_INS_point->Pitch;

    //Pitch相对角度（丝杆不存在）
    // feedback_update->gimbal_pitch_motor.relative_angle =motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
	// 	                                                                               feedback_update->gimbal_pitch_motor.offset_ecd);

    //Pitch 角速度-作为二阶线性控制器的变量（用的时候再改）、 转速-用来写pid速度环，已经封装了
    // feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_INS_point->Gyro[1];
		// feedback_update->gimbal_pitch_motor.motor_speed =feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;

    //Yaw绝对角度
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_INS_point->Yaw;

    //Yaw相对角度
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.Motor_GM6020.Rx_Encoder,
                                                                                 feedback_update->gimbal_yaw_motor.offset_ecd);

    // Yaw角速度-作为二阶线性控制器的变量（用的时候再改）、转速-用来写pid速度环，已经封装了
    // feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[Z]) 
    //                                               -arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[X]);
    // feedback_update->gimbal_yaw_motor.motor_speed = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    } 
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          云台模式改变时，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    /*if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW 
        && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else*/ if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO
             && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    // else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE
    //          && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    // {
    //     gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    // }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    /*if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW 
        && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else*/ if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO
             && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    // else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE
    //          && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    // {
    //     gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    // }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
		//遥控器数值赋予
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    //yaw电机模式控制
    /*if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else*/ if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    // else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    // {
    //     //enconde模式下，电机编码角度控制
    //     gimbal_yaw_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    // }
		else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRONOLIMIT)
    {   //陀螺仪控制的小陀螺
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch电机模式控制
    /*if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else*/ if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    // else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    // {
    //     //enconde模式下，电机编码角度控制
    //     gimbal_pitch_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    // }
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
		angle_set = gimbal_motor->absolute_angle_set;	
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // now angle error
    // 当前控制误差角度
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //calculate max add_angle
            //计算出一个最大的添加角度
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add <	gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
		gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
					gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set <	gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = 	gimbal_motor->min_relative_angle;
    }
}
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
}


/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
		
   if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    if (&(control_loop->gimbal_yaw_motor) == NULL)
    {
      return;
    }
    control_loop->gimbal_yaw_motor.current_set = control_loop->gimbal_yaw_motor.raw_cmd_current;
    // control_loop->gimbal_yaw_motor.given_current = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
    control_loop->gimbal_yaw_motor.Motor_GM6020.Out = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
  }
	
  else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    if (&(control_loop->gimbal_yaw_motor) == NULL)
    {
      return;
    }

    // 计算云台yaw电机控制电流
    control_loop->gimbal_yaw_motor.Motor_GM6020.Target_Angle = control_loop->gimbal_yaw_motor.absolute_angle_set;
    // yaw轴 pid输出电流
    Motor_GM6020_TIM_PID_PeriodElapsedCallback(&gimbal_control.gimbal_yaw_motor.Motor_GM6020);
    //    control_loop->gimbal_yaw_motor.current_set = gimbal_motor_second_order_linear_controller_calc(&(control_loop->gimbal_yaw_motor.YAW_SOLC), control_loop->gimbal_yaw_motor.absolute_angle_set, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.motor_gyro, control_loop->gimbal_yaw_motor.gimbal_motor_measure->given_current);
    //    // 赋值电流值
    //    control_loop->gimbal_yaw_motor.given_current = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
    // stm32_step_yaw(control_loop->gimbal_yaw_motor.absolute_angle_set,control_loop->gimbal_yaw_motor.absolute_angle,control_loop->gimbal_yaw_motor.motor_speed);
    // control_loop->gimbal_yaw_motor.given_current=stm32_Y_yaw.Out1;
  }
	
//   else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//   {
//     if (&(control_loop->gimbal_yaw_motor) == NULL)
//     {
//       return;
//     }
// //    control_loop->gimbal_yaw_motor.current_set = gimbal_motor_second_order_linear_controller_calc(&(control_loop->gimbal_yaw_motor.YAW_SOLC), control_loop->gimbal_yaw_motor.absolute_angle_set, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.motor_gyro, control_loop->gimbal_yaw_motor.gimbal_motor_measure->given_current);
// //    // 赋值电流值
// //    control_loop->gimbal_yaw_motor.given_current = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
// 			stm32_step_yaw(control_loop->gimbal_yaw_motor.relative_angle_set,control_loop->gimbal_yaw_motor.relative_angle,control_loop->gimbal_yaw_motor.motor_speed);
// 			control_loop->gimbal_yaw_motor.given_current=stm32_Y_yaw.Out1;
//   }

  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
      if (&(control_loop->gimbal_pitch_motor) == NULL)
      {
          return;
      }
    //   stm32_step_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, 0);
    //   control_loop->gimbal_pitch_motor.given_current = stm32_Y_pitch.Out1;
      control_loop->gimbal_yaw_motor.Motor_GM6020.Out = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
      Motor_GM6020_Set_Out(&control_loop->gimbal_yaw_motor.Motor_GM6020, control_loop->gimbal_yaw_motor.Motor_GM6020.Out);
  }

  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
      if (&(control_loop->gimbal_pitch_motor) == NULL)
      {
          return;
      }

      // 计算云台yaw电机控制电流
      control_loop->gimbal_pitch_motor.Motor_C620.Target_Angle = control_loop->gimbal_pitch_motor.absolute_angle_set;
      // PITCH轴 pid输出电流
      Motor_C620_TIM_PID_PeriodElapsedCallback(&gimbal_control.gimbal_pitch_motor.Motor_C620);
      //    control_loop->gimbal_pitch_motor.current_set = gimbal_motor_second_order_linear_controller_calc(&(control_loop->gimbal_pitch_motor.PITCH_SOLC), control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_gyro, control_loop->gimbal_pitch_motor.gimbal_motor_measure->given_current);
      //    // 赋值电流值
      //    control_loop->gimbal_pitch_motor.given_current = (int16_t)(control_loop->gimbal_pitch_motor.current_set);
    //   stm32_step_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_speed);
    //   control_loop->gimbal_pitch_motor.given_current = stm32_Y_pitch.Out1;
  }

//     else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//     {
// 				if (&(control_loop->gimbal_pitch_motor) == NULL)
// 				{
// 						return;
// 				}
// //    control_loop->gimbal_pitch_motor.current_set = gimbal_motor_second_order_linear_controller_calc(&(control_loop->gimbal_pitch_motor.PITCH_SOLC), control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_gyro, control_loop->gimbal_pitch_motor.gimbal_motor_measure->given_current);
// //    // 赋值电流值
// //    control_loop->gimbal_pitch_motor.given_current = (int16_t)(control_loop->gimbal_pitch_motor.current_set);
// 			stm32_step_pitch(control_loop->gimbal_pitch_motor.relative_angle_set,control_loop->gimbal_pitch_motor.relative_angle,control_loop->gimbal_pitch_motor.motor_speed);
// 			control_loop->gimbal_pitch_motor.given_current=stm32_Y_pitch.Out1;
//     }
}

/*static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
		
		
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
				if (&(control_loop->gimbal_yaw_motor) == NULL)
				{
						return;
				}
				control_loop->gimbal_yaw_motor.current_set = control_loop->gimbal_yaw_motor.raw_cmd_current;
				control_loop->gimbal_yaw_motor.given_current = (int16_t)(control_loop->gimbal_yaw_motor.current_set);
    }
		
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
				if (&(control_loop->gimbal_yaw_motor) == NULL)
				{
						return;
				}
				stm32_step_yaw(control_loop->gimbal_yaw_motor.absolute_angle_set,control_loop->gimbal_yaw_motor.absolute_angle,0);
				control_loop->gimbal_yaw_motor.given_current=stm32_Y_yaw.Out1;

    }
		
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
				if (&(control_loop->gimbal_yaw_motor) == NULL)
				{
						return;
				}
				stm32_step_yaw(control_loop->gimbal_yaw_motor.relative_angle_set,control_loop->gimbal_yaw_motor.relative_angle,0);
				control_loop->gimbal_yaw_motor.given_current=stm32_Y_yaw.Out1;
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
				if (&(control_loop->gimbal_pitch_motor) == NULL)
				{
						return;
				}
				control_loop->gimbal_pitch_motor.current_set = control_loop->gimbal_pitch_motor.raw_cmd_current;
				control_loop->gimbal_pitch_motor.given_current = (int16_t)(control_loop->gimbal_pitch_motor.current_set);
    }
		
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
				if (&(control_loop->gimbal_pitch_motor) == NULL)
				{
						return;
				}
				stm32_step_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set,control_loop->gimbal_pitch_motor.absolute_angle,0);
				control_loop->gimbal_pitch_motor.given_current=stm32_Y_pitch.Out1;
    }
		
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
				if (&(control_loop->gimbal_pitch_motor) == NULL)
				{
						return;
				}
				stm32_step_pitch(control_loop->gimbal_pitch_motor.relative_angle_set,control_loop->gimbal_pitch_motor.relative_angle,0);
				control_loop->gimbal_pitch_motor.given_current=stm32_Y_pitch.Out1;

    }
}*/


static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out)
{
    // 前馈项系数
    controller->k_feed_forward = k_feed_forward;
    // 反馈矩阵系数
    controller->k_angle_error = k_angle_error;
    controller->k_angle_speed = k_angle_speed;
    // 设置最大输出值
    controller->max_out = max_out;
    // 设置最小输出值
    controller->min_out = min_out;
}

// /**
//  * @brief 云台二阶线性控制器计算
//  *
//  * @param controller 云台二阶线性控制器结构体
//  * @param set_angle 角度设置值
//  * @param cur_angle 当前角度
//  * @param cur_angle_speed 当前角速度
//  * @param cur_current 当前电流
//  * @return 返回系统输入
//  */
// static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current)
// {
//     // 赋值
//     controller->cur_angle = cur_angle;
//     controller->set_angle = set_angle;
//     controller->cur_angle_speed = cur_angle_speed;
//     // 将当前电流值乘以一个小于1的系数当作阻挡系统固有扰动的前馈项
//     controller->feed_forward = controller->k_feed_forward * cur_current;
//     // 计算误差 = 设定角度 - 当前角度
//     controller->angle_error = controller->set_angle - controller->cur_angle;
//     // 将误差值限制 -PI ~ PI 之间
//     controller->angle_error = rad_format(controller->angle_error);
//     // 计算输出值 = 前馈值 + 角度误差值 * 系数 + 角速度 * 系数
//     controller->output = controller->feed_forward + controller->angle_error * controller->k_angle_error + (-controller->cur_angle_speed * controller->k_angle_speed);

//     //限制输出值，防止出现电机崩溃的情况
//     if (controller->output >= controller->max_out)
//     {
//         controller->output = controller->max_out;
//     }
//     else if (controller->output <= controller->min_out)
//     {
//         controller->output = controller->min_out;
//     }

//     return controller->output;
// }


//获取场地正方向
fp32 get_yaw_positive_direction(void)
{
    return gimbal_control.yaw_positive_direction;
}

// /**
//   * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
//   * @param[in]      yaw_offse:yaw 中值
//   * @param[in]      pitch_offset:pitch 中值
//   * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
//   * @param[in]      min_yaw:yaw 最小相对角度
//   * @param[in]      max_yaw:pitch 最大相对角度
//   * @param[in]      min_yaw:pitch 最小相对角度
//   * @retval         返回空
//   * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
//   */
// void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
// {
//     gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
//     gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
//     gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

//     gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
//     gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
//     gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
// }

// /**
//   * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
//   * @param[out]     yaw 中值 指针
//   * @param[out]     pitch 中值 指针
//   * @param[out]     yaw 最大相对角度 指针
//   * @param[out]     yaw 最小相对角度 指针
//   * @param[out]     pitch 最大相对角度 指针
//   * @param[out]     pitch 最小相对角度 指针
//   * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
//   * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
//   */
// bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
// {
//     if (gimbal_control.gimbal_cali.step == 0)
//     {
//         gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
//         //保存进入时候的数据，作为起始数据，来判断最大，最小值
//         gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
//         gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
//         gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
//         gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
//         gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
//         gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
//         gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
//         gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
//         return 0;
//     }
//     else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
//     {
//         calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
//         (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
//         (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
//         (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
//         (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
//         gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
//         gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
//         gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
//         //gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
//         gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
//         gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
//         gimbal_control.gimbal_cali.step = 0;
//         return 1;
//     }
//     else
//     {
//         return 0;
//     }
// }

// /**
//   * @brief          calc motor offset encode, max and min relative angle
//   * @param[out]     yaw_offse:yaw middle place encode
//   * @param[out]     pitch_offset:pitch place encode
//   * @param[out]     max_yaw:yaw max relative angle
//   * @param[out]     min_yaw:yaw min relative angle
//   * @param[out]     max_yaw:pitch max relative angle
//   * @param[out]     min_yaw:pitch min relative angle
//   * @retval         none
//   */
// /**
//   * @brief          云台校准计算，将校准记录的中值,最大 最小值
//   * @param[out]     yaw 中值 指针
//   * @param[out]     pitch 中值 指针
//   * @param[out]     yaw 最大相对角度 指针
//   * @param[out]     yaw 最小相对角度 指针
//   * @param[out]     pitch 最大相对角度 指针
//   * @param[out]     pitch 最小相对角度 指针
//   * @retval         none
//   */
// static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
// {
// }
