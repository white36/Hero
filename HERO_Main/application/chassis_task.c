/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "can_comm_task.h"
#include "cmsis_os.h"

#include "arm_math.h"
// #include "pid.h"
#include "PID_control.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "bsp_usart.h"

#include "referee.h"
int anglesr;
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
//�����˶�����
chassis_move_t chassis_move;
int8_t QA=0,EA=0;
int16_t cnts=0;
extern gimbal_control_t gimbal_control;
extern vision_rxfifo_t *vision_rx;
extern int MODE;

/**
 * @brief CAN���Ļص�����
 *
 * @param Rx_Buffer CAN���յ���Ϣ�ṹ��
 */
void CAN_Motor_Call_Back_ChassisCAN1(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
    case (0x201):
    {
        Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[0].Motor_C620, Rx_Buffer->Data);
        break;
    }
    case (0x202):
    {
        Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[1].Motor_C620, Rx_Buffer->Data);
        break;
    }
    case (0x203):
    {
        Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[2].Motor_C620, Rx_Buffer->Data);
        break;
    }
    case (0x204):
    {
        Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[3].Motor_C620, Rx_Buffer->Data);
        break;
    }
    }
}

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
    //���̳�ʼ��
    chassis_init(&chassis_move);
    //�жϵ��̵���Ƿ�����
    /*while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }*/

    while (1)
    {
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //����ģʽ�л� �������ݹ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݷ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
//	   	  CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
       // if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
      // {
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            // if (toe_is_error(DBUS_TOE))
            // {
            //     // CAN_cmd_chassis(0, 0, 0, 0);//
            // }
            // else
            // {
                
                // //���Ϳ��Ƶ���
                // CAN_cmd_chassis(chassis_move.Chassis_3508[0].give_current, chassis_move.       Chassis_3508[1].give_current,
                //                 chassis_move.Chassis_3508[2].give_current, chassis_move.Chassis_3508[3].give_current);
//				CAN_cmd_chassis(0, 0,
//                                chassis_move.Chassis_3508[2].give_current, 0);
            // }
     //   }
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    CAN_Init(&CHASSIS_CAN, CAN_Motor_Call_Back_ChassisCAN1);
    // �����ٶȻ�pidֵ
    //    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    // ���̽Ƕ�pidֵ
    //    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    // ���ʿ���
    // const static fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD}; // ���ʻ�PID����

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_point = get_INS_point();
    //��ȡ��̨�������ָ��//*****************************************************���޸�*********************************************** */
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //��ȡ���̵������ָ�룬��ʼ��PID
    for (Enum_CAN_Motor_ID i = CAN_Motor_ID_0x201; i < CAN_Motor_ID_0x204; i++)
    {
        Motor_C620_Init(&chassis_move_init->Chassis_3508[i - CAN_Motor_ID_0x201].Motor_C620, &CHASSIS_CAN, i, Control_Method_Velocity);
        PID_Init(&chassis_move_init->Chassis_3508[i - CAN_Motor_ID_0x201].Motor_C620.PID_Velocity, 
                 M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD,
                 M3508_MOTOR_SPEED_PID_KF, M3508_MOTOR_SPEED_PID_MAX_IOUT, M3508_MOTOR_SPEED_PID_MAX_OUT, PID_D_T, M3508_MOTOR_SPEED_PID_DEAD_ZONE,
                 M3508_MOTOR_I_Variable_Speed_A, M3508_MOTOR_I_Variable_Speed_B, M3508_MOTOR_I_Separate_Threshold, PID_D_First_ENABLE);
    }
    //��ʼ���Ƕ�PID
    PID_Init(&chassis_move_init->chassis_angle_pid, CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD,
             CHASSIS_FOLLOW_GIMBAL_PID_KF, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, PID_D_T, CHASSIS_FOLLOW_GIMBAL_PID_DEAD_ZONE,
             CHASSIS_FOLLOW_GIMBAL_I_Variable_Speed_A, CHASSIS_FOLLOW_GIMBAL_I_Variable_Speed_B, CHASSIS_FOLLOW_GIMBAL_I_Separate_Threshold, PID_D_First_DISABLE);
    // // ���ʻ�PID
    // PID_init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);

    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
        //UI��ʾ���̽Ƕ�
		anglesr=abs((int)(chassis_move.chassis_yaw_motor->relative_angle*100));
		if(anglesr>157&&anglesr<314){
		anglesr=314-anglesr;
		}
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
           && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //���������̽Ƕ�ģʽ�����ޣ�
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
                && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW)
                && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    
    // ����Ϊ�ϴ����ģʽ���Ա��¸�ѭ����ȡ
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȡ����ٶ� ���ٶȵ�PID΢��
        chassis_move_update->Chassis_3508[i].speed = chassis_move_update->Chassis_3508[i].Motor_C620.Now_Velocity; // ��ɾ����ǰ��speed=rpm*...,��װ���˵��������
        // chassis_move_update->Chassis_3508[i].accel = chassis_move_update->Chassis_3508[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE; //
    }
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->Chassis_3508[0].Motor_C620.Now_Velocity + chassis_move_update->Chassis_3508[1].Motor_C620.Now_Velocity + chassis_move_update->Chassis_3508[2].Motor_C620.Now_Velocity - chassis_move_update->Chassis_3508[3].Motor_C620.Now_Velocity) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->Chassis_3508[0].Motor_C620.Now_Velocity - chassis_move_update->Chassis_3508[1].Motor_C620.Now_Velocity + chassis_move_update->Chassis_3508[2].Motor_C620.Now_Velocity + chassis_move_update->Chassis_3508[3].Motor_C620.Now_Velocity) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->Chassis_3508[0].Motor_C620.Now_Velocity - chassis_move_update->Chassis_3508[1].Motor_C620.Now_Velocity - chassis_move_update->Chassis_3508[2].Motor_C620.Now_Velocity - chassis_move_update->Chassis_3508[3].Motor_C620.Now_Velocity) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    //    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    //    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    //    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

//    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN+vision_rx->vx;
//    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_rx->vy;
		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
		
    //���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = -chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = -chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = -chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = -chassis_move_rc_to_vector->vy_min_speed;
    }

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		
		if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z)
		{
				*vx_set = 0.0f;
				*vy_set = 0.0f;
		}
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f,angle_set;
		
		//fp32 vx_set = vision_rx->vx, vy_set = vision_rx->vy,angle_set;
		
    //get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

		
    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�

        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //������תPID���ٶ�
//			if(fabs(chassis_move_control->chassis_relative_angle_set-chassis_move_control->chassis_yaw_motor->relative_angle)>=0.02f)
        chassis_move_control->chassis_angle_pid.Target = chassis_move_control->chassis_relative_angle_set;
        chassis_move_control->chassis_angle_pid.Now = chassis_move_control->chassis_yaw_motor->relative_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&chassis_move_control->chassis_angle_pid);
        chassis_move_control->wz_set = -chassis_move_control->chassis_angle_pid.Out;
        //			else
        //				chassis_move_control->wz_set=0;
        // �ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)//(����)
    {
        fp32 delat_angle = 0.0f;
        //���õ��̿��ƵĽǶ�
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //������ת�Ľ��ٶ�
        chassis_move_control->chassis_angle_pid.Target = delat_angle;
        chassis_move_control->chassis_angle_pid.Now = 0.0f;
        PID_TIM_Adjust_PeriodElapsedCallback(&chassis_move_control->chassis_angle_pid);
        chassis_move_control->wz_set = chassis_move_control->chassis_angle_pid.Out;
        // �ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_BPIN)
    {

        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;			
			  sin_yaw =(arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle));
        cos_yaw =(arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle));
			
			  chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
				
				chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
       // fp32 chassis_wz = 10.0f;
			
      if(MODE==1){
				if(chassis_move.chassis_RC->rc.ch[4]>600)
				{chassis_move_control->wz_set = angle_set;}
				if(chassis_move.chassis_RC->rc.ch[4]<-600)
				{chassis_move_control->wz_set = -angle_set;}
			}
	    else
			{
				chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);}
			}
		}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	  wheel_speed[0] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set -(CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//	
//	  wheel_speed[0] = vx_set - vy_set -MOTOR_DISTANCE_TO_CENTER * wz_set*0.95f;
//    wheel_speed[1] = -vx_set - vy_set -MOTOR_DISTANCE_TO_CENTER * wz_set*1.1f;
//    wheel_speed[2] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[3] = vx_set + vy_set  -MOTOR_DISTANCE_TO_CENTER * wz_set*1.1f;
}
//1.180f 1.0f

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
//		static int32_t input_power;
//		static int8_t release_energy = 0;
		
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->Chassis_3508[i].Motor_C620.Out = (int16_t)(wheel_speed[i]);
            Motor_C620_Set_Out(&chassis_move_control_loop->Chassis_3508[i].Motor_C620, chassis_move_control_loop->Chassis_3508[i].Motor_C620.Out);
        }
        //raw����ֱ�ӷ���
        return;
    }
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->Chassis_3508[i].Motor_C620.Target_Velocity = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->Chassis_3508[i].Motor_C620.Target_Velocity); // fab����Ϊ��ǰ����
        if (max_vector < temp)
        {
            max_vector = temp;//���xС�ڵ�ǰ�趨�ٶȣ�x=��ǰ�ٶ�
        }
    }
    if (max_vector > MAX_WHEEL_SPEED)//xС�ڵ�ǰ�ٶ�ʱ���ڵ�ǰ�ٶȣ�����ǰ�ٶȴ����������
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;//�����ʼ�Ϊ ����ٶ�/��ǰ�ٶ�
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->Chassis_3508[i].Motor_C620.Target_Velocity *= vector_rate; // �������
        }
    }

	//calculate pid
    //���pid����ĵ���
    for (i = 0; i < 4; i++)
    {
        Motor_C620_TIM_PID_PeriodElapsedCallback(&chassis_move_control_loop->Chassis_3508[i].Motor_C620);
    }
    //��ֵ����ֵ�����������
    // for (i = 0; i < 4; i++)
    // {
    //     chassis_move_control_loop->Chassis_3508[i].give_current = (int16_t)(chassis_move_control_loop->Chassis_3508[i].Out);
    // }

}

uint8_t can_send_tmp = 0;

// /**
//   * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
//   * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
//   * @retval         none
//   */
// void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor)
// {
// 	can_send_tmp++;
// 	//
// 	uint16_t max_power_limit = 10000;
// 	fp32 input_power = 0;		 // ���빦��(����������)
// 	fp32 scaled_motor_power[4];
// 	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  �˲������������ת��ΪŤ��
// 	fp32 k2 = 1.23e-07;						 // �Ŵ�ϵ��
// 	fp32 k1 = 1.453e-07;					 // �Ŵ�ϵ��
	
	
// 	fp32 constant = 4.081f;  //3508����Ļ�е���
// 	chassis_motor->power_control.POWER_MAX = 0; //���յ��̵������
// 	chassis_motor->power_control.forecast_total_power = 0; // Ԥ���ܹ���

// 	//PID_Calc(&chassis_motor->buffer_pid, chassis_motor->chassis_power_buffer, 30); //ʹ��������ά����һ���ȶ��ķ�Χ,�����PIDû��Ҫ��ֲ�ҵģ�������һ������
// 	  PID_calc(&chassis_motor->buffer_pid, chassis_motor->chassis_power_buffer,30);
	
// 	if(chassis_motor->chassis_power_MAX>120)
// 	{max_power_limit =120;}
// 	else
// 	{max_power_limit = chassis_motor->chassis_power_MAX; } //��ò���ϵͳ�Ĺ���������ֵ
	
// 	input_power = max_power_limit - chassis_motor->buffer_pid.out; //ͨ������ϵͳ�������
	
// 	chassis_motor->power_control.power_charge = input_power; //�������ݵ�����繦��
	
// 	if(chassis_motor->power_control.power_charge>150)		{chassis_motor->power_control.power_charge =150;}//�ο�������ư����������繦�ʣ�Ϫ�ذ��ӵ����ϲ�һ��
	
// 	if(can_send_tmp % 25  == 0)
// 		CAN_cmd_cap(chassis_motor->power_control.power_charge); // ���ó���ĳ�繦��
	
//  // CAN_cmd_cap(chassis_motor->power_control.power_charge);//��������

// 	if ((get_capB.output_voltage /1000) > 16) //�������ѹ����ĳ��ֵ(��ֹC620����)
// 	{
// 		if (chassis_move.chassis_RC->mouse.press_r)   //�������磬һ�������𲽼���or���or����or���£�chassis_move.key_CΪ�˴����г��翪������

// 		{
// 			chassis_motor->power_control.POWER_MAX = 150;		
// 		}
// 		else
// 		{  
// 				chassis_motor->power_control.POWER_MAX = input_power+10;  //�������磬����������ܵø����
	
// 		}
// 	}
// 	else
// 	{ 
// 		chassis_motor->power_control.POWER_MAX = chassis_motor->chassis_power_MAX;
// 	}
//	if(chassis_move.chassis_power_buffer<23 ){		chassis_motor->power_control.POWER_MAX = chassis_motor->chassis_power_MAX-30;}
//
//	for (uint8_t i = 0; i < 4; i++) // �������3508����Ĺ��ʺ��ܹ���
//	{
//		chassis_motor->power_control.forecast_motor_power[i] =
//    		chassis_motor->Chassis_3508[i].give_current * toque_coefficient * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm //ת�ع���
//		    +k1 * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm//�ٶȹ��� 
//		    +k2* chassis_motor->Chassis_3508[i].give_current *chassis_motor->Chassis_3508[i].give_current 
//		    + constant;
//
//		if (chassis_motor->power_control.forecast_motor_power < 0)  	continue; // ���Ը���
//		
//		chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i];//����ܹ���+Ԥ�⹦��=ʵ�ʹ���
//	}
//	
// 	if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // ������ģ��˥��
// 	{
// 		fp32 power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;
// 		for (uint8_t i = 0; i < 4; i++)
// 		{
// 			scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // ���˥����Ĺ���
			
// 			if (scaled_motor_power[i] < 0)		continue;

// 			fp32 b = toque_coefficient * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm;
// 			fp32 c = k1 * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm * chassis_motor->Chassis_3508[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

// 			if (chassis_motor->Chassis_3508[i].give_current> 0)  //���ⳬ��������
// 			{					
// 				chassis_motor->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);  
// 				if (chassis_motor->power_control.MAX_current[i] > 16000)
// 				{
// 					chassis_motor->Chassis_3508[i].give_current = 16000;
// 				}
// 				else
// 					chassis_motor->Chassis_3508[i].give_current = chassis_motor->power_control.MAX_current[i];
// 			}
// 			else
// 			{
// 				chassis_motor->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
// 				if (chassis_motor->power_control.MAX_current[i] < -16000)
// 				{
// 					chassis_motor->Chassis_3508[i].give_current = -16000;
// 				}
// 				else
// 					chassis_motor->Chassis_3508[i].give_current = chassis_motor->power_control.MAX_current[i];
// 			}
// 		}
// 	}
// }

