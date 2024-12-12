/**
 * @file can_comm_task.h
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据队列发送
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H 
#include "can_comm.h"
#include "can.h"

//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 100
//can通信任务运行时间间隔 单位ms
#define CAN_COMM_TASK_TIME 1
////云台can设备
//#define GIMBAL_CAN hcan1
////双板can通信设备
//#define BOARD_CAN hcan1
////发弹can通信设备
//#define SHOOT_CAN hcan1
////裁判系统can通信
//#define REFEREE_CAN hcan1
//typedef enum
////{
////	
////		CAP_ID = 0x211,
////	  CAN_UI=0x212,
////		CAN_TUI=0x213,

////} can_msg_id_e;


//can通信任务结构体
typedef struct
{
    //can通信队列结构体
    can_comm_queue_t *can_comm_queue;
    
}can_comm_task_t;


/**
 * @brief  can通信任务
 * 
 */
void can_comm_task(void const* pvParameters);



/**
 * @brief 双板通信数据发送，云台控制底盘，将数据添加到can_comm线程通信队列中
 * 
 * @param relative_angle 云台相对角
 * @param chassis_vx 底盘x轴速度方向分量
 * @param chassis_vy 底盘y轴速度方向分量
 * @param chassis_behaviour 底盘运动模式
 */
void can_comm_UIT(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour);

/**
 * @brief 发弹电机通信数据发送，发送值为电机电流值，将发送数据添加到can_comm线程的通信队列中
 * 
 * @param fric1 摩擦轮电机电流值
 * @param fric2 摩擦轮电机电流值
 * @param trigger 拨弹盘电机电流值
 */

void can_comm_gimbal(int16_t KEy_1, int16_t KEy_2, int16_t KEy_3 ,int16_t KEy_4);
void can_comm_UIO(int16_t KEY_1, int16_t KEY_2, int16_t KEY_3 ,int16_t KEY_4);

void can_comm_supercap(uint8_t temPower);

bool can_comm_task_init_finish(void);

#endif // !CAN_COMM_TASK_H
