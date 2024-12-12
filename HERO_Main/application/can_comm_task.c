/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据顺序发送
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "can_comm_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crc8_crc16.h"
#include "CAN_receive.h"
#define  UIT_CAN hcan1
#define  UIO_CAN hcan1
#define  CAP     hcan1
/**
 * @brief can通信线程初始化, 主要任务为开辟线程队列
 * 
 * @param can_comm_init can通信线程初始化结构体
 */
static void can_comm_task_init(can_comm_task_t *can_comm_init);

/**
 * @brief can通信任务发送函数，通信数据队列发送 
 * 
 * @param can_comm_transmit can通信任务控制结构体
 */
static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit);

/**
 * @brief can通信队列添加函数 
 * 
 * @param add_comm_queue can通信任务函数
 * @param comm_data can通信数据
 */
static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data);

/**
 * @brief can通信队列数据更新
 * 
 * @param feedback_update can通信队列结构体
 */
static void can_comm_feedback_update(can_comm_task_t *feedback_update);

//云台can通信数据
//static can_comm_data_t gimbal_can_comm_data = {
//    .can_handle = &GIMBAL_CAN, // 初始化云台通信设备can
//    .can_comm_target = CAN_COMM_GIMBAL,
//};

//双板can通信数据
static can_comm_data_t board_can_comm_data = {
    .can_handle = &UIT_CAN, // 初始化双板通信设备can
    .can_comm_target = CAN_COMM_UIT,
};

//发弹can通信数据
static can_comm_data_t shoot_can_comm_data = {
    .can_handle = &UIO_CAN, // 初始化发弹通信设备can
    .can_comm_target = CAN_COMM_UIO,
};

//裁判系统通信数据
//static can_comm_data_t cap_data = {
//    .can_handle = &CAP, // 初始化裁判系统通信设备can
//    .can_comm_target = CAN_COMM_CAP,
//};

bool init_finish = false;

//实例化can通信线程结构体,全局变量，保证数据一直存在
can_comm_task_t can_comm = { 0 };


void can_comm_task(void const* pvParameters)
{ 
    //要在云台和底盘任务开始之前完成该任务的初始化
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    //can通信任务初始化
    can_comm_task_init(&can_comm);
    init_finish = true;
    while(1)
    {
        //can通信参数更新
        can_comm_feedback_update(&can_comm);
        //can通信数据发送
        can_comm_task_transmit(&can_comm);
        //can数据数据发送 
        vTaskDelay(CAN_COMM_TASK_TIME);
    }
}

static void can_comm_feedback_update(can_comm_task_t *feedback_update)
{
    feedback_update->can_comm_queue->size = can_comm_queue_size(feedback_update->can_comm_queue);
}

static void can_comm_task_init(can_comm_task_t *can_comm_init)
{
    if (can_comm_init == NULL)
        return; 
    //创建并初始化can通信队列
    can_comm_init->can_comm_queue = can_comm_queue_init(); 
}

static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit)
{
    if (can_comm_transmit == NULL)
        return;
    //队列非空发送
    if (!can_comm_queue_is_empty(can_comm_transmit->can_comm_queue))
    {
        can_comm_data_t *data = can_comm_queue_pop(can_comm_transmit->can_comm_queue);
        if(data)
        {
            can_transmit(data);
        }
    }
}


static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data)
{
    if (add_comm_queue == NULL || comm_data == NULL)
        return;
    //添加数据到发送队列中
    can_comm_queue_push(add_comm_queue->can_comm_queue, comm_data);
}


//void can_comm_gimbal(int16_t KEy_1, int16_t KEy_2, int16_t KEy_3 ,int16_t KEy_4)
//{
//    //配置can发送数据
//    gimbal_can_comm_data.transmit_message.StdId = 0x1FF;
//    gimbal_can_comm_data.transmit_message.IDE = CAN_ID_STD;
//    gimbal_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
//    gimbal_can_comm_data.transmit_message.DLC = 0x08;
//    gimbal_can_comm_data.data[0] =(KEy_1 >> 8);
//    gimbal_can_comm_data.data[1] = KEy_1;
//    gimbal_can_comm_data.data[2] = (KEy_2 >> 8);
//    gimbal_can_comm_data.data[3] = KEy_2;
//    gimbal_can_comm_data.data[4] = (KEy_3 >> 8);
//    gimbal_can_comm_data.data[5] = KEy_3;
//    gimbal_can_comm_data.data[6] = (KEy_4 >> 8);
//    gimbal_can_comm_data.data[7] = KEy_4;
//    //添加数据到通信队列
//    add_can_comm_queue(&can_comm, &gimbal_can_comm_data);
//}

void can_comm_UIT(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour)
{
    //配置can发送数据
    board_can_comm_data.transmit_message.StdId = 0x209;
    board_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    board_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    board_can_comm_data.transmit_message.DLC = 0x08;
    board_can_comm_data.data[0] = (relative_angle >> 8);
    board_can_comm_data.data[1] = relative_angle;
    board_can_comm_data.data[2] = (chassis_vx >> 8);
    board_can_comm_data.data[3] = chassis_vx;
    board_can_comm_data.data[4] = (chassis_vy >> 8);
    board_can_comm_data.data[5] = chassis_vy;
    board_can_comm_data.data[6] = (chassis_behaviour >> 8);
    board_can_comm_data.data[7] = chassis_behaviour;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}

void can_comm_UIO(int16_t KEY_1, int16_t KEY_2, int16_t KEY_3 ,int16_t KEY_4)
{
    //配置can发送数据
    shoot_can_comm_data.transmit_message.StdId = 0x208;
    shoot_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    shoot_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    shoot_can_comm_data.transmit_message.DLC = 0x08;
    shoot_can_comm_data.data[0] = (KEY_1 >> 8);
    shoot_can_comm_data.data[1] = KEY_1;
    shoot_can_comm_data.data[2] = (KEY_2 >> 8);
    shoot_can_comm_data.data[3] = KEY_2;
    shoot_can_comm_data.data[4] = (KEY_3 >> 8);
    shoot_can_comm_data.data[5] = KEY_3;
    shoot_can_comm_data.data[6] = (KEY_4 >> 8);
    shoot_can_comm_data.data[7] = KEY_4;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &shoot_can_comm_data); 
}

//void can_comm_supercap(uint8_t temPower )
//{
//	can_control_typedef send_cap_data = {0};
////	  uint32_t send_mail_box;
//    cap_data.transmit_message.StdId = 0x21;
//    cap_data.transmit_message.IDE = CAN_ID_STD;
//    cap_data.transmit_message.RTR = CAN_RTR_DATA;
//    cap_data.transmit_message.DLC = 0x08;
//	
//	send_cap_data.p_set = temPower;
//	send_cap_data.power_source = CAPACITY;
//	send_cap_data.out_auto_en = 1;
//	send_cap_data.wireless_en = 1;
//	send_cap_data.freq_feedback = 100;
//	
//	append_CRC8_check_sum((uint8_t *) &send_cap_data,sizeof(send_cap_data));
//	memcpy(cap_data.data,&send_cap_data,8);
//	add_can_comm_queue(&can_comm, &cap_data); 
//}



bool can_comm_task_init_finish(void)
{
    return init_finish;
}

