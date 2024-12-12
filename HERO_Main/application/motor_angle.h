#ifndef MORTORANGLE_TASK_H
#define MORTORANGLE_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "stdlib.h"
#include "shoot_task.h"


#define ANGLE_PID_KP        1800.0f
#define ANGLE_PID_KI        0.0f
#define ANGLE_PID_KD        10.0f

#define ABGLE_MAX_OUT   15000.0f
#define ABGLE_MAX_IOUT  10000.0f

//一些角度控制的结构体变量
typedef struct
{
	float POS_GAOL;//目标位置
	float POS_ABS;//绝对位置0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}ANGLE_TypeDef;



typedef struct
{
    uint8_t temperate;
	uint16_t  real_angle;
	int16_t  real_current;
	int16_t  speed_rpm;
	uint16_t  angle_value;
}TRRIGER_MOTOR_TypeDef;


typedef struct
{
 float P;
 float I;
 float D;
 float OUT;
 float err;
 float err_err;
 float err_old;
 float P_OUT;
 float I_OUT;
 float D_OUT;
 float I_LIMIT;
 float OUT_LIMIT;

}_PID;

typedef struct
{
	ANGLE_TypeDef ANGLE[8];
	_PID   PID_SPEED[8];
	_PID   PID_ANGLE[8];
	
}MOTOR_TypeDef;



float Motor_Auto_Run();


#endif 

