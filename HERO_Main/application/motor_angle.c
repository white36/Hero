#include "main.h"
#include "motor_angle.h"

float motor_err[10]; //死区控制

MOTOR_TypeDef motor;

static const fp32 ANGLE_PID[3] = {ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD};
	
motor_measure_t motor_chassis[10];
TRRIGER_MOTOR_TypeDef MOTOR_FEEDBACK[1];
void Get_date(){
		{


			MOTOR_FEEDBACK[1].angle_value  =motor_chassis[9].ecd;
			MOTOR_FEEDBACK[1].speed_rpm    =motor_chassis[9].speed_rpm;
			//MOTOR_FEEDBACK[1].real_current =motor_chassis[9].given_current；
			//MOTOR_FEEDBACK[1].temperature = motor_chassis[9].temperate;
			//MOTOR_FEEDBACK[1].real_angle   = motor_chassis[9].angle_value/8192.0f*360.0f;

		}
}
float ABS(float abs){
if(abs<0){return -abs;}	
else{return abs;}
}

/*void PID_Angle_init(_PID *PID, const fp32 ANGLE_PID[3])
{
    PID->P = ANGLE_PID[0];
    PID->I = ANGLE_PID[1];
    PID->D = ANGLE_PID[2];
    PID->OUT_LIMIT = OUT_LIMIT;
    PID->I_LIMIT = I_LIMIT;
    PID->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    PID->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}



float PID_Cal_Limt(_PID *PID, float limit, float get, float set)//PID死区修改
{
	PID->err = set - get;
	PID->err_err = PID->err - PID->err_old;
	
	PID->P_OUT  = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT  = PID->D * PID->err_err;
	
	PID->I_OUT = (PID->I_OUT > PID->I_LIMIT)?(PID->I_LIMIT):((PID->I_OUT < -PID->I_LIMIT)?(-PID->I_LIMIT):(PID->I_OUT));
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = (PID->OUT > PID->OUT_LIMIT)?(PID->OUT_LIMIT):((PID->OUT < -PID->OUT_LIMIT)?(-PID->OUT_LIMIT):(PID->OUT));
	
	if(ABS(PID->err) <= ABS(limit))
	{
	  PID->I_OUT=0;
	  PID->OUT=0;
	}
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}*/


/// @brief 
/// @param motor_num 
/// @param T 


float Motor_Auto_Run()
{
	motor_err[5]=50;
	
	float abs_err[10];
	static float abs_err_old[10]; 
	Motor_Angle_Cal(5,360);//得到绝对角度
	//发射
	if(mode == 0)
	{
		
	motor.ANGLE[5].POS_GAOL = angle_a;//设定发射角度值
		
		
	PID_Cal_Limt(&motor.PID_ANGLE[5], motor_err[5], motor.ANGLE[5].POS_ABS,motor.ANGLE[5].POS_GAOL);//pid控制计算目标位置
	

	
	abs_err[5] = motor.ANGLE[5].POS_ABS - abs_err_old[5];//绝对误差
	
	
	PID_Cal_Limt( &motor.PID_SPEED[5], 10, abs_err[5], motor.PID_ANGLE[5].OUT);

		
	
	abs_err_old[5] = motor.ANGLE[5].POS_ABS;

		 return (int16_t)(motor.PID_SPEED[6].OUT);//电机速度最终结果
	}


	//回拨
	else if (mode == 1)
	{
		
	motor.ANGLE[5].POS_GAOL = angle_b  ;
	PID_Cal_Limt(&motor.PID_ANGLE[6], motor_err[5], motor.ANGLE[5].POS_ABS,motor.ANGLE[5].POS_GAOL);
	
	abs_err[5] = motor.ANGLE[5].POS_ABS - abs_err_old[5];//
		
	PID_Cal_Limt(&motor.PID_SPEED[6], 10, abs_err[5], motor.PID_ANGLE[6].OUT);
		
	abs_err_old[5] = motor.ANGLE[5].POS_ABS;
		
	return (int16_t)(motor.PID_SPEED[6].OUT);//电机速度最终结果
		
	}

}


//PID




