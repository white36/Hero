#include "bsp_laser.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,100);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
}
