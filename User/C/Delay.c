#include "Delay.h"
void Delay_us(int time)
{
    __HAL_TIM_SetCounter(&Delay_Tim, 0);
    HAL_TIM_Base_Start(&Delay_Tim);
    while (__HAL_TIM_GetCounter(&Delay_Tim) < time)
        ;
    HAL_TIM_Base_Stop(&Delay_Tim);
}
void Delay_ms(int time)
{
    while (time--)
        Delay_us(1000);
}
void Delay_s(int time)
{
    while (time--)
        Delay_ms(1000);
}
void Delay_min(int time)
{
    while (time--)
        Delay_s(60);
}
