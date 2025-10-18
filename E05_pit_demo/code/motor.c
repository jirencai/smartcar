/*
 * motor.c
 *
 *  Created on: 2025年10月9日
 *      Author: ji_rencai
 */
#include "motor.h"
//#define PIT_NUM                 (CCU60_CH0 )                                     // 使用的周期中断编号

//#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
//#define DIR_R1              (P02_4)
//#define PWM_R1              (ATOM0_CH5_P02_5)
//#define DIR_L1              (P02_6)
//#define PWM_L1              (ATOM0_CH7_P02_7)


/*电机初始化*/
uint16_t dutyL = 0;
uint16_t dutyR = 0;
//电机初始化过程
void motorInit(void)
{
//    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
//    pwm_init(PWM_R1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
//    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
//    pwm_init(PWM_L1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
}




