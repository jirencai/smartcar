/*
 * motor.h
 *
 *  Created on: 2025年10月9日
 *      Author: ji_rencai
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
//声明头文件
#include "zf_common_headfile.h"

//宏定义变量

#define DIR_R               (P21_5)
#define PWM_R               (ATOM0_CH2_P21_4)
#define DIR_L               (P21_3)
#define PWM_L               (ATOM1_CH0_P21_2)

#define DUTYMAX             4000      //最大占空比

extern uint16_t dutyL;              //左侧电机占空比
extern uint16_t dutyR;              //右侧电机占空比


void motorInit(void);               //电机初始化
void motorLeftWrite(int16_t compare);
void motorRightWrite(int16_t compare);

#endif /* CODE_MOTOR_H_ */
