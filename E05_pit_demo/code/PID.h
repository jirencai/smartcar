/*
 * PID.h
 *
 *  Created on: 2025年10月11日
 *      Author: ji_rencai
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

#define Limit(x,y)          (x>y? y: (x<-(y)? -(y): x))

#define Duty_max          1500
#define Duty_min         -1500

typedef struct
{
    float output;                   //位置式输出值
    float outputSpeed;              //增量式输出

    float target_val;               //目标值
    float target_last_val;          //上一次的目标值
    float actual_val;               //实际值PID
    float uff;                      //开环力矩补偿
    float integral;                 //定义积分值

    float err;                      //定义偏差值
    float err_last;                 //  e(k-1)
    float err_previous;             //  e(k-2)

    float acc;              //加速度
    float Kp;               //定义比例、积分、微分系数
    float Ki;               //定义比例、积分、微分系数
    float Kd;               //定义比例、积分、微分系数

    float kff1;             //速度前馈
    float kff2;             //加速度前馈

    float limit;            //积分限幅
    float p_result;         //比例、积分、微分运算结果
    float i_result;         //比例、积分、微分运算结果
    float d_result;         //比例、积分、微分运算结果
    float inte_exce;        //积分过度
}pidTypedef;

typedef struct
{
    float error;        //本次图像误差
    float last_error;   //上一次图像误差
    float gyro;         //陀螺仪数值

    float Kp;           //第一级比例项
    float Kp2;          //第二级比例项
    float Ki;           //积分项
    float Kd;           //微分项
    float GKD;          //陀螺仪参数

    float outLimit;     //输出限幅
    float output;       //pid输出项即转角值
}turnPidTypedef;

extern pidTypedef pid_speed_r;
extern pidTypedef pid_speed_l;
extern turnPidTypedef turnPid;

void PID_Motor(pidTypedef *p,float nowSpeed);                       //电机转速位置式pid
void PID_SpeedMotor(pidTypedef *p,float nowSpeed);                  //电机转速增量式pid
void PID_torque_compensation(pidTypedef *p);                        //开环力矩前馈补偿
float PID_turn(turnPidTypedef *p, float imgError, float gyroValue); //转角映射

#endif /* CODE_PID_H_ */
