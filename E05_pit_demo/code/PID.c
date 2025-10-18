/*
 * PID.c
 *
 *  Created on: 2025年10月11日
 *      Author: ji_rencai
 */
#include "PID.h"



pidTypedef pid_speed_r = {
        0, 0, 0,
        0, 0, 0,
        10, 1.2, 0,
        500.0,
        0, 0, 0, 100.0
};

pidTypedef pid_speed_l = {
        0, 0, 0,
        0, 0, 0,
        10, 1.2, 0,
        500.0,
        0, 0, 0, 100.0
};

void PID_Motor(pidTypedef *p,float nowSpeed)//电机转速pid
{
    p->err = p->target_val - nowSpeed;
//位置式
//    if(p->actual_val>Duty_max&&p->err<0)    p->integral += p->err  ;
//    else if(p->actual_val<Duty_min&&p->err>0) p->integral += p->err;
    p->integral += p->err;
//    p->integral += p->err;
    p->integral = Limit(p->integral,p->limit);
    p->p_result = p->Kp * p->err;
    p->i_result = p->Ki * p->integral;
    p->d_result = p->Kd * (p->err-p->err_last);
//    p->p_result = p->Kp * (p->err-p->err_last);
//    p->i_result = p->Ki * p->err;
//    p->d_result = p->Kd * (p->err-2*p->err_last+p->err_previous);


    p->actual_val = p->p_result
                  + p->i_result
                  + p->d_result;

    p->actual_val = Limit(p->actual_val,Duty_max);

    p->err_previous= p->err_last;    //误差传递
    p->err_last    = p->err;
}
