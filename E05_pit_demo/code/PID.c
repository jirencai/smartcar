/*
 * PID.c
 *
 *  Created on: 2025年10月11日
 *      Author: ji_rencai
 */
#include "PID.h"



pidTypedef pid_speed_r = {
        0,                      //输出值
        0, 0, 0, 0, 0,          //目标值，上一次的目标值，实际值，积分量
        0, 0, 0,                //此次误差，上次误差，上上次误差
        0, 5, 0, 1,          //acc, Kp, Ki, Kd
        10, 0.1,                //kff1, kff2
        1000.0,                 //积分限幅
        0, 0, 0, 100.0          //Kp结果, Ki结果, Kd结果, 积分过度
};

pidTypedef pid_speed_l = {
        0,
        0, 0, 0, 0, 0,
        0, 0, 0,
        0, 5, 0, 1,
        10, 0.1,
        1000.0,
        0, 0, 0, 100.0
};


turnPidTypedef turnPid = {
        0, 0, 0,
        80, 5, 0, 0, 10,
        1200, 0
};
/*位置式PID电机算法*/
void PID_Motor(pidTypedef *p,float nowSpeed)//电机转速pid
{
    p->err = p->target_val - nowSpeed;

    /*位置式PID*/
    p->integral += p->err;                                  //得到误差
    p->integral = Limit(p->integral,p->limit);              //积分限幅

    p->p_result = p->Kp * p->err;                           //计算Kp
    p->i_result = p->Ki * p->integral;                      //计算Ki
    p->d_result = p->Kd * (p->err-p->err_last);             //计算Kd

    p->actual_val = p->p_result                             //得到闭环输出
                  + p->i_result
                  + p->d_result;

    /*输出PID*/
    p->output = p->actual_val;
    p->output = Limit(p->output,Duty_max);                  //对输出结果限幅

    /*误差传递*/
    p->err_previous= p->err_last;                           //误差传递
    p->err_last    = p->err;
}

/*增量式PID电机算法*/
void PID_SpeedMotor(pidTypedef *p,float nowSpeed)
{
    p->err = p->target_val - nowSpeed;

    /*增量式PID*/
    p->p_result = p->Kp * (p->err-p->err_last);
    p->i_result = p->Ki * p->err;
    p->d_result = p->Kd * (p->err-2*p->err_last+p->err_previous);

    p->actual_val = p->p_result                             //得到闭环输出
                  + p->i_result
                  + p->d_result;

    /*输出PID*/
    p->outputSpeed = p->actual_val;
    p->outputSpeed = Limit(p->outputSpeed,Duty_max);                  //对输出结果限幅

    /*误差传递*/
    p->err_previous= p->err_last;                           //误差传递
    p->err_last    = p->err;
}

/*
    开环力矩前馈补偿位置式PID
    需要在PID函数被调用以后再用
*/
void PID_torque_compensation(pidTypedef *p)
{
    /*开环力矩*/
    p->acc = p->target_val - p->target_last_val;            //得到加速度
    p->uff = p->kff1 * p->target_val + p->kff2 * p->acc;    //前馈值

    /*开环力矩补偿PID*/
    p->output += p->uff;

    /*误差传递*/
    p->target_last_val = p->target_val;                     //传递加速度
}

/*转角映射*/
float PID_turn(turnPidTypedef *p, float imgError, float gyroValue)
{
    /*误差参数的获取*/
    p->error = imgError;
    p->gyro = gyroValue;

    /*PID计算得到*/
    p->output = p->error * p->Kp
            + p->error * abs(p->error) * p->Kp2
            + (p->error - p->last_error) * p->Kd
            - p->gyro * p->GKD;

    /*误差传递*/
    p->last_error = p->error;
    p->output = Limit(p->output, p->outLimit);
    /*输出返回值*/
    return p->output;
}


