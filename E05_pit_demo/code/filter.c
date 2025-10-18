/*
 * filter.c
 *
 *  Created on: 2025年10月18日
 *      Author: ji_rencai
 */
#include "filter.h"

extern float err_s; //主动偏差系数与被动偏差系数
float fc = 20.5f;      //截止频率
float fc_out = 1.0f;   //截止频率
float Ts = 0.002f;     //采样周期
float pi = 3.14159f;   //π
float alpha = 0;       //滤波系数
float alpha_out = 0;   //滤波系数

void low_pass_filter_Init(void)
{
    float b = 2.0 * pi * fc * Ts;
    alpha = b / (b + 1);

    b = 2.0 * pi * fc_out * Ts;
    alpha_out = b / (b + 1);
}

float filter_r(float value)                                                     //右轮滤波
{
    static float out_last = 0; //上一次滤波值
    float out;
    
    /***************** 如果第一次进入，则给 out_last 赋值 ******************/
    static char fisrt_flag = 1;
    if (fisrt_flag == 1)
    {
        fisrt_flag = 0;
        out_last = value;
    }
    
    /*************************** 一阶滤波 *********************************/
    out = out_last + alpha * (value - out_last);
    out_last = out;
    
    return out;
}

float filter_l(float value)                                                //左轮滤波
{
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

