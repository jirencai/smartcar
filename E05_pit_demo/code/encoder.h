/*
 * encoder.h
 *
 *  Created on: 2025年10月11日
 *      Author: ji_rencai
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

//编码器定时中断宏定义
#define ENCODER_2                   (TIM5_ENCODER)
#define ENCODER_2_A                 (TIM5_ENCODER_CH1_P10_3)
#define ENCODER_2_B                 (TIM5_ENCODER_CH2_P10_1)

#define ENCODER_1                   (TIM2_ENCODER)
#define ENCODER_1_A                 (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_1_B                 (TIM2_ENCODER_CH2_P33_6)

#define ENCODER_4                   (TIM6_ENCODER)
#define ENCODER_4_A                 (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_4_B                 (TIM6_ENCODER_CH2_P20_0)

#define ENCODER_3                   (TIM4_ENCODER)
#define ENCODER_3_A                 (TIM4_ENCODER_CH1_P02_8)
#define ENCODER_3_B                 (TIM4_ENCODER_CH2_P00_9)

extern float encoder_data_4;
extern float encoder_data_2;

extern float encoder_data_4_;
extern float encoder_data_2_;


void encoderInit(void);

#endif /* CODE_ENCODER_H_ */
