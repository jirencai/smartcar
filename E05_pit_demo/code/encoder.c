/*
 * encoder.c
 *
 *  Created on: 2025年10月11日
 *      Author: ji_rencai
 */
#include "encoder.h"

float encoder_data_4 = 0;
float encoder_data_2 = 0;

float encoder_data_4_ = 0;
float encoder_data_2_ = 0;


void encoderInit(void)
{
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_dir_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
}
