/*
 * key.h
 *
 *  Created on: 2025年10月9日
 *      Author: ji_rencai
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_
#include "zf_common_headfile.h"
#include "motor.h"

#define key1      P33_11    //S1
#define key2      P33_12    //S2
#define key3      P20_6     //S3
#define key4      P20_7     //S4
#define key5      P11_2     //S5
#define key6      P11_3     //S6
//设置各个按键的宏定义

struct keys
{
        uint8_t key_read;
        uint8_t key_sta;
        uint8_t key_singleFlag;
        uint8_t key_longFlag;
        uint16_t key_time;
};

void keyProcess(void);      //按键识别过程
void keyScan(void);         //按键检测
void keyInit(void);         //按键初始化


#endif /* CODE_KEY_H_ */
