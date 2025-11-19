/*
 * battery.h
 *
 *  Created on: 2025年11月18日
 *      Author: ji_rencai
 */

#ifndef CODE_BATTERY_H_
#define CODE_BATTERY_H_

#include "zf_common_headfile.h"

#define LOW_VOLTAGE 11.25
#define HIGH_VOLTAGE 12.60

extern uint8_t isLow;                                  //检测电池是否过放
extern uint8_t isHigh;                                 //检测电池是否过冲
extern float batteryVoltage;                               //adc得到的电池电压

void batteryInit(void);             //电池检测初始化
void batteryDetect(void);           //电池检测函数

#endif /* CODE_BATTERY_H_ */
