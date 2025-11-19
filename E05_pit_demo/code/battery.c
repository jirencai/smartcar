/*
 * battery.c
 *
 *  Created on: 2025年11月18日
 *      Author: ji_rencai
 */
#include "battery.h"

uint8_t isLow = 0;                                  //检测电池是否过放
uint8_t isHigh = 0;                                 //检测电池是否过冲
float batteryVoltage;                               //adc得到的电池电压

void batteryInit(void)
{
    /*adc引脚初始化*/
    adc_init(ADC0_CH11_A11, ADC_12BIT);

    /*蜂鸣器引脚初始化*/
    gpio_init(P33_10, GPO, 1, GPO_PUSH_PULL);

}

void batteryDetect(void)
{
    batteryVoltage = adc_mean_filter_convert(ADC0_CH11_A11, 10);        //均值滤波得到电池电压
    /*过放现象*/
    if (batteryVoltage < LOW_VOLTAGE)
    {
        isLow = 1;
        gpio_set_level(P33_10, 0);
    }
    /*过冲现象*/
    else if (batteryVoltage > HIGH_VOLTAGE)
    {
        isHigh = 1;
        gpio_set_level(P33_10, 0);
    }
}

