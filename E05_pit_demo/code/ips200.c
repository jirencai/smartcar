/*
 * ips200.c
 *
 *  Created on: 2025年10月10日
 *      Author: ji_rencai
 */
#include "ips200.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8

/*
 * 屏幕初始化
 */
void ips200Init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}

