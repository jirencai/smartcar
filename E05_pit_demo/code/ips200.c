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
void drawleftline(void)//画左边线
{
    uint16 x,y;
    for(int i=0;i<POINTS_MAX_LEN-2;i+=1)
    {
        x=ipts0[i][0];    //之前是搜边界没标出来
        y=ipts0[i][1];
//        ips200_draw_point(x+80,i,RGB565_RED);
        ips200_draw_point(x,y,RGB565_RED);
    }
}
void drawrightline(void)//画右边线
{
    uint16 x,y;
    for(int i=0;i<POINTS_MAX_LEN-2;i+=1)
    {
        x=ipts1[i][0];
        y=ipts1[i][1];
//        ips200_draw_point(x+80,i,RGB565_RED);
        ips200_draw_point(x,y,RGB565_BLUE);
    }
}

