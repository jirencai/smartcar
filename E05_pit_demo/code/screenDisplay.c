/*
 * screenDisplay.c
 *
 *  Created on: 2025年11月6日
 *      Author: ji_rencai
 */

#include "screenDisplay.h"

extern uint8_t new_image[MT9V03X_H][MT9V03X_W];

void displayProcess(void)
{
    rawImgDisplay();
    lineImgDisplay();
}

void rawImgDisplay(void)
{

   switch(show_flag)
   {
       case 1:
           ips200_show_gray_image(0,0,(uint8*)mt9v03x_image,MT9V03X_W, MT9V03X_H, 188, 120, 160);

           break;
       case 0:
           memcpy(new_image, new_image1, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
           // 绘制左边线
           for (int i = 0; i < ipts0_num; i++) {
               int x = ipts0[i][0];
               int y = ipts0[i][1];
               if (y >= 0 && y < MT9V03X_H && x >= 0 && x < MT9V03X_W) {
                   new_image[y][x] = 255;  // 左边线设为白色
               }
           }

           // 绘制右边线
           for (int i = 0; i < ipts1_num; i++) {
               int x = ipts1[i][0];
               int y = ipts1[i][1];
               if (y >= 0 && y < MT9V03X_H && x >= 0 && x < MT9V03X_W) {
                   new_image[y][x] = 255;  // 右边线设为白色
               }
           }
           for (int i = 0; i < far_ipts0_num; i++) {
               int x = far_ipts0[i][0];
               int y = far_ipts0[i][1];
               if (y >= 0 && y < MT9V03X_H && x >= 0 && x < MT9V03X_W) {
                   new_image[y][x] = 255;  // 左边线设为白色
               }
           }
           for (int i = 0; i < far_ipts1_num; i++) {
               int x = far_ipts1[i][0];
               int y = far_ipts1[i][1];
               if (y >= 0 && y < MT9V03X_H && x >= 0 && x < MT9V03X_W) {
                   new_image[y][x] = 255;  // 左边线设为白色
               }
           }
           ips200_show_gray_image(0,0,(uint8*)new_image,MT9V03X_W, MT9V03X_H, 188, 120, 0);
//               ips200_show_gray_image(0,0,binaryImage,MT9V03X_W, MT9V03X_H, 188, 120, 0);

           break;

   }

}
/*画线函数*/
void lineImgDisplay(void)
{
    clear_image(&img_line);
    SetImg_MoveAndZoom(-16,0.6);//设置平移和缩放(仅对传入的坐标有效,对名称含direct的函数无效)
    float_line_to_img(&img_line, rpts0s, rpts0s_num);//滤波边线
    float_line_to_img(&img_line, rpts1s, rpts1s_num);
    if(1){
        float_line_to_img(&img_line, rptsc0, rptsc0_num);//滤波中线
        float_line_to_img(&img_line, rptsc1, rptsc1_num);
    }else
        float_line_to_img(&img_line, rptsn, rptsn_num);//归一化中线
    // 绘制锚点
    if(rptsn_num)draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
    draw_x(&img_line, rptsn[1][0], rptsn[1][1], 3, 255);
    // 绘制角点
    if(Lpt0_found)draw_o(&img_line, rpts0s[Lpt0_rpts0s_id1][0], rpts0s[Lpt0_rpts0s_id1][1], 3, 255);
//                if(Lpt0_found)draw_o(&img_line, rpts0s[Lpt0_rpts0s_id1+25][0], rpts0s[Lpt0_rpts0s_id1+25][1], 3, 255);
    if(Lpt1_found)draw_o(&img_line, rpts1s[Lpt1_rpts1s_id1][0], rpts1s[Lpt1_rpts1s_id1][1], 3, 255);
//                if(Lpt1_found)draw_o(&img_line, rpts1s[Lpt1_rpts1s_id1+25][0], rpts1s[Lpt1_rpts1s_id1+25][1], 3, 255);
    // 绘制道路元素
    draw_cross();
    draw_circle();
    // 绘制标准宽度参考(45cm
//    draw_road_reference(&img_line, 255);
    ips200_show_gray_image(0, 120, img_line.data[0], MT9V03X_W, MT9V03X_H, 188, 120, 0); //IPM_map img_line.data[0]
}

