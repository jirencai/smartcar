/*
 * garage.c
 *
 *  Created on: 2025年11月5日
 *      Author: ji_rencai
 */

#include "garage.h"

enum garage_type_e garage_type = GARAGE_OUT;

uint8 garage_cnt = 0;                       //斑马线计数

// 编码器，用于防止重复触发等情况。
float garage_encoder = 0;
float garage_dist;

bool zebra_flag = 0;                        //斑马线标志位
float garage_yaw = 0;                       //斑马线角度

int zebra_cross_flag_begin = 0;             //斑马线经过开始标志位
int zebra_cross_flag0[30];                  //起始点向左, 这次到下次跳变间的像素数
int zebra_cross_flag0_num = 0;
int zebra_cross_flag1[30];                  //起始点向右, 这次到下次跳变间的像素数
int zebra_cross_flag1_num = 0;

float (*garage_rpts)[2];                    //斑马线坐标
int garage_rpts_num;

void check_garage() {
//    // 记录出库陀螺仪初值
//    if (!garage_inited) {
//        garage_encoder = Total_mileage;
//        garage_yaw=Yaw_a;
//        garage_inited = true;
//    }
    // 非车库模式下识别车库
//    if (garage_type == GARAGE_NONE) {
        // 根据当前的L角点情况决定，取左(右)中线，进行斑马线的寻找
        if (is_straight1 && !Lpt0_found && !Lpt1_found) {
            // 在右中线上找斑马线
            garage_rpts = rptsc1;
            garage_rpts_num = rptsc1_num;
        } else if (is_straight0 && !Lpt0_found && !Lpt1_found) {
            // 仅右角点，则在左中线上找斑马线
            garage_rpts = rptsc0;
            garage_rpts_num = rptsc0_num;
        } else {
            // 其余情况不找
            garage_rpts_num = 0;
        }
        int pt[2];
        // 没有能找的中线就退出
        if (garage_rpts_num == 0) return;
        // 在中线上一定范围内找斑马线
        for (int i = 10; i < MIN(80, garage_rpts_num); i++) {
            // 反变换后，超出图像范围则跳过
//            if (!map_inv(garage_rpts[i], pt)) return;
            if (!Re_IPM(garage_rpts[i][0], garage_rpts[i][1], pt)) return;
            // 调试绘图
            //draw_x(&img_raw, pt[0], pt[1], 3, 0);

            zebra_cross_flag_begin = AT_IMAGE(&img_raw, pt[0], pt[1]) > thres;// 起始点黑白情况

            memset(zebra_cross_flag0, 0, sizeof(zebra_cross_flag0));
            zebra_cross_flag0_num = 0;//起始点向左跳变次数
            for (int x = pt[0] - 1; x >= MAX(0, pt[0] - 50); x--) {
                if (zebra_cross_flag_begin == 0) { // 起始为黑 even white, odd black
                    if        (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // 白->白 current even, current white
                        zebra_cross_flag0[  zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // 白->黑 current even, current black
                        zebra_cross_flag0[++zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // 黑->白 current odd, current white
                        zebra_cross_flag0[++zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // 黑->黑 current odd, current black
                        zebra_cross_flag0[  zebra_cross_flag0_num]++;
                    }
                } else { // 起始为白 even black, odd white
                    if        (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // 白->白 current even, current white
                        zebra_cross_flag0[++zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // 白->黑 current even, current black
                        zebra_cross_flag0[  zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // 黑->白 current odd, current white
                        zebra_cross_flag0[  zebra_cross_flag0_num]++;
                    } else if (zebra_cross_flag0_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // 黑->黑 current odd, current black
                        zebra_cross_flag0[++zebra_cross_flag0_num]++;
                    }
                }
            }

            memset(zebra_cross_flag1, 0, sizeof(zebra_cross_flag1));
            zebra_cross_flag1_num = 0;//起始点向右跳变次数
            for (int x = pt[0] + 1; x <= MIN(img_raw.width - 2, pt[0] + 50); x++) {
                if (zebra_cross_flag_begin == 0) { // 起始为黑 even white, odd black
                    if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // current even, current white
                        zebra_cross_flag1[zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // current even, current black
                        zebra_cross_flag1[++zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // current odd, current white
                        zebra_cross_flag1[++zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // current odd, current black
                        zebra_cross_flag1[zebra_cross_flag1_num]++;
                    }
                } else { // 起始为白 even black, odd white
                    if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // current even, current white
                        zebra_cross_flag1[++zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 0 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // current even, current black
                        zebra_cross_flag1[zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) > thres) { // current odd, current white
                        zebra_cross_flag1[zebra_cross_flag1_num]++;
                    } else if (zebra_cross_flag1_num % 2 == 1 && AT_IMAGE(&img_raw, x, pt[1]) < thres) { // current odd, current black
                        zebra_cross_flag1[++zebra_cross_flag1_num]++;
                    }
                }
            }

//            disp(13,zebra_cross_flag0_num,"zebra0");
//            disp(14,zebra_cross_flag1_num,"zebra1");

            // 判断连续跳变的阈值条件以识别斑马线
            int i0 = 1;
            for (; i0 < zebra_cross_flag0_num - 1; i0++) {
                if (zebra_cross_flag0[i0] < 1 || zebra_cross_flag0[i0] >= 20 || abs(zebra_cross_flag0[i0 + 1] - zebra_cross_flag0[i0]) >= 10) break;
            }   //两次跳变间的像素数需在2~20之间, 相邻两个间隔之差小于10个像素
            bool is_zebra0 = i0 > 5;//单边跳变次数大于一定值

            int i1 = 1;
            for (; i1 < zebra_cross_flag1_num - 1; i1++) {
                if (zebra_cross_flag1[i1] < 1 || zebra_cross_flag1[i1] >= 20 || abs(zebra_cross_flag1[i1 + 1] - zebra_cross_flag1[i1]) >= 10) break;
            }
            bool is_zebra1 = i1 > 5;

            if (is_zebra0 && is_zebra1)
            {
                zebra_flag = 1;
//                zebra_time = fre_cy.time;
//                garage_type = GARAGE_IN;
//                garage_yaw = Yaw_a + Rad2Ang*get_straight_yaw(1);
//                garage_encoder=Total_mileage;
//                garage_dist=120.0f-garage_rpts[i][1];
//            }
            }
       }
}

//斑马线进行时
void run_garage(void) {
    if(garage_cnt){
        garage_cnt--;
        return;
    }
    switch (garage_type) {
        case GARAGE_OUT:
//            now_aim_distance = 0.6;
//            track_route = TRACK_LEFT;
//            track_method = ROUND_SCAN;
            // 转向45度，则出库完毕
            if (Total_mileage-garage_encoder>0.6f) {
//                Slope_Mode=0;
                garage_type = GARAGE_NONE;
//                track_method = EXTEND_LINE;
            }
            break;
        case GARAGE_IN:
//            now_aim_distance = 0.7;
//            track_route = TRACK_LEFT;
//            Slope_Mode=1;
//            track_method = ROUND_SCAN;
            if (is_straight0 && is_straight1){
                garage_yaw=Yaw_a+Rad2Ang*get_straight_yaw(0);
            }else if(is_straight0){
                garage_yaw=Yaw_a+Rad2Ang*get_straight_yaw(-1);
            }else if(is_straight1){
                garage_yaw=Yaw_a+Rad2Ang*get_straight_yaw(1);
            }
            // 转向45度且识别到双L角点，则入库完毕
//            if (get_yaw_error(garage_yaw) > 50) {
                garage_type = GARAGE_STOP;
//                basic_img_with_IN=1;
//            }
            break;
        case GARAGE_STOP:
            if (Total_mileage-garage_encoder>/*garage_dist+*/1.0f) {
                garage_type = GARAGE_END;
//                Slope_Mode=0;
            }
            break;
        default:
            (void) 0;
    }
}

