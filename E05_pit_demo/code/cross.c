/*
 * cross.c
 *
 *  Created on: 2025年11月4日
 *      Author: ji_rencai
 */
#include "cross.h"

bool cross_angle = 0;
int cross_width = 35;                               //十字边线距离边界的距离
uint8 quit_value = 25;                              //出环参数
uint8 top_value = 40;                               //最低点参数

int not_have_line = 0;                              //丢线行为

enum cross_type_e cross_type = CROSS_NONE;

/*编码器值，防止出现一些重复触发*/
float cross_encoder;

//双L角点,切十字模式
void check_cross(void) {
    bool Xfound = Lpt0_found < 0 && Lpt1_found < 0 && Lpt_open==1 && cross_angle;
    if (cross_type == CROSS_NONE && Xfound){
        cross_type = CROSS_BEGIN;
        Extra_search_path=0;

//        float dir_diff = get_corner_yaw(0)+PI/2.0f;
//        float dx = 0.01f*(0.5f*(rpts0s[Lpt0_rpts0s_id1][0]+rpts1s[Lpt1_rpts1s_id1][0])-94.0f);
//        float cor_dist=0.5f*(rpts0s[Lpt0_rpts0s_id1][1]+rpts1s[Lpt1_rpts1s_id1][1]);
//        float dy = (120.0f-cor_dist)/117.0f;
//        float dist = dx*dx+dy*dy;
//        float dir = atan2f(dy,dx);
//
//        Aim_X_Pos_c = dist*cosf(dir-dir_diff);//a=d-pi/2,sin_a=-cos_d,cos_a=sin_d
//        aim_yaw_a_c = Yaw_a+Rad2Ang*dir_diff/*-0.5f*Roll_a*/;
    }
}
uint8 use_cross_aim=0;

/*十字进行时*/
void run_cross(void) {
    bool Xfound = Lpt0_found < 0 && Lpt1_found < 0 && Lpt_open==1 && cross_angle;
    static uint32_t cross_begin_start_time = 0; // 记录进入该状态的起始时间（单位：秒）

    if (cross_type == CROSS_BEGIN) {
        // 如果刚进入该状态，记录当前时间
        if (cross_begin_start_time == 0) {
            cross_begin_start_time = fre_cy.time;
        }
        // 如果当前状态持续超过2秒，则超时处理
        if (fre_cy.time - cross_begin_start_time >= 2) {
            cross_type = CROSS_NONE;
            cross_begin_start_time = 0; // 重置定时器
            return; // 直接返回，避免后续状态处理
        }
    } else {
        // 当不处于上述两种状态时，清空定时器变量
        cross_begin_start_time = 0;
    }
    //检测到十字，先按照近线走
    if (cross_type == CROSS_BEGIN) {
//        Taryaw = Yaw_a;
//        now_aim_distance = 0.4;
        if (Xfound) {
////            IN_NAV_local_Ready=0;//重新初始化局部惯导
////            Reset_coord_local(1);
//
////            Aim_X_Pos_c = -IN_NAV_local_X+cross_offset[cross_count];
////            aim_yaw_a_c = Yaw_A0_local+90.0f;
            use_cross_aim=2;
        }
        if (Lpt0_found<0 && Lpt1_found<0 && Lpt1_rpts1s_id1 > Lpt0_rpts0s_id1) {
            track_route = TRACK_RIGHT;
//            rptsc1_num = rpts1s_num = Lpt1_rpts1s_id1;
        }else if(Lpt0_found<0 && Lpt1_found<0 && Lpt1_rpts1s_id1 < Lpt0_rpts0s_id1){
            track_route = TRACK_LEFT;
//            rptsc0_num = rpts0s_num = Lpt0_rpts0s_id1;
        }
//        rptsc1_num = rpts1s_num = Lpt1_rpts1s_id1;
//        rptsc0_num = rpts0s_num = Lpt0_rpts0s_id1;
//        flag = 1;
//        if(Lpt0_found<0 && Lpt1_found<0 && (rpts1s_num > 4 && rpts0s_num > 4)) cross_fill();
        //近角点过少，进入远线控制
        //(Lpt0_rpts0s_id1 < clip(aim_idx-30,0,aim_idx) || Lpt1_rpts1s_id1 < clip(aim_idx-30,0,aim_idx) ) rpts_num - begin_id <= 15
        if (Xfound && (Lpt0_rpts0s_id1 < aim_idx_f - quit_value && Lpt1_rpts1s_id1 < aim_idx_f - quit_value)) { //   || IN_NAV_local_Y>(-now_aim_distance) -8   26
            cross_type = CROSS_IN;
            cross_encoder = Total_mileage;
//            aim_yaw_a_c = aim_yaw_a;
//            lock_yaw = 1;
//            target_yaw = Yaw_a + img_error*1.5f;  //img_error
        }
    }
        //远线控制进十字,begin_y渐变靠近防丢线
    else if (cross_type == CROSS_IN) {
        //寻远线,算法与近线相同
//        farline = 1;
        cross_farline();
        use_cross_aim=1;
        if (far_Lpt1_found) { track_route = TRACK_RIGHT; }
        else if (far_Lpt0_found) { track_route = TRACK_LEFT; }
//        else if (not_have_line > 0 && rpts1s_num > rpts1s_num) { track_route = TRACK_RIGHT;}
//        else if (not_have_line > 0 && rpts0s_num > rpts1s_num) { track_route = TRACK_LEFT; }
        else if ((begin_y - far_y1) > (begin_y - far_y2) && far_y1 && far_y2) { track_route = TRACK_RIGHT; }  // && far_y1 && far_y2
        else if ((begin_y - far_y1) < (begin_y - far_y2) && far_y1 && far_y2) { track_route = TRACK_LEFT; }  // && far_y1 && far_y2
        else if (far_rpts0s_num == 0 && far_rpts1s_num) track_route = TRACK_RIGHT;
        else if (far_rpts1s_num == 0 && far_rpts0s_num) track_route = TRACK_LEFT;
        if (rpts1s_num < 5 && rpts0s_num < 5) { not_have_line++; }
//        if(Lpt0_found<0 && Lpt1_found<0 && (rpts1s_num > 4 && rpts0s_num > 4)) cross_fill();
        if ((not_have_line && (rpts1s_num > 15 && rpts0s_num > 15)) || Total_mileage-cross_encoder>0.55f) {   //||IN_NAV_local_Y>0.3f || Total_mileage-cross_encoder>0.5f
            far_rpts0s_num = 0;
            far_rpts1s_num = 0;
            far_ipts0_num = 0;
            far_ipts1_num = 0;
            cross_type = CROSS_NONE;
//            farline = 0;
//            cross_count++;
//            if(cross_count>3)cross_count=0;
            Extra_search_path=1;
            not_have_line = 0;
            use_cross_aim = 0;
//            if(no_line_cnt>50) no_line_cnt=30;
        }

    }
}

int far_x1 = 64, far_x2 = 124, far_y1, far_y2;                  //远点初始化

/*寻找远点以及提取边线*/
void cross_farline(void) {
//    int cross_width = 35;
    far_x1 = cross_width, far_x2 = img_raw.width -cross_width;
    far_y1 = 0, far_y2 = 0 ;


    int x1 = 94 , y1_c = begin_y; //- begin_dx
    bool white_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);

    //在begin_y向两边找黑线
    for(;x1>cross_width; x1--)
    {
        if(AT_IMAGE(&img_raw, x1-1, y1_c) < thres) {  //&img_raw
          far_x1 = x1 - 4;
          break;
        }
    }
    //全白  far_x1 = 0,从边界找
    for (; y1_c > 0; y1_c--) {
        //先黑后白，先找white
        if (RAW_IMG(far_x1, y1_c) >= thres) { white_found = true; }
        if (RAW_IMG(far_x1, y1_c) < thres && (white_found || far_x1 == cross_width)) {
            far_y1 = y1_c;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (RAW_IMG(far_x1, far_y1 + 1) >= thres && far_y1 > top_value)  //&& far_y1 > 60
        findline_lefthand_adaptive(block_size, clip_value, far_x1, far_y1, far_ipts0, &far_ipts0_num);
    else far_ipts0_num = 0;

    int x2 = 94 , y2_c = begin_y; //+ begin_dx
    white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);

    //在begin_y向两边找黑线
    for(;x2<img_raw.width-cross_width; x2++)
    {
      if(AT_IMAGE(&img_raw, x2+1, y2_c) < thres) {
        far_x2 = x2 + 4;
        break;
      }
    }
    //全白  far_x2 = 0,从边界找
    for (; y2_c > 0; y2_c--) {
        //先黑后白，先找white
        if (RAW_IMG(far_x2, y2_c) >= thres) { white_found = true; }
        if (RAW_IMG(far_x2, y2_c) < thres && (white_found || far_x2 == img_raw.width - cross_width)) {
            far_y2 = y2_c;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (RAW_IMG(far_x2, far_y2 + 1) >= thres && far_y2 > top_value)  //&& far_y2 > 60
        findline_righthand_adaptive(block_size, clip_value, far_x2, far_y2, far_ipts1, &far_ipts1_num);
    else far_ipts1_num = 0;

    // 遍历左边线的每个点，将 x 坐标减去 offset
    for (int i = 0; i < far_ipts0_num; i++) {
        far_ipts0[i][0] += offset;
        if(far_ipts0[i][0] > 187) far_ipts0[i][0] = 187;
        else if(far_ipts0[i][0] < 0) far_ipts0[i][0] = 0;
    }

    // 遍历右边线的每个点，将 x 坐标减去 offset
    for (int i = 0; i < far_ipts1_num; i++) {
        far_ipts1[i][0] += offset;
        if(far_ipts1[i][0] > 187) far_ipts1[i][0] = 187;
        else if(far_ipts1[i][0] < 0) far_ipts1[i][0] = 0;
    }
    // 透视变换
    IPM_init(Pitch_a,-Roll_a);
//    IPM_init_new();
    for (int i = 0; i < far_ipts0_num; i++) {
        IPM(far_ipts0[i][0], far_ipts0[i][1], far_rpts0[i]);
//        IPM_new(far_ipts0[i][0]-RAW_IMG_Offset, far_ipts0[i][1], far_rpts0[i]);
    }
    far_rpts0_num = far_ipts0_num;
    for (int i = 0; i < far_ipts1_num; i++) {
        IPM(far_ipts1[i][0], far_ipts1[i][1], far_rpts1[i]);
//        IPM_new(far_ipts1[i][0]-RAW_IMG_Offset, far_ipts1[i][1], far_rpts1[i]);
    }
    far_rpts1_num = far_ipts1_num;

    // 边线滤波
    blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int) round(line_blur_kernel));
    far_rpts0b_num = far_rpts0_num;
    blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int) round(line_blur_kernel));
    far_rpts1b_num = far_rpts1_num;

    // 边线等距采样
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, sample_dist * pixel_per_meter);
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int) round(angle_dist / sample_dist));
    far_rpts0a_num = far_rpts0s_num;
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int) round(angle_dist / sample_dist));
    far_rpts1a_num = far_rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int) round(angle_dist / sample_dist) * 2 + 1);
    far_rpts0an_num = far_rpts0a_num;
    nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int) round(angle_dist / sample_dist) * 2 + 1);
    far_rpts1an_num = far_rpts1a_num;

    // 找远线上的L角点
    int dist = (int) roundf(angle_dist / sample_dist);
    far_Lpt0_found = far_Lpt1_found = false;
    for (int i = 0; i < MIN(far_rpts0s_num, 100); i++) {
        if (far_rpts0an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        float conf = fabs(far_rpts0a[i]) - (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;
        if (60. / 180. * PI < conf && conf < 120. / 180. * PI && i < 100 && far_rpts0s[clip(i-10,0,far_rpts0s_num)][0] - far_rpts0s[i][0] < -2 && (far_rpts0s[clip(i+dist,0,far_rpts0s_num-1)][1] + far_rpts0s[clip(i-dist,0,far_rpts0s_num-1)][1])*0.5f < far_rpts0s[i][1] && far_rpts0s[i][1] > top_value-15) {
            far_Lpt0_rpts0s_id = i;
            far_Lpt0_found = true;
            break;
        }
    }
    for (int i = 0; i < MIN(far_rpts1s_num, 100); i++) {
        if (far_rpts1an[i] == 0) continue;
        int im1 = clip(i - (int) round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        int ip1 = clip(i + (int) round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        float conf = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;

        if (60. / 180. * PI < conf && conf < 120. / 180. * PI && i < 100 && far_rpts1s[clip(i-10,0,far_rpts1s_num)][0] - far_rpts1s[i][0] > 2 && (far_rpts1s[clip(i+dist,0,far_rpts1s_num-1)][1] + far_rpts1s[clip(i-dist,0,far_rpts1s_num-1)][1])*0.5f < far_rpts1s[i][1] && far_rpts1s[i][1] > top_value-15) {
            far_Lpt1_rpts1s_id = i;
            far_Lpt1_found = true;
            break;
        }
    }
}
