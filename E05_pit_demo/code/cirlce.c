/*
 * cirlce.c
 *
 *  Created on: 2025年11月5日
 *      Author: ji_rencai
 */

#include "circle.h"

/*编码器部分，防止一些重复触发*/
float circle_encoder;                           //圆环旋转角编码器计数
float circle_Yaw;                               //圆环旋转角
float circle_lenout=0.3;                        //编码器判断结果（有待商榷）


enum circle_type_e circle_type = CIRCLE_NONE;

uint8 circle_cnt0 = 0;                          //左边线无圆环计数
uint8 circle_cnt1 = 0;                          //右边线无圆环计数

/*检查左圆环和右圆环*/
void check_circle(void) {
    // 非圆环模式下，单边L角点(且角点向下张开), 单边长直道
    if (circle_type == CIRCLE_NONE && Lpt0_found<0 && !Lpt1_found && is_straight1 && abs(Lpt0_found) < 2 && rpts0s[clip(Lpt0_rpts0s_id1+20,Lpt0_rpts0s_id1,rpts0s_num)][0]-rpts0s[Lpt0_rpts0s_id1][0]<-10 && rpts0s[Lpt0_rpts0s_id1][1] < 65) {
//        if(rpts0s[clip(Lpt0_rpts0s_id1+20,Lpt0_rpts0s_id1,rpts0s_num)][0]-rpts0s[Lpt0_rpts0s_id1][0]>-10)return;
        circle_cnt0++;
    }else{
        if(circle_cnt0) circle_cnt0--;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found<0 && is_straight0 && abs(Lpt1_found) < 2 && rpts1s[clip(Lpt1_rpts1s_id1+20,Lpt1_rpts1s_id1,rpts1s_num)][0]-rpts1s[Lpt1_rpts1s_id1][0]>10 && rpts1s[Lpt1_rpts1s_id1][1] < 65) {
        circle_cnt1++;
    }else{
        if(circle_cnt1) circle_cnt1--;
    }
    if(circle_cnt0 > 2){
        circle_type = CIRCLE_LEFT_BEGIN;
        circle_Yaw = Yaw_a + Rad2Ang*get_straight_yaw(1);
        circle_encoder = Total_mileage;
    }else if(circle_cnt1 > 2)
    {
        circle_type = CIRCLE_RIGHT_BEGIN;
        circle_Yaw = Yaw_a + Rad2Ang*get_straight_yaw(-1);
        circle_encoder = Total_mileage;
    }
//    dispf(4,circle_Yaw,"circle_Yaw");
}
#define none_line_thres 0.15f                       //小于该长度判定为丢线
#define have_line_thres 0.5f                        //大于该长度判定为有线
int none_left_line = 0, none_right_line = 0;        //左边线丢失和右边线丢失
int have_left_line = 0, have_right_line = 0;        //左边线找到和右边线找到
uint8 Circle_In_Ready=0;                            //进入圆环准备

/*圆环进行时*/
void run_circle() {
//    now_aim_distance = 0.45;
    static uint32_t circle_begin_start_time = 0; // 记录进入该状态的起始时间（单位：秒）

    if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN)
    {
        if(abs(Lpt0_found) == 2 || abs(Lpt1_found) == 2)
        {
            circle_type = CIRCLE_NONE;
            return;
        }
        // 如果刚进入该状态，记录当前时间
        if (circle_begin_start_time == 0)
        {
            circle_begin_start_time = fre_cy.time;
        }
        // 如果当前状态持续超过2秒，则超时处理
        if (fre_cy.time - circle_begin_start_time >= 3 || (is_long_straight0 && is_long_straight1)) {
            circle_type = CIRCLE_NONE;
            circle_begin_start_time = 0; // 重置定时器
            return; // 直接返回，避免后续状态处理
        }
    }
    else
    {
        // 当不处于上述两种状态时，清空定时器变量
        circle_begin_start_time = 0;
    }

    // 左环开始，寻外直道右线
    if (circle_type == CIRCLE_LEFT_BEGIN)
    {
//        Set_Speed_Key(now_using_map,circle_begin);
//        if(none_left_line&&(!have_left_line))
        Extra_search_path = false;
        track_route = TRACK_RIGHT;
        if (Lpt0_found<0 && is_straight1)circle_encoder = Total_mileage;
        if(!none_left_line && is_straight1)circle_Yaw = 0.5f * (circle_Yaw + Yaw_a + Rad2Ang*get_straight_yaw(1));
        //先丢左线后有线
        if (rpts0s_num < none_line_thres / sample_dist) { none_left_line++; }
        if ((Lpt0_found==0 && rpts0s_num > have_line_thres / sample_dist && none_left_line > 2)||Total_mileage-circle_encoder> circle_lenout) {
            have_left_line++;
            if (have_left_line > 2) {
                if(!Circle_In_Ready)Circle_In_Ready=1;
                if(Circle_In_Ready==2){
                    Circle_In_Ready=0;
                    circle_type = CIRCLE_LEFT_IN;
                    none_left_line = 0;
                    have_left_line = 0;
                    circle_encoder = Total_mileage;
                }
            }
        }
    }
    //入环，寻内圆左线
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_route = TRACK_LEFT;
        //编码器打表过1/4圆   应修正为右线为转弯无拐点
        if ((/*(rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f )  ||*/
//            current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            get_yaw_error(circle_Yaw)>90.0f) && !infinite_circle) { circle_type = CIRCLE_LEFT_RUNNING; }
    }
    //正常巡线，寻外圆右线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_route = TRACK_RIGHT;
//        if (Lpt1_found) rpts1s_use = Lpt1_rpts1s_id1;//外环拐点(右L点)
        if (Lpt1_found) {  //&& Lpt1_rpts1s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_route = TRACK_LEFT;

        //右线为长直道
        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
//            Set_Speed_Key(now_using_map,circle_end);
        }
    }
    //走过圆环，寻右线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_route = TRACK_RIGHT;

        //左线先丢后有
        if (rpts0s_num < none_line_thres / sample_dist) { none_left_line++; }
        if (rpts0s_num > have_line_thres / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
            Extra_search_path = true;
        }
    }
    //右环控制，前期寻左直道
    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
//        quit_time = fre_cy.time;
//        if(fre_cy.time - quit_time > 2){
//            circle_type = CIRCLE_NONE;
//            quit_time = 0;
//        }
//        Set_Speed_Key(now_using_map,circle_begin);
        track_route = TRACK_LEFT;
        if (Lpt1_found<0 && is_straight0)circle_encoder = Total_mileage;
//        if(none_left_line&&(!have_left_line))
            Extra_search_path = false;
        if(!none_right_line && is_straight0) circle_Yaw = 0.5f * (circle_Yaw + Yaw_a + Rad2Ang*get_straight_yaw(-1));
        //先丢右线后有线
        if (rpts1s_num < none_line_thres / sample_dist) { none_right_line++; }
        if ((Lpt1_found==0 && rpts1s_num > have_line_thres / sample_dist && none_right_line > 2)||Total_mileage-circle_encoder> circle_lenout) {
            have_right_line++;
            if (have_right_line > 2) {
                if(!Circle_In_Ready)Circle_In_Ready=1;
                if(Circle_In_Ready==2){
                    Circle_In_Ready=0;
                    circle_type = CIRCLE_RIGHT_IN;
                    none_right_line = 0;
                    have_right_line = 0;
                    circle_encoder = Total_mileage;
                }

            }
        }
    }
    //入右环，寻右内圆环
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_route = TRACK_RIGHT;

        //编码器打表过1/4圆   应修正为左线为转弯无拐点
        if ((/*( rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f ) ||*/
            get_yaw_error(circle_Yaw)<(-90.0f)
//            ||current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            ) && !infinite_circle) { circle_type = CIRCLE_RIGHT_RUNNING; }

    }
    //正常巡线，寻外圆左线
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_route = TRACK_LEFT;

        //外环存在拐点,可再加拐点距离判据(左L点)
//        if (Lpt0_found) rpts0s_use = Lpt0_rpts0s_id1;
        if (Lpt0_found) { //&& Lpt0_rpts0s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_route = TRACK_RIGHT;

        //左长度加倾斜角度  应修正左右线找到且为直线
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
        //走过圆环，寻左线
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_route = TRACK_LEFT;

        //右线先丢后有,
        if (rpts1s_num < none_line_thres / sample_dist) { none_right_line++; }
        if (rpts1s_num > have_line_thres  / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
            Extra_search_path = true;
        }
    }
}

void Circle_In_Ready_Check(void){
    static float dist_min, dist_now;//, dir_diff;
    if (track_route == TRACK_LEFT){
        find_nearest_dist(rptsn[aim_idx][0], rptsn[aim_idx][1], rptsc1, rptsc1_num, &dist_now, 100.0f);
//        float dx1 = rptsn[aim_idx][0] - rptsn[clip(aim_idx+, 0, num - 1)][0];
//        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
//        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
//        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
    } else {
        find_nearest_dist(rptsn[aim_idx][0], rptsn[aim_idx][1], rptsc0, rptsc0_num, &dist_now, 100.0f);
//        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
//        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
//        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
//        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
    }
    if(dist_now<dist_min)dist_min=dist_now;
//    dir_diff = atan2f(dx1*dy2-dy1*dx2,dx1*dx2+dy1*dy2);
    if (dist_now<5.0f || dist_now-dist_min>5.0f) Circle_In_Ready=2;


}
