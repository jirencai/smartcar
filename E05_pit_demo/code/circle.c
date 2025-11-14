/*
 * circle.c
 *
 *  Created on: 2025锟斤拷11锟斤拷5锟斤拷
 *      Author: ji_rencai
 */

#include "circle.h"

/*编码器累加值，防止出现一些意外*/
float circle_encoder;                           //转弯时候编码器累加值
float circle_Yaw;                               //进入圆环的航向角
float circle_lenout=0.3;                        //偏置航线



enum circle_type_e circle_type = CIRCLE_NONE;

uint8 circle_cnt0 = 0;                          //左圆环确认计数次数
uint8 circle_cnt1 = 0;                          //右圆环确认计数次数

/*圆环检测函数*/
void check_circle(void) {
    // 如果当前并非圆环模式，检测有没有角点来判断圆环位置
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
#define none_line_thres 0.15f                       //无边线阈值
#define have_line_thres 0.5f                        //有边线阈值
int none_left_line = 0, none_right_line = 0;        //初始化无边线计数
int have_left_line = 0, have_right_line = 0;        //初始化有边线计数
uint8 Circle_In_Ready=0;                            //初始化是否进入圆环准备状态

/*进入圆环进行时*/
void circle_run()
{
//    now_aim_distance = 0.45;
    static uint32_t circle_begin_start_time = 0; // 初始化记录进入圆环开始时间

    if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN)
    {
        if(abs(Lpt0_found) == 2 || abs(Lpt1_found) == 2)
        {
            circle_type = CIRCLE_NONE;
            return;
        }
        // 记录进入圆环开始时间
        if (circle_begin_start_time == 0)
        {
            circle_begin_start_time = fre_cy.time;
        }
        // 二次确认是否是圆环
        if (fre_cy.time - circle_begin_start_time >= 3 || (is_long_straight0 && is_long_straight1)) {
            circle_type = CIRCLE_NONE;
            circle_begin_start_time = 0; // 重新开始计数同时退出圆环模式
            return; // 直接退出圆环模式
        }
    }
    else
    {
        // 重新设置圆环开始时间
        circle_begin_start_time = 0;
    }

    // 此时确认圆环为左圆环
    if (circle_type == CIRCLE_LEFT_BEGIN)
    {
//        Set_Speed_Key(now_using_map,circle_begin);
//        if(none_left_line&&(!have_left_line))
        Extra_search_path = false;
        track_route = TRACK_RIGHT;          //进入左圆环则开始右巡线
        if (Lpt0_found<0 && is_straight1)circle_encoder = Total_mileage;
        if(!none_left_line && is_straight1)circle_Yaw = 0.5f * (circle_Yaw + Yaw_a + Rad2Ang*get_straight_yaw(1));
        //判断左边线是否丢线
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
    //此时处理进入圆环状态
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_route = TRACK_LEFT;       //此时切换为左巡线
        //如果此时已经经过1/4圆环则进一步将状态改为CIRCLE_LEFT_RUNNING
        if ((/*(rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f )  ||*/
//            current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            get_yaw_error(circle_Yaw)>90.0f) && !infinite_circle) { circle_type = CIRCLE_LEFT_RUNNING; }
    }
    //正式进入圆环以后将巡线模式改为右巡线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_route = TRACK_RIGHT;
//        if (Lpt1_found) rpts1s_use = Lpt1_rpts1s_id1;
        //如果此时右巡线发现角点，则认为即将出弯
        if (Lpt1_found) {  //&& Lpt1_rpts1s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //处理即将出弯的状态，此时切换为做巡线，同时检查右边界
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_route = TRACK_LEFT;

        //如果此时右边界为直道，则认为此时已经出弯结束
        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
//            Set_Speed_Key(now_using_map,circle_end);
        }
    }
    //当出弯结束以后，将巡线模式重新改为右巡线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_route = TRACK_RIGHT;

        /*以下部分与上述同理，故不再进行注释翻译*/

        //锟斤拷锟斤拷锟饺讹拷锟斤拷锟斤拷
        if (rpts0s_num < none_line_thres / sample_dist) { none_left_line++; }
        if (rpts0s_num > have_line_thres / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
            Extra_search_path = true;
        }
    }
    //锟揭伙拷锟斤拷锟狡ｏ拷前锟斤拷寻锟斤拷直锟斤拷
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
        //锟饺讹拷锟斤拷锟竭猴拷锟斤拷锟斤拷
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
    //锟斤拷锟揭伙拷锟斤拷寻锟斤拷锟斤拷圆锟斤拷
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_route = TRACK_RIGHT;

        //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�1/4圆   应锟斤拷锟斤拷为锟斤拷锟斤拷为转锟斤拷锟睫拐碉拷
        if ((/*( rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f ) ||*/
            get_yaw_error(circle_Yaw)<(-90.0f)
//            ||current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            ) && !infinite_circle) { circle_type = CIRCLE_RIGHT_RUNNING; }

    }
    //锟斤拷锟斤拷巡锟竭ｏ拷寻锟斤拷圆锟斤拷锟斤拷
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_route = TRACK_LEFT;

        //锟解环锟斤拷锟节拐碉拷,锟斤拷锟劫加拐碉拷锟斤拷锟斤拷芯锟�(锟斤拷L锟斤拷)
//        if (Lpt0_found) rpts0s_use = Lpt0_rpts0s_id1;
        if (Lpt0_found) { //&& Lpt0_rpts0s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //锟斤拷锟斤拷锟斤拷寻锟斤拷圆
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_route = TRACK_RIGHT;

        //锟襟长度硷拷锟斤拷斜锟角讹拷  应锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭碉拷锟斤拷为直锟斤拷
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
        //锟竭癸拷圆锟斤拷锟斤拷寻锟斤拷锟斤拷
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_route = TRACK_LEFT;

        //锟斤拷锟斤拷锟饺讹拷锟斤拷锟斤拷,
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

// 绘制圆环模式下的调试图像
void draw_circle(void)
{
    for(int i=0; i<circle_type; i++)
        draw_o_direct(&img_line, 5+10*i, 5, 3, 255);
}
