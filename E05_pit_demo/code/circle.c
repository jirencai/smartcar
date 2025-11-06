/*
 * circle.c
 *
 *  Created on: 2025��11��5��
 *      Author: ji_rencai
 */

#include "circle.h"

/*���������֣���ֹһЩ�ظ�����*/
float circle_encoder;                           //Բ����ת�Ǳ���������
float circle_Yaw;                               //Բ����ת��
float circle_lenout=0.3;                        //�������жϽ�����д���ȶ��


enum circle_type_e circle_type = CIRCLE_NONE;

uint8 circle_cnt0 = 0;                          //�������Բ������
uint8 circle_cnt1 = 0;                          //�ұ�����Բ������

/*�����Բ������Բ��*/
void check_circle(void) {
    // ��Բ��ģʽ�£�����L�ǵ�(�ҽǵ������ſ�), ���߳�ֱ��
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
#define none_line_thres 0.15f                       //С�ڸó����ж�Ϊ����
#define have_line_thres 0.5f                        //���ڸó����ж�Ϊ����
int none_left_line = 0, none_right_line = 0;        //����߶�ʧ���ұ��߶�ʧ
int have_left_line = 0, have_right_line = 0;        //������ҵ����ұ����ҵ�
uint8 Circle_In_Ready=0;                            //����Բ��׼��

/*Բ������ʱ*/
void run_circle() {
//    now_aim_distance = 0.45;
    static uint32_t circle_begin_start_time = 0; // ��¼�����״̬����ʼʱ�䣨��λ���룩

    if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN)
    {
        if(abs(Lpt0_found) == 2 || abs(Lpt1_found) == 2)
        {
            circle_type = CIRCLE_NONE;
            return;
        }
        // ����ս����״̬����¼��ǰʱ��
        if (circle_begin_start_time == 0)
        {
            circle_begin_start_time = fre_cy.time;
        }
        // �����ǰ״̬��������2�룬��ʱ����
        if (fre_cy.time - circle_begin_start_time >= 3 || (is_long_straight0 && is_long_straight1)) {
            circle_type = CIRCLE_NONE;
            circle_begin_start_time = 0; // ���ö�ʱ��
            return; // ֱ�ӷ��أ��������״̬����
        }
    }
    else
    {
        // ����������������״̬ʱ����ն�ʱ������
        circle_begin_start_time = 0;
    }

    // �󻷿�ʼ��Ѱ��ֱ������
    if (circle_type == CIRCLE_LEFT_BEGIN)
    {
//        Set_Speed_Key(now_using_map,circle_begin);
//        if(none_left_line&&(!have_left_line))
        Extra_search_path = false;
        track_route = TRACK_RIGHT;
        if (Lpt0_found<0 && is_straight1)circle_encoder = Total_mileage;
        if(!none_left_line && is_straight1)circle_Yaw = 0.5f * (circle_Yaw + Yaw_a + Rad2Ang*get_straight_yaw(1));
        //�ȶ����ߺ�����
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
    //�뻷��Ѱ��Բ����
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_route = TRACK_LEFT;
        //�����������1/4Բ   Ӧ����Ϊ����Ϊת���޹յ�
        if ((/*(rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f )  ||*/
//            current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            get_yaw_error(circle_Yaw)>90.0f) && !infinite_circle) { circle_type = CIRCLE_LEFT_RUNNING; }
    }
    //����Ѳ�ߣ�Ѱ��Բ����
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_route = TRACK_RIGHT;
//        if (Lpt1_found) rpts1s_use = Lpt1_rpts1s_id1;//�⻷�յ�(��L��)
        if (Lpt1_found) {  //&& Lpt1_rpts1s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //������Ѱ��Բ
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_route = TRACK_LEFT;

        //����Ϊ��ֱ��
        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
//            Set_Speed_Key(now_using_map,circle_end);
        }
    }
    //�߹�Բ����Ѱ����
    else if (circle_type == CIRCLE_LEFT_END) {
        track_route = TRACK_RIGHT;

        //�����ȶ�����
        if (rpts0s_num < none_line_thres / sample_dist) { none_left_line++; }
        if (rpts0s_num > have_line_thres / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
            Extra_search_path = true;
        }
    }
    //�һ����ƣ�ǰ��Ѱ��ֱ��
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
        //�ȶ����ߺ�����
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
    //���һ���Ѱ����Բ��
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_route = TRACK_RIGHT;

        //�����������1/4Բ   Ӧ����Ϊ����Ϊת���޹յ�
        if ((/*( rpts1s_num < 0.1 / sample_dist && current_encoder - circle_encoder >= ENCODER_PER_METER * 0.3f ) ||*/
            get_yaw_error(circle_Yaw)<(-90.0f)
//            ||current_encoder - circle_encoder >= ENCODER_PER_METER_f * 3.14f * 0.5f
            ) && !infinite_circle) { circle_type = CIRCLE_RIGHT_RUNNING; }

    }
    //����Ѳ�ߣ�Ѱ��Բ����
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_route = TRACK_LEFT;

        //�⻷���ڹյ�,���ټӹյ�����о�(��L��)
//        if (Lpt0_found) rpts0s_use = Lpt0_rpts0s_id1;
        if (Lpt0_found) { //&& Lpt0_rpts0s_id1 < (now_aim_distance) / sample_dist
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //������Ѱ��Բ
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_route = TRACK_RIGHT;

        //�󳤶ȼ���б�Ƕ�  Ӧ�����������ҵ���Ϊֱ��
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
        //�߹�Բ����Ѱ����
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_route = TRACK_LEFT;

        //�����ȶ�����,
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
