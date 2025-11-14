/*
 * circle.h
 *
 *  Created on: 2025��11��5��
 *      Author: ji_rencai
 */

#ifndef CODE_CIRCLE_H_
#define CODE_CIRCLE_H_

#include "zf_common_headfile.h"

enum circle_type_e {
    CIRCLE_NONE = 0,                            // ��Բ��ģʽ
    CIRCLE_LEFT_BEGIN, CIRCLE_RIGHT_BEGIN,      // Բ����ʼ��ʶ�𵽵���L�ǵ���һ�೤ֱ����
    CIRCLE_LEFT_IN, CIRCLE_RIGHT_IN,            // Բ�����룬���ߵ�һ��ֱ����һ��Բ����λ�á�
    CIRCLE_LEFT_RUNNING, CIRCLE_RIGHT_RUNNING,  // Բ���ڲ���
    CIRCLE_LEFT_OUT, CIRCLE_RIGHT_OUT,          // ׼����Բ������ʶ�𵽳�������L�ǵ㡣
    CIRCLE_LEFT_END, CIRCLE_RIGHT_END,          // Բ�����������ٴ��ߵ�����ֱ����λ�á�
    CIRCLE_BREAK, //Բ�����жϣ���·�ϣ�
    CIRCLE_NUM,
};
extern enum circle_type_e circle_type;

extern uint8 Circle_In_Ready;                   //圆环进入准备标志位

void check_circle(void);
void circle_run();
void Circle_In_Ready_Check(void);

void draw_circle(void);




#endif /* CODE_CIRCLE_H_ */
