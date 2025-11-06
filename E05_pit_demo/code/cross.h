/*
 * cross.h
 *
 *  Created on: 2025年11月4日
 *      Author: ji_rencai
 */

#ifndef CODE_CROSS_H_
#define CODE_CROSS_H_

#include "zf_common_headfile.h"

extern bool cross_angle;
extern int cross_width;
extern uint8 quit_value;

enum cross_type_e {
    CROSS_NONE = 0,     // 非十字模式
    CROSS_BEGIN,        // 找到左右两个L角点
    CROSS_IN,           // 两个L角点很近，即进入十字内部(此时切换远线控制)
    CROSS_NUM,
};
extern enum cross_type_e cross_type;

extern uint8 use_cross_aim;


extern int far_x1, far_x2, far_y1, far_y2;
void check_cross(void);
void run_cross(void);
void cross_farline(void);


#endif /* CODE_CROSS_H_ */
