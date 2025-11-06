/*
 * IPM.h
 *
 *  Created on: 2025年11月4日
 *      Author: ji_rencai
 */

#ifndef CODE_IPM_H_
#define CODE_IPM_H_

#include "zf_common_headfile.h"

typedef struct {
    float w, x, y, z;
} Quaternion;
extern Quaternion q_imu;

void IPM_init(float ay, float ax);                      //变换参数的初始化
void IPM(sint16 x, sint16 y, float *result);            //坐标变换
bool Re_IPM(float x, float y, int *result);             //坐标反变换



#endif /* CODE_IPM_H_ */
