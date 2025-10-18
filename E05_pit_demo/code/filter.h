/*
 * filter.h
 *
 *  Created on: 2025年10月18日
 *      Author: ji_rencai
 */

#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "zf_common_headfile.h"

void low_pass_filter_Init(void);                    //低通滤波器初始化
float filter_r(float value);                        //右轮滤波
float filter_l(float value);                        //左轮滤波

#endif /* CODE_FILTER_H_ */
