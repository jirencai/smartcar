/*
 * garage.h
 *
 *  Created on: 2025年11月5日
 *      Author: ji_rencai
 */

#ifndef CODE_GARAGE_H_
#define CODE_GARAGE_H_

#include "zf_common_headfile.h"

enum garage_type_e {
    GARAGE_NONE = 0,  // 非车库模式
    GARAGE_OUT,       // 出库，陀螺仪转过45°，即出库完毕
    GARAGE_IN,        // 进库，发现车库后判断第几次，从而决定是否进库
    GARAGE_STOP,      // 进库完毕，停车
    GARAGE_END,
    GARAGE_NUM,
};
extern enum garage_type_e garage_type;

void check_garage();
void run_garage(void);


#endif /* CODE_GARAGE_H_ */
