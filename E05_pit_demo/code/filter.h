/*
 * filter.h
 *
 *  Created on: 2025Äê10ÔÂ18ÈÕ
 *      Author: ji_rencai
 */

#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "zf_common_headfile.h"

void low_pass_filter_Init(void);
float filter_r(float value);                                                     //ÓÒÂÖÂË²¨
float filter_l(float value);                                                //×óÂÖÂË²¨

#endif /* CODE_FILTER_H_ */
