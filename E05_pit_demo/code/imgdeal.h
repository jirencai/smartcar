/*
 * imgdeal.h
 *
 *  Created on: 2025年11月3日
 *      Author: ji_rencai
 */

#ifndef __IMGDEAL_H__
#define __IMGDEAL_H__

#include "zf_common_headfile.h"

typedef struct image {
    uint8_t (*data)[MT9V03X_W];
    uint32_t width;
    uint32_t height;
    uint32_t step;
} image_t;

extern uint8_t new_image1[MT9V03X_H][MT9V03X_W];            //对cpu1中的new_image1进行声明
extern image_t img_raw,img_OSTU,img_line;                   //对image_t结构体的声明

extern int offset;                                          //补偿参数

#define AT  AT_IMAGE                                                    //AuTop源代码定义的宏
#define AT_IMAGE(img, x, y)          ((img)->data[(y)][(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))

#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=h}          //配合image_t使用
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.height}

void process_image(void);
void find_corner(void);


uint8_t getOSTUThreshold(image_t *img, uint8_t MinThreshold, uint8_t MaxThreshold);
void undistortImage(uint8_t srcImage[120][188], uint8_t dstImage[120][188]);
void findline_lefthand_adaptive(int block_size, int clip_value, int x, int y, sint16 pts[][2], uint16 *num);
void findline_righthand_adaptive(int block_size, int clip_value, int x, int y, sint16 pts[][2], uint16 *num);
void blur_points(float pts_in[][2], uint16 num, float pts_out[][2], sint16 kernel);
void resample_points(float pts_in[][2], uint16 num1, float pts_out[][2], uint16 *num2, float dist);
void local_angle_points(float pts_in[][2], uint16 num, float angle_out[], int dist);
void nms_angle(float angle_in[], uint16 num, float angle_out[], sint16 half);
float get_angle_3points(float* point1, float* point2, float* point3);

float get_straight_yaw(sint8 L_or_R);
float get_yaw_error(float ref);

void find_nearest_dist(float x0, float y0, float points[][2], int serch_range, float* result_dist, float min_dist);


#endif



