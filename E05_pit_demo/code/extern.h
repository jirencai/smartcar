/*
 * extern.h
 *
 *  Created on: 2025年11月3日
 *      Author: ji_rencai
 */

#ifndef CODE_EXTERN_H_
#define CODE_EXTERN_H_

#include "zf_common_headfile.h"

extern int clip(int x, int low, int up);            //夹取中间的值（low < x < up）
extern float fclip(float x, float low, float up);   //夹取中间的值（返回float）

extern uint8 Image_Processing;                      //0:未使用 1:正在处理 2:处理已完成

/*图像宏*/
#define IMAGEH MT9V03X_H
#define IMAGEW MT9V03X_W
#define RAW_IMG(x,y) new_image1[y][x]  //mt9v03x_image[y][x]

extern float deltax;
extern float height;
extern float cam_angle;
extern float alpha;


extern bool Extra_search_path;                      //搜边线时，搜到图像边界后是否继续向上搜索
extern bool Dynamic_begin_x;                        //是否使用动态搜索起点

//左右边线二值化阈值，用于寻找起点
extern uint8 thres_L;                               //左边线二值化阈值
extern uint8 thres_R;                               //右边线二值化阈值
extern uint8 thres_use;                             //大津法阈值
extern uint8 thres;                                 //二值化阈值，主要用于找起始点(边线使用自适应阈值，不使用该阈值)

//自适应迷宫法巡线参数
extern sint16 block_size;                           //自适应阈值的block大小10
extern sint16 clip_value;                           //自适应阈值的阈值裁减量 15

extern sint8 OSTU_clip;                             //大津法阈值偏差
extern sint16 line_blur_kernel;                     // 边线三角滤波核的大小

extern sint16 begin_dx;                             // 起始点距离图像中心的左右偏移量
extern sint16 begin_y;                              // 起始点距离图像底部的上下偏移量

extern float pixel_per_meter;                       // 俯视图中像素与实际单位的转换
extern float sample_dist;                           // 边线等距采样的间距
extern float angle_dist;                            // 计算边线转角时，三个计算点的距离

/*屏幕显示参数*/
extern uint8_t show_flag;

/*图像处理中的参数*/
#define POINTS_MAX_LEN     (180)                    //路径点最大数量
#define FAR_POINTS_MAX_LEN (120)                    //远点路径点最大数量

/*图像结果参数*/
extern sint16 aim_idx, aim_idx_f, aim_idx_near, aim_idx1;
                                                    //远预锚点位置, 近预锚点位置

// 原图左右边线
extern sint16 ipts0[POINTS_MAX_LEN][2];
extern sint16 ipts1[POINTS_MAX_LEN][2];
extern uint16 ipts0_num, ipts1_num;
// 变换后左右边线
extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern uint16 rpts0_num, rpts1_num;
// 变换后左右边线+滤波
extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern uint16 rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern uint16 rpts0s_num, rpts1s_num;
extern uint16 rpts0s_use, rpts1s_use;
// 左右边线局部角度变化率
extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern uint16 rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern uint16 rpts0an_num, rpts1an_num;
// 左/右中线
extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern uint16 rptsc0_num, rptsc1_num;
// 归一化中线
extern float rptsn[POINTS_MAX_LEN][2];
extern uint16 rptsn_num;

/*十字图像结果参数*/
//原图远线左右边线
extern sint16 far_ipts0[FAR_POINTS_MAX_LEN][2];
extern sint16 far_ipts1[FAR_POINTS_MAX_LEN][2];
extern uint16 far_ipts0_num, far_ipts1_num;
//变换后左右远边线
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern uint16 far_rpts0_num, far_rpts1_num;
//变换后左右远边线+滤波
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern uint16 far_rpts0b_num, far_rpts1b_num;
//变换后左右远边线+等距采样
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern uint16 far_rpts0s_num, far_rpts1s_num;
// 左右远边线局部角度变化率
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern uint16 far_rpts0a_num, far_rpts1a_num;
// 左右远边线局部角度变化率+非极大抑制
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern uint16 far_rpts0an_num, far_rpts1an_num;

/*左右角点寻找参数*/
extern sint8 Lpt0_found, Lpt1_found;
extern uint8 Lpt_open;
extern uint8 Lpt_close;
extern sint16 Lpt0_rpts0s_id1, Lpt1_rpts1s_id1;
extern sint16 Lpt0_rpts0s_id2, Lpt1_rpts1s_id2;

extern sint8 down_side0;
extern sint8 down_side1;

/*左右远角点寻找参数*/
extern sint8 far_Lpt0_found, far_Lpt1_found;
extern sint16 far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;


/*直道等判断参数*/
extern bool is_straight0, is_straight1;
extern bool is_long_straight0, is_long_straight1;
extern bool is_short_straight0, is_short_straight1;
extern bool is_medium_straight0, is_medium_straight1;

extern float long_straight_thres_use;
#define long_straight_thres 1.5
#define straight_thres 1.0
#define short_straight_thres 0.15
#define medium_straight_thres 0.8


extern float Total_mileage,Total_mileage_use;

// 当前巡线模式
enum track_route_e {
    TRACK_LEFT,
    TRACK_RIGHT,
};
extern enum track_route_e track_route;

extern uint8 infinite_circle;

#endif /* CODE_EXTERN_H_ */
