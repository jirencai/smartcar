/*
 * extern.c
 *
 *  Created on: 2025年11月3日
 *      Author: ji_rencai
 */

#include "extern.h"

int clip(int x, int low, int up) {return x > up ? up : x < low ? low : x;}
float fclip(float x, float low, float up) {return x > up ? up : x < low ? low : x;}


/*开始运行时间*/
uint16 start_time = 0;
bool lock_yaw = 0;

/*电机控制参数*/
float turn_value;               //电机转角值
float straight_value = 0;           //直道速度
/*屏幕显示参数*/
uint8_t show_flag = 0;


/*图像处理进程*/
uint8 Image_Processing=0;//0:未使用 1:正在处理 2:处理已完成


/*摄像头参数*/
float deltax = 2.0f;
float height = 30.0f;
float cam_angle = 60.5f;
float alpha = 50.7f;

/* (图像处理输出的) 方向误差*/
volatile float error = 0;
volatile float img_error = 0;
volatile float far_error = 0;
volatile float last_error = 0;
volatile float delta_error = 0;

/*图像处理参数*/
//模式调配
bool Extra_search_path = true;          //搜边线时，搜到图像边界后是否继续向上搜索
bool Dynamic_begin_x = true;            //是否使用动态搜索起点

int begin_id = 0;                       //中间数组的索引
int far_begin_id = 0;                   //远线中间数组的索引
float R = 0,K;


float now_aim_distance=0.55;            //0.55预瞄点定长度

//左右边线二值化阈值，用于寻找起点
uint8 thres_L = 140;                    //左边线二值化阈值
uint8 thres_R = 140;                    //右边线二值化阈值
uint8 thres_use = 140;                  //大津法阈值
uint8 thres = 140;                      //二值化阈值，主要用于找起始点(边线使用自适应阈值，不使用该阈值)

//迷宫巡线参数
sint16 block_size = 10;                 // 自适应阈值的block大小10
sint16 clip_value = 13;                 // 自适应阈值的阈值裁减量 15

sint8 OSTU_clip = 0;                    //大津法阈值偏差
sint16 line_blur_kernel = 7;            // 边线三角滤波核的大小

sint16 begin_dx = 0;                    // 起始点距离图像中心的左右偏移量
sint16 begin_y = 85;                    // 起始点距离图像底部的上下偏移量

float pixel_per_meter=107.8;            // 65.6 // 俯视图中每米的像素数（可能需要调参） 115
float sample_dist=0.01;                 // 边线等距采样的间距
float angle_dist=0.12;                  // 计算边线转角时，三个计算点的距离 0.12

/*图像结果参数*/
sint16 aim_idx=0, aim_idx_f=0, aim_idx_near=0, aim_idx1=0;
                                        //远预锚点位置, 近预锚点位置

//原图左右边线
sint16 ipts0[POINTS_MAX_LEN][2];        //左边线坐标参数
sint16 ipts1[POINTS_MAX_LEN][2];        //右边线坐标参数
uint16 ipts0_num, ipts1_num;            //左右边线坐标个数
// 变换后左右边线
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
uint16 rpts0_num, rpts1_num;
// 变换+滤波后左右边线
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
uint16 rpts0b_num, rpts1b_num;
// 变换+滤波+等距采样后左右边线
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
uint16 rpts0s_num, rpts1s_num;
uint16 rpts0s_use, rpts1s_use;
// 左右边线局部角度变化率
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
uint16 rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
uint16 rpts0an_num, rpts1an_num;
// 左/右中线
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
uint16 rptsc0_num, rptsc1_num;
// 中线
float rpts1[POINTS_MAX_LEN][2];
uint16 rpts1_num;
float (*rpts)[2];
uint16 rpts_num;
// 归一化中线
float rptsn[POINTS_MAX_LEN][2];
uint16 rptsn_num;


/*十字图像结果参数*/
// 原图远点左右边线
sint16 far_ipts0[FAR_POINTS_MAX_LEN][2];
sint16 far_ipts1[FAR_POINTS_MAX_LEN][2];
uint16 far_ipts0_num, far_ipts1_num;
// 变换后的左右点边线
float far_rpts0[FAR_POINTS_MAX_LEN][2];
float far_rpts1[FAR_POINTS_MAX_LEN][2];
uint16 far_rpts0_num, far_rpts1_num;
// 变换+滤波后左右远边线
float far_rpts0b[FAR_POINTS_MAX_LEN][2];
float far_rpts1b[FAR_POINTS_MAX_LEN][2];
uint16 far_rpts0b_num, far_rpts1b_num;
//变换后左右远边线+等距采样
float far_rpts0s[FAR_POINTS_MAX_LEN][2];
float far_rpts1s[FAR_POINTS_MAX_LEN][2];
uint16 far_rpts0s_num, far_rpts1s_num;
// 左右远边线局部角度变化率
float far_rpts0a[FAR_POINTS_MAX_LEN];
float far_rpts1a[FAR_POINTS_MAX_LEN];
uint16 far_rpts0a_num, far_rpts1a_num;
// 左右远边线局部角度变化率+非极大抑制
float far_rpts0an[FAR_POINTS_MAX_LEN];
float far_rpts1an[FAR_POINTS_MAX_LEN];
uint16 far_rpts0an_num, far_rpts1an_num;

/*左右角点寻找参数*/
sint8 Lpt0_found, Lpt1_found;           //大于0为第一个角点向上张开（╯或╰）, 小于0为第一个角点向下张开（╮或╭）, 绝对值为个数, 0为没有
uint8 Lpt_open;                         //两角点后是否向外张开(1为张开,0为闭合)
uint8 Lpt_close;
sint16 Lpt0_rpts0s_id1, Lpt1_rpts1s_id1;
sint16 Lpt0_rpts0s_id2, Lpt1_rpts1s_id2;
sint8 down_side0 = 0;                   //大于0为向右张开 小于0为向左张开 0为没有
sint8 down_side1 = 0;

/*左右远角点寻找参数*/
sint8 far_Lpt0_found, far_Lpt1_found;
sint16 far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;

/*直道等判断参数*/
bool is_straight0, is_straight1;
bool is_long_straight0, is_long_straight1;
bool is_short_straight0, is_short_straight1;
bool is_medium_straight0, is_medium_straight1;

float long_straight_thres_use = 1.2f;

float Total_mileage,Total_mileage_use;  //总里程

/*当前巡线模式：左巡线和右巡线*/
enum track_route_e track_route = TRACK_RIGHT;
/*当前跟踪模式：延伸线模式*/
enum track_method_e track_method = EXTEND_LINE;//可防止特殊情况下寻到绕回来的线（如入库，断路）

/*元素部分识别*/
bool cross_enable = 1;                  //是否启用十字元素
bool circle_enable = 1;                 //是否启用圆环元素

uint16 barrier_count = 0;               //障碍物计数
bool barrier_pm = 0;                    //障碍物标志位

uint8 infinite_circle = 0;              //无限圆环
