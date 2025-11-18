/*
 * imgdeal.c
 *
 *  Created on: 2025年11月3日
 *      Author: ji_rencai
 */

#include "imgdeal.h"
#include "mapX.h"
#include "mapY.h"

#define EPS_Y 0.5f                              //


image_t img_raw = DEF_IMAGE(new_image1, IMAGEW, IMAGEH);  //(mt9v03x_image, IMAGEW, IMAGEH)
image_t img_OSTU = DEF_IMAGE(&new_image1[59], IMAGEW, IMAGEH - 60);

uint8_t img_line_data[IMAGEH][IMAGEW];
image_t img_line = DEF_IMAGE(img_line_data, IMAGEW, IMAGEH);

float Img_Zoom=1, Img_Move=0;                   //图像平移和缩放参数

volatile int thres_use0, thres_use1;            //左边线阈值和右边线阈值
int offset = 0;                                 //补偿参数

#define middle 184                              //针对path_max的路径的中间索引
#define path_max 183+200-16                     //针对path_max的路径的最大索引
uint8 search_path[path_max][2]={
    /* 竖直方向 */
    {2,18},{2,19},{2,20},{2,21},{2,22},{2,23},{2,24},{2,25},{2,26},{2,27},{2,28},{2,29},{2,30},{2,31},{2,32},{2,33},{2,34},{2,35},{2,36},{2,37},{2,38},{2,39},{2,40},{2,41},
    {2,42},{2,43},{2,44},{2,45},{2,46},{2,47},{2,48},{2,49},{2,50},{2,51},{2,52},{2,53},{2,54},{2,55},{2,56},{2,57},{2,58},{2,59},{2,60},{2,61},{2,62},{2,63},{2,64},{2,65},
    {2,66},{2,67},{2,68},{2,69},{2,70},{2,71},{2,72},{2,73},{2,74},{2,75},{2,76},{2,77},{2,78},{2,79},{2,80},{2,81},{2,82},{2,83},{2,84},{2,85},{2,86},{2,87},{2,88},{2,89},
    {2,90},{2,91},{2,92},{2,93},{2,94},{2,95},{2,96},{2,97},{2,98},{2,99},{2,100},{2,101},{2,102},{2,103},{2,104},{2,105},{2,106},{2,107},{2,108},
//    {2,109},{2,110},{2,111},{2,112},{2,113},{2,114},{2,115},{2,116},{2,117},
    /*水平方向*/
    {2,109},{3,110},{4,111},{5,112},{6,113},{7,114},{8,115},{9,116},{10,117},
    {11,117},{12,117},{13,117},{14,117},{15,117},{16,117},{17,117},{18,117},{19,117},{20,117},{21,117},{22,117},{23,117},{24,117},{25,117},{26,117},{27,117},{28,117},{29,117},
    {30,117},{31,117},{32,117},{33,117},{34,117},{35,117},{36,117},{37,117},{38,117},{39,117},{40,117},{41,117},{42,117},{43,117},{44,117},{45,117},{46,117},{47,117},{48,117},
    {49,117},{50,117},{51,117},{52,117},{53,117},{54,117},{55,117},{56,117},{57,117},{58,117},{59,117},{60,117},{61,117},{62,117},{63,117},{64,117},{65,117},{66,117},{67,117},
    {68,117},{69,117},{70,117},{71,117},{72,117},{73,117},{74,117},{75,117},{76,117},{77,117},{78,117},{79,117},{80,117},{81,117},{82,117},{83,117},{84,117},{85,117},{86,117},
    {87,117},{88,117},{89,117},{90,117},{91,117},{92,117},{93,117},{94,117},{95,117},{96,117}, /* 93为对称轴,为数组第100+93-9=184个元素(未减1) */
    {97,117},{98,117},{99,117},{100,117},{101,117},{102,117},{103,117},{104,117},{105,117},{106,117},{107,117},{108,117},{109,117},{110,117},{111,117},
    {112,117},{113,117},{114,117},{115,117},{116,117},{117,117},{118,117},{119,117},{120,117},{121,117},{122,117},{123,117},{124,117},{125,117},{126,117},{127,117},{128,117},
    {129,117},{130,117},{131,117},{132,117},{133,117},{134,117},{135,117},{136,117},{137,117},{138,117},{139,117},{140,117},{141,117},{142,117},{143,117},{144,117},{145,117},
    {146,117},{147,117},{148,117},{149,117},{150,117},{151,117},{152,117},{153,117},{154,117},{155,117},{156,117},{157,117},{158,117},{159,117},{160,117},{161,117},{162,117},
    {163,117},{164,117},{165,117},{166,117},{167,117},{168,117},{169,117},{170,117},{171,117},{172,117},{173,117},{174,117},{175,117},{176,117},{177,117},
    {178,117},{179,116},{180,115},{181,114},{182,113},{183,112},{184,111},
    /*水平方向结束*/
//    {184,117},{184,116},{184,115},{184,114},{184,113},{184,112},{184,111},
    {184,110},{184,109},{184,108},{184,107},{184,106},{184,105},{184,104},{184,103},{184,102},{184,101},{184,100},{184,99},{184,98},{184,97},{184,96},{184,95},{184,94},
    {184,93},{184,92},{184,91},{184,90},{184,89},{184,88},{184,87},{184,86},{184,85},{184,84},{184,83},{184,82},{184,81},{184,80},{184,79},{184,78},{184,77},{184,76},{184,75},
    {184,74},{184,73},{185,72},{185,71},{185,70},{185,69},{185,68},{185,67},{185,66},{185,65},{185,64},{185,63},{185,62},{185,61},{185,60},{185,59},{185,58},{185,57},{185,56},
    {185,55},{185,54},{185,53},{185,52},{185,51},{185,50},{185,49},{185,48},{185,47},{185,46},{185,45},{185,44},{185,43},{185,42},{185,41},{185,40},{185,39},{185,38},{185,37},
    {185,36},{185,35},{185,34},{185,33},{185,32},{185,31},{185,30},{185,29},{185,28},{185,27},{185,26},{185,25},{185,24},{185,23},{185,22},{185,21},{185,20},{185,19},{185,18}
};//起点搜索路径

int begin_x_idx = middle;               //记忆的中线位置，作为下次搜索的起始点（为什么要-5？）

unsigned int                            //此处暂时不知道是何意味
    thres_sum, \
    thres_sum_cnt, \
    thres_sum_L, \
    thres_sum_cnt_L, \
    thres_sum_R, \
    thres_sum_cnt_R;

uint16 begin_L, begin_R;                //左右边线的起始搜索点
sint16 RAW_IMG_Offset = -3;               //边线光补偿

void process_image(void)
{
    //大津法阈值计算（这里使用大津法会不会对算力进行浪费?）
    thres_use = getOSTUThreshold(&img_OSTU, 10, 254);

    thres_sum_L=0; thres_sum_cnt_L=0; thres_sum_R=0; thres_sum_cnt_R=0;
    uint16 path_idx_min = 40, path_idx_max = path_max - 40;  //搜索范围为搜索路径第40个点到倒数第40个点

    if(Dynamic_begin_x)  //是否用动态起始点
    {
        begin_L = begin_x_idx  - begin_dx;
        begin_R = begin_x_idx  + begin_dx;
    }//用begin_x_idx  - begin_dx动态计算起始索引点（没发现对begin_dx的赋值？）
    else
    {
        begin_L = middle;
        begin_R = middle;
    }//没发现对RAW_IMG_Offset的赋值

    if(!Extra_search_path)          //如果不使用额外搜索路径
    {
        path_idx_min += 45;
        path_idx_max -= 45;
    }//左右搜索路径检索减少

    /*寻找左右边线起点*/
    ipts0_num = POINTS_MAX_LEN;     //路径点数量初始设置为最大值（为什么为180？）
    for (; begin_L > path_idx_min; begin_L--)
    {
        // 从 begin_L 向左搜索，直到像素值小于左边线阈值 thres_L 或达到 path_idx_min，确定左边线起点
        if (RAW_IMG(search_path[begin_L-1][0],search_path[begin_L-1][1]) < thres_L) break;
    }
    ipts1_num = POINTS_MAX_LEN;
    for (; begin_R < path_idx_max; begin_R++)
    {
        // 从 begin_R 向右搜索，直到像素值小于右边线阈值 thres_R 或达到 path_idx_max，确定右边线起点
        if (RAW_IMG(search_path[begin_R+1][0],search_path[begin_R+1][1]) < thres_R) break;
    }

    /*如果上面预设的阈值没有找到边线起点*/
    if(begin_L==path_idx_min || begin_R==path_idx_max)      //搜索到达边界（没找到起始点）
    {
        thres=thres_L=thres_R=getOSTUThreshold(&img_OSTU, 10, 254);  //使用OSTU算法计算自适应阈值，范围10到254
        //是否启用动态起点模式
        if(Dynamic_begin_x)
        {
            begin_L = begin_x_idx  - begin_dx;
            begin_R = begin_x_idx  + begin_dx;
        }
        else
        {
            begin_L = middle;
            begin_R = middle;
        }

        //寻找左右边线起点，与上文同理
        for (; begin_L > path_idx_min; begin_L--)
        {
            if (RAW_IMG(search_path[begin_L-1][0],search_path[begin_L-1][1]) < thres_L) break;
        }
        for (; begin_R < path_idx_max; begin_R++)
        {
            if (RAW_IMG(search_path[begin_R+1][0],search_path[begin_R+1][1]) < thres_R) break;
        }

        //找到起点以后使用左手迷宫法寻找左边线
        if (RAW_IMG(search_path[begin_L][0],search_path[begin_L][1]) >= thres_L)
        {
            findline_lefthand_adaptive(block_size, clip_value, search_path[begin_L][0],search_path[begin_L][1], ipts0, &ipts0_num);
        }
        else
        {
            ipts0_num = 0;
        }
        //找到起点以后使用右手迷宫法寻找右边线
        if (RAW_IMG(search_path[begin_R][0],search_path[begin_R][1]) >= thres_R)
        {
            findline_righthand_adaptive(block_size, clip_value, search_path[begin_R][0],search_path[begin_R][1], ipts1, &ipts1_num);
        }
        else
        {
            ipts1_num = 0;
        }
    }
    /*如果使用上面的阈值直接找到了左右边线的起点*/
    else
    {
        //直接使用左手迷宫法寻找左边线
        if (RAW_IMG(search_path[begin_L][0],search_path[begin_L][1]) >= thres_L)
        {
            findline_lefthand_adaptive(block_size, clip_value, search_path[begin_L][0],search_path[begin_L][1], ipts0, &ipts0_num);
        }
        else
        {
            ipts0_num = 0;
        }
        //直接使用右手迷宫法寻找右边线
        if (RAW_IMG(search_path[begin_R][0],search_path[begin_R][1]) >= thres_R)
        {
            findline_righthand_adaptive(block_size, clip_value, search_path[begin_R][0],search_path[begin_R][1], ipts1, &ipts1_num);
        }
        else
        {
            ipts1_num = 0;
        }
        // 使用搜边线时得到的大量局部自适应二值化阈值更新阈值
        float rate_L = 0.1f*(float)thres_sum_cnt_L/40.0f;  //左边线y>80部分的相关变量
        if(ipts0_num>80)  //左边线点数大于80
            thres_L = (uint8)clip((int)((1.0f-rate_L)*thres_L + rate_L*(thres_sum_L/thres_sum_cnt_L)) , 80, 200);  //y>80部分来计算的平均阈值

        float rate_R = 0.1f*(float)thres_sum_cnt_R/40.0f;  //右边线y>80部分的相关变量
        if(ipts1_num>80)
            thres_R = (uint8)clip((int)((1.0f-rate_R)*thres_R + rate_R*(thres_sum_R/thres_sum_cnt_R)) , 80, 200);

        thres = (thres_L + thres_R) / 2;  //用左右两边的平均阈值来计算全局阈值
    }

    /*记忆中线位置，作为下次找种子的起始点*/
    if(ipts0_num && ipts1_num) begin_x_idx = (int)(0.5f*(float)begin_L + 0.5f*(float)begin_R);  //如果左右边线都找到，取平均值作为下次搜索的起始点
    else if (ipts0_num) begin_x_idx = begin_L+15;  //如果只找到左边线，向右偏移10
    else if (ipts1_num) begin_x_idx = begin_R-15;  //如果只找到右边线，向左偏移10
    else begin_x_idx = (int)(0.04f*(float)middle+0.96f*(float)begin_x_idx);  //如果都没找到，使用middle和当前begin_x_idx加权平均
    begin_x_idx = clip(begin_x_idx, 100, 100+169);  //限制begin_x_idx

    IPM_init(Pitch_a, -Roll_a);  //初始化透视变换，使用俯仰角Pitch_a和横滚角Roll_a

    Image_Processing = 1;           /*处理状态的变换*/
    /*左右边线坐标变换*/
    for (int i = 0; i < ipts0_num; i++)
    {
        IPM(ipts0[i][0] - RAW_IMG_Offset, ipts0[i][1], rpts0[i]);  //对边线光轴补偿后逆透视变换
//        IPM_new(ipts0[i][0]-RAW_IMG_Offset, ipts0[i][1], rpts0[i]);
    }
    rpts0_num = ipts0_num;  //设置变换后的点数

    for (int i = 0; i < ipts1_num; i++) {
        IPM(ipts1[i][0]-RAW_IMG_Offset, ipts1[i][1], rpts1[i]);
//        IPM_new(ipts1[i][0]-RAW_IMG_Offset, ipts1[i][1], rpts1[i]);
    }
    rpts1_num = ipts1_num;

    // 边线三角滤波
    blur_points(rpts0, rpts0_num, rpts0b, line_blur_kernel);
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, line_blur_kernel);
    rpts1b_num = rpts1_num;

    // 边线等距采样边线点
    rpts0s_num = POINTS_MAX_LEN;
    resample_points(rpts0, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts0s_use = rpts0s_num;
    rpts1s_num = POINTS_MAX_LEN;
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);
    rpts1s_use = rpts1s_num;

    // 利用等距采样的边线点计算局部角度变化率
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int) roundf(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num; //设置用于计算角度变化率的点数
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int) roundf(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;

    // 角度变化率非极大抑制（用于识别拐点，防止正常边线误判）
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int) roundf(angle_dist / sample_dist));
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int) roundf(angle_dist / sample_dist));
    rpts1an_num = rpts1a_num;
}

sint16 straight_idx0=0,straight_idx1=0;
float ramp_angle0 = 0.0f;
float ramp_angle1 = 0.0f;
float dn_test;

/*寻找图像中的角点*/
void find_corner(void)
{
//    uint8 con_error=0;                                      //角点记录错误次数（哪里用到了？）
    sint16 dist = (sint16) roundf(angle_dist / sample_dist);      //angle_dist
    //注意强制类型转换
    Lpt0_found = Lpt1_found = 0;                            //角点个数初始化

    Lpt_close = 0;
//    barrier_angle0 = 0;
//    barrier_angle1 = 0;
    is_long_straight0 = rpts0s_num > long_straight_thres_use / sample_dist;
    is_long_straight1 = rpts1s_num > long_straight_thres_use / sample_dist;
    is_straight0 = rpts0s_num > straight_thres / sample_dist;
    is_straight1 = rpts1s_num > straight_thres / sample_dist;
    is_short_straight0 = rpts0s_num > short_straight_thres / sample_dist;
    is_short_straight1 = rpts1s_num > short_straight_thres / sample_dist;
    is_medium_straight0 = rpts0s_num > medium_straight_thres / sample_dist;
    is_medium_straight1 = rpts1s_num > medium_straight_thres / sample_dist;

    straight_idx0 = straight_idx1 = 0;

    for (sint16 i = dist; i < rpts0s_num-dist; i++)
    {
        if (rpts0an[i] == 0) continue;
        int im1 = clip(i - dist, 0, rpts0s_num - 1);
        int ip1 = clip(i + dist, 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i])- (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
//        //Y角点阈值
//        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist) {
//            Ypt0_rpts0s_id = i;
//            Ypt0_found = true;
//        }
        //L角点阈值
        if (abs(Lpt0_found)<2 && 60.f / 180.f * PI < conf && conf < 120.f / 180.f * PI && (i < 1.5f/sample_dist||(Lpt0_found && i < 1.7f/sample_dist)))
        {
            if((rpts0s[clip(i+dist,0,rpts0s_num-1)][1] + rpts0s[clip(i-dist,0,rpts0s_num-1)][1])*0.5f > rpts0s[i][1])
            {
                if(rpts0s[clip(i+25,0,rpts0s_num-1)][0] < rpts0s[i][0]) down_side0 = -1;
                else down_side0 = 1;

                if(Lpt0_found==0)
                {
                    Lpt0_found = -1;//向下张开（╮或╭）
                    Lpt0_rpts0s_id1 = i;
                }
                else
                {
                    Lpt0_found *= 2;
                    Lpt0_rpts0s_id2 = i;
                }
            }
            else
            {
                if(Lpt0_found==0)
                {
                    Lpt0_found =  1;//向上张开（╯或╰）
                    Lpt0_rpts0s_id1 = i;
                }
                else
                {
                    Lpt0_found *= 2;
                    Lpt0_rpts0s_id2 = i;
                }
            }
        }
        //长直道阈值
//        if (conf > 55.0f / 180. * PI) barrier_angle0++;
//        else barrier_angle0 = 0;
        if (conf > 9.0f / 180. * PI) {
            if(i < long_straight_thres / sample_dist){
                is_long_straight0 = false;
                if(i < straight_thres / sample_dist){
                    is_straight0 = false;
                    if(i < medium_straight_thres / sample_dist){
                        is_medium_straight0 = false;
                        if(i < short_straight_thres / sample_dist){
                            is_short_straight0 = false;
                        }
                    }
                }
            }
        }else if(i < straight_thres / sample_dist && is_straight0){
            straight_idx0=i;
        }
        if (/*Ypt0_found == true &&*/ abs(Lpt0_found)==2 && is_straight0 == false) break;
    }

    for (sint16 i = dist; i < rpts1s_num-dist; i++)
    {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - dist, 0, rpts1s_num - 1);
        int ip1 = clip(i + dist, 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i])- (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
//        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist) {
//            Ypt1_rpts1s_id = i;
//            Ypt1_found = true;
//        }
        if (abs(Lpt1_found)<2 && 60.f / 180.f * PI < conf && conf < 120.f / 180.f * PI && (i < 1.5f/sample_dist||(Lpt1_found && i < 1.7f/sample_dist))) {  //60 120
            if((rpts1s[clip(i+dist,0,rpts1s_num-1)][1] + rpts1s[clip(i-dist,0,rpts1s_num-1)][1])*0.5f > rpts1s[i][1]){
                if(rpts1s[clip(i+25,0,rpts1s_num-1)][0] < rpts1s[i][0]) down_side1 = -1;
                else down_side1 = 1;
                if(Lpt1_found==0){
                    Lpt1_found = -1;//向下张开（╮或╭）
                    Lpt1_rpts1s_id1 = i;
                }else{
                    Lpt1_found *= 2;
                    Lpt1_rpts1s_id2 = i;
                }
            }else{
                if(Lpt1_found==0){
                    Lpt1_found =  1;//向上张开（╯或╰）
                    Lpt1_rpts1s_id1 = i;
                }else{
                    Lpt1_found *= 2;
                    Lpt1_rpts1s_id2 = i;
                }
            }
        }
//        if (conf > 55.0f / 180. * PI) barrier_angle1++;
//        else barrier_angle1 = 0;
        if (conf > 9.0f / 180. * PI) {
            if(i < long_straight_thres / sample_dist){
                is_long_straight1 = false;
                if(i < straight_thres / sample_dist){
                    is_straight1 = false;
                    if(i < medium_straight_thres / sample_dist){
                        is_medium_straight1 = false;
                        if(i < short_straight_thres / sample_dist){
                            is_short_straight1 = false;
                        }
                    }
                }
            }
        }else if(i < straight_thres / sample_dist && is_straight1){
            straight_idx1=i;
        }
        if (/*Ypt1_found == true &&*/ abs(Lpt1_found)==2 && is_straight1 == false) break;
    }

    if(rpts0s_use && rpts1s_use)
    { //边线弯曲程度
        ramp_angle0 = get_angle_3points(rpts0s[0],
                rpts0s[(uint16)(fclip(0.6f/sample_dist,0,rpts0s_use-1))],
                rpts0s[(uint16)(fclip(1.2f/sample_dist,0,rpts0s_use-1))]);  //左负右正
        ramp_angle1 = get_angle_3points(rpts1s[0],
                rpts1s[(uint16)(fclip(0.6f/sample_dist,0,rpts1s_use-1))],
                rpts1s[(uint16)(fclip(1.2f/sample_dist,0,rpts1s_use-1))]);
    }

    //直道二次检查
    if(is_short_straight0)
    {
        float straight_ang=get_angle_3points(rpts0s[0],
                rpts0s[(uint16)(0.5f*short_straight_thres / sample_dist)],
                rpts0s[(uint16)(short_straight_thres / sample_dist)]);
        if(straight_ang>3.0f || straight_ang<-3.0f)is_short_straight0=false;
    }
    if(is_straight0)
    {
        float straight_ang=get_angle_3points(rpts0s[0],
                rpts0s[(uint16)(0.5f*straight_thres / sample_dist)],
                rpts0s[(uint16)(straight_thres / sample_dist)]);
        if(straight_ang>7.0f || straight_ang<-7.0f)is_straight0=false;
    }
    if(is_long_straight0)
    {
        float straight_ang=get_angle_3points(rpts0s[0],
                rpts0s[(uint16)(0.5f*long_straight_thres / sample_dist)],
                rpts0s[(uint16)(long_straight_thres / sample_dist)]);
        if(straight_ang>5.0f || straight_ang<-5.0f)is_long_straight0=false;
    }
    if(is_medium_straight0)
    {
        float straight_ang=get_angle_3points(rpts0s[0],
                rpts0s[(uint16)(0.5f*medium_straight_thres / sample_dist)],
                rpts0s[(uint16)(medium_straight_thres / sample_dist)]);
        if(straight_ang>5.0f || straight_ang<-5.0f)is_medium_straight0=false;
    }
    if(is_short_straight1)
    {
        float straight_ang=get_angle_3points(rpts1s[0],
                rpts1s[(uint16)(0.5f*short_straight_thres / sample_dist)],
                rpts1s[(uint16)(short_straight_thres / sample_dist)]);
        if(straight_ang>3.0f || straight_ang<-3.0f)is_short_straight1=false;
    }
    if(is_straight1)
    {
        float straight_ang=get_angle_3points(rpts1s[0],
                rpts1s[(uint16)(0.5f*straight_thres / sample_dist)],
                rpts1s[(uint16)(straight_thres / sample_dist)]);
        if(straight_ang>7.0f || straight_ang<-7.0f)is_straight1=false;
    }
    if(is_long_straight1)
    {
        float straight_ang=get_angle_3points(rpts1s[0],
                rpts1s[(uint16)(0.5f*long_straight_thres / sample_dist)],
                rpts1s[(uint16)(long_straight_thres / sample_dist)]);
        if(straight_ang>5.0f || straight_ang<-5.0f)is_long_straight1=false;
    }
    if(is_medium_straight1)
    {
        float straight_ang=get_angle_3points(rpts1s[0],
                rpts1s[(uint16)(0.5f*medium_straight_thres / sample_dist)],
                rpts1s[(uint16)(medium_straight_thres / sample_dist)]);
        if(straight_ang>5.0f || straight_ang<-5.0f)is_medium_straight1=false;
    }

    // 双L点二次检查, 依据L角点距离及角点后张开特性
//    disp(13,Lpt0_found,"Lpt0_found");
//    disp(14,Lpt1_found,"Lpt1_found");
    if (Lpt0_found && Lpt1_found)
    {
        float dx = rpts0s[Lpt0_rpts0s_id1][0] - rpts1s[Lpt1_rpts1s_id1][0];
        float dy = rpts0s[Lpt0_rpts0s_id1][1] - rpts1s[Lpt1_rpts1s_id1][1];
        float dn = sqrtf(dx * dx + dy * dy);
        dn_test = dn;
        if (fabs(dn - 0.45 * pixel_per_meter) < 0.075f * pixel_per_meter)
        {  //0.11 0.08
            cross_angle = 1;
//            bridge_angle = 0;
            float dwx = rpts0s[clip(Lpt0_rpts0s_id1+30, 0, rpts0s_num - 1)][0] -rpts1s[clip(Lpt1_rpts1s_id1+30, 0, rpts1s_num - 1)][0];
            float dwy = rpts0s[clip(Lpt0_rpts0s_id1+30, 0, rpts0s_num - 1)][1] -rpts1s[clip(Lpt1_rpts1s_id1+30, 0, rpts1s_num - 1)][1];
            float dwn = sqrtf(dwx * dwx + dwy * dwy);
            if (    dwn > 0.7 * pixel_per_meter
                &&
                    rpts0s[clip(Lpt0_rpts0s_id1 + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_rpts0s_id1][0]
                &&
                    rpts1s[clip(Lpt1_rpts1s_id1 + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_rpts1s_id1][0]
               )Lpt_open=1;
            if (    (rpts0s[clip(Lpt0_rpts0s_id1 + 40, 0, rpts0s_num - 1)][0] - rpts0s[Lpt0_rpts0s_id1][0]) > 20
                &&
                    (rpts1s[clip(Lpt1_rpts1s_id1 + 40, 0, rpts1s_num - 1)][0] - rpts1s[Lpt1_rpts1s_id1][0]) < -20
               )Lpt_close=1;
//            else if(abs(Lpt0_found)==2&&abs(Lpt1_found)==2){
//                Lpt_open=0;
//                float dxl = rpts0s[Lpt0_rpts0s_id1][0] - rpts1s[Lpt1_rpts1s_id2][0];
//                float dyl = rpts0s[Lpt0_rpts0s_id1][1] - rpts1s[Lpt1_rpts1s_id2][1];
//                float dnl = sqrtf(dxl * dxl + dyl * dyl);
//                float dxr = rpts0s[Lpt0_rpts0s_id2][0] - rpts1s[Lpt1_rpts1s_id1][0];
//                float dyr = rpts0s[Lpt0_rpts0s_id2][1] - rpts1s[Lpt1_rpts1s_id1][1];
//                float dnr = sqrtf(dxr * dxr + dyr * dyr);
//                if(!(dnl < 0.1 * pixel_per_meter && dnr < 0.1 * pixel_per_meter)){
//                    Lpt0_found = Lpt1_found = 0;
//                    con_error=3;
//                }
//            }else{
//                Lpt0_found = Lpt1_found = 0;
//                con_error=2;
//            }
        }
        else
        {
            cross_angle = 0;
//            bridge_angle = 1;
//            Lpt0_found = Lpt1_found = 0;
//            con_error=1;
        }
    }

}
// 左边线跟踪中线
void track_leftline(float pts_in[][2], uint16 num, float pts_out[][2], uint16* len, uint16 approx_num, float dist) {
    *len = 0;
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrtf(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[*len][0] = pts_in[i][0] - dy * dist;
        pts_out[*len][1] = pts_in[i][1] + dx * dist;
        if(pts_out[*len][1]<120)(*len)++;
//        (*len)++;
    }
}
void track_leftline_c(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist){
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}
// 右边线跟踪中线
void track_rightline(float pts_in[][2], uint16 num, float pts_out[][2], uint16* len, uint16 approx_num, float dist) {
    *len = 0;
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrtf(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[*len][0] = pts_in[i][0] + dy * dist;
        pts_out[*len][1] = pts_in[i][1] - dx * dist;
        if(pts_out[*len][1]<120)(*len)++;
//        (*len)++;
    }
}
void track_rightline_c(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrtf(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}
void Track_CenterLine(void){
    // 左右中线跟踪
    if(Lpt0_found<0){
        rpts0s_use=Lpt0_rpts0s_id1;
        track_leftline (rpts0s, rpts0s_use, rptsc0, &rptsc0_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
    }
//    else if(Lpt0_found>0){
//        rpts0s_use-=Lpt0_rpts0s_id1;
//        track_leftline (rpts0s+Lpt0_rpts0s_id1, rpts0s_use, rptsc0, &rptsc0_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
//    }
    else track_leftline (rpts0s, rpts0s_use, rptsc0, &rptsc0_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
    if(Lpt1_found<0){
        rpts1s_use=Lpt1_rpts1s_id1;
        track_rightline (rpts1s, rpts1s_use, rptsc1, &rptsc1_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
    }
//    else if(Lpt1_found>0){
//        rpts1s_use-=Lpt1_rpts1s_id1;
//        track_rightline(rpts1s+Lpt1_rpts1s_id1, rpts1s_use, rptsc1, &rptsc1_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
//    }
    else{
        track_rightline (rpts1s, rpts1s_use, rptsc1, &rptsc1_num, 5, pixel_per_meter * ROAD_WIDTH / 2);
    }
//    for(uint16 i = 0; i < 0.6f*MIN(rpts0s_use,rpts1s_use); i++){
//        if(barrier_count1 > 15) {
//            barrier_pm1 = 1;
//            break;
//        }
//        else if(barrier_count0 > 15){
//            barrier_pm0 = 1;
//            break;
//        }
//        else {
//            barrier_pm1 = 0;
//            barrier_pm0 = 0;
//        }
//        if(rptsc0[i][0] - rptsc1[i][0] > 0.1f * pixel_per_meter) barrier_count0++;
//        if(rptsc1[i][0] - rptsc0[i][0] > 0.1f * pixel_per_meter) barrier_count1++;
//    }
    barrier_count = 0;

    for (uint8 y = 110; y >= 10; --y) {
        float x0 = 0.0f, x1 = 0.0f;
        uint8 found0 = 0, found1 = 0;
        uint8 i0 = 0, i1 = 0;
        while (i0 < rptsc0_num-1 && rptsc0[i0][1] > y + EPS_Y)
            ++i0;
        if (i0 < rptsc0_num && fabsf(rptsc0[i0][1] - (float)y) < EPS_Y) {
            x0     = rptsc0[i0][0];
            found0 = 1;
        }

        /* ---- 在 edge1 中找 y 行 ---- */
        while (i1 < rptsc1_num-1 && rptsc1[i1][1] > y + EPS_Y)
            ++i1;
        if (i1 < rptsc1_num && fabsf(rptsc1[i1][1] - (float)y) < EPS_Y) {
            x1     = rptsc1[i1][0];
            found1 = 1;
        }

        /* 两边都找到才记录，否则跳过该行 */
        if (found0 && found1) {
            if(x0 - x1 > 0.065f * pixel_per_meter) barrier_count++;
        }
        if(barrier_count > 5) barrier_pm = 1;
        else barrier_pm = 0;
        //555
    }
}
// 圆环等距跟踪
void round_scan(sint16* aim_idx, float cx, float cy, float aim_dist){
    // 找距离追踪点的距离最接近目标值的点
    sint16 idx = *aim_idx;
    float dx = rptsn[idx][0] - cx;
    float dy = rptsn[idx][1] - cy;
    float dist, min_dist = fabsf(sqrtf(dx * dx + dy * dy)-aim_dist);
    for (sint16 i = 0; i < rptsn_num; i++) {
        dx = rptsn[i][0] - cx;
        dy = rptsn[i][1] - cy;
        dist = fabsf(sqrtf(dx * dx + dy * dy)-aim_dist);
        if (dist < min_dist && dist > 20.0f && rptsn[i][1]>0 && rptsn[i][1]<120) {
            min_dist = dist;
            idx = i;
        }
    }
    *aim_idx = idx;
}
//等线距跟踪
void extend_line(float(*data)[2], float cx, float cy, int N){
    // 计算数据点的平均值
    float x_avg = 0, y_avg = 0, delta_x;
    for (int i = 0; i < N; i++) {
        x_avg += data[i][0];
        y_avg += data[i][1];
    }
    x_avg /= (float)N;
    y_avg /= (float)N;

    // 计算最小二乘拟合的斜率和截距
    float numerator = 0, denominator = 0;
    for (int i = 0; i < N; i++) {
        delta_x = data[i][0] - x_avg;
        numerator += delta_x * (data[i][1] - y_avg);
        denominator += delta_x * delta_x;
    }
    float k = numerator / denominator;
    float b = y_avg - k * x_avg;

//    // 输出最小二乘拟合的方程
//    printf("y = %.2f * x + %.2f\n", k, b);

    // 计算待求点到拟合直线的垂足坐标
    float y0 = ((k*cy+cx)*k+b) / (k * k + 1.0f);
    float x0 = (y0-b)/k;
    //调试信息
//    disp(13,x0,"x0");
//    disp(14,y0,"y0");

    // 输出垂足坐标
    data[0][0] = x0;
    data[0][1] = y0;
    // 检查剩余点的方向
    // 沿拟合直线向前的方向向量：(-1/k,-1)
//    float dx,dy;
//    for(int i = 1; i < N; i++){
//        dx = data[i][0]-data[i-1][0];
//        dy = data[i][1]-data[i-1][1];
//        if(dx/k+dy<=0.0f){//若反向则舍弃
//            data[i][0] = data[i-1][0];
//            data[i][1] = data[i-1][1];
//        }
//    }
}


// 点集等距采样2  使采样后点与点的距离为`dist`
// TODO: fix bug
void resample_points2(float pts_in[][2], uint16 num1, float pts_out[][2], uint16 *num2, float dist) {
    if (num1 < 0) {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    uint16 len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrtf(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrtf(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);

    }
    *num2 = len;
}


/*
 * min_dist:可接受的最大距离，若没有小于该距离的点则返回-1
 */
void find_nearest_point(float x0, float y0, float points[][2], int serch_range, int* result_idx, float min_dist){
    *result_idx = -1;
    float dx,dy,dist;
    for (int i = 0; i < serch_range; i++) {
        dx = points[i][0] - x0;
        dy = points[i][1] - y0;
        dist = sqrtf(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            *result_idx = i;
        }
    }
}
//返回x0,y0到points数组中最短点的距离
void find_nearest_dist(float x0, float y0, float points[][2], int serch_range, float* result_dist, float min_dist){
//    *result_dist = -1;
    float dx,dy,dist;
//    int num = MIN(serch_range, sizeof(points) / sizeof(points[0]));
    for (int i = 0; i < serch_range; i++) {
        dx = points[i][0] - x0;
        dy = points[i][1] - y0;
        dist = sqrtf(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            *result_dist = dist;
        }
    }
}

/*
 * 返回从左/右直道相对前进方向的角度(竖直向上为正方向)
 * 注意返回值为弧度制
 */
float get_straight_yaw(sint8 L_or_R){
    static float yaw_L, yaw_R, (*data)[2];
    static uint8 L_OK, R_OK;
    int i, dist, len;
    dist = (int)roundf(0.5f*angle_dist / sample_dist);
    switch (L_or_R){
        case -1://左侧
            if(L_OK) return yaw_L;
            if(is_long_straight0) len = long_straight_thres / sample_dist - 2*dist;
            else if(is_straight0) len = straight_thres / sample_dist - 2*dist;
            else if(Lpt0_found) len = Lpt0_rpts0s_id1 - 2*dist;
            else return 0;
            data=&rpts0s[dist];
            break;
        case 1://右侧
            if(R_OK) return yaw_R;
            if(is_long_straight1) len = long_straight_thres / sample_dist - 2*dist;
            else if(is_straight1) len = straight_thres / sample_dist - 2*dist;
            else if(Lpt1_found) len = Lpt1_rpts1s_id1 - 2*dist;
            else return 0;
            data=&rpts1s[dist];
            break;
        case 0://两侧
            return 0.5f*get_straight_yaw(-1)+0.5f*get_straight_yaw(1);
        default:
            L_OK = 0;
            R_OK = 0;
            return 0;
    }
    // 计算数据点的平均值
    float x_avg = 0, y_avg = 0, delta_x;
    for (i = 0; i < len; i++) {
        x_avg -= data[i][1];
        y_avg -= data[i][0];
    }
    x_avg /= (float)len;
    y_avg /= (float)len;

    // 计算最小二乘拟合的斜率和截距
    float numerator = 0, denominator = 0;
    for (i = 0; i < len; i++) {
        delta_x = -data[i][1] - x_avg;
        numerator -= delta_x * (data[i][0] + y_avg);
        denominator += delta_x * delta_x;
    }
    float k = numerator / denominator;
    if(L_or_R==1){
        yaw_R = atanf(k);
        R_OK=1;
        return yaw_R;
    }else{
        yaw_L = atanf(k);
        L_OK=1;
        return yaw_L;
    }
}

//获得当前偏航角相对ref的误差
float get_yaw_error(float ref){
    static float Error;
    Error = Yaw_a - ref;
    while(Error >  180) Error-=360;
    while(Error < -180) Error+=360;
    return Error;
}

/*
 * 图像绘制
 * 将数组中的点绘制到图像中
 * *img 传递的图像地址
 * line[][2] 点集坐标
 * len 点集个数
 */
uint8 float_line_to_img(image_t *img, float line[][2], uint16 len){
    int x,y;
    for(uint16 i = 0; i < len; i++){
//        if(!Image_Processing) return 1;//边线正被使用，画线中止
        x = Img_Zoom*(line[i][0]-94.0f)+94.0f;
        y = Img_Zoom*(line[i][1]-120.0f-Img_Move)+120.0f;
        if(x<=0||x>=img->width||y<=0||y>=img->height)continue;
        AT_IMAGE(img, x, y)=255;
    }
    return 0;//处理成功
}
/*
 * 图像清理函数
 * img 传递的图像地址
 */
void clear_image(image_t *img) {
//    assert(img && img->data);
    memset(img->data[0], 0, img->height * img->width);
//    if (IMAGEW == img->step) {
//        memset(img->data, 0, IMAGEW * IMAGEH);
//    } else {
//        for (int y = 0; y < IMAGEH; y++) {
//            memset(&AT(img, 0, y), 0, IMAGEW);
//        }
//    }
}
/*
 * 图像平移和缩放初始化函数
 * move 平移参数
 * zoom 缩放参数
 */
void SetImg_MoveAndZoom(float move, float zoom){
    Img_Move = move;
    Img_Zoom = zoom;
}
/*
 * 绘制×
 * img 传递的图像地址
 * x 绘制点的横坐标
 * y 绘制点的纵坐标
 * len ×的大小
 * value ×的颜色
 */
void draw_x(image_t *img, int x, int y, int len, uint8_t value)
{
    x = Img_Zoom*(x-94)+94;
    y = Img_Zoom*(y-120-Img_Move)+120;
    for (int i = -len; i <= len; i++) {
            AT(img, clip(x + i, 0, IMAGEW - 1), clip(y + i, 0, IMAGEH - 1)) = value;
            AT(img, clip(x - i, 0, IMAGEW - 1), clip(y + i, 0, IMAGEH - 1)) = value;
    }
}

/*
 * 直接绘制圆环
 * img 传递的图像地址
 * x 绘制点的横坐标
 * y 绘制点的纵坐标
 * radius 圆环半径
 * value 绘制的颜色
 */
void draw_o_direct(image_t *img, int x, int y, int radius, uint8_t value) {
    for (float i = -PI; i <= PI; i += PI / 10) {
            AT(img, clip(x + radius * cosf(i), 0, IMAGEW - 1), clip(y + radius * sinf(i), 0, IMAGEH - 1)) = value;
    }
}
/*
 * 绘制o
 * img 传递的图像地址
 * x 绘制点的横坐标
 * y 绘制点的纵坐标
 * len o的大小
 * value o的颜色
 */
void draw_o(image_t *img, int x, int y, int radius, uint8_t value)
{
    x = Img_Zoom*(x-94)+94;
    y = Img_Zoom*(y-120-Img_Move)+120;
    for (float i = -PI; i <= PI; i += PI / 10) {
            AT(img, clip(x + radius * cosf(i), 0, IMAGEW - 1), clip(y + radius * sinf(i), 0, IMAGEH - 1)) = value;
    }
}

//取三个点得到角度（弧度制）
float get_angle_3points(float* point1, float* point2, float* point3) {
        float dx1 = point2[0] - point1[0];
        float dy1 = point2[1] - point1[1];
        float dx2 = point3[0] - point2[0];
        float dy2 = point3[1] - point2[1];
        return Rad2Ang*atan2f(dx1*dy2-dy1*dx2,dx1*dx2+dy1*dy2);
}

// 角度变化率非极大抑制(kernel % 2 == 1)
void nms_angle(float angle_in[], uint16 num, float angle_out[], sint16 half) {
//    assert(kernel % 2 == 1);
    for (uint16 i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (sint16 j = -half; j <= half; j++) {  //在点领域[-half, half]内遍历
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;  //如果领域内某个点的角度绝对值大于当前点 则将当前点角度置零
                break;
            }
        }
    }
}
// 点集局部角度变化率
void local_angle_points(float pts_in[][2], uint16 num, float angle_out[], int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= dist || i >= num - dist) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        angle_out[i] = atan2f(dx1*dy2-dy1*dx2,dx1*dx2+dy1*dy2);
    }
}

// 点集等距采样  使走过的采样前折线段的距离为`dist`
void resample_points(float pts_in[][2], uint16 num1, float pts_out[][2], uint16 *num2, float dist){
    int i, j, len = 0, seg_num=0;
    float r=0, x0, y0, dx, dy, dn, dn_sum=0;
    for(i=0; i<num1-1 && len<*num2; i++){
        x0 = pts_in[i][0];
        y0 = pts_in[i][1];
        dx = pts_in[i+1][0] - x0;
        dy = pts_in[i+1][1] - y0;
        dn = sqrtf(dx*dx+dy*dy);
        if(dn_sum+dn < dist) {
            dn_sum += dn;
            continue;
        }
        dx /= dn;
        dy /= dn;

        seg_num = (int)((dn+dn_sum)/dist);
        for(j=1; j<=seg_num && len<*num2; j++){
            r = dist*j-dn_sum;
            pts_out[len][0] = x0 + dx * r;
            pts_out[len][1] = y0 + dy * r;
            len++;
        }
        dn_sum += dn-seg_num*dist;
    }
    *num2 = (uint16)len;
}



// 点集三角滤波（kernel % 2 == 1）
void blur_points(float pts_in[][2], uint16 num, float pts_out[][2], sint16 kernel) {
//    assert(kernel % 2 == 1);
    sint16 i, j, half = kernel / 2;
    for (i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}


/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const sint8 dir_front[4][2] = { {0,-1}, {1,0}, {0,1}, {-1,0} };
const sint8 dir_frontleft[4][2] = { {-1,-1}, {1,-1}, {1,1}, {-1,1} };
const sint8 dir_frontright[4][2] = { {1,-1}, {1,1}, {-1,1}, {-1,-1} };
//左手迷宫巡线
void findline_lefthand_adaptive(int block_size, int clip_value, int x, int y, sint16 pts[][2], uint16 *num)
{
//    assert(num && *num >= 0);
//    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;              //搜索区域的半径
    int step = 0, dir = 0, turn = 0;
    while (step < *num && 1 < x && x < IMAGEW - 3 && 1 < y && y < IMAGEH - 2 && turn < 4) {//路径点未满？、未到图像边界、转弯次数小于4（周围找不到）时继续执行
        int local_thres = 0, pixel_cnt = 0, X,Y;
        for (int dy = -half; dy <= half; dy++) { //遍历block_size × block_size 的局部区域，计算灰度均值
            for (int dx = -half; dx <= half; dx++) {
                X = x + dx;  //计算局部区域像素的x坐标
                Y = y + dy;  //计算局部区域像素的y坐标
                if(0>X || X>IMAGEW-2 || 0>Y || Y>IMAGEH-1)continue;//跳过边界外的像素
                local_thres += RAW_IMG(X,Y);
                pixel_cnt++;
            }
        }
        local_thres /= pixel_cnt;  //计算局部灰度均值
        local_thres -= clip_value;  //减去clip_value得到局部阈值（不知道有什么意义？）
        if(y>80){  //累加y>80的局部阈值，并计数
            thres_sum_L += local_thres;
            thres_sum_cnt_L++;
        }
//        static int thres_use = 0;
//        if(bridge_type != BRIDGE_NONE || Total_mileage < 1.5 || zebra_flag)  //大津法阈值和局部阈值的切换
//            thres_use0 = thres_use;
//        else thres_use0 = local_thres;
//        int current_value = RAW_IMG( x, y);
        thres_use0 = local_thres;
        int front_value = RAW_IMG( x + dir_front[dir][0], y + dir_front[dir][1]);  //获取前方像素的灰度值（注意是前方 不是上方）
        int frontleft_value = RAW_IMG( x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);  //获得左前方的灰度值
        if (front_value < thres_use0) {  //如果上方是赛道边界
            dir = (dir + 1) % 4;  //右转
            turn++;  //转弯次数+1
        } else if (frontleft_value < thres_use0) {  //如果左前方是赛道边界
            x += dir_front[dir][0];  //沿当前方向向上移动
            y += dir_front[dir][1];
            pts[step][0] = (sint16)x;  //记录当前坐标到边线数组
            pts[step][1] = (sint16)y;
            step++;
            turn = 0;  //重置转弯次数
        } else {  //左前方和前方都没有赛道边界
            x += dir_frontleft[dir][0];  //向左前方移动
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;  //左转
            pts[step][0] = (sint16)x;  //记录当前坐标到边线数组
            pts[step][1] = (sint16)y;
            step++;
            turn = 0;
        }
    }
    *num = (uint16)step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(int block_size, int clip_value, int x, int y, sint16 pts[][2], uint16 *num) {
//    assert(img && img->data);
//    assert(num && *num >= 0);
//    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < *num && 1 < x && x < IMAGEW - 3 && 1 < y && y < IMAGEH - 2 && turn < 4) {//到图像边界截止
        int local_thres = 0, pixel_cnt = 0, X,Y;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                X = x + dx;
                Y = y + dy;
                if(0>X || X>IMAGEW-2 || 0>Y || Y>IMAGEH-1)continue;//改进了接近边界时的处理
                local_thres += RAW_IMG(X,Y);
                pixel_cnt++;
            }
        }
        local_thres /= pixel_cnt;
        local_thres -= clip_value;
        if(y>80){
            thres_sum_R += local_thres;
            thres_sum_cnt_R++;
        }
//        static int thres_use = 0;
//        if(bridge_type != BRIDGE_NONE || Total_mileage < 1.5 || zebra_flag) //大津法阈值和局部阈值的切换
//            thres_use1 = thres_use;
//        else thres_use1 = local_thres;
//        int current_value = RAW_IMG( x, y);
        thres_use1 = thres_use;
        int front_value = RAW_IMG( x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = RAW_IMG( x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < thres_use1) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < thres_use1) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = (sint16)x;
            pts[step][1] = (sint16)y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = (sint16)x;
            pts[step][1] = (sint16)y;
            step++;
            turn = 0;
        }
    }
    *num = (uint16)step;
}
/*
 * 大津法取阈值
返回类型:uint8_t
输入类型:image_t
输入变量:MinThreshold——最小输出阈值   MaxThreshold——最大输出阈值
*/
uint8_t getOSTUThreshold(image_t *img, uint8_t MinThreshold, uint8_t MaxThreshold)
{
    MaxThreshold=MAX(MaxThreshold,254);//防止达到uint8能表示最大整数(255)而进入死循环
    uint16_t Histogram[256];//灰度频次 0~255
    uint8_t OUSTThreshold = 0;
    uint32_t PixelAmount = 0, Value_Sum = 0;
    uint64_t sigma = 0, maxSigma = 0;
    float w1 = 0, w2 = 0;
    int32_t u1 = 0, u2 = 0;
    uint8_t MinValue = 0, MaxValue = 255;

    //各灰度的像素点个数
    uint8_t *ptr = img->data[0];
    uint8_t *ptrEnd = img->data[0] + img->height * img->width;
    while (ptr != ptrEnd) {
        ++Histogram[*ptr++];
    }

    for (uint8_t m = 0; m < MinThreshold; m++) {
        Histogram[m] = 0;
    }

    //寻找最大和最小灰度
    for (MinValue = 0; Histogram[MinValue] == 0 && MinValue < 255; ++MinValue);
    for (MaxValue = 255; Histogram[MaxValue] == 0 && MaxValue > 0; --MaxValue);

    if (MaxValue == MinValue) return (uint8_t)clip((int)(MaxValue-OSTU_clip),(int)MinThreshold,(int)MaxThreshold);    // 只有一个颜色
    if (MinValue + 1 == MaxValue) return (uint8_t)clip((int)(MinValue-OSTU_clip),(int)MinThreshold,(int)MaxThreshold);// 只有二个颜色

    if (MinValue < MinThreshold) {
        MinValue = MinThreshold;
    }
    if (MaxValue > MaxThreshold) {
        MaxValue = MaxThreshold;
    }

    uint32_t Pixel_Integral[256] = {0};   //像素积分
    uint32_t Value_Integral[256] = {0};    //灰度积分
    for (uint8_t i = MinValue; i <= MaxValue; ++i) {
        PixelAmount += Histogram[i];      //像素总数
        Value_Sum += Histogram[i] * i;     //灰度总和
        Pixel_Integral[i] = PixelAmount;
        Value_Integral[i] = Value_Sum;
    }
    for (uint8_t i = MinValue; i <= MaxValue; ++i) {
        w1 = (float) Pixel_Integral[i] / PixelAmount;  //前景像素点比例
        w2 = 1 - w1;                               //背景比例
        u1 = (int32_t) (Value_Integral[i] / w1);                   //前景平均灰度
        u2 = (int32_t) ((Value_Sum - Value_Integral[i]) / w2);      //背景平均灰度
        sigma = (uint64_t) (w1 * w2 * (u1 - u2) * (u1 - u2));
        if (sigma >= maxSigma) {
            maxSigma = sigma;
            OUSTThreshold = i;
        }
//        else {
//            break;
//        }
    }
    return (uint8_t)clip((int)(OUSTThreshold - OSTU_clip), (int)MinThreshold, (int)MaxThreshold);
}

/* 图像去畸变
 * 输入原图像地址
 * 输出去畸变图像地址
 */
void undistortImage(uint8_t srcImage[120][188], uint8_t dstImage[120][188])
{
    for (int y = 0; y < 120; y++)
    {
        for (int x = 0; x < 188; x++)
        {
            float newX = mapX[y][x];
            float newY = mapY[y][x];

            if (newX >= 0 && newY >= 0)
            {
                int srcX = (int)(newX + 0.5f);  // 最近邻取整
                int srcY = (int)(newY + 0.5f);

                // 防止取越界
                if (srcX >= 0 && srcX < 188 && srcY >= 0 && srcY < 120)
                {
                    dstImage[y][x] = srcImage[srcY][srcX];
                }
                else
                {
                    dstImage[y][x] = 255;  // 白色
                }
            }
            else
            {
                dstImage[y][x] = 255;  // 无效区域设成白色
            }
        }
    }
}


