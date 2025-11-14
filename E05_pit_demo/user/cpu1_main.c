/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


uint8 no_line_cnt=0,straight_cnt=0;         //无边线计数
uint16_t testNum = 0;

// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
//    ips200_set_dir(1);     //定义屏幕方向
//    ips200_init(IPS200_TYPE_SPI);        //屏幕初始化
//    ips200_clear();
//
    ips200Init();             //屏幕初始化

    ips200_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 80, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
    }
    ips200_show_string(0, 16, "init success.");
//    mt9v03x_set_exposure_time(30);



    // 此处编写用户代码 例如外设初始化代码等

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {

//         此处编写需要循环执行的代码
        if(mt9v03x_finish_flag)
        {
//            ips200_show_int(3, 16, testNum, 5);
//            testNum += 1;
//            ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);                       // 显示原始图像
//            ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 64);     // 显示二值化图像
            undistortImage(mt9v03x_image, new_image1);              //图像去畸变
            process_image();                                        //边线处理和提取
            if(rpts0s_num<5 && rpts1s_num<5)
            {  //无边线状态计数器
                if(no_line_cnt<200)no_line_cnt++;
            }
            else
            {
                if(no_line_cnt>=0)no_line_cnt--;
            }
//                if(no_line_cnt > 15) Dynamic_begin_x = 0;
//                else Dynamic_begin_x = 1;


            find_corner();     // 角点提取&筛选

            /*初始巡线模式*/
            // 单侧线少，切换巡线方向  切外向圆
            if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60) {
                track_route = TRACK_RIGHT;
            } else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60) {
                track_route = TRACK_LEFT;
            } else if (rpts0s_num < 60 && rpts1s_num > rpts0s_num) {
                track_route = TRACK_RIGHT;
            } else if (rpts1s_num < 60 && rpts0s_num > rpts1s_num) {
                track_route = TRACK_LEFT;
            }

            /*初始跟线模式*/
            track_method = EXTEND_LINE;

            /*元素检测*/
//            ①先检测斑马线元素
            check_garage();

//            ②再按级别检测其他元素
            while(1)
            {
                if (cross_enable)
                {
                    if (cross_type == CROSS_NONE) check_cross();
                    if (cross_type != CROSS_NONE) break;
                }
                if (circle_enable)
                {
                    if (circle_type == CIRCLE_NONE) check_circle();
                    if (circle_type != CIRCLE_NONE) break;
                }
            }

//            ③识别到元素以后开始进行元素识别处理
            if (cross_type != CROSS_NONE)
            {
                cross_run();
                circle_type = CIRCLE_NONE;
            }
            else if (circle_type != CIRCLE_NONE)
            {
                circle_run();
            }


            /*不同情况下的中线跟踪*/
            if (cross_type == CROSS_IN)
            {
                if (track_route == TRACK_LEFT)
                {
                    track_leftline_c(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
                                     (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);  //+ far_Lpt0_rpts0s_id  - far_Lpt0_rpts0s_id
                    rpts_num = far_rpts0s_num - far_Lpt0_rpts0s_id; //更新远边线点集数量
                }
                else
                {
                    track_rightline_c(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
                                     (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2); // + far_Lpt1_rpts1s_id  - far_Lpt1_rpts1s_id
                    rpts_num = far_rpts1s_num - far_Lpt1_rpts1s_id; //更新远边线点集数量
                }

            }
            else
            {
                Track_CenterLine();  //计算中线点
                if (track_route == TRACK_LEFT)
                {
                    rpts = rptsc0;
                    rpts_num = rptsc0_num;
                }
                else
                {
                    rpts = rptsc1;
                    rpts_num = rptsc1_num;
                }
            }
            /*车轮坐标参数初始化*/
            static float cx=94;
            static float far_cx=94;
            static float cy=119;
            static float far_cy=60;

            /*中间数组的索引*/
            begin_id = 0;
            find_nearest_point(cx, cy, rpts, rpts_num/2, &begin_id, 1000.0f);

            far_begin_id = 0;
            find_nearest_point(far_cx, far_cy, rpts, rpts_num/2, &far_begin_id, 1000.0f);
            far_cx = rpts[far_begin_id][0];
            far_cy = rpts[far_begin_id][1];

            if (begin_id >= 0 && rpts_num && rpts_num - begin_id >= 5)
            {
                if(cross_type != CROSS_IN) aim_idx_f = (int)(now_aim_distance / sample_dist);
                else aim_idx_f = (int)(0.35f / sample_dist);  //锚点索引计算
                switch(track_method)
                {
                    case ROUND_SCAN:
                        // 中线等距采样
                        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
                        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
                        // 圆等距跟踪
                        round_scan(&aim_idx,cx,cy,now_aim_distance*pixel_per_meter);
                        break;
                    case EXTEND_LINE:
                        // 线等距跟踪（新归一化方法）
                        //TODO:fix bug
                        if(cross_type != CROSS_IN)
                        {
                            extend_line(rpts+begin_id,cx,cy,MIN(10,rpts_num-begin_id));//中线线性拟合(最近的N个) 并改变初始点if(cross_type != CROSS_IN)
                        }
                        // 中线等距采样
                        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
                        resample_points2(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
                        break;
                }
                if(Circle_In_Ready==1) Circle_In_Ready_Check();

                // 计算远锚点偏差值
                float dx = cx - rptsn[aim_idx][0];
                float dy = cy - rptsn[aim_idx][1]; //+ 0.2 * pixel_per_meter;
                float far_dx = far_cx - rptsn[rptsn_num - 1][0];
                float far_dy = far_cy - rptsn[rptsn_num - 1][1];

                error = atan2f(dx, dy) * Rad2Ang;//左转为正 单位为度

                far_error = atan2f(far_dx, far_dy) * Rad2Ang;

                if(lock_yaw){
                    target_yaw = Yaw_a + img_error; // + img_error
                    lock_yaw = 0;
                }
//                if(abs(last_error - error) > 25) error = 0.0;
//                if(cross_type == CROSS_IN || bridge_type == BRIDGE_IN) img_error = target_yaw-Yaw_a;  //CROSS_IN和BRIDGE_IN状态下锁yaw值
//                    if(cross_type == CROSS_BEGIN) img_error = 0.5f * error + 0.5f * img_error; //CROSS_BEGIN状态下尽量准确的巡线 以防过滤掉应当偏转的角度
//                    else img_error = 0.6f * error + 0.4f * img_error;  //平滑误差
//                if(bridge_type == BRIDGE_IN && !(rpts0s_num > 5 && rpts1s_num > 5)) {
//                    img_error = last_error;
//                }
                if(cross_type == CROSS_IN) img_error = 0.4f * error + 0.6f * img_error;
                else img_error = 0.5f * error + 0.5f * img_error;
                delta_error = fabs(img_error - last_error);  //计算偏差值变化量
                last_error = img_error; //记录本次偏差值用于下一次的变化量计算
                R = fabsf((aim_idx / (pixel_per_meter*2))/sinf(img_error * Ang2Rad));
                K = 1.0f/R;
//                if(bridge_type == BRIDGE_IN && delta_error > 5.0f) aim_yaw_a = Yaw_a + last_error;
//                else aim_yaw_a = Yaw_a + img_error;
                aim_yaw_a = Yaw_a + img_error;
//                    Image_Processing=0;
                mt9v03x_finish_flag = 0;
                fre_cy.img_end_time = fre_cy.m10stime;

//                    fre_cy.
                //直道计数
//                if(is_straight1 && straight_cnt<5)straight_cnt++;
//                if(!is_straight1 && straight_cnt)straight_cnt--;
//                    aim_yaw_a = Yaw_a + img_error;//fclip(angle_diff, -15, 15);//输出角度至转向环 目标偏航角
//                    Aim_X_Pos = (rptsn[0][0] - cx)*0.01f;
//                    Image_Processing=0;
            } else
            {
                    // 中线点过少(出现问题)，则不控制舵机
                    rptsn_num = 0;
                    aim_yaw_a = Yaw_a + img_error*1.0;//原速转弯
                    mt9v03x_finish_flag = 0;
            }

        }




        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
