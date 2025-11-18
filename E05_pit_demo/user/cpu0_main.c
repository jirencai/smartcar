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
* 文件名称          cpu0_main
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
#include "isr_config.h"
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

uint8_t new_image[MT9V03X_H][MT9V03X_W];
uint8_t new_image1[MT9V03X_H][MT9V03X_W];   //去畸变以后的原始图像

#define LED1                    (P20_9)
#define LED2                    (P20_8)

//定时中断宏定义
#define KEYGET                  (CCU60_CH0 )                                     // 使用的周期中断编号
#define MOTORCTL                (CCU60_CH1 )
#define IMUGET                  (CCU61_CH1 )
//#define MOTORCTL                (CCU61_CH0 )
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    /*device初始化*/
    motorInit();                //电机初始化
    keyInit();                  //按键初始化
//    ips200Init();             //屏幕初始化
    wirelessUartInit();         //无线串口初始化
    Init_ICM42688();            //陀螺仪初始化
    Filter_Init();              //陀螺仪六路滤波初始化

    if(IMU_calibrate_flag == 1)
    {
        gyro_x_correction = gyro_x_correction_experience;
        gyro_y_correction = gyro_y_correction_experience;
        gyro_z_correction = gyro_z_correction_experience;//零漂修正
    }else{
        IMU_calibration();   //陀螺仪校准
        gyro_x_correction_experience = gyro_x_correction;
        gyro_y_correction_experience = gyro_y_correction;
        gyro_z_correction_experience = gyro_z_correction;//零漂修正
    }




    /*低通滤波器初始化*/
    low_pass_filter_Init();

    /*编码器及其中断初始化*/
    encoderInit();                                                              // 编码器初始化

    pit_ms_init(MOTORCTL, 1);                                                   // 初始化 CCU6_0_CH1 为周期中断 2ms 周期
    pit_ms_init(KEYGET, 20);                                                    // 初始化 CCU6_0_CH0 为周期中断 20ms 周期
    pit_us_init(IMUGET, 250);                                                   // 初始化 CCU6-1 CH1 位周期 250us 周期
    //    pit_ms_init(MOTORCTL, 25);                                                       // 初始化 CCU6_0_CH0 为周期中断 20ms 周期
//    motorLeftWrite(2000);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
	while (TRUE)
	{

        // 此处编写需要循环执行的代码
	    keyScan();
	    motorTest();
	    readBuffer();
        // 此处编写需要循环执行的代码

	}
}


#pragma section all restore
