/*
 * wirelessUart.c
 *
 *  Created on: 2025年10月10日
 *      Author: ji_rencai
 */
#include "wirelessUart.h"
#define LED1                    (P20_9)

char textDisplay[64];
uint8_t textNum;
uint8_t data_buffer[16];
//uint8 data_len;
void wirelessUartInit(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
    while(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
        system_delay_ms(100);                                               // 短延时快速闪灯表示异常
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息

}

void wirelessUartDisplay(void)
{
    sprintf(textDisplay, "%f,%f,%f,%f,%f,%f\r\n",
            (float)error, (float)img_error, (float)far_error, (float)last_error, (float)delta_error, (float)turn_value);
    wireless_uart_send_string(textDisplay);
}
void icmTest(void)
{
    sprintf(textDisplay, "%f,%f,%f,%f\r\n",
            (float)Yaw_gyro, (float)Pitch_a, (float)Roll_a, (float)Yaw_a);
    wireless_uart_send_string(textDisplay);

            //转向加速度         各种角度值
}
void motorTest(void)
{
    sprintf(textDisplay, "%f,%f,%f,%f,%f,%f\r\n",
            (float)pid_speed_l.output, (float)pid_speed_r.output, (float)encoder_data_2, (float)encoder_data_4, (float)img_error, (float)Yaw_gyro);
    wireless_uart_send_string(textDisplay);

}

void readBuffer(void)
{
    textNum = (uint8_t)wireless_uart_read_buffer(data_buffer, 16);
    if (textNum > 0)
    {
        data_buffer[textNum] = 0; // !!! 必须加字符串结束符
        if (strncmp((char *)data_buffer, "turnKp=", 7) == 0)                turnPid.Kp = atof((char *)data_buffer + 7);
        else if (strncmp((char *)data_buffer, "turnKp2=", 8) == 0)          turnPid.Kp2 = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "turnGKD=", 8) == 0)          turnPid.GKD = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "motorKp=", 8) == 0)          pid_speed_r.Kp = pid_speed_l.Kp = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "motorKi=", 8) == 0)          pid_speed_r.Ki = pid_speed_l.Ki = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "motorKd=", 8) == 0)          pid_speed_r.Kd = pid_speed_l.Kd = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "motorIntMax=", 12) == 0)     pid_speed_r.limit = pid_speed_l.limit = atof((char *)data_buffer + 12);
        else if (strncmp((char *)data_buffer, "dutyMax=", 8) == 0)          turnPid.outLimit = atof((char *)data_buffer + 8);
        else if (strncmp((char *)data_buffer, "speed=", 6) == 0)            straight_value = atof((char *)data_buffer + 6);

        memset(data_buffer, 0, 16);

    }
    system_delay_ms(50);

}

#define FRAME_HEADER1   0xAA
#define FRAME_HEADER2   0x55
#define FRAME_MAX_LEN   64



