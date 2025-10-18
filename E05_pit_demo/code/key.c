/*
 * key.c
 *
 *  Created on: 2025年10月9日
 *      Author: ji_rencai
 */
#include "key.h"
#define KEYSUM 6

struct keys key[KEYSUM] = {{0,0,0,0,0}};
uint8_t i;
void keyInit(void)
{
    gpio_init(key1, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
    gpio_init(key2, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
    gpio_init(key3, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
    gpio_init(key4, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
    gpio_init(key5, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
    gpio_init(key6, GPI, GPIO_HIGH, GPI_PULL_UP);                           // GPIO 初始化为输入 默认上拉输入高
}

/*按键识别过程*/
void keyProcess(void)
{
    key[1 - 1].key_read = gpio_get_level(key1); // key1 (对应按键 S1)
    key[2 - 1].key_read = gpio_get_level(key2); // key2 (对应按键 S2)
    key[3 - 1].key_read = gpio_get_level(key3); // key3 (对应按键 S3)
    key[4 - 1].key_read = gpio_get_level(key4); // key4 (对应按键 S4)
    key[5 - 1].key_read = gpio_get_level(key5); // key5 (对应按键 S5)
    key[6 - 1].key_read = gpio_get_level(key6); // key6 (对应按键 S6)

    for (i = 0; i < KEYSUM; i++)
    {
        switch(key[i].key_sta)
        {

            case 0: //发现电平发生变化，进行判断
            {
                if (key[i].key_read == 0)
                {
                    key[i].key_sta = 1;
                    key[i].key_time = 0;
                }

            }
            break;

            case 1: //进入状态1，进一步验证是否是按键
            {
                if (key[i].key_read == 0)
                {
                    key[i].key_sta = 2;
                }
                else
                {
                    key[i].key_sta = 0;
                }
            }
            break;

            case 2:
            {
                if (key[i].key_read == 1) //判断此时为按键松开，计算时间是否为短按键
                {
                    key[i].key_sta = 0;
                    if (key[i].key_time < 70)
                    {
                        key[i].key_singleFlag = 1;
                    }
                }
                else //若还没有松开，实时判断是不是长按键
                {
                    key[i].key_time += 1;
                    if (key[i].key_time >= 70)
                    {
                        key[i].key_longFlag = 1;
                    }
                }
            }
            break;
        }
    }

}
char text[20];
/*按键扫描*/
void keyScan(void)
{

/*
    按键测试：
        S3按键按下，左右电机同时增加占空比
        S4按键按下，左右电机同时减小占空比
*/
    if (key[3 - 1].key_singleFlag == 1)
    {
        key[3 - 1].key_singleFlag = 0;
        pid_speed_r.target_val += 20;
        pid_speed_l.target_val += 20;

//        wireless_uart_send_string("key3 is pressed.\r\n");
//        if (dutyL + 5 <= 100)
//        {
//            dutyL += 5;
//        }
//        if (dutyR + 5 <= 100)
//        {
//            dutyR += 5;
//        }
//        pwm_set_duty(PWM_L, dutyL * (DUTYMAX / 100));                  // 计算占空比
//        pwm_set_duty(PWM_R, dutyR * (DUTYMAX / 100));                  // 计算占空比
//        sprintf(text, "dutyL: %d, dutyR: %d\n", dutyL, dutyR);
//        wireless_uart_send_string(text);
    }
    if (key[4 - 1].key_singleFlag == 1)
    {
        key[4 - 1].key_singleFlag = 0;
        pid_speed_r.target_val -= 20;
        pid_speed_l.target_val -= 20;

//        wireless_uart_send_string("key4 is pressed.\r\n");
//
//        if (dutyL - 5 >= 0)
//        {
//            dutyL -= 5;
//        }
//        if (dutyR - 5 >= 0)
//        {
//            dutyR -= 5;
//        }
//        pwm_set_duty(PWM_L, dutyL * (DUTYMAX / 100));                  // 计算占空比
//        pwm_set_duty(PWM_R, dutyR * (DUTYMAX / 100));                  // 计算占空比
//        sprintf(text, "dutyL: %d, dutyR: %d\n", dutyL, dutyR);
//        wireless_uart_send_string(text);
    }
}


