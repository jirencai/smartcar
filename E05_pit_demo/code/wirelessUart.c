/*
 * wirelessUart.c
 *
 *  Created on: 2025年10月10日
 *      Author: ji_rencai
 */
#include "wirelessUart.h"
#define LED1                    (P20_9)

char textDisplay[64];
//uint8 data_buffer[32];
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
    sprintf(textDisplay, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",\
            (float)encoder_data_2, (float)encoder_data_4,(float)pid_speed_l.target_val, (float)pid_speed_r.target_val, (float)pid_speed_l.integral, (float)pid_speed_r.output);
    wireless_uart_send_string(textDisplay);
}
//void wirelessUartReceive(void)
//{
//    data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
//    if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
//    {
//        wireless_uart_send_buffer(data_buffer, data_len);                     // 将收到的消息发送回去
//        memset(data_buffer, 0, 32);
//        func_uint_to_str((char *)data_buffer, data_len);
//        wireless_uart_send_string("\r\ndata len:");                                 // 显示实际收到的数据信息
//        wireless_uart_send_buffer(data_buffer, strlen((const char *)data_buffer));    // 显示收到的数据个数
//        wireless_uart_send_string(".\r\n");
//    }
//    system_delay_ms(50);
//
//}

#define FRAME_HEADER1   0xAA
#define FRAME_HEADER2   0x55
#define FRAME_MAX_LEN   64

static uint8_t rx_buf[FRAME_MAX_LEN];
static uint8_t rx_index = 0;
static uint8_t frame_len = 0;
static uint8_t recv_state = 0; // 0:头1  1:头2  2:长度  3:数据

// 处理完整帧的回调
void parse_frame(uint8_t *frame, uint8_t len)
{
    // 用户可根据协议自定义处理
    wireless_uart_send_string("Frame OK: ");
    wireless_uart_send_buffer(frame, len);
    wireless_uart_send_string("\r\n");
}

// 主接收函数（循环调用）
void wirelessUartReceive(void)
{
    uint8_t tmp_buf[32];
    uint8_t data_len = (uint8_t)wireless_uart_read_buffer(tmp_buf, sizeof(tmp_buf));
    if(data_len == 0) return;

    for(uint8_t i = 0; i < data_len; i++)
    {
        uint8_t byte = tmp_buf[i];

        switch(recv_state)
        {
            case 0: // 等待帧头1
                if(byte == FRAME_HEADER1)
                {
                    rx_buf[0] = byte;
                    recv_state = 1;
                }
                break;

            case 1: // 等待帧头2
                if(byte == FRAME_HEADER2)
                {
                    rx_buf[1] = byte;
                    recv_state = 2;
                }
                else
                {
                    recv_state = 0;
                }
                break;

            case 2: // 接收长度
                frame_len = byte;
                if(frame_len > FRAME_MAX_LEN - 3) // 防止越界
                {
                    recv_state = 0;
                    break;
                }
                rx_buf[2] = byte;
                rx_index = 3;
                recv_state = 3;
                break;

            case 3: // 接收数据
                rx_buf[rx_index++] = byte;
                if(rx_index >= frame_len + 3) // 数据接收完毕
                {
                    parse_frame(rx_buf, rx_index);
                    recv_state = 0;
                    rx_index = 0;
                }
                break;

            default:
                recv_state = 0;
                rx_index = 0;
                break;
        }
    }
}


