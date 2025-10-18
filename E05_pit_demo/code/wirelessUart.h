/*
 * wirelessUart.h
 *
 *  Created on: 2025年10月10日
 *      Author: ji_rencai
 */

#ifndef CODE_WIRELESSUART_H_
#define CODE_WIRELESSUART_H_

#include "zf_common_headfile.h"
// 接入无线转串口模块
//      模块管脚            单片机管脚
//      RX                  查看 zf_device_wrieless_uart.h 中 WIRELESS_UART_RX_PINx  宏定义 默认 P10_6
//      TX                  查看 zf_device_wrieless_uart.h 中 WIRELESS_UART_TX_PINx  宏定义 默认 P10_5
//      RTS                 查看 zf_device_wrieless_uart.h 中 WIRELESS_UART_RTS_PINx 宏定义 默认 P10_2
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

extern char textDisplay[];
void wirelessUartInit(void);
void wirelessUartDisplay(void);


#endif /* CODE_WIRELESSUART_H_ */
