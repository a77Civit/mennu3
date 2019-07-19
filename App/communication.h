#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "common.h"
#include "include.h"

int communication_init(void);          // 通信初始化


void Laser_dective(void);
void Laser_handler2(void);
void Laser_handler3(void);


void uart_sentn(UARTn_e uartn, const uint8 *pt, int bit_len);  // 串口定长发送

#define LASER_PORT3 UART3  // 三轮激光测距串口
//#define ECHO_PORT UART3  // 超声测距串口
#define LASER_PORT2 UART4  // 二轮激光测距串口

#define BLUE_BUFF_LEN 10  // 蓝牙接收缓冲长度
#define ECHO_BUFF_LEN 15  // 超声数组接收缓冲长度
#define ECHO_KEEP_LEN 3   // 超声数字接收缓冲长度

extern unsigned char rxempty;
extern unsigned short int length_val;
extern unsigned short int real_length;
extern unsigned char rxflag,rxcnt,waitflag;
extern int Raser_Flag;

#endif