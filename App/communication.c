
/**
 * name:       communication.c
 * usage:      --
 * author:     [[
 * date:       2018-07-11 09:01:09
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"


int Raser_Flag = 0;     // 激光获取数据

unsigned char rxbuf[16], rxempty, rxcnt, rxflag, waitflag;
unsigned short int length_val = 2000, real_length = 2000;  //激光测距解算最终返回结果。
unsigned int timeout;


/**
 * 定长发送 (不需要以 '\0' 结尾)
 * input:
 *   uartn: uart 模块号
 *   pt: 数据首地址, 强制转义为 uint8 以符合发送规则
 *   bit_len: 数据的字节数
 */
void uart_sentn(UARTn_e uartn, const uint8 *pt, int bit_len) {
  int aa = 0;
  for (aa = 0; aa < bit_len; aa++) {
    uart_putchar(uartn, *pt++);
  }
}

/**
 * 通信初始化, 配置uart模块和中断
 */
int communication_init(void) {
  // UART 初始化


  // UART4 中断初始化, 蓝牙或者WiFi
  //set_vector_handler(UART4_RX_TX_VECTORn, car_communication_handler);
  //UART_C2_REG(UARTN[UART4]) |= UART_C2_RIE_MASK;  // 使能UART接收中断
  //enable_irq((IRQn_t)(UART4_RX_TX_IRQn));         // 使能IRQ中断
  //uart_rx_irq_en (BLUE_PORT); 
  
  // UART3 中断初始化, 激光测距
  //设置抢占优先级为1，子优先级为2
  //NVIC_SetPriority(UART3_RX_TX_VECTORn,
  //                 NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 2));
  



  return 0;
}




/*
TOF激光测距主动读取程序
Version 1.0
Author：Civic Cheng
Input： None
OutPUt：NOne
Interrupt Model：UART3
Function：
 */
void Laser_dective(void)
{
  uint8 i;
  //被动读取读数。
  //timeout = 50000;
  //while ((rxflag == 0) );//注释掉之后不等待了，放进中断里不等待了如果主程序调用没有读取完跳过此次读取
  if (rxflag)  //接收到1组有效数据
  {
    for (i = 0; i < rxcnt; i++) {
      if (rxbuf[i] == 'm') {
        if (rxbuf[i + 1] == 'm')  // ASCII码转换为16进制数据，单位mm
        {
          if ((i > 0) && (rxbuf[i - 1] >= '0') && (rxbuf[i - 1] <= '9'))
            length_val = rxbuf[i - 1] - '0';
          if ((i > 1) && (rxbuf[i - 2] >= '0') && (rxbuf[i - 2] <= '9'))
            length_val += (rxbuf[i - 2] - '0') * 10;
          if ((i > 2) && (rxbuf[i - 3] >= '0') && (rxbuf[i - 3] <= '9'))
            length_val += (rxbuf[i - 3] - '0') * 100;
          if ((i > 3) && (rxbuf[i - 4] >= '0') && (rxbuf[i - 4] <= '9'))
            length_val += (rxbuf[i - 4] - '0') * 1000;
          break;
        }
      }
    }
    real_length = length_val;
    rxflag = 0;
    rxcnt = 0;

    
   
}
}

/*---------------------------------------------------------------
【函    数】UART3_Handler
【功    能】TOF测距的的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void Laser_handler2(void)
{
  if (UART_S1_REG(UARTN[LASER_PORT2]) & UART_S1_RDRF_MASK)
  {
    if (rxflag == 0) {
      rxbuf[rxcnt++] = UART_D_REG(UARTN[LASER_PORT2]);  //读取接收到的数据
      if (rxcnt > 2) {
        if (waitflag == 1) {
          if ((rxbuf[rxcnt - 2] == 'o') &&//设置被动读取的指令的反馈检测，检测到OK字符即成功设置，下一次进入距离督促结算。
              (rxbuf[rxcnt - 1] =='k'))  //串口接收距离数据格式ASCII码0mm~2000mm
          {
            waitflag = 2;
          }
        } else 
        {
          if ((rxbuf[rxcnt - 1] == 'm') &&
              (rxbuf[rxcnt - 2] == 'm'))  //串口接收距离数据格式ASCII码0mm~2000mm
          {
            rxflag = 1;
            Laser_dective();
            
          }
        }
      }
      if (rxcnt >= 16) {
        rxcnt = 0;
      }
    } else {
      rxempty = UART_D_REG(UARTN[LASER_PORT2]);  //读取接收到的数据
    }
  }


  }



/*---------------------------------------------------------------
【函    数】UART4_Handler
【功    能】TOF测距的的中断服务函数
【参    数】无
【返 回 值】无
【注意事项】注意进入后要清除中断标志位
----------------------------------------------------------------*/
void Laser_handler3(void)
{
  if (UART_S1_REG(UARTN[LASER_PORT3]) & UART_S1_RDRF_MASK)
  {
    if (rxflag == 0) {
      rxbuf[rxcnt++] = UART_D_REG(UARTN[LASER_PORT3]);  //读取接收到的数据
      if (rxcnt > 2) {
        if (waitflag == 1) {
          if ((rxbuf[rxcnt - 2] == 'o') &&//设置被动读取的指令的反馈检测，检测到OK字符即成功设置，下一次进入距离督促结算。
              (rxbuf[rxcnt - 1] =='k'))  //串口接收距离数据格式ASCII码0mm~2000mm
          {
            waitflag = 2;
          }
        } else 
        {
          if ((rxbuf[rxcnt - 1] == 'm') &&
              (rxbuf[rxcnt - 2] == 'm'))  //串口接收距离数据格式ASCII码0mm~2000mm
          {
            rxflag = 1;
            Laser_dective();
            
          }
        }
      }
      if (rxcnt >= 16) {
        rxcnt = 0;
      }
    } else {
      rxempty = UART_D_REG(UARTN[LASER_PORT3]);  //读取接收到的数据
    }
  }


  }
