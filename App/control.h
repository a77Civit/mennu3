#ifndef CONTROL_H_
#define CONTROL_H_

#include "common.h"

int interupt_init(void);           // 中断配置总函数
void PORTA_IRQHandler(void);       // 摄像头中断
void DMA0_IRQHandler(void);        // 摄像头中断
void PIT0_IRQHandler(void);        // 编码器中断
void PIT0_IRQHandler_sport(void);  // 运动模式编码器中断
//void PIT2_IRQHandler(void);       // 电磁传感器中断
void PIT1_hander(void);            // 按键定时查询
void ADCReceive(void);             // ADC采集
void AngleControl(void);           // 直立环
void SpeedControl(void);           // 速度环
void SpeedControlOutPut(void);     // 速度环输出
void DirectionControl(void);       // 方向环
void DirectionControlOutPut(void); // 方向环输出
void DealDeath(void);              // 死区控制
void PIT2_hander(void);
void Changepara(int model);        // 改变参数（姿态变化）
int isblock(void);                 // 判断障碍
float avoidblock(int direction, int yset);              // 横断壁障（输出方向控制）
float dealangle(float angle);
void Transition(int Model);        // 变换过渡
int isramp(void);                  // 坡道判断

extern float GYROZero_x;
extern float GYROZero_y;
extern int ELEC[4];
extern int ACC[3];
extern int GYRO[3];
extern float Gyro_y;   
extern float Angle;
extern float SpeedSet;
extern int ModelFlag;
extern int CircleState;





#endif
