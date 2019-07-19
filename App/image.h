#ifndef IMAGE_H_
#define IMAGE_H_

#include <math.h>
#include <stdio.h>
#include "common.h"

#define Black_Point 0
#define White_Point 255

#define Trans 

extern uint8 imgbuff1[CAMERA_SIZE]; // 
extern uint8 imgbuff2[CAMERA_SIZE]; // 
extern uint8 *imgbuffdeal;
extern uint8 *imgbuffback;
extern uint8 img[CAMERA_H][CAMERA_W];  // 声明图像解压数组为外部变量
extern int road_info[4][CAMERA_H];     // 声明道路信息为外部变量
extern int Image_Flag;

extern float Fast;
extern float Slow;
extern int LossStart;    // 判断丢线起始行
extern int CircleSen;    // 判断环的电磁敏感度
extern int StartLine;
extern int EndLine;
extern int EndCount;

// 现在用到的函数
void camera_img_deal(uint8 *imgbuff);        // 获取图像并解压图像

int check_time(void);              // 更新时间
int dealLine(uint8 *line);         // 提取行中线
int curb(uint8 *line, int *road);  // 提取行中线和边线
int deal_image(void);  // 获取处理图像, 提取道路信息, 更新时间, 显示图像
int show_center_line(int *center, uint16_t rgb565);  // 显示存在全局路况中的中线
int getroadinfo(void);
int getcentralline(void);
int getboundryline(void);
int getboundrypoint(int L, int *B, int Start);
int searchboundry(int Line, int Lastj);
int showline(int *Linex, int *Liney, int size, uint16_t rgb565);
int showboundry(int *Line, int size, uint16_t rgb565);
float curvature(int u1,int v1,float u2,float v2);
float dealcorner(void);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
int Transform(void);            // 坐标变换生成常量表
int iscircle(void);             // 判断环岛
int intocircle(int direction);             // 入环获取边界（补线）
int incircle(int direction);       // 环内寻线
int incircleb(int x,int y);        // 判断环内是否补线
int outcircle(int direction);            // 出环补线
float SpeedCalculate(void);
#endif
