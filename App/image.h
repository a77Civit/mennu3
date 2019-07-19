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
extern uint8 img[CAMERA_H][CAMERA_W];  // ����ͼ���ѹ����Ϊ�ⲿ����
extern int road_info[4][CAMERA_H];     // ������·��ϢΪ�ⲿ����
extern int Image_Flag;

extern float Fast;
extern float Slow;
extern int LossStart;    // �ж϶�����ʼ��
extern int CircleSen;    // �жϻ��ĵ�����ж�
extern int StartLine;
extern int EndLine;
extern int EndCount;

// �����õ��ĺ���
void camera_img_deal(uint8 *imgbuff);        // ��ȡͼ�񲢽�ѹͼ��

int check_time(void);              // ����ʱ��
int dealLine(uint8 *line);         // ��ȡ������
int curb(uint8 *line, int *road);  // ��ȡ�����ߺͱ���
int deal_image(void);  // ��ȡ����ͼ��, ��ȡ��·��Ϣ, ����ʱ��, ��ʾͼ��
int show_center_line(int *center, uint16_t rgb565);  // ��ʾ����ȫ��·���е�����
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
int Transform(void);            // ����任���ɳ�����
int iscircle(void);             // �жϻ���
int intocircle(int direction);             // �뻷��ȡ�߽磨���ߣ�
int incircle(int direction);       // ����Ѱ��
int incircleb(int x,int y);        // �жϻ����Ƿ���
int outcircle(int direction);            // ��������
float SpeedCalculate(void);
#endif
