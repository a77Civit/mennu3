/*
 * File:    queue.h
 * Purpose: Implement a first in, first out linked list
 *
 * Notes:
 */

#ifndef __QUEUE_H_
#define __QUEUE_H_

typedef struct queue_arr
{
    int data[10];
    int front;
    int rear;
    int count;
} que;

typedef struct queuef_arr
{
    float data[10];
    int front;
    int rear;
    int count;
} quef;

void InitQueue(que * q);               //初始化队列
int FullQueue(que * q);                //判断队满
void EnQueue(que * q, int data);       //入队函数
int qmean(que * q);                    //计算队列加权均值
void InitQueuef(quef * q);               //初始化队列
int FullQueuef(quef * q);                //判断队满
void EnQueuef(quef * q, float data);       //入队函数
float qmeanf(quef * q);                    //计算队列加权均值


#endif