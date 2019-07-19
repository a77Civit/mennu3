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

void InitQueue(que * q);               //��ʼ������
int FullQueue(que * q);                //�ж϶���
void EnQueue(que * q, int data);       //��Ӻ���
int qmean(que * q);                    //������м�Ȩ��ֵ
void InitQueuef(quef * q);               //��ʼ������
int FullQueuef(quef * q);                //�ж϶���
void EnQueuef(quef * q, float data);       //��Ӻ���
float qmeanf(quef * q);                    //������м�Ȩ��ֵ


#endif