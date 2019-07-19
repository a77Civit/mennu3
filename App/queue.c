/*
 * File:    queue.c
 * Purpose: Implement a first in, first out linked list
 *
 * Notes:   
 */


#include <stdio.h>
#include <stdlib.h>
#include "queue.h"
#include "common.h"
#include "include.h"


int ARR_SIZE = LEN;

//��ʼ������
void InitQueue(que * q)
{
    q->front = 0;
    q->rear = 0;
    q->count = 0;//�������ж϶ӿա�����
    
}
 
//�ж϶���
int FullQueue(que * q)
{
    //���г���Ϊ ARR_SIZE
    if(q->count < ARR_SIZE)
        return 0;
    return 1;
}
 
//��Ӻ������ӵ����һ����
void EnQueue(que * q, int data)
{
    if(FullQueue(q))
    {
        q->front = (q->front+1)%ARR_SIZE;
        q->data[q->rear] = data;
        q->rear = (q->rear+1)%ARR_SIZE;
    }
    else
    {
        q->data[q->rear] = data;
        q->rear = (q->rear+1)%ARR_SIZE;
        q->count++;
    }
}
 



// float����

//��ʼ������
void InitQueuef(quef * q)
{
    q->front = 0;
    q->rear = 0;
    q->count = 0;//�������ж϶ӿա�����
    
}
 
//�ж϶���
int FullQueuef(quef * q)
{
    //���г���Ϊ ARR_SIZE
    if(q->count < ARR_SIZE)
        return 0;
    return 1;
}
 
//��Ӻ������ӵ����һ����
void EnQueuef(quef * q, float data)
{
    if(FullQueuef(q))
    {
        q->front = (q->front+1)%ARR_SIZE;
        q->data[q->rear] = data;
        q->rear = (q->rear+1)%ARR_SIZE;
    }
    else
    {
        q->data[q->rear] = data;
        q->rear = (q->rear+1)%ARR_SIZE;
        q->count++;
    }
}

float qmeanf(quef *q){
  int count=0 , L=LEN;
  float s=0;
  //���в��������
  if (!FullQueuef(q)) {
    return 0;
  }
  
  while(count < L){
    s = s + 0.1*q->data[(q->front+count)%L];
    count++;
  }
  return s;
}