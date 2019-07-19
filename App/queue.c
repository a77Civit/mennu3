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

//初始化队列
void InitQueue(que * q)
{
    q->front = 0;
    q->rear = 0;
    q->count = 0;//可用于判断队空、队满
    
}
 
//判断队满
int FullQueue(que * q)
{
    //队列长度为 ARR_SIZE
    if(q->count < ARR_SIZE)
        return 0;
    return 1;
}
 
//入队函数（加到最后一个）
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
 



// float队列

//初始化队列
void InitQueuef(quef * q)
{
    q->front = 0;
    q->rear = 0;
    q->count = 0;//可用于判断队空、队满
    
}
 
//判断队满
int FullQueuef(quef * q)
{
    //队列长度为 ARR_SIZE
    if(q->count < ARR_SIZE)
        return 0;
    return 1;
}
 
//入队函数（加到最后一个）
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
  //队列不满则继续
  if (!FullQueuef(q)) {
    return 0;
  }
  
  while(count < L){
    s = s + 0.1*q->data[(q->front+count)%L];
    count++;
  }
  return s;
}