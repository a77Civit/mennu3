
/**
 * name:       electromagnetism.c
 * usage:      --
 * author:     [[
 * date:       2019-03-10 14:00:00
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"

que elec_que1;
que elec_que2;
que elec_que3;
que elec_que4;


//加权系数
float a[10]={0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};


/**
 * 电磁传感器初始化
 */
void elec_init(void){
  //adc初始化
  adc_init(ADC1_SE4a);
  adc_init(ADC1_SE5a);
  adc_init(ADC1_SE6a);
  adc_init(ADC1_SE7a);
  
  //队列初始化
  InitQueue(&elec_que1);
  InitQueue(&elec_que2);
  InitQueue(&elec_que3);
  InitQueue(&elec_que4);
  
}

/**
 * 更新电磁队列
 */
void elec_renew(void){
  int i;
  int num=10;
  //读取电磁信息并加入队列
    //EnQueue(&elec_que1,adc_once(ADC1_SE4a, ADC_10bit));
    //EnQueue(&elec_que2,adc_once(ADC1_SE5a, ADC_10bit));
    //EnQueue(&elec_que3,adc_once(ADC1_SE6a, ADC_10bit));
    //EnQueue(&elec_que4,adc_once(ADC1_SE7a, ADC_10bit));

    //ELEC[0] = (int) (elec_deal(&elec_que1));
    //ELEC[1] = (int) (elec_deal(&elec_que2));
    //ELEC[2] = (int) (elec_deal(&elec_que3));
    //ELEC[3] = (int) (elec_deal(&elec_que4));
  
  ELEC[0]=0;
  ELEC[1]=0;
  ELEC[2]=0;
  ELEC[3]=0;
  
  for (i=0;i<num;i++){
    ELEC[0]+=adc_once(ADC1_SE4a, ADC_10bit);
    ELEC[1]+=adc_once(ADC1_SE5a, ADC_10bit);
    ELEC[2]+=adc_once(ADC1_SE6a, ADC_10bit);
    ELEC[3]+=adc_once(ADC1_SE7a, ADC_10bit);
  }
  
  ELEC[0] = ELEC[0]/num;
  ELEC[1] = ELEC[1]/num;
  ELEC[2] = ELEC[2]/num;
  ELEC[3] = ELEC[3]/num;
  
}

     

/**
 * 电磁滤波
 */
float elec_deal(que *q){
  int count=0 , L=LEN;
  float s=0;
  //队列不满则继续
  if (!FullQueue(q)) {
    return 0;
  }
  
  while(count < L){
    s = s + a[count]*q->data[(q->front+count)%L];
    count++;
  }
  return s;
}