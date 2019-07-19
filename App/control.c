
/**
 * name:       control.c
 * usage:      --
 * author:     [[
 * date:       2018-07-14 09:09:07
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"
#include "math.h"

#if PID_TUNER
uint16 speed_record[2][200];
#endif


int RampFlag = 0;       // 判断坡道
int RampCount = 0;      
int DisRamp = 0;

int JudgeFlag = 0;      // 判断到断路标记
int CompleteFlag = 0;   // 标记变形是否完成
int TransFlag = 0;      // 标记改断路是否变形
int TransPeriod = 20;   // 变换过渡周期  20*5 = 100ms
int TransCount = 0;
float Angle3 = 23;          // 三轮状态陀螺仪数值
float Angle_Trans = 0;     // 变换时刻陀螺仪初始数值

float UpFlag = 1;
uint32 T0up = 0; // 抬头时刻（不判断环）
float corner = 0;
int BuzzCount = 0;
float theta = 0;
float P_Angle = 0.8;
float D_Angle = 0.2;
float Speed2 = 350;
float Speed3 = 350;
int SpeedTemp[3] = {0,0,0};
int BrokenSensitive = 5;
int Blocklength = 0;

int ModelFlag = 1; // 1：直立模式    0：三轮模式
int BlockFlag = 0;
int BlockCount = 0;
int BrokenFlag = 0; // 断路标记
int ChangeFlag = 0; // 断路内标记
int CircleFlag = 1; // 标记左环还是右环    1：左环    2：右环
int CircleState = 0; // 标记入环状态    0：正常模式    1：入环模式     2：环内模式   3：出环
uint32 T0Circle = 0; // 入环时刻
uint32 T1Circle = 0; // 出环时刻
uint32 T0Broken = 0; // 断路判断时刻
float KELEC,KCAM;    // 电磁、摄像头比重
float AngleCircle = 0; // 陀螺仪出环记角度
float AngleCircleSet = 300; // 陀螺仪出环值
float AngleBlock = 0;    // 壁障角度
float AngleBlock0 = 0;   // 记录壁障阶段一的转角
float DisBlockx = 0;     // 横向位移量
float DisBlocky = 0;     // 纵向位移量
float DisBroken = 0;

//直立环的PID参数
float k1 = 8;
float k2 = 0.45;
float ksp = 0.3;

float ksi = 0;     
float maxspeederr = 200;
//float kp_speed_3 = 0.9;
//float ki_speed_3 = 0.4;
float kp_speed_3 = 0.87;
float ki_speed_3 = 0.35;
float maxspeed = 40;
int globalcount=0;

float ACCEL_LSB  = 180/(3.14*16384);      // 加速度量程为正负2g，16384/9.8=1671.8367   9.8/16384=0.000598
float GYRO_LSB = 1/32.8;      // 陀螺仪量程为正负1000    1/32.8

float ACCZero = 0;              // 加速度零点
float GYROZero_x = 0;              // 角速度零点
float GYROZero_y = 0;              // 角速度零点

int PITCOUNT = -1;           // 循环计数器
int ELEC[4] = {0,0,0,0};    // 电磁数据
int ACC[3] = {0,0,0};       // 加速度
int GYRO[3] = {0,0,0};      // 角速度

float AngleControlOut;      // 直立环输出
float LeftVal;          // 左电压
float RightVal;         // 右电压

quef Gyro_yy;        //角速度队列
float Gyro_y;        //Y轴陀螺仪角速度
float Angle_gy;      //由角速度计算的倾斜角度
float Angle_az;      //由加速度计算的倾斜角度
float Angle = 0;         //小车最终倾斜角度
float Angle1 = 0;

int LeftSpeed;     // 左轮速度
int RightSpeed;    // 右轮速度
float CurrentSpeederr = 0;  // 最新速度
float Speederr = 0;         // 小车速度差
float SpeedErrorTemp[3] = {0,0,0};
float SpeedSet = 0;      // 设定速度
float errIntegral;      // 位移（速度积分）
float MaxIntegral;   // 积分上限
float SpeedControlOut;  // 速度环输出
float SpeedControlOutOld = 0; // 上一次输出
float SpeedControlOutNew = 0; // 这一次输出
int SpeedControlCount = 0;  // 速度控制周期0.1s
int SpeedPeriod = 20;
int RotationCount = 0;
int RotationPeriod = 10;

int RazerCount = 0;         // 激光计数
int RazerPeriod = 4;       // 激光周期

float Turn_Offset;
float DirectionControlOutNew = 0;
float DirectionControlOutOld = 0;
float DirectionControlOut;
int DirectionControlCount = 0;
int DirectionPeriod = 4;
float P_ELEC = 0.8;
float D_ELEC = 0.36;
float P_ELEC3 = 0.8;
float D_ELEC3 = 0.25;
float P_CAM = 7;
float D_CAM = 0.21;
float Gyro_x;        //X轴陀螺仪角速度
quef Gyro_xx;        //角速度队列


float LeftDeadVal = 0;
float RightDeadVal = 0;
float MotoMax = 70;

/**
 * 配置中断
 */
int interupt_init(void) {
  
  // 配置摄像头中断函数
  // 设置 PORTA 的中断服务函数为 PORTA_IRQHandler
  set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);
  // 设置 DMA0 的中断服务函数为 DMA0_DMA16_VECTORn
  set_vector_handler(DMA0_DMA16_VECTORn, DMA0_IRQHandler);
  
  // FTM1 FTM2 正交解码初始化
  // 所用的管脚可查 port_cfg.h 的 FTM1_QDPHA_PIN 和 FTM1_QDPHB_PIN
  ftm_quad_init(FTM1, FTM_PS_1, FTM1_QDPHA_PIN, FTM1_QDPHB_PIN);
  ftm_quad_init(FTM2, FTM_PS_1, FTM2_QDPHA_PIN, FTM2_QDPHB_PIN);
  // 初始化PIT0，正交解码定时时间为： 50ms
  pit_init_ms(PIT0, 5);
  InitQueuef(&Gyro_yy);

  // 设置 PIT0 的中断服务函数为 PIT0_IRQHandler, 控制编码器定时和电机
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  
  // 设置UART3，4中断服务函数
  set_vector_handler(UART4_RX_TX_VECTORn, Laser_handler2);
  set_vector_handler(UART3_RX_TX_VECTORn, Laser_handler3);
  uart_init(LASER_PORT2, 9600);  // UART3 激光波特率
  uart_init(LASER_PORT3, 9600);   // UART4 激光波特率
  
  
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);           
    NVIC_SetPriority(PORTA_IRQn,1);         //配置优先级
    NVIC_SetPriority(DMA0_DMA16_IRQn,2);          //配置优先级
    NVIC_SetPriority(UART3_RX_TX_IRQn,1);      //配置优先级
    NVIC_SetPriority(UART4_RX_TX_IRQn,1);      //配置优先级
    NVIC_SetPriority(PIT0_IRQn,3);          //配置优先级
    //NVIC_SetPriority(PIT1_IRQn,4);          //配置优先级
    
    
  // 激光测距初始化1
  UART_C2_REG(UARTN[UART3]) |= UART_C2_RIE_MASK;  // 使能UART接收中断
  enable_irq((IRQn_t)(UART3_RX_TX_IRQn));         // 使能IRQ中断
  waitflag = 1;
  uart_sentn(LASER_PORT3, "s5-1#",5);  //设为主动发送被动读取模式
  while (waitflag == 1);
  waitflag = 0; 
  DELAY_MS(500);
  rxcnt = 0;
  
  // 激光测距初始化2
  UART_C2_REG(UARTN[UART4]) |= UART_C2_RIE_MASK;  // 使能UART接收中断
  enable_irq((IRQn_t)(UART4_RX_TX_IRQn));         // 使能IRQ中断
  waitflag = 1;
  uart_sentn(LASER_PORT2, "s5-1#",5);  //设为主动发送被动读取模式
  while (waitflag == 1);
  waitflag = 0; 
  DELAY_MS(500);
  rxcnt = 0;

 //   NVIC_SetPriority(PIT2_IRQn,4);          //配置优先级

  
      
 // pit_init_ms(PIT2, 50);                          // PIT2 定时 50ms
 // set_vector_handler(PIT2_VECTORn, PIT2_hander);  // 设置按键查询中断
  
  // 使能中断
  enable_irq(PIT0_IRQn);
                      
 // enable_irq(PIT2_IRQn);                         

  return 0;
}

/**
 * PORTA中断服务函数
 * input:
 *   global imbuff
 */
void PORTA_IRQHandler(void) {
  uint8 n;  // 引脚号
  uint32 flag;

  //while (!PORTA_ISFR) ;// 
   
  flag = PORTA_ISFR;
  PORTA_ISFR = ~0;  // 清中断标志位

  n = 29;               // 场中断
  if (flag & (1 << n))  // PTA29触发中断
  {
    camera_vsync();

  }

#if (CAMERA_USE_HREF == 1)  //使用行中断
  n = 28;
  if (flag & (1 << n))  // PTA28触发中断
  {
    camera_href();
  }
#endif
}

/**
 * DMA0中断服务函数
 */
void DMA0_IRQHandler(void) {

  camera_dma(); 

}

/**
 * 按键定时查询函数
 */
void PIT1_hander(void) {
  // 把按键扫描程序加入到定时中断服务函数里，定时执行
  key_IRQHandler();
  PIT_Flag_Clear(PIT1);
}


/**
 * PIT2中断服务函数
 * input:
 *   
 */
//void PIT2_hander(void) {


//}

/**
 * PIT0中断服务函数
 * input:
 *   global rotational_speed
 */
void PIT0_IRQHandler(void) {
  if(ISLCD_ON_FLAG)
  {
    //开液晶的话液晶程序放进if里就好，本质是全局条用ISLCDON_Flag
    //位置坐标可以不用初始化
  }
  //Site_t loc1 = {85, 5};
  //Site_t loc2 = {85, 25};
  //Site_t loc3 = {85, 45};
  //Site_t loc4 = {85, 65};
  //Site_t loc5 = {85, 85};
  //Site_t loc6 = {85, 105};
  //int a,b,c,d,e,f;


  // 清中断标志位
  PIT_Flag_Clear(PIT0);
  
  //pit_time_start  (PIT2);
  
  if (PITCOUNT<5)
    PITCOUNT++;
  else PITCOUNT = 0;
  
  BuzzCount++;
  if (BuzzCount == 100){
    BrokenFlag = 0;
    BuzzCount = 0;
    gpio_set (BUZZ, 0);
  }
  if (JudgeFlag == 1){
    timevar = pit_time_get_ms    (PIT2);
    if (timevar-T0Broken>5000){
      JudgeFlag = 0;
    }
  }

  // ADC采集数据
  ADCReceive();
  // 计算角度并输出控制值
  AngleControl();
  // 速度环控制速度
  SpeedControlOutPut();
  SpeedControlCount++;
  if (SpeedControlCount == SpeedPeriod){
    SpeedControl();
    SpeedControlCount = 0;
  }
  
  // 方向环控制差速
  DirectionControlOutPut();
  DirectionControlCount++;
  if (DirectionControlCount == DirectionPeriod){
    DirectionControl();
    DirectionControlCount = 0;
  }
  
  // 经过死区调整
  DealDeath();

  moto_power(LeftVal,RightVal);

  //a=ELEC[0];
  //a=(int)(AngleBlock*10/145*180);
  //b = (int)(SpeedControlOut*10);
  //b=(int)(Gyro_y*10);
  //b=150;
  //b=TransFlag*100;
  //b = (int)(SpeedSet);
  //b=(int)(DisBlockx);
  //c=(int)(DisBlocky);
  //b=real_length;
  //b=(int)(Angle*10);
  //b=LeftSpeed;
  //b=(int)(corner*100);
  //c=DisBlock;
  //c=(int)(Turn_Offset);
  //c = (int)(AngleControlOut*10);
  //c=(LeftSpeed+RightSpeed)/2;
  //c = (int)((LeftVal + RightVal)/2)*10;
  //c = TransFlag * 100;
  //c=(int)(Gyro_y);
  //c=ELEC[2];
  //c = JudgeFlag*100;
  //c = (int)(Angle + ACCZero)*10;
  //c = (int)(ACCZero*10);
  //c=(int)(SpeedControlOutOld*10);
  //c = SpeedSet;
  //d=ELEC[3];
  //d=(int)(DisBlocky+real_length);
  //d=(int)(globalcount);
  //d=(int)(corner*200);
  //d=(int)(theta*10/145*180);
  
  //e=(int)(Gyro_x*10);
  //f=DisBlocky;

    //lcd_snum_c(loc1, a, BLUE, RED);
    //lcd_snum_c(loc2, b, BLUE, RED);
    //lcd_snum_c(loc3, c, BLUE, RED);
    //lcd_snum_c(loc4, d, BLUE, RED);
    //lcd_snum_c(loc5, e, BLUE, RED);
    //lcd_snum_c(loc6, f, BLUE, RED);
  
}

int timevar;
pit_d
/**
 * ADC采集
 * input:
 *   
 */
void ADCReceive(void){
  //uint32 timevar;
  
  // 获取激光数据
  RazerCount++;
  if (RazerCount == RazerPeriod){
    if (ModelFlag == 1){
      Raser_Flag = 2;
    }else{
      Raser_Flag = 3;
    }
    RazerCount = 0;
    Site_t loc6 = {10, 95};
    lcd_snum_c(loc6, real_length, BLUE, RED);
  }

  // 获取陀螺仪数据
  if (TransFlag == 1){
    Transition(ModelFlag);
  }
  Gyro_y =  - MPU6050_get_y_gyro()*GYRO_LSB - GYROZero_y;  // 111us
  Angle_az = -MPU6050_get_z_accel()*ACCEL_LSB - ACCZero;  // 直立状态
  EnQueuef(&Gyro_yy,Gyro_y);
  Gyro_y = qmeanf(&Gyro_yy);
  Gyro_x = -MPU6050_get_x_gyro()*GYRO_LSB - GYROZero_x;  
  EnQueuef(&Gyro_xx,Gyro_x);

  // 获取编码器数据
  RotationCount++;
if (RotationCount == RotationPeriod){
  RotationCount = 0;
  // 获取FTM 正交解码 的脉冲数(负数表示反方向)
  LeftSpeed = ftm_quad_get(FTM1);
  ftm_quad_clean(FTM1);

  // 获取FTM 正交解码 的脉冲数(负数表示反方向)
  RightSpeed = -ftm_quad_get(FTM2);
  ftm_quad_clean(FTM2);
  SpeedTemp[2] = SpeedTemp[1];
  SpeedTemp[1] = SpeedTemp[0];
  SpeedTemp[0] = (LeftSpeed+RightSpeed)/2;
}
}

/**
 * 直立环控制
 * input:
 *   
 */
void AngleControl(void){
  
  Kalman_Filter(Angle_az,Gyro_y);       //卡尔曼滤波计算倾角
  //Site_t loc5 = {10, 75};
  //lcd_snum_c(loc5, (int)(Angle*10), BLUE, RED);
  AngleControlOut = k1*Angle + k2*Gyro_y;
  
}


/**
 * 变换过渡
 * input:
 *   
 */
void Transition(int Model){
  if (Model == 0){
    TransCount ++;
  }else{
    TransCount += 20;
  }
  
  if (Model == 0){         
    ACCZero = (Angle3 - Angle_Trans)*TransCount/TransPeriod + Angle_Trans;   // 二轮切三轮
  }else{
    ACCZero = (0 - Angle3)*TransCount/TransPeriod + Angle3;          // 三轮切二轮
  }
  
  if (TransCount>=TransPeriod){
    TransFlag = 0;
    TransCount = 0;
    if (ModelFlag == 0){
      SpeedControlCount = 0;
      SpeedControl();
    }else{
      //SpeedSet = (LeftSpeed+RightSpeed)/3;
      SpeedSet = 200;
    }
      
  }
      
}

/**
 * 速度环控制
 * input:
 *   
 */
void SpeedControl(void){

  CurrentSpeederr = SpeedSet - (LeftSpeed + RightSpeed)*0.5 ;
  if (CurrentSpeederr>0){
    CurrentSpeederr = (CurrentSpeederr>maxspeederr?maxspeederr:CurrentSpeederr);//速度差限幅
  }else{
    CurrentSpeederr = (CurrentSpeederr<-maxspeederr?-maxspeederr:CurrentSpeederr);//速度差限幅
  }
  
  SpeedErrorTemp[2] = SpeedErrorTemp[1];
  SpeedErrorTemp[1] = SpeedErrorTemp[0];
  SpeedErrorTemp[0] = CurrentSpeederr;
  //Speederr = Speederr*0.8 + CurrentSpeederr*0.2;
  //errIntegral += Speederr*0.005;
  
  if (ModelFlag){
    errIntegral += CurrentSpeederr*0.1;
    SpeedControlOutOld = SpeedControlOutNew;
    SpeedControlOutNew = ksp*CurrentSpeederr + ksi*errIntegral;  
  }else{
    SpeedControlOutOld = SpeedControlOutNew;
    SpeedControlOutNew += kp_speed_3*0.05*(SpeedErrorTemp[0]-SpeedErrorTemp[1]) + 0.05*ki_speed_3*CurrentSpeederr;
    //限幅输出
    if (SpeedControlOutNew-SpeedControlOutOld>=0){
      SpeedControlOutNew = ((SpeedControlOutNew-SpeedControlOutOld)>=maxspeed?(SpeedControlOutOld+maxspeed):SpeedControlOutNew);
    }else{
      SpeedControlOutNew = ((SpeedControlOutNew-SpeedControlOutOld)<=-maxspeed?(SpeedControlOutOld-maxspeed):SpeedControlOutNew);
    }
  }
}

/**
 * 速度环输出
 * input:
 *   周期：0.1s
 */
void SpeedControlOutPut(void){
  float fvalue;
  fvalue = SpeedControlOutNew - SpeedControlOutOld;
  SpeedControlOut = fvalue*(SpeedControlCount+1)/SpeedPeriod + SpeedControlOutOld;
  
}

/**
 * 方向环控制
 * input:
 *   
 */
void DirectionControl(void){
  uint32 timevar;
  //float corner = 0;
  int cornerflag=1;   // 若conerflag变为0 则通过电磁寻线

    elec_renew();     // 更新电磁数据  115us
    Gyro_x = qmeanf(&Gyro_xx); // 计算x轴角速度
    
    // 解压图像
    img_extract((uint8 *)img, imgbuffdeal, CAMERA_SIZE); // 60*80 247us     120*180 920us

      //Site_t site = {0, 0};                   // 显示图像左上角位置
      //Size_t imgsize = {CAMERA_W, CAMERA_H};  // 图像大小
      //Size_t size = {80, 60};                 // 显示区域图像大小
      // Debug 小液晶显示图像
      //lcd_img_binary_z(site, size, imgbuffdeal, imgsize, BLACK, WHITE);  // 5215us

    if (ChangeFlag==0 && BlockFlag==0){      
    // 出环过渡（期间不判断环）
    if (CircleState == 3){
      led (LED2,LED_ON);
      if (outcircle(CircleFlag)==0){
      //if (getboundryline()==0){
        cornerflag = 0;
      }
      timevar =  pit_time_get_ms (PIT2);
      if (timevar-T1Circle>2000){
        CircleState = 0;                  // 两秒过后恢复正常状态
      }
      if (EndCount < BrokenSensitive || cornerflag == 0){
        KELEC = 1;
        KCAM = 0;
        if (Angle + ACCZero>-1){
          BrokenFlag++;
        }
        //SpeedSet = 50;
      }else{
        KELEC = 0;
        KCAM = 1;
        //SpeedSet = SpeedCalculate();
      }
    }

    // 环内模式补线
    if (CircleState == 2){
      led (LED2,LED_OFF);
      AngleCircle += Gyro_x*0.02;
      if (AngleCircle > AngleCircleSet*3/5 || AngleCircle < -AngleCircleSet*3/5){
        if (incircle(CircleFlag)==0){
          cornerflag = 0;
        }
      }else{
        if (getboundryline()==0){
          cornerflag = 0;
        }
      }
      if (AngleCircle > AngleCircleSet || AngleCircle < -AngleCircleSet){
        gpio_set (BUZZ, 1);
        CircleState = 3;
        T1Circle = pit_time_get_ms (PIT2);
      }
      
      KELEC = 0.1;
      KCAM = 1;
    }
    
    // 入环模式补线
    if (CircleState == 1){
      led (LED2,LED_ON);
      if (intocircle(CircleFlag)==0){
        cornerflag = 0;
      }
      timevar =  pit_time_get_ms (PIT2);
      AngleCircle += Gyro_x*0.02;
      //if (timevar-T0Circle>800){
      if (AngleCircle > AngleCircleSet/4 || AngleCircle < -AngleCircleSet/4){
        CircleState = 2;           // 转为环内模式
      }
      
      KELEC = 0.1;
      KCAM = 1;
    }
    
    // 正常模式摄像头寻线
    if (CircleState == 0){
      led (LED2,LED_OFF);
      if (getboundryline()==0){
        cornerflag = 0;
      }
      if (iscircle() == 1){        // 识别环
        led (LED2,LED_ON);
        gpio_set (BUZZ, 1);
        CircleFlag = 1;            // 判断左右环
        CircleState = 1;           // 开始入环模式
        AngleCircle = 0;           // 判断环角度清零
        T0Circle = pit_time_get_ms (PIT2);   // 开始计时
      }
      if (iscircle() == 2){
        led (LED2,LED_ON);
        gpio_set (BUZZ, 1);
        CircleFlag = 2;
        CircleState = 1;
        AngleCircle = 0;           // 判断环角度清零
        T0Circle = pit_time_get_ms (PIT2);
      }
      // 摄像头寻线失败则用电磁
      if (EndCount < BrokenSensitive || cornerflag == 0){
        KELEC = 1;
        KCAM = 0;
        if (Angle + ACCZero>-1){
          BrokenFlag++;
        }
        //SpeedSet = 50;
      }else{
        KELEC = 0.1;
        KCAM = 1;
        //SpeedSet = SpeedCalculate();
      }
    }
    
    if (cornerflag){
      getcentralline();
      corner = -dealcorner();
    }
    
    }

    
    // 特殊路段判断 断路和障碍
    // 认定环内不可能出现断路或障碍
    if (CircleState == 0 || CircleState == 3){
      if (ChangeFlag){
        DisBroken += (LeftSpeed+RightSpeed)/2*0.176/2.5;
        timevar = pit_time_get_ms    (PIT2);
        
        if (DisBroken >100 && CompleteFlag == 0){
          ModelFlag = (ModelFlag==1?0:1);
          Changepara(ModelFlag);       // 切换形态改变参数
          Angle_Trans = Angle;
          if (ModelFlag == 0){
            SpeedControlOutNew = (LeftVal+RightVal)/3; 
            SpeedControlOutOld = (LeftVal+RightVal)/3;
            SpeedControlOut = 0;
          }else{
            SpeedControlOutNew = 0; 
            SpeedControlOutOld = 0;
            SpeedControlOut = 0;
          }
          SpeedControlCount = 0;
          TransFlag = 1;
          CompleteFlag = 1;
        }
        
        
        //if (timevar-T0Broken>2000){
        // 断路距离
        if (DisBroken >2000){
          DisBroken = 0;
          TransFlag = 0;
          ChangeFlag=0;
          BrokenFlag = 0;
          CompleteFlag = 0;
          SpeedSet =  SpeedCalculate();
        }
        KELEC = 1;
        KCAM = 0;
      }
      
      // 判断断路
      if (ChangeFlag==0 && BlockFlag==0 && JudgeFlag == 0 && RampFlag == 0){
        if (BrokenFlag>6){
          gpio_set (BUZZ, 1);
          led_turn (LED1);
          T0Broken = pit_time_get_ms    (PIT2);
          DisBroken = 0;
          //ChangeFlag = 1;
          JudgeFlag = 1;
          KELEC = 1;
          KCAM = 0;
          //SpeedSet = 200;
          if (ModelFlag == 0){
            SpeedSet = 150;
          }
        }
      }
      
      
      // 判断障碍
      if (BlockFlag == 0 && ChangeFlag == 0 && RampFlag == 0){
        if (isblock()){
          BlockCount++;
        }else{
          BlockCount = 0;
        }
        if (BlockCount>4){
          Blocklength = real_length;
          BlockFlag = 1;
          gpio_set (BUZZ, 1);
          BlockCount = 0;
          DisBlocky = 0;
          DisBlockx = 0;
          AngleBlock = 0;
        }
      }
      if (BlockFlag>0){
        SpeedSet = 150;
        // 计算转角
        AngleBlock += -Gyro_x*0.02;
        // 计算横向以及纵向位移量
        DisBlocky += (LeftSpeed+RightSpeed)/2*cos(AngleBlock/145*3.14)*0.176/2.5;
        DisBlockx += (LeftSpeed+RightSpeed)/2*sin(AngleBlock/145*3.14)*0.176/2.5;   // 左正右负
        DirectionControlOutOld = DirectionControlOutNew;
        DirectionControlOutNew = avoidblock(2,Blocklength);     // 避障模式估计转角
        return ;
      }
      
      /*
      // 判断坡道
      if (BlockFlag == 0 && ChangeFlag == 0 && RampFlag == 0){
        if (isramp()){
          gpio_set (BUZZ, 1);
          RampFlag = 1;
          DisRamp = 0;
          ACCZero = Angle3 - 14;
          kp_speed_3 = 0.9;
          ki_speed_3 = 0.01;
          k1 = 0;
        }
      }
      if (RampFlag == 1){
        DisRamp += (LeftSpeed+RightSpeed)/2*0.176/2.5;
        if (DisRamp > 1000){
          RampFlag = 0;
          DisRamp = 0;
          ACCZero = Angle3;
          kp_speed_3 = 0.87;
          ki_speed_3 = 0.35;
          k1 = 8;
        }
      }
      */
      
      
        
      
    }
    
    /**
    if (real_length<1600){
      //if (Gyro_x<20 && Gyro_x>-20){
        SpeedSet = 100;
      //}
    }else{
      SpeedSet = Speed3;
    }
    */
    
  
  if (Angle + ACCZero < 12){
    Turn_Offset = ((float)(ELEC[0] - ELEC[3])/(float)(ELEC[0] + ELEC[3]+0.01))*100;
    //P_ELEC = 0.475;
    P_ELEC = 0.75;
    D_ELEC = 0.18;
  }else{
    Turn_Offset = ((float)(ELEC[0] - ELEC[3])/(float)(500))*100;
    Turn_Offset = Turn_Offset>100?100:Turn_Offset;
    Turn_Offset = Turn_Offset<-100?-100:Turn_Offset;
    //Turn_Offset = ((float)(ELEC[0] - ELEC[3])/(float)(ELEC[0] + ELEC[3]+0.01))*100;
    P_ELEC = 0.55;
    D_ELEC = 0.2;
  }
  //Turn_Offset = Turn_Offset*(Turn_Offset*Turn_Offset/2500.0+2)/6;
  //Turn_Offset = Turn_Offset*(Turn_Offset*Turn_Offset+1)*50;
  DirectionControlOutOld = DirectionControlOutNew;
  
  DirectionControlOutNew = KCAM*(P_CAM*corner*200+D_CAM*(Gyro_x))
                          +KELEC*(P_ELEC*Turn_Offset + D_ELEC*(Gyro_x));
  
  
}

/**
 * 参数改变（姿态变换）
 * input:
 *   周期：0.025s
 */
void Changepara(int model){
  // 三轮
  if (model==0){
    maxspeederr = 250;
    ACCZero = 0;              // 加速度零点
    //P_ELEC = 0.45;
    //D_ELEC = 0.2;
    //P_ELEC = 0.295;
    //D_ELEC = 0.15;
    P_CAM = 3;
    D_CAM = 0.2;
    StartLine = 59;
    EndLine = 5;
    LossStart = 40;  // 判断丢线起始行
    CircleSen = 500;       // 判断环的电磁敏感度
    LeftDeadVal = 8;
    RightDeadVal = 8;
    SpeedPeriod = 10;
    AngleCircleSet = 290;
    SpeedSet = Speed3;
    BrokenSensitive = 20;
    ACCZero = Angle3;
  }
  // 二轮
  if (model==1){  
    maxspeederr = 300;
    ACCZero = 0;              // 加速度零点
    //P_ELEC = 0.475;
    //D_ELEC = 0.175;
    //P_ELEC = 0.6;
    //D_ELEC = 0.22;
    
    P_CAM = 5;
    D_CAM = 0.21;
    StartLine = 59;
    EndLine = 15;
    LossStart = 48;  // 判断丢线起始行
    CircleSen = 450;       // 判断环的电磁敏感度
    LeftDeadVal = 0;
    RightDeadVal = 0;
    SpeedPeriod = 20;
    AngleCircleSet = 330;
    SpeedSet = Speed2;
    BrokenSensitive = 10;
    ACCZero = 0;
  }
}
  
  


/**
 * 方向环输出
 * input:
 *   周期：0.025s
 */
void DirectionControlOutPut(void){
  float fvalue;
  fvalue = DirectionControlOutNew - DirectionControlOutOld;
  DirectionControlOut = fvalue*(DirectionControlCount+1)/DirectionPeriod + DirectionControlOutOld;
  
}

/**
* 死区控制
 * input:
 *   global 
 */
void DealDeath(void){
  if (TransFlag){
    LeftVal = AngleControlOut;
    RightVal = AngleControlOut;
  }else{
    if (ModelFlag){
      LeftVal = AngleControlOut - SpeedControlOut; //- DirectionControlOut;
      RightVal = AngleControlOut - SpeedControlOut; //+ DirectionControlOut;
    }else{
      LeftVal = AngleControlOut + SpeedControlOut; //- DirectionControlOut;
      RightVal = AngleControlOut + SpeedControlOut; //+ DirectionControlOut;
    }
  }
  
  if (LeftVal > 0)
    LeftVal += LeftDeadVal;
  else if (LeftVal < 0)
    LeftVal -= LeftDeadVal;
  
  if (RightVal > 0)
    RightVal += RightDeadVal;
  else if (RightVal < 0)
    RightVal -= RightDeadVal;
  
  if (LeftVal > MotoMax)
    LeftVal = MotoMax;
  if (LeftVal < -MotoMax)
    LeftVal = -MotoMax;
  if (RightVal > MotoMax)
    RightVal = MotoMax;
  if (RightVal < -MotoMax)
    RightVal = -MotoMax;
  
  LeftVal -= DirectionControlOut;
  RightVal += DirectionControlOut;
  
}

/**
* 判断坡道
 * input:
 *   global 
 */
int isramp(void){
  if (ModelFlag == 0){
    if (EndCount < 50){
      return 0;
    }
    if (corner*200>1 || corner*200<-1){
      return 0;
    }
    
    if (SpeedTemp[0] - SpeedTemp[2] < -150 && Angle < -5){
      return 1;
    }
  }
  if (ModelFlag == 1){
    if (SpeedTemp[0] - SpeedTemp[2] < -150){
      return 1;
    }
  }
  return 0;
}
    

/**
* 判断坡道
 * input:
 *   global 
 */
int onramp(void){
  
}
  

    

/**
* 判断障碍
 * input:
 *   global 
 */
int isblock(void){
  
  //if (real_length>1200){return 0;}   // 400转
  if (real_length>1250){return 0;}
  if (ModelFlag == 1){
    if (real_length<700){
      return 0;
    }
    if (Angle + ACCZero > 13){
      return 0;
    }
    if (EndCount > 40){
      //return 0;
    }
    
  }else{
    if (EndCount > 50){
      return 0;
    }
  }
  //if (real_length<700){return 0;}
  //if (real_length>700){return 0;}
  if (ChangeFlag){return 0;}
  //if (corner*200>1.2 || corner*200<-1.2){return 0;}
  if (Gyro_x>20 || Gyro_x<-20){return 0;}
  //if (Gyro_y>20 || Gyro_y<-20){return 0;}
  
  
  return 1;
}



/**
* 横断避障（输出方向控制）
 * input:   1：向左   2：向右
 *   global 
 */
float avoidblock(int direction, int yset){
  float xset = 380;      // 横向距离 (mm)
  //float yset = 900;     // 纵向距离 (mm)  400转
  //float yset = 1000;  // 纵向距离 (mm)  100转
  float turncorner = 65; // 固定打角

  if (BlockFlag == 1){
      if (DisBlocky>yset+150){
        BlockFlag = 3;
      }
      if (DisBlockx>xset || DisBlockx<-xset){
        BlockFlag = 2;
      }
    if (direction == 1){
      return dealangle(turncorner);
    }else{
      return dealangle(-turncorner);
    }
  }
  
  if (BlockFlag == 2){
    if (DisBlocky>yset+50){
      BlockFlag = 3;
    }
    return dealangle(0);
  }
  
  if (BlockFlag == 3){
    if (DisBlockx<150 && DisBlockx>-150){
      BlockFlag = 0;
      AngleBlock = 0;
      SpeedSet = SpeedCalculate();
      return 0;
    }
    
    if (direction == 1){
      return dealangle(-turncorner);
    }else{
      return dealangle(turncorner);
    }
    
    //theta = atan(DisBlockx/(2000-DisBlocky))/3.14*145;
    /**
    if (direction == 1){
      if (AngleBlock>-theta){
        return -turncorner;
      }else{
        return 0;
      }
    }
    if (direction == 2){
      if (AngleBlock<-theta){
        return turncorner;
      }else{
        return 0;
      }
    }
    */
    
  }
  return 0;
  
}


/**
* 控制车的角度（壁障用）
 * input:   目标角度
 *   global 
 */
float dealangle(float angle){
  //float p=0.8;
  //float d=0.2;
  float controlout;
  controlout = P_Angle*(angle - AngleBlock) + D_Angle*Gyro_x;
  
  return controlout;
  
}



