

/*------------------mian-----------------------

usage:      主函数
date:       2018-07-21 10:50:37
version:    v1.1.4

---------------------[[------------------------

v1.1.4: 解决了发车停车会车和远端会车

-----------------主要全局变量-------------------

CONTROL_S control_handle      系统控制句柄
uint8 img[CAMERA_H][CAMERA_W] 图像解压数组
int road_info[4][CAMERA_H]    图像每行的中点,
左边线, 右边线, 插值中点位置

---------------------------------------------*/

#include "common.h"
#include "include.h"




void main(void) {
  // 路况标志
  //int stopFlag = 0;  // 停车标志
  //DELAY_MS(1000);

  // 检查是否烧录FWD
  ASSERT(enter_init());

  led_init (LED1);
  led_init (LED2);
  //gpio_init (BUZZ, GPO,0);	//初始化蜂鸣器
  Transform();           // 生成坐标变换表
  lcd_init(RED);         // 小液晶初始化
  //elec_init();           // 电磁初始化
  pit_time_start  (PIT2); // 开始计时
  
  // 摄像头初始化
  //imgbuffback = imgbuff1;  
  //imgbuffdeal = imgbuff2;
  //camera_init(imgbuffback);  
  
  power_init();          // 电机和舵机初始化
  key_init(KEY_MAX);     // 按键初始化
  //communication_init();  //通讯初始化 开启接收中断
  MPU6050_init();        // 陀螺仪初始化
  
  Changepara(ModelFlag);
  

  // 配置按键中断
  pit_init_ms(PIT1, 50);                          // PIT1 定时 50ms
  set_vector_handler(PIT1_VECTORn, PIT1_hander);  // 设置按键查询中断
  enable_irq(PIT1_IRQn);  

  // 选择行驶模式
  
startmenu();  // 开始菜单

  //interupt_init();       // 中断初始化

  // 配置时间

  lptmr_time_start_ms();  // 计时器清零

  while(1)
  {
    //moto_power(13,13);
    /************************ 图像采集和显示  ***********************/
    
    while (!Image_Flag && Raser_Flag == 0);
    if (Image_Flag){
      Image_Flag = 0;
	  camera_get_img();     // 摄像头获取图像 
    }
    if (Raser_Flag == 2){
      Raser_Flag = 0;
      uart_sentn(LASER_PORT2, "r6#",3);
    }
    if (Raser_Flag == 3){
      Raser_Flag = 0;
      uart_sentn(LASER_PORT3, "r6#",3);
    }
      
  }

}
