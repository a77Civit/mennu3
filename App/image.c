
/**
 * name:       image.c
 * usage:      --
 * author:     [[
 * date:       2018-05-23 15:11:08
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"



// 全局变量定义
uint8 *imgbuffdeal;     // 定义存储接收图像的指针
uint8 *imgbuffback;
uint8 imgbuff1[CAMERA_SIZE]; // 定义存储接收图像的数组
uint8 imgbuff2[CAMERA_SIZE]; 
uint8 img[CAMERA_H][CAMERA_W];  // 定义图像解压数组
int Image_Flag = 1;
int road_info[4][CAMERA_H];  // 图像每行的中点, 左边线, 右边线, 插值中点位置
float Tranx[60][80];
float Trany[60][80];
float Tranx3[60][80];
float Trany3[60][80];

int LeftBoundry[60];
int RightBoundry[60];
int BoundryPoint[2];
int MiddleLinex[60];
int MiddleLiney[60];
int EndCount;
int EndCountTemp[5];
int LeftLoss; 
int RightLoss;

int StartLine = 58;
int EndLine = 15;
int LossStart = 0;  // 判断丢线起始行
int CircleSen = 450;       // 判断环的电磁敏感度
float Fast = 0;
float Slow = 0;



/**
 * 判断是否找不到路线
 * input:
 *   周期：
 */


/**
 * 设置目标速度
 * input:
 *   周期：
 */
float SpeedCalculate(void){
  /*
  if (CircleState==0){
    if (LeftLoss>4 || RightLoss>4){
      return Fast;
    }
  }
  if (CircleState==1){
    return Slow;
  }
  if (CircleState==2){
    return Fast;
  }
  if (CircleState==3){
    return Fast;
  }
  
  return Fast;
  */
  if (ModelFlag == 1){
    return 350;
  }else{
    return 350;
  }
    
}

/**
 * 出环模式
 * input:
 *   
 * output: 
 *   
 */
int outcircle(int direction){
  int i,f;
  int LeftCount = 0;
  int RightCount = 0;
  
  // 首先获取前5行的边界
  for (i=StartLine;i>StartLine-5;i--){
    f = getboundrypoint(i,BoundryPoint,39);
    if (f == 0){ return 0;}
    LeftBoundry[LeftCount] = BoundryPoint[0];
    LeftCount++;
    RightBoundry[RightCount] = BoundryPoint[1];
    RightCount++;
       
  }
  
  
  // 左环
  if (direction == 1){
    // 查找左边界跳变点并补线
    for (i=StartLine-5;i>=EndLine;i--){
      //LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
      RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
      if (RightBoundry[RightCount]==-1){
          EndCount = RightCount-1;
          return 1;
      }
      f = getboundrypoint(i,BoundryPoint,RightBoundry[RightCount]);
      if (f==0){
          EndCount = RightCount-1;
          return 1;
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      
      LeftCount++;
      RightCount++;
    
    }

  }
  
    


  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  
  // 右环
  if (direction == 2){
    // 查找左边界跳变点并补线
    for (i=StartLine-5;i>=EndLine;i--){
      LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
      //RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
      if (LeftBoundry[LeftCount]==-1){
          EndCount = LeftCount-1;
          return 1;
      }
      f = getboundrypoint(i,BoundryPoint,LeftBoundry[LeftCount]);
      if (f==0){
          EndCount = RightCount-1;
          return 1;
      }
      RightBoundry[RightCount] = BoundryPoint[1];
      
      LeftCount++;
      RightCount++;
    
    }

  
  }
  EndCount = LeftCount-1;
  
  return 1;
  
  
}

/**
 * 环内模式判断是否需要补线
 * input:
 *   
 * output: 
 *   
 */
int incircleb(int x,int y){
  int i;
  for (i=x-1;i>=x-4;i--){
    if (img[i][y] == Black_Point){
      return 0;
    }
  }
  return 1;
}




/**
 * 环内模式
 * input:
 *   
 * output: 
 *   
 */
int incircle(int direction){
  int i,j,m,f;
  int flag = 0;
  int LeftCount = 0;
  int RightCount = 0;
  float k;
  
  if (ModelFlag==0){
    k=1.5;
  }else{
    k=3;
  }
  
  for (i=StartLine;i>StartLine-2;i--){
    f = getboundrypoint(i,BoundryPoint,39);
    if (f == 0){ return 0;}
    LeftBoundry[LeftCount] = BoundryPoint[0];
    LeftCount++;
    RightBoundry[RightCount] = BoundryPoint[1];
    RightCount++;
       
  }

  
  // 左环
  if (direction == 1){
    for (i=StartLine-2;i>=EndLine;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //if (img[i][m] == Black_Point){ 
      //  EndCount = LeftCount-1;
      //  return 1;
      //}
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      if (flag==0){
        if (incircleb(i,RightBoundry[RightCount])){
          flag = 1;
          for (j=1;i-j>=EndLine;j++){
            if (img[i-j][RightBoundry[RightCount]-(int)(k*j)+1]==Black_Point){
              break;
            }else{
              img[i-j][RightBoundry[RightCount]-(int)(k*j)+1] = Black_Point;
            }
          }
        }
      }
      LeftCount++;
      RightCount++;
    }
  }
  
  // 右环
  if (direction == 2){
    for (i=StartLine-2;i>=EndLine;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //if (img[i][m] == Black_Point){ 
      //  EndCount = LeftCount-1;
      //  return 1;
      //}
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      if (flag==0){
        if (incircleb(i,LeftBoundry[LeftCount])){
          flag = 1;
          for (j=1;i-j>=EndLine;j++){
            if (img[i-j][LeftBoundry[LeftCount]+(int)(k*j)-1]==Black_Point){
              break;
            }else{
              img[i-j][LeftBoundry[LeftCount]+(int)(k*j)-1] = Black_Point;
            }
          }
        }
      }
   //   if (LeftBoundry[LeftCount]<=LeftBoundry[LeftCount-1]){
   //     LeftBoundry[LeftCount] = LeftBoundry[LeftCount-1]+1;
   //   }
      
      LeftCount++;
      RightCount++;
    }
  }
  
  EndCount = LeftCount-1;
  
    
  return 1;
  
}


/**
 * 入环模式
 * input:
 *   
 * output: 
 *   
 */
int intocircle(int direction){
  int i,j,f,m,k;
  //int t=0;
  int flag = 1;
  int LeftCount = 0;
  int RightCount = 0;
  
  if (ModelFlag == 1){
    k = 2;
  }else{
    k = 1;
  }

  // 首先获取前1行的边界
  f = getboundrypoint(StartLine,BoundryPoint,39);
  if (f == 0){ return 0;}
  LeftBoundry[LeftCount] = BoundryPoint[0];
  RightBoundry[RightCount] = BoundryPoint[1];
  LeftCount++;
  RightCount++;
  
  // 左环
  if (direction == 1){
    for (i=StartLine-1;i>LossStart;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //
      if (flag == 0 && img[i][m] == Black_Point){ 
        EndCount = LeftCount-1;
        return 1;
      }
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      // 跳变点补线
      if (flag && RightBoundry[RightCount]-RightBoundry[RightCount-1]<-8){
        flag = 0;
        for (j=1;j<=RightCount;j++){
            if (RightBoundry[RightCount]+(int)(0.6*j)<RightBoundry[RightCount-j]){
              RightBoundry[RightCount-j] = RightBoundry[RightCount]+(int)(0.6*j);
            }else{
              break;
            }
        }
      }
      LeftCount++;
      RightCount++;
       
    }

    for (i=LossStart;i>=EndLine;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //
      if (flag == 0 && img[i][m] == Black_Point){ 
        EndCount = LeftCount-1;
        return 1;
      }
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      // 跳变点补线
      if (flag && RightBoundry[RightCount]-RightBoundry[RightCount-1]<-5){
        flag = 0;
        for (j=1;j<=RightCount;j++){
            if (RightBoundry[RightCount]+(int)(k*j)<RightBoundry[RightCount-j]){
              RightBoundry[RightCount-j] = RightBoundry[RightCount]+(int)(k*j);
            }else{
              break;
            }
        }
      }
      
      if (flag && LeftBoundry[LeftCount]-LeftBoundry[LeftCount-1]>4 && 
          img[i-1][LeftBoundry[LeftCount]+3]==White_Point){
        flag = 0;
        for (j=0;j<=RightCount;j++){
            if (LeftBoundry[LeftCount]+(int)(k*j)<RightBoundry[RightCount-j]){
              RightBoundry[RightCount-j] = LeftBoundry[LeftCount]+(int)(k*j);
            }else{
              break;
            }
        }
      }
      
      LeftCount++;
      RightCount++;
    }
  }
  
  
  
  // 右环
  if (direction == 2){
    for (i=StartLine-1;i>LossStart;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //
      if (flag == 0 &&img[i][m] == Black_Point){ 
        EndCount = LeftCount-1;
        return 1;
      }
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      // 跳变点补线
      if (flag && LeftBoundry[LeftCount]-LeftBoundry[LeftCount-1]>8){
        flag = 0;
        for (j=1;j<=LeftCount;j++){
            if (LeftBoundry[LeftCount]-(int)(0.6*j)>LeftBoundry[LeftCount-j]){
              LeftBoundry[LeftCount-j] = LeftBoundry[LeftCount]-(int)(0.6*j);
            }else{
              break;
            }
        }
      }
      LeftCount++;
      RightCount++;
       
    }

    for (i=LossStart;i>=EndLine;i--){
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      //
      if (flag == 0 &&img[i][m] == Black_Point){ 
        EndCount = LeftCount-1;
        return 1;
      }
      f = getboundrypoint(i,BoundryPoint,m);
      if (f == 0){
        EndCount = LeftCount-1;
        return 1; 
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
      
      // 跳变点补线
      if (flag && LeftBoundry[LeftCount]-LeftBoundry[LeftCount-1]>5){
        flag = 0;
        for (j=1;j<=LeftCount;j++){
            if (LeftBoundry[LeftCount]-(int)(k*j)>LeftBoundry[LeftCount-j]){
              LeftBoundry[LeftCount-j] = LeftBoundry[LeftCount]-(int)(k*j);
            }else{
              break;
            }
        }
      }
      
      if (flag && RightBoundry[RightCount]-RightBoundry[RightCount-1]<-4 && 
          img[i-1][RightBoundry[RightCount]-3]==White_Point){
        flag = 0;
        for (j=0;j<=LeftCount;j++){
            if (RightBoundry[RightCount]-(int)(k*j)>LeftBoundry[LeftCount-j]){
              LeftBoundry[LeftCount-j] = RightBoundry[RightCount]-(int)(k*j);
            }else{
              break;
            }
        }
      }
      
      LeftCount++;
      RightCount++;
    }
  }
      
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
     
      
  /**
    // 左环  
    if (direction == 1){
    // 跳变点并补线
    for (i=StartLine-5;i>=EndLine;i--){
      //LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
      RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
      if (RightBoundry[RightCount]==-1){
          EndCount = RightCount-1;
          return 1;
      }
      f = getboundrypoint(i,BoundryPoint,RightBoundry[RightCount]);
      LeftBoundry[LeftCount] = BoundryPoint[0];
      if (LeftBoundry[LeftCount]-LeftBoundry[LeftCount-1]>13){
        for (j=0;j<=RightCount;j++){
          if (LeftBoundry[LeftCount]+j<RightBoundry[RightCount-j]){
            RightBoundry[RightCount-j] = LeftBoundry[LeftCount]+j;
            }else{
            break;
          }
        }
        t=i;
        break;
      
      }
      LeftCount++;
      RightCount++;
    
    }

  
    if (t>EndLine){
      for (i=t;i>=EndLine;i--){
        LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
        if (LeftBoundry[LeftCount] == -1){
          EndCount = LeftCount-1;
          return 1;
        }
        f = getboundrypoint(i,BoundryPoint,LeftBoundry[LeftCount]);
        if (f == 0){
          EndCount = LeftCount-1;
          return 1;
        }
        RightBoundry[RightCount] = BoundryPoint[1];
        LeftCount++;
        RightCount++;
      }
    }
  }


  
  
  // 右环
  if (direction == 2){
    // 查找左边界跳变点并补线
    for (i=StartLine-5;i>=EndLine;i--){
      LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
      //RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
      if (LeftBoundry[LeftCount]==-1){
          EndCount = LeftCount-1;
          return 1;
      }
      f = getboundrypoint(i,BoundryPoint,LeftBoundry[LeftCount]);
      RightBoundry[RightCount] = BoundryPoint[1];
      if (RightBoundry[RightCount]-RightBoundry[RightCount-1]<-13){
        for (j=0;j<=LeftCount;j++){
          if (RightBoundry[RightCount]-j>LeftBoundry[LeftCount-j]){
            LeftBoundry[LeftCount-j] = RightBoundry[RightCount]-j;
            }else{
            break;
          }
        }
        t=i;
        break;
      
      }
      LeftCount++;
      RightCount++;
    
    }

  
    if (t>EndLine){
      for (i=t;i>=EndLine;i--){
        RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
        if (RightBoundry[RightCount] == -1){
          EndCount = RightCount-1;
          return 1;
        }
        f = getboundrypoint(i,BoundryPoint,RightBoundry[LeftCount]);
        if (f == 0){
          EndCount = RightCount-1;
          return 1;
        }
        LeftBoundry[LeftCount] = BoundryPoint[0];
        LeftCount++;
        RightCount++;
      }
    }
  }
 */
  EndCount = LeftCount-1;

  return 1;
  
}


  

/**
 * 环岛判断
 * input:
 *   
 * output: 0:no   1:left  2:right
 *   
 */
int iscircle(void){
  //int flag,i;
  //左环判断
  
  if (ModelFlag == 0){
    if (ELEC[0]>850 && ELEC[1]>550 && ELEC[2]<150 && ELEC[3]>90){
      return 1;
    }
    if (ELEC[3]>850 && ELEC[2]>550 && ELEC[1]<150 && ELEC[0]>90){
      return 2;
    }
  }
  
  /**
  if (ModelFlag == 1){
    if (ELEC[0]>450 && ELEC[3]>450 && ELEC[0]+ELEC[3]>1100){
      if (LeftLoss>=3 && RightLoss<=4){
        return 1;
      }
      if (LeftLoss<=4 && RightLoss>=3){
        return 2;
      }
    }
  }
  */
      
  
  if (ModelFlag == 1){
    if (ELEC[0]+ELEC[3]>1300 && ELEC[1]>150 && ELEC[2]<70 ){
      return 1;
    }
    if (ELEC[0]+ELEC[3]>1300 && ELEC[2]>150 && ELEC[1]<70){
      return 2;
    }
  }
  
  
  
  /**
  if (LeftLoss>4 && RightLoss<=4){
    flag = 1;
    for (i=20;i<60;i++){
      if (img[i][39]==Black_Point){
        flag = 0;
      }
    }
  }
  */
  /**
  if (ELEC[0]+ELEC[3]>2*CircleSen+200 && ELEC[0]>CircleSen && ELEC[3]>CircleSen){
    
    // 左环
    if (ModelFlag == 0){
      if (ELEC[1]>200 && ELEC[2]<100){
        return 1;
      }
    }else{
      if (LeftLoss>=3 && RightLoss<=4){
        return 1;
      }
    }
    // 右环
    if (ModelFlag == 0){
      if (ELEC[1]<100 && ELEC[2]>200){
        return 2;
      }
    }else{
      if (LeftLoss<=4 && RightLoss>=3){
        return 2;
      }
    }
    
  }
  */
  
  
  return 0;

  
    

}





/**
 * 中线提取
 * input:
 *   
 * output:
 *   
 */
int getcentralline(void){
  int i;
  int LeftEff=0;
  int RightEff=0;
  /**
  getboundryline();
  roadinfo = getroadinfo();
  // 直线小S弯直接计算中线
  if (roadinfo==0){
    for (i=0;i<=EndCount;i++){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[i])/2;
      MiddleLinex[i]=StartLine-i;
    }
  }
  // 左大弯
  if (roadinfo==1){
    for (i=0;i<LeftEff;i++){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[i])/2;
      MiddleLinex[i]=StartLine-i;
    }
    for (i=LeftEff;i<=EndCount;i++){
      MiddleLiney[i]=(LeftBoundry[LeftEff]+RightBoundry[i])/2;
      MiddleLinex[i]=(StartLine-LeftEff+StartLine-i)/2;
    }
  }
  // 右大弯
  if (roadinfo==2){
    for (i=0;i<RightEff;i++){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[i])/2;
      MiddleLinex[i]=StartLine-i;
    }
    for (i=RightEff;i<=EndCount;i++){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[RightEff])/2;
      MiddleLinex[i]=(StartLine-i+StartLine-RightEff)/2;
    }
  }
  // 十字
  if (roadinfo==3){
    for (i=0;i<=EndCount;i++){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[i])/2;
      MiddleLinex[i]=StartLine-i;
    }
    
  }
      
  */
  //if (getboundryline()==0){return 0;}
  for (i=0;i<=EndCount;i++){
    if (LeftBoundry[i]==0 && RightBoundry[i]==79){
      MiddleLiney[i]=39;
      MiddleLinex[i]=StartLine-i;
      //LeftEff = i;
      //RightEff = i;
    }
    
    if (LeftBoundry[i]!=0 && RightBoundry[i]!=79){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[i])/2;
      MiddleLinex[i]=StartLine-i;
      LeftEff = i;
      RightEff = i;
    }
    
    // 左丢线
    if (LeftBoundry[i]==0 && RightBoundry[i]!=79){
      MiddleLiney[i]=(LeftBoundry[LeftEff]+RightBoundry[i])/2;
      MiddleLinex[i]=(StartLine-i+StartLine-LeftEff)/2;
      RightEff = i;
    }
    
    //右丢线
    if (LeftBoundry[i]!=0 && RightBoundry[i]==79){
      MiddleLiney[i]=(LeftBoundry[i]+RightBoundry[RightEff])/2;
      MiddleLinex[i]=(StartLine-i+StartLine-RightEff)/2;
      LeftEff = i;
    }
  }
  
    showline(MiddleLinex,MiddleLiney,EndCount+1,BLUE);
    showboundry(LeftBoundry,EndCount+1,RED);
    showboundry(RightBoundry,EndCount+1,RED);
  return 1;  
  
  
}


/**
 * 路况判断
 * input:
 *   
 * output:
     -1:错误 保持之前的判断
 *   0：左右均没太多丢线 直接取中点即可
     1：左弯  2：右弯  
     3：十字         4：左环    5：右环
 */
int getroadinfo(void){
  if (EndCount<6){return -1;}
  
  // 左右均未丢线
  if (LeftLoss<4 && RightLoss<4){
    return 0;
  }
  
  // 右丢线
  if (LeftLoss<4){
    // 寻找起始丢线位置
  
  }
  
  // 左丢线
  if (RightLoss<4){
    // 寻找起始丢线位置
 
  }
  
  // 左右均丢线
  if (LeftLoss>=4 && RightLoss>=4){}
    
  
  
  return -1;
}



/**
 * 提取边界
 * input:
 *   
 * output:
 *   
 */
int getboundryline(void){
  int i,f,m;
  int LeftCount = 0;
  int RightCount = 0;
  
  // 左右丢线清零
  LeftLoss=0;
  RightLoss=0;
  // 首先获取前5行的边界
  for (i=StartLine;i>StartLine-5;i--){
    f = getboundrypoint(i,BoundryPoint,39);
    if (f == 0){ return 0;}
    LeftBoundry[LeftCount] = BoundryPoint[0];
    LeftCount++;
    RightBoundry[RightCount] = BoundryPoint[1];
    RightCount++;
  }
  
  
  // 借助上一行的位置查找边界
  for (i=StartLine-5;i>=EndLine;i--){
    LeftBoundry[LeftCount] = searchboundry(i,LeftBoundry[LeftCount-1]);
    RightBoundry[RightCount] = searchboundry(i,RightBoundry[RightCount-1]);
    
    // 
    if (LeftBoundry[LeftCount]!=-1 && RightBoundry[RightCount]!=-1){
      // 中间点为黑点则重新查找
      if (img[i][m] == Black_Point){
        m = (LeftBoundry[LeftCount] + RightBoundry[RightCount])/2;
        f = getboundrypoint(i,BoundryPoint,m);
        if (f==0) {
          EndCount = LeftCount-1;
          return 1;
      }
        LeftBoundry[LeftCount] = BoundryPoint[0];
        RightBoundry[RightCount] = BoundryPoint[1];
      }
    }else {
      m = (LeftBoundry[LeftCount-1] + RightBoundry[RightCount-1])/2;
      f = getboundrypoint(i,BoundryPoint,m);
      if (f==0) {
        EndCount = LeftCount-1;
        return 1;
      }
      LeftBoundry[LeftCount] = BoundryPoint[0];
      RightBoundry[RightCount] = BoundryPoint[1];
    }
    
    LeftCount++;
    RightCount++;
    if (i<LossStart){
      if (LeftBoundry[LeftCount-1] == 0){
        LeftLoss++;
      }
      if (RightBoundry[RightCount-1] == 79){
        RightLoss++;
      }
    }
    
  }
  EndCount = LeftCount-1;
  return 1;
  

}


/**
* 通过上一行查找当前行的边界点
 * input:
 *   
 * output:
 *   
 */
int searchboundry(int Line, int Lastj){
  int j,color;
  
  if (Lastj==-1)
    return -1;
  if (Lastj==0 || Lastj==79)
    if (img[Line][Lastj] == White_Point)
      return Lastj;

  color = img[Line][Lastj];
  for (j=1;j<7;j++){
    if (Lastj+j==79 && img[Line][79]==White_Point)
      return 79;
    if (Lastj+j<=79)
      if (img[Line][Lastj+j] != color)
        if (color == Black_Point){
          return Lastj+j;
        }else
          return Lastj+j-1;
    
    if (Lastj-j==0 && img[Line][0]==White_Point)
      return 0;
    if (Lastj-j>=0)
      if (img[Line][Lastj-j] != color)
        if (color == Black_Point){
          return Lastj-j;
        }else
          return Lastj-j+1;
  }
  return -1;
}




/**
 * 获取当前行的边界点
 * input:
 *   
 * output:
 *   
 */
int getboundrypoint(int L, int *B, int Start){
  int flag=0;
  int Sensitive,j;
  // 三轮
  // (15,26) (15,54)    28
  // (48,1) (48,79)     78
  // 50/33
  // 二轮
  // (42,24) (42,59)    35
  // (60,10) (60,73)    63
  // 28/18
  if (ModelFlag){
    //Sensitive = (int)((24+1.47*(L-26))/3);   // 直立模式赛道宽度敏感度
    Sensitive = (int)((42+1.556*(L-42))/1.5);
  }else{
    //Sensitive = (int)((24+1.47*(L-26))/1.5);
    Sensitive = (int)((15+1.515*(L-15))/1.5);
  }
  B[0]=0;
  B[1]=OV7725_EAGLE_W-1;
  if (img[L][Start] == Black_Point){
    for (j=0;j<10;j++){
      if (img[L][39+j] == White_Point){
        Start = 39+j;
        flag = 1;
      }
      if (img[L][39-j] == White_Point){
        Start = 39-j;
        flag = 1;
      }
    }
  }else{
    flag = 1;
  }
  
  if (flag == 0){return 0;}  
  
  for (j=Start+1;j<80;j++){
    if (img[L][j] == Black_Point){
      B[1] = j-1;
      break;
    }
  }
  for (j=Start-1;j>-1;j--){
    if (img[L][j] == Black_Point){
      B[0] = j+1;
      break;
    }
  }
  
  if (B[1]-B[0] < Sensitive){ return 0;}
  return 1;
    
}
    

/**
 * 显示图像的中点
 * input:
 * output:
 *   0: 函数成功退出
 */
int showline(int *Linex, int *Liney, int size, uint16_t rgb565) {
  int aa;              // 循环计数器
  Site_t redline[size];  // 显示坐标

  for (aa = 0; aa < size; aa++) {
    redline[aa].y = Linex[aa];
    redline[aa].x = Liney[aa];
  }

  lcd_points(redline, size, rgb565);

  return 0;
}

/**
 * 显示图像的边界
 * input:
 * output:
 *   0: 函数成功退出
 */      
int showboundry(int *Line, int size, uint16_t rgb565){
  int aa;
  Site_t redline[size];  // 显示坐标

  for (aa = 0; aa < size; aa++) {
    redline[aa].y = StartLine-aa;
    redline[aa].x = Line[aa];
  }

  lcd_points(redline, size, rgb565);
  
  return 0;
}




/**
 * 两点法求曲率半径
 * input：
 * output：
 *   
 */
float curvature(int x1,int y1,float x2,float y2) { 
  float R; //曲率半径
  if (y1==y2){
    return 99999;
  }
  R=((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))/(2.0*(y2-y1));//两点法曲率半径公式
  return R;
}

/**
 * 根据中线计算转角
 * input：
 * output：
 *   
 */
float dealcorner(void){
  int i=0;
  float R,k=0;
  float x,y;
  if (ModelFlag == 1){
    for (i=0;i<=EndCount;i++){
      x = Tranx[MiddleLinex[i]][MiddleLiney[i]];
      y = Trany[MiddleLinex[i]][MiddleLiney[i]];
      R = curvature(68,40,x,y);
      k += (1/R)/(EndCount+1);
    }
  }
  
  if (ModelFlag == 0){
    for (i=0;i<=EndCount;i++){
      x = Tranx3[MiddleLinex[i]][MiddleLiney[i]];
      y = Trany3[MiddleLinex[i]][MiddleLiney[i]];
      R = curvature(68,40,x,y);
      k += (1/R)/(EndCount+1);
    }
  }
  return k;
}


/**
 * 坐标变换
 * input：
 * output：
 *   
 */
int Transform(void){
  int x,y;
  float rot[9]={-13.3197,   -7.2131,   -0.1803,
                0,   -2.0984,         0,
                219.7951,  123.9344,    1.0000};
  
  float rot3[9]={11.0588,    5.1765,    0.1294,
                 -0.0000,    3.5294,    0.0000,
                -107.0588,  -101.1765,  1.0000};
  float z1,z2,z3;
  for (x=1;x<=60;x++)
    for (y=1;y<=80;y++){
      z1 = rot[0]*x+rot[3]*y+rot[6];
      z2 = rot[1]*x+rot[4]*y+rot[7];
      z3 = rot[2]*x+rot[5]*y+rot[8];
      Tranx[x-1][y-1] = z1/z3;
      Trany[x-1][y-1] = z2/z3;
    }
  for (x=1;x<=60;x++)
    for (y=1;y<=80;y++){
      z1 = rot3[0]*x+rot3[3]*y+rot3[6];
      z2 = rot3[1]*x+rot3[4]*y+rot3[7];
      z3 = rot3[2]*x+rot3[5]*y+rot3[8];
      Tranx3[x-1][y-1] = z1/z3;
      Trany3[x-1][y-1] = z2/z3;
    }
  
  
  
  
  return 1;
}
    
//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
        uint8 colour[2] = {255, 0};   //0 和 1 分别对应的颜色
                                               //注：山外的摄像头 0 表示 白色，1表示 黑色
        uint8 tmpsrc;
        while(srclen --)
        {
                tmpsrc = *src++;
                *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
                *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
         }
}
