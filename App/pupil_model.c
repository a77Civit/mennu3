
/**
 * name:       pupil_model.c
 * usage:      --
 * author:     [[
 * date:       2018-07-21 10:51:01
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"

/**
 * 小学生教学模式
 * input:
 * output:
 */
int pupil_model(void) {
  // 速度常量
  // const int circle_speed = 20; // 弯道速度
  // const int line_speed = 0;  // 直线速度

  // 路况标志
  ROAD_PUPIL road_kind;  // 弯道标志
  float corner = 0;      // 舵机偏转角度
  int stop_flag = 0;     // 停车标志

  // 获取路况
  road_kind = pupil_road_judge();
  road_kind = road_kind_pirate(road_kind);

  // // 发车前 10s 关闭超声测距中断
  // if (control_handle.loop_time < 10000) {
  //   disable_irq((IRQn_t)(UART3_RX_TX_IRQn));
  // } else {
  //   enable_irq((IRQn_t)(UART3_RX_TX_IRQn));
  // }

  // 靠边行驶准备会车
  meeting_car();

  // 按照道路确定行驶函数
  switch (road_kind) {
    // 出界停车
    case pOutside:
      moto_control(0, 0);  // 解除 PID 控制
      moto_power(0, 0);    // 关闭电机
      while (1) {
        // 向对方车告别
        say_goodbye();
      }
      break;

    // 斑马线返回停车标志
    case pZbraStripe:
      stop_flag = 1;
      break;

#if WHICH_CAR
    // * GONG
    // 左边入环

    case pLeftCycle:
      steer(1.25);
      break;

    // 右边入环
    case pRightCycle:
      steer(-1.20); 
      break;

    // 出环
    case pOutCycle:
      switch (control_handle.cycle_side) {
        // 出左环
        case 1:
          steer(1.70);
          break;
        // 出右环
        case 2:
          steer(-1.70);
          break;
      }
      break;

    case pTurnLeft:
      steer_control(0);
      moto_control(250, 350);
      break;

    case pTurnRight:
      steer_control(0);
      moto_control(350, 250);
      break;

    case pStraight:
      steer_control(0);
      moto_control(370, 370);
      break;

    // 正常行车
    default:
      steer_control(0);
      moto_control(350, 350);
      break;
#else
    // * SHOU
    // 左边入环
    case pLeftCycle:
      steer(1.20);
      break;

    // 右边入环
    case pRightCycle:
      steer(-1.15);
      break;

    // 出环
    case pOutCycle:
      switch (control_handle.cycle_side) {
        // 出左环
        case 1:
          steer(1.80);
          break;
        // 出右环
        case 2:
          steer(-1.80);
          break;
      }
      break;

    case pTurnLeft:
      steer_control(0);
      moto_control(250, 350);
      break;

    case pTurnRight:
      steer_control(0);
      moto_control(350, 250);
      break;

    case pStraight:
      steer_control(0);
      moto_control(370, 370);
      break;

    // 正常行车
    default:
      steer_control(0);
      steer(pupil_correction(1));
      moto_control(350, 350);
      break;

#endif
  }

  // 文字显示坐标
  Site_t loc[14] = {{0, 70},   {0, 85},   {0, 100}, {65, 70},  {65, 85},
                    {65, 100}, {25, 70},  {25, 85}, {25, 100}, {88, 70},
                    {88, 85},  {88, 100}, {88, 40}, {88, 55}};

  lcd_str(loc[0], "rot", BLUE, RED);  // 转速
  lcd_num_c(loc[6], control_handle.left_speed, BLUE, RED);

  lcd_str(loc[1], "rot", BLUE, RED);  // 转速
  lcd_num_c(loc[7], control_handle.right_speed, BLUE, RED);

  lcd_str(loc[2], "met", BLUE, RED);  // 会车
  lcd_num_c(loc[8], 0, BLUE, RED);    // 识别是否有车

  lcd_str(loc[3], "dis", BLUE, RED);  // 车距
  lcd_num_c(loc[9], control_handle.distance, BLUE, RED);

  lcd_str(loc[4], "cor", BLUE, RED);  // 舵机
  lcd_num_c(loc[10], (int)(corner > 0 ? corner : -corner), BLUE, RED);

  lcd_str(loc[5], "rl", BLUE, RED);  // 路型
  lcd_num_c(loc[11], road_kind, BLUE, RED);

  return stop_flag;
}

/**
 * 小学生弯道修正函数
 *                  *****************
 *                  **     |  *    **
 *                  **     | *     **
 *                  **     |*      **
 *                  *****************
 * 统计 图像中心(|) 与 道路中线(*) 的偏差, 并通过线性方程
 *                      y = kx + b
 * 来回归舵机偏转度量
 * input:
 *   int model:
 *     -1 靠左侧行驶         1 靠右侧行驶
 *     -2 靠左侧 1/3 处行驶  2 靠有右侧 1/3 处行驶
 *      0 正常行驶
 * output:
 *   舵机转角
 */
float pupil_correction(int model) {
  float corner;    // 舵机偏转量
  float distance;  // 道路中线与图像中线的误差

  // 统计总偏差

  distance = road_info[0][45] + road_info[0][46] + road_info[0][47] +
             road_info[0][48] + road_info[0][49] + road_info[0][50] +
             road_info[0][51] + road_info[0][52] + road_info[0][53] +
             road_info[0][54];

  // 根据行驶模式选择回归函数
  switch (model) {
    case -1:
      corner = -0.01 * distance + 4.6;
      break;
    case 1:
      corner = -0.01 * distance + 3.4;
      break;
    case 0:
      corner = -0.01 * distance + 4;
      break;
    case 2:
      corner = -0.01 * distance + 3.8;
      break;
    case -2:
      corner = -0.01 * distance + 4.2;
      break;
    default:
      corner = -0.01 * distance + 4.00;
      break;
  }
  return corner;
}

/**
 * 会车靠边行驶舵机修正函数
 * 取中线的点行与小学生舵机修正函数不同
 */
float pupil_meeting(int model) {
  float corner;    // 舵机偏转量
  float distance;  // 道路中线与图像中线的误差

  // 统计总偏差

  distance = road_info[0][45] + road_info[0][46] + road_info[0][47] +
             road_info[0][48] + road_info[0][49] + road_info[0][50] +
             road_info[0][51] + road_info[0][52] + road_info[0][53] +
             road_info[0][54];

  // 根据行驶模式选择回归函数
  switch (model) {
    case -1:
      corner = -0.01 * distance + 4.6;
      break;
    case 1:
      corner = -0.01 * distance + 3.4;
      break;
    case 0:
      corner = -0.01 * distance + 4;
      break;
    case 2:
      corner = -0.01 * distance + 3.8;
      break;
    case -2:
      corner = -0.01 * distance + 4.2;
      break;
    default:
      corner = -0.01 * distance + 4.00;
      break;
  }
  return corner;
}

/**
 * 判断路况
 * input:
 *   global image
 * output:
 *   道路状况
 */
ROAD_PUPIL pupil_road_judge(void) {
  // 底部区块 特征组
  const int SCAN_LEN = 10;   // 底部矩形对角线扫描长度
  const int L_START_X = 1;   // 底部左侧矩形起点横坐标
  const int L_START_Y = 59;  // 底部左侧矩形起点纵坐标
  const int R_START_X = 78;  // 底部右侧矩形起点横坐标
  const int R_START_Y = 59;  // 底部右侧矩形起点纵坐标

  // 斜线 特征组
  const int ACROSS_LINE_LEN = 30;  // 斜线扫描长度
  const int L_LINE_START_X = 30;   // 左侧斜线起点横坐标
  const int L_LINE_START_Y = 59;   // 左侧斜线起点纵坐标
  const int R_LINE_START_X = 48;   // 右侧斜线起点横坐标
  const int R_LINE_START_Y = 59;   // 右侧斜线起点纵坐标

  // 横线特征组
  const int STOP_LINE = 40;  // 紧急停车线高度

  // 特征线颜色标志
  int l_flag = 0;          // 左侧标志 1 白 0 黑
  int r_flag = 0;          // 右侧标志 1 白 0 黑
  int lh_flag = 0;         // 左侧高斜线 1 白 0 黑
  int rh_flag = 0;         // 右侧高斜线 1 白 0 黑
  int stop_flag = 1;       // 出界停车标志 1 白 0 黑
  int cycle_flag = 0;      // 确认环标志
  int out_cycle_flag = 0;  // 出环标志

  int count = 0;                // 白点计数器
  int aa;                       // 循环标志
  ROAD_PUPIL flag = pStraight;  // 道路状况

  // 时间
  static int pre_cycle_flag;  // 入环预判断

  // 检查左侧是否出现白色区域
  for (aa = 0; aa < SCAN_LEN; aa++) {
    count += img[L_START_Y - aa][L_START_X + aa];
  }

  if (count == SCAN_LEN) {
    l_flag = 1;
  }

  count = 0;

  // 检查右侧是否出现白色区域
  for (aa = 0; aa < SCAN_LEN; aa++) {
    count += img[R_START_Y - aa][R_START_X - aa];
  }

  if (count == SCAN_LEN) {
    r_flag = 1;
  }

  count = 0;

  // 检测左斜线的颜色
  for (aa = 0; aa < ACROSS_LINE_LEN; aa++) {
    count += img[L_LINE_START_Y - aa][L_LINE_START_X - aa];
  }

  if (count == ACROSS_LINE_LEN) {
    lh_flag = 1;
  }

  count = 0;

  // 检测右斜线的颜色
  for (aa = 0; aa < ACROSS_LINE_LEN; aa++) {
    count += img[R_LINE_START_Y - aa][R_LINE_START_X + aa];
  }

  if (count == ACROSS_LINE_LEN) {
    rh_flag = 1;
  }

  count = 0;

  // 检测前方是否出现黑线
  for (aa = 0; aa < CAMERA_W; aa++) {
    count += img[STOP_LINE][aa];
  }

  if (count < 10) {
    stop_flag = 0;
  }

  count = 0;

  // ! 更改入环逻辑
  // 入环检测, 出环 1s 后检测入环
  // 增加了control_handle.pre_cycle_time 与 pre_cycle_flag
  if (control_handle.loop_time - control_handle.out_time > 1500 &&
      control_handle.cycle_side == 0) {
    // 图形符合则进入环判断
    if (test_cycle() > 0) {
      // 若初次识别入环图形则记录时间与预入环标志
      if (pre_cycle_flag == 0) {
        control_handle.pre_cycle_time = control_handle.loop_time;
        pre_cycle_flag = test_cycle();
        // 时间段内进行二次判断，若二次判断符合则可以判断入环,根据实际情况修改入环时间限制
      } else if (control_handle.loop_time - control_handle.pre_cycle_time >
#if WHICH_CAR 
                 (139000/ // SHOU 142000 取两个编码器的最大值 GONG 139000
#else
                 (142000/ // SHOU 142000 取两个编码器的最大值 GONG 139000                 
#endif
                 (control_handle.left_speed > control_handle.right_speed ? 
                 control_handle.left_speed: control_handle.right_speed)) && 
                 control_handle.loop_time - control_handle.pre_cycle_time < 3500 &&
                 pre_cycle_flag == test_cycle()) {
        cycle_flag = test_cycle();
        pre_cycle_flag = 0;
        control_handle.pre_cycle_time = 0;
        // 若时间段内无二次判断环则为误判
      } else if (control_handle.loop_time - control_handle.pre_cycle_time >=
                 3500) {
        pre_cycle_flag = 0;
      }
    }
  }

  // if (control_handle.loop_time - control_handle.out_time > 1000) {
  //   cycle_flag = test_cycle();
  // }

  // 出环检测, 入环 0.55s 后才开始检测出环
  if (control_handle.cycle_side > 0 &&
      control_handle.loop_time - control_handle.cycle_time > 550) {
    out_cycle_flag = out_cycle_detection(control_handle.cycle_side);
  }

  // 推断道路状况
  if (cycle_flag == 1) {
    // 左边环岛
    flag = pLeftCycle;
    control_handle.cycle_side = cycle_flag;
    control_handle.cycle_time = control_handle.loop_time;
  } else if (cycle_flag == 2) {
    // 右边环岛
    flag = pRightCycle;
    control_handle.cycle_side = cycle_flag;
    control_handle.cycle_time = control_handle.loop_time;
  } else if (out_cycle_flag == 1) {
    // 出环
    flag = pOutCycle;
  } else if (lh_flag == 1 && rh_flag == 0) {
    // 左侧斜线白色 右侧斜线黑色 左转
    flag = pTurnLeft;
  } else if (lh_flag == 0 && rh_flag == 1) {
    // 左侧斜线黑色 右侧斜线白色 右转
    flag = pTurnRight;
  } else if (zebra_detection() == 1) {
    // 斑马线上部白色 下部白色 中部黑白相间 停车
    flag = pZbraStripe;
  } else if (lh_flag == 0 && rh_flag == 0 && stop_flag == 1) {
    // 左侧斜线黑色 右侧斜线黑色 停车线白色 直行
    flag = pStraight;
  } else if (lh_flag == 1 && rh_flag == 1 && stop_flag == 1) {
    // 左侧斜线白色 右侧斜线白色 停车线白色 直行
    flag = pStraight;
  } else if (l_flag == 0 && r_flag == 0 && lh_flag == 0 && rh_flag == 0 &&
             stop_flag == 0) {
    // 左侧黑色 右侧黑色 停车线黑色 停车
    flag = pOutside;
  }

  return flag;
}

/**
 * 显示路况判断的特征线
 */
int pupil_factor_line(void) {
  int aa;                       // 循环计数器
  Site_t stop_top_line[12];     // 上部 斑马线检测线
  Site_t stop_center_line[12];  // 中部 斑马线检测线
  Site_t stop_low_line[12];     // 下部 斑马线检测线
  Site_t left_line[30];         // 左侧斜线
  Site_t right_line[30];        // 右侧斜线
  Site_t up_line[12];           // 上部横线
  Site_t up_left_line[12];      // 上部横线
  Site_t up_right_line[12];     // 上部横线

  // 上方 斑马线检测线
  for (aa = 0; aa < 11; aa++) {
    stop_top_line[aa].y = 30;
    stop_top_line[aa].x = 34 + aa;
  }
  lcd_points(stop_top_line, 12, RED);

  // 中部 斑马线检测线
  for (aa = 0; aa < 20; aa++) {
    stop_center_line[aa].y = 40;
    stop_center_line[aa].x = 29 + aa;
  }
  lcd_points(stop_center_line, 21, RED);

  // 下部 斑马线检测线
  for (aa = 0; aa < 20; aa++) {
    stop_low_line[aa].y = 50;
    stop_low_line[aa].x = 29 + aa;
  }
  lcd_points(stop_low_line, 21, RED);

  // 斜线
  const int ACROSS_LINE_LEN = 30;  // 斜线扫描长度
  const int L_LINE_START_X = 30;   // 左侧斜线起点横坐标
  const int L_LINE_START_Y = 59;   // 左侧斜线起点纵坐标
  const int R_LINE_START_X = 48;   // 右侧斜线起点横坐标
  const int R_LINE_START_Y = 59;   // 右侧斜线起点纵坐标

  for (aa = 0; aa < ACROSS_LINE_LEN; aa++) {
    left_line[aa].y = L_LINE_START_Y - aa;
    left_line[aa].x = L_LINE_START_X - aa;
  }
  lcd_points(left_line, 30, RED);
  for (aa = 0; aa < ACROSS_LINE_LEN; aa++) {
    right_line[aa].y = R_LINE_START_Y - aa;
    right_line[aa].x = R_LINE_START_X + aa;
  }
  lcd_points(right_line, 30, RED);

  // 上部横线
  for (aa = 0; aa < 11; aa++) {
    up_line[aa].y = 25;
    up_line[aa].x = 34 + aa;
  }
  lcd_points(up_line, 12, GREEN);

  // 上部左侧横线
  for (aa = 0; aa < 12; aa++) {
    up_left_line[aa].y = 25 + aa;
    up_left_line[aa].x = 6;
  }
  lcd_points(up_left_line, 12, RED);
  for (aa = 0; aa < 12; aa++) {
    up_left_line[aa].y = 25;
    up_left_line[aa].x = aa;
  }
  lcd_points(up_left_line, 6, RED);

  // 上部右侧横线
  for (aa = 0; aa < 12; aa++) {
    up_right_line[aa].y = 25;
    up_right_line[aa].x = CAMERA_W - aa;
  }
  lcd_points(up_right_line, 12, RED);
  for (aa = 0; aa < 12; aa++) {
    up_right_line[aa].y = 25 + aa;
    up_right_line[aa].x = 74;
  }
  lcd_points(up_right_line, 6, RED);

  return 0;
}

/**
 * 检测环的存在
 * output:
 *   0 没有环, 1 左边有环, 2 右边有环
 */
int test_cycle(void) {
  int aa;
  int black_line_point = 0;  // 预设黑线位置 白色编码器 22

  // 检测图像两边下方垂直边线是否为白色，若是白色，则继续判断是否有环
  for (aa = 59; aa > 45; aa--) {
    if (img[aa][2] == Black_Point || img[aa][77] == Black_Point) {
      return 0;
    }
  }

  // 搜索左边黑点位置
  for (aa = 54; aa > 15; aa--) {
    if (img[aa][5] == Black_Point) {
      black_line_point = aa;
      break;
    }
  }

  if (black_line_point == 0) {
    return 0;
  }

  // 检测左边是否有环
  for (aa = 1; aa < 16; aa++) {
    // 检测左边直线
    if (img[black_line_point + 2][aa] != White_Point ||
        img[black_line_point - 1][aa] != Black_Point) {
      break;
    } else if (img[black_line_point + 5][74] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point - 5][74] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point][74] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point + 2][39] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point - 4][39] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point - 4][35] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point + 5][2] ==
                   White_Point) {  // 检测直线下是否黑色
      return 1;
    }
  }

  black_line_point = 0;

  // 右边搜索黑点位置
  for (aa = 54; aa > 15; aa--) {
    if (img[aa][74] == Black_Point) {
      black_line_point = aa;
      break;
    }
  }

  if (black_line_point == 0) {
    return 0;
  }

  // 检测右边直线
  for (aa = 1; aa < 16; aa++) {
    if (img[black_line_point + 2][79 - aa] != White_Point ||
        img[black_line_point - 1][79 - aa] != Black_Point) {
      return 0;
    } else if (img[black_line_point + 5][5] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point - 5][5] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point][5] ==
                   Black_Point &&  // 检测是否十字路口
               img[black_line_point + 2][40] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point - 4][40] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point - 4][44] ==
                   White_Point &&  // 检测中间是否白色
               img[black_line_point + 5][77] ==
                   White_Point) {  // 检测直线下是否黑色
      return 2;
    }
    return 0;
  }
  return 0;
}

/**
 * 出环检测
 * input:
 *   1 左边环, 2 右边环
 * output:
 *   0 还在环内, 1 出环
 */
int out_cycle_detection(int side) {
  int aa, bb, vertical_line;

  // 隔点识别是否下方图像是否全白
  for (aa = 58; aa > 15; aa--) {
    if (img[aa][40] == Black_Point) {
      vertical_line = aa;
      break;
    }
  }

  // 若白色区域太小则认为不是出环标志
  if (vertical_line > 32) {
    return 0;
  }

  for (aa = 1; aa < 14; aa++) {
    for (bb = 57; bb > vertical_line + 5; bb--)
      if (img[bb][aa*6 - 3] == Black_Point) {
        return 0;
      }
    }
  return 1;
}

/**
 * 斑马线检测
 * 取上方, 中央, 下方三条检测线
 * 如果上方和下方的检测线 有超过 38/40 个白点
 * 而将中央检测线分成[20,30),[30,40),[40,50),
 * [50,60), 四段每段里面由4/10个黑点，4/10个白
 * 点 就认为是斑马线
 *                  *******************
 *                  ****   *   *   ****
 *                  ***   =*= =*=   ***
 *                  **     *   *     **
 *                  *******************
 * output:
 *   0 没有斑马线, 1 检测到斑马线
 */
int zebra_detection(void) {
  int aa, bb;           // 循环索引
  int white_count = 0;  // 白点个数
  int black_count = 0;  // 黑点个数
  int line_up = 0;      // 上方检测线
  int line_low = 0;     // 下方检测线
  int line_center = 0;  // 中间检测线

  // 下方的白线有超过 38/40 个白点就认为是白色
  for (aa = 35; aa <= 45; aa++) {
    if (img[18][aa] == White_Point) {
      white_count++;
    }
    if (white_count > 6) {
      line_up = 1;
    }
  }

  // 下方的白线有超过 15/20 个白点就认为是白色
  white_count = 0;
  for (aa = 30; aa <= 50; aa++) {
    if (img[38][aa] == White_Point) {
      white_count++;
    }
    if (white_count > 15) {
      line_low = 1;
    }
  }

  // 将中央检测线分成[15,30),[30,45),[45,60)
  // 每段里面由5/15个黑点，5/15个白点
  white_count = 0;
  for (bb = 1; bb < 3; bb++) {
    for (aa = (bb - 1) * 15 + 27; aa < (bb * 15 + 27); aa++) {
      if (img[25][aa] == White_Point) {
        white_count++;
      }
      if (img[25][aa] == Black_Point) {
        black_count++;
      }
    }
    if (black_count >= 2 && white_count >= 2) {
      line_center++;
    }
    white_count = 0;
    black_count = 0;
  }

  // 上方, 下方检测线是白色, 中央检测线有三段斑马, 则认为是斑马线
  if (line_up == 1 && line_low == 1 && line_center > 1) {
    return 1;
  }

  return 0;
}

/**
 * 斑马线停车
 * output:
 *   0: 函数成功退出
 */
int pupil_stop(void) {
  uint32 stop_time = 0;  // 计时器读数 (ms)

  // 获取时间
  stop_time = control_handle.loop_time;

  // 解除 PID 电机控制
  moto_control(0, 0);
  moto_power(0, 0);

  // 先带动力滑行 1s
  while (control_handle.loop_time < stop_time + 1000) {
    only_deal_image();
    steer_control(0);
    moto_power(20, 20);
    // moto_back(40, 40);  // 刹车
  }

  // 关闭电机
  moto_power(0, 0);
  // moto_back(15, 15);  // 刹车

  // 再无动力滑行 3s
  while (control_handle.loop_time < stop_time + 3000) {
    only_deal_image();
    steer(pupil_correction(0));  // 使用小学生舵机控制
  }

  // 舵机回正
  steer(0);

  // 显示行车时间
  Site_t time_site = {50, 70};
  lcd_num_c(time_site, stop_time, GREEN, RED);

  return 0;
}

/**
 * 路况标志劫持函数
 * input:
 *   ROAD_PUPIL road_kind 路况
 * output:
 *   被篡改的路况
 */
ROAD_PUPIL road_kind_pirate(ROAD_PUPIL road_kind) {
  static ROAD_PUPIL keep_flag;  // 保存的flag
  static int cycle_step;        // 进环步骤
  int aa, bb;
  int count = 0;             // 入环交还控制权的flag

  // 如果遇到入环, 则存入标志
  if (road_kind == pLeftCycle || road_kind == pRightCycle) {
    keep_flag = road_kind;
    control_handle.cycle_side = road_kind - 3;
    control_handle.cycle_time = control_handle.loop_time;
    cycle_step = 1;
    return road_kind;
  } else if (road_kind == pOutCycle) {
    // 如果遇到出环标志而距离入环时间小于 1.5s
    keep_flag = pOutCycle;
    // 如果遇到出环识别标志则为 2
    cycle_step = 2;
  } else if (keep_flag == pOutCycle && road_kind != pOutCycle) {
    // 如果已经出环, 取消环标志
    control_handle.out_time = control_handle.loop_time;
    control_handle.cycle_side = 0;
  }

  // 边界两端小直线判断入环转交控制权
  if (keep_flag == pLeftCycle) {
    // 若进入左环, 则用小横线判断右边边界白色向黑色跳变 // ! 交还控制权的函数还需优化
    for (bb = 15; bb < 40; bb++) {
      for (aa = 0; aa < 5; aa++) {
        if (img[bb][78 - aa] == White_Point &&
            img[bb][68 - aa] == Black_Point) {
          count++;
        }
      }
      if (count >= 4) {
        // 如果已经跳变则交还控制权
        cycle_step = 0;
      }
    }
  } else if (keep_flag == pRightCycle) {
    // 若进入右环,则用小横线判断左边边界白色向黑色跳变
    for (bb = 15; bb < 40; bb++) {
      for (aa = 0; aa < 5; aa++) {
        if (img[bb][2 + aa] == White_Point && img[bb][11 + aa] == Black_Point) {
          count++;
        }
      }
      if (count >= 4) {
        // 如果已经跳变则交还控制权
        cycle_step = 0;
      }
    }
  }

  // 根据返回值决定是否转交控制权
  if (keep_flag == pLeftCycle && cycle_step == 1) {
    // 继续进左环, 不交还控制权
    return pLeftCycle;
  } else if (keep_flag == pRightCycle && cycle_step == 1) {
    // 继续进右环, 不交还控制权
    return pRightCycle;
  } else if (keep_flag == pOutside && cycle_step == 2) {
    // 继续出环, 不交还控制权
    return pOutCycle;
  } else {
    // 转交控制权
    keep_flag = road_kind;
    return road_kind;
  }
}

/**
 * pid 舵机修正函数
 * input:
 *   int target 与中线的目标累积偏差 车往左偏为正 右偏为负
 * output:
 *   0 正常退出
 */
int steer_control(int target) {
  int distance;  // 道路中线与图像中线的误差

  // 统计总偏差

  distance = road_info[0][30] + road_info[0][31] + road_info[0][32] +
             road_info[0][33] + road_info[0][34] + road_info[0][35] +
             road_info[0][36] + road_info[0][37] + road_info[0][38] +
             road_info[0][39] - 405;

#if WHICH_CAR
  // * GONG
  static PidStruct steerPID = {0.010, 0.000003, 0.01};  // 右电机PID控制量
#else
  // * SHOU
  static PidStruct steerPID = {0.010, 0.0000018, 0.01};  // 右电机PID控制量
#endif
  // 计算 PID 控制量
  pid_control(target, distance, &steerPID);

  // 设置控制量上下限
  if (steerPID.control > 15) {
    steerPID.control = 15;
  } else if (steerPID.control < -15) {
    steerPID.control = -15;
  }

  // 控制电机
  steer(steerPID.control);

  return 0;
}

/**
 * 中途会车操作函数
 * // 在 GONG 中超声测距中断调用
 * // 在 SHOU 中有蓝牙信息接收中断调用
 * ! 不能在中断中调用, 摄像头中断的优先级和 uart 的优先级相同, 会无法获取图像
 */
int meeting_car(void) {
  // 如果距离大于 1m 不会车 或者 没有收到会车信号
  if (control_handle.meeting_flag == 0) {
    return 0;
  }
  // 向 GONG 发送会车信号
  uart_putstr(BLUE_PORT, "mmmm");
  // 显示会车标志
  Site_t time_site = {25, 100};
  lcd_num_c(time_site, 1, GREEN, RED);

  // 保持 3s 靠边行驶
  while (control_handle.loop_time - control_handle.distance_time < 3000) {
    // 获取和处理图像
    deal_image();
    // 靠右行驶
    steer_control(80);
    moto_control(200, 200);
  }

  control_handle.distance = 3440;
  control_handle.meeting_flag = 0;

  if (control_handle.meeting_end == 1) {
    moto_control(0, 0);           // 关闭 PID 电机控制
    moto_power(0, 0);             // 关闭电机
    steer(0);                     // 舵机回正
    Site_t time_site = {50, 70};  // 显示行车时间位置
    lcd_num_c(time_site, control_handle.loop_time, GREEN, RED);  // 显示行车时间
    while (1)
      ;
  }

  return 0;
}

/**
 * 冲线时斑马线会车函数
 */
int meeting_stop(void) {
  // 如果对向车对向车已经出界 或者 单车运行
  // 则按照常规方式滑过斑马线
  if (control_handle.meeting_die == 1 || control_handle.meeting_hello == 0) {
    // 常规斑马线停车
    pupil_stop();
    return 1;
  }

  // 获取停车时间
  uint32 stop_time = control_handle.loop_time;

  // 解除 PID 电机控制
  moto_control(0, 0);
  moto_power(0, 0);

  // 发送等待信号
  uart_putstr(BLUE_PORT, "wwww");

  // 停车
  while (control_handle.loop_time < stop_time + 300) {
    only_deal_image();
    steer_control(0);
    moto_back(40, 40);
  }

  // 关闭电机
  moto_power(0, 0);

  // 舵机回正
  steer(0);

  // 等待对方车冲线或者出界
  while (control_handle.meeting_flag == 0 && control_handle.meeting_die == 0) {
    check_time();
  }

  // 获取时间
  stop_time = control_handle.loop_time;

  // 道左相逢
  while (control_handle.loop_time < stop_time + 2000) {
    deal_image();
    steer_control(80);
    moto_control(200, 200);
  }

  // 解除 PID 电机控制, 回正舵机
  moto_control(0, 0);
  moto_power(0, 0);
  steer(0);

  // 显示行车时间
  Site_t time_site = {50, 70};
  lcd_num_c(time_site, stop_time, GREEN, RED);

  return 0;
}

/**
 * 发车时会车
 */
int meeting_start(void) {
  // 检查对向车是否存在
  if (control_handle.meeting_hello == 0 || control_handle.meeting_flag == 0) {
    return 0;
  }
#if WHICH_CAR
  // * GONG
  uart_putstr(BLUE_PORT, "sssssss");
  // 等待对方车准备信号
  while (control_handle.meeting_start == 0)
    ;
  DELAY_MS(control_handle.start_wait_time);
  if (control_handle.start_wait_time > 1000) {
    return 0;
  }
#else
  // * SHOU
  // 等待对方车准备信号
  while (control_handle.meeting_start == 0)
    ;
  // 回复准备就绪信号
  uart_putstr(BLUE_PORT, "sssssss");
  DELAY_MS(50);
  DELAY_MS(control_handle.start_wait_time);
  if (control_handle.start_wait_time > 1000) {
    return 0;
  }
#endif

  // 直线发车
  steer(0);
  moto_control(200, 200);
  DELAY_MS(1500);

  // 清除会车标志, 防止发车时的超声超声数据对此后产生干扰
  control_handle.meeting_flag = 0;
  control_handle.distance = 3440;

  return 0;
}

/**
 * 近端斑马线识别
 * output:
 *   1 有斑马线, 0 没有斑马线
 */
int near_zebra(void) {
  // 斑马线特征组
  const int ZEBRA_UP = 30;      // 斑马线上部特征线
  const int ZEBRA_CENTER = 50;  // 斑马线中央特征线
  int zebra_up_flag = 0;        // 斑马线上部标志 1 白 0 黑
  int zebra_center_flag = 0;    // 斑马线中央标志 1 白 0 黑
  int aa = 0;
  int count = 0;

  // 检测斑马线上方是否为白色
  for (aa = road_info[1][ZEBRA_UP]; aa < road_info[2][ZEBRA_UP]; aa++) {
    count += img[ZEBRA_UP][aa];
  }

  if (count + 3 < road_info[2][ZEBRA_UP] - road_info[1][ZEBRA_UP]) {
    zebra_up_flag = 1;
  }

  count = 0;

  // 检测斑马线中央是否为白色
  for (aa = 20; aa < 60; aa++) {
    count += img[ZEBRA_CENTER][aa];
  }

  if (count > 10 && count < 30) {
    zebra_center_flag = 1;
  }

  count = 0;

  if (zebra_up_flag == 1 && zebra_center_flag == 1) {
    return 1;
  }

  return 0;
}