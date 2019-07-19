#ifndef PUPIL_MODEL_H_
#define PUPIL_MODEL_H_

#include <math.h>
#include <stdio.h>
#include "image.h"

// 路况枚举
typedef enum {
  pZbraStripe = 0,  // 斑马线
  pTurnLeft = 1,    // 左转
  pTurnRight = 2,   // 右转
  pStraight = 3,    // 直行
  pLeftCycle = 4,   // 左边环
  pRightCycle = 5,  // 右边环
  pOutside = 6,     // 出界
  pOutCycle = 7     // 出环
} ROAD_PUPIL;

int pupil_model(void);                              // 小学生模式主函数
float pupil_correction(int model);                  // 小学生舵机修正
ROAD_PUPIL pupil_road_judge(void);                  // 小学生路况判断
ROAD_PUPIL road_kind_pirate(ROAD_PUPIL road_kind);  // 路况劫持
int pupil_factor_line(void);                        // 显示特征线
int test_cycle(void);                               // 测试环的存在
int meeting_car(void);                              // 会车操作函数
int zebra_detection(void);                          // 斑马线检测
int pupil_stop(void);                               // 斑马线停车
int out_cycle_detection(int side);                  // 出环检测
float pupil_meeting(int model);                     // 光电会车检测
int steer_control(int target);                      // PID 舵机修正
int meeting_stop(void);                             // 会车停车
int meeting_start(void);                            // 会车发车
int near_zebra(void);                               // 近端斑马线检测

#endif