
#ifndef __MOTO_H__
#define __MOTO_H__

#include "common.h"
#include "include.h"

// 强调: 一个模块同一时间只能干一个活, 而且频率必须是相同的.
// FTM是多功能的定时器, 可实现多种功能.
// 但同一时间, 一个 FTM0 拿去 PWM 输出, 就不要再用来做正交解码, 输入捕捉,
// 或者其他事情. 而且, FTM0 的通道0输出频率为 20k, 通道1也必须, 只能输出 20k 的
// pwm.

// 配置 舵机
#if 1
#define S3010_FTM FTM3
#define S3010_CH FTM_CH0
#define S3010_HZ (100)
#define S3010_PIN FTM3_CH0_PIN
#else
#define S3010_FTM FTM1
#define S3010_CH FTM_CH0
#define S3010_HZ (100)
#define S3010_PIN FTM1_CH0_PIN
#endif

// 配置 电机IO管脚
#define MOTOR1_IO PTD15
#define MOTOR2_IO PTA19
#define MOTOR3_IO PTA5
#define MOTOR4_IO PTA24

// 配置 电机PWM
#define MOTOR_FTM FTM0
#define MOTOR1_PWM FTM_CH3
#define MOTOR2_PWM FTM_CH4
#define MOTOR3_PWM FTM_CH5
#define MOTOR4_PWM FTM_CH6

// 配置 电机PWM管脚
#define MOTOR1_PWM_IO FTM0_CH3_PIN
#define MOTOR2_PWM_IO FTM0_CH4_PIN
#define MOTOR3_PWM_IO FTM0_CH5_PIN
#define MOTOR4_PWM_IO FTM0_CH6_PIN

// 配置电机频率
#define MOTOR_HZ 21000  // (21*1000)
#define MOTOR_DUTY 80

int power_init(void);                                       // 动力初始化

// 内联函数
inline int moto_power(float left, float right);  // 电机 PWM 控制

/** base
 * 电机正反转
 * input:
 *   left:  左电机 PWM 占空比
 *   right: 右电机 PWM 占空比
 * output:
 *   0: 函数成功退出
 */
inline int moto_power(float left, float right) {
  // 车模向后跑
  left = -left;
  right = -right;
  // 控制左后轮电机
  ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, (left < 0) ? 0 : left);
  ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, (left < 0) ? (-left) : 0);

  // 控制右后轮电机
  ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, (right < 0) ? (-right) : 0);
  ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, (right < 0) ? 0 : right);

  return 0;
}



#endif  // __MOTO_H__