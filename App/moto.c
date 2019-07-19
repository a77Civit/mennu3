
/**
 * name:       moto.c
 * usage:      --
 * author:     [[
 * date:       2018-07-14 09:09:00
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
 */

#include "common.h"
#include "include.h"
#include "time.h"


/**
 * 舵机和电机初始化
 * input:
 * output:
 *   0: 函数成功退出
 */
int power_init(void) {
  // 初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 100,
               MOTOR1_PWM_IO);  //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 100,
               MOTOR2_PWM_IO);  //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 100,
               MOTOR3_PWM_IO);  //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 100,
               MOTOR4_PWM_IO);  //初始化 电机 PWM

  // 电机IO管脚配置
  gpio_init(MOTOR1_IO, GPO, LOW);
  gpio_init(MOTOR2_IO, GPO, LOW);
  gpio_init(MOTOR3_IO, GPO, LOW);
  gpio_init(MOTOR4_IO, GPO, LOW);

  return 0;
}
