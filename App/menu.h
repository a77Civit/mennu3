
#ifndef MENU_H_
#define MENU_H_

#include "common.h"
#include "include.h"

#define NUMBER_LENGTH 4  // 小液晶按键可以改的操作数的位数
//根据你的模式的数量先更改模式的数量的宏，和手动添进模式数组的初始化里。
#define Num_of_mode 2
#define LCD_ON 1
#define LCD_OFF  0
#define PID_IS_CHANGE 1
void startmenu(void);
void PIDjudgement(void);
int show_select_location(int location);


// char Modeoption[Num_of_mode][12] = {"保底模式", "激情模式"};

extern uint8 ISLCD_ON_FLAG;
/*
typedef struct 
{
 const char *option_1 = "保底模式";
  const char *option_2 = "激情模式";
    
}menu_option;
*/
#endif