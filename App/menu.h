
#ifndef MENU_H_
#define MENU_H_

#include "common.h"
#include "include.h"

#define NUMBER_LENGTH 4  // 小液晶按键可以改的操作数的位�?
//根据你的模式的数量先更改模式的数量的宏，和手动添进模式数组的初始化里�?
#define Num_of_mode 2
#define LCD_ON 1
#define LCD_OFF  0
#define PID_IS_CHANGE 0
void startmenu(void);
void PIDjudgement(void);
int show_select_location(int location);
void Element_Entry_menu(void) 
uint8 chang_element_number(uint8 input, KEY_MSG_t keysta);
extern uint8 ElementBUFF[10];

extern uint8 ISLCD_ON_FLAG;

#endif