
/**
 * name:       menu.c
 * usage:      --
 * author:     [[Civiv[]
 * date:       2019-07-19 2:10:44
 * version:    1.0
 * Env.:       IAR 7.8, WIN 10
**/

#include "common.h"
#include "include.h"
#include "menu.h"
uint8 ISLCD_ON_FLAG = 0;
uint8 ElementBUFF[10]={0};

int welcome_page(void) {
 

  Site_t site_jijun1 = {10, 30};
  Site_t site_jijun2 = {10, 45};
  Site_t site_jijun3 = {10, 60};
  Site_t site_jijun4 = {10, 75};
  Site_t site_jijun5 = {10, 100};
  lcd_clear(RED);
  lcd_str_ench(site_jijun1, "��Ѿ���˧", BLUE, RED);
  lcd_str_ench(site_jijun2, " Ϊʲô������", BLUE, RED);
  lcd_str_ench(site_jijun3, "�ǲ������", BLUE, RED);
  lcd_str_ench(site_jijun4, "��ô��", BLUE, RED);
  lcd_str_ench(site_jijun5, "Ҫ��Ҫ����", BLUE, RED);
  DELAY_MS(3000);
  lcd_clear(RED);
  return 0;
}

void startmenu()
{   
    int select_number = 0;
    int select_flage = 0;  // �??�??按下标志
    KEY_MSG_t keymsg;      // 按键消息队列
    Site_t option1_size={10,15};
    Site_t option2_size = {10,option1_size.y+15};
    welcome_page();//彩蛋

    lcd_str_ench(option1_size, "��Ѿ���˧", RED, BLUE);
    lcd_str_ench(option2_size, "�ⲻ�����??", BLUE, RED);

        // 等待�??�??�??按下
    
    while (select_flage == 0) 
    {
        // 等待获取按键消息, get_key_msg 加括�??, 强制其比 keymsg.status 先执�??
        if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) 
        {
            // 根据按键改变数字
            switch (keymsg.key) {
            case KEY_U:
              select_number = (select_number + Num_of_mode - 1) % Num_of_mode;
              break;
            case KEY_D:
              select_number = (select_number + 1) % Num_of_mode;
              break;
            case KEY_B:
                select_flage = 1;
            default:
                break;
            };

            lcd_str_ch(option1_size, "��Ѿ���˧", RED, BLUE);
            lcd_str_ench(option2_size, "�ⲻ�����=", BLUE, RED);

            // LCD 做出相应改变
            switch (select_number) {
            case 0:
              lcd_str_ch(option2_size, "�ⲻ����", BLUE, RED);
              lcd_str_ch(option1_size, "��Ѿ�˧", RED, BLUE);
              break;
            case 1:
              
              lcd_str_ch(option2_size, "�ⲻ����", RED, BLUE);
              lcd_str_ch(option1_size, "��Ѿ�˧", BLUE, RED);
              break;
            
            default:
                break;
            };
        }
       
    }

    lcd_clear(RED);
   
    switch (select_number)
    {
        case 1:
          ISLCD_ON_FLAG = LCD_ON;
          break;
        case 2:
          ISLCD_ON_FLAG = LCD_OFF;
          break;
    default:
      break;
    };

#if PID_IS_CHANGE
    PIDjudgement();
#endif


}

void PIDjudgement(void)
{
  KEY_MSG_t keymsg;        // 按键消息队列
  Site_t site_tittle = {50, 20};  // 初�?�化当前变量名显示位�??
  Site_t site_number = {50,70};//初�?�化数字显示位置
  int kp_init_num = 2000;
  int ki_init_num = 2000;
  int kd_init_num = 2000;
  int location = 0;  // 选择位置 位置0为最低位
  int select_flag = 0;
  int digitals[NUMBER_LENGTH];
  uint32 temp_kp_num = kp_init_num;
  uint32 temp_ki_num = ki_init_num;
  uint32 temp_kd_num = kd_init_num;

  char buff[30] = "\0";  // 初�?�化文字缓冲�??
  
  //初�?�化参数数字拆分放进buf数组�??
  uint32 kp_buff[NUMBER_LENGTH+1];
  uint32 ki_buff[NUMBER_LENGTH+1];
  uint32 kd_buff[NUMBER_LENGTH+1];

  for (int aa = 0; aa < NUMBER_LENGTH; aa++)
  {
    kp_buff[aa] = temp_kp_num % 10;
    ki_buff[aa] = temp_ki_num % 10;
    kd_buff[aa] = temp_kd_num % 10;
    
    temp_kp_num /= 10;
    temp_ki_num /= 10;//�??�??后全部temp归零
    temp_kd_num /= 10;

    }
  

  lcd_str_ench(site_tittle, "KP��������", BLUE, RED);
  lcd_num_c(site_number, kp_init_num, BLUE, RED);

  while (select_flag == 0)
  {
    if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) 
    {
      switch (keymsg.key)//按键位置�??�??
      {
      case KEY_L:
        location = (location + 1) % NUMBER_LENGTH;
        show_select_location(location);
        break;
      case KEY_R:
        location = (location - 1 + NUMBER_LENGTH) % NUMBER_LENGTH;
        show_select_location(location);
        break;
      case KEY_U:
        kp_buff[location] = (kp_buff[location] + 1) % 10;
        break;
      case KEY_D:
        kp_buff[location] = (kp_buff[location] - 1 + 10) % 10;
        break;
      case KEY_B:
        select_flag = 1;
        break;
      default:
        break;
      };

      temp_kp_num = 0;

      //合并P参数
      for (int aa = NUMBER_LENGTH-1; aa >= 0; aa--) {
        temp_kp_num *= 10;
        temp_kp_num += kp_buff[aa];
      }

      kp_init_num = temp_kp_num;
      lcd_num_c(site_number, kp_init_num, BLUE, RED);
      temp_kp_num = 0;
    }

  }
  select_flag = 0;//ȷ�ϰ�������
  location = 0;
  //合并完成根据想�?�给的整数位和小树位进�?�划分�?�制给全局PID参数即可
  //此时的kp_init_num�??一�??四位整数�??
  {
    // eg://k1 = kp_init_num/100.00;将参数传递到直立�??的kp上；
  }

  lcd_str_ench(site_tittle, "KD", BLUE, RED);
  lcd_num_c(site_number, kd_init_num, BLUE, RED);

  while (select_flag == 0) {
    if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) {
      switch (keymsg.key)  //���ĸ�����������
      {
        case KEY_L:
          location = (location + 1) % NUMBER_LENGTH;
          show_select_location(location);
          break;
        case KEY_R:
          location = (location - 1 + NUMBER_LENGTH) % NUMBER_LENGTH;
          show_select_location(location);
          break;
        case KEY_U:
          kd_buff[location] = (kd_buff[location] + 1) % 10;

          break;
        case KEY_D:
          kd_buff[location] = (kd_buff[location] - 1 + 10) % 10;
          break;
        case KEY_B:
          select_flag = 1;
          break;
        default:
          break;
      };
      
      temp_kd_num = 0;

      //合并P参数
      for (int aa = NUMBER_LENGTH; aa >= 0; aa--) {
        //temp_kd_num += kd_buff[aa];
        //kd_buff[aa + 1] *= 10;
        temp_kd_num *= 10;
        temp_kd_num += kd_buff[aa];
        
      }

      kd_init_num = temp_kd_num;
      lcd_num_c(site_number, kd_init_num, BLUE, RED);
      temp_kd_num = 0;
    }
  }
  select_flag == 0;  //重新清零�??认键的flag
  {
    //参数用途这里赋�??

  }
}
/**
 * 指示选择的位�??
 * input:
 *   location: 显示 ^ 的位�??
 */
int show_select_location(int location) {
  Site_t site = {50, 85};
  char buff[NUMBER_LENGTH + 1] = "    ";
  buff[NUMBER_LENGTH - 1 - location] = '^';
  lcd_str(site, (uint8 *)buff, BLUE, RED);
  return 0;
}

/*
��·Ԫ�ض��в�����
����ĵ�һ�������ǵ�һ��Ԫ�ص�����ֵҲ����Ԫ�ص�������
*/

void Element_Entry_menu(void)
{
  Site_t site_tittle = {50,20};
  Site_t site_body = {50,70};

  KEY_MSG_t keymsg;
  //uint8 location = 0;//��һ������Ԫ�أ��������������Ϊ0������Ϊ��һ��Ԫ��
  uint8 select_flag = 0;
  uint8 Num_init = 2;
  //uint16 temp = 0;
  //uint8 *point;
  uint8 select_number = Num_init;
  lcd_str_ench(site_tittle,"��·Ԫ�ض�������",BLUE,RED);
  lcd_num_c(site_body,Num_init,BLUE,RED);

  while(select_flag == 0)
  {
    if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN)
      {
        switch(keymsg.key){
          case KEY_U:
            select_number = chang_element_number(select_number, keymsg);
            lcd_num_c(site_body, select_number, BLUE, RED);
            break; 
          case KEY_D:
            select_number = chang_element_number(select_number, keymsg);
            lcd_num_c(site_body,select_number,BLUE,RED);
            break;
          case KEY_B:
            select_flag = 1;
          default:
            break;

        };

      }
  }
  //��� i = 4,��buff���㿪ʼ������3��2��1��0�����ĸ�Ԫ�ء�
  for (int i = 0 ;i<select_number-1;i ++)
  {
    ElementBUFF[i]=select_number - i -1;
  }
}

uint8 chang_element_number(uint8 input, KEY_MSG_t keysta)
{
  switch(keysta.key)
  {
    case KEY_U:
      return (input + 1)%10;
      break;
    case KEY_D:
      return (input - 1 + 10)%10;
      break;
  };

}
