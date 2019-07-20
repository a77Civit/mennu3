
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
  lcd_str_ench(site_jijun1, "李佳军最帅", BLUE, RED);
  lcd_str_ench(site_jijun2, " 为什么会这样", BLUE, RED);
  lcd_str_ench(site_jijun3, "是不是真的", BLUE, RED);
  lcd_str_ench(site_jijun4, "怎么办", BLUE, RED);
  lcd_str_ench(site_jijun5, "要不要退赛", BLUE, RED);
  DELAY_MS(3000);
  lcd_clear(RED);
  return 0;
}

void startmenu()
{   
    int select_number = 0;
    int select_flage = 0;  // 锟??锟??鎸変笅鏍囧織
    KEY_MSG_t keymsg;      // 鎸夐敭娑堟伅闃熷垪
    Site_t option1_size={10,15};
    Site_t option2_size = {10,option1_size.y+15};
    welcome_page();//褰╄泲

    lcd_str_ench(option1_size, "李佳军最帅", RED, BLUE);
    lcd_str_ench(option2_size, "这不是真的??", BLUE, RED);

        // 绛夊緟锟??锟??锟??鎸変笅
    
    while (select_flage == 0) 
    {
        // 绛夊緟鑾峰彇鎸夐敭娑堟伅, get_key_msg 鍔犳嫭锟??, 寮哄埗鍏舵瘮 keymsg.status 鍏堟墽锟??
        if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) 
        {
            // 鏍规嵁鎸夐敭鏀瑰彉鏁板瓧
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

            lcd_str_ch(option1_size, "李佳军最帅", RED, BLUE);
            lcd_str_ench(option2_size, "这不是真的=", BLUE, RED);

            // LCD 鍋氬嚭鐩稿簲鏀瑰彉
            switch (select_number) {
            case 0:
              lcd_str_ch(option2_size, "这不是真", BLUE, RED);
              lcd_str_ch(option1_size, "李佳军帅", RED, BLUE);
              break;
            case 1:
              
              lcd_str_ch(option2_size, "这不是真", RED, BLUE);
              lcd_str_ch(option1_size, "李佳军帅", BLUE, RED);
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
  KEY_MSG_t keymsg;        // 鎸夐敭娑堟伅闃熷垪
  Site_t site_tittle = {50, 20};  // 鍒濓拷?锟藉寲褰撳墠鍙橀噺鍚嶆樉绀轰綅锟??
  Site_t site_number = {50,70};//鍒濓拷?锟藉寲鏁板瓧鏄剧ず浣嶇疆
  int kp_init_num = 2000;
  int ki_init_num = 2000;
  int kd_init_num = 2000;
  int location = 0;  // 閫夋嫨浣嶇疆 浣嶇疆0涓烘渶浣庝綅
  int select_flag = 0;
  int digitals[NUMBER_LENGTH];
  uint32 temp_kp_num = kp_init_num;
  uint32 temp_ki_num = ki_init_num;
  uint32 temp_kd_num = kd_init_num;

  char buff[30] = "\0";  // 鍒濓拷?锟藉寲鏂囧瓧缂撳啿锟??
  
  //鍒濓拷?锟藉寲鍙傛暟鏁板瓧鎷嗗垎鏀捐繘buf鏁扮粍锟??
  uint32 kp_buff[NUMBER_LENGTH+1];
  uint32 ki_buff[NUMBER_LENGTH+1];
  uint32 kd_buff[NUMBER_LENGTH+1];

  for (int aa = 0; aa < NUMBER_LENGTH; aa++)
  {
    kp_buff[aa] = temp_kp_num % 10;
    ki_buff[aa] = temp_ki_num % 10;
    kd_buff[aa] = temp_kd_num % 10;
    
    temp_kp_num /= 10;
    temp_ki_num /= 10;//锟??锟??鍚庡叏閮╰emp褰掗浂
    temp_kd_num /= 10;

    }
  

  lcd_str_ench(site_tittle, "KP参数调整", BLUE, RED);
  lcd_num_c(site_number, kp_init_num, BLUE, RED);

  while (select_flag == 0)
  {
    if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) 
    {
      switch (keymsg.key)//鎸夐敭浣嶇疆锟??锟??
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

      //鍚堝苟P鍙傛暟
      for (int aa = NUMBER_LENGTH-1; aa >= 0; aa--) {
        temp_kp_num *= 10;
        temp_kp_num += kp_buff[aa];
      }

      kp_init_num = temp_kp_num;
      lcd_num_c(site_number, kp_init_num, BLUE, RED);
      temp_kp_num = 0;
    }

  }
  select_flag = 0;//确认按键亲临
  location = 0;
  //鍚堝苟瀹屾垚鏍规嵁鎯筹拷?锟界粰鐨勬暣鏁颁綅鍜屽皬鏍戜綅杩涳拷?锟藉垝鍒嗭拷?锟藉埗缁欏叏灞�PID鍙傛暟鍗冲彲
  //姝ゆ椂鐨刱p_init_num锟??涓�锟??鍥涗綅鏁存暟锟??
  {
    // eg://k1 = kp_init_num/100.00;灏嗗弬鏁颁紶閫掑埌鐩寸珛锟??鐨刱p涓婏紱
  }

  lcd_str_ench(site_tittle, "KD", BLUE, RED);
  lcd_num_c(site_number, kd_init_num, BLUE, RED);

  while (select_flag == 0) {
    if ((get_key_msg(&keymsg) == 1) && keymsg.status == KEY_DOWN) {
      switch (keymsg.key)  //是哪个按键摁下了
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

      //鍚堝苟P鍙傛暟
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
  select_flag == 0;  //閲嶆柊娓呴浂锟??璁ら敭鐨刦lag
  {
    //鍙傛暟鐢ㄩ�旇繖閲岃祴锟??

  }
}
/**
 * 鎸囩ず閫夋嫨鐨勪綅锟??
 * input:
 *   location: 鏄剧ず ^ 鐨勪綅锟??
 */
int show_select_location(int location) {
  Site_t site = {50, 85};
  char buff[NUMBER_LENGTH + 1] = "    ";
  buff[NUMBER_LENGTH - 1 - location] = '^';
  lcd_str(site, (uint8 *)buff, BLUE, RED);
  return 0;
}

/*
道路元素队列并输入
输入的第一个数字是第一个元素的特征值也是总元素的数量。
*/

void Element_Entry_menu(void)
{
  Site_t site_tittle = {50,20};
  Site_t site_body = {50,70};

  KEY_MSG_t keymsg;
  //uint8 location = 0;//第一个输入元素，即数组的索引零为0，最左为第一个元素
  uint8 select_flag = 0;
  uint8 Num_init = 2;
  //uint16 temp = 0;
  //uint8 *point;
  uint8 select_number = Num_init;
  lcd_str_ench(site_tittle,"道路元素队列输入",BLUE,RED);
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
  //如果 i = 4,则buff从零开始依次是3，2，1，0，共四个元素。
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
