/**
 *  MPU6050 使用IIC协议
 *  本程序拟定使用PTD8(SCL) PTD9(SDA)这两个端口
 *  使用K656模块的I2C0模块，
 *
 *  name:         MPU6050.c
 *  version       1.0
 *  Author:       Lee
 *  Date:         2019. 1.22
 * Env.:          IAR 7.8, WIN 10
 *  /      模块通道    端口          可选范围              建议
 *  I2C0_SCL_PIN    PTD8        // PTB0、PTB2、PTD8
 *  I2C0_SDA_PIN    PTD9        // PTB1、PTB3、PTD9
 *  备注：更改自ChengXuFan和某一开源MPU6050库
 *  设计参考了山外K66代码的MMA7455
 */
#include "include.h"

/**
 *  函数原型:
 *  功　　能:	    修改 指定设备 指定寄存器一个字节 中的多个位
 *  input:
 *          I2Cn_e    I2C模块(I2C0、I2C1)
 *          SlaveID   目标设备地址,从机地址(7位地址)
 *          reg	      寄存器地址
 *          bitStart  目标字节的起始位，注意是寄存器地址从高位到低位的方向
 *          length    位长度
 *          data      存放改变目标字节位的值
 *  output:
 *
 *  例：修改MPU6050的时钟源
 *      寄存器位置为:0x6B  一共有八位
 *      bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 *      要修改时钟源需要更改 bit2,bit1,bit0的数据
 *      bitStart记录2，length记录的修改的位数
 *
 *   内部函数，无需关注
 */
void i2c_write_bits(I2Cn_e i2cn, uint8 SlaveID, uint8 reg, uint8 bitStart,
                    uint8 length, uint8 data) {
  uint8 b;
  uint8 mask;
  b = i2c_read_reg(i2cn, SlaveID, reg);
  mask = (0xFF << (bitStart + 1)) | (0xFF >> ((8 - bitStart) + length - 1));
  data <<= (8 - length);
  data >>= (7 - bitStart);
  b &= mask;
  b |= data;
  i2c_write_reg(i2cn, SlaveID, reg, b);
}
/**
 *功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator                    MPU6050_CLOCK_INTERNAL
 * 1       | PLL with X Gyro reference              MPU6050_CLOCK_PLL_XGYRO
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference   MPU6050_CLOCK_PLL_EXT32K
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *
 */
void MPU6050_set_clock_source(uint8 source) {
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_PWR_MGMT_1,\
                 MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** 设置陀螺仪计（角速度计）最大量程
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_set_full_scale_gyro_range(uint8_t range) {
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_GYRO_CONFIG,\
                 MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH,\
                 range);
}

/**
 *函数原型:
 *功　　能:	    设置  MPU6050 加速度计的最大量程
 *MPU6050_ACCEL_FS_2 配置为正负2g
 */
void MPU6050_set_full_scale_accel_range(uint8_t range) {
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_CONFIG,\
                 MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,\
                 range);
}

/**
 *函数原型:
 *功      能:设置低通滤波器配置
 *   input:mode
 *   取值：
 *        MPU6050_DLPF_BW_98   低通滤波器 98HZ
 *
 * */
void MPU6050_set_DLPF_mode(uint8_t mode) {
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_CONFIG,\
                 MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_set_sleep_enabled(uint8 enabled){
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_PWR_MGMT_1,\
                 MPU6050_PWR1_SLEEP_BIT, 1, enabled);
}

void MPU6050_set_i2c_bypass_enabled(uint8 enabled){
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_INT_PIN_CFG,\
                 MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1, enabled);
}

void MPU6050_set_i2c_master_mode_enabled(uint8 enabled){
  i2c_write_bits(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_USER_CTRL,\
                 MPU6050_USERCTRL_I2C_MST_EN_BIT, 1, enabled);
}

/**
 *函数原型:
 *功　　能:	    初始化 	MPU6050 以进入可用状态。
 *
 */
void MPU6050_init(void) {
  //uint32 b;
  i2c_init(MPU6050_I2C, 400*1000);

  MPU6050_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);  //设置时钟
  MPU6050_set_DLPF_mode(MPU6050_DLPF_BW_98);  //设置低通滤波器，98Hz
  MPU6050_set_sleep_enabled(0);
  MPU6050_set_i2c_bypass_enabled(0);
  MPU6050_set_i2c_master_mode_enabled(1);
  MPU6050_set_full_scale_gyro_range(
      MPU6050_GYRO_FS_1000);  //陀螺仪最大量程 +-1000度每秒
  MPU6050_set_full_scale_accel_range(
      MPU6050_ACCEL_FS_2);                    //加速度度最大量程 +-2G
  
  DELAY_MS(500);
  MPU6050_zeroinit();
}
/**
 *函数原型:
 *功　　能:	    读取MPU6050双字节的数据
 */
int16 MPU6050_get_data(uint8 regL, uint8 regH) {
  uint8 H, L;
  //H = i2c_read_reg(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, regH);
  //L = i2c_read_reg(MPU6050_I2C, MPU6050_ADDRESS_AD0_LOW, regL);
  H = MPU6050_read_reg(regH);
  L = MPU6050_read_reg(regL);

  return ((H << 8) + L);
  //合成数据,有效数据10位，用两个8位数据分开存，
}


/**
 *函数原型:
 *功　　能:	    角速度去零漂
 */
void MPU6050_zeroinit(void){
  int i;
  float k = 1/32.8;
  for (i=0;i<25;i++){
    GYROZero_y +=  - MPU6050_get_y_gyro()*k;  
    GYROZero_x +=  - MPU6050_get_x_gyro()*k;
  }
  GYROZero_y = GYROZero_y/25;
  GYROZero_x = GYROZero_x/25;
}
    

