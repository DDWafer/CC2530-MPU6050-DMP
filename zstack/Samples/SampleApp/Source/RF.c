#include <ioCC2530.h>
#define uint unsigned int 
#define uchar unsigned char
// 红外38KHZ载波信号 占用资源： T3定时器通道0,1的输出比较功能 P1_6 P1_7 IO口

//void main(void)
//{
//  //系统时钟选择 32MHz  默认是16MHz
//  CLKCONCMD = 0;
//  while (CLKCONSTA != 0);// wait until stable
// 
void RF_on(void)
{
  //使用TIM3通道0，1输出比较功能  P1_6 P1_7 备用位置2
  PERCFG |= 0x01<<5; // 备用位置2 
  P1SEL |= 0x03<<6;  //P1_6 P1_7外设功能
  P1DIR |= 0x03<<6; // p1_6 P1_7 output
  
  //T3 setup
  T3CTL = 0;
  T3CTL |= 0x02<<5; // 4 分频
  T3CTL |= 0x02;   // 模计数 模式 0x00-->T3CC0
  
  T3CC0 = 210;  // ~38KHz
  T3CCTL0 = 0;
  T3CCTL0 |= 0x01<<6;// T3 CH0 IE 中断使能
  T3CCTL0 |= 0x01<<2; // compare mode
  T3CCTL0 |= 0x03<<3;     // 3: set compare-up, clear compare-down (up/down) or set compare, clear 0
  T3CCTL0 |= 0x07<<3;     // init output pin
  
  T3CC1 = T3CC0/3;
  T3CCTL1 = 0;
  T3CCTL1 |= 0x01<<6;  //T3 CH1 IE 通道1中断使能
  T3CCTL1 |= 0x01<<2; // compare mode
  T3CCTL1 |= 0x04<<3;  //4: 达到比较值时，状态切换， 达到0时，状态切换
  T3CCTL1 |= 0x07<<3;  //init output pin
  
  T3CTL |= 0x01<<2;  //计数值 清零
  T3CTL |= 0x01<<4;   //启动定时器
}
void RF_off(void)
{
    //使用TIM3通道0，1输出比较功能  P1_6 P1_7 备用位置2
  PERCFG &= (~0x01)<<5; // 备用位置2 
  P1SEL &= (~0x03)<<6;  //P1_6 P1_7外设功能
  //P1DIR |= (~0x03)<<6; // p1_6 P1_7 output
  P1_6=0;
  P1_7=0;
  //T3 setup
  T3CTL = 0x08;
  //恢复默认
  T3CC0 = 0;  // ~38KHz
  T3CCTL0 = 0x40;
  T3CC1=0;
  T3CCTL1=0x04;
}