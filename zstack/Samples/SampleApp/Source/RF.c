#include <ioCC2530.h>
#define uint unsigned int 
#define uchar unsigned char
// ����38KHZ�ز��ź� ռ����Դ�� T3��ʱ��ͨ��0,1������ȽϹ��� P1_6 P1_7 IO��

//void main(void)
//{
//  //ϵͳʱ��ѡ�� 32MHz  Ĭ����16MHz
//  CLKCONCMD = 0;
//  while (CLKCONSTA != 0);// wait until stable
// 
void RF_on(void)
{
  //ʹ��TIM3ͨ��0��1����ȽϹ���  P1_6 P1_7 ����λ��2
  PERCFG |= 0x01<<5; // ����λ��2 
  P1SEL |= 0x03<<6;  //P1_6 P1_7���蹦��
  P1DIR |= 0x03<<6; // p1_6 P1_7 output
  
  //T3 setup
  T3CTL = 0;
  T3CTL |= 0x02<<5; // 4 ��Ƶ
  T3CTL |= 0x02;   // ģ���� ģʽ 0x00-->T3CC0
  
  T3CC0 = 210;  // ~38KHz
  T3CCTL0 = 0;
  T3CCTL0 |= 0x01<<6;// T3 CH0 IE �ж�ʹ��
  T3CCTL0 |= 0x01<<2; // compare mode
  T3CCTL0 |= 0x03<<3;     // 3: set compare-up, clear compare-down (up/down) or set compare, clear 0
  T3CCTL0 |= 0x07<<3;     // init output pin
  
  T3CC1 = T3CC0/3;
  T3CCTL1 = 0;
  T3CCTL1 |= 0x01<<6;  //T3 CH1 IE ͨ��1�ж�ʹ��
  T3CCTL1 |= 0x01<<2; // compare mode
  T3CCTL1 |= 0x04<<3;  //4: �ﵽ�Ƚ�ֵʱ��״̬�л��� �ﵽ0ʱ��״̬�л�
  T3CCTL1 |= 0x07<<3;  //init output pin
  
  T3CTL |= 0x01<<2;  //����ֵ ����
  T3CTL |= 0x01<<4;   //������ʱ��
}
void RF_off(void)
{
    //ʹ��TIM3ͨ��0��1����ȽϹ���  P1_6 P1_7 ����λ��2
  PERCFG &= (~0x01)<<5; // ����λ��2 
  P1SEL &= (~0x03)<<6;  //P1_6 P1_7���蹦��
  //P1DIR |= (~0x03)<<6; // p1_6 P1_7 output
  P1_6=0;
  P1_7=0;
  //T3 setup
  T3CTL = 0x08;
  //�ָ�Ĭ��
  T3CC0 = 0;  // ~38KHz
  T3CCTL0 = 0x40;
  T3CC1=0;
  T3CCTL1=0x04;
}