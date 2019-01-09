#include <ioCC2530.h>
#include <delay.h>
#include <iic.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <math.h>
#include <stdio.h>
/*宏常量定义*/

#define uint unsigned int 
#define uchar unsigned char

#define IIC_SCL P1_4
#define IIC_SDA P1_3
#define IIC_INT2 P1_2
#define IIC_INT1 P1_1

/*宏函数定义*/
#define SDA_IN()  {P1DIR &= ~(0x01<<3);}  //IIC_SDA配置为输入脚 默认上拉
#define SDA_OUT() {P1DIR |= 0x01<<3; }
#define SCL_OUT() {P1DIR |= 0x01<<4; }
/*MPU6050配置参数*/
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Pitch,Roll,Yaw;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
/*函数声明*/
void Delay_us(uint t);
//void delay_ms(uint t);

/*函数定义*/
void Delay_us(uint t)   //延时 t us
{
  uint i = 0;
  for(i=0;i<t;i++)
  {
      asm("NOP");
  }
}
void delay_ms(uint t)  // 延时 t ms
{
  uint i = 0;
  for(i=0; i<t ; i++)
    Delay_us(1000);
}

//IIC协议配置
//初始化IIC
void IIC_Init_IO(void)
{
  SCL_OUT();
  SDA_OUT();
}

//IIC开始信号
void IIC_Start(void)
{
  SDA_OUT();
  IIC_SDA = 1;
  IIC_SCL = 1;
  Delay_us(4);
  IIC_SDA = 0;
  Delay_us(4);
  IIC_SCL = 0;
}
//IIC停止信号
void IIC_Stop(void)
{
  SDA_OUT();
  IIC_SCL = 0;
  IIC_SDA = 0;
  Delay_us(4);
  IIC_SCL = 1;
  IIC_SDA = 1;
  Delay_us(4);
}
//IIC 等待应答信号
// 返回值 ： 1：应答失败  0： 应答成功
uchar IIC_Wait_Ack(void )
{
  uint ErrTime = 0;
  SDA_IN();
  Delay_us(1);
  IIC_SCL=1;Delay_us(1);
  while(IIC_SDA)
  {
    ErrTime++;
    if(ErrTime > 250)
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL = 0;
  return 0;
}
//IIC产生ACK应答
void IIC_Ack(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  Delay_us(2);
  IIC_SCL = 1;
  Delay_us(2);
  IIC_SCL = 0;
}
//IIC 不产生Ack 应答
void IIC_Nack(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  Delay_us(2);
  IIC_SCL = 1;
  Delay_us(2);
  IIC_SCL = 0;
}
//IIC 发送一个字节  txd
void IIC_Send_Byte(uchar txd)
{
  uchar t;
  SDA_OUT();
  IIC_SCL = 0;  //拉低时钟信号，开始数据传输
  for(t=0;t<8;t++)
  {
    IIC_SDA = (txd & 0x80)>>7;
    txd <<= 1;
    Delay_us(2);
    IIC_SCL = 1;
    Delay_us(2);
    IIC_SCL = 0;
    Delay_us(2);
  }
}
//IIC 读取一个字节 
// 参数 ack: 1:发送Ack  0: 发送Nack
uchar IIC_Read_Byte(void)
{
  uchar i=0, receive=0;
  SDA_IN();
  for(i=0;i<8;i++)
  {
    IIC_SCL = 0;
    Delay_us(2);
    IIC_SCL = 1;
    receive <<= 1;
    if(IIC_SDA)
      receive++;
    Delay_us(1);
  }
//  if(!ack)
//    IIC_Nack();
//  else
//    IIC_Ack();
  return receive;
}
//ADXL345加速度传感器 配置与使用
#define ADXL_WRITE 0xD0
#define ADXL_READ  0xD1
//向ADXL345 特定寄存器中写入 值
//addr: 寄存器地址   val:写入的值
// ADXL345初始化
uchar ADXL345_Init(void)
{
  IIC_Init_IO();
  return 0;
}
uchar ADXL345_sleep(void)//拉低输出电压
{
  P1DIR |= (0x03)<<3;
  P1_4=0;
  P1_3=0;
  P1DIR &= (~0x03)<<1; // p1_6 P1_7 output
  return 0;
}
void ADXL345_WR_Reg(uchar addr, uchar val)
{
  IIC_Start();
  IIC_Send_Byte(ADXL_WRITE);
  IIC_Wait_Ack();
  IIC_Send_Byte(addr);
  IIC_Wait_Ack();
  IIC_Send_Byte(val);
  IIC_Wait_Ack();
  IIC_Stop();
}
//读取ADXL345指定寄存器值
//addr: 目标寄存器地址
//返回值：  目标寄存器值
uchar ADXL345_RD_Reg(uchar addr)
{
  uchar temp=0;
  IIC_Start();
  IIC_Send_Byte(ADXL_WRITE);
  IIC_Wait_Ack();
  IIC_Send_Byte(addr);
  IIC_Wait_Ack();
  IIC_Start();  // 重新启动
  IIC_Send_Byte(ADXL_READ);
  IIC_Wait_Ack();
  temp = IIC_Read_Byte();  // 0表示发送Nack  读取一个字节 ，不再继续读取
  IIC_Stop();
  return temp;
}
// USART0 串口0配置
void Usart0_Init(void)
{
  PERCFG = 0x00;  // 使用备用位置1
  P0SEL = 0x0c;  // P0_2 P0_3 为外设功能
  P2DIR &= ~0XC0;
  U0CSR |= 0x80;
  U0GCR |= 8; 
  U0BAUD |= 59;
  UTX0IF = 0; //UART0 TX 中断标志初始置位
}
// USART0发送 一个字符串数据
void UartTX_Send_String(uchar *Data,int len)
{
  int j;
  for(j=0;j<len;j++)
  {
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}
//void main(void)
//{
//  uchar temp[10] = {0x21,0};
//  /*系统时钟选择 32MHz  默认是16MHz*/
//  CLKCONCMD = 0;
//  //while (CLKCONSTA != 0);// wait until stable
//  
//  /* IO口配置 */
//  /* 外设初始化 */
//  
//  Usart0_Init();
//  ADXL345_Init();
//void ADXL345_read(void){
//  unsigned char temp[10] = {0x21,0};
//  temp[0] = ADXL345_RD_Reg(0x00);
//  //UartTX_Send_String(temp , 2);
//  delay_ms(1000);
//  ADXL345_WR_Reg(0x1e, 0x45);
//  temp[0] = ADXL345_RD_Reg(0x1e);
//  //UartTX_Send_String(temp , 2);
//  delay_ms(1000);
//  ADXL345_WR_Reg(0x1f, 0x1f);
//  temp[0] = ADXL345_RD_Reg(0x1f);
//  //UartTX_Send_String(temp , 2);
//  delay_ms(1000);
//  ADXL345_WR_Reg(0x20, 0x20);
//  temp[0] = ADXL345_RD_Reg(0x20);
//  //UartTX_Send_String(temp , 2);
//  delay_ms(1000);
//}
//}
extern int mpu_init(void);
void MPU6050_Init(void)
{
	int result=0;
	IIC_Init_IO();
	result=mpu_init();
	//printf("%u\n",result);
	if(!result)
	{	 		 
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))		//mpu_set_sensor
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//mpu_configure_fifo
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  		//mpu_set_sample_rate
		if(!dmp_load_motion_driver_firmware())   	  			//dmp_load_motion_driver_firmvare
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		    DMP_FEATURE_GYRO_CAL))		   	 					 //dmp_enable_feature
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	 			 //dmp_set_fifo_rate
		run_self_test();		//??
		if(!mpu_set_dmp_state(1));
	}
}
#define q30  1073741824.0f
extern short gyro[3], accel[3];
void MPU6050_Pose(void)
{
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 
	if(sensors & INV_WXYZ_QUAT )
	{
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;

		Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}
}

//对接DMP库
typedef unsigned char u8;
u8 I2C_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC_Start();
    //I2C_Send_Byte(addr << 1 | 0);//
		IIC_Send_Byte(0XD0);
    if (IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return 0;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++) 
	{
        IIC_Send_Byte(*data);
        if (IIC_Wait_Ack()) 
		{
            IIC_Stop();
            return 0;   //?áè?ê§°ü
        }
		data++;
    }
    IIC_Stop();
    return 1;
}


u8 I2C_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf)
{
    IIC_Start();
    //I2C_Send_Byte(addr << 1 | 0);
		IIC_Send_Byte(0Xd0);
    if (IIC_Wait_Ack())
	{
        IIC_Stop();
        return 0;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Start();
    //I2C_Send_Byte(addr << 1 | 1);
		IIC_Send_Byte(0XD1);
    IIC_Wait_Ack();
    while (len)
	{
        *buf = IIC_Read_Byte();
        if (len == 1)
            IIC_Nack();
        else
            IIC_Ack();
        buf++;
        len--;
    }
    IIC_Stop();
    return 1;
}


int I2C_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(I2C_Read_Buffer(addr,reg,len,buf))
		return 0;
	else
		return -1;
}

int I2C_Write(u8 addr, u8 reg, u8 len, u8* data)
{
 //       u8 *buf=NULL;
 //       u8 i=0;
//        for(i=0;i<len;i++)
//            printf("%d",data[i]);
	if(I2C_Write_Buffer(addr,reg,len,data))
        {
//          printf("%d",reg);
//          printf("**\n");
//          for(i=0;i<len;i++)
//            printf("%02X",data[i]);
//          printf("\n");
//          I2C_Read_Buffer(addr,reg,len,buf);
//          for(i=0;i<len;i++)
//            printf("%02X",buf[i]);
//          printf("\n");
		return 0;
        }
	else
        {
          printf("error");
		return -1;
        }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////