#ifndef _iic_H
#define _iic_H

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


#define I2C_SCL_H GPIO_SetBits(GPIO_I2C,I2C_SCL)
#define I2C_SCL_L GPIO_ResetBits(GPIO_I2C,I2C_SCL)

#define I2C_SDA_H GPIO_SetBits(GPIO_I2C,I2C_SDA)
#define I2C_SDA_L GPIO_ResetBits(GPIO_I2C,I2C_SDA)

/* 声明全局函数 */
void I2C_Init_IO(void);
//void I2C_SDA_OUT(void);
//void I2C_SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
uchar   I2C_Wait_Ack(void);
void I2C_Send_Byte(uchar txd);
uchar   I2C_Read_Byte(void);

#endif
