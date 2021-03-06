#include "I2C.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 

//////////////////////////////////////////////////////////////////////////////////
 
//初始化IIC
//void IIC_Init(void)
//{					     
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
//	   
//	GPIO_InitStructure.GPIO_Pin = PIN_CLK|PIN_SDA;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //输出
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(IIC_GPIOx, &GPIO_InitStructure);
//	GPIO_SetBits(IIC_GPIOx,PIN_CLK|PIN_SDA); 	//PB13,PB14 输出高
//}
//产生IIC起始信号
void IIC_Start(void)
{

	IIC_SDA_H();	  	  
	IIC_CLK_H();
	Delay_us(4);
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	IIC_CLK_L();//钳住I2C总线，准备发送或接收数据 



}	  
//产生IIC停止信号
void IIC_Stop(void)
{

	IIC_CLK_L();
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	IIC_CLK_H(); 
	Delay_us(4);
	IIC_SDA_H();//发送I2C总线结束信号
	Delay_us(4);
	

}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	unsigned int ucErrTime=0;

	IIC_SDA_H();Delay_us(1);	   
	IIC_CLK_H();Delay_us(1);	 
	while(SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
//			printf("no ack\r\n");
			return 1;
		}
	}
	IIC_CLK_L();//时钟输出0 
	Delay_us(4);
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_CLK_L();

	IIC_SDA_L();
	Delay_us(4);
	IIC_CLK_H();
	Delay_us(4);
	IIC_CLK_L();
	Delay_us(4);
	IIC_SDA_H();
	

	
}
//产生NACK应答		    
void IIC_NAck(void)
{
	IIC_CLK_L();

	IIC_SDA_H();
	Delay_us(4);
	IIC_CLK_H();
	Delay_us(4);
	IIC_CLK_L();

	
}					 				     
  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    
    IIC_CLK_L();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_H();
		else
			IIC_SDA_L();
		txd<<=1; 	  
		Delay_us(5);   //对TEA5767这三个延时都是必须的
		IIC_CLK_H();
		Delay_us(5); 
		IIC_CLK_L();	
		Delay_us(5);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;

    for(i=0;i<8;i++ )
	{
        IIC_CLK_L(); 
        Delay_us(4);
				IIC_CLK_H();
        receive<<=1;
        if(SDA_READ())receive++;   
		Delay_us(2); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
//---------------------------------------------
int I2C_ReadRegister_9250(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	int ret,i;
	IIC_Start();
	IIC_Send_Byte(Address<<1);
	ret=IIC_Wait_Ack();//ret=1 failed
	if(ret)
	{
		printf("read no ack\r\n");
		IIC_Stop();
		return 1;
	}
	else
	{
		IIC_Send_Byte(RegisterAddr);
		IIC_Wait_Ack();
		IIC_Start();
		IIC_Send_Byte((Address<<1)|0x01);
		IIC_Wait_Ack();
		for(i=0;i<RegisterLen;i++)
		{
			if(i==(RegisterLen-1))
			{RegisterValue[i]=IIC_Read_Byte(0);}
			else
			{RegisterValue[i]=IIC_Read_Byte(1);}
		}
		IIC_Stop();
		return 0;
	}
	
}
int I2C_WriteRegister_9250(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	int ret,i;
	IIC_Start();
	IIC_Send_Byte(Address<<1);
	ret=IIC_Wait_Ack();//ret=1 failed
	if(ret)
	{
		printf("write no ack\r\n");
		return 1;
	}
	else
	{
		IIC_Send_Byte(RegisterAddr);
		IIC_Wait_Ack();
		for(i=0;i<RegisterLen;i++)
		{
			IIC_Send_Byte(RegisterValue[i]);
			IIC_Wait_Ack();			
		}
		IIC_Stop();
		return 0;
	}
}
	

	


























