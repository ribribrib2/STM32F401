#include "I2C_jacy.h"
#include "ALIENTEK_BSP.h"
#include "Configuration.h"

u8 key_up=1;  //松开标志

/**********************************
* Function Name  : I2C_GPIO_Config
* Description    : I2C GPIO set 
* Input          : None
* Output         : None
* Return         : None

**********************************/
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
  GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;         //SCL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽  
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;         //SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;         //CFGb
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;         //RSTb
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	
}

void SDA_IN(void)//单片机输入数据
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
void SDA_OUT(void)//单片机输出数据
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void RST_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}


/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
   delay_us(4);
}

void delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
void I2C_Start(void)
{
	SDA_OUT();
	SDA_H;
	SCL_H;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_L;	 
}
void i2c_clock(void)
{
    SCL_L;//SCL=0
    I2C_delay();
    SCL_H;//SCL=1
    I2C_delay();
    SCL_L;//SCL=0
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SDA_OUT();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	delay_us(4);
	SDA_OUT();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;

	SDA_IN();
	delay_us(30);
	}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SDA_OUT();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	SDA_IN();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
//void I2C_WaitAck(void) 	 
//{
//	unsigned char i;
//	SDA_IN();			
//	I2C_delay();
//	I2C_delay();
//	I2C_delay();
//	I2C_delay();
//	I2C_delay();
//	I2C_delay();
//	SCL_H;
//	I2C_delay();
//	i=0x00F;
//    do
//    {
//    }
//    while(--i!=0);
//	SCL_L;
//	SDA_OUT();      
//}

void I2C_WaitAck(void)
{
	SDA_IN(); 
	while(Read_SDA == 1);
	SCL_H;
	delay_us(20);
//	while(Read_SDA == 0); 
	SCL_L;
}

/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void outbyt(unsigned char i2c_data) //单片机输出数据 数据从高位到低位//
{
	unsigned char i;
	SDA_OUT();
	SCL_L;
	for(i=0;i<8;i++)
	{
		if(i2c_data&0x80)
		{	
			SDA_H;//SDA=1
		}
		else 
		{
			SDA_L;//SDA=0
		}	
		i2c_data <<=1;
		i2c_clock();
	}
	SDA_H;
	SDA_IN();
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
u16 inbyt(void)//单片机读数据
{
	u16 i,i2c_data,TmpIn;
	SDA_IN();//SDA turn to input mode
	i2c_data=0;
	i=8;
	while(i--)
	{
		i2c_data<<=1;
		SCL_L;//SCL=0
		I2C_delay();
		SCL_H;//SCL=1
		TmpIn=SDA_DATA;
		TmpIn &=SDA;
		TmpIn >>=SDA_num;
		i2c_data|=TmpIn;
		I2C_delay();
	}
	SCL_L;//SCL=0
	return(i2c_data);
}

void RcvAck(void)
{
	SDA_IN();
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    SDA_IN();
	
}			  	

u8 KEY_Scan(void)
{	 
	u8 key = 0;
	if(key_up && (KEY2==1))
	{
		key = KEY2;
		delay_ms(10);
		key_up=0;
		if(KEY2==1)
		{
			return 3;
		}
	}
	if(KEY2==0)
		key_up=1; 	    
	return 0;
}

