#include "IIC.h"

/**********************************
* Function Name  : I2C_GPIO_Config
* Description    : I2C GPIO set 
* Input          : None
* Output         : None
* Return         : None

**********************************/
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();	
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;            // PA8 --> SCL
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = GPIO_PIN_9;            // PC9 --> SDA
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SDA_IN(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}
void SDA_OUT(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();	
	
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins*/
  GPIO_InitStruct.Pin = GPIO_PIN_9;            // PC9 --> SDA
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void delay_ms(uint8_t time)
{	
   HAL_Delay(time); 
}
void delay_us(uint8_t time)
{
	 DWT_Delay_us(time);
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
	 delay_us(2);
	 SDA_L;
	 delay_us(2);
	 SCL_L;	 
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
	 delay_us(4);
	 SCL_H;
	 delay_us(4);
	 SDA_H;
	 delay_us(4);
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
	delay_us(4);
	SCL_H;
	delay_us(4);
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
	delay_us(4);
	SCL_H;
	delay_us(4);
	SCL_L;
	SDA_IN();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master wait slave ACK
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
uint8_t I2C_WaitAck(void)
{
	uint16_t tempTime = 0;
	SDA_IN(); 
	
	while(SDA_read)
	{
		tempTime++;
		if(tempTime>550)
		{
			I2C_Stop();
			return 1;
		}	 
	}
	
	SCL_H;
	delay_us(4);
	SCL_L;

	return 0;
}
/*******************************************************************************
* Function Name  : I2C_Clock
* Description    : Master output CLOCK
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Clock(void)
{
   SCL_L;//SCL=0
   delay_us(4);
   SCL_H;//SCL=1
   delay_us(4);
	 SCL_L;//SCL=0
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void outbyt(unsigned char i2c_data)
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
		I2C_Clock();
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
uint8_t inbyt(void)
{
	uint8_t i,i2c_data;
	SDA_IN();//SDA turn to input mode
	i2c_data=0;
	i=8;
	while(i--)
	{
		i2c_data<<=1;
		SCL_L;//SCL=0
		delay_us(4);
		SCL_H;//SCL=1
		i2c_data|= SDA_read;
		delay_us(4);
	}
	SCL_L;//SCL=0
	return(i2c_data);
}

void Write_Configuration(uint8_t Addr,uint32_t wdata)
{
	uint8_t temp[3];
	uint32_t wd=wdata;
	temp[0]=(wd>>16) & 0xff;
	temp[1]=(wd>>8) & 0xff;
	temp[2]=wd & 0xff;
	
	I2C_Start();
	outbyt(I2C_WRITE);       
	I2C_WaitAck();
  outbyt(Addr);	           
	I2C_WaitAck();
	
	outbyt(temp[0]);   
	I2C_WaitAck();
	outbyt(temp[1]);   
	I2C_WaitAck();
	outbyt(temp[2]);   
	I2C_WaitAck();
	
	I2C_Stop();
}

uint32_t Read_Configuration(unsigned int Addr)
{
	char i,Data_Read[3];
	uint32_t data;
	I2C_Start();
	outbyt(I2C_WRITE);   //WRITE
	I2C_WaitAck();
  outbyt(Addr);	       
	I2C_WaitAck();

	I2C_Start();
	outbyt(I2C_READ);    //READ
	I2C_WaitAck();
	Data_Read[0] = inbyt();
	I2C_Ack();
	Data_Read[1] = inbyt();
	I2C_Ack();
	Data_Read[2] = inbyt();
	I2C_NoAck();
	I2C_Stop();
	
	data = Data_Read[0] << 16 | Data_Read[1] << 8 | Data_Read[2];
	return data;
}

