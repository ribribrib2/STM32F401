#ifndef  _I2C_jacy_H
#define  _I2C_jacy_H

#include "gpio.h"
#include "main.h"
#include "dwt_stm32_delay.h"

#define SCL_H         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
#define SCL_L         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
   
#define SDA_H         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
#define SDA_L         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

#define SDA_read      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)

#define 	I2C_READ	0xB1
#define 	I2C_WRITE	0xB0

void I2C_GPIO_Config(void);
void SDA_IN(void);
void SDA_OUT(void);
void RST_IN(void);
void delay_us(uint8_t time);
void delay_ms(uint8_t time);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void RcvAck(void);
void i2c_clock(void);
uint8_t inbyt(void);
void outbyt(unsigned char i2c_data);
void Write_Configuration(uint8_t Addr,uint32_t wdata);
uint32_t Read_Configuration(unsigned int Addr);

#endif
