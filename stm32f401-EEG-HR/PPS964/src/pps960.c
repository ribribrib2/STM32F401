/*
 * pps960.c
 *
 *  Created on: 2016年9月8日
 *      Author: cole
 */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "afe4404_hw.h"
#include "agc_V3_1_19.h"
#include "hqerror.h"
#include "pps960.h"
#include "IIC.h"
#include "main.h"
#include "dwt_stm32_delay.h"
#include "IIC.h"
#include "usart.h"

#define PPS960_ADDR (0xB0)

extern uint16_t lifeQhrm;
extern int8_t skin;
 
uint16_t acc_check=0;
uint16_t acc_check2=0;

uint8_t pps964_is_init = 0;

int8_t hr_okflag=false;
int8_t Stablecnt=0;

int8_t Unstablecnt=0;

int8_t HR_HRV_enable=0;//0=>HR;1=>HRV;2=>HR+HRV;

uint32_t displayHrm = 0;
uint32_t pps_count;
uint32_t pps_intr_flag=0;
int8_t accPushToQueueFlag=0;
extern uint16_t AccBuffTail;

void PPS_DELAY_MS(uint32_t ms)
{
		HAL_Delay(ms);
}

void pps960_Rest_SW(void)
{
    PPS960_writeReg(0,0x8);
    PPS_DELAY_MS(50);
}

void pps960_disable(void)
{
	  pps964_is_init = 0;
		HAL_GPIO_WritePin(PPS_EN_GPIO_Port,PPS_EN_Pin,GPIO_PIN_RESET);
		HAL_Delay(200);
}

void init_pps960_sensor(void)
{
	  pps964_is_init = 1;
		HAL_GPIO_WritePin(PPS_EN_GPIO_Port,PPS_EN_Pin,GPIO_PIN_SET);
		HAL_Delay(200);
    HAL_GPIO_WritePin(PPS_RESET_GPIO_Port,PPS_RESET_Pin,GPIO_PIN_RESET);
	  DWT_Delay_us(30);
		HAL_GPIO_WritePin(PPS_RESET_GPIO_Port,PPS_RESET_Pin,GPIO_PIN_SET);
	  DWT_Delay_us(30);
//	  pps960_Rest_SW();
 
    init_PPS960_register();
    PPS960_init();
		
}

extern uint8_t control;
uint8_t pps_test_flag=0;
//uint8_t pps960_init_flag = 0;
void pps960_sensor_task(void)
{
      pps_intr_flag = 0;
			if(acc_check){
							// 进入该函数获取原始数据
							ALGSH_retrieveSamplesAndPushToQueue();//read pps raw data
							//move ALGSH_dataToAlg(); to message queue loop. and then send message at here.
							ALGSH_dataToAlg();
			}
}

uint8_t cnt=0;
uint16_t lifeHR = 0;
uint16_t lifeskin = 0;
void pps960_sensor_task2(void)
{
		if(acc_check) {
//						sample=GetHRSampleCount();
						ClrHRSampleCount();
						cnt++;if(cnt>255)cnt=0;
						lifeQhrm = pps_getHR();
//						snrValue=PP_GetHRConfidence();//for snr check
						skin = PPS_get_skin_detect();
			
						//cut       -->    计数
						//lifeQhrm  -->    心率值
						//snrValue  -->    信噪比
						//sample    -->    原始数据采样率,现为25HZ
						//skin      -->    皮肤接触标志位,接触时为1,未接触为0
						if(skin == 0)
						{
							lifeQhrm = 0;
						}
//						mode1_tran_data[750] = lifeQhrm;
//						mode1_tran_data[751] = skin;
//						HAL_UART_Transmit(&huart2,senddata,2,0xff);
						displayHrm = lifeQhrm;// 
		}
}

void PPS960_writeReg(uint8_t regaddr, uint32_t wdata)
{
		Write_Configuration(regaddr,wdata);
} 

uint32_t PPS960_readReg(uint8_t regaddr)
{
	return Read_Configuration(regaddr);
}

//#endif


