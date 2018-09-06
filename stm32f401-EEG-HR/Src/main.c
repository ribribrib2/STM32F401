/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "dwt_stm32_delay.h"

#include "ads1x9x_hal_driver.h"

#include "IIC.h"
#include "pps960.h"
#include "afe4404_hw.h"
#include "agc_V3_1_19.h"
#include "hqerror.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[8] = {0};           
uint8_t ADCData[165] = {0};           

uint8_t mode1_tran_data[167] = {0};   
uint8_t mode2_tran_data[17] = {0};    
uint8_t mode3_tran_data[21] = {0};     

uint8_t dataIsOk = 0;          
uint8_t sensor_open = 0;             
uint8_t sensor_is_init = 0;           

extern uint8_t ads1291_is_init;        
extern uint16_t acc_check;            

uint16_t lifeQhrm = 0;           
int8_t skin = 0;                   
uint8_t Fatigue = 0;               

uint16_t HeartPack1 = 0;            
uint16_t HeartPack2 = 0;            
uint16_t HeartPack3 = 0;              

uint8_t lifeQhrmMax = 85;
uint8_t FatigueMax = 80;
uint8_t AlarmType = 0;

uint8_t Alarm_status = 0;
uint8_t work_mode = 0;

void pps960_sensor_task(void);
void pps960_sensor_task2(void);
void sensor_switch(void);
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	if(DWT_Delay_Init())
  {
    Error_Handler(); /* Call Error Handler */
  }
	HAL_UART_Receive_IT(&huart2, aRxBuffer, 8);
	I2C_GPIO_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		sensor_switch();
		if(dataIsOk == 1 && work_mode == 1)
		{
			HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_SET);
			HAL_Delay(5);
			HAL_UART_Transmit(&huart2,mode1_tran_data,167,0xff);
			dataIsOk = 0;
			HAL_Delay(5);
			HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_RESET);
		}
		if(dataIsOk == 1 && work_mode == 2)
		{
		  HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_SET);
			HAL_Delay(5);
			HAL_UART_Transmit(&huart2,mode2_tran_data,17,0xff);
			dataIsOk = 0;
			HAL_Delay(5);
			HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_RESET);
		}
		if(Alarm_status == 1 && work_mode == 2)
		{
		  HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_SET);
			HAL_Delay(5);
			HAL_UART_Transmit(&huart2,mode3_tran_data,21,0xff);
			Alarm_status = 0;
			HAL_Delay(5);
			HAL_GPIO_WritePin(DATA_READY_GPIO_Port,DATA_READY_Pin,GPIO_PIN_RESET);
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t cnt = 0;               
	static uint16_t TimeNum = 0;          
	uint8_t Rx[6] = {0};     						
	uint32_t EEG_Data = 0;	              
	uint8_t LOFF_State = 0;               
	uint8_t Data[3] = {0};               
	if(ads1291_is_init == 1)
	{
		ADS_ReadData(Rx,6);

		EEG_Data = ((Rx[3]*0xFFFFFF)+(Rx[4]*0xFFFF)+Rx[5]*0xFF+0x80000000)>>8;
		Data[0] = EEG_Data >> 16;
		Data[1] = EEG_Data >> 8 % 0xFF;
		Data[2] = EEG_Data & 0xFE;
		
		memcpy((ADCData + cnt * 3),Data,3);
		cnt ++;
		
		if(cnt == 50)
		{
			LOFF_State = ((Rx[0]<<4) & 0x10) | ((Rx[1] & 0x80)>>4);
			cnt = 0;
			TimeNum ++;
			HeartPack1 ++;
			
			mode1_tran_data[0] = 0xBB;
			mode1_tran_data[1] = 0xBB;
			mode1_tran_data[2] = 0xBB;
			mode1_tran_data[3] = 0x00;
			mode1_tran_data[4] = 0x9F;  
			mode1_tran_data[5] = HeartPack1 >> 8;
			mode1_tran_data[6] = HeartPack1 & 0xFF;
			mode1_tran_data[7] = 0x01;
			mode1_tran_data[8] = skin;
			mode1_tran_data[9] = lifeQhrm;
			mode1_tran_data[10] = 0x02;
			mode1_tran_data[11] = Fatigue;
			mode1_tran_data[12] = 0x03;
			mode1_tran_data[13] =  LOFF_State;   
			
			memcpy(mode1_tran_data+14,ADCData,150);
			memset(ADCData,0,150);
			
			mode1_tran_data[164] = 0xFF;
			mode1_tran_data[165] = 0xFF;    
			mode1_tran_data[166] = 0xFF;

			if(work_mode == 1)
			{
				dataIsOk = 1;	
			}
		}	
		if(TimeNum == 1500)
		{
			TimeNum = 0;
			HeartPack2 ++;
			
			mode2_tran_data[0] = 0xBB;
			mode2_tran_data[1] = 0xBB;
			mode2_tran_data[2] = 0xBB;
			mode2_tran_data[3] = 0x00;
			mode2_tran_data[4] = 0x09;  
			mode2_tran_data[5] = HeartPack2 >> 8;
			mode2_tran_data[6] = HeartPack2 & 0xFF;
			mode2_tran_data[7] = 0x01;
			mode2_tran_data[8] = skin;
			mode2_tran_data[9] = lifeQhrm;
			mode2_tran_data[10] = 0x02;
			mode2_tran_data[11] = Fatigue;
			mode2_tran_data[12] = 0x03;
			mode2_tran_data[13] =  LOFF_State;   
			mode2_tran_data[14] = 0xFF;
			mode2_tran_data[15] = 0xFF;
			mode2_tran_data[16] = 0xFF;
			if(work_mode == 2)
			{
				dataIsOk = 1;	
			}
		}	
		if(lifeQhrm > lifeQhrmMax || Fatigue > FatigueMax)    
		{
			HeartPack3 ++;
			if(lifeQhrm > lifeQhrmMax)
			{
				AlarmType = 0x01;
			}
			if(Fatigue > FatigueMax)
			{
				AlarmType = 0x02;
			}
			
			mode3_tran_data[0] = 0xBB;
			mode3_tran_data[1] = 0xBB;
			mode3_tran_data[2] = 0xBB;
			mode3_tran_data[3] = 0x00;
			mode3_tran_data[4] = 0x0D;  
			mode3_tran_data[5] = HeartPack3 >> 8;
			mode3_tran_data[6] = HeartPack3 & 0xFF;
			mode3_tran_data[7] = 0x00;
			mode3_tran_data[8] = AlarmType;
			mode3_tran_data[9] = lifeQhrmMax;
			mode3_tran_data[10] = FatigueMax;
			mode3_tran_data[11] = 0x01;
			mode3_tran_data[12] = skin;
			mode3_tran_data[13] = lifeQhrm;
			mode3_tran_data[14] = 0x02;
			mode3_tran_data[15] = Fatigue;
			mode3_tran_data[16] = 0x03;
			mode3_tran_data[17] = LOFF_State;    
			mode3_tran_data[18] = 0xFF;
			mode3_tran_data[19] = 0xFF;
			mode3_tran_data[20] = 0xFF; 

			Alarm_status = 1;
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{
		// 获取心率数值
		pps960_sensor_task2();  //1s触发一次
	}
	if (htim->Instance == htim2.Instance)
	{
		// 获取心率原始数据
		pps960_sensor_task();  //40ms触发一次
	}
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t AT_MODE0[8] = "AT+MODE0";	
	uint8_t AT_MODE1[8] = "AT+MODE1";
	uint8_t AT_MODE2[8] = "AT+MODE2";
	if(huart->Instance == USART2)  
	{   
		HAL_UART_Receive_IT(&huart2,aRxBuffer,8); 
		if (memcmp(AT_MODE0, aRxBuffer, 8) == 0)
		{
			  sensor_open = 0;
			  work_mode = 0;	
				printf("OK+MODE0");
		}
		if (memcmp(AT_MODE1, aRxBuffer, 8) == 0)
		{
			  sensor_open = 1;
			  work_mode = 1;
				printf("OK+MODE1");
		}
		if (memcmp(AT_MODE2, aRxBuffer, 8) == 0)
		{
			  sensor_open = 1;
			  work_mode = 2;
				printf("OK+MODE2");
		}
	}
}
void sensor_switch(void)
{
		if(sensor_open == 1 && sensor_is_init == 0)
		{
			sensor_is_init = 1;
			//pps960初始化
			init_pps960_sensor();
			acc_check = 1;
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim3);
			//1291初始化
			ads1291_init();
		}
		if(sensor_open == 0 && sensor_is_init == 1)
		{
			sensor_is_init = 0;
		  //1291去初始化
    	ads1291_disable();
			//pps960去初始化	
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);
			pps960_disable();
			acc_check = 0;
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
