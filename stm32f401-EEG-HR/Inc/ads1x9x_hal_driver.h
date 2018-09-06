#ifndef		__ADS1X9X_HAL_DRIVER_H
#define		__ADS1X9X_HAL_DRIVER_H

#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"

extern SPI_HandleTypeDef hspi1;

#define ADS_REG_ID_ADDRESS				0x00

#define ADS_REG_CONFIG1_ADDRESS		0x01
#define ADS_REG_CONFIG1_DEFAULT   0x00
#define ADS_SINGLE_SHOT						0x80
#define ADS_DR(n)									0x01*(n)

#define ADS_REG_CONFIG2_ADDRESS		0x02
#define ADS_REG_CONFIG2_DEFAULT		0x80
#define ADS_PDB_LOFF_COMP					0x40
#define ADS_PDB_REFBUF						0x20
#define ADS_VREF_4V								0x10
#define ADS_CLK_EN								0x08
#define ADS_INT_TEST							0x02
#define ADS_TEST_FREQ							0x01

#define ADS_REG_LOFF_ADDRESS			0x03
#define ADS_REG_LOFF_DEFAULT			0x10
#define ADS_COMP_TH(n)						0x20*(n)
#define ADS_ILEAD_OFF(n)					0x04*(n)
#define ADS_FLEAD_OFF							0x01

#define ADS_REG_CH1SET_ADDRESS		0x04
#define ADS_REG_CH1SET_DEFAULT		0x00
#define ADS_PD1										0x80
#define ADS_GAIN1(n)							0x10*(n)
#define ADS_MUX1(n)								0x01*(n)

#define ADS_REG_CH2SET_ADDRESS		0x05
#define ADS_REG_CH2SET_DEFAULT		0x00
#define ADS_PD2										0x80
#define ADS_GAIN2(n)							0x10*(n)
#define ADS_MUX2(n)								0x01*(n)

#define ADS_REG_RLDSENS_ADDRESS		0x06
#define ADS_REG_RLDSENS_DEFAULT		0x00
#define ADS_CHOP(n)								0x40*(n)
#define ADS_PDB_RLD								0x20
#define ADS_RLD_LOFF_SENS					0x10
#define ADS_RLD2N									0x08
#define ADS_RLD2P									0x04
#define ADS_RLD1N									0x02
#define ADS_RLD1P									0x01

#define ADS_REG_LOFFSENS_ADDRESS	0x07
#define ADS_REG_LOFFSENS_DEFAULT	0x00
#define ADS_FLIP2									0x20
#define ADS_FLIP1									0x10
#define ADS_LOFF2N								0x08
#define ADS_LOFF2P								0x04
#define ADS_LOFF1N								0x02
#define ADS_LOFF1P								0x01

#define ADS_REG_LOFFSTAT_ADDRESS	0x08
#define ADS_REG_LOFFSTAT_DEFAULT	0x00
#define ADS_CLK_DIV								0x40
#define ADS_RLD_STAT							0x10
#define ADS_IN2N_OFF							0x08
#define ADS_IN2P_OFF							0x04
#define ADS_IN1N_OFF							0x02
#define ADS_IN1P_OFF							0x01

#define ADS_REG_RESP1_ADDRESS			0x09
#define ADS_REG_RESP1_DEFAULT			0x02
#define ADS_RESP_DEMOD_EN1				0x80
#define ADS_RESP_MOD_EN						0x40
#define ADS_RESP_PH(n)						0x04*(n)
#define ADS_RESP_CTRL							0x01

#define ADS_REG_RESP2_ADDRESS			0x0A
#define ADS_REG_RESP2_DEFAULT			0x01
#define ADS_CALIB_ON							0x80
#define ADS_RESP_FREQ							0x04
#define ADS_RLDREF_INT						0x02

#define ADS_REG_GPIO_ADDRESS			0x0B
#define ADS_REG_GPIO_DEFAULT			0x0C
#define ADS_GPIOC2								0x08
#define ADS_GPIOC1								0x04
#define ADS_GPIOD2								0x02
#define ADS_GPIOD1								0x01

#define ADS_WAKEUP								0x02
#define ADS_STANDBY								0x04
#define ADS_RESET									0x06
#define ADS_START									0x08
#define ADS_STOP									0x0A
#define ADS_OFFSETCAL							0x1A
#define ADS_RDATAC								0x10
#define ADS_SDATAC								0x11
#define ADS_RDATA									0x12
#define ADS_RREG									0x20
#define ADS_WREG									0x40

#define ADS_PIN_START_L()                         HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_RESET)
#define ADS_PIN_START_H()                         HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_SET)
#define ADS_PIN_RESET_L()                         HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_RESET)
#define ADS_PIN_RESET_H()                         HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_SET)
#define ADS_PIN_MAINCLKSEL_L()                    HAL_GPIO_WritePin(ADS_MAIN_CLKSEL_GPIO_Port, ADS_MAIN_CLKSEL_Pin, GPIO_PIN_RESET)
#define ADS_PIN_MAINCLKSEL_H()                    HAL_GPIO_WritePin(ADS_MAIN_CLKSEL_GPIO_Port,ADS_MAIN_CLKSEL_Pin,GPIO_PIN_SET);
#define ADS_PIN_CS_L()                            HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET)
#define ADS_PIN_CS_H()                            HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET)   
#define ADS_PIN_EN_L()                            HAL_GPIO_WritePin(ADS_EN_GPIO_Port, ADS_EN_Pin, GPIO_PIN_RESET)
#define ADS_PIN_EN_H()                            HAL_GPIO_WritePin(ADS_EN_GPIO_Port, ADS_EN_Pin, GPIO_PIN_SET)   


#define ADS_SPI_Write(Data,Size)           HAL_SPI_Transmit(&hspi1, Data, Size, 1000) 
#define ADS_SPI_Read(Data,Size)            HAL_SPI_Receive(&hspi1,Data, Size, 1000)
#define ADS_SPI_Delay(n)							     DWT_Delay_us(n) 


typedef struct {
	uint8_t Address;
	uint8_t Value;
}ADS_RegisterDef;


typedef struct {
	ADS_RegisterDef CONFIG1;
	ADS_RegisterDef CONFIG2;
	ADS_RegisterDef LOFF;
	ADS_RegisterDef CH1SET;
	ADS_RegisterDef CH2SET;
	ADS_RegisterDef RLD_SENS;
	ADS_RegisterDef LOFF_SENS;
	ADS_RegisterDef LOFF_STAT;
	ADS_RegisterDef RESP1;
	ADS_RegisterDef RESP2;
	ADS_RegisterDef GPIO;
}ADS_ConfigDef;

void ads1291_disable(void);
void ads1291_init(void);
extern void ADS_ReadStatue(uint8_t REG,uint8_t Num,uint8_t *pData,uint16_t Size);
extern void ADS_Setting(uint8_t REG,uint8_t Num,uint8_t *pData,uint16_t Size );
extern void ADS_Command(uint8_t CMD);
extern void ADS_ReadData(uint8_t *pRxData,uint16_t Size);
extern void ADS_Config_Init(ADS_ConfigDef *Config);
extern void ADS_Config(ADS_ConfigDef *Config);
extern void ADS_init(void);

#endif
