#include "ads1x9x_hal_driver.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"

#define ADS_Delay_Num       		10

uint8_t ads1291_is_init = 0;

ADS_ConfigDef ADS_Config1;

void ads1291_disable(void)
{
	ads1291_is_init = 0;
	ADS_PIN_EN_L();
}

void ads1291_init(void)
{
		ads1291_is_init = 1;
		ADS_PIN_EN_H();
		HAL_Delay(200);
	  ADS_PIN_MAINCLKSEL_H();
	  HAL_Delay(100);
  	ADS_PIN_RESET_H();
  	HAL_Delay(1000);
	  ADS_PIN_RESET_L();
	  HAL_Delay(10);
	  ADS_PIN_RESET_H();
	  ADS_PIN_START_L();
    HAL_Delay(10);
  	ADS_Command(ADS_SDATAC);
  	ADS_init();
		ADS_PIN_START_H();	
	  ADS_Command(ADS_RDATAC);	
}

void ADS_init(void)
{
	ADS_Config_Init(&ADS_Config1);
	ADS_Config1.CONFIG1.Value = ADS_DR(1);
	ADS_Config1.CONFIG2.Value |= ADS_PDB_LOFF_COMP + ADS_PDB_REFBUF + ADS_INT_TEST + ADS_TEST_FREQ;
	ADS_Config1.LOFF.Value |= ADS_COMP_TH(0) + ADS_ILEAD_OFF(0);//
	ADS_Config1.CH1SET.Value |=ADS_GAIN1(6)+ADS_MUX1(0);
	ADS_Config1.CH2SET.Value |=ADS_PD2 + ADS_GAIN2(0) + ADS_MUX2(1);//
	ADS_Config1.RLD_SENS.Value |= ADS_RLD_LOFF_SENS + ADS_CHOP(0) + ADS_PDB_RLD + ADS_RLD1N + ADS_RLD1P;//
	ADS_Config1.LOFF_SENS.Value |=  ADS_FLIP1 +ADS_LOFF1N + ADS_LOFF1P;
	ADS_Config1.LOFF_STAT.Value |= NULL;
	ADS_Config1.RESP1.Value |= NULL;
	ADS_Config1.RESP2.Value |= ADS_RLDREF_INT ;//+ ADS_CALIB_ON
	ADS_Config1.GPIO.Value |= NULL;
	ADS_Config(&ADS_Config1);
}
	
void ADS_Config_Init(ADS_ConfigDef *Config)
{
	(*Config).CONFIG1.Address		=	ADS_REG_CONFIG1_ADDRESS;
	(*Config).CONFIG1.Value			=	ADS_REG_CONFIG1_DEFAULT;
	(*Config).CONFIG2.Address		=	ADS_REG_CONFIG2_ADDRESS;
	(*Config).CONFIG2.Value			=	ADS_REG_CONFIG2_DEFAULT;
	(*Config).LOFF.Address				=	ADS_REG_LOFF_ADDRESS;
	(*Config).LOFF.Value					=	ADS_REG_LOFF_DEFAULT;
	(*Config).CH1SET.Address			=	ADS_REG_CH1SET_ADDRESS;
	(*Config).CH1SET.Value				=	ADS_REG_CH1SET_DEFAULT;
	(*Config).CH2SET.Address			= 	ADS_REG_CH2SET_ADDRESS;
	(*Config).CH2SET.Value				=	ADS_REG_CH2SET_DEFAULT;
	(*Config).RLD_SENS.Address		=	ADS_REG_RLDSENS_ADDRESS;
	(*Config).RLD_SENS.Value			=	ADS_REG_RLDSENS_DEFAULT;
	(*Config).LOFF_SENS.Address	=	ADS_REG_LOFFSENS_ADDRESS;
	(*Config).LOFF_SENS.Value		=	ADS_REG_LOFFSENS_DEFAULT;
	(*Config).LOFF_STAT.Address	=	ADS_REG_LOFFSTAT_ADDRESS;
	(*Config).LOFF_STAT.Value		=	ADS_REG_LOFFSTAT_DEFAULT;
	(*Config).RESP1.Address			=	ADS_REG_RESP1_ADDRESS;
	(*Config).RESP1.Value				=	ADS_REG_RESP1_DEFAULT;
	(*Config).RESP2.Address			=	ADS_REG_RESP2_ADDRESS;
	(*Config).RESP2.Value				=	ADS_REG_RESP2_DEFAULT;	
	(*Config).GPIO.Address				=	ADS_REG_GPIO_ADDRESS;
	(*Config).GPIO.Value					=	ADS_REG_GPIO_DEFAULT;
}	

void ADS_Config(ADS_ConfigDef *Config)
{	
	ADS_Setting((*Config).CONFIG1.Address,0,&(*Config).CONFIG1.Value,1);
	ADS_Setting((*Config).CONFIG2.Address,0,&(*Config).CONFIG2.Value,1);
	ADS_Setting((*Config).LOFF.Address,0,&(*Config).LOFF.Value,1);
	ADS_Setting((*Config).CH1SET.Address,0,&(*Config).CH1SET.Value,1);
	ADS_Setting((*Config).CH2SET.Address,0,&(*Config).CH2SET.Value,1);
	ADS_Setting((*Config).RLD_SENS.Address,0,&(*Config).RLD_SENS.Value,1);
	ADS_Setting((*Config).LOFF_SENS.Address,0,&(*Config).LOFF_SENS.Value,1);
	ADS_Setting((*Config).LOFF_STAT.Address,0,&(*Config).LOFF_STAT.Value,1);
	ADS_Setting((*Config).RESP1.Address,0,&(*Config).RESP1.Value,1);
	ADS_Setting((*Config).RESP2.Address,0,&(*Config).RESP2.Value,1);
	ADS_Setting((*Config).GPIO.Address,0,&(*Config).GPIO.Value,1);	
}	
void ADS_Setting(uint8_t REG,uint8_t Num,uint8_t *pData,uint16_t Size )
{
		REG |=ADS_WREG;
		ADS_PIN_CS_L();
		ADS_SPI_Delay(ADS_Delay_Num);
		ADS_SPI_Write(&REG,1);
		ADS_SPI_Write(&Num,1);
		ADS_SPI_Write(pData,Size);
		ADS_SPI_Delay(ADS_Delay_Num);
		ADS_PIN_CS_H();
}

void ADS_Command(uint8_t CMD)
{
		ADS_PIN_CS_L();
		ADS_SPI_Delay(ADS_Delay_Num);
		ADS_SPI_Write(&CMD,1);
		ADS_SPI_Delay(ADS_Delay_Num);
		ADS_PIN_CS_H();
}

void ADS_ReadData(uint8_t *pRxData,uint16_t Size)
{
	ADS_PIN_CS_L();
	ADS_SPI_Delay(ADS_Delay_Num);
	ADS_SPI_Read(pRxData,Size);
	ADS_SPI_Delay(ADS_Delay_Num);
  ADS_PIN_CS_H();
}

void ADS_ReadStatue(uint8_t REG,uint8_t Num,uint8_t *pData,uint16_t Size)
{
    REG |= ADS_RREG;
    ADS_PIN_CS_L();
	  ADS_SPI_Delay(ADS_Delay_Num);
    ADS_SPI_Write(&REG,1);
		ADS_SPI_Write(&Num,1);
		ADS_SPI_Read(pData,Size);
		ADS_SPI_Delay(ADS_Delay_Num);
    ADS_PIN_CS_H();
}

