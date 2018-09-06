#include "stm32f4xx_hal.h"
#include "USER_SPI_IO.h"

#define SPI_IO_CLK_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define SPI_IO_CLK_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

#define SPI_IO_SIMO_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define SPI_IO_SIMO_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)

#define SPI_IO_SOMI  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)

#define SPI_IO_CS_L()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_IO_CS_H()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define SPI_IO_FCLK()  user_spi_delay_us(1)


void user_spi_delay_us(uint16_t delay)
{
	
	while(delay--)
	{

	}
}


void spi_io_write(uint8_t TxData)
{
	uint8_t i;
	SPI_IO_CLK_L();  
	for(i = 0;i<8;i++)
	{
		SPI_IO_FCLK();
		if(TxData&(0x80>>i))
		{
			SPI_IO_SIMO_H() ;
		}
		else
		{
			SPI_IO_SIMO_L() ;
		}
		SPI_IO_CLK_H();
		SPI_IO_FCLK(); 
		SPI_IO_CLK_L();
	}
	SPI_IO_FCLK(); 
}

uint8_t spi_io_read(void)
{
	uint8_t i,RxData = 0;
	SPI_IO_CLK_L();  
	for(i = 0;i<8;i++)
	{
		SPI_IO_FCLK();		
		SPI_IO_CLK_H();
		SPI_IO_FCLK(); 
		SPI_IO_CLK_L();		
		if(SPI_IO_SOMI)
		{
			RxData |= (0x80>>i);
		}
		else
		{

		}

	}
	SPI_IO_FCLK(); 
	return RxData;
}

void spi_io_writeread(uint8_t *pTxData,uint16_t TxSize,uint8_t *pRxData,uint16_t RxSize)
{
	uint16_t i,j;

	for(i = 0;i<TxSize;i++)
	{
		spi_io_write(pTxData[i]);
	}
	for(j = 0;j<RxSize;j++)
	{
		pRxData[j] = spi_io_read();
	}

}

void spi_io_multiwrite(uint8_t *pTxData,uint16_t TxSize)
{
	uint16_t i;
	for(i = 0;i<TxSize;i++)
	{
		spi_io_write(pTxData[i]);
	}
}

void spi_io_multiread(uint8_t *pRxData,uint16_t RxSize)
{
	uint16_t i;
	for(i = 0;i<RxSize;i++)
	{
		pRxData[i] = spi_io_read();
	}
}
	
