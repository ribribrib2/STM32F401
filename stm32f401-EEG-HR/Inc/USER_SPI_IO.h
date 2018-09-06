#ifndef __USER_SPI_IO_H
#define  __USER_SPI_IO_H

#include "stm32f4xx_hal.h"


void user_spi_delay_us(uint16_t delay);

extern void spi_io_write(uint8_t TxData);
extern uint8_t spi_io_read(void);

extern void spi_io_writeread(uint8_t *pTxData,uint16_t TxSize,uint8_t *pRxData,uint16_t RxSize);

extern void spi_io_multiwrite(uint8_t *pTxData,uint16_t TxSize);
extern void spi_io_multiread(uint8_t *pRxData,uint16_t RxSize);



#endif
