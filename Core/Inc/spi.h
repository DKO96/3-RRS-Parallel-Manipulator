#ifndef SPI_H_
#define SPI_H_

#include "stm32f446xx.h"

void spi1_init(void);
uint8_t spi_transfer(uint8_t data);

#endif /* SPI_H_ */