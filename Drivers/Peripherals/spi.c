#include "spi.h"

void spi1_init(void) {
  // Enable clock for SPI 1
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // Configure serial clock baud rate to 16
  SPI1->CR1 &= ~SPI_CR1_BR;
  SPI1->CR1 |= (6U << SPI_CR1_BR_Pos);

  // Configure SPI mode 0
  SPI1->CR1 &= ~((SPI_CR1_CPOL) | (SPI_CR1_CPHA));

  // Select full duplex mode
  SPI1->CR1 &= ~((SPI_CR1_BIDIMODE) | (SPI_CR1_RXONLY));

  // Configure MSB first
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

  // Disable hardware CRC
  SPI1->CR1 &= ~SPI_CR1_CRCEN;

  // Enable software and internal slave select
  SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);

  // Configure STM32 as master
  SPI1->CR1 |= SPI_CR1_MSTR;

  // Set data frame format to 8 bits
  SPI1->CR1 &= ~SPI_CR1_DFF;

  // Enable SPI1
  SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t spi_transfer(uint8_t data) {
  while (!(SPI1->SR & SPI_SR_TXE))
    ;

  SPI1->DR = data;

  while (!(SPI1->SR & SPI_SR_RXNE))
    ;

  return SPI1->DR;
}