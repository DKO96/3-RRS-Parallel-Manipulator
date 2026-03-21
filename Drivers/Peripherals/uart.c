#include "uart.h"

#define PCLK 45000000
#define BAUDRATE 115200

// Function definitions
void uart_init(USART_TypeDef *UARTx) {
  if (UARTx == USART2) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  }

  uint32_t usartdiv = (PCLK + (BAUDRATE / 2U)) / BAUDRATE;
  UARTx->BRR = usartdiv;

  UARTx->CR1 |= USART_CR1_RXNEIE;

  UARTx->CR1 |= USART_CR1_TE;  // enable transmit
  UARTx->CR1 |= USART_CR1_RE;  // enable receive
  UARTx->CR1 |= USART_CR1_UE;  // enable UARTx

  NVIC_SetPriority(USART2_IRQn, 5);
  NVIC_EnableIRQ(USART2_IRQn);
}

void usart2_write(int ch) {
  while (!(USART2->SR & USART_SR_TXE))
    ;                        // wait until transmit data register is empty
  USART2->DR = (ch & 0xFF);  // write character to the data register
}

uint8_t usart2_read(void) {
  while (!(USART2->SR & USART_SR_RXNE))
    ;                        // wait until receive data register is not empty
  return USART2->DR & 0xFF;  // return the received character
}

void printS(const char myString[]) {
  uint8_t i = 0;
  while (myString[i]) {
    usart2_write(myString[i]);
    i++;
  }
}

void printH(uint8_t value) {
  const char hex_digits[] = "0123456789ABCDEF";
  usart2_write(hex_digits[(value >> 4) & 0x0F]);
  usart2_write(hex_digits[value & 0x0F]);
}

void printI(int32_t number) {
  char buffer[12];
  int i = 0;

  if (number < 0) {
    usart2_write('-');
    number = -number;
  }

  do {
    buffer[i++] = (number % 10) + '0';
    number /= 10;
  } while (number > 0);

  while (i > 0) {
    usart2_write(buffer[--i]);
  }
}

void printF(float number) {
  if (number < 0) {
    usart2_write('-');
    number = -number;
  }

  uint32_t integer_part = (uint32_t)number;
  printI(integer_part);
  usart2_write('.');

  float frac = number - (float)integer_part;

  for (uint8_t i = 0; i < 6; i++) {
    frac *= 10.0f;
    uint8_t digit = (uint8_t)frac;
    usart2_write('0' + digit);
    frac -= digit;
  }
}