#ifndef UART_H
#define UART_H

#include "stm32f446xx.h"

/**
 * @brief Initialize USART2 for monitoring robot status
 */
void uart_init(USART_TypeDef *UARTx);

/**
 * @brief Write character to USART2 data register
 */
void usart2_write(int ch);

/**
 * @brief Read character for USART2 data register
 */
uint8_t usart2_read(void);

/**
 * @brief Print strings via USART2
 */
void printS(const char myString[]);

/**
 * @brief Print hexadecimals via USART2
 */
void printH(uint8_t value);

/**
 * @brief Print integers via USART2
 */
void printI(int32_t number);

/**
 * @brief Print floats via USART2
 */
void printF(float number);

#endif /* UART_H */