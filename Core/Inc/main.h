#ifndef MAIN_H_
#define MAIN_H_

#include "config.h"
#include "controller.h"
#include "ik.h"

// Peripherals
#include "gpio.h"
#include "timer.h"
#include "uart.h"

/**
 * @brief Initialize system clock for 180MHz
 */
void system_init(void);

/**
 * @brief Microsecond delay using DWT
 */
void delay_us(uint32_t us);

/**
 * @brief Millisecond delay using DWT
 */
void delay_ms(uint32_t ms);

#endif /* MAIN_H_ */