#ifndef MAIN_H_
#define MAIN_H_

#include "config.h"
#include "ik.h"
#include "trajectory.h"

// Peripherals
#include "exti.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

// Hardware
#include "amt222b.h"

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