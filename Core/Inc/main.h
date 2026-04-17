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

extern const uint32_t step_set[NUM_MOTORS];
extern const uint32_t step_reset[NUM_MOTORS];

typedef struct {
  volatile uint32_t step_period;
  volatile uint32_t step_counter;
  volatile uint32_t steps_remaining;
  volatile int32_t current_steps;
  volatile int32_t last_waypoint;
  volatile uint16_t angle;
  volatile int32_t target_steps;
  volatile int8_t step_dir;
} Motor;

typedef struct {
  Motor motor[NUM_MOTORS];
  volatile uint32_t pending_resets;
} MotorController;

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