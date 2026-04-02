#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "config.h"
#include "stm32f446xx.h"

#define STEP_SET_PIN_0 GPIO_BSRR_BS0
#define STEP_SET_PIN_1 GPIO_BSRR_BS1
#define STEP_SET_PIN_2 GPIO_BSRR_BS4
#define STEP_RESET_PIN_0 GPIO_BSRR_BR0
#define STEP_RESET_PIN_1 GPIO_BSRR_BR1
#define STEP_RESET_PIN_2 GPIO_BSRR_BR4

#define DIR_SET_PIN_0 GPIO_BSRR_BS2
#define DIR_SET_PIN_1 GPIO_BSRR_BS1
#define DIR_SET_PIN_2 GPIO_BSRR_BS15
#define DIR_RESET_PIN_0 GPIO_BSRR_BR2
#define DIR_RESET_PIN_1 GPIO_BSRR_BR1
#define DIR_RESET_PIN_2 GPIO_BSRR_BR15

extern const uint32_t step_set[NUM_MOTORS];
extern const uint32_t step_reset[NUM_MOTORS];

// extern volatile uint32_t step_period[NUM_MOTORS];
// extern volatile uint32_t step_counter[NUM_MOTORS];
// extern volatile uint8_t motor_enabled[NUM_MOTORS];
// extern volatile uint32_t steps_remaining[NUM_MOTORS];
// extern volatile int32_t current_steps[NUM_MOTORS];
// extern volatile int32_t target_steps[NUM_MOTORS];
// extern volatile uint32_t pending_resets;
// extern volatile int8_t step_dir[NUM_MOTORS];

typedef struct {
  volatile uint32_t step_period;
  volatile uint32_t step_counter;
  volatile uint8_t motor_enabled;
  volatile uint32_t steps_remaining;
  volatile int32_t current_steps;
  volatile int32_t target_steps;
  volatile int8_t step_dir;
} Motor;

typedef struct {
  Motor motor[NUM_MOTORS];
  volatile uint32_t pending_resets;
} MotorController;

uint32_t angle_to_steps(float angle);
uint8_t move_complete(MotorController *mc);
void follow_trajectory(MotorController *mc, float n_start[3], float h_start,
                       float n_end[3], float h_end);

#endif /* CONTROLLER_H_ */