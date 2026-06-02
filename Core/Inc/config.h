#ifndef CONFIG_H_
#define CONFIG_H_

#include "stm32f446xx.h"

/* Math */
#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

/* Robot Geometry */
#define L1 35.0f
#define L2 85.0f
#define BASE 50.0f
#define PLATFORM 50.0f

#define NUM_MOTORS 4
#define STEPS_PER_REV 3200.0f
#define ALPHA (6.2831853f / STEPS_PER_REV)

/* Motor Behaviour */
#define BASE_VEL 400
#define TRAP_STEPS 100

// #define angle_resolution 0.04363325f
// #define angle_resolution 0.0872665
#define angle_resolution 0.174533f
#define height_resolution 0.5f

#define CONTROL_FREQ 100.0f
#define CONTROL_DT (1.0f / CONTROL_FREQ)
#define TIM1_FREQ 100000
#define TICKS_PER_CONTROL (TIM1_FREQ / (uint32_t)CONTROL_FREQ)

#define VEL_MAX 167.0f
#define ACC_MAX 600.0f

#define TRAJ_BUF_SIZE 64

#define STEP_SET_PIN_0 GPIO_BSRR_BS0
#define STEP_SET_PIN_1 GPIO_BSRR_BS1
#define STEP_SET_PIN_2 GPIO_BSRR_BS4
#define STEP_SET_PIN_3 GPIO_BSRR_BS8
#define STEP_RESET_PIN_0 GPIO_BSRR_BR0
#define STEP_RESET_PIN_1 GPIO_BSRR_BR1
#define STEP_RESET_PIN_2 GPIO_BSRR_BR4
#define STEP_RESET_PIN_3 GPIO_BSRR_BR8

#define DIR_SET_PIN_0 GPIO_BSRR_BS2
#define DIR_SET_PIN_1 GPIO_BSRR_BS1
#define DIR_SET_PIN_2 GPIO_BSRR_BS15
#define DIR_SET_PIN_3 GPIO_BSRR_BS14
#define DIR_RESET_PIN_0 GPIO_BSRR_BR2
#define DIR_RESET_PIN_1 GPIO_BSRR_BR1
#define DIR_RESET_PIN_2 GPIO_BSRR_BR15
#define DIR_RESET_PIN_3 GPIO_BSRR_BR14

extern const uint32_t step_set[NUM_MOTORS];
extern const uint32_t step_reset[NUM_MOTORS];

typedef struct {
  volatile float n_x;
  volatile float n_y;
  volatile float n_z;
  volatile float h;
  volatile float z;
} Pose_t;

typedef struct {
  volatile uint32_t step_period;
  volatile uint32_t step_counter;
  volatile uint32_t steps_remaining;
  volatile float angle;
  volatile int8_t step_dir;
} Motor_t;

typedef struct {
  Motor_t motor[NUM_MOTORS];
  volatile uint32_t pending_resets;
  Pose_t pose;
} MotorController_t;

#endif /* CONFIG_H_ */