#ifndef CONFIG_H_
#define CONFIG_H_

/* Math */
#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

/* Robot Geometry */
#define L1 55.0f
#define L2 75.0f
#define BASE 50.0f
#define PLATFORM 50.0f

#define NUM_MOTORS 3
#define STEPS_PER_REV 3200.0f
#define ALPHA (6.2831853f / STEPS_PER_REV)

/* Motor Behaviour */
#define BASE_SPEED 600
#define TRAP_STEPS 100

#define angle_resolution 0.0872665f
#define height_resolution 0.5f

#define CONTROL_FREQ 100.0f
#define CONTROL_DT (1.0f / CONTROL_FREQ)
#define TIM1_FREQ 100000
#define TICKS_PER_CONTROL (TIM1_FREQ / (uint32_t)CONTROL_FREQ)

// #define VEL_MAX 167.0f
#define VEL_MAX 167.0f
#define ACC_MAX 600.0f

#define TRAJ_BUF_SIZE 64

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

#endif /* CONFIG_H_ */