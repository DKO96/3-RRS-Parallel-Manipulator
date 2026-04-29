#include "main.h"

#include "math.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f446xx.h"
#include "task.h"

QueueHandle_t traj_queue;
static SemaphoreHandle_t rotary_complete_semphr;
static volatile uint8_t rotary_active = 0;

#define ANGLE(raw) ((float)(raw * 2.0f * M_PI / 16384.0f))
#define STEPS(angle) ((int32_t)(angle * STEPS_PER_REV / (2.0f * M_PI)))

const uint32_t step_set[NUM_MOTORS] = {STEP_SET_PIN_0, STEP_SET_PIN_1,
                                       STEP_SET_PIN_2, STEP_SET_PIN_3};
const uint32_t step_reset[NUM_MOTORS] = {STEP_RESET_PIN_0, STEP_RESET_PIN_1,
                                         STEP_RESET_PIN_2, STEP_RESET_PIN_3};
static const uint32_t dir_set[NUM_MOTORS] = {DIR_SET_PIN_0, DIR_SET_PIN_1,
                                             DIR_SET_PIN_2, DIR_SET_PIN_3};
static const uint32_t dir_reset[NUM_MOTORS] = {
    DIR_RESET_PIN_0, DIR_RESET_PIN_1, DIR_RESET_PIN_2, DIR_RESET_PIN_3};

/* Initialize motor*/
MotorController_t controller = {
    .motor = {{.step_period = BASE_VEL, .angle = ANGLE(12355), .step_dir = 1},
              {.step_period = BASE_VEL, .angle = ANGLE(12094), .step_dir = 1},
              {.step_period = BASE_VEL, .angle = ANGLE(12341), .step_dir = 1},
              {.step_period = BASE_VEL, .angle = ANGLE(0), .step_dir = 1}},
    .pending_resets = 0};

volatile uint8_t safety_flag = 1;

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR |= EXTI_PR_PR13;
  }

  if (safety_flag) {
    safety_flag = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
    printS("SAFETY: OFF\r\n");
  } else {
    safety_flag = 1;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    printS("SAFETY: ON\r\n");
  }
}

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint32_t pins_to_set = 0;
  uint32_t resets = 0;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (controller.motor[i].steps_remaining == 0)
      continue;

    controller.motor[i].step_counter++;
    if (controller.motor[i].step_counter >= controller.motor[i].step_period) {
      controller.motor[i].step_counter = 0;
      pins_to_set |= step_set[i];
      resets |= step_reset[i];
      controller.motor[i].steps_remaining--;
    }
  }

  if (rotary_active && controller.motor[3].steps_remaining == 0) {
    rotary_active = 0;
    xSemaphoreGiveFromISR(rotary_complete_semphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  if (pins_to_set) {
    GPIOA->BSRR = pins_to_set;
    controller.pending_resets = resets;

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
  }
}

void TIM2_IRQHandler(void) {
  TIM2->SR &= ~TIM_SR_UIF;
  GPIOA->BSRR = controller.pending_resets;
}

static void generate_rotary(MotorController_t *m, int32_t z_start,
                            int32_t z_end) {
  int32_t steps = z_end - z_start;

  if (steps < 0) {
    GPIOB->BSRR = dir_set[3];
    m->motor[3].step_dir = -1;
  } else {
    GPIOB->BSRR = dir_reset[3];
    m->motor[3].step_dir = 1;
  }

  m->motor[3].steps_remaining =
      (steps < 0) ? (uint32_t)(-steps) : (uint32_t)steps;

  rotary_active = 1;
}

static void trajectory_task(void *pvParameters) {
  (void)pvParameters;
  float n_start[3] = {0.0f, 0.0f, 1.0f};
  float h_start = 120.0f;
  float n_end[3] = {0.0f, 0.0f, 1.0f};
  float h_end = 100.0f;
  generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
  vTaskDelay(2000);

  n_start[0] = 0.0f;
  n_start[1] = 0.0f;
  n_start[2] = 1.0f;
  h_start = 100.0f;
  n_end[0] = 0.371391f;
  n_end[1] = 0.0f;
  n_end[2] = 0.928477f;
  h_end = 100.0f;
  generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
  vTaskDelay(2000);

  int32_t z_start = 0;
  int32_t z_end = 6000;
  generate_rotary(&controller, z_start, z_end);
  xSemaphoreTake(rotary_complete_semphr, portMAX_DELAY);
  vTaskDelay(2000);

  for (;;) {
    n_start[0] = 0.371391f;
    n_start[1] = 0.0f;
    n_start[2] = 0.928477f;
    h_start = 100.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.371391f;
    n_end[2] = 0.928477f;
    h_end = 100.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    z_start = 6000;
    z_end = 9000;
    generate_rotary(&controller, z_start, z_end);
    xSemaphoreTake(rotary_complete_semphr, portMAX_DELAY);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = 0.371391f;
    n_start[2] = 0.928477f;
    h_start = 100.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 100.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    z_start = 9000;
    z_end = -6000;
    generate_rotary(&controller, z_start, z_end);
    xSemaphoreTake(rotary_complete_semphr, portMAX_DELAY);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = 0.0f;
    n_start[2] = 1.0f;
    h_start = 100.0f;
    n_end[0] = 0.0f;
    n_end[1] = -0.371391f;
    n_end[2] = 0.928477f;
    h_end = 100.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    z_start = -6000;
    z_end = -9000;
    generate_rotary(&controller, z_start, z_end);
    xSemaphoreTake(rotary_complete_semphr, portMAX_DELAY);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = -0.371391f;
    n_start[2] = 0.928477f;
    h_start = 100.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 100.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    z_start = -9000;
    z_end = 0;
    generate_rotary(&controller, z_start, z_end);
    xSemaphoreTake(rotary_complete_semphr, portMAX_DELAY);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = 0.0f;
    n_start[2] = 1.0f;
    h_start = 100.0f;
    n_end[0] = 0.371391f;
    n_end[1] = 0.0f;
    n_end[2] = 0.928477f;
    h_end = 100.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);
  }
}

static void encoder_task(void *pvParameters) {
  (void)pvParameters;

  TickType_t last_wake = xTaskGetTickCount();
  uint8_t encoder_pins[NUM_MOTORS] = {8, 6, 5};

  for (;;) {
    for (uint8_t i = 0; i < 3; i++) {
      uint16_t raw;

      amt222b_read(encoder_pins[i], &raw);

      float joint_angle = ANGLE(raw);

      if (joint_angle > M_PI)
        joint_angle -= 2.0f * M_PI;

      controller.motor[i].angle = joint_angle;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
  }
}

static void controller_task(void *pvParameters) {
  (void)pvParameters;

  TickType_t last_wake = xTaskGetTickCount();
  static TrajectoryPoint_t prev_point;
  static uint8_t have_prev_waypoint = 0;

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

    TrajectoryPoint_t point;
    if (xQueueReceive(traj_queue, &point, 0) != pdTRUE)
      continue;

    /* VALIDATE TRAJECTORY WAYPOINT REACHED */
    if (have_prev_waypoint) {
      uint8_t reached = 1;
      for (uint8_t i = 0; i < 3; i++) {
        float error = controller.motor[i].angle - prev_point.target_angle[i];
        if (fabsf(error) > 0.02f) {
          reached = 0;
          printS("MISS\r\n");
          break;
        }
      }

      if (reached) {
        controller.pose = prev_point.pose;
        printS("Reached: ");
        printS("nx: ");
        printF(controller.pose.n_x);
        printS(" ny: ");
        printF(controller.pose.n_y);
        printS(" nz: ");
        printF(controller.pose.n_z);
        printS(" h: ");
        printF(controller.pose.h);
        printS("\r\n");
      }
      /* else didn't reach pose */
    }

    /* COMMAND NEXT MOVE */
    __disable_irq();

    uint32_t max_steps = 0;
    for (uint8_t i = 0; i < 3; i++) {
      float delta_angle = point.target_angle[i] - controller.motor[i].angle;
      int32_t delta_steps = (int32_t)(delta_angle / ALPHA);

      if (delta_steps < 0) {
        GPIOB->BSRR = dir_reset[i];
        controller.motor[i].step_dir = -1;
        controller.motor[i].steps_remaining = (uint32_t)(-delta_steps);
      } else {
        GPIOB->BSRR = dir_set[i];
        controller.motor[i].step_dir = 1;
        controller.motor[i].steps_remaining = (uint32_t)(delta_steps);
      }

      if (controller.motor[i].steps_remaining > max_steps) {
        max_steps = controller.motor[i].steps_remaining;
      }
    }

    for (uint8_t i = 0; i < 3; i++) {
      if (controller.motor[i].steps_remaining == 0)
        continue;

      controller.motor[i].step_period =
          TICKS_PER_CONTROL / controller.motor[i].steps_remaining;

      if (controller.motor[i].step_period == 0) {
        controller.motor[i].step_period = 1;
      }
    }

    __enable_irq();

    /* SAVE TRAJECTORY WAYPOINT */
    prev_point = point;
    have_prev_waypoint = 1;
  }
}

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  exti_init();
  uart_init(USART2);
  timer1_init();
  timer2_init();
  spi1_init();

  printS("\r\n=========PROGRAM START=============\r\n");
  while (safety_flag)
    ;

  /* Initialize rtos */
  traj_queue = xQueueCreate(TRAJ_BUF_SIZE, sizeof(TrajectoryPoint_t));
  rotary_complete_semphr = xSemaphoreCreateBinary();

  xTaskCreate(trajectory_task, "traj", 512, NULL, 2, NULL);
  xTaskCreate(encoder_task, "encd", 128, NULL, 4, NULL);
  xTaskCreate(controller_task, "ctrl", 256, NULL, 3, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  return 0;
}
