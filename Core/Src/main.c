#include "main.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f446xx.h"
#include "task.h"

QueueHandle_t traj_queue;

#define LED_ON() (GPIOA->ODR |= GPIO_ODR_OD5)
#define LED_OFF() (GPIOA->ODR &= ~GPIO_ODR_OD5)

const uint32_t step_set[NUM_MOTORS] = {STEP_SET_PIN_0, STEP_SET_PIN_1,
                                       STEP_SET_PIN_2};
const uint32_t step_reset[NUM_MOTORS] = {STEP_RESET_PIN_0, STEP_RESET_PIN_1,
                                         STEP_RESET_PIN_2};
static const uint32_t dir_set[NUM_MOTORS] = {DIR_SET_PIN_0, DIR_SET_PIN_1,
                                             DIR_SET_PIN_2};
static const uint32_t dir_reset[NUM_MOTORS] = {DIR_RESET_PIN_0, DIR_RESET_PIN_1,
                                               DIR_RESET_PIN_2};

/* Initialize motor*/
MotorController controller = {.motor = {{.step_period = BASE_SPEED,
                                         .current_steps = 800,
                                         .angle = 12365,
                                         .step_dir = 1},
                                        {.step_period = BASE_SPEED,
                                         .current_steps = 800,
                                         .angle = 120,
                                         .step_dir = 1},
                                        {.step_period = BASE_SPEED,
                                         .current_steps = 800,
                                         .angle = 14270,
                                         .step_dir = 1}},
                              .pending_resets = 0};

volatile uint8_t safety_flag = 1;
uint16_t en0;
uint16_t en1;
uint16_t en2;
uint16_t *encoder[NUM_MOTORS] = {&en0, &en1, &en2};

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR |= EXTI_PR_PR13;
  }

  if (safety_flag) {
    safety_flag = 0;
    LED_OFF();
    TIM1->CR1 |= TIM_CR1_CEN;
    printS("SAFETY: OFF\r\n");
  } else {
    safety_flag = 1;
    LED_ON();
    TIM1->CR1 &= ~TIM_CR1_CEN;
    printS("SAFETY: ON\r\n");
  }
}

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

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

      controller.motor[i].current_steps += controller.motor[i].step_dir;
    }
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

static void trajectory_task(void *pvParameters) {
  (void)pvParameters;
  float n_start[3] = {0.0f, 0.0f, 1.0f};
  float h_start = 130.0f;
  float n_end[3] = {0.0f, 0.0f, 1.0f};
  float h_end = 110.0f;
  generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
  vTaskDelay(2000);

  n_start[0] = 0.0f;
  n_start[1] = 0.0f;
  n_start[2] = 1.0f;
  h_start = 110.0f;
  n_end[0] = 0.371391f;
  n_end[1] = 0.0f;
  n_end[2] = 0.928477f;
  h_end = 110.0f;
  generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
  vTaskDelay(2000);

  for (;;) {
    n_start[0] = 0.371391f;
    n_start[1] = 0.0f;
    n_start[2] = 0.928477f;
    h_start = 110.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.371391f;
    n_end[2] = 0.928477f;
    h_end = 110.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = 0.371391f;
    n_start[2] = 0.928477f;
    h_start = 110.0f;
    n_end[0] = -0.371391f;
    n_end[1] = 0.0f;
    n_end[2] = 0.928477f;
    h_end = 110.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    n_start[0] = -0.371391f;
    n_start[1] = 0.0f;
    n_start[2] = 0.928477f;
    h_start = 110.0f;
    n_end[0] = 0.0f;
    n_end[1] = -0.371391f;
    n_end[2] = 0.928477f;
    h_end = 110.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);

    n_start[0] = 0.0f;
    n_start[1] = -0.371391f;
    n_start[2] = 0.928477f;
    h_start = 110.0f;
    n_end[0] = 0.371391f;
    n_end[1] = 0.0f;
    n_end[2] = 0.928477f;
    h_end = 110.0f;
    generate_trajectory(n_start, h_start, n_end, h_end, traj_queue);
    vTaskDelay(2000);
  }
}

static void encoder_task(void *pvParameters) {
  (void)pvParameters;

  TickType_t last_wake = xTaskGetTickCount();
  uint8_t encoder_pins[NUM_MOTORS] = {8, 6, 5};

  for (;;) {
    // amt222b_read(8, &encoder0);
    // amt222b_read(6, &encoder1);
    // amt222b_read(5, &encoder2);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      amt222b_read(encoder_pins[i], encoder[i]);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
  }
}

static void controller_task(void *pvParameters) {
  (void)pvParameters;

  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

    TrajectoryPoint_t point;
    if (xQueueReceive(traj_queue, &point, 0) != pdTRUE)
      continue;

    /* VALIDATE TRAJECTORY WAYPOINT REACHED */

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      printS("encoder[");
      printI(i);
      printS("]: ");
      printI(*encoder[i]);
      printS("\t");
    }
    printS("\r\n");

    __disable_irq();

    uint32_t max_steps = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      // int32_t delta_angle = (point.target_angle[i] * (2 * M_PI / 16384)) -
      //                       controller.motor[i].angle_init;

      int32_t delta =
          point.target_position[i] - controller.motor[i].current_steps;

      if (delta < 0) {
        GPIOB->BSRR = dir_set[i];
        controller.motor[i].step_dir = -1;
        controller.motor[i].steps_remaining = (uint32_t)(-delta);
      } else {
        GPIOB->BSRR = dir_reset[i];
        controller.motor[i].step_dir = 1;
        controller.motor[i].steps_remaining = (uint32_t)(delta);
      }

      if (controller.motor[i].steps_remaining > max_steps) {
        max_steps = controller.motor[i].steps_remaining;
      }
    }

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (controller.motor[i].steps_remaining == 0)
        continue;

      controller.motor[i].step_period =
          TICKS_PER_CONTROL / controller.motor[i].steps_remaining;

      if (controller.motor[i].step_period == 0) {
        controller.motor[i].step_period = 1;
      }
    }

    __enable_irq();
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
  LED_ON();
  while (safety_flag)
    ;

  /* Initialize rtos */
  traj_queue = xQueueCreate(TRAJ_BUF_SIZE, sizeof(TrajectoryPoint_t));

  xTaskCreate(trajectory_task, "traj", 512, NULL, 2, NULL);
  xTaskCreate(encoder_task, "encd", 128, NULL, 4, NULL);
  xTaskCreate(controller_task, "ctrl", 256, NULL, 3, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  return 0;
}
