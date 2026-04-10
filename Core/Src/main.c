#include "main.h"

#include "math.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

QueueHandle_t ik_queue;
SemaphoreHandle_t motion_completel_semphr;

#define LED_ON() (GPIOA->ODR |= GPIO_ODR_OD5)
#define LED_OFF() (GPIOA->ODR &= ~GPIO_ODR_OD5)

/* Initialize motor*/
MotorController controller = {
    .motor = {{.step_period = BASE_SPEED, .current_steps = 800, .step_dir = 1},
              {.step_period = BASE_SPEED, .current_steps = 800, .step_dir = 1},
              {.step_period = BASE_SPEED, .current_steps = 800, .step_dir = 1}},
    .pending_resets = 0};

volatile uint8_t safety_flag = 1;

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
    if (controller.motor[i].steps_remaining == 0) continue;

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

static void ik_task(void *pvParameters) {}

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  exti_init();
  uart_init(USART2);
  timer1_init();
  timer2_init();

  printS("\r\n=========PROGRAM START=============\r\n");
  LED_ON();
  while (safety_flag)
    ;

  // while (1) {
  // }

  /* Initialize rtos */
  ik_queue = xQueueCreate(1, 1);

  motion_completel_semphr = xSemaphoreCreateBinary();

  xTaskCreate();

  /* Start scheduler */
  vTaskStartScheduler();

  return 0;
}
