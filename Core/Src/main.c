#include "main.h"

/* Initialize motor*/
MotorController controller = {
    .motor = {{.step_period = 200, .current_steps = 800, .step_dir = 1},
              {.step_period = 200, .current_steps = 800, .step_dir = 1},
              {.step_period = 200, .current_steps = 800, .step_dir = 1}},
    .pending_resets = 0};

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

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  uart_init(USART2);
  timer1_init();
  timer2_init();

  float n_start[3];
  float h_start;
  float n_end[3];
  float h_end;

  while (1) {
    n_start[0] = 0.0f;
    n_start[1] = 0.0f;
    n_start[2] = 1.0f;
    h_start = 120.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 80.0f;

    printS("moving to height: ");
    printI(80);
    printS("\r\n");
    follow_trajectory(&controller, n_start, h_start, n_end, h_end);

    delay_ms(2000);

    n_start[0] = 0.0f;
    n_start[1] = 0.0f;
    n_start[2] = 1.0f;
    h_start = 80.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 120.0f;

    printS("moving to height: ");
    printI(120);
    printS("\r\n");

    follow_trajectory(&controller, n_start, h_start, n_end, h_end);

    delay_ms(2000);
  }

  return 0;
}
