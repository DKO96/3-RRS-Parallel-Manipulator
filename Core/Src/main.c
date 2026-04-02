#include "main.h"

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  uint32_t pins_to_set = 0;
  uint32_t resets = 0;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (steps_remaining[i] == 0) continue;

    step_counter[i]++;
    if (step_counter[i] >= step_period[i]) {
      step_counter[i] = 0;
      pins_to_set |= step_set[i];
      resets |= step_reset[i];
      steps_remaining[i]--;

      current_steps[i] += step_dir[i];
    }
  }

  if (pins_to_set) {
    GPIOA->BSRR = pins_to_set;
    pending_resets = resets;

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
  }
}

void TIM2_IRQHandler(void) {
  TIM2->SR &= ~TIM_SR_UIF;
  GPIOA->BSRR = pending_resets;
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
    h_start = 130.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 100.0f;

    follow_trajectory(n_start, h_start, n_end, h_end);

    delay_ms(3000);

    n_start[0] = 0.0f;
    n_start[1] = 0.0f;
    n_start[2] = 1.0f;
    h_start = 100.0f;
    n_end[0] = 0.0f;
    n_end[1] = 0.0f;
    n_end[2] = 1.0f;
    h_end = 130.0f;

    follow_trajectory(n_start, h_start, n_end, h_end);

    delay_ms(3000);
  }

  return 0;
}
