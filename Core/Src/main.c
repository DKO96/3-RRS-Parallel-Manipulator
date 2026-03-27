#include "main.h"

#define STEP_PIN_0 GPIO_BSRR_BS0
#define STEP_PIN_1 GPIO_BSRR_BS1
#define STEP_PIN_2 GPIO_BSRR_BS4
#define RESET_PIN_0 GPIO_BSRR_BR0
#define RESET_PIN_1 GPIO_BSRR_BR1
#define RESET_PIN_2 GPIO_BSRR_BR4

static const uint32_t step_set[NUM_MOTORS] = {STEP_PIN_0, STEP_PIN_1,
                                              STEP_PIN_2};
static const uint32_t step_reset[NUM_MOTORS] = {RESET_PIN_0, RESET_PIN_1,
                                                RESET_PIN_2};

volatile uint32_t step_period[NUM_MOTORS] = {100, 100, 100};
volatile uint32_t step_counter[NUM_MOTORS] = {0, 0, 0};
volatile uint8_t motor_enabled[NUM_MOTORS] = {1, 1, 1};
volatile uint32_t steps_remaining[NUM_MOTORS] = {3200, 3200, 3200};
volatile uint32_t pending_resets = 0;

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

  while (1) {
  }

  return 0;
}