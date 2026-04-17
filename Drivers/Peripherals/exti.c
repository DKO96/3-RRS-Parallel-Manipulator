#include "exti.h"

void exti_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

  EXTI->IMR |= EXTI_IMR_MR13;

  // Detect on falling edge
  EXTI->FTSR |= EXTI_FTSR_TR13;

  EXTI->PR |= EXTI_PR_PR13;

  NVIC_SetPriority(EXTI15_10_IRQn, 1);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}