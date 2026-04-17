#include "timer.h"

void timer1_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->PSC = 9;
  TIM1->ARR = 89;

  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->EGR |= TIM_EGR_UG;
  TIM1->SR &= ~TIM_SR_UIF;
  TIM1->DIER |= TIM_DIER_UIE;

  // TIM1->CR1 |= TIM_CR1_CEN;

  // Configure TIM1 interrupt in NVIC
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 6);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void timer2_init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->PSC = 0;
  TIM2->ARR = 90;

  TIM2->CR1 |= TIM_CR1_OPM;
  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR &= ~TIM_SR_UIF;

  // Configure TIM1 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 5);
  NVIC_EnableIRQ(TIM2_IRQn);
}