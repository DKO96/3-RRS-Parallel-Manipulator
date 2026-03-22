#include "gpio.h"

void gpio_usart2(void) {
  // Configure PA2, PA3 for USART2
  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
  GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

  // AF7 for pins PA2, PA3
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOA->AFR[0] |=
      ((7U << GPIO_AFRL_AFSEL2_Pos) | (7U << GPIO_AFRL_AFSEL3_Pos));
}

void gpio_stepper1(void) {
  // Configure PA0 for TMC2209(1) step pin
  GPIOA->MODER &= ~GPIO_MODER_MODE0;
  GPIOA->MODER |= GPIO_MODER_MODE0_0;

  GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;
  GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED0_Pos);

  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;

  // Configure PB2 for TMC2209(2) dir pin
  GPIOB->MODER &= ~GPIO_MODER_MODE2;
  GPIOB->MODER |= GPIO_MODER_MODE2_0;
}

void gpio_stepper2(void) {
  // Configure PA1 for TMC2209(2) step pin
  GPIOA->MODER &= ~GPIO_MODER_MODE1;
  GPIOA->MODER |= GPIO_MODER_MODE1_0;

  GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;
  GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED1_Pos);

  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;

  // Configure PB1 for TMC2209(1) dir pin
  GPIOB->MODER &= ~GPIO_MODER_MODE1;
  GPIOB->MODER |= GPIO_MODER_MODE1_0;
}

void gpio_init(void) {
  // Enable clock access to GPIO A, B
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  gpio_usart2();
  gpio_stepper1();
  gpio_stepper2();
}