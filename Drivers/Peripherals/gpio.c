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

void gpio_i2c1(void) {
  /* Configure PB8, PB9 for I2C1
      PB8 -> I2C1_SCL (AF mode)
      PB9 -> I2C1_SDA (AF mode)
  */
  GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
  GPIOB->MODER |= ((2U << GPIO_MODER_MODE8_Pos) | (2U << GPIO_MODER_MODE9_Pos));

  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
  GPIOB->AFR[1] |=
      ((4U << GPIO_AFRH_AFSEL8_Pos) | (4U << GPIO_AFRH_AFSEL9_Pos));

  GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);

  GPIOB->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEED8) | (GPIO_OSPEEDR_OSPEED9));
  GPIOB->OSPEEDR |=
      ((3U << GPIO_OSPEEDR_OSPEED8_Pos) | (3U << GPIO_OSPEEDR_OSPEED9_Pos));

  GPIOB->PUPDR &= ~((GPIO_PUPDR_PUPD8) | (GPIO_PUPDR_PUPD9));
  GPIOB->PUPDR |= ((1U << GPIO_PUPDR_PUPD8_Pos) | (1U << GPIO_PUPDR_PUPD9_Pos));
}

void gpio_init(void) {
  // Enable clock access to GPIO A, B
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  gpio_usart2();
  gpio_i2c1();
}