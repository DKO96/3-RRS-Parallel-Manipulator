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

void gpio_button(void) {
  // Configure GPIOC mode port for PC13
  GPIOC->MODER &= ~GPIO_MODER_MODE13;

  // Enable pull-up resistors
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD13_0;
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

void gpio_stepper3(void) {
  // Configure PA4 for TMC2209(3) step pin
  GPIOA->MODER &= ~GPIO_MODER_MODE4;
  GPIOA->MODER |= GPIO_MODER_MODE4_0;

  GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4;
  GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED4_Pos);

  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;

  // Configure PB15 for TMC2209(3) dir pin
  GPIOB->MODER &= ~GPIO_MODER_MODE15;
  GPIOB->MODER |= GPIO_MODER_MODE15_0;
}

void gpio_spi1(void) {
  /* Configure PA5, PA6, PA7, PB6 for SPI1
      PA5 -> SPI1_SCK (AF5)
      PA6 -> SPI1_MISO (AF5)
      PA7 -> SPI1_MOSI (AF5)

      PC8 -> SPI1_CS (GP)
      PC6 -> SPI1_CS (GP)
      PC5 -> SPI1_CS (GP)
  */

  // Configure SPI1
  GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOA->MODER |= ((2U << GPIO_MODER_MODE5_Pos) | (2U << GPIO_MODER_MODE6_Pos) |
                   (2U << GPIO_MODER_MODE7_Pos));

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
  GPIOA->AFR[0] |=
      ((5U << GPIO_AFRL_AFSEL5_Pos) | (5U << GPIO_AFRL_AFSEL6_Pos) |
       (5U << GPIO_AFRL_AFSEL7_Pos));

  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

  GPIOA->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEED5) | (GPIO_OSPEEDR_OSPEED6) |
                      (GPIO_OSPEEDR_OSPEED7));
  GPIOA->OSPEEDR |=
      ((3U << GPIO_OSPEEDR_OSPEED5_Pos) | (3U << GPIO_OSPEEDR_OSPEED6_Pos) |
       (3U << GPIO_OSPEEDR_OSPEED7_Pos));

  GPIOA->PUPDR &=
      ~((GPIO_PUPDR_PUPD5) | (GPIO_PUPDR_PUPD6) | (GPIO_PUPDR_PUPD7));
  GPIOA->PUPDR |= ((1U << GPIO_PUPDR_PUPD5_Pos) | (1U << GPIO_PUPDR_PUPD6_Pos) |
                   (1U << GPIO_PUPDR_PUPD7_Pos));

  // Configure CS pins
  GPIOC->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE6 | GPIO_MODER_MODE5);
  GPIOC->MODER |= ((1U << GPIO_MODER_MODE8_Pos) | (1U << GPIO_MODER_MODE6_Pos) |
                   (1U << GPIO_MODER_MODE5_Pos));

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD5);
  GPIOC->PUPDR |= ((1U << GPIO_PUPDR_PUPD8_Pos) | (1U << GPIO_PUPDR_PUPD6_Pos) |
                   (1U << GPIO_PUPDR_PUPD5_Pos));

  // Setting PC high before enabling SPI
  // Prevents the chip from being falsely selected
  GPIOC->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS6 | GPIO_BSRR_BS5;
}

void gpio_init(void) {
  // Enable clock access to GPIO A, B
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  gpio_usart2();
  gpio_button();
  gpio_stepper1();
  gpio_stepper2();
  gpio_stepper3();
  gpio_spi1();
}