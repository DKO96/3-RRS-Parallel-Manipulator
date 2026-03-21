/*
 *    Configure system clock for the stm32f446re
 *
 * MCU clock speeds
 *
 * SYSCLK           = 180MHz
 * PCLK1_timer      = 90MHz
 * PCLK1_peripheral = 45MHz
 * PCLK2_timer      = 180MHz
 * PCLK2_peripheral = 90MHz
 *
 */

#include "main.h"

#define PLL_M 4
#define PLL_N 180
#define PLL_P 0
#define PLL_Q 7

#define SYSCLK 180000000
#define PCLK 45000000
#define BAUDRATE 115200

void system_clock_init(void) {
  // Enable HSE (HSEON) and wait for HSE to be ready (HSERDY)
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;

  // Enable the power interface clock (PWREN)
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  // Enable the voltage regulator (VOS) and set to scale 1
  PWR->CR &= ~PWR_CR_VOS;
  PWR->CR |= (PWR_CR_VOS_0 | PWR_CR_VOS_1);

  // Enable instruction cache (ICEN), data cache (DCEN), and pre-fetch (PRFTEN)
  FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);

  // Configure latency (LATENCY)
  FLASH->ACR &= ~(FLASH_ACR_LATENCY);
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

  // Configure bus pre-scalers (HPRE, PPRE1, PPRE2)
  RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
  RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 |   // AHB1 pre-scaler, SYSCLK not divided
                RCC_CFGR_PPRE1_DIV4 |  // APB1 pre-scaler, HCLK divided by 4
                RCC_CFGR_PPRE2_DIV2    // APB2 pre-scaler, HCLK divided by 2
  );

  // Configure parameters (PLLM, PLLN, PLLP, PLLQ) and set HSE as source
  // (PLLSRC)
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP |
                    RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC);

  RCC->PLLCFGR |=
      ((PLL_M << RCC_PLLCFGR_PLLM_Pos) | (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
       (PLL_P << RCC_PLLCFGR_PLLP_Pos) | (PLL_Q << RCC_PLLCFGR_PLLQ_Pos) |
       RCC_PLLCFGR_PLLSRC_HSE);

  // Enable PLL (PLLON) and wait for PLL to be ready (PLLRDY)
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  // Configure system clock source (SW) to use PLL and wait for it to be set
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;
}

void dwt_init(void) {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SYSCLK / 1000000);
  while ((DWT->CYCCNT - start) < ticks)
    ;
}

void delay_ms(uint32_t ms) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = ms * (SYSCLK / 1000);
  while ((DWT->CYCCNT - start) < ticks)
    ;
}

void system_init(void) {
  system_clock_init();
  dwt_init();

  // Enable FPU
  SCB->CPACR |=
      ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
}