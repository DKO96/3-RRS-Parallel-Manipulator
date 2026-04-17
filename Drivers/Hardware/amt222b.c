#include "amt222b.h"

#include "main.h"
#include "spi.h"

void amt222b_read(uint8_t pin, uint16_t *position) {
  GPIOC->BSRR = (1U << (pin + 16));
  delay_us(3);

  uint8_t high = spi_transfer(0x00);
  delay_us(3);
  uint8_t low = spi_transfer(0x00);

  delay_us(3);
  GPIOC->BSRR = (1U << pin);

  // Odd parity over odd-position bits
  uint8_t k1 = !(((high >> 5) ^ (high >> 3) ^ (high >> 1) ^ (low >> 7) ^
                  (low >> 5) ^ (low >> 3) ^ (low >> 1)) &
                 1);

  // Even parity over even-position bits
  uint8_t k0 = !(((high >> 4) ^ (high >> 2) ^ (high >> 0) ^ (low >> 6) ^
                  (low >> 4) ^ (low >> 2) ^ (low >> 0)) &
                 1);

  if ((k1 == ((high >> 7) & 1)) && (k0 == ((high >> 6) & 1))) {
    *position = (uint16_t)((high & 0x3F) << 8 | low);
  }
}
