#include "controller.h"

#include "config.h"

uint32_t angle_to_steps(float angle) {
  if (angle < 0) {
    angle = -angle;
  }

  return (uint32_t)(angle / ALPHA);
}
