#include "controller.h"

#include "ik.h"
#include "math.h"
#include "uart.h"

const uint32_t step_set[NUM_MOTORS] = {STEP_SET_PIN_0, STEP_SET_PIN_1,
                                       STEP_SET_PIN_2};
const uint32_t step_reset[NUM_MOTORS] = {STEP_RESET_PIN_0, STEP_RESET_PIN_1,
                                         STEP_RESET_PIN_2};
static const uint32_t dir_set[NUM_MOTORS] = {DIR_SET_PIN_0, DIR_SET_PIN_1,
                                             DIR_SET_PIN_2};
static const uint32_t dir_reset[NUM_MOTORS] = {DIR_RESET_PIN_0, DIR_RESET_PIN_1,
                                               DIR_RESET_PIN_2};

uint32_t angle_to_steps(float angle) {
  if (angle < 0) {
    angle = -angle;
  }

  return (uint32_t)(angle / ALPHA);
}

static void move_to_pose(MotorController *mc, float n[3], float h,
                         float speed_factor) {
  float theta[3] = {0, 0, 0};
  RRS_ik(n, h, theta);

  // DEBUG: print IK inputs and outputs
  // printS("--- move_to_pose ---\r\n");
  // printS("n: ");
  // printF(n[0]);
  // printS(", ");
  // printF(n[1]);
  // printS(", ");
  // printF(n[2]);
  // printS("\r\n");
  // printS("h: ");
  // printF(h);
  // printS("\r\n");

  // printS("theta: ");
  // printF(theta[0]);
  // printS(", ");
  // printF(theta[1]);
  // printS(", ");
  // printF(theta[2]);
  // printS("\r\n");

  __disable_irq();

  uint32_t max_steps = 0;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    uint32_t steps = angle_to_steps(theta[i]);

    mc->motor[i].target_steps =
        (theta[i] >= 0) ? (int32_t)steps : -(int32_t)steps;

    int32_t delta = mc->motor[i].target_steps - mc->motor[i].current_steps;

    if (delta < 0) {
      GPIOB->BSRR = dir_set[i];
      mc->motor[i].step_dir = -1;
    } else {
      GPIOB->BSRR = dir_reset[i];
      mc->motor[i].step_dir = 1;
    }

    mc->motor[i].steps_remaining =
        (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;

    max_steps = (mc->motor[i].steps_remaining > max_steps)
                    ? mc->motor[i].steps_remaining
                    : max_steps;
  }

  // DEBUG: print motion plan per motor
  // for (uint8_t i = 0; i < NUM_MOTORS; i++) {
  //   printS("M");
  //   printI(i);
  //   printS(" cur=");
  //   printI(mc->motor[i].current_steps);
  //   printS(" tgt=");
  //   printI(mc->motor[i].target_steps);
  //   printS(" rem=");
  //   printI(mc->motor[i].steps_remaining);
  //   printS(" dir=");
  //   printI(mc->motor[i].step_dir);
  //   printS("\r\n");
  // }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (mc->motor[i].steps_remaining == 0)
      continue;

    uint32_t scaled_speed = (uint32_t)(BASE_SPEED / speed_factor);

    mc->motor[i].step_period =
        scaled_speed * max_steps / mc->motor[i].steps_remaining;
  }

  __enable_irq();
}

uint8_t move_complete(MotorController *mc) {
  return (mc->motor[0].steps_remaining == 0 &&
          mc->motor[1].steps_remaining == 0 &&
          mc->motor[2].steps_remaining == 0);
}

static float trapezoidal_control(uint32_t i, uint32_t num_steps) {
  float min_speed = 0.2f;

  /* Triangular Profile */
  if (num_steps <= 2 * TRAP_STEPS) {
    uint32_t half_steps = num_steps / 2;

    if (i <= half_steps) {
      return min_speed + (1.0f - min_speed) * ((float)i / (float)half_steps);
    } else {
      return min_speed +
             (1.0f - min_speed) * ((float)(num_steps - i) / (float)half_steps);
    }
  }

  /* Trapezoidal Profile */
  if (i <= TRAP_STEPS) {
    return min_speed + (1.0f - min_speed) * ((float)i / (float)TRAP_STEPS);
  } else if (i >= num_steps - TRAP_STEPS) {
    return min_speed +
           (1.0f - min_speed) * ((float)(num_steps - i) / (float)TRAP_STEPS);
  } else {
    return 1.0f;
  }
}

void follow_trajectory(MotorController *mc, float n_start[3], float h_start,
                       float n_end[3], float h_end) {
  // Calculate number of steps
  float angle_change = acosf(vec3_dot(n_start, n_end));
  float height_change = fabsf(h_end - h_start);

  uint32_t angle_steps = (uint32_t)(angle_change / angle_resolution);
  uint32_t height_steps = (uint32_t)(height_change / height_resolution);
  uint32_t num_steps =
      (angle_steps > height_steps) ? angle_steps : height_steps;

  if (num_steps == 0)
    return;

  // Precompute first waypoint
  int32_t next_target[NUM_MOTORS];
  float t = 1.0f / (float)num_steps;
  float n[3] = {0.0f, 0.0f, 0.0f};
  n[0] = (1 - t) * n_start[0] + t * n_end[0];
  n[1] = (1 - t) * n_start[1] + t * n_end[1];
  n[2] = (1 - t) * n_start[2] + t * n_end[2];

  float mag = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  n[0] /= mag;
  n[1] /= mag;
  n[2] /= mag;

  float h = (1 - t) * h_start + t * h_end;

  float theta[3];
  RRS_ik(n, h, theta);

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    uint32_t steps = angle_to_steps(theta[i]);
    next_target[i] = (theta[i] >= 0) ? (int32_t)steps : -(int32_t)steps;
  }

  for (uint32_t i = 1; i <= num_steps; i++) {
    float speed_factor = trapezoidal_control(i, num_steps);
    uint32_t scaled_speed = (uint32_t)(BASE_SPEED / speed_factor);
    __disable_irq();

    uint32_t max_steps = 0;
    for (uint8_t j = 0; j < NUM_MOTORS; j++) {
      int32_t delta = next_target[j] - mc->motor[j].current_steps;

      // DEBUG: print delta
      // printS("M");
      // printI(j);
      // printS(" d=");
      // printI(delta);
      // printS(" ");

      if (delta < 0) {
        GPIOB->BSRR = dir_set[j];
        mc->motor[j].step_dir = -1;
        mc->motor[j].steps_remaining = (uint32_t)(-delta);
      } else {
        GPIOB->BSRR = dir_reset[j];
        mc->motor[j].step_dir = 1;
        mc->motor[j].steps_remaining = (uint32_t)delta;
      }

      if (mc->motor[j].steps_remaining > max_steps) {
        max_steps = mc->motor[j].steps_remaining;
      }
    }
    // printS("\r\n");

    for (uint8_t j = 0; j < NUM_MOTORS; j++) {
      if (mc->motor[j].steps_remaining == 0)
        continue;

      mc->motor[j].step_period =
          scaled_speed * max_steps / mc->motor[j].steps_remaining;
    }

    __enable_irq();

    // DEBUG: output steps remaining before move
    // printS("W");
    // printI(i);
    // printS(" rem: ");
    // printI(mc->motor[0].steps_remaining);
    // printS(" ");
    // printI(mc->motor[1].steps_remaining);
    // printS(" ");
    // printI(mc->motor[2].steps_remaining);
    // printS("\r\n");

    if (i < num_steps) {
      float t = (float)(i + 1) / (float)num_steps;

      float n[3] = {0.0f, 0.0f, 0.0f};
      n[0] = (1 - t) * n_start[0] + t * n_end[0];
      n[1] = (1 - t) * n_start[1] + t * n_end[1];
      n[2] = (1 - t) * n_start[2] + t * n_end[2];

      float mag = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      n[0] /= mag;
      n[1] /= mag;
      n[2] /= mag;

      float h = (1 - t) * h_start + t * h_end;

      float theta[3];
      RRS_ik(n, h, theta);

      for (uint8_t j = 0; j < NUM_MOTORS; j++) {
        uint32_t steps = angle_to_steps(theta[j]);
        next_target[j] = (theta[j] >= 0) ? (int32_t)steps : -(int32_t)steps;
      }
    }
    while (!move_complete(mc))
      ;
  }
}

// void follow_trajectory(MotorController *mc, float n_start[3], float h_start,
//                        float n_end[3], float h_end) {
//   // Calculate number of steps
//   float angle_change = acosf(vec3_dot(n_start, n_end));
//   float height_change = fabsf(h_end - h_start);

//   uint32_t angle_steps = (uint32_t)(angle_change / angle_resolution);
//   uint32_t height_steps = (uint32_t)(height_change / height_resolution);
//   uint32_t num_steps =
//       (angle_steps > height_steps) ? angle_steps : height_steps;

//   if (num_steps == 0)
//     return;

//   for (uint32_t i = 1; i <= num_steps; i++) {
//     float t = (float)i / (float)num_steps;

//     float speed_factor = trapezoidal_control(i, num_steps);

//     float n[3] = {0.0f, 0.0f, 0.0f};
//     n[0] = (1 - t) * n_start[0] + t * n_end[0];
//     n[1] = (1 - t) * n_start[1] + t * n_end[1];
//     n[2] = (1 - t) * n_start[2] + t * n_end[2];

//     float mag = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
//     n[0] /= mag;
//     n[1] /= mag;
//     n[2] /= mag;

//     float h = (1 - t) * h_start + t * h_end;

//     move_to_pose(mc, n, h, speed_factor);

//     while (!move_complete(mc))
//       ;
//   }
// }