#include "controller.h"

#include "FreeRTOS.h"
#include "ik.h"
#include "math.h"
#include "queue.h"
#include "trajectory.h"
#include "uart.h"

const uint32_t step_set[NUM_MOTORS] = {STEP_SET_PIN_0, STEP_SET_PIN_1,
                                       STEP_SET_PIN_2};
const uint32_t step_reset[NUM_MOTORS] = {STEP_RESET_PIN_0, STEP_RESET_PIN_1,
                                         STEP_RESET_PIN_2};
static const uint32_t dir_set[NUM_MOTORS] = {DIR_SET_PIN_0, DIR_SET_PIN_1,
                                             DIR_SET_PIN_2};
static const uint32_t dir_reset[NUM_MOTORS] = {DIR_RESET_PIN_0, DIR_RESET_PIN_1,
                                               DIR_RESET_PIN_2};

void generate_trajectory(MotorController *mc, float n_start[3], float h_start,
                         float n_end[3], float h_end, QueueHandle_t *q) {
  /* 1. Compute trajectory endpoints */
  float theta_start[3], theta_end[3];
  RRS_ik(n_start, h_start, theta_start);
  RRS_ik(n_end, h_end, theta_end);

  // Convert joint angles into steps
  int32_t start_steps[3], end_steps[3];
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    start_steps[i] = (int32_t)(theta_start[i] / ALPHA);
    end_steps[i] = (int32_t)(theta_end[i] / ALPHA);
  }

  /* 2. Determine max distance (steps) from motors */
  uint32_t max_delta = 0;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    int32_t delta = end_steps[i] - start_steps[i];
    uint32_t abs_delta = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)(delta);

    if (abs_delta > max_delta) {
      max_delta = abs_delta;
    }
  }

  if (max_delta == 0) return;

  /* 3. Build trapezoidal profile from max distance */
  TrapezoidalProfile_t profile;
  trap_profile_compute(&profile, (float)max_delta);

  if (profile.T <= 0.0f) return;

  /* 4. Sample trajectory waypoints */
  float t = 0.0f;
  while (t <= profile.T) {
    float s = trap_progress(&profile, t);

    // Interpolate
    float n[3];
    n[0] = (1.0f - s) * n_start[0] + s * n_end[0];
    n[1] = (1.0f - s) * n_start[1] + s * n_end[1];
    n[2] = (1.0f - s) * n_start[2] + s * n_end[2];

    float mag = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    n[0] /= mag;
    n[1] /= mag;
    n[2] /= mag;

    float h = (1.0f - s) * h_start + s * h_end;

    // Solve IK
    float theta[3];
    RRS_ik(n, h, theta);

    // Push waypoint to queue
    TrajectoryPoint_t point;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      point.target_position[i] = (int32_t)(theta[i] / ALPHA);
    }
    xQueueSend(&q, &point, portMAX_DELAY);

    t += CONTROL_DT;
  }

  /* 5. Push final waypoint to queue */
  TrajectoryPoint_t final_point;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    final_point.target_position[i] = end_steps[i];
  }
  xQueueSend(&q, &final_point, portMAX_DELAY);
}
