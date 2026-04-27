#include "trajectory.h"

#include "FreeRTOS.h"
#include "ik.h"
#include "math.h"
#include "queue.h"
#include "trajectory.h"
#include "uart.h"

void trap_profile_compute(TrapezoidalProfile_t *p, float distance) {
  //
  if (distance <= 0.0f) {
    p->D = 0.0f;
    p->T = 0.0f;
    p->t_accel = 0.0f;
    p->t_const = 0.0f;
    p->velocity = 0.0f;
    p->accel = 0.0f;
    return;
  }

  float t_accel = VEL_MAX / ACC_MAX;
  float d_accel = 0.5f * ACC_MAX * t_accel * t_accel;

  if (2.0f * d_accel > distance) {
    // Triangular profile
    t_accel = sqrtf(distance / ACC_MAX);
    p->velocity = ACC_MAX * t_accel;
    p->t_const = 0.0f;
  } else {
    // Trapezoidal profile
    float d_const = distance - 2.0f * d_accel;
    p->t_const = d_const / VEL_MAX;
    p->velocity = VEL_MAX;
  }

  p->D = distance;
  p->T = 2.0f * t_accel + p->t_const;
  p->t_accel = t_accel;
  p->accel = ACC_MAX;
}

float trap_progress(TrapezoidalProfile_t *p, float time) {
  // Check if move has already completed
  if (p->D <= 0.0f) {
    return 1.0f;
  }

  if (time >= p->T) {
    return 1.0f;
  }

  // Check if beginning of move
  if (time <= 0.0f) {
    return 0.0f;
  }

  // Determine phase
  float pos;
  if (time <= p->t_accel) {
    // in acceleration phase
    pos = 0.5f * p->accel * time * time;
  } else if (time <= p->t_accel + p->t_const) {
    // in constant velocity phase
    float d_accel = 0.5 * p->accel * p->t_accel * p->t_accel;
    pos = d_accel + p->velocity * (time - p->t_accel);
  } else {
    // in deceleration phase
    float t_remain = p->T - time;
    pos = p->D - 0.5f * p->accel * t_remain * t_remain;
  }

  return pos / p->D;
}

void generate_trajectory(float n_start[3], float h_start, float n_end[3],
                         float h_end, QueueHandle_t q) {
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
      point.target_angle[i] = theta[i];
    }
    xQueueSend(q, &point, portMAX_DELAY);

    t += CONTROL_DT;
  }

  /* 5. Push final waypoint to queue */
  TrajectoryPoint_t final_point;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    final_point.target_angle[i] = theta_end[i];
  }
  xQueueSend(q, &final_point, portMAX_DELAY);
}
