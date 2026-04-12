#include "trajectory.h"

#include "math.h"

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
