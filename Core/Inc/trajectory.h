#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "FreeRTOS.h"
#include "config.h"
#include "queue.h"
#include "stm32f446xx.h"

typedef struct {
  float D;         // total distance [steps]
  float T;         // total move time [seconds]
  float t_accel;   // acceleration phase duration [seconds]
  float t_const;   // constant velocity duration [seconds]
  float velocity;  // constant velocity [steps/sec]
  float accel;     // acceleration [steps/sec^2] either ACC_MAX or 0
} TrapezoidalProfile_t;

typedef struct {
  int32_t target_position[NUM_MOTORS];
} TrajectoryPoint_t;

typedef struct {
  TrajectoryPoint_t points[TRAJ_BUF_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} TrajectoryBuffer_t;

void trap_profile_compute(TrapezoidalProfile_t *p, float distance);
float trap_progress(TrapezoidalProfile_t *p, float time);

void generate_trajectory(float n_start[3], float h_start, float n_end[3],
                         float h_end, QueueHandle_t q);

#endif /* TRAJECTORY_H_ */