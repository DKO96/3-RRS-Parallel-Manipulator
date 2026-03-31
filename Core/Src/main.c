#include "main.h"

#include "math.h"

#define STEP_SET_PIN_0 GPIO_BSRR_BS0
#define STEP_SET_PIN_1 GPIO_BSRR_BS1
#define STEP_SET_PIN_2 GPIO_BSRR_BS4
#define STEP_RESET_PIN_0 GPIO_BSRR_BR0
#define STEP_RESET_PIN_1 GPIO_BSRR_BR1
#define STEP_RESET_PIN_2 GPIO_BSRR_BR4

#define DIR_SET_PIN_0 GPIO_BSRR_BS2
#define DIR_SET_PIN_1 GPIO_BSRR_BS1
#define DIR_SET_PIN_2 GPIO_BSRR_BS15
#define DIR_RESET_PIN_0 GPIO_BSRR_BR2
#define DIR_RESET_PIN_1 GPIO_BSRR_BR1
#define DIR_RESET_PIN_2 GPIO_BSRR_BR15

static const uint32_t step_set[NUM_MOTORS] = {STEP_SET_PIN_0, STEP_SET_PIN_1,
                                              STEP_SET_PIN_2};
static const uint32_t step_reset[NUM_MOTORS] = {
    STEP_RESET_PIN_0, STEP_RESET_PIN_1, STEP_RESET_PIN_2};
static const uint32_t dir_set[NUM_MOTORS] = {DIR_SET_PIN_0, DIR_SET_PIN_1,
                                             DIR_SET_PIN_2};
static const uint32_t dir_reset[NUM_MOTORS] = {DIR_RESET_PIN_0, DIR_RESET_PIN_1,
                                               DIR_RESET_PIN_2};

volatile uint32_t step_period[NUM_MOTORS] = {200, 200, 200};
volatile uint32_t step_counter[NUM_MOTORS] = {0, 0, 0};
volatile uint8_t motor_enabled[NUM_MOTORS] = {1, 1, 1};
volatile uint32_t steps_remaining[NUM_MOTORS] = {0, 0, 0};
volatile int32_t current_steps[NUM_MOTORS] = {800, 800, 800};
volatile int32_t target_steps[NUM_MOTORS] = {0, 0, 0};
volatile uint32_t pending_resets = 0;
volatile int8_t step_dir[NUM_MOTORS] = {1, 1, 1};

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  uint32_t pins_to_set = 0;
  uint32_t resets = 0;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (steps_remaining[i] == 0) continue;

    step_counter[i]++;
    if (step_counter[i] >= step_period[i]) {
      step_counter[i] = 0;
      pins_to_set |= step_set[i];
      resets |= step_reset[i];
      steps_remaining[i]--;

      current_steps[i] += step_dir[i];
    }
  }

  if (pins_to_set) {
    GPIOA->BSRR = pins_to_set;
    pending_resets = resets;

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
  }
}

void TIM2_IRQHandler(void) {
  TIM2->SR &= ~TIM_SR_UIF;
  GPIOA->BSRR = pending_resets;
}

void move_to_pose(float n[3], float h) {
  float theta[3] = {0, 0, 0};

  RRS_ik(n, h, theta);

  printS("delta: ");
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    uint32_t steps = angle_to_steps(theta[i]);

    target_steps[i] = (theta[i] >= 0) ? (int32_t)steps : -(int32_t)steps;

    int32_t delta = target_steps[i] - current_steps[i];
    printI(delta);
    printS("\t");

    if (delta < 0) {
      GPIOB->BSRR = dir_set[i];
      step_dir[i] = -1;
    } else {
      GPIOB->BSRR = dir_reset[i];
      step_dir[i] = 1;
    }

    steps_remaining[i] = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
  }
}

uint8_t move_complete(void) {
  return (steps_remaining[0] == 0 && steps_remaining[1] == 0 &&
          steps_remaining[2] == 0);
}

void follow_trajectory(float n_start[3], float h_start, float n_end[3],
                       float h_end) {
  // Calculate number of steps
  float angle_change = acosf(vec3_dot(n_start, n_end));
  float height_change = fabsf(h_end - h_start);

  uint32_t angle_steps = (uint32_t)(angle_change / angle_resolution);
  uint32_t height_steps = (uint32_t)(height_change / height_resolution);
  uint32_t num_steps =
      (angle_steps > height_steps) ? angle_steps : height_steps;

  if (num_steps == 0) return;

  // Send each waypoint to timer
  for (uint32_t i = 1; i <= num_steps; i++) {
    float t = (float)i / (float)num_steps;

    float n[3] = {0.0f, 0.0f, 0.0f};
    n[0] = (1 - t) * n_start[0] + t * n_end[0];
    n[1] = (1 - t) * n_start[1] + t * n_end[1];
    n[2] = (1 - t) * n_start[2] + t * n_end[2];

    float mag = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    n[0] /= mag;
    n[1] /= mag;
    n[2] /= mag;

    float h = (1 - t) * h_start + t * h_end;

    move_to_pose(n, h);

    while (!move_complete())
      ;
  }
}

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  uart_init(USART2);
  timer1_init();
  timer2_init();

  float n_start[3] = {0.0f, 0.0f, 1.0f};
  float h_start = 108.0f;

  float n_end[3] = {0.0f, -0.5f, 1.0f};
  float h_end = 70.0f;

  move_to_pose(n_end, h_end);
  // follow_trajectory(n_start, h_start, n_end, h_end);

  printS("\r\n");

  while (1) {
  }

  return 0;
}
