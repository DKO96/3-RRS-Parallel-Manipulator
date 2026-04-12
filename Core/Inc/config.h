#ifndef CONFIG_H_
#define CONFIG_H_

/* Math */
#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

/* Robot Geometry */
#define L1 65.0f
#define L2 65.0f
#define BASE 50.0f
#define PLATFORM 50.0f

#define NUM_MOTORS 3
#define STEPS_PER_REV 3200.0f
#define ALPHA (6.2831853f / STEPS_PER_REV)

/* Motor Behaviour */
#define BASE_SPEED 600
#define TRAP_STEPS 100

#define angle_resolution 0.0872665f
#define height_resolution 0.5f

#define CONTROL_FREQ 1000.0f
#define CONTROL_DT (1.0f / CONTROL_FREQ)

#define VEL_MAX 167.0f
#define ACC_MAX 1000.0f

#define TRAJ_BUF_SIZE 64

#endif /* CONFIG_H_ */