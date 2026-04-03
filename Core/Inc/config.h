#ifndef CONFIG_H_
#define CONFIG_H_

/* Math */
#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.f))

/* Robot Geometry */
#define L1 65.0f
#define L2 65.0f
#define BASE 50.0f
#define PLATFORM 50.0f

#define NUM_MOTORS 3
#define STEPS_PER_REV 3200.0f
#define ALPHA (6.2831853f / STEPS_PER_REV)

/* Motor Behaviour */
#define BASE_SPEED 200
#define TRAP_STEPS 50

#define angle_resolution 0.01745329f
#define height_resolution 0.05f

#endif /* CONFIG_H_ */