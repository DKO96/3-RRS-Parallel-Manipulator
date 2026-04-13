#include "config.h"
#include "math.h"
#include "stm32f446xx.h"
#include "uart.h"

static float p[3] = {PLATFORM, 0.0f, 0.0f};
static const float alpha[3] = {
    0,
    DEG_TO_RAD(120.0f),
    DEG_TO_RAD(240.0f),
};

void mat_mul_3x3(float A[3][3], float B[3][3], float result[3][3]) {
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      result[i][j] = 0.0;
      for (uint8_t k = 0; k < 3; k++) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void mat_vec_mul_3x1(float A[3][3], float v[3], float result[3]) {
  for (uint8_t i = 0; i < 3; i++) {
    result[i] = 0.0;
    for (uint8_t j = 0; j < 3; j++) {
      result[i] += A[i][j] * v[j];
    }
  }
}

void vec3_add(float a[3], float b[3], float result[3]) {
  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];
  result[2] = a[2] + b[2];
}

float vec3_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void RRS_ik(float n[3], float h, float theta[3]) {
  float psi_y = asinf(n[0]);

  if (fabsf(psi_y) == 1) return;

  float psi_x = asinf(-n[1] / cosf(psi_y));
  float psi_z =
      atanf((-sinf(psi_x) * sinf(psi_y)) / (cosf(psi_x) + cosf(psi_y)));

  float sx = sinf(psi_x);
  float cx = cosf(psi_x);
  float sy = sinf(psi_y);
  float cy = cosf(psi_y);
  float sz = sinf(psi_z);
  float cz = cosf(psi_z);

  float R[3][3];
  R[0][0] = cy * cz;
  R[0][1] = -cy * sz;
  R[0][2] = sy;

  R[1][0] = cx * sz + sx * sy * cz;
  R[1][1] = cx * cz - sx * sy * sz;
  R[1][2] = -sx * cy;

  R[2][0] = sx * sz - cx * sy * cz;
  R[2][1] = sx * cz + cx * sy * sz;
  R[2][2] = cx * cy;

  float Q[3];
  Q[0] = (PLATFORM * (R[0][0] - R[1][1])) * 0.5f;
  Q[1] = -R[1][0] * PLATFORM;
  Q[2] = h;

  for (uint8_t i = 0; i < 3; i++) {
    float sa = sinf(alpha[i]);
    float ca = cosf(alpha[i]);

    float Rz[3][3];
    Rz[0][0] = ca;
    Rz[0][1] = -sa;
    Rz[0][2] = 0;

    Rz[1][0] = sa;
    Rz[1][1] = ca;
    Rz[1][2] = 0;

    Rz[2][0] = 0;
    Rz[2][1] = 0;
    Rz[2][2] = 1;

    float temp1[3];
    float temp2[3];
    float S[3];
    mat_vec_mul_3x1(Rz, p, temp1);
    mat_vec_mul_3x1(R, temp1, temp2);
    vec3_add(Q, temp2, S);

    float A = 2 * L1 * ca * (-S[0] + BASE * ca);
    float B = 2 * L1 * S[2] * ca * ca;
    float C = S[0] * S[0] - 2 * BASE * S[0] * ca +
              ca * ca * (BASE * BASE + L1 * L1 - L2 * L2 + S[2] * S[2]);

    // float t = (-B + sqrtf(A * A + B * B - C * C)) / (C - A);
    // theta[i] = -2 * atanf(t);

    float disc = A * A + B * B - C * C;

    // DEBUG: output A, B, C
    // if (i == 0) {
    //   printS("A=");
    //   printF(A);
    //   printS(" B=");
    //   printF(B);
    //   printS(" C=");
    //   printF(C);
    //   printS(" disc=");
    //   printF(disc);
    //   printS(" C-A=");
    //   printF(C - A);
    //   printS("\r\n");
    // }

    if (disc < 0.0f) {
      printS("WARN: disc<0 motor ");
      printI(i);
      printS(" disc=");
      printF(disc);
      printS("\r\n");
      disc = 0.0f;
    }

    float t = (-B + sqrtf(disc)) / (C - A);
    theta[i] = -2 * atanf(t);
  }

  // DEBUG: output theta
  // printS("theta: ");
  // printF(RAD_TO_DEG(theta[0]));
  // printS(" \t");
  // printF(RAD_TO_DEG(theta[1]));
  // printS(" \t");
  // printF(RAD_TO_DEG(theta[2]));
  // printS("\r\n");
}