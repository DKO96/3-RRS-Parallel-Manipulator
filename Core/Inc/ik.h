#ifndef IK_H_
#define IK_H_

void mat_mul_3x3(float A[3][3], float B[3][3], float result[3][3]);
void mat_vec_mul_3x1(float A[3][3], float v[3], float result[3]);
void vec3_add(float a[3], float b[3], float result[3]);
void RRS_ik(float n[3], float h, float theta[3]);

#endif /* IK_H_ */