#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#define M_PI_F 3.14159265358979323846f
#define ONE_BY_SQRT3 0.57735026919f
#define TWO_BY_SQRT3 1.15470053838f
#define SQRT3_BY_2 0.86602540378f
#define CLAMP(value, min_value, max_value) (((value) < (min_value)) ? (min_value) : (((value) > (max_value)) ? (max_value) : (value)))

typedef struct {
    float d;
    float q;
} Float2D;

int svm(float alpha, float beta, float *t_a, float *t_b, float *t_c);
float wrap_pm(float x, float y);
float fmodf_pos(float x, float y);
float wrap_pm_pi(float x);

#endif
