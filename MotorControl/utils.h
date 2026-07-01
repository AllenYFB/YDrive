#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#define M_PI_F 3.14159265358979323846f
#define ONE_BY_SQRT3 0.57735026919f
#define TWO_BY_SQRT3 1.15470053838f
#define SQRT3_BY_2 0.86602540378f

typedef struct {
    float d;
    float q;
} Float2D;

static inline float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static inline uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

int svm(float alpha, float beta, float *t_a, float *t_b, float *t_c);
float wrap_pm(float x, float y);
float fmodf_pos(float x, float y);
float wrap_pm_pi(float x);

#endif
