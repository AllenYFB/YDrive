#include "utils.h"

#include <math.h>

int svm(float alpha, float beta, float *t_a, float *t_b, float *t_c)
{
    int sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sextant = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;
        } else {
            sextant = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
        }
    } else {
        if (alpha >= 0.0f) {
            sextant = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        } else {
            sextant = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
        }
    }

    switch (sextant) {
        case 1: {
            float t1 = alpha - ONE_BY_SQRT3 * beta;
            float t2 = TWO_BY_SQRT3 * beta;
            *t_a = (1.0f - t1 - t2) * 0.5f;
            *t_b = *t_a + t1;
            *t_c = *t_b + t2;
        } break;

        case 2: {
            float t2 = alpha + ONE_BY_SQRT3 * beta;
            float t3 = -alpha + ONE_BY_SQRT3 * beta;
            *t_b = (1.0f - t2 - t3) * 0.5f;
            *t_a = *t_b + t3;
            *t_c = *t_a + t2;
        } break;

        case 3: {
            float t3 = TWO_BY_SQRT3 * beta;
            float t4 = -alpha - ONE_BY_SQRT3 * beta;
            *t_b = (1.0f - t3 - t4) * 0.5f;
            *t_c = *t_b + t3;
            *t_a = *t_c + t4;
        } break;

        case 4: {
            float t4 = -alpha + ONE_BY_SQRT3 * beta;
            float t5 = -TWO_BY_SQRT3 * beta;
            *t_c = (1.0f - t4 - t5) * 0.5f;
            *t_b = *t_c + t5;
            *t_a = *t_b + t4;
        } break;

        case 5: {
            float t5 = -alpha - ONE_BY_SQRT3 * beta;
            float t6 = alpha - ONE_BY_SQRT3 * beta;
            *t_c = (1.0f - t5 - t6) * 0.5f;
            *t_a = *t_c + t5;
            *t_b = *t_a + t6;
        } break;

        default: {
            float t6 = -TWO_BY_SQRT3 * beta;
            float t1 = alpha + ONE_BY_SQRT3 * beta;
            *t_a = (1.0f - t6 - t1) * 0.5f;
            *t_c = *t_a + t1;
            *t_b = *t_c + t6;
        } break;
    }

    return ((*t_a >= 0.0f) && (*t_a <= 1.0f) &&
            (*t_b >= 0.0f) && (*t_b <= 1.0f) &&
            (*t_c >= 0.0f) && (*t_c <= 1.0f)) ? 0 : -1;
}

float wrap_pm(float x, float y)
{
    float intval = nearbyintf(x / y);
    return x - intval * y;
}

float fmodf_pos(float x, float y)
{
    float res = wrap_pm(x, y);
    if (res < 0.0f) {
        res += y;
    }
    return res;
}

float wrap_pm_pi(float x)
{
    return wrap_pm(x, 2.0f * M_PI_F);
}
