
#include "utils.h"

#include "board.h"
#include "tim.h"

#include <float.h>
#include <math.h>
#include <stdint.h>


/****************************************************************************/
// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta.
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2.
// Returns 0 on success and -1 if the input was out of range.
int SVM(float alpha, float beta, float* tA, float* tB, float* tC)
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

float fast_atan2(float y, float x)
{
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    float a = fminf(abs_x, abs_y) / (fmaxf(abs_x, abs_y) + FLT_MIN);
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (abs_y > abs_x) {
        r = 1.57079637f - r;
    }
    if (x < 0.0f) {
        r = 3.14159274f - r;
    }
    if (y < 0.0f) {
        r = -r;
    }

    return r;
}

uint32_t deadline_to_timeout(uint32_t deadline_ms)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t timeout_ms = deadline_ms - now_ms;

    return ((timeout_ms & 0x80000000UL) != 0U) ? 0U : timeout_ms;
}

uint32_t timeout_to_deadline(uint32_t timeout_ms)
{
    return HAL_GetTick() + timeout_ms;
}

int is_in_the_future(uint32_t time_ms)
{
    return deadline_to_timeout(time_ms) != 0U;
}

/****************************************************************************/
uint32_t micros(void)
{
    uint32_t ms_before;
    uint32_t ms_after;
    uint32_t timer_us;

    do {
        ms_before = HAL_GetTick();
        timer_us = TIM_TIME_BASE->CNT;
        ms_after = HAL_GetTick();
    } while (ms_before != ms_after);

    return (ms_before * 1000U) + timer_us;
}

uint32_t elapsed_us(uint32_t start_us)
{
    return micros() - start_us;
}

void delay_us(uint32_t us)
{
    uint32_t start_us = micros();

    while (elapsed_us(start_us) < us) {
        __NOP();
    }
}
