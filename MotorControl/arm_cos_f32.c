#include "arm_sin_cos_f32.h"

#include <stdint.h>

#define FAST_MATH_TABLE_SIZE 512U
#define INV_TWO_PI 0.159154943092f
#define SIN_TABLE_STEP_SIN 0.012271538286f
#define SIN_TABLE_STEP_COS 0.999924701839f

static float sin_table_f32[FAST_MATH_TABLE_SIZE + 1U];
static uint8_t sin_table_ready;

static void arm_fast_math_init_table(void)
{
    float sin_value = 0.0f;
    float cos_value = 1.0f;

    for (uint32_t i = 0U; i < FAST_MATH_TABLE_SIZE; ++i) {
        float next_sin;
        float next_cos;

        sin_table_f32[i] = sin_value;

        next_sin = sin_value * SIN_TABLE_STEP_COS + cos_value * SIN_TABLE_STEP_SIN;
        next_cos = cos_value * SIN_TABLE_STEP_COS - sin_value * SIN_TABLE_STEP_SIN;
        sin_value = next_sin;
        cos_value = next_cos;
    }

    sin_table_f32[FAST_MATH_TABLE_SIZE] = 0.0f;
    sin_table_ready = 1U;
}

float arm_sin_table_lookup_f32(float normalized_phase)
{
    float fract;
    float findex;
    uint16_t index;
    int32_t whole_cycles;
    float a;
    float b;

    if (sin_table_ready == 0U) {
        arm_fast_math_init_table();
    }

    whole_cycles = (int32_t)normalized_phase;
    if (normalized_phase < 0.0f) {
        whole_cycles--;
    }

    normalized_phase -= (float)whole_cycles;
    findex = (float)FAST_MATH_TABLE_SIZE * normalized_phase;
    index = (uint16_t)findex;

    if (index >= FAST_MATH_TABLE_SIZE) {
        index = 0U;
        findex -= (float)FAST_MATH_TABLE_SIZE;
    }

    fract = findex - (float)index;
    a = sin_table_f32[index];
    b = sin_table_f32[index + 1U];

    return (1.0f - fract) * a + fract * b;
}

float arm_cos_f32(float x)
{
    return arm_sin_table_lookup_f32(x * INV_TWO_PI + 0.25f);
}
