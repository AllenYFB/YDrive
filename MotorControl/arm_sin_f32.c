#include "arm_sin_cos_f32.h"

#define INV_TWO_PI 0.159154943092f

float arm_sin_table_lookup_f32(float normalized_phase);

float arm_sin_f32(float x)
{
    return arm_sin_table_lookup_f32(x * INV_TWO_PI);
}
