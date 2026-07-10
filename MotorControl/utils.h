
#ifndef __UTILS_LIB_H
#define __UTILS_LIB_H

#include "main.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>


/****************************************************************************/
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif
#define one_by_sqrt3  0.57735026919f
#define two_by_sqrt3  1.15470053838f
#define sqrt3_by_2    0.86602540378f
/****************************************************************************/
int SVM(float alpha, float beta, float* tA, float* tB, float* tC);
float fast_atan2(float y, float x);
uint32_t deadline_to_timeout(uint32_t deadline_ms);
uint32_t timeout_to_deadline(uint32_t timeout_ms);
int is_in_the_future(uint32_t time_ms);
uint32_t micros(void);
uint32_t elapsed_us(uint32_t start_us);
void delay_us(uint32_t us);
/****************************************************************************/
static inline float SQ(float x)
{
	return x * x;
}

static inline float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static inline int round_int(float x)
{
#ifdef __arm__
	int res;
	asm("vcvtr.s32.f32   %[res], %[x]"
		: [res] "=X" (res)
		: [x] "w" (x));
	return res;
#else
	return (int)nearbyintf(x);
#endif
}

static inline int is_nan(float x)
{
	return __builtin_isnan(x);
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
static inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
	float intval = (float)round_int(x / y);
#else
	float intval = nearbyintf(x / y);
#endif
	return x - intval * y;
}

// Same as fmodf but result is positive and y must be positive
static inline float fmodf_pos(float x, float y) {
	float res = wrap_pm(x, y);
	if (res < 0) res += y;
	return res;
}

static inline float wrap_pm_pi(float x) {
	return wrap_pm(x, 2 * M_PI);
}

static inline float horner_poly_eval(float x, const float *coeffs, size_t count)
{
	float result = 0.0f;

	for (size_t idx = 0U; idx < count; ++idx) {
		result = (result * x) + coeffs[idx];
	}

	return result;
}

static inline int mod_int(int dividend, int divisor)
{
	int r = dividend % divisor;

	if (r < 0) {
		r += divisor;
	}

	return r;
}
/****************************************************************************/
typedef struct 
{
	float phA;
	float phB;
	float phC;
} Iph_ABC_t;

typedef struct 
{
	float d;
	float q;
} float2D;
/****************************************************************************/


#endif

