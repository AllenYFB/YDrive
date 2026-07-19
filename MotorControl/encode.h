#ifndef __ENCODE_LIB_H
#define __ENCODE_LIB_H

#include "utils.h"

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************/
#define ENCODER_CPR_DEFAULT 4000
#define ENCODER_CPR_SPI_AS5047P_DEFAULT 16384
#define ENCODER_MODE_DEFAULT ENCODER_MODE_SPI_AS5047P
#define ENCODER_BANDWIDTH_DEFAULT 1000.0f
#define ENCODER_CALIB_RANGE_DEFAULT 0.02f
#define ENCODER_CALIB_SCAN_DISTANCE_DEFAULT (16.0f * M_PI)
#define ENCODER_CALIB_SCAN_OMEGA_DEFAULT (4.0f * M_PI)
#define ENCODER_CALIB_SCAN_VOLTAGE_DEFAULT 0.6f

/****************************************************************************/
typedef enum
{
	ENCODER_MODE_INCREMENTAL = 0,
	ENCODER_MODE_SPI_AS5047P,
} ENCODER_MODE;

typedef struct
{
	/* Config: user-selected encoder model and calibration parameters. */
	ENCODER_MODE mode;                  /* 编码器类型：增量 ABZ 或 AS5047P SPI */
	int32_t cpr;                        /* 每机械圈的编码器计数，ABZ=4000，AS5047P=16384 */
	int32_t direction;                  /* 编码器方向，offset 校准后为 1 或 -1 */
	int32_t phase_offset;               /* 电角度零点对应的整数编码器偏置 */
	float phase_offset_float;           /* 电角度零点的小数编码器偏置，用于更细的相位补偿 */
	float bandwidth;                    /* PLL 跟随带宽，越大跟随越快但越容易带入噪声 */
	float calib_range;                  /* offset 校准允许的相对误差范围 */
	float calib_scan_distance;          /* offset 校准时开环扫描的电角度距离，单位 rad */
	float calib_scan_omega;             /* offset 校准时开环扫描的电角速度，单位 rad/s */
	bool pre_calibrated;                /* 是否已经完成 offset 校准 */
	bool enable_phase_interpolation;    /* 是否用速度在两个 count 之间做插值 */
	float phase_;                       /* 给 FOC 使用的电角度，单位 rad，范围 -pi..pi */
	float phase_vel_;                   /* 给 FOC 使用的电角速度，单位 rad/s */
} ENCODER_CONFIG;

typedef struct
{
	/* Source samples: ABI timer count or absolute SPI position. */
	bool is_ready;                      /* 编码器是否已经校准好，可以输出有效电角度 */
	int32_t shadow_count;               /* 累计位置计数，可正可负，允许超过一圈 */
	int32_t count_in_cpr;               /* 圈内位置计数，范围 0..cpr-1 */
	int32_t pos_abs;                    /* SPI 绝对位置，AS5047P 范围 0..16383 */
	uint16_t abs_spi_raw;               /* SPI 原始 16bit 数据，用于调试 raw=xxxx */
	uint32_t abs_spi_error_flags;       /* SPI 错误标志，用于调试 spi_flags */
	int16_t tim_cnt_sample;             /* ABZ 模式下 TIM3 的采样值 */
	bool abs_spi_pos_updated;           /* 本周期是否成功读到新的 SPI 绝对位置 */
	float spi_error_rate;               /* SPI 读取失败率的一阶滤波值 */

	/* PLL/interpolation estimates. */
	float interpolation;                /* count 内部插值，0.0 表示刚进格，1.0 表示快到下一格 */
	float pos_estimate_counts;          /* PLL 估算的累计位置，单位 count */
	float pos_cpr_counts;               /* PLL 估算的圈内位置，单位 count */
	float vel_estimate_counts;          /* PLL 估算速度，单位 count/s */
	float pll_kp;                       /* PLL 位置修正增益 */
	float pll_ki;                       /* PLL 速度修正增益 */
	float pos_estimate;                 /* 机械位置估算，单位 turn */
	float vel_estimate;                 /* 机械速度估算，单位 turn/s */

	/* Offset calibration diagnostics. */
	float calib_scan_response;          /* offset 校准实际扫到的编码器 count 数 */
	float calib_scan_expected;          /* offset 校准理论应该扫到的编码器 count 数 */
	float calib_scan_error;             /* offset 校准相对误差 */
	int32_t calib_init_count;           /* offset 校准开始时的 shadow_count */
	int32_t calib_final_count;          /* offset 校准结束正向扫描时的 shadow_count */
	bool pos_estimate_valid;            /* PLL 位置估算是否有效 */
	bool vel_estimate_valid;            /* PLL 速度估算是否有效 */
} ENCODER_STATE;

extern ENCODER_CONFIG encoder_config;
extern ENCODER_STATE encoder_state;
/****************************************************************************/
void encoder_init(void);
void encoder_sample_now(void);
bool encoder_update(void);
bool encoder_run_offset_calibration(void);
/****************************************************************************/

#endif
