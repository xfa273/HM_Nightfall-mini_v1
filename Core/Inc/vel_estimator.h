#ifndef INC_VEL_ESTIMATOR_H_
#define INC_VEL_ESTIMATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 軽量速度融合（IMU加速度×エンコーダ）
 * - 1kHz周期で呼び出されることを前提
 * - 3ms窓でエンコーダ速度を生成し、その遅れはIMU加速度の積分で補う
 */

void velest_init(float dt_s,
                 uint8_t enc_window_ms,
                 float k_blend,
                 float k_bias,
                 float acc_lpf_alpha);

void velest_reset(void);

/* 1kHz周期で呼び出し（interrupt.c TIM5内など）
 * imu_acc_mms2: IMUの前後方向加速度 [mm/s^2]
 * enc_v_inst_mms: そのmsのエンコーダ瞬時速度（左右平均）[mm/s]
 */
void velest_tick(float imu_acc_mms2, float enc_v_inst_mms);

/* 推定速度（融合結果）[mm/s] */
float velest_get_v(void);

/* 直近の窓平均エンコーダ速度 [mm/s]（更新タイミングのみ変化） */
float velest_get_v_enc(void);

/* LPF済みIMU加速度（バイアス補正後）[mm/s^2] */
float velest_get_a_lpf(void);

/* 推定しているIMU加速度バイアス [mm/s^2] */
float velest_get_a_bias(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_VEL_ESTIMATOR_H_ */
