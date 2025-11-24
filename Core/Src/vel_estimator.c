#include "vel_estimator.h"
#include "params.h"
#include <math.h>

static float s_dt = 0.001f;             // 1ms 既定
static uint8_t s_win_ms = 3;            // エンコーダ窓 [ms]
static float s_k = 0.30f;               // 速度補正ブレンド係数
static float s_k_bias = 0.001f;         // バイアス学習係数
static float s_alpha = 0.2f;            // IMU加速度LPF係数（1ms）

static float s_v_est = 0.0f;            // 融合推定速度 [mm/s]
static float s_a_bias = 0.0f;           // IMU加速度バイアス [mm/s^2]
static float s_a_lpf = 0.0f;            // LPF済み加速度（バイアス補正後）[mm/s^2]
static float s_v_enc_win = 0.0f;        // 直近の窓平均エンコーダ速度 [mm/s]

static float s_dist_accum_mm = 0.0f;    // 窓内の累積距離 [mm]
static uint8_t s_ms_accum = 0;          // 窓内のms累計
static uint16_t s_stop_ticks = 0;       // 近傍停止の連続検出カウンタ [ms]

void velest_init(float dt_s,
                 uint8_t enc_window_ms,
                 float k_blend,
                 float k_bias,
                 float acc_lpf_alpha)
{
    if (dt_s > 0.0f) s_dt = dt_s; else s_dt = 0.001f;
    if (enc_window_ms == 0) enc_window_ms = 3;
    s_win_ms = enc_window_ms;
    s_k = k_blend;
    s_k_bias = k_bias;
    s_alpha = acc_lpf_alpha;

    velest_reset();
}

void velest_reset(void)
{
    s_v_est = 0.0f;
    s_a_bias = 0.0f;
    s_a_lpf = 0.0f;
    s_v_enc_win = 0.0f;
    s_dist_accum_mm = 0.0f;
    s_ms_accum = 0;
}

void velest_tick(float imu_acc_mms2, float enc_v_inst_mms)
{
    // 近傍停止判定（エンコーダの瞬時速度が十分小さい）
    #ifdef VELEST_NEARSTOP_V_THR
    const float stop_thr = VELEST_NEARSTOP_V_THR * 0.5f; // 閾値の50%で厳しめ判定
    if (fabsf(enc_v_inst_mms) < stop_thr) {
        if (s_stop_ticks < 1000) s_stop_ticks++; // 最大1秒まで
    } else {
        s_stop_ticks = 0;
    }
    #else
    s_stop_ticks = 0;
    #endif
    // 1) 窓内にエンコーダ距離を加算
    s_dist_accum_mm += enc_v_inst_mms * s_dt; // [mm]
    if (s_ms_accum < 250) { // 過大な蓄積を避ける保険
        s_ms_accum++;
    }

    // 2) IMU加速度のLPF（バイアス補正後）
    float a_corr = imu_acc_mms2 - s_a_bias;
    s_a_lpf = s_a_lpf + s_alpha * (a_corr - s_a_lpf);
    // 異常値ガード（過大な加速度に対するクランプ）
    #ifdef VELEST_A_LPF_CLAMP
    if (s_a_lpf > VELEST_A_LPF_CLAMP)  s_a_lpf = VELEST_A_LPF_CLAMP;
    if (s_a_lpf < -VELEST_A_LPF_CLAMP) s_a_lpf = -VELEST_A_LPF_CLAMP;
    #endif

    // 3) 予測（IMU積分）: 停止近傍では積分を抑制
    float v_pred = s_v_est;
    if (s_stop_ticks < 8) { // 約8ms未満なら通常積分、静止が続くときは抑制
        v_pred += s_a_lpf * s_dt;
    }

    // 4) 窓更新タイミングで補正
    if (s_ms_accum >= s_win_ms) {
        const float T = s_dt * (float)s_ms_accum; // 窓長 [s]
        if (T > 0.0f) {
            s_v_enc_win = s_dist_accum_mm / T;     // [mm/s]
        }

        // 安全のため推定速度に上限を設ける（暴走ガード）
        #ifdef VELEST_V_CLAMP
        if (s_v_est > VELEST_V_CLAMP)  s_v_est = VELEST_V_CLAMP;
        if (s_v_est < -VELEST_V_CLAMP) s_v_est = -VELEST_V_CLAMP;
        #endif

        // 近傍停止のスナップ（微小ノイズで揺れないように）
        #ifdef VELEST_NEARSTOP_V_THR
        if (fabsf(s_v_enc_win) < VELEST_NEARSTOP_V_THR) {
            s_v_enc_win = 0.0f;
        }
        #endif

        float e = s_v_enc_win - v_pred;            // 速度誤差
        s_v_est = v_pred + s_k * e;                // 速度補正

        // IMU加速度バイアスの学習（遅く・安定に）
        if (T > 0.0f) {
            // e/T は加速度相当。過大にならないようクランプしてから適用
            float a_err = e / T;
            #ifdef VELEST_A_ERR_CLAMP
            if (a_err > VELEST_A_ERR_CLAMP)  a_err = VELEST_A_ERR_CLAMP;
            if (a_err < -VELEST_A_ERR_CLAMP) a_err = -VELEST_A_ERR_CLAMP;
            #endif
            s_a_bias -= s_k_bias * a_err;
            #ifdef VELEST_A_BIAS_CLAMP
            if (s_a_bias > VELEST_A_BIAS_CLAMP)  s_a_bias = VELEST_A_BIAS_CLAMP;
            if (s_a_bias < -VELEST_A_BIAS_CLAMP) s_a_bias = -VELEST_A_BIAS_CLAMP;
            #endif
        }

        // 停止が継続している場合は、推定速度を強制減衰し、バイアスを速めにゼロ化
        if (s_stop_ticks >= 8) { // ~8ms以上停止継続
            s_v_est *= 0.90f;                // 速度は素早く0へ
            s_a_bias += 0.05f * s_a_lpf;     // a_lpfを打ち消す方向にバイアスを寄せる
            s_a_lpf *= 0.85f;                // LPF状態も減衰
        }

        // 窓をリセット
        s_dist_accum_mm = 0.0f;
        s_ms_accum = 0;
    } else {
        // 予測のみ
        s_v_est = v_pred;
    }
}

float velest_get_v(void)      { return s_v_est; }
float velest_get_v_enc(void)  { return s_v_enc_win; }
float velest_get_a_lpf(void)  { return s_a_lpf; }
float velest_get_a_bias(void) { return s_a_bias; }
