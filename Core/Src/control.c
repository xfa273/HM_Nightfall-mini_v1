/*
 * control.c
 *
 *  Created on: 2023/07/29
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>

/*エンコーダから速度と位置を取得する*/
void read_encoder(void) {
    // エンコーダのパルスカウンタを取得
    encoder_count_r = TIM8->CNT;
    encoder_count_l = TIM4->CNT;

    // パルスカウンタの増加量を速度として取得
    encoder_speed_r = encoder_count_r - 30000;
    encoder_speed_l = encoder_count_l - 30000;

    // マイナス速度用にパルスカウントを30000にセット
    TIM8->CNT = 30000;
    TIM4->CNT = 30000;

    encoder_speed_r = -encoder_speed_r; // 右の速度の符号を補正

    // 速度の換算 →[mm/s]
    // 注意: 新機体はエンコーダがホイール側搭載のため減速比は不要
    // Cpr_wheel = 400[PPR] × 4[逓倍] = 1600[count/rev]
    // 係数K = 1000[ms→s] × (1/Cpr_wheel) = 1000*(1/(400*4)) = 0.625
    encoder_speed_r = encoder_speed_r * 0.625 * D_TIRE * 3.1415;
    encoder_speed_l = encoder_speed_l * 0.625 * D_TIRE * 3.1415;

    // 回転方向を補正
    encoder_speed_r = DIR_ENC_R * encoder_speed_r;
    encoder_speed_l = DIR_ENC_L * encoder_speed_l;

    // 走行距離カウンタを加算
    encoder_distance_r += encoder_speed_r * 0.001;
    encoder_distance_l += encoder_speed_l * 0.001;

    // 並進のPID制御用に格納
    real_velocity = (encoder_speed_r + encoder_speed_l) * 0.5;
    real_distance = (encoder_distance_r + encoder_distance_l) * 0.5;

    // パルスカウントを保存
    previous_encoder_count_r = encoder_count_r;
    previous_encoder_count_l = encoder_count_l;
}

/*IMUから角速度と角度を取得する*/
void read_IMU(void) {
    // 時計回りが正
    IMU_DataUpdate();
    real_omega = -omega_z_true * KP_IMU;
    IMU_angle += omega_z_true * 0.001;
    real_angle = IMU_angle;
    IMU_acceleration = accel_y_true * 1000;
}

/*並進の積算計算*/
void calculate_translation(void) {
    const float dt = 0.001f; // 1kHz

    if (profile_active) {
        // sベース台形プロファイル
        float d_rem = profile_s_end - real_distance; // 実距離に対する残距離
        if (d_rem < 0.0f) d_rem = 0.0f;

        // 減速側から導出される上限速度（この距離で v_out まで確実に減速できる上限）
        float v_cap = 0.0f;
        float term = profile_v_out * profile_v_out + 2.0f * profile_a_decel * d_rem;
        if (term > 0.0f) v_cap = sqrtf(term);

        // 上限（最大速度・減速上限）
        float v_lim = profile_v_max;
        if (v_cap < v_lim) v_lim = v_cap;
        if (v_lim < 0.0f) v_lim = 0.0f;

        // 目標速度の次ステップ
        float v = velocity_interrupt;
        float v_next;
        if (v < v_lim) {
            v_next = v + profile_a_accel * dt;
            if (v_next > v_lim) v_next = v_lim;
        } else {
            v_next = v - profile_a_decel * dt;
            if (v_next < v_lim) v_next = v_lim;
        }

        // 加速度（interrupt側）を逆算して一貫性を保つ
        acceleration_interrupt = (v_next - v) / dt;
        velocity_interrupt = v_next;

        // 目標距離の積分（ロギング・互換用途）
        target_distance += velocity_interrupt * dt;
    } else {
        // 従来の積分（プロファイル未使用時）
        velocity_interrupt += acceleration_interrupt * dt;
        target_distance += velocity_interrupt * dt;
    }
}

/*回転の積算計算*/
void calculate_rotation(void) {
    // 設定された角加速度から角速度を計算
    omega_interrupt += alpha_interrupt * 0.001;

    /*
    omega_interrupt += wall_control * 0.01;
    if (wall_control != 0) {
        omega_integral = 0;
    }
    */

    // 角速度から角度を計算
    target_angle += omega_interrupt * 0.001;
}

/*並進速度のPID制御*/
void velocity_PID(void) {

    // 速度フィードバック: 目標速度(velocity_interrupt) - 実測速度(real_velocity)
    // 距離PIDによるカスケードは廃止し、単純な速度FBのみで並進を制御する
    target_velocity = velocity_interrupt;
    velocity_error = target_velocity - real_velocity;

    if (velocity_error > 10000 || velocity_error < -10000) {
        MF.FLAG.FAILED = 1;
    }

    // I項（先に積分→クランプ）
    velocity_integral += velocity_error;
    // 速度I項クランプ（KI=0回避のため微小値で割る）
    {
        const float ki = (KI_VELOCITY != 0.0f) ? KI_VELOCITY : 1e-6f;
        const float limit_int = VEL_I_LIMIT / ki; // 積分の生値に対する上限
        if (velocity_integral >  limit_int) velocity_integral =  limit_int;
        if (velocity_integral < -limit_int) velocity_integral = -limit_int;
    }

    // D項
    velocity_error_error = velocity_error - previous_velocity_error;

    // フィードフォワード（粘性・加速度・クーロン）
    float ff = KFF_VELOCITY * target_velocity
             + KFF_ACCEL    * acceleration_interrupt
             + KFF_COULOMB  * ((target_velocity >= 0.0f) ? 1.0f : -1.0f);

    // モータ制御量を計算（FF + PID）
    out_translation = ff
                    + KP_VELOCITY * velocity_error
                    + KI_VELOCITY * velocity_integral
                    + KD_VELOCITY * velocity_error_error;

    // 並進速度の偏差を保存
    previous_velocity_error = velocity_error;
}

/*並進距離のPID制御*/
void distance_PID(void) {

    // P項
    distance_error = target_distance - real_distance;

    // I項
    distance_integral += distance_error;

    // D項
    distance_error_error = distance_error - previous_distance_error;

    // 位置→速度のカスケードは廃止。目標速度の更新は行わない（速度FB単独）。
    // target_velocity は velocity_PID 内で velocity_interrupt を反映する。

    // 並進位置の偏差を保存
    previous_distance_error = distance_error;
}

/*角速度のPID制御*/
void omega_PID(void) {

    // P項
    // omega_error = target_omega - real_omega;

    omega_error =
        omega_interrupt - real_omega + wall_control + diagonal_control;

    // I項
    omega_integral += omega_error;

    // D項
    omega_error_error = angle_error - previous_omega_error;

    // モータ制御量を計算

    out_rotate = KP_OMEGA * omega_error + KI_OMEGA * omega_integral +
                 KD_OMEGA * omega_error_error;
    // out_rotate = omega_interrupt * 0.5 + KP_OMEGA * omega_error +
    // KI_OMEGA *omega_integral + KD_OMEGA *omega_error_error;

    // 角速度の偏差を保存
    previous_omega_error = omega_error;
}

/*角度のPID制御*/
void angle_PID(void) {

    // P項
    angle_error = target_angle - real_angle;

    // I項
    angle_integral += angle_error;

    // D項
    angle_error_error = angle_error - previous_angle_error;

    // 目標角速度を計算
    target_omega = KP_ANGLE * angle_error + KI_ANGLE * angle_integral +
                   KD_ANGLE * angle_error_error;

    // 角度の偏差を保存
    previous_angle_error = angle_error;
}

/*壁のPID制御*/
void wall_PID(void) {

    // 制御フラグがあれば制御
    if (MF.FLAG.CTRL) {

        float wall_error = 0;
        uint16_t wall_thr_r;
        uint16_t wall_thr_l;
        float sense_diff_r;
        float sense_diff_l;

        sense_diff_r = ad_r - previous_ad_r;
        sense_diff_l = ad_l - previous_ad_l;

        if (fabsf(sense_diff_r) > WALL_DIFF_THR) {
            wall_thr_r = WALL_BASE_R + 30; //30
        } else {
            wall_thr_r = WALL_BASE_R;
        }
        if (fabsf(sense_diff_l) > WALL_DIFF_THR) {
            wall_thr_l = WALL_BASE_L + 30;
        } else {
            wall_thr_l = WALL_BASE_L;
        }

        if (ad_r > wall_thr_r && ad_l > wall_thr_l) {
            // 左右壁が両方ある場合
            wall_error = (ad_l - base_l) - (ad_r - base_r);
            latest_wall_error = wall_error;
        } else if (ad_r < wall_thr_r && ad_l < wall_thr_l) {
            // 左右壁が両方ない場合
            wall_error = 0;
            latest_wall_error = wall_error;
        } else if (ad_r > wall_thr_r && ad_l < wall_thr_l) {
            // 右壁のみある場合
            wall_error = -2 * (ad_r - base_r);
            latest_wall_error = wall_error*0.5;
        } else if (ad_r < wall_thr_r && ad_l > wall_thr_l) {
            // 左壁のみある場合
            wall_error = 2 * (ad_l - base_l);
            latest_wall_error = wall_error*0.5;
        }

        wall_control = wall_error * kp_wall;

        if(fabsf(out_l)<50 && fabsf(out_r)<50){
            wall_control = 0;
        }

        if (wall_control > 0) {
            wall_control = max(wall_control, WALL_CTRL_MAX);
        } else {
            wall_control = min(wall_control, -WALL_CTRL_MAX);
        }

        previous_ad_r = ad_r;
        previous_ad_l = ad_l;

    } else {
        // 制御フラグがなければ制御値0
        wall_control = 0;
    }
}

/*斜めの制御*/
void diagonal_CTRL(void) {

    // 制御フラグがあれば制御
    if (MF.FLAG.CTRL_DIAGONAL) {

        if (ad_fr > diagonal_control_thr && ad_fl > diagonal_control_thr) {
            // 左右センサが閾値以上なら，差分で制御
            if (ad_fr > ad_fl) {
                diagonal_control = -kp_diagonal * (ad_fr - ad_fl);
            } else {
                diagonal_control = kp_diagonal * (ad_fl - ad_fr);
            }

        } else if (ad_fr > diagonal_control_thr) {
            // 右センサのみHigh
            diagonal_control = -kp_diagonal * ad_fr;
        } else if (ad_fl > diagonal_control_thr) {
            // 左センサのみHigh
            diagonal_control = kp_diagonal * ad_fl;
        }
    }
}
