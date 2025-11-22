/*
 * control.c
 *
 *  Created on: 2023/07/29
 *      Author: yuho-
 */

#include "global.h"
#include "sensor_distance.h"
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
    // 速度フィードバック: 目標速度（distance_PIDで算出） - 実測速度
    // target_velocity は distance_PID() 内で更新される
    velocity_error = target_velocity - real_velocity;

    if (velocity_error > 10000 || velocity_error < -10000) {
        MF.FLAG.FAILED = 1;
    }

    // I項（単純積分）
    velocity_integral += velocity_error;

    // D項
    velocity_error_error = velocity_error - previous_velocity_error;

    // 吸引ON/OFFでゲイン切替
    const float kp_v = MF.FLAG.SUCTION ? KP_VELOCITY_FAN_ON : KP_VELOCITY_FAN_OFF;
    const float ki_v = MF.FLAG.SUCTION ? KI_VELOCITY_FAN_ON : KI_VELOCITY_FAN_OFF;
    const float kd_v = MF.FLAG.SUCTION ? KD_VELOCITY_FAN_ON : KD_VELOCITY_FAN_OFF;

    // モータ制御量を計算（純PID）
    out_translation =   kp_v * velocity_error
                    + ki_v * velocity_integral
                    + kd_v * velocity_error_error;

    // 並進速度の偏差を保存
    previous_velocity_error = velocity_error;
}

/*並進距離のPID制御*/
void distance_PID(void) {
    // 位置PID（距離誤差から目標速度を生成）
    // 誤差
    distance_error = target_distance - real_distance;
    // I項
    distance_integral += distance_error;
    // D項
    distance_error_error = distance_error - previous_distance_error;

    // 目標速度（プロファイル速度 + 位置PIDの補正）
    const float kp_d = MF.FLAG.SUCTION ? KP_DISTANCE_FAN_ON : KP_DISTANCE_FAN_OFF;
    const float ki_d = MF.FLAG.SUCTION ? KI_DISTANCE_FAN_ON : KI_DISTANCE_FAN_OFF;
    const float kd_d = MF.FLAG.SUCTION ? KD_DISTANCE_FAN_ON : KD_DISTANCE_FAN_OFF;

    float v_fb = (kp_d * distance_error)
               + (ki_d * distance_integral)
               + (kd_d * distance_error_error);
    target_velocity = velocity_interrupt + v_fb;

    // 誤差履歴更新
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

        float wall_error = 0; // ADベース（最新の挙動判定用に保持）
        float wall_error_mm = 0.0f; // 距離ベースの誤差[mm]

        // 目標距離は左右とも固定値（params.h）を使用
        const float d_l_target = WALL_TARGET_DIST_L_MM;
        const float d_r_target = WALL_TARGET_DIST_R_MM;
        // 現在距離をAD→距離に換算
        const float d_l_now = sensor_distance_from_l(ad_l);
        const float d_r_now = sensor_distance_from_r(ad_r);
        // 距離ベースの壁有無判定
        const bool r_has = (d_r_now <= WALL_DETECT_DIST_R_MM);
        const bool l_has = (d_l_now <= WALL_DETECT_DIST_L_MM);

        if (r_has && l_has) {
            // 左右壁が両方ある場合
            wall_error = (ad_l - base_l) - (ad_r - base_r);
            latest_wall_error = wall_error;
            wall_error_mm = (d_l_now - d_l_target) - (d_r_now - d_r_target);
        } else if (!r_has && !l_has) {
            // 左右壁が両方ない場合
            wall_error = 0;
            latest_wall_error = wall_error;
            wall_error_mm = 0.0f;
        } else if (r_has && !l_has) {
            // 右壁のみある場合
            wall_error = -2 * (ad_r - base_r);
            latest_wall_error = wall_error*0.5;
            wall_error_mm = -2.0f * (d_r_now - d_r_target);
        } else if (!r_has && l_has) {
            // 左壁のみある場合
            wall_error = 2 * (ad_l - base_l);
            latest_wall_error = wall_error*0.5;
            wall_error_mm = 2.0f * (d_l_now - d_l_target);
        }

        // 横壁制御は距離[mm]ベースの誤差で実行（符号を系に合わせて反転）
        // 正のkp_wallで「左が近い→右旋回」「右が近い→左旋回」になるようにする
        wall_control = - (wall_error_mm * kp_wall);

        if(fabsf(out_l)<50 && fabsf(out_r)<50){
            wall_control = 0;
        }

        // Clamp wall_control magnitude to +/-WALL_CTRL_MAX
        {
            const float lim = WALL_CTRL_MAX;
            if (wall_control > lim) {
                wall_control = lim;
            } else if (wall_control < -lim) {
                wall_control = -lim;
            }
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
