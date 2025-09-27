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
    // 設定された加速度から並進速度を計算
    velocity_interrupt += acceleration_interrupt * 0.001;

    // 並進速度から目標位置を計算
    target_distance += velocity_interrupt * 0.001;
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

    // P項
    velocity_error = target_velocity;

    if (velocity_error > 10000 || velocity_error < -10000) {
        MF.FLAG.FAILED = 1;
    }

    // I項
    velocity_integral += velocity_error;

    // D項
    velocity_error_error = velocity_error - previous_velocity_error;

    // モータ制御量を計算
    out_translation = KP_VELOCITY * velocity_error +
                      KI_VELOCITY * velocity_integral +
                      KD_VELOCITY * velocity_error_error;

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

    // 目標並進速度を計算
    target_velocity = KP_DISTANCE * distance_error +
                      KI_DISTANCE * distance_integral +
                      KD_DISTANCE * distance_error_error;

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
        } else if (ad_r < wall_thr_r && ad_l < wall_thr_l) {
            // 左右壁が両方ない場合
            wall_error = 0;
        } else if (ad_r > wall_thr_r && ad_l < wall_thr_l) {
            // 右壁のみある場合
            wall_error = -2 * (ad_r - base_r);
        } else if (ad_r < wall_thr_r && ad_l > wall_thr_l) {
            // 左壁のみある場合
            wall_error = 2 * (ad_l - base_l);
        }
        // 探索側で参照できるように最新の壁誤差を公開
        latest_wall_error = wall_error;

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
