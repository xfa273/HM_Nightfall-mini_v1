/*
 * interrupt.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "interrupt.h"
#include "logging.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        // TIM1の割り込み処理

        // printf("TIM1 Interrupt\n");
    }

    if (htim->Instance == htim5.Instance) {
        // TIM5の割り込み処理 1kHz

        // センサ値の取得
        if (ADC_task_counter == 0) {

            // 左右壁センサ値の取得

            // LED OFFのセンサ値
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
            tim1_wait_us(IR_WAIT_US);
            ad_r_off = get_sensor_value_r();
            ad_l_off = get_sensor_value_l();

            // LED ONのセンサ値
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
            tim1_wait_us(IR_WAIT_US);
            ad_r_raw = get_sensor_value_r();
            ad_l_raw = get_sensor_value_l();
            ad_r = max(ad_r_raw - ad_r_off - wall_offset_r, 0);
            ad_l = max(ad_l_raw - ad_l_off - wall_offset_l, 0);
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);

            ADC_task_counter++;

        } else if (ADC_task_counter == 1) {
            
            // 前壁センサ値の取得

            // LED OFFのセンサ値
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
            tim1_wait_us(IR_WAIT_US);
            ad_fr_off = get_sensor_value_fr();
            ad_fl_off = get_sensor_value_fl();

            // LED ONのセンサ値
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
            tim1_wait_us(IR_WAIT_US);
            ad_fr_raw = get_sensor_value_fr();
            ad_fl_raw = get_sensor_value_fl();
            ad_fr = max(ad_fr_raw - ad_fr_off - wall_offset_fr, 0);
            ad_fl = max(ad_fl_raw - ad_fl_off - wall_offset_fl, 0);
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);

            ADC_task_counter++;

        } else {
            // バッテリー電圧値の取得
            ad_bat = get_battery_value();
            ADC_task_counter = 0;
        }

        // エンコーダ値の取得
        read_encoder();

        // IMU値の取得
        read_IMU();

        // バッテリー電圧の監視
        if (ad_bat > 3000) { // 3.3*3060/4095*3=7.4[V]で発動
            // バッテリーOK

        } else {
            // バッテリー消耗
        }

        if (MF.FLAG.OVERRIDE == 0) {

            // 壁制御
            wall_PID();
            diagonal_CTRL();

            // 目標値の積算計算
            calculate_translation();
            calculate_rotation();

            // 並進位置→並進速度のPID
            distance_PID();
            velocity_PID();

            // 角度→角速度のPID
            // angle_PID();
            omega_PID();

            drive_motor();
        }
    }

    if (wall_end_count > 1) {
        wall_end_count--;
    }

    if (buzzer_count > 1) {
        buzzer_count--;
    } else if (buzzer_count) {
        // ブザーを止める
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        buzzer_count = 0;
    }

    // 新しいロギング機能の実装
    if (MF.FLAG.GET_LOG_1) {
        // HAL_GetTickから現在の時間を取得
        uint32_t current_time = HAL_GetTick();
        
        // テスト用に固定値と実際の値を混ぜてログに記録
        static float test_val = 1.23f;
        test_val += 0.01f;
        if (test_val > 10.0f) test_val = 1.0f;
        
        // 必要なデータをログに記録（バッファ制限のチェックはlog_add_entry内で行う）
        log_add_entry(
            (uint16_t)log_buffer.count,     // インデックス
            omega_interrupt,                        // 目標角速度（テスト値）
            real_omega,                      // 実際の角速度
            KP_OMEGA * omega_error,          // P項
            KI_OMEGA * omega_integral,       // I項
            KD_OMEGA * omega_error_error,    // D項
            (float)out_r,                    // 右モーター出力
            (float)out_l,                    // 左モーター出力
            current_time                     // タイムスタンプ
        );
    }


} /* HAL_TIM_PeriodElapsedCallback */

//+++++++++++++++++++++++++++++++++++++++++++++++
// tim1_wait_us
// 1us毎にカウントアップするTIM5を使ってusマイクロ秒処理を止める関数。
// （whileループ中にオーバーフローが起こると機能しないのでTIM5タイマ更新割り込みハンドラ内のみで使用することを推奨する）
// 引数：us …… 処理を止めたいマイクロ秒
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void tim1_wait_us(uint32_t us) {
    // TIM1 を 1us タイマとして使用（TIM5 ISR 内での待ち時間測定に同一タイマを使わない）
    // sensor_init() で HAL_TIM_Base_Start_IT(&htim1) 済みである前提
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); // 例: 1000（0..ARRでカウント）
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim1);

    uint32_t elapsed = 0;
    while (elapsed < us) {
        uint32_t now = __HAL_TIM_GET_COUNTER(&htim1);
        if (now >= start) {
            elapsed = now - start;
        } else {
            // ARR を跨いだ場合の経過時間
            elapsed = (arr + 1u - start) + now;
        }
    }
}
