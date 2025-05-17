/*
 * interrupt.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        // TIM1の割り込み処理

        // printf("TIM1 Interrupt\n");
    }

    if (htim->Instance == htim5.Instance) {
        // TIM5の割り込み処理 1kHz

        if (HAL_GPIO_ReadPin(SW_POWER_GPIO_Port, SW_POWER_Pin) == 0) {
            while (1)
                ;
        }

        // センサ値の取得
        if (ADC_task_counter == 0) {
            // 左右壁センサ値の取得
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
            tim1_wait_us(IR_WAIT_US);
            ad_r = get_sensor_value_r();
            ad_l = get_sensor_value_l();
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);

            ADC_task_counter++;

        } else if (ADC_task_counter == 1) {
            // 前壁センサ値の取得
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
            tim1_wait_us(IR_WAIT_US);
            ad_fr = get_sensor_value_fr();
            ad_fl = get_sensor_value_fl();
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);

            ADC_task_counter++;

        } else {
            // バッテリー電圧値の取得
            ad_bat = get_battery_value();
            ADC_task_counter = 0;
        }

        indicate_sensor();

        // 壁切れ
        wall_end();

        // エンコーダ値の取得
        read_encoder();

        // IMU値の取得
        read_IMU();

        // バッテリー電圧の監視
        if (ad_bat > 3000) { // 3.3*3060/4095*3=7.4[V]で発動
            // バッテリーOK
            //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
            //HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

        } else {
            // バッテリー消耗
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
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

    if (MF.FLAG.GET_LOG_1) {
        if (log_cnt < 1000) {

            log_1[log_cnt] = log_cnt;
            log_2[log_cnt] = omega_interrupt;
            log_3[log_cnt] = real_omega;
            log_4[log_cnt] = KP_OMEGA * omega_error;
            log_5[log_cnt] = KI_OMEGA * omega_integral;
            log_6[log_cnt] = KD_OMEGA * omega_error_error;
            // log_7[log_cnt] = alpha_interrupt * FF_OMEGA;
            log_7[log_cnt] = out_r;
            log_8[log_cnt] = out_l;

            /*
            log_1[log_cnt] = log_cnt;
            log_2[log_cnt] = target_distance;
            log_3[log_cnt] = real_distance;
            log_4[log_cnt] = KP_DISTANCE * distance_error;
            log_5[log_cnt] = KI_DISTANCE * distance_integral;
            log_6[log_cnt] = KD_DISTANCE * distance_error_error;
            log_7[log_cnt] = 0;
            log_8[log_cnt] = 0;
            */

            /*
            log_1[log_cnt] = log_cnt;
            log_2[log_cnt] = target_velocity;
            log_3[log_cnt] = real_velocity;
            log_4[log_cnt] = KP_VELOCITY * velocity_error;
            log_5[log_cnt] = KI_VEROCITY * velocity_integral;
            log_6[log_cnt] = KD_VEROCITY * velocity_error_error;
            log_7[log_cnt] = 0;
            log_8[log_cnt] = 0;
            */

            /*
            log_1[log_cnt] = log_cnt;
            log_2[log_cnt] = target_angle;
            log_3[log_cnt] = -real_angle;
            log_4[log_cnt] = KP_ANGLE * angle_error;
            log_5[log_cnt] = KI_ANGLE * angle_integral;
            log_6[log_cnt] = KD_ANGLE * angle_error_error;
            log_7[log_cnt] = 0;
            log_8[log_cnt] = 0;
            */

            log_cnt++;
        }
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
    uint32_t dest = __HAL_TIM_GET_COUNTER(&htim1) + us;
    while (__HAL_TIM_GET_COUNTER(&htim1) < dest)
        ;
}
