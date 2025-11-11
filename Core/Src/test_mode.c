/*
 * test_mode.c
 *
 *  Created on: Dec 23, 2023
 *      Author: yuho-
 */

#include "global.h"
#include "solver.h"
#include "logging.h"
#include "main.h"

// drive.c と同じ条件でPWM反転するための定義（DIR==Lowで反転が既定）
#ifndef PWM_INVERT_DIR_LEVEL
#define PWM_INVERT_DIR_LEVEL 0
#endif

void test_mode() {

    int mode = 0;

    // テストモードのフラグを有効化（壁切れ補正など距離が変化する補正を無効化するためのゲートに使用）
    MF.FLAG.TEST_MODE = 1;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:

            printf("Test Mode 0.\n");

            break;

        case 1:

            printf("Test Mode 1 IMU Check.\n");

            drive_variable_reset();
            IMU_GetOffset();

            drive_fan(250);

            while (1) {

                printf("omega: %.3f [deg/s] ,  ", real_omega);

                printf("angle: %.3f [deg] ,  ", real_angle);

                printf("accl_x: %.3f ,  ", accel_x_true);

                printf("accl_y: %.3f\n", accel_y_true);

                HAL_Delay(100);
            }

            break;

        case 2:
            printf("Test Mode 2 Encoder Check.\n");

            drive_variable_reset();

            while (1) {

                printf("speed L R: %.3f %.3f [mm/s] ,  ", encoder_speed_l,
                       encoder_speed_r);
                printf("distance: %.3f [mm]\n",
                       (encoder_distance_l + encoder_distance_r) * 0.5);

                HAL_Delay(100);
            }

            break;
        case 3:
            printf("Test Mode 3 Sensor AD Value Check.\n");

            while (1) {
                printf("R: %d, L: %d, FR: %d, FL: %d, BAT: %d\n", ad_r, ad_l,
                       ad_fr, ad_fl, ad_bat);

                HAL_Delay(300);
            }
            break;

        case 4:
            printf("Test Mode 4 Enkai-Gei.\n");

            led_flash(10);

            IMU_GetOffset();
            drive_enable_motor();

            drive_variable_reset();

            // 回転角度カウントをリセット
            real_angle = 0;
            IMU_angle = 0;
            target_angle = 0;

            // 走行距離カウントをリセット
            real_distance = 0;
            encoder_distance_r = 0;
            encoder_distance_l = 0;

            led_write(1, 1);

            drive_start();

            drive_fan(1000);

            break;
        case 5:
            printf("Test Mode 5: Motor Sequence (DIR x DUTY, BOTH wheels, 2s each).\n");

            // 割り込み内の速度制御・drive_motor()呼び出しを無視
            MF.FLAG.OVERRIDE = 1;

            // モータドライバを有効化（STBYを上げ、PWMを開始）
            drive_enable_motor();

            // シーケンス条件（両輪のみ）
            const uint8_t dirs[2] = {FORWARD, BACK};
            const uint8_t duty_pct_list[3] = {10, 30, 60};

            while (1) {
                for (uint8_t di = 0; di < 2; di++) {
                    uint8_t dir_code = dirs[di];

                    // 安全のため一旦アイドルへ（IN1==IN2）
                    uint32_t arr_idle = __HAL_TIM_GET_AUTORELOAD(&htim2);
                    GPIO_PinState cur_dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
                    GPIO_PinState cur_dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (cur_dir_l == GPIO_PIN_SET) ? arr_idle : 0u);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (cur_dir_r == GPIO_PIN_SET) ? arr_idle : 0u);
                    HAL_Delay(20);

                    // 進行方向を設定
                    drive_set_dir(dir_code);
                    HAL_Delay(20);

                    for (uint8_t pi = 0; pi < 3; pi++) {
                        uint8_t duty_pct = duty_pct_list[pi];

                            const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
                            uint32_t duty_counts = (arr * (uint32_t)duty_pct) / 100u;
                            if (duty_counts == 0) duty_counts = 1;
                            if (duty_counts >= arr) duty_counts = arr - 1u;

                            // DIR実レベル読み取り（アイドル設定に使用）
                            GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
                            GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
                            uint8_t dir_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
                            uint8_t dir_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

                            // 実機挙動: DIRピンがHighのときは実効Dutyが「逆数」になる
                            // → DIR==High の側は CCR = ARR - duty_counts（Low期間がduty%）
                            // → DIR==Low  の側は CCR = duty_counts
                            uint32_t ccr_l = 0, ccr_r = 0;
                            ccr_l = dir_high_l ? (arr - duty_counts) : duty_counts;
                            ccr_r = dir_high_r ? (arr - duty_counts) : duty_counts;

                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_l);
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_r);

                            // 進行状況を出力
                            const char *dir_str = (dir_code == FORWARD) ? "FWD" : "BACK";
                            const char *side_str = "BOTH";
                            printf("[DIR=%s][SIDE=%s][DUTY=%u%%] CCR_L=%lu, CCR_R=%lu (ARR=%lu)\n",
                                   dir_str, side_str, (unsigned)duty_pct,
                                   (unsigned long)ccr_l, (unsigned long)ccr_r, (unsigned long)arr);

                            // 2秒維持
                            HAL_Delay(2000);
                    }
                }
            }

            // breakには到達しない
            break;
        case 6:

            printf("Test Mode 6 Circuit.\n");

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 10000; // 5000
            velocity_straight = 3000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();
            led_flash(5);
            get_base();
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(10, velocity_straight, 0);
            run_straight(8, velocity_straight, 0);
            run_straight(10, velocity_l_turn_90, 0);

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90();

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90();

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90();

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90();

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90();

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90();

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90();

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 8回目のターン
            l_turn_R90();

            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 7:

            printf("Test Mode 7 Translation FF Identification.\n");
            printf("Select sub-mode with buttons: 0:200, 1:400, 2:600, 3:800, 4:1000, 9:ALL\n");

            // サブモード選択（0..4=単独速度, 9=ALL）
            int sub = 0;
            sub = select_mode(sub);

            // 安全初期化
            velocity_interrupt = 0;
            led_flash(10);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();
            led_flash(5);

            // 壁制御の影響を排除（ゲイン0）
            kp_wall = 0.0f;
            MF.FLAG.CTRL = 0;
            MF.FLAG.CTRL_DIAGONAL = 0;

            // 走行パラメータ（控えめの加速度）
            acceleration_straight = 15000.0f;
            acceleration_straight_dash = 15000.0f;
            velocity_straight = 3000.0f; // 本テストの上限速度

            // テストする等速速度[mm/s]
            const float speeds_all[] = { 1000.0f, 1500.0f, 2000.0f, 2500.0f, 3000.0f };
            const int NS_ALL = (int)(sizeof(speeds_all)/sizeof(speeds_all[0]));

            // 実行対象配列を決定
            float speeds[5];
            int NS = 0;
            if (sub >= 0 && sub <= 4) {
                speeds[0] = speeds_all[sub];
                NS = 1;
                printf("[ID1] Single speed mode: v=%.0f mm/s. Ensure ~0.6m straight space.\n", speeds[0]);
            } else {
                for (int i=0;i<NS_ALL;i++) speeds[i]=speeds_all[i];
                NS = NS_ALL;
                printf("[ID1] ALL speeds mode (long track).\n");
            }

            const float HALF_MM_CONST = (float)DIST_HALF_SEC; // 半区画長[mm]

            for (int i = 0; i < NS; ++i) {
                float v = speeds[i];
                if (v > velocity_straight) v = velocity_straight;

                printf("[ID1] step %d/%d: v=%.0f mm/s\n", i+1, NS, v);

                drive_fan(1000);

                // ロギング開始（速度プロファイルを使用）
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());

                // まず1区画分（2.0 blocks）で目標速度 v まで加速（上限もvに固定）
                run_straight_const(2.0f, v);

                // 等速保持時間 ≈ 0.4s 分を blocks に換算
                const float hold_time_s = 0.1f;
                float hold_blocks = (v * hold_time_s) / HALF_MM_CONST;
                // 最小でも2mm相当は確保
                const float min_blocks = (2.0f / HALF_MM_CONST);
                if (hold_blocks < min_blocks) hold_blocks = min_blocks;

                // 等速保持（上限もvに固定）
                run_straight_const(hold_blocks, v);

                // ログ停止（まず記録を止める）
                log_stop();

                // 減速して停止（1区画で0まで）
                run_straight(2.0f, 0.0f, 0);

                drive_fan(0);

                // 安全停止を維持しながら、右前センサで確認待ち（走行後に出力）
                velocity_interrupt = 0;
                omega_interrupt = 0;
                drive_variable_reset();
                drive_stop();
                printf("[ID1] Show FR sensor to confirm (FR>1500 && FL<600) for v=%.0f mm/s...\n", v);
                while (!(ad_fr > 1500 && ad_fl < 600)) {
                    HAL_Delay(10);
                }
                buzzer_enter(900);

                // CSV出力（並進制御値）
                log_print_translation_csv();

                HAL_Delay(200);
            }

            led_flash(5);
            drive_stop();

            // 次の試行のための案内
            printf("[ID1] Completed. Reposition the robot at the start.\n");

            break;

        case 8:

            printf("Test Mode 8: Wall control baseline calibration.\n");

            // 安全停止と制御の無効化
            velocity_interrupt = 0;
            omega_interrupt = 0;
            drive_variable_reset();
            drive_stop();
            MF.FLAG.CTRL = 0;            // 壁制御を無効化
            MF.FLAG.CTRL_DIAGONAL = 0;   // 斜め制御を無効化

            // 案内
            printf("[CAL] Place the robot centered in the corridor with both side walls present.\n");
            printf("[CAL] Press ENTER key on serial to start, or wait until both side walls are detected.\n");

            // 自動開始: 両側壁の存在を待つ
            while (!(ad_r > WALL_BASE_R && ad_l > WALL_BASE_L)) {
                HAL_Delay(10);
            }
            buzzer_enter(1000);

            // 測定・保存（約1.5秒）
            {
                HAL_StatusTypeDef st = sensor_calibrate_wall_ctrl_base_and_save(1500);
                if (st == HAL_OK) {
                    printf("[CAL] Done. base_l=%u, base_r=%u, base_f=%u saved to Flash.\n",
                           (unsigned)base_l, (unsigned)base_r, (unsigned)base_f);
                    led_flash(3);
                } else {
                    printf("[CAL] Failed to save to Flash. HAL=%d\n", st);
                    led_flash(1);
                }
            }

            break;

        case 9:

            printf("Test Mode 9 Circuit.\n");

            HAL_StatusTypeDef st = sensor_recalibrate_and_save();
                if (st == HAL_OK) {
                        printf("Sensor parameters saved to Flash successfully.\n");
                        buzzer_beep(1200);
                    } else {
                        printf("Failed to save sensor parameters. HAL status=%d\n", st);
                        buzzer_beep(3000);
                    }
            break;
        }
    }
}
