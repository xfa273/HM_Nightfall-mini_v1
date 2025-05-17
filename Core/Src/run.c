/*
 * run.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>

void run(void) {


    drive_start();

    speed_now = 0;
    velocity_interrupt = 0;
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();
    led_flash(5);
    get_base();
    led_flash(5);

    first_sectionA();

    for (uint8_t path_count = 0; path[path_count] != 0; path_count++) {
        if (200 < path[path_count] && path[path_count] < 300) {
            // 直進

            float straight_sections = (path[path_count] - 200) * DIST_HALF_SEC;

            // 直線の加減速区画を計算

            // 加速時間、加速距離、減速距離の計算
            float t_acc =
                velocity_straight / acceleration_straight_dash; // 加速時間 [s]
            float d_acc = 0.5f * acceleration_straight_dash * t_acc *
                          t_acc; // 加速距離 [mm]

            // 最高到達速度が走行距離内で到達できるかのチェック
            float d_total_acc_dec = 2 * d_acc; // 加速距離と減速距離の合計 [mm]
            float d_constant = 0.0f;                     // 等速距離 [mm]
            float max_reached_speed = velocity_straight; // 最高到達速度 [mm/s]

            if (d_total_acc_dec > straight_sections) {
                // 最高速度に達しない場合
                d_acc = straight_sections / 2; // 加速距離と減速距離は等しい
                d_constant = 0.0f;
                t_acc = sqrtf(2 * d_acc / acceleration_straight_dash);
                max_reached_speed = acceleration_straight_dash * t_acc;
            } else {
                // 最高速度に達する場合
                d_constant = straight_sections - d_total_acc_dec;
            }

            // 各距離を区画数に変換
            float d_acc_blocks = d_acc / DIST_HALF_SEC;
            float d_constant_blocks = d_constant / DIST_HALF_SEC;
            float d_dec_blocks = d_acc_blocks; // 減速距離は加速距離と等しい

            // 加速区間
            run_straight(d_acc_blocks, max_reached_speed, 0);

            // 等速区間
            run_straight(d_constant_blocks, max_reached_speed, 0);

            // 減速区間

            if (path[path_count + 1] < 500) {
                // 次が通常旋回
                run_straight(d_dec_blocks, velocity_turn90, dist_wall_end + 90);
            } else if (path[path_count + 1] < 600) {
                // 次が右大回り旋回
                uint8_t l_turn_sections = path[path_count + 1] - 500;

                if (l_turn_sections == 2) {
                    // 180deg大回り旋回
                    run_straight(d_dec_blocks, velocity_l_turn_180,
                                 dist_wall_end);
                } else {
                    // 90deg大回り旋回
                    run_straight(d_dec_blocks, velocity_l_turn_90,
                                 dist_wall_end);
                }
            } else if (path[path_count + 1] < 700) {
                // 次が左大回り旋回
                uint8_t l_turn_sections = path[path_count + 1] - 600;

                if (l_turn_sections == 2) {
                    // 180deg大回り旋回
                    run_straight(d_dec_blocks, velocity_l_turn_180,
                                 dist_wall_end);
                } else {
                    // 90deg大回り旋回
                    run_straight(d_dec_blocks, velocity_l_turn_90,
                                 dist_wall_end);
                }
            } else if (path[path_count + 1] < 800) {
                // 次が45degターン
                run_straight(d_dec_blocks, velocity_turn45in, dist_wall_end);
            } else if (path[path_count + 1] < 900) {
                // 次がV90degターン
                run_straight(d_dec_blocks, velocity_turnV90, dist_wall_end);
            } else if (path[path_count + 1] < 1000) {
                // 次が135degターン
                run_straight(d_dec_blocks, velocity_turn135in, dist_wall_end);
            } else {
                // 次が終了
                run_straight(d_dec_blocks, 0, 0);
            }

        } else if (path[path_count] < 400) {
            // 右旋回

            // uint8_t turn_sections = path[path_count] - 300;
            // printf("Turn R %d Sections.\n", path[path_count] - 300);

            led_write(0, 1);

            turn_R90(1);

            turn_dir(DIR_TURN_R90); // マイクロマウス内部位置情報でも右回転処理

            led_write(0, 0);

        } else if (path[path_count] < 500) {
            // 左旋回

            // uint8_t turn_sections = path[path_count] - 400;
            // printf("Turn L %d Sections.\n", path[path_count] - 400);

            led_write(1, 0);

            turn_L90(1);

            turn_dir(DIR_TURN_L90); // マイクロマウス内部位置情報でも右回転処理

            led_write(0, 0);

        } else if (path[path_count] < 600) {
            // 右大回り旋回

            uint8_t l_turn_sections = path[path_count] - 500;
            // printf("Large Turn R %d Sections.\n", l_turn_sections);

            if (l_turn_sections == 2) {
                l_turn_R180(0);
            } else {
                l_turn_R90();
            }
        } else if (path[path_count] < 700) {
            // 左大回り旋回

            uint8_t l_turn_sections = path[path_count] - 600;
            // printf("Large Turn L %d Sections.\n", l_turn_sections);

            if (l_turn_sections == 2) {
                l_turn_L180(0);
            } else {
                l_turn_L90();
            }
        } else if (path[path_count] == 701) {
            // 45degターン右入り

            turn_R45_In();

        } else if (path[path_count] == 702) {
            // 45degターン左入り

            turn_L45_In();

        } else if (path[path_count] == 703) {
            // 45degターン右出

            turn_R45_Out();

        } else if (path[path_count] == 704) {
            // 45degターン左出

            turn_L45_Out();

        } else if (path[path_count] == 801) {
            // V90degターン右

            turn_RV90();

        } else if (path[path_count] == 802) {
            // V90degターン左

            turn_LV90();

        } else if (path[path_count] == 901) {
            // 135degターン右入り

            turn_R135_In();

        } else if (path[path_count] == 902) {
            // 135degターン左入り

            turn_L135_In();

        } else if (path[path_count] == 903) {
            // 135degターン右出

            turn_R135_Out();

        } else if (path[path_count] == 904) {
            // 135degターン左出

            turn_L135_Out();

        } else if (1000 < path[path_count] && path[path_count] < 1100) {
            // 斜め直進

            float straight_sections =
                (path[path_count] - 1000) * DIST_D_HALF_SEC;

            // 直線の加減速区画を計算

            // 加速時間、加速距離、減速距離の計算
            float t_acc = velocity_d_straight /
                          acceleration_d_straight_dash; // 加速時間 [s]
            float d_acc = 0.5f * acceleration_d_straight_dash * t_acc *
                          t_acc; // 加速距離 [mm]

            // 最高到達速度が走行距離内で到達できるかのチェック
            float d_total_acc_dec = 2 * d_acc; // 加速距離と減速距離の合計 [mm]
            float d_constant = 0.0f; // 等速距離 [mm]
            float max_reached_speed =
                velocity_d_straight; // 最高到達速度 [mm/s]

            if (d_total_acc_dec > straight_sections) {
                // 最高速度に達しない場合
                d_acc = straight_sections / 2; // 加速距離と減速距離は等しい
                d_constant = 0.0f;
                t_acc = sqrtf(2 * d_acc / acceleration_d_straight_dash);
                max_reached_speed = acceleration_d_straight_dash * t_acc;
            } else {
                // 最高速度に達する場合
                d_constant = straight_sections - d_total_acc_dec;
            }

            // 各距離を区画数に変換
            float d_acc_blocks = d_acc / DIST_D_HALF_SEC;
            float d_constant_blocks = d_constant / DIST_D_HALF_SEC;
            float d_dec_blocks = d_acc_blocks; // 減速距離は加速距離と等しい

            // 加速区間
            run_diagonal(d_acc_blocks, max_reached_speed);

            // 等速区間
            run_diagonal(d_constant_blocks, max_reached_speed);

            // 減速区間
            if (path[path_count + 1] < 800) {
                // 次が45degターン
                run_diagonal(d_dec_blocks, velocity_turn45out);
            } else if (path[path_count + 1] < 900) {
                // 次がV90degターン
                run_diagonal(d_dec_blocks, velocity_turnV90);
            } else if (path[path_count + 1] < 1000) {
                // 次が135degターン
                run_diagonal(d_dec_blocks, velocity_turn135out);
            } else {
                // 次が終了
                run_diagonal(d_dec_blocks, 0);
            }
        }
    }

    half_sectionD(0);

    led_flash(5);
    buzzer_beep(300);

    drive_stop();
}
