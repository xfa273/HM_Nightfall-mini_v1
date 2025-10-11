/*
 * run.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>
#include "../Inc/shortest_run_params.h"
#include "../Inc/path.h"
#include "../Inc/solver.h"

void run(void) {


    drive_start();

    speed_now = 0;
    velocity_interrupt = 0;
    drive_variable_reset();
    drive_enable_motor();
    get_base();

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

            turn_R90(1);

            turn_dir(DIR_TURN_R90); // マイクロマウス内部位置情報でも右回転処理

        } else if (path[path_count] < 500) {
            // 左旋回

            turn_L90(1);

            turn_dir(DIR_TURN_L90); // マイクロマウス内部位置情報でも右回転処理

        } else if (path[path_count] < 600) {
            // 右大回り旋回

            uint8_t l_turn_sections = path[path_count] - 500;

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

void run_shortest(uint8_t mode, uint8_t case_index) {
    // case_index: 3..7 -> idx 0..4
    uint8_t idx = 0;
    if (case_index >= 3 && case_index <= 7) {
        idx = (uint8_t)(case_index - 3);
    } else {
        // フォールバック: 0 を使用
        idx = 0;
    }

    const ShortestRunModeParams_t *pm = NULL;
    const ShortestRunCaseParams_t *pcases = NULL;
    switch (mode) {
        case 2: pm = &shortestRunModeParams2; pcases = &shortestRunCaseParamsMode2[0]; break;
        case 3: pm = &shortestRunModeParams3; pcases = &shortestRunCaseParamsMode3[0]; break;
        case 4: pm = &shortestRunModeParams4; pcases = &shortestRunCaseParamsMode4[0]; break;
        case 5: pm = &shortestRunModeParams5; pcases = &shortestRunCaseParamsMode5[0]; break;
        case 6: pm = &shortestRunModeParams6; pcases = &shortestRunCaseParamsMode6[0]; break;
        case 7: pm = &shortestRunModeParams7; pcases = &shortestRunCaseParamsMode7[0]; break;
        default: pm = &shortestRunModeParams2; pcases = &shortestRunCaseParamsMode2[0]; break;
    }

    const ShortestRunCaseParams_t *p = &pcases[idx];

    printf("Mode %d-%d Shortest Run.\n", mode, case_index);

    // 経路の重み（テーブルから設定）
    straight_weight = p->straight_weight;
    diagonal_weight = p->diagonal_weight;

    // 経路作成（新ソルバを使用）
    solver_build_path(mode, case_index);

    // 走行フラグ
    MF.FLAG.RUNNING = 1;

    // パラメータ適用
    // 直線（caseごと）
    acceleration_straight      = p->acceleration_straight;
    acceleration_straight_dash = p->acceleration_straight_dash;
    velocity_straight          = p->velocity_straight;
    // ターン（mode共通）
    velocity_turn90            = pm->velocity_turn90;
    alpha_turn90               = pm->alpha_turn90;
    acceleration_turn          = pm->acceleration_turn;
    dist_offset_in             = pm->dist_offset_in;
    dist_offset_out            = pm->dist_offset_out;
    val_offset_in              = pm->val_offset_in;
    angle_turn_90              = pm->angle_turn_90;
    velocity_l_turn_90         = pm->velocity_l_turn_90;
    alpha_l_turn_90            = pm->alpha_l_turn_90;
    angle_l_turn_90            = pm->angle_l_turn_90;
    dist_l_turn_in_90          = pm->dist_l_turn_in_90;
    dist_l_turn_out_90         = pm->dist_l_turn_out_90;
    velocity_l_turn_180        = pm->velocity_l_turn_180;
    alpha_l_turn_180           = pm->alpha_l_turn_180;
    angle_l_turn_180           = pm->angle_l_turn_180;
    dist_l_turn_in_180         = pm->dist_l_turn_in_180;
    dist_l_turn_out_180        = pm->dist_l_turn_out_180;
    // 壁制御（caseごと）
    kp_wall                    = p->kp_wall;

    // 壁切れ後の距離・ケツ当て
    dist_wall_end = 0;
    duty_setposition = 40;

    velocity_interrupt = 0;

    // センサ・モータ初期化
    led_flash(10);
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();

    MF.FLAG.SCND = 1;
    MF.FLAG.RETURN = 0;

    led_flash(5);
    get_base();

    led_write(1,1);

    // ファン出力（mode共通）
    drive_fan(pm->fan_power);

    // 実行
    run();

    // 後処理
    drive_fan(0);
    MF.FLAG.RUNNING = 0;

    led_write(0,0);
}
