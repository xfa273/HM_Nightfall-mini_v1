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

    bool skip_next = false; // 壁切れ即時ターンで次要素（ターン）を1回だけスキップするためのフラグ

    for (uint8_t path_count = 0; path[path_count] != 0; path_count++) {
        if (skip_next) {
            skip_next = false;
            continue; // 次要素を1回だけ無処理で飛ばす
        }
        if (200 < path[path_count] && path[path_count] < 300) {
            // 直進

            float straight_mm = (path[path_count] - 200) * DIST_HALF_SEC; // [mm]

            // 次動作に応じたターン入口速度を決定（0なら終端）
            uint16_t next_code = path[path_count + 1];
            float v_next = 0.0f; // [mm/s]
            if (next_code >= 300 && next_code < 400) {
                // 右90
                v_next = velocity_turn90;
            } else if (next_code >= 400 && next_code < 500) {
                // 左90
                v_next = velocity_turn90;
            } else if (next_code >= 500 && next_code < 600) {
                uint8_t l_turn_sections = next_code - 500;
                v_next = (l_turn_sections == 2) ? velocity_l_turn_180 : velocity_l_turn_90;
            } else if (next_code >= 600 && next_code < 700) {
                uint8_t l_turn_sections = next_code - 600;
                v_next = (l_turn_sections == 2) ? velocity_l_turn_180 : velocity_l_turn_90;
            } else if (next_code >= 701 && next_code < 800) {
                v_next = velocity_turn45in;
            } else if (next_code >= 801 && next_code < 900) {
                v_next = velocity_turnV90;
            } else if (next_code >= 901 && next_code < 1000) {
                v_next = velocity_turn135in;
            } else {
                v_next = 0.0f; // 次なし（終端）
            }

            // ゴール停止時（次コード=0）は、最後に half_sectionD(0) で確実に止めるため
            // 直前直進の終端速度をわずかに残す（0にしてしまうと half_sectionD が走らない）
            if (next_code == 0) {
                const float GOAL_ENTRY_SPEED = 120.0f; // [mm/s] 実機で微調整可
                v_next = GOAL_ENTRY_SPEED;
            }

            // 次が「実際のターン」かを明示的に判定（直進や終端は含めない）
            bool next_is_turn =
                (next_code >= 300 && next_code < 700) ||
                (next_code == 701 || next_code == 702 || next_code == 703 || next_code == 704) ||
                (next_code == 801 || next_code == 802) ||
                (next_code >= 901 && next_code <= 904);

            // 壁切れ用のバッファ距離（次がターンのときのみ適用）
            const float buffer_mm_cfg = WALL_END_BUFFER_MM; // params.h から設定
            bool next_is_small_turn = (next_code >= 300 && next_code < 500);
            float buffer_mm = 0.0f;
            if (next_is_turn) {
                buffer_mm = buffer_mm_cfg;
                // 小回りターンでは壁切れ後に半区間前進するため、
                // 事前の直進距離を半区間ぶん短縮してトータル距離を一定に保つ（= バッファを半区間ぶん拡張）
                if (next_is_small_turn) {
                    buffer_mm += (float)DIST_HALF_SEC;
                }
            }
            if (buffer_mm > straight_mm) buffer_mm = straight_mm; // 過剰保護

            // バッファを除いた距離で従来の加減速（v_nextで収束）
            float main_mm = straight_mm - buffer_mm; // [mm]

            // 直線の加減速区画を計算（メイン区間）
            float t_acc = velocity_straight / acceleration_straight_dash; // [s]
            float d_acc = 0.5f * acceleration_straight_dash * t_acc * t_acc; // [mm]

            // 最高到達速度が走行距離内で到達できるかのチェック
            float d_total_acc_dec = 2 * d_acc; // 加速距離と減速距離の合計 [mm]
            float d_constant = 0.0f;                     // 等速距離 [mm]
            float max_reached_speed = velocity_straight; // 最高到達速度 [mm/s]

            if (d_total_acc_dec > main_mm) {
                // 最高速度に達しない場合（等加速→等減速の三角形）
                d_acc = main_mm / 2.0f; // 加速距離と減速距離は等しい
                d_constant = 0.0f;
                t_acc = sqrtf(2.0f * d_acc / acceleration_straight_dash);
                max_reached_speed = acceleration_straight_dash * t_acc;
            } else {
                // 最高速度に達する場合（台形）
                d_constant = main_mm - d_total_acc_dec;
            }

            // 各距離を区画数に変換
            float d_acc_blocks = d_acc / DIST_HALF_SEC;
            float d_constant_blocks = d_constant / DIST_HALF_SEC;
            float d_dec_blocks = d_acc_blocks; // 減速距離は加速距離と等しい（従来踏襲）

            // メイン区間 実行
            if (d_acc_blocks > 0.0f) {
                // 加速区間
                run_straight(d_acc_blocks, max_reached_speed, 0);
            }
            if (d_constant_blocks > 0.0f) {
                // 等速区間
                run_straight(d_constant_blocks, max_reached_speed, 0);
            }
            if (d_dec_blocks > 0.0f) {
                // 減速区間（ターン入口速度へ）
                run_straight(d_dec_blocks, v_next, 0);
            }

            // バッファ区間（等速: v_next）で壁切れ検知をアーム
            if (buffer_mm > 0.0f) {
                float buffer_blocks = buffer_mm / DIST_HALF_SEC;
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;
                MF.FLAG.WALL_END = 1; // アーム
                // 可視化: バッファ区間に入ったことをブザーで通知
                buzzer_interrupt(1200);

                // 小刻み実行して壁切れ検知したら即座に中断してターンへ
                float remaining = buffer_blocks;
                const float step_blocks = (2.0f / DIST_HALF_SEC); // 2mm ステップ
                bool triggered = false;
                while (remaining > 0.0f) {
                    float step = (remaining < step_blocks) ? remaining : step_blocks;
                    run_straight(step, v_next, 0);
                    if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                        triggered = true;
                        // 消費したので下げる
                        MF.FLAG.R_WALL_END = 0;
                        MF.FLAG.L_WALL_END = 0;
                        break;
                    }
                    remaining -= step;
                }
                if (triggered) {
                    MF.FLAG.WALL_END = 0; // 解除
                    // 検知後の追従距離を進む（v_next 等速）
                    // 小回りターン（300-499）の場合は半区間、その他は dist_wall_end
                    float follow_mm = ((next_code >= 300 && next_code < 500) ? (float)DIST_HALF_SEC : dist_wall_end);
                    if (follow_mm > 0.0f && v_next > 0.0f) {
                        float extra_blocks = follow_mm / DIST_HALF_SEC;
                        run_straight(extra_blocks, v_next, 0);
                    }
                    bool consumed = false;
                    // 壁切れ検知: 次の動作（ターン）を即時開始し、次のパス要素は消費する
                    if (next_code >= 300 && next_code < 400) {
                        // 右旋回
                        turn_R90(1);
                        turn_dir(DIR_TURN_R90);
                        consumed = true;
                    } else if (next_code >= 400 && next_code < 500) {
                        // 左旋回
                        turn_L90(1);
                        turn_dir(DIR_TURN_L90);
                        consumed = true;
                    } else if (next_code >= 500 && next_code < 600) {
                        // 右大回り
                        uint8_t l_turn_sections = next_code - 500;
                        if (l_turn_sections == 2) {
                            l_turn_R180(0);
                        } else {
                            l_turn_R90();
                        }
                        consumed = true;
                    } else if (next_code >= 600 && next_code < 700) {
                        // 左大回り
                        uint8_t l_turn_sections = next_code - 600;
                        if (l_turn_sections == 2) {
                            l_turn_L180(0);
                        } else {
                            l_turn_L90();
                        }
                        consumed = true;
                    } else if (next_code == 701) {
                        turn_R45_In();
                        consumed = true;
                    } else if (next_code == 702) {
                        turn_L45_In();
                        consumed = true;
                    } else if (next_code == 703) {
                        turn_R45_Out();
                        consumed = true;
                    } else if (next_code == 704) {
                        turn_L45_Out();
                        consumed = true;
                    } else if (next_code == 801) {
                        turn_RV90();
                        consumed = true;
                    } else if (next_code == 802) {
                        turn_LV90();
                        consumed = true;
                    } else if (next_code == 901) {
                        turn_R135_In();
                        consumed = true;
                    } else if (next_code == 902) {
                        turn_L135_In();
                        consumed = true;
                    } else if (next_code == 903) {
                        turn_R135_Out();
                        consumed = true;
                    } else if (next_code == 904) {
                        turn_L135_Out();
                        consumed = true;
                    } else {
                        // 次動作なし（終端）
                    }

                    // 次のパス要素（ターン）を消費した場合のみ、次の1要素をスキップ
                    if (consumed) {
                        skip_next = true; // for文のインクリメントに任せ、次の1要素のみスキップ
                        continue;
                    }
                } else {
                    // 壁切れがまだ出ていない: バッファを延長して v_next 等速で継続（検知まで）
                    if (next_is_turn) {
                        // 本来の距離（straight_mm）に加えて最大延長まで探す
                        const float extend_mm_cfg = WALL_END_EXTEND_MAX_MM;
                        float extend_remaining_blocks = extend_mm_cfg / DIST_HALF_SEC;

                        bool trig2 = false;
                        while (!trig2 && extend_remaining_blocks > 0.0f) {
                            float step = step_blocks; // 2mm 相当
                            if (extend_remaining_blocks < step) step = extend_remaining_blocks;
                            run_straight(step, v_next, 0);
                            extend_remaining_blocks -= step;
                            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                                // 消費
                                MF.FLAG.R_WALL_END = 0;
                                MF.FLAG.L_WALL_END = 0;
                                trig2 = true;
                                break;
                            }
                        }

                        MF.FLAG.WALL_END = 0; // 解除

                        // 検知後の追従距離を進む（v_next 等速）: 検知が成立した場合のみ
                        if (trig2) {
                            float follow_mm2 = ((next_code >= 300 && next_code < 500) ? (float)DIST_HALF_SEC : dist_wall_end);
                            if (follow_mm2 > 0.0f && v_next > 0.0f) {
                                float extra_blocks = follow_mm2 / DIST_HALF_SEC;
                                run_straight(extra_blocks, v_next, 0);
                            }
                        }

                        // 即時ターン開始し、次のパス要素を消費
                        bool consumed2 = false;
                        if (next_code >= 300 && next_code < 400) {
                            // 右旋回
                            turn_R90(1);
                            turn_dir(DIR_TURN_R90);
                            consumed2 = true;
                        } else if (next_code >= 400 && next_code < 500) {
                            // 左旋回
                            turn_L90(1);
                            turn_dir(DIR_TURN_L90);
                            consumed2 = true;
                        } else if (next_code >= 500 && next_code < 600) {
                            // 右大回り
                            uint8_t l_turn_sections = next_code - 500;
                            if (l_turn_sections == 2) {
                                l_turn_R180(0);
                            } else {
                                l_turn_R90();
                            }
                            consumed2 = true;
                        } else if (next_code >= 600 && next_code < 700) {
                            // 左大回り
                            uint8_t l_turn_sections = next_code - 600;
                            if (l_turn_sections == 2) {
                                l_turn_L180(0);
                            } else {
                                l_turn_L90();
                            }
                            consumed2 = true;
                        } else if (next_code == 701) {
                            turn_R45_In();
                            consumed2 = true;
                        } else if (next_code == 702) {
                            turn_L45_In();
                            consumed2 = true;
                        } else if (next_code == 703) {
                            turn_R45_Out();
                            consumed2 = true;
                        } else if (next_code == 704) {
                            turn_L45_Out();
                            consumed2 = true;
                        } else if (next_code == 801) {
                            turn_RV90();
                            consumed2 = true;
                        } else if (next_code == 802) {
                            turn_LV90();
                            consumed2 = true;
                        } else if (next_code == 901) {
                            turn_R135_In();
                            consumed2 = true;
                        } else if (next_code == 902) {
                            turn_L135_In();
                            consumed2 = true;
                        } else if (next_code == 903) {
                            turn_R135_Out();
                            consumed2 = true;
                        } else if (next_code == 904) {
                            turn_L135_Out();
                            consumed2 = true;
                        } else {
                            // 次動作なし（終端）
                        }

                        if (consumed2) {
                            skip_next = true; // for文のインクリメントに任せ、次の1要素のみスキップ
                            continue;
                        }
                    } else {
                        // 次動作がない場合は解除のみ
                        MF.FLAG.WALL_END = 0; // 解除
                    }
                }
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

    // ゴール演出（LED/Buzzer）は run_shortest() 側で必要に応じて実施する。
    // ここでは直後にファン停止やLED消灯を行うため、一時的な再点灯/再起動を避ける目的で呼ばない。
    drive_stop();
}

void run_shortest(uint8_t mode, uint8_t case_index) {
    // case_index: 1..9 -> idx 0..8（mode2/3 は9要素、他は従来通り）
    uint8_t idx = 0;
    if (case_index >= 1 && case_index <= 9) {
        idx = (uint8_t)(case_index - 1);
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

    // モードごとのケース数でクランプ（mode2/3/4:9要素=idx0..8、その他:5要素=idx0..4）
    uint8_t max_idx = 4;
    if (mode == 2 || mode == 3 || mode == 4) {
        max_idx = 8;
    }
    if (idx > max_idx) idx = max_idx;

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
    // 斜め直線（caseごと）
    acceleration_d_straight      = p->acceleration_d_straight;
    acceleration_d_straight_dash = p->acceleration_d_straight_dash;
    velocity_d_straight          = p->velocity_d_straight;
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
    // 斜めターン（mode共通）
    velocity_turn45in          = pm->velocity_turn45in;
    alpha_turn45in             = pm->alpha_turn45in;
    angle_turn45in             = pm->angle_turn45in;
    dist_turn45in_in           = pm->dist_turn45in_in;
    dist_turn45in_out          = pm->dist_turn45in_out;
    velocity_turn45out         = pm->velocity_turn45out;
    alpha_turn45out            = pm->alpha_turn45out;
    angle_turn45out            = pm->angle_turn45out;
    dist_turn45out_in          = pm->dist_turn45out_in;
    dist_turn45out_out         = pm->dist_turn45out_out;
    velocity_turnV90           = pm->velocity_turnV90;
    alpha_turnV90              = pm->alpha_turnV90;
    angle_turnV90              = pm->angle_turnV90;
    dist_turnV90_in            = pm->dist_turnV90_in;
    dist_turnV90_out           = pm->dist_turnV90_out;
    velocity_turn135in         = pm->velocity_turn135in;
    alpha_turn135in            = pm->alpha_turn135in;
    angle_turn135in            = pm->angle_turn135in;
    dist_turn135in_in          = pm->dist_turn135in_in;
    dist_turn135in_out         = pm->dist_turn135in_out;
    velocity_turn135out        = pm->velocity_turn135out;
    alpha_turn135out           = pm->alpha_turn135out;
    angle_turn135out           = pm->angle_turn135out;
    dist_turn135out_in         = pm->dist_turn135out_in;
    dist_turn135out_out        = pm->dist_turn135out_out;
    // 壁制御（caseごと）
    kp_wall                    = p->kp_wall;

    // 壁切れ後の距離・ケツ当て
    dist_wall_end = pm->dist_wall_end;
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
    led_wait();
}
