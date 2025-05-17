/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/turn_time_calculator.h"

void mode4() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: // 通常ターンの調整

            printf("Mode 4-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // ターン
            velocity_turn90 = 1000;
            alpha_turn90 = 22600;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
            val_offset_in = 680;
            angle_turn_90 = 88;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1000);
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 1: // 90deg大回りターンの調整

            printf("Mode 4-1 Large Turn 90deg.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 90°大回りターン
            velocity_l_turn_90 = 1300;
            alpha_l_turn_90 = 7150;
            angle_l_turn_90 = 88;
            dist_l_turn_out_90 = 45;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1300);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 2: // 180deg大回りターンの調整

            printf("Mode 4-2 Large Turn 180deg.\n");

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 0;
            velocity_straight = 0;

            // 180°大回りターン
            velocity_l_turn_180 = 1300;
            alpha_l_turn_180 = 7300;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 40;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(180);

            led_flash(20);

            half_sectionA(1300);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 3:
            printf("Mode 4-3.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(0);

            // 直線
            acceleration_straight = 5555.6;
            acceleration_straight_dash = 12000;
            velocity_straight = 3500;
            // ターン
            velocity_turn90 = 1000;
            alpha_turn90 = 22400;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 52;
            val_offset_in = 1100;
            angle_turn_90 = 88;
            // 壁制御とケツ当て
            kp_wall = 0.1;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(250);

            led_flash(3);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 4:
            printf("Mode 4-4.\n");

            // 100パターンの経路対比用の配列
            uint16_t best_path[256] = {0}; // 最適経路を保存する配列
            int best_path_length = 0;       // 最適経路の長さ
            float best_time = 999999.0f;     // 最短所要時間（大きな初期値）
            int best_sw = 0;                // 最適な直線の重み
            int best_dw = 0;                // 最適な斜めの重み
            const ShortestRunParam_t *p = &shortestRunParamsMode4[0]; // 走行パラメータ
            
            printf("\n*** 経路対比開始 ***\n");
            
            // 100パターンの経路を生成し所要時間計算
            for (int sw = 0; sw < 10; sw++) {
                for (int dw = 0; dw < 10; dw++) {
                    // 経路の重み設定
                    straight_weight = sw; // 直線の優先度
                    diagonal_weight = dw; // 斜めの優先度
                    
                    // path配列を初期化
                    for (int k = 0; k < 256; k++) {
                        path[k] = 0;
                    }
                    
                    // 経路対比モード（pathのみ作成、実行はしない）
                    makePath(1);
                    
                    // 経路の長さを数える
                    int path_length = 0;
                    while (path[path_length] != 0) {
                        path_length++;
                    }
                    
                    // 所要時間計算
                    float total_time = calculate_total_run_time(path, 256, p);
                    
                    // 経路生成結果表示
                    printf("SW=%d, DW=%d: 所要時間=%.5f秒, 長さ=%d\n", 
                           sw, dw, total_time, path_length);
                    
                    // より短い所要時間が見つかった場合は更新
                    if (total_time < best_time) {
                        best_time = total_time;
                        best_sw = sw;
                        best_dw = dw;
                        best_path_length = path_length;
                        
                        // 最適経路を保存
                        for (int i = 0; i <= path_length; i++) { // 0終端も含めてコピー
                            best_path[i] = path[i];
                        }
                    }
                }
            }
            
            // 最適経路をpath配列に返す
            for (int i = 0; i <= best_path_length; i++) { // 0終端も含めてコピー
                path[i] = best_path[i];
            }
            
            // 最適値を設定
            straight_weight = best_sw;
            diagonal_weight = best_dw;
            
            // 結果表示
            printf("\n*** 最適経路結果 ***\n");
            printf("最適パラメータ: SW=%d, DW=%d\n", best_sw, best_dw);
            printf("最短所要時間: %.5f秒\n", best_time);
            printf("経路ステップ数: %d\n", best_path_length);
            
            // 最適経路の詳細表示
            printf("\n*** 最適走行経路 ***\n");
            for (int i = 0; i < best_path_length; i++) {
                printf("path[%d] = %d\n", i, path[i]);
            }
            printf("*** 経路終了 ***\n\n");
            
            // 最適経路の所要時間詳細
            printf("\n*** 最適経路の所要時間詳細 ***\n");
            print_path_times(path, 256, p);
            
            {
                const ShortestRunParam_t *p = &shortestRunParamsMode4[0];
                
                // 走行全体の所要時間を計算して表示
                printf("\n*** 走行全体の所要時間計算（case4） ***\n");
                
                // 各動作の所要時間を計算して表示
                printf("走行経路の各動作の所要時間:\n");
                print_path_times(path, 100, p);  // path配列の最大サイズを100と仮定
                
                // ターン走行の所要時間計算も表示（従来の処理）
                printf("\n*** ターン走行所要時間計算 ***\n");
                print_turn_times(p);
                printf("*** 計算終了 ***\n\n");
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.25;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(250);

            led_flash(3);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 5:
            printf("Mode 4-5.\n");

            // 経路の重み
            straight_weight = 5; // 直線の優先度
            diagonal_weight = 5; // 斜めの優先度

            makePath(1);
            
            // path配列の内容を表示
            printf("\n*** 走行経路 ***\n");
            for (int i = 0; path[i] != 0; i++) {
                printf("path[%d] = %d\n", i, path[i]);
            }
            printf("*** 経路終了 ***\n\n");

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode4[2];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.25;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(250);

            led_flash(3);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 6:

            printf("Mode 4-6.\n");

            // 経路の重み
            straight_weight = 3; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);
            
            // path配列の内容を表示
            printf("\n*** 走行経路 ***\n");
            for (int i = 0; path[i] != 0; i++) {
                printf("path[%d] = %d\n", i, path[i]);
            }
            printf("*** 経路終了 ***\n\n");

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode4[2];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.25;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(250);

            led_flash(3);

            run();

            drive_fan(0);

            led_wait();

            break;

        case 7:

            printf("Mode 4-7.\n");

            // 経路の重み
            straight_weight = 0; // 直線の優先度
            diagonal_weight = 0; // 斜めの優先度

            makePath(1);
            
            // path配列の内容を表示
            printf("\n*** 走行経路 ***\n");
            for (int i = 0; path[i] != 0; i++) {
                printf("path[%d] = %d\n", i, path[i]);
            }
            printf("*** 経路終了 ***\n\n");

            {
                const ShortestRunParam_t *p = &shortestRunParamsMode4[3];
                acceleration_straight      = p->acceleration_straight;
                acceleration_straight_dash = p->acceleration_straight_dash;
                velocity_straight          = p->velocity_straight;
                velocity_turn90            = p->velocity_turn90;
                alpha_turn90               = p->alpha_turn90;
                acceleration_turn          = p->acceleration_turn;
                dist_offset_in             = p->dist_offset_in;
                dist_offset_out            = p->dist_offset_out;
                val_offset_in              = p->val_offset_in;
                angle_turn_90              = p->angle_turn_90;
                velocity_l_turn_90         = p->velocity_l_turn_90;
                alpha_l_turn_90            = p->alpha_l_turn_90;
                angle_l_turn_90            = p->angle_l_turn_90;
                dist_l_turn_out_90         = p->dist_l_turn_out_90;
                velocity_l_turn_180        = p->velocity_l_turn_180;
                alpha_l_turn_180           = p->alpha_l_turn_180;
                angle_l_turn_180           = p->angle_l_turn_180;
                dist_l_turn_out_180        = p->dist_l_turn_out_180;
            }
            // 壁制御とケツ当て
            kp_wall = 0.25;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            get_base();

            drive_fan(250);

            led_flash(3);

            run();

            drive_fan(0);

            led_wait();

            break;




        }
    }
}
