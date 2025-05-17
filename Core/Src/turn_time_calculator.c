/**
 * @file turn_time_calculator.c
 * @brief ターン走行の所要時間を計算する関数の実装
 */

#include "turn_time_calculator.h"
#include <stdio.h>
#include <stdlib.h>    // abs

/**
 * @brief ターン走行の所要時間を計算する
 * @param turning_angle_deg 旋回角度 [deg]
 * @param translational_velocity 並進速度 [mm/s]
 * @param angular_acceleration_deg 角加速度 [deg/s²]
 * @param front_offset_distance 前オフセット距離 [mm]
 * @param rear_offset_distance 後オフセット距離 [mm]
 * @return ターン走行の所要時間 [s]
 */
float calculate_turn_time(float turning_angle_deg, float translational_velocity, 
                         float angular_acceleration_deg, float front_offset_distance, 
                         float rear_offset_distance) {
    // ラジアンに変換
    float theta_tot = turning_angle_deg * (M_PI / 180.0f);
    float alpha_mag = angular_acceleration_deg * (M_PI / 180.0f);
    
    // 旋回角を3等分（「入り」設定）
    float theta1 = theta_tot / 3.0f;
    float theta2 = theta_tot / 3.0f;
    
    // フェーズ2：クロソイド入り（加速）の時間
    float t1 = sqrtf(2.0f * theta1 / alpha_mag);
    
    // フェーズ3：円弧区間（定角速度）の時間
    float w1 = -alpha_mag * t1;  // フェーズ2終了時の角速度
    float t2 = theta2 / fabsf(w1);
    
    // フェーズ4：クロソイド出口（減速）の時間
    float t3 = t1;
    
    // 前後直進部分の時間
    float straight_time = 0.0f;
    if (translational_velocity > 0.0f) {
        straight_time = (front_offset_distance + rear_offset_distance) / translational_velocity;
    }
    
    // 合計時間
    return t1 + t2 + t3 + straight_time;
}

/**
 * @brief shortest_run_paramsからパラメータを取得してターン所要時間を計算して表示
 * @param params 走行パラメータ構造体
 */
void print_turn_times(const ShortestRunParam_t *params) {
    // 通常の90度ターンの所要時間計算
    float time_90turn = calculate_turn_time(
        params->angle_turn_90,           // 旋回角度 [deg]
        params->velocity_turn90,         // 並進速度 [mm/s]
        params->alpha_turn90,            // 角加速度 [deg/s²]
        params->dist_offset_in,          // 前オフセット距離 [mm]
        params->dist_offset_out          // 後オフセット距離 [mm]
    );
    
    // 大回り90度ターンの所要時間計算
    float time_l_90turn = calculate_turn_time(
        params->angle_l_turn_90,         // 旋回角度 [deg]
        params->velocity_l_turn_90,      // 並進速度 [mm/s]
        params->alpha_l_turn_90,         // 角加速度 [deg/s²]
        params->dist_offset_in,          // 前オフセット距離 [mm]
        params->dist_l_turn_out_90       // 後オフセット距離 [mm]
    );
    
    // 大回り180度ターンの所要時間計算
    float time_l_180turn = calculate_turn_time(
        params->angle_l_turn_180,        // 旋回角度 [deg]
        params->velocity_l_turn_180,     // 並進速度 [mm/s]
        params->alpha_l_turn_180,        // 角加速度 [deg/s²]
        params->dist_offset_in,          // 前オフセット距離 [mm]
        params->dist_l_turn_out_180      // 後オフセット距離 [mm]
    );
    
    // 結果の表示
    printf("==== ターン走行所要時間計算結果 ====\n");
    printf("通常90度ターン: %.5f 秒\n", time_90turn);
    printf("大回り90度ターン: %.5f 秒\n", time_l_90turn);
    printf("大回り180度ターン: %.5f 秒\n", time_l_180turn);
}

/**
 * @brief 直進の所要時間を計算する（run.cの実装に合わせて計算）
 * @param section_count マス数
 * @param velocity 最高速度 [mm/s]
 * @param acceleration 加速度 [mm/s^2]
 * @return 直進の所要時間 [s]
 */
float calculate_straight_time(int section_count, float velocity, float acceleration) {
    // 1マス = 90mmとして計算（DIST_HALF_SECと同等）
    const float DIST_PER_SECTION = 90.0f;
    float distance = section_count * DIST_PER_SECTION;
    
    // run.cの実装に合わせて計算
    
    // 加速時間、加速距離の計算
    float t_acc = velocity / acceleration; // 加速時間 [s]
    float d_acc = 0.5f * acceleration * t_acc * t_acc; // 加速距離 [mm]
    
    // 最高到達速度が走行距離内で到達できるかチェック
    float d_total_acc_dec = 2 * d_acc; // 加速距離と減速距離の合計 [mm]
    float d_constant = 0.0f;            // 等速距離 [mm]
    float max_reached_speed = velocity; // 最高到達速度 [mm/s]
    float total_time = 0.0f;            // 全体の所要時間 [s]
    
    if (d_total_acc_dec > distance) {
        // 最高速度に達しない場合
        d_acc = distance / 2.0f; // 加速距離と減速距離は等しい
        d_constant = 0.0f;
        t_acc = sqrtf(2.0f * d_acc / acceleration);
        max_reached_speed = acceleration * t_acc;
        
        // 所要時間 = 加速時間 + 減速時間
        total_time = 2.0f * t_acc;
    } else {
        // 最高速度に達する場合
        d_constant = distance - d_total_acc_dec;
        
        // 所要時間 = 加速時間 + 等速時間 + 減速時間
        float t_constant = d_constant / velocity; // 等速時間 [s]
        total_time = 2.0f * t_acc + t_constant;
    }
    
    return total_time;
}

/**
 * @brief 走行全体の所要時間を計算する
 * @param path 走行動作を格納したpath配列
 * @param path_size path配列のサイズ
 * @param params 走行パラメータ構造体
 * @return 走行全体の所要時間 [s]
 */
float calculate_total_run_time(const uint16_t *path, int path_size, const ShortestRunParam_t *params) {
    float total_time = 0.0f;
    
    for (int i = 0; i < path_size; i++) {
        // 動作の種類を取得（上位1桁を選択）
        int action_type = (path[i] / 100) * 100;
        // 回数を取得（下位2桁）
        int action_count = path[i] % 100;
        
        if (action_type == ACTION_TYPE_STRAIGHT) {
            // 直進
            // action_count はマス数
            total_time += calculate_straight_time(action_count, params->velocity_straight, params->acceleration_straight);
            
        } else if (action_type == ACTION_TYPE_SMALL_R || action_type == ACTION_TYPE_SMALL_L) {
            // 小回りターン（右と左で所要時間は同じ）
            for (int j = 0; j < action_count; j++) {
                // 小回りは90度ターンと同じと仮定
                total_time += calculate_turn_time(
                    params->angle_turn_90,
                    params->velocity_turn90,
                    params->alpha_turn90,
                    params->dist_offset_in,
                    params->dist_offset_out
                );
            }
            
        } else if (action_type == ACTION_TYPE_LARGE_R || action_type == ACTION_TYPE_LARGE_L) {
            // 大回りターン（右と左で所要時間は同じ）
            if (action_count == 1) {
                // 90度大回り
                total_time += calculate_turn_time(
                    params->angle_l_turn_90,
                    params->velocity_l_turn_90,
                    params->alpha_l_turn_90,
                    params->dist_offset_in,
                    params->dist_l_turn_out_90
                );
            } else if (action_count == 2) {
                // 180度大回り
                total_time += calculate_turn_time(
                    params->angle_l_turn_180,
                    params->velocity_l_turn_180,
                    params->alpha_l_turn_180,
                    params->dist_offset_in,
                    params->dist_l_turn_out_180
                );
            } else {
                // その他の値の場合は回数に応じた90度大回りを計算
                for (int j = 0; j < action_count; j++) {
                    total_time += calculate_turn_time(
                        params->angle_l_turn_90,
                        params->velocity_l_turn_90,
                        params->alpha_l_turn_90,
                        params->dist_offset_in,
                        params->dist_l_turn_out_90
                    );
                }
            }
            
        } else if (path[i] >= 701 && path[i] <= 704) {
            // 45度ターン
            // 簡易的に90度ターンの半分の時間と計算
            total_time += calculate_turn_time(
                params->angle_turn_90 / 2.0f, 
                params->velocity_turn90,
                params->alpha_turn90,
                params->dist_offset_in / 2.0f,
                params->dist_offset_out / 2.0f
            );
            
        } else if (path[i] == 0) {
            // 終了マーカー
            break;
        }
    }
    
    return total_time;
}

/**
 * @brief path配列の内容を解析して各動作の所要時間を表示する
 * @param path 走行動作を格納したpath配列
 * @param path_size path配列のサイズ
 * @param params 走行パラメータ構造体
 */
void print_path_times(const uint16_t *path, int path_size, const ShortestRunParam_t *params) {
    printf("\n==== 走行全体の所要時間計算 ====\n");
    printf("動作\t所要時間[s]\n");
    printf("----------------------------\n");
    
    float total_time = 0.0f;
    float action_time = 0.0f;
    char action_name[50];
    
    for (int i = 0; i < path_size; i++) {
        if (path[i] == 0) {
            break;  // 終了マーカー
        }
        
        // 動作の種類を取得（上位1桁を選択）
        int action_type = (path[i] / 100) * 100;
        // 回数を取得（下位2桁）
        int action_count = path[i] % 100;
        
        if (action_type == ACTION_TYPE_STRAIGHT) {
            // 直進
            action_time = calculate_straight_time(action_count, params->velocity_straight, params->acceleration_straight);
            sprintf(action_name, "直進(%dマス)", action_count);
            
        } else if (action_type == ACTION_TYPE_SMALL_R) {
            // 右小回りターン
            action_time = calculate_turn_time(
                params->angle_turn_90 * action_count,
                params->velocity_turn90,
                params->alpha_turn90,
                params->dist_offset_in,
                params->dist_offset_out
            );
            
            if (action_count == 1) {
                strcpy(action_name, "右小回り90度");
            } else {
                sprintf(action_name, "右小回り(%d回)", action_count);
            }
            
        } else if (action_type == ACTION_TYPE_SMALL_L) {
            // 左小回りターン
            action_time = calculate_turn_time(
                params->angle_turn_90 * action_count,
                params->velocity_turn90,
                params->alpha_turn90,
                params->dist_offset_in,
                params->dist_offset_out
            );
            
            if (action_count == 1) {
                strcpy(action_name, "左小回り90度");
            } else {
                sprintf(action_name, "左小回り(%d回)", action_count);
            }
            
        } else if (action_type == ACTION_TYPE_LARGE_R) {
            // 右大回りターン
            if (action_count == 1) {
                // 90度大回り
                action_time = calculate_turn_time(
                    params->angle_l_turn_90,
                    params->velocity_l_turn_90,
                    params->alpha_l_turn_90,
                    params->dist_offset_in,
                    params->dist_l_turn_out_90
                );
                strcpy(action_name, "右大回り90度");
            } else if (action_count == 2) {
                // 180度大回り
                action_time = calculate_turn_time(
                    params->angle_l_turn_180,
                    params->velocity_l_turn_180,
                    params->alpha_l_turn_180,
                    params->dist_offset_in,
                    params->dist_l_turn_out_180
                );
                strcpy(action_name, "右大回り180度");
            } else {
                // その他の値
                action_time = action_count * calculate_turn_time(
                    params->angle_l_turn_90,
                    params->velocity_l_turn_90,
                    params->alpha_l_turn_90,
                    params->dist_offset_in,
                    params->dist_l_turn_out_90
                );
                sprintf(action_name, "右大回り(%d回)", action_count);
            }
            
        } else if (action_type == ACTION_TYPE_LARGE_L) {
            // 左大回りターン
            if (action_count == 1) {
                // 90度大回り
                action_time = calculate_turn_time(
                    params->angle_l_turn_90,
                    params->velocity_l_turn_90,
                    params->alpha_l_turn_90,
                    params->dist_offset_in,
                    params->dist_l_turn_out_90
                );
                strcpy(action_name, "左大回り90度");
            } else if (action_count == 2) {
                // 180度大回り
                action_time = calculate_turn_time(
                    params->angle_l_turn_180,
                    params->velocity_l_turn_180,
                    params->alpha_l_turn_180,
                    params->dist_offset_in,
                    params->dist_l_turn_out_180
                );
                strcpy(action_name, "左大回り180度");
            } else {
                // その他の値
                action_time = action_count * calculate_turn_time(
                    params->angle_l_turn_90,
                    params->velocity_l_turn_90,
                    params->alpha_l_turn_90,
                    params->dist_offset_in,
                    params->dist_l_turn_out_90
                );
                sprintf(action_name, "左大回り(%d回)", action_count);
            }
            
        } else if (path[i] == ACTION_TURN_R45_IN || path[i] == ACTION_TURN_L45_IN ||
                  path[i] == ACTION_TURN_R45_OUT || path[i] == ACTION_TURN_L45_OUT) {
            // 45度ターン
            action_time = calculate_turn_time(
                params->angle_turn_90 / 2.0f, 
                params->velocity_turn90,
                params->alpha_turn90,
                params->dist_offset_in / 2.0f,
                params->dist_offset_out / 2.0f
            );
            
            if (path[i] == ACTION_TURN_R45_IN) strcpy(action_name, "右45度ターン入り");
            else if (path[i] == ACTION_TURN_L45_IN) strcpy(action_name, "左45度ターン入り");
            else if (path[i] == ACTION_TURN_R45_OUT) strcpy(action_name, "右45度ターン出");
            else strcpy(action_name, "左45度ターン出");
        } else {
            // 不明な動作
            sprintf(action_name, "不明な動作(code:%d)", path[i]);
            action_time = 0.1f; // 仮の値
        }
        
        printf("%s\t%.5f\n", action_name, action_time);
        total_time += action_time;
    }
    
    printf("----------------------------\n");
    printf("合計所要時間: %.5f 秒\n", total_time);
}
