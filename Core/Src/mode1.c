/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/search_run_params.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"
#include "../Inc/sensor_distance.h"
#include <math.h>

// 探索パラメータの簡易調整用オーバーライド（未設定は NaN）
static float s_override_velocity_straight = NAN;
static float s_override_accel_straight = NAN;
static float s_override_kp_wall = NAN;
static float s_override_dist_wall_end = NAN;

// （mode4相当のヘルパは不要のため削除）

// === Front sensor 3-point calibration helpers ===
static void wait_sensor_enter_and_settle(uint32_t settle_ms)
{
    // センサEnter: 前方センサを一時的に近づけて ad_fr/ad_fl のどちらかが閾値超え
    while (1) {
        if (ad_fr > 1500 || ad_fl > 1500) break;
        HAL_Delay(50);
    }
    // 手を離して落ち着くのを待つ
    HAL_Delay(settle_ms);
}

static void sample_front_ad_avg_ms(uint32_t duration_ms, uint16_t *out_fl, uint16_t *out_fr)
{
    uint32_t start = HAL_GetTick();
    uint32_t sum_fl = 0, sum_fr = 0;
    uint32_t cnt = 0;
    while ((HAL_GetTick() - start) < duration_ms) {
        sum_fl += (uint16_t)ad_fl;
        sum_fr += (uint16_t)ad_fr;
        cnt++;
        HAL_Delay(2);
    }
    if (cnt == 0) cnt = 1;
    if (out_fl) *out_fl = (uint16_t)(sum_fl / cnt);
    if (out_fr) *out_fr = (uint16_t)(sum_fr / cnt);
}

static void enforce_strict_decreasing(uint16_t *a0, uint16_t *a1, uint16_t *a2)
{
    // validate_lut は厳密単調減少を要求。ノイズで逆転した場合は最小限の補正を入れる。
    if (*a1 >= *a0) *a1 = (*a0 > 0) ? (uint16_t)(*a0 - 1) : 0;
    if (*a2 >= *a1) *a2 = (*a1 > 0) ? (uint16_t)(*a1 - 1) : 0;
}

// Helper loader: apply exploration common params
static void apply_search_run_params(void) {
    const SearchRunParams_t *p = &searchRunParams;
    // 直線
    acceleration_straight = p->acceleration_straight;
    acceleration_straight_dash = p->acceleration_straight_dash;
    velocity_straight = p->velocity_straight;
    // 壁制御
    kp_wall = p->kp_wall;
    // ターン・オフセット・角度
    velocity_turn90 = p->velocity_turn90;
    alpha_turn90 = p->alpha_turn90;
    acceleration_turn = p->acceleration_turn;
    dist_offset_in = p->dist_offset_in;
    dist_offset_out = p->dist_offset_out;
    angle_turn_90 = p->angle_turn_90;
    // 壁切れ後追従
    dist_wall_end = p->dist_wall_end;

    // オーバーライドを適用
    if (!isnan(s_override_velocity_straight)) {
        if (s_override_velocity_straight < 100.0f) s_override_velocity_straight = 100.0f;
        if (s_override_velocity_straight > 3000.0f) s_override_velocity_straight = 3000.0f;
        velocity_straight = s_override_velocity_straight;
    }
    if (!isnan(s_override_accel_straight)) {
        if (s_override_accel_straight < 200.0f) s_override_accel_straight = 200.0f;
        if (s_override_accel_straight > 20000.0f) s_override_accel_straight = 20000.0f;
        acceleration_straight = s_override_accel_straight;
    }
    if (!isnan(s_override_kp_wall)) {
        if (s_override_kp_wall < 0.0f) s_override_kp_wall = 0.0f;
        if (s_override_kp_wall > 0.5f) s_override_kp_wall = 0.5f;
        kp_wall = s_override_kp_wall;
    }
    if (!isnan(s_override_dist_wall_end)) {
        if (s_override_dist_wall_end < 0.0f) s_override_dist_wall_end = 0.0f;
        if (s_override_dist_wall_end > 60.0f) s_override_dist_wall_end = 60.0f;
        dist_wall_end = s_override_dist_wall_end;
    }
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 調整サブモード: サブcase1で通常ターンのみ（探索用共通パラメータ）

            printf("Mode 1-0 Turn Tuning (search params): sub=1 only.\n");
            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            switch (sub) {
            case 0:
                // 初期区画 + 1区画の距離チェック（探索走行用パラメータ）
                printf("Sub 0: First + 1 section distance check (search params).\n");

                // 探索用の共通パラメータを適用
                apply_search_run_params();

                // 準備
                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(2);

                // 基準値の取得（壁制御基準ベース）
                get_base();

                // 実行: 初期区画 + 1区画
                first_sectionA();   // 初期加速区間（DIST_FIRST_SEC）
                half_sectionU();    // 半区画 等速前進
                half_sectionD(0);   // 半区画 減速停止

                drive_stop();
                led_flash(5);
                break;
            case 1: // 通常ターン 90deg（探索用共通パラメータ）
                // 探索用の共通パラメータを適用
                apply_search_run_params();
                // 調整モードでは壁制御を無効化
                kp_wall = 0.0f;

                velocity_interrupt = 0;

                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();

                led_flash(5);

                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());

                half_sectionA(velocity_turn90);
                turn_R90(0);
                half_sectionD(0);

                log_stop();
                led_flash(5);
                drive_stop();

                // センサEnter待ち（右前:角速度ログ, 左前:角度ログ）
                while (1) {
                    if (ad_fr > 1500) {
                        log_print_omega_all();
                        break;
                    } else if (ad_fl > 1500) {
                        log_print_angle_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            case 2: // 前壁補正テスト（R90, search params, LOG_PROFILE_CUSTOM）
                // 探索用の共通パラメータを適用
                apply_search_run_params();

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(2);

                // 基準値（壁制御用ベース）の取得
                get_base();

                // ログ設定（前壁補正テスト）
                log_init();
                log_set_profile(LOG_PROFILE_CUSTOM);
                log_start(HAL_GetTick());

                // 接近→前壁補正→R90ターン→減速
                half_sectionA(velocity_turn90);
                one_sectionU(0);
                turn_R90(1);
                half_sectionD(0);

                // ログ停止（出力は後でセンサEnterで実行）
                log_stop();

                // 走行停止・LED通知
                drive_stop();
                led_flash(5);

                // センサEnter待ち（右前/左前でCSV出力）
                while (1) {
                    if (ad_fr > 700 && ad_fl < 200) {
                        log_print_frontwall_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            case 3: // 前壁補正テスト（L90, search params, LOG_PROFILE_CUSTOM）
                // 探索用の共通パラメータを適用
                apply_search_run_params();

                velocity_interrupt = 0;
                led_flash(10);
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                led_flash(2);

                // 基準値（壁制御用ベース）の取得
                get_base();

                // ログ設定（前壁補正テスト）
                log_init();
                log_set_profile(LOG_PROFILE_CUSTOM);
                log_start(HAL_GetTick());

                // 接近→前壁補正→L90ターン→減速
                half_sectionA(velocity_turn90);
                turn_L90(1);
                half_sectionD(0);

                // ログ停止（出力は後でセンサEnterで実行）
                log_stop();

                // 走行停止・LED通知
                drive_stop();
                led_flash(5);

                // センサEnter待ち（右前/左前でCSV出力）
                while (1) {
                    if (ad_fr > 1500 || ad_fl > 1500) {
                        log_print_frontwall_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                printf("(empty)\n");
                break;
            case 9: { // 前壁距離 3点キャリブレーション（0,24,114mm）
                printf("Mode 1-0-9 Front Distance 3-point Calibration (0,24,114mm)\n");
                printf("Place mouse at target distance, then do sensor ENTER (wave hand close to front) to capture.\n");

                // 安全のためモータ停止
                drive_stop();
                drive_disable_motor();

                uint16_t fl0=0, fr0=0, fl24=0, fr24=0, fl114=0, fr114=0;

                // 0mm
                printf("[Step 1/3] 0 mm: place and ENTER...\n");
                wait_sensor_enter_and_settle(800);
                led_flash(50);
                sample_front_ad_avg_ms(600, &fl0, &fr0);
                printf("  captured: FL=%u, FR=%u\n", (unsigned)fl0, (unsigned)fr0);
                buzzer_enter(300);

                // 24mm
                printf("[Step 2/3] 24 mm: place and ENTER...\n");
                wait_sensor_enter_and_settle(800);
                led_flash(50);
                sample_front_ad_avg_ms(600, &fl24, &fr24);
                printf("  captured: FL=%u, FR=%u\n", (unsigned)fl24, (unsigned)fr24);
                buzzer_enter(300);

                // 114mm
                printf("[Step 3/3] 114 mm: place and ENTER...\n");
                wait_sensor_enter_and_settle(800);
                led_flash(50);
                sample_front_ad_avg_ms(600, &fl114, &fr114);
                printf("  captured: FL=%u, FR=%u\n", (unsigned)fl114, (unsigned)fr114);
                buzzer_enter(300);

                // 単調減少の担保（ノイズ時の最小補正）
                enforce_strict_decreasing(&fl0, &fl24, &fl114);
                enforce_strict_decreasing(&fr0, &fr24, &fr114);

                // 既存LUTは変えず、距離ワープを適用する。
                // 1) 測定ADの合計から、現行LUTでの推定距離 mm_est を求める
                uint16_t ad_sum0 = (uint16_t)((uint32_t)fl0 + (uint32_t)fr0);
                uint16_t ad_sum24 = (uint16_t)((uint32_t)fl24 + (uint32_t)fr24);
                uint16_t ad_sum114 = (uint16_t)((uint32_t)fl114 + (uint32_t)fr114);
                float x_est[3];
                x_est[0] = sensor_distance_from_fsum_unwarped(ad_sum0);
                x_est[1] = sensor_distance_from_fsum_unwarped(ad_sum24);
                x_est[2] = sensor_distance_from_fsum_unwarped(ad_sum114);

                // 2) 真値（ターゲット）を用意
                float y_true[3] = {0.0f, 24.0f, 114.0f};

                // 3) ワープを適用（RAM）し、Flashにも保存
                sensor_distance_set_warp_front_sum_3pt(x_est, y_true);
                HAL_StatusTypeDef st = sensor_front_warp_save_to_flash(x_est, y_true);

                printf("Applied distance warp (mm_est -> mm_true):\n");
                printf("  (%.2f -> %.2f), (%.2f -> %.2f), (%.2f -> %.2f)\n",
                       x_est[0], y_true[0], x_est[1], y_true[1], x_est[2], y_true[2]);
                if (st == HAL_OK) {
                    printf("Saved warp to Flash.\n");
                } else {
                    printf("Flash save failed (HAL=%d).\n", st);
                }

                // 終了合図
                for(int i=0; i<3; i++) {
                    buzzer_enter(300);
                    HAL_Delay(300);
                }
                break;
            }
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            break;
        }

        case 1: // 探索: ゴールを目指し、到達で終了

            printf("Mode 1-1 (Explore: Goal Stop).\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            // ケース1の壁切れ補正 有効/無効設定
            g_enable_wall_end_search = WALL_END_ENABLE_SEARCH_CASE1;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // 準備
            led_flash(10);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);

            adachi();

            led_wait();

            break;

        case 2: // 探索: ゴールを目指し、到達後にスタートへ戻る（サブ選択対応: 0=従来,1/2=前壁補正テスト）

            printf("Mode 1-2 (Explore: Goal -> Return to Start).\n");

            // サブ選択: 0=従来動作, 1=前壁補正テスト(R90), 2=前壁補正テスト(L90)
            {
                int sub2 = 0;
                sub2 = select_mode(sub2);
                if (sub2 == 1 || sub2 == 2) {
                    // ===== 前壁補正テスト（探索環境） =====
                    apply_search_run_params();

                    // 壁切れ補正は本テストでは不問（通常どおり）
                    // g_enable_wall_end_search は変更しない

                    // 準備
                    velocity_interrupt = 0;
                    led_flash(10);
                    drive_variable_reset();
                    IMU_GetOffset();
                    drive_enable_motor();
                    led_flash(2);

                    // 基準値の取得
                    get_base();

                    // ログ設定（前壁補正テスト）
                    log_init();
                    log_set_profile(LOG_PROFILE_CUSTOM);
                    log_start(HAL_GetTick());

                    // 接近→前壁補正→ターン→減速
                    half_sectionA(velocity_turn90);
                    if (sub2 == 1) {
                        turn_R90(1);
                    } else {
                        turn_L90(1);
                    }
                    half_sectionD(0);

                    // ログ停止（出力は後でセンサEnterで実行）
                    log_stop();

                    // 走行停止・LED通知
                    drive_stop();
                    led_flash(5);

                    // センサEnter待ち（右前/左前でCSV出力）
                    while (1) {
                        if (ad_fr > 1500 || ad_fl > 1500) {
                            log_print_frontwall_all();
                            break;
                        }
                        HAL_Delay(50);
                    }

                    break; // case 2 を終了
                }
            }

            // ===== 走行パラメータ（探索共通パラメータを適用） =====
            apply_search_run_params();
            // ケース2の壁切れ補正 有効/無効設定
            g_enable_wall_end_search = WALL_END_ENABLE_SEARCH_CASE2;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // ===== 事前準備 =====
            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false; // ゴールセルを到達判定
            goal_x = GOAL_X; goal_y = GOAL_Y; // 念のため明示
            search_end = false;
            adachi();

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0; // 復路ではゴール判定フラグに依存しない
            g_goal_is_start = true; // スタート座標を到達判定に使用
            goal_x = START_X; goal_y = START_Y; // 経路導出もスタートへ
            search_end = false;
            adachi();

            // 後処理
            g_goal_is_start = false; // 後続モードへの影響を避ける

            led_wait();

            break;

        case 3: // 探索: ゴール到達後に全面探索へ切り替え

            printf("Mode 1-3 (Explore: Goal -> Full Explore).\n");

            // ===== 走行パラメータ（探索共通パラメータを適用） =====
            apply_search_run_params();
            // ケース3の壁切れ補正 有効/無効設定
            g_enable_wall_end_search = WALL_END_ENABLE_SEARCH_CASE3;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            // ===== 事前準備 =====
            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true; // 切替直後の最初の停止保存を1回スキップ
            search_end = false;
            adachi();

            led_wait();

            break;

        case 4: // 探索: 最初から全面探索

            printf("Mode 1-4 (Explore: Full).\n");

            // 探索用の共通パラメータを適用
            apply_search_run_params();
            // ケース4の壁切れ補正 有効/無効設定
            g_enable_wall_end_search = WALL_END_ENABLE_SEARCH_CASE4;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 1;

            velocity_interrupt = 0;

            // 準備
            led_flash(10);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            // 全面探索モード
            set_search_mode(SEARCH_MODE_FULL);
            search_end = true;

            adachi();

            led_wait();

            break;

        case 5:
            printf("Mode 1-5: (empty)\n");
            break;

        case 6:
            printf("Mode 1-6: (empty)\n");
            break;

        case 7:
            printf("Mode 1-7: (empty)\n");
            break;

        case 8:
            printf("Mode 1-8: (empty)\n");
            break;

        case 9:
            printf("Mode 1-9: (empty)\n");
            break;
        }
    }
}
