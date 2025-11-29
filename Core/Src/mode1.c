/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "sensor_distance.h"
#include "../Inc/logging.h"

// 探索走行（mode1 標準）の基本パラメータを適用
static void apply_explore_params_mode1_basic(void)
{
    // 直線
    acceleration_straight = 1000;
    acceleration_straight_dash = 0; // run()は使わず one_sectionU 系で走るため0でOK
    // ターン
    velocity_turn90 = 300;
    alpha_turn90 = 8850;
    acceleration_turn = 0;
    dist_offset_in = 10;   // 8
    dist_offset_out = 16.5; // 15.5
    val_offset_in = 1750;
    angle_turn_90 = 89.5;
    // 壁切れ後の距離
    dist_wall_end = 0;

    // 壁制御とケツ当て
    kp_wall = 0.015f; // 探索時の既定
    duty_setposition = 40;

    // 壁判断しきい値の係数
    sensor_kx = 1.0f;

    MF.FLAG.WALL_ALIGN = 0;
}

// 探索パラメータでの直進テスト＋ログ出力（距離/速度プロファイルを選択）
static void straight_test_explore_params(LogProfile profile)
{
    // パラメータを探索仕様に設定し、テストでは壁制御は切る
    apply_explore_params_mode1_basic();
    kp_wall = 0.0f; // mode2 case8 と同様、テスト時は壁制御を無効化

    // 走行準備
    led_flash(4);
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();
    get_base();

    // ログ開始
    log_init();
    log_set_profile(profile);
    log_start(HAL_GetTick());

    // 直進シナリオ: 初期 half_sectionA + S3 + 最後 half_sectionD

    speed_now = 0;
    half_sectionA(0);
    one_sectionU(0);
    one_sectionU(0);
    one_sectionU(0);
    half_sectionD(0);

    // ログ停止
    log_stop();

    // CSV出力の選択（FR=速度, FL=距離）
    printf("[mode1 straight-test] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
    while (1) {
        if (ad_fr > WALL_BASE_FR) {
            log_print_velocity_all();
            break;
        } else if (ad_fl > WALL_BASE_FL) {
            log_print_distance_all();
            break;
        }
        HAL_Delay(50);
    }

    led_flash(3);
}

//============================================================
// 前壁追従（match_position連続実行）テスト
//  with_print!=0 でFR/FLのAD値と距離[mm]を定期表示
//  PUSHボタン押下で終了
//============================================================
static void front_follow_continuous(int with_print)
{
    printf("[FrontFollow] match_position continuous. Press PUSH to exit.\n");
    led_flash(3);

    // 側壁制御を無効化して前壁のみで合わせる
    MF.FLAG.CTRL = 0;
    kp_wall = 0.0f;

    // 走行開始準備
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();
    drive_start();

    uint32_t last_print = HAL_GetTick();

    while (1) {
        // 抜け条件：PUSHボタン
        if (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) {
            buzzer_enter(900);
            break;
        }

        // 前壁が見えているときに位置合わせを実行
        if (ad_fr > F_ALIGN_DETECT_THR && ad_fl > F_ALIGN_DETECT_THR) {
            match_position(0);
        } else {
            // 待機（見えていない間は停止）
            velocity_interrupt = 0;
            omega_interrupt = 0;
            HAL_Delay(50);
        }

        // 任意の表示
        if (with_print) {
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 200) {
                float d_fr = sensor_distance_from_fr(ad_fr);
                float d_fl = sensor_distance_from_fl(ad_fl);
                printf("FR=%u (%.1fmm), FL=%u (%.1fmm)\n",
                       (unsigned)ad_fr, d_fr, (unsigned)ad_fl, d_fl);
                last_print = now;
            }
        }

        HAL_Delay(10);
    }

    // 停止処理
    velocity_interrupt = 0;
    omega_interrupt = 0;
    drive_variable_reset();
    drive_stop();
    led_flash(2);
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // テストモード（mode2同様にサブ選択）

            printf("Mode 1-0 Test (sub 0..3).\n");
            printf("  sub0: Front wall follow (continuous)\n");
            printf("  sub1: Front wall follow + print (continuous)\n");
            printf("  sub2: Straight test (explore params) + DISTANCE log\n");
            printf("  sub3: Straight test (explore params) + VELOCITY log\n");

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            switch (sub) {
            case 0:
                front_follow_continuous(0);
                break;
            case 1:
                front_follow_continuous(1);
                break;
            case 2:
                straight_test_explore_params(LOG_PROFILE_DISTANCE);
                break;
            case 3:
                straight_test_explore_params(LOG_PROFILE_VELOCITY);
                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            break;
        }

        case 8: { // 直進テスト or 足立法（ゴール到達で終了）

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.12;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            velocity_interrupt = 0;

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
        }

        case 1: { // 

            printf("Mode 1-1.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.12;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 1;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;
        }

        case 2: // 足立法全面探索 300mm/s

            printf("Mode 1-2.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 3: // 足立法全面探索 300mm/s しきい値高め

            printf("Mode 1-3.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.1;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 4: // // 足立法全面探索 300mm/s  しきい値低め
            printf("Mode 1-4.\n");

            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 0.9;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();

            adachi();

            led_wait();

            break;

        case 5: // 吸引探索 600mm/s

            printf("Mode 1-5.\n");

            MF.FLAG.RUNNING = 1;

            // 直線
            acceleration_straight = 4000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            // 壁判断しきい値の係数
            sensor_kx = 1.0;

            MF.FLAG.WALL_ALIGN = 0;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_fan(300);
            led_flash(3);

            drive_start();

            adachi();

            drive_fan(0);

            led_wait();

            break;

        case 6: // まずゴール探索→保存→全面探索（300mm/s）

            printf("Mode 1-6 (Goal->Save->Full Explore).\n");

            // ===== 走行パラメータ（case 2 と同一） =====
            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

            // 一旦マップ保存
            store_map_in_eeprom();

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            // フル探索に切り替えた直後の「最初の停止での保存」を1回抑制
            g_suppress_first_stop_save = true;
            search_end = false;
            adachi();

            led_wait();

            break;

        case 7: // ゴール探索→保存→スタートへ復帰（300mm/s）

            printf("Mode 1-7 (Goal->Save->Return to Start).\n");

            // ===== 走行パラメータ（case 2 と同一） =====
            // 直線
            acceleration_straight = 1000;
            acceleration_straight_dash = 0; // 5000
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 8850;
            acceleration_turn = 0;
            dist_offset_in = 10;   // 8
            dist_offset_out = 16.5; // 15.5
            val_offset_in = 1750;
            angle_turn_90 = 89.5;
            // 壁切れ後の距離
            dist_wall_end = 0;

            // 壁制御とケツ当て
            kp_wall = 0.015;
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

            // ゴール到達後に一度だけ安全に保存（Uターン時に保存済みなら二重保存を避ける）
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1; // 以後の自動保存を抑制
                }
            }

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
        }
    }
}
