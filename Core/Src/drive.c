/*
 * drive.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "main.h"
#include "params.h"
#include "drive.h"
#include "sensor.h"
#include "sensor_distance.h"
#include "interrupt.h"
#include "logging.h"
#include "search_run_params.h"
#include <math.h>

// 半区画距離を返すヘルパ
// 探索走行中(g_is_search_run=true)は探索用の半区画長、その他は通常の半区画長を返す
static inline float HALF_MM(void) {
    return g_is_search_run ? (float)DIST_HALF_SEC_SEARCH : (float)DIST_HALF_SEC;
}

// ==== Motor control params (sign-magnitude) ====
// 周波数は TIM 設定に従う（本実装では変更しない）
// ARR はランタイムで読み出して使用する（__HAL_TIM_GET_AUTORELOAD）
#ifndef PWM_INVERT_DIR_LEVEL
#define PWM_INVERT_DIR_LEVEL 1  // DIR がこのレベルのとき PWM を反転（0:Lowで反転, 1:Highで反転）
#endif
#ifndef MOTOR_MIN_DUTY_PCT
#define MOTOR_MIN_DUTY_PCT 0    // デッドゾーン除去用の最低デューティ[%]
#endif
#ifndef MOTOR_BOOST_DUTY_PCT
#define MOTOR_BOOST_DUTY_PCT 0  // 起動ブーストのデューティ[%]
#endif
#ifndef MOTOR_BOOST_TIME_MS
#define MOTOR_BOOST_TIME_MS 0   // 起動ブースト時間[ms]
#endif

// 起動検出・ブースト管理用のローカル状態
static uint16_t s_prev_cmd_l = 0;
static uint16_t s_prev_cmd_r = 0;
static uint32_t s_boost_until_ms_l = 0;
static uint32_t s_boost_until_ms_r = 0;

// 現在のDIRピンレベル（High/Low）を保持
static uint8_t s_dir_pin_high_l = 0; // 0:Low, 1:High
static uint8_t s_dir_pin_high_r = 0; // 0:Low, 1:High

// CCR更新抑止フラグ：有効化/無効化シーケンス中の微小回転を防止
static volatile uint8_t s_outputs_locked = 0;

// モータドライバの有効/無効状態（STBY+PWM）
static volatile uint8_t s_motor_enabled = 0;

// 最後にファンを停止した時刻（ms）: ブザー起動のクールダウン制御用
volatile uint32_t fan_last_off_ms = 0;

/*==========================================================
    走行系 上位関数
==========================================================*/
/*
    マウスフラグ(MF)
      6Bit:デフォルトインターバルフラグ
      5Bit:減速フラグ
      4Bit:加速フラグ
      3Bit:制御フラグ
      1Bit:二次走行フラグ
*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// first_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void first_sectionA(void) {
    float speed_out;

    speed_out =
        sqrt(speed_now * speed_now + 2 * acceleration_straight * HALF_MM());

    MF.FLAG.CTRL = 1;
    driveA(DIST_FIRST_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && !val) {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight_dash * HALF_MM());
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * HALF_MM());
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(HALF_MM(), speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
    // ロック中にDIRが変わった場合でも、IN1==IN2 を維持
    if (s_outputs_locked) {
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionAD
// 斜め半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionAD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && !val) {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * DIST_D_HALF_SEC);
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(DIST_D_HALF_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight_dash * HALF_MM());
        } else {
            speed_out = 0;
        }
    }

    MF.FLAG.CTRL = 1;

    driveA(HALF_MM(), speed_now, speed_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (!val) {
        velocity_interrupt = 0;
    }

    // get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionDD
// 斜め半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionDD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else {
            speed_out = 0;
        }
    }

    MF.FLAG.CTRL = 1;

    driveA(DIST_D_HALF_SEC, speed_now, speed_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (!val) {
        velocity_interrupt = 0;
    }

    // get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionA
// 1区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void) {
    float speed_out;
    if (MF.FLAG.SCND || known_straight) {
        speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight_dash * HALF_MM() * 2);
    } else {
        speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight * HALF_MM() * 2);
    }

    MF.FLAG.CTRL = 1;
    // kp_wall = kp_wall * 2;
    driveA(HALF_MM() * 2, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    // kp_wall = kp_wall * 1 / 2;
    speed_now = speed_out;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionD
//  1区間減速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void) {
    // 既知区間からの減速 1区画。2mm刻みで一定減速度を保ちつつ壁切れ検知を監視する。
    float v0 = speed_now;
    // 目標終端速度（従来式）
    float accel_lin = (MF.FLAG.SCND || acceled) ? acceleration_straight_dash : acceleration_straight; // [mm/s^2]
    float speed_out = sqrtf(fmaxf(0.0f, v0 * v0 - 2.0f * accel_lin * (HALF_MM() * 2.0f)));

    MF.FLAG.CTRL = 1;

    if (ad_fl > WALL_BASE_FL || ad_fr > WALL_BASE_FR) {
        MF.FLAG.F_WALL_STOP = 1;
    }

    // 全距離に対する一定減速度 a_const を算出（負）
    const float total_mm = (HALF_MM() * 2.0f);
    const float a_const = (speed_out * speed_out - v0 * v0) / (2.0f * total_mm); // 期待上は -accel_lin

    float remaining_blocks = 2.0f; // 1区画
    const float step_blocks = (2.0f / HALF_MM()); // 2mm相当
    const float step_mm = step_blocks * HALF_MM();
    bool triggered = false;

    // 壁切れ補正が無効な場合は、検出をアームせずにそのまま一定減速度で1区画を完了
    if (!g_enable_wall_end_search) {
        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            float v_in = speed_now; // run_straight が更新していく現在速度
            float v2 = v_in * v_in + 2.0f * a_const * (step * HALF_MM());
            if (v2 < 0.0f) v2 = 0.0f;
            float v_next = sqrtf(v2);
            run_straight(step, v_next, 0);
            remaining_blocks -= step;
        }

        MF.FLAG.F_WALL_STOP = 0;
        MF.FLAG.CTRL = 0;
        get_wall_info();
        return;
    }

    // 壁切れ検知をアーム（探索でも検出できるようSCNDを一時有効化）
    uint8_t prev_scnd = MF.FLAG.SCND;
    MF.FLAG.R_WALL_END = 0;
    MF.FLAG.L_WALL_END = 0;
    MF.FLAG.WALL_END   = 1;
    MF.FLAG.SCND       = 1;

    while (remaining_blocks > 0.0f) {
        float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
        // 次ステップの目標速度（一定減速度）
        float v_in = speed_now; // run_straight が更新していく現在速度
        float v2 = v_in * v_in + 2.0f * a_const * (step * HALF_MM());
        if (v2 < 0.0f) v2 = 0.0f;
        float v_next = sqrtf(v2);

        run_straight(step, v_next, 0);

        if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
            // 消費（クリア）
            MF.FLAG.R_WALL_END = 0;
            MF.FLAG.L_WALL_END = 0;
            triggered = true;
            break;
        }
        remaining_blocks -= step;
    }

    // 検出アーム解除とSCND復帰
    MF.FLAG.WALL_END = 0;
    MF.FLAG.SCND = prev_scnd;

    if (triggered) {
        // 検出後は dist_wall_end（合計距離）でターン速度(velocity_turn90)まで減速する
        float v_init = speed_now;                 // 検出直後の現在速度
        float v_target = velocity_turn90;         // 探索の小回りターン入口速度
        if (v_target < 0.0f) v_target = 0.0f;     // 念のためクランプ
        if (v_target > v_init) v_target = v_init; // 減速のみ（加速はしない）
        float follow_mm = dist_wall_end;
        if (follow_mm > 0.0f) {
            // 一定減速度 a_follow を算出（v_target^2 = v_init^2 + 2*a*follow_mm）
            float a_follow = (v_target * v_target - v_init * v_init) / (2.0f * follow_mm);

            float remaining_follow_blocks = follow_mm / HALF_MM();
            const float step_blocks_f = (2.0f / HALF_MM()); // 2mm

            while (remaining_follow_blocks > 0.0f) {
                float step_b = (remaining_follow_blocks < step_blocks_f) ? remaining_follow_blocks : step_blocks_f;
                float step_mm_f = step_b * HALF_MM();
                float v_in = speed_now;
                // v_out^2 = v_in^2 + 2*a*ds
                float v2 = v_in * v_in + 2.0f * a_follow * step_mm_f;
                if (v2 < 0.0f) v2 = 0.0f;
                float v_out = sqrtf(v2);
                // 最終ステップはターゲットに合わせ込み
                if (step_b == remaining_follow_blocks) {
                    v_out = v_target;
                }
                run_straight(step_b, v_out, 0);
                remaining_follow_blocks -= step_b;
            }
        }
    } else {
        // 未検知: 残り距離があれば一定減速度で完走（理論上 remaining_blocks は0）
        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            float v_in = speed_now;
            float v2 = v_in * v_in + 2.0f * a_const * (step * HALF_MM());
            if (v2 < 0.0f) v2 = 0.0f;
            float v_next = sqrtf(v2);
            run_straight(step, v_next, 0);
            remaining_blocks -= step;
        }
    }

    MF.FLAG.F_WALL_STOP = 0;

    MF.FLAG.CTRL = 0;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_section
// 1区画分進んで停止する。1区画走行用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void) {}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(uint8_t CTRL) {
    (void)CTRL;
    // 壁制御ONのまま、TEST_MODE では壁切れ補正なしで固定距離を走行
    MF.FLAG.CTRL = 1;

    const float v_const = speed_now; // 等速維持

    // 壁切れ補正が無効な場合、またはテストモードでは補正を行わず等速で1区画を走行
    if (!g_enable_wall_end_search || MF.FLAG.TEST_MODE) {
        run_straight_const(2.0f, v_const);
        MF.FLAG.CTRL = 0;
        speed_now = v_const;
        get_wall_info();
        return;
    }

    // 通常動作: 1区画を2mm刻みで走行しながら壁切れ検知を監視
    // 壁切れ検知のアーム（探索中でも検出できるようにSCNDを一時的に有効化）
    uint8_t prev_scnd = MF.FLAG.SCND;
    MF.FLAG.R_WALL_END = 0;
    MF.FLAG.L_WALL_END = 0;
    MF.FLAG.WALL_END   = 1;
    MF.FLAG.SCND       = 1; // detect_wall_end() のゲート条件を満たすため一時的に有効化

    // 1区画 = 半区画2つ分 = blocksで2.0
    float remaining_blocks = 2.0f;
    const float step_blocks = (2.0f / HALF_MM()); // 2mm相当で刻む
    bool triggered = false;

    while (remaining_blocks > 0.0f) {
        float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
        run_straight(step, v_const, 0);
        if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
            // 消費（クリア）
            MF.FLAG.R_WALL_END = 0;
            MF.FLAG.L_WALL_END = 0;
            triggered = true;
            break;
        }
        remaining_blocks -= step;
    }

    // 検出アーム解除とSCND復帰
    MF.FLAG.WALL_END = 0;
    MF.FLAG.SCND = prev_scnd;

    if (triggered) {
        // 検出後は dist_wall_end（合計距離）を等速で追従
        float follow_mm = dist_wall_end;
        if (follow_mm > 0.0f) {
            float extra_blocks = follow_mm / HALF_MM();
            run_straight(extra_blocks, v_const, 0);
        }
    } else {
        // 未検知の場合、残りがあれば従来通り1区画分を完了
        if (remaining_blocks > 0.0f) {
            run_straight(remaining_blocks, v_const, 0);
        }
    }

    MF.FLAG.CTRL = 0;
    speed_now = v_const;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionU(void) {
    MF.FLAG.CTRL = 1;
    driveA(HALF_MM(), speed_now, speed_now, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_now;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_straight
// 指定区画を指定速度で走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_straight(float section, float spd_out, float dist_wallend) {
    (void)dist_wallend;

    MF.FLAG.CTRL = 1;
    driveA(HALF_MM() * section, speed_now, spd_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_straight_const
// 上限速度を v_const に固定して指定区画を走る（等速保持用）
// 引数：section …… 半区画単位の距離（2.0=1区画）
//       v_const …… 目標等速[mm/s]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_straight_const(float section, float v_const) {
    // クランプ（安全）
    if (v_const < 0.0f) v_const = 0.0f;
    if (v_const > velocity_straight) v_const = velocity_straight;

    const float dist = HALF_MM() * section; // [mm]

    // 参照リセット＋現在速度を初期値として開始
    target_distance = 0.0f;
    // velocity_interrupt は calculate_translation 側で更新する

    // 角度側は本区間中は使わないためゼロ化（互換）
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 実距離カウンタをゼロ化
    real_distance = 0.0f;
    encoder_distance_r = 0.0f;
    encoder_distance_l = 0.0f;

    // sベースプロファイル（上限＝v_const、終端＝v_const）
    {
        float a_base = (MF.FLAG.SCND || known_straight) ? acceleration_straight_dash : acceleration_straight;
        if (a_base < 0.0f) a_base = -a_base;
        profile_active  = 1;
        profile_s_end   = dist;
        profile_v_out   = v_const;
        profile_v_max   = v_const; // ここがポイント：保持中に上限を超えない
        profile_a_accel = a_base;
        profile_a_decel = a_base;
    }

    MF.FLAG.CTRL = 1;
    drive_start();

    // 実距離が規定値に達するまで制御ISRを回す
    while (real_distance < dist) {
        background_replan_tick();
    }

    // 終了処理（プロファイル/参照のリセット）
    drive_variable_reset();
    real_distance = 0.0f;
    encoder_distance_r = 0.0f;
    encoder_distance_l = 0.0f;
    profile_active = 0;

    MF.FLAG.CTRL = 0;
    speed_now = v_const;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_trapezoid_distance_mm
// 台形（または三角）加減速プロファイルで直線 total_mm を走行する
// 加速は acceleration_straight_dash、最高速度は v_max（上限 velocity_straight）
// 終端速度は 0[mm/s]
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_trapezoid_distance_mm(float total_mm, float v_max) {
    if (total_mm <= 0.0f) return;

    // 使用パラメータ
    float a_cfg = (acceleration_straight_dash > 0.0f) ? acceleration_straight_dash : acceleration_straight;
    if (a_cfg <= 0.0f) {
        // フォールバック（安全）
        a_cfg = 1000.0f;
    }
    float v_limit = (v_max > 0.0f) ? v_max : velocity_straight;
    if (v_limit > velocity_straight) v_limit = velocity_straight;
    if (v_limit < 0.0f) v_limit = 0.0f;

    // 開始速度（通常0）
    float v0 = speed_now;
    if (v0 < 0.0f) v0 = 0.0f;
    if (v0 > v_limit) v0 = v_limit;

    // 対称プロファイル（終端速度0）を想定
    // v0→v_peak まで加速、v_peak→0 まで減速。
    // 加速距離 d_acc は a_cfg, v0, v_peak から派生。まず v_peak=v_limit を仮置きしてチェック。
    float v_peak = v_limit;

    // 加速距離: d = (v^2 - v0^2)/(2a)
    float d_acc = (v_peak * v_peak - v0 * v0) / (2.0f * a_cfg);
    if (d_acc < 0.0f) d_acc = 0.0f;
    // 減速距離: d = (v_peak^2 - 0)/(2a) （同じ加速度量で対称）
    float d_dec = (v_peak * v_peak) / (2.0f * a_cfg);
    float d_total = d_acc + d_dec;

    if (d_total > total_mm) {
        // 最高速度に達しない三角形プロファイル
        // total_mm = d_acc + d_dec、対称なら各半分
        float d_half = total_mm * 0.5f;
        // v_peak を再計算（v^2 = v0^2 + 2 a d_acc）
        float v2 = v0 * v0 + 2.0f * a_cfg * d_half;
        if (v2 < 0.0f) v2 = 0.0f;
        v_peak = sqrtf(v2);
        d_acc = d_half;
        d_dec = d_half;
    } else {
        // 台形：等速距離
        float d_const = total_mm - d_total;
        // 実行（加速→等速→減速）
        if (d_acc > 0.0f) {
            run_straight(d_acc / HALF_MM(), v_peak, 0);
        }
        if (d_const > 0.0f) {
            run_straight(d_const / HALF_MM(), v_peak, 0);
        }
        if (d_dec > 0.0f) {
            run_straight(d_dec / HALF_MM(), 0.0f, 0);
        }
        return;
    }

    // 三角形：加速→減速のみ
    if (d_acc > 0.0f) {
        run_straight(d_acc / HALF_MM(), v_peak, 0);
    }
    if (d_dec > 0.0f) {
        run_straight(d_dec / HALF_MM(), 0.0f, 0);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void) {
    driveR(ANGLE_ROTATE_90_R);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void) {
    driveR(-ANGLE_ROTATE_90_L);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void) { driveR(ANGLE_ROTATE_90_R * 2); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R90
// スラロームで右に90度旋回前進する
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R90(uint8_t fwall) {
    MF.FLAG.SLALOM_R = 1;
    MF.FLAG.CTRL = 0;

    if (fwall) {
        if (MF.FLAG.F_WALL) {
            driveFWall(dist_offset_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_offset_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_offset_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSR(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 0;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L90
// スラロームで左に90度旋回前進する
// 引数：引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L90(uint8_t fwall) {
    MF.FLAG.SLALOM_L = 1;
    MF.FLAG.CTRL = 0;

    if (fwall) {

        if (MF.FLAG.F_WALL) {
            driveFWall(dist_offset_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_offset_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_offset_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSL(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 0;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R90
// 90度大回り右旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R90(void) {

    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（必要なら）
    if (dist_l_turn_in_90 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_90, speed_now, velocity_l_turn_90, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.CTRL = 1;
    {
        // 壁切れ補正: 90°大回りターン直後の直進は全区間を検出区間とする
        const float v_const = velocity_l_turn_90;

        // 検出をアーム
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
        MF.FLAG.WALL_END   = 1;

        // 最大探索距離: 既定の出オフセット距離 + 追加上限
        float max_mm = dist_l_turn_out_90 + WALL_END_EXTEND_MAX_MM;
        float remaining_blocks = max_mm / HALF_MM();
        const float step_blocks = (2.0f / HALF_MM()); // 2mm相当で刻む
        bool triggered = false;

        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            run_straight(step, v_const, 0);
            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                // 消費（クリア）
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;
                triggered = true;
                break;
            }
            remaining_blocks -= step;
        }

        // 検出アーム解除
        MF.FLAG.WALL_END = 0;

        // 検出後の追従距離（一定速度でそのまま追従）
        if (triggered) {
            float follow_mm = dist_wall_end;
            if (follow_mm > 0.0f) {
                float extra_blocks = follow_mm / HALF_MM();
                run_straight(extra_blocks, v_const, 0);
            }
        }
    }
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L90
// 90度大回り左旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L90(void) {

    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（必要なら）
    if (dist_l_turn_in_90 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_90, speed_now, velocity_l_turn_90, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.CTRL = 1;
    {
        // 壁切れ補正: 90°大回りターン直後の直進は全区間を検出区間とする
        const float v_const = velocity_l_turn_90;

        // 検出をアーム
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
        MF.FLAG.WALL_END   = 1;

        // 最大探索距離: 既定の出オフセット距離 + 追加上限
        float max_mm = dist_l_turn_out_90 + WALL_END_EXTEND_MAX_MM;
        float remaining_blocks = max_mm / HALF_MM();
        const float step_blocks = (2.0f / HALF_MM()); // 2mm相当で刻む
        bool triggered = false;

        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            run_straight(step, v_const, 0);
            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                // 消費（クリア）
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;
                triggered = true;
                break;
            }
            remaining_blocks -= step;
        }

        // 検出アーム解除
        MF.FLAG.WALL_END = 0;

        // 検出後の追従距離（一定速度でそのまま追従）
        if (triggered) {
            float follow_mm = dist_wall_end;
            if (follow_mm > 0.0f) {
                float extra_blocks = follow_mm / HALF_MM();
                run_straight(extra_blocks, v_const, 0);
            }
        }
    }
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R180
// 180度大回り右旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R180(uint8_t fwall) {
    (void)fwall;
    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（必要なら）
    if (dist_l_turn_in_180 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_180, speed_now, velocity_l_turn_180, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.CTRL = 1;
    {
        // 壁切れ補正: 180°大回りターン直後の直進は全区間を検出区間とする
        const float v_const = velocity_l_turn_180;

        // 検出をアーム
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
        MF.FLAG.WALL_END   = 1;

        // 最大探索距離: 既定の出オフセット距離 + 追加上限
        float max_mm = dist_l_turn_out_180 + WALL_END_EXTEND_MAX_MM;
        float remaining_blocks = max_mm / HALF_MM();
        const float step_blocks = (2.0f / HALF_MM()); // 2mm相当で刻む
        bool triggered = false;

        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            run_straight(step, v_const, 0);
            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                // 消費（クリア）
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;
                triggered = true;
                break;
            }
            remaining_blocks -= step;
        }

        // 検出アーム解除
        MF.FLAG.WALL_END = 0;

        // 検出後の追従距離（一定速度でそのまま追従）
        if (triggered) {
            float follow_mm = dist_wall_end;
            if (follow_mm > 0.0f) {
                float extra_blocks = follow_mm / HALF_MM();
                run_straight(extra_blocks, v_const, 0);
            }
        }
    }
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L180
// 180度大回り左旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L180(uint8_t fwall) {
    (void)fwall;
    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（必要なら）
    if (dist_l_turn_in_180 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_180, speed_now, velocity_l_turn_180, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.CTRL = 1;
    {
        // 壁切れ補正: 180°大回りターン直後の直進は全区間を検出区間とする
        const float v_const = velocity_l_turn_180;

        // 検出をアーム
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
        MF.FLAG.WALL_END   = 1;

        // 最大探索距離: 既定の出オフセット距離 + 追加上限
        float max_mm = dist_l_turn_out_180 + WALL_END_EXTEND_MAX_MM;
        float remaining_blocks = max_mm / HALF_MM();
        const float step_blocks = (2.0f / HALF_MM()); // 2mm相当で刻む
        bool triggered = false;

        while (remaining_blocks > 0.0f) {
            float step = (remaining_blocks < step_blocks) ? remaining_blocks : step_blocks;
            run_straight(step, v_const, 0);
            if (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END) {
                // 消費（クリア）
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;
                triggered = true;
                break;
            }
            remaining_blocks -= step;
        }

        // 検出アーム解除
        MF.FLAG.WALL_END = 0;

        // 検出後の追従距離（一定速度でそのまま追従）
        if (triggered) {
            float follow_mm = dist_wall_end;
            if (follow_mm > 0.0f) {
                float extra_blocks = follow_mm / HALF_MM();
                run_straight(extra_blocks, v_const, 0);
            }
        }
    }
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    // 前直進（必要量）
    driveA(dist_turn45in_in, speed_now, velocity_turn45in, 0);
    // 旋回
    driveSR(angle_turn45in, alpha_turn45in);
    // 出オフセット
    driveA(dist_turn45in_out, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSR(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    // 前直進（必要量）
    driveA(dist_turn45in_in, speed_now, velocity_turn45in, 0);
    // 旋回
    driveSL(angle_turn45in, alpha_turn45in);
    // 出オフセット
    driveA(dist_turn45in_out, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSL(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_RV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_RV90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSR(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_LV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_LV90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSL(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSR(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSR(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSL(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSL(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_diagonal
// 斜め直進
// 引数：区画数，終端速度
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_diagonal(float section, float spd_out) {
    MF.FLAG.CTRL = 0;
    if(spd_out>1){
        MF.FLAG.CTRL_DIAGONAL = 1;
    }
    driveA(DIST_D_HALF_SEC * section, speed_now, spd_out, 0);

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_R90
// スラロームで右に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_R90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_R90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_L90
// スラロームで左に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_L90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_L90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：sw …… 0以外ならget_base()する
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(void) {

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;


    // 設定距離だけ後進
    velocity_interrupt = -130;
    drive_start();
    HAL_Delay(400);
    drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ前進
    driveA(DIST_SET_POSITION * 0.5, 0, 200, 0);
    driveA(DIST_SET_POSITION * 0.5, 200, 0, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// match_position
// 壁との距離を基準値に合わせる
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void match_position(uint16_t target_value) {
    (void)target_value; // 未使用（パラメータは params.h の定数を使用）

    // 前壁が見えていなければ何もしない
    /*
    if (ad_fr < F_ALIGN_DETECT_THR || ad_fl < F_ALIGN_DETECT_THR) {
        printf("match_position: front wall not detected (FR=%u, FL=%u)\r\n",
               (unsigned)ad_fr, (unsigned)ad_fl);
        buzzer_beep(2500);
        return;
    }
    */

    // 側壁の自動壁制御は無効化（前壁のみで位置・角度を合わせる）
    uint8_t ctrl_prev = MF.FLAG.CTRL;
    float kp_wall_prev = kp_wall;
    MF.FLAG.CTRL = 0;
    kp_wall = 0.0f;

    // 走行制御の状態を初期化して開始
    drive_variable_reset();
    acceleration_interrupt = 0;
    alpha_interrupt = 0;
    // 以降、並進は velocity_interrupt を直接更新して速度FBを用いる
    // （calculate_translation は target_distance を更新するが、distance_PID は未使用）
    velocity_interrupt = 0;
    omega_interrupt = 0;
    drive_start();

    int stable_count = 0;
    uint32_t count = 0;

    while (count < 2000 && ad_fr > WALL_BASE_FR * 1.5 && ad_fl > WALL_BASE_FL * 1.5) {

        // 距離[mm]ベースの誤差算出（+は目標より遠い/右が遠い）
        float d_fr = sensor_distance_from_fr(ad_fr); // warp適用済み距離
        float d_fl = sensor_distance_from_fl(ad_fl);
        float e_fr = d_fr - F_ALIGN_TARGET_FR; // [mm]
        float e_fl = d_fl - F_ALIGN_TARGET_FL; // [mm]
        float e_pos = 0.5f * (e_fr + e_fl);    // 並進：平均を使う（正: 遠い→前進）
        float e_ang = (e_fr - e_fl);           // 角度：差分を使う（正: 右が遠い）

        // 収束判定（両センサが目標±MATCH_POS_TOL 内に連続して入ったら終了）
        if (fabsf(e_fr) <= MATCH_POS_TOL && fabsf(e_fl) <= MATCH_POS_TOL) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        if (stable_count >= MATCH_POS_STABLE_COUNT) {
            printf("match_position: converged (stable_count=%d)\r\n", stable_count);
            break;
        }

        // 並進は速度FBへ直接指示（velocity_interrupt を更新）
        // 距離が遠い(+e_pos)ときは前進(+)させる
        float v_cmd = MATCH_POS_KP_TRANS * e_pos; // [mm/s]
        if (v_cmd >  MATCH_POS_VEL_MAX) v_cmd =  MATCH_POS_VEL_MAX;
        if (v_cmd < -MATCH_POS_VEL_MAX) v_cmd = -MATCH_POS_VEL_MAX;
        velocity_interrupt = v_cmd;

        // 角度は omega_interrupt を直接与えて omega_PID を活用
        float w_cmd = MATCH_POS_KP_ROT * e_ang; // [deg/s]
        if (w_cmd >  MATCH_POS_OMEGA_MAX) w_cmd =  MATCH_POS_OMEGA_MAX;
        if (w_cmd < -MATCH_POS_OMEGA_MAX) w_cmd = -MATCH_POS_OMEGA_MAX;
        omega_interrupt = w_cmd;

        HAL_Delay(2); // 2ms周期で更新（ISRは1kHz）
        count++;
    }

    // 停止（収束 or 安全離脱）
    omega_interrupt = 0;
    velocity_interrupt = 0;
    drive_variable_reset();

    // 復帰
    MF.FLAG.CTRL = ctrl_prev;
    kp_wall = kp_wall_prev;
}

/*==========================================================
    走行系 基幹関数
==========================================================*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveA
// 指定パルス分加速走行する
// 引数1：dist …… 走行する距離[mm]
// 引数2: spd_in …… 初速度[mm/sec]
// 引数3: spd_out …… 到達速度[mm/sec]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(float dist, float spd_in, float spd_out, float dist_wallend) {

    (void)dist_wallend;

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // 実距離で終了判定するため、参照値をクリアし速度を初期化する
    target_distance = 0;
    velocity_interrupt = spd_in;

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // sベース台形プロファイルを有効化
    {
        float a_base = (MF.FLAG.SCND || known_straight) ? acceleration_straight_dash : acceleration_straight;
        profile_active  = 1;
        profile_s_end   = dist;
        profile_v_out   = spd_out;
        profile_v_max   = fmaxf(velocity_straight, fmaxf(spd_in, spd_out));
        profile_a_accel = fabsf(a_base);
        profile_a_decel = fabsf(a_base);
    }

    drive_start();

    // 実距離（real_distance）が目標距離に達するまで走行
    if (acceleration_interrupt > 0) {

        while (real_distance < dist) {
            background_replan_tick();
        }

    } else if (acceleration_interrupt <= 0) {

        if (MF.FLAG.F_WALL_STOP) {
            while (real_distance < dist && velocity_interrupt > 0 &&
                   (ad_fl + ad_fr) < thr_f_wall) {
                background_replan_tick();
            };
        } else {
            while (real_distance < dist && velocity_interrupt > 0) {
                background_replan_tick();
            };
        }
    }
    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // プロファイル無効化（保険）
    profile_active = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveR
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行する距離[mm]
// 引数2：spd …… 速度[mm/s]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveR(float angle) {

    // 角加速度を設定
    if (angle >= 0) {
        alpha_interrupt = ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = -ALPHA_ROTATE_90;
    }

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    velocity_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    if (angle >= 0) {
        while (real_angle > -angle * 0.333 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle * 0.333 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    if (angle >= 0) {
        while (real_angle > -angle * 0.666 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle * 0.666 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    if (angle >= 0) {
        alpha_interrupt = -ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = +ALPHA_ROTATE_90;
    };
    if (angle >= 0) {
        while (real_angle > -angle && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    alpha_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveC
// 指定パルス分デフォルトインターバルで走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(int dist) {
    (void)dist;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveM
// センサ値が基準値になるまで前進or後進する
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveM(uint16_t sens_tgt) {
    (void)sens_tgt;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSR
// 指定速度でスラロームで右旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSR(float angle_turn, float alpha_turn) {

    // printf("%f\n", alpha_interrupt);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    drive_start();

    // 角加速度と並進加速度を設定
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle > -angle_turn * 0.333 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;

    while (real_angle > -angle_turn * 0.666 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle > -angle_turn && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSL
// 指定速度でスラロームで左旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSL(float angle_turn, float alpha_turn) {

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;

    // 角加速度と並進加速度を設定
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle < angle_turn * 0.333 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;
    while (real_angle < angle_turn * 0.666 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle < angle_turn && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
}

void driveFWall(float dist, float spd_in, float spd_out) {

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    // エンコーダ距離は用いず、前壁距離がしきい値に達するまで直進する。
    // しきい値は以下の優先順：
    //  (1) searchRunParams.front_dist_turn_start_moving (>0 のとき)
    //  (2) 従来式 FRONT_DIST_AT_CELL_ENTRY_MM - dist_offset_in
    float d_front_thr;
    if (searchRunParams.front_dist_turn_start_moving > 0.0f) {
        d_front_thr = searchRunParams.front_dist_turn_start_moving;
    } else {
        d_front_thr = FRONT_DIST_AT_CELL_ENTRY_MM - dist_offset_in;
        if (d_front_thr < 0.0f) d_front_thr = 0.0f;
    }
    if (MF.FLAG.SLALOM_R) {
        while (1) {
            background_replan_tick();
            // 速度を spd_out にクランプ（加速し過ぎない）
            if (acceleration_interrupt > 0.0f && velocity_interrupt >= spd_out) {
                acceleration_interrupt = 0.0f;
                velocity_interrupt = spd_out;
            }
            // 合成距離（FR+FL）で前壁までの距離を推定し、d_front_thr 以下で到達と判定
            uint32_t fsum = (uint32_t)ad_fl + (uint32_t)ad_fr;
            if (fsum > 0xFFFFu) fsum = 0xFFFFu;
            float d_front = sensor_distance_from_fsum((uint16_t)fsum);
            if (d_front <= d_front_thr) {
                break;
            }
            if (MF.FLAG.FAILED) break;
        }
    } else if (MF.FLAG.SLALOM_L) {
        while (1) {
            background_replan_tick();
            // 速度を spd_out にクランプ（加速し過ぎない）
            if (acceleration_interrupt > 0.0f && velocity_interrupt >= spd_out) {
                acceleration_interrupt = 0.0f;
                velocity_interrupt = spd_out;
            }
            // 合成距離（FR+FL）で前壁までの距離を推定し、d_front_thr 以下で到達と判定
            uint32_t fsum = (uint32_t)ad_fl + (uint32_t)ad_fr;
            if (fsum > 0xFFFFu) fsum = 0xFFFFu;
            float d_front = sensor_distance_from_fsum((uint16_t)fsum);
            if (d_front <= d_front_thr) {
                break;
            }
            if (MF.FLAG.FAILED) break;
        }
    }

    // エンコーダ由来の距離では抜けない設計に変更したため、延長待ちは不要

    velocity_interrupt = spd_out;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

void driveWallEnd(float dist, float spd_in, float spd_out) {
    driveA(dist, spd_in, spd_out, 0);
}

void adjust_wallend(void) {}

/*==========================================================
    初期化関数・設定関数・その他関数
==========================================================*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_init
// 走行系の変数の初期化，モータードライバ関係のGPIO設定とPWM出力に使うタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void) {
    // エンコーダの読み取り開始
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    wall_end_count = 0;

    // 探索走行の壁切れ補正は既定で有効
    g_enable_wall_end_search = 1;

    //====走行系の変数の初期化====

    //====マウスフラグの初期化===
    MF.FLAGS = 0; // フラグクリア
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_variable_reset
// 割込み内の走行系の変数の初期化
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_variable_reset(void) {

    // 速度
    acceleration_interrupt = 0;
    target_distance = 0;

    // 角度
    alpha_interrupt = 0;
    omega_interrupt = 0;
    target_angle = 0;

    // プロファイル
    profile_active = 0;
    profile_s_end = 0.0f;
    profile_v_out = 0.0f;
    profile_v_max = 0.0f;
    profile_a_accel = 0.0f;
    profile_a_decel = 0.0f;

    // PIDの積算項（距離）
    distance_error = 0;
    previous_distance_error = 0;
    distance_error_error = 0;
    distance_integral = 0;

    // PIDの積算項（速度）
    velocity_error = 0;
    previous_velocity_error = 0;
    velocity_error_error = 0;
    velocity_integral = 0;

    // 位置→速度補正（外側PI）
    pos2vel_integral = 0;
    pos2vel_prev_error = 0;
    pos2vel_correction = 0;

    // PIDの積算項（角度）
    angle_error = 0;
    previous_angle_error = 0;
    angle_error_error = 0;
    angle_integral = 0;

    // PIDの積算項（角速度）
    target_omega = 0;
    omega_error = 0;
    previous_omega_error = 0;
    omega_error_error = 0;
    omega_integral = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_enable_motor
// モータドライバの電源ON
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_enable_motor(void) {
    // すでに有効なら何もしない（冪等）
    if (s_motor_enabled) {
        s_outputs_locked = 0;
        return;
    }

    // プリスタート・リセット
    // 機体設置時にタイヤが動いてしまった場合でも、初回動作で不要なFBが乗らないよう
    // 走行系の参照・積分・実距離/速度・エンコーダカウンタをクリアする
    {
        // FB系の積分・履歴のみクリア（コマンド値は維持する）
        distance_error = 0.0f;
        previous_distance_error = 0.0f;
        distance_error_error = 0.0f;
        distance_integral = 0.0f;

        velocity_error = 0.0f;
        previous_velocity_error = 0.0f;
        velocity_error_error = 0.0f;
        velocity_integral = 0.0f;

        angle_error = 0.0f;
        previous_angle_error = 0.0f;
        angle_error_error = 0.0f;
        angle_integral = 0.0f;

        omega_error = 0.0f;
        previous_omega_error = 0.0f;
        omega_error_error = 0.0f;
        omega_integral = 0.0f;

        // 位置→速度補正の内部状態
        pos2vel_integral = 0.0f;
        pos2vel_prev_error = 0.0f;
        pos2vel_correction = 0.0f;

        // 参照距離（pos2velの原点）
        target_distance = 0.0f;

        // 実距離・エンコーダ距離/速度のクリア
        real_distance = 0.0f;
        encoder_distance_r = 0.0f;
        encoder_distance_l = 0.0f;
        real_velocity = 0.0f;
        encoder_speed_r = 0.0f;
        encoder_speed_l = 0.0f;

        // エンコーダハードウェアカウンタを基準値へ初期化
        TIM8->CNT = 30000;
        TIM4->CNT = 30000;
        encoder_count_r = 30000;
        encoder_count_l = 30000;
        previous_encoder_count_r = 30000;
        previous_encoder_count_l = 30000;

        // 角度系も初期化（初動の不要な角度FBを防止）
        real_angle = 0.0f;
        IMU_angle = 0.0f;
        target_angle = 0.0f;
    }

    // 現在のDIRピンレベルを取得
    GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
    GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
    s_dir_pin_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
    s_dir_pin_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    // 1) CCR更新をロックし、まず IN1==IN2 となるアイドル（短絡ブレーキ相当）を作る
    s_outputs_locked = 1;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 2) PWM を開始（安全CCRで IN1==IN2 を提示）
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    tim1_wait_us(50);

    // 3) STBY を有効化（出力段を有効化）
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    tim1_wait_us(100);

    // 4) 有効化完了
    s_motor_enabled = 1;
    s_outputs_locked = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_disable_motor
// モータードライバの電源OFF
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_disable_motor(void) {
    // すでに無効なら何もしない（冪等）
    if (!s_motor_enabled) {
        HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
        return;
    }

    // 停止時は IN1==IN2 となるアイドルを明示してから停止
    s_outputs_locked = 1;
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);

    // PWM 停止 → STBY 無効化
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

    s_motor_enabled = 0;
    s_outputs_locked = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_brake
// 短絡ブレーキのON/OFF（STBY/PWM状態は変更しない）
// 引数：enable …… trueでブレーキ、falseで解除
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_brake(bool enable) {
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    if (enable) {
        // DIRピンから現在レベルを取得して IN1==IN2 を生成
        GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
        GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
        s_dir_pin_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
        s_dir_pin_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

        s_outputs_locked = 1;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    } else {
        // 通常更新に戻す
        s_outputs_locked = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_start
// 走行を開始する
// （pulse_l,pulse_rを0にリセットしてタイマを有効にする）
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void) {
    // 統合：STBY と PWM を安全に同時運用へ
    drive_enable_motor();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_stop
// 走行を終了する
// （タイマを止めてタイマカウント値を0にリセットする）
// 引数1：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void) {
    // 互換：従来の drive_stop は「ブレーキ」に読み替え
    drive_brake(true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_set_dir
// 進行方向を設定する
// 引数1：d_dir …… どの方向に進行するか  0桁目で左，1桁目で右の方向設定
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_set_dir(uint8_t d_dir) {
    // printf("DIR\n");

    //====左モータ====
    switch (d_dir & 0x0f) { // 0~3ビット目を取り出す
    //----正回転----
    case 0x00: // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_FWD_L);
        s_dir_pin_high_l = (DIR_FWD_L == GPIO_PIN_SET) ? 1 : 0;
        // 左を前進方向に設定
        break;
    //----逆回転----
    case 0x01: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_BACK_L);
        s_dir_pin_high_l = (DIR_BACK_L == GPIO_PIN_SET) ? 1 : 0;
        // 左を後進方向に設定
        break;
    }
    //====右モータ====
    switch (d_dir & 0xf0) { // 4~7ビット目を取り出す
    case 0x00:              // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_FWD_R);
        s_dir_pin_high_r = (DIR_FWD_R == GPIO_PIN_SET) ? 1 : 0;
        // 右を前進方向に設定
        break;
    //----逆回転----
    case 0x10: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_BACK_R);
        s_dir_pin_high_r = (DIR_BACK_R == GPIO_PIN_SET) ? 1 : 0;
        // 右を後進方向に設定
        break;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_motor
// モータを回す
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_motor(void) {
    // 有効化/停止シーケンス中はCCR更新を抑止し、IN1==IN2のアイドルを維持
    if (s_outputs_locked) {
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
        return;
    }

    // 並進と回転の出力を合算
    out_r = out_translation - out_rotate;
    out_l = out_translation + out_rotate;

    // 回転のフェイルセーフ
    if (((out_r - out_l > FAIL_LR_ERROR || out_r - out_l < -FAIL_LR_ERROR)) &&
        MF.FLAG.RUNNING) {
        fail_count_lr++;
    } else {
        fail_count_lr = 0;
    }

    if (fail_count_lr > FAIL_COUNT_LR) {
        MF.FLAG.FAILED = 1;
        fail_count_lr = 0;
    }

    // 並進衝突のフェイルセーフ
    if ((IMU_acceleration < -FAIL_ACC || IMU_acceleration > FAIL_ACC) &&
        MF.FLAG.RUNNING) {
        fail_count_acc++;
    } else {
        fail_count_acc = 0;
    }

    if (fail_count_acc > FAIL_COUNT_ACC && MF.FLAG.RUNNING) {
        MF.FLAG.FAILED = 1;
        fail_count_acc = 0;
    }

    // 左右モータの回転方向の指定（sign-magnitude）
    // DIR ピンで方向を出し、PWM は常に High アクティブのまま使用する
    if (out_r >= 0 && out_l >= 0) {
        drive_set_dir(0x00);
    } else if (out_r >= 0 && out_l < 0) {
        drive_set_dir(0x01);
        out_l = -out_l; // duty 計算用に絶対値へ
    } else if (out_r < 0 && out_l >= 0) {
        drive_set_dir(0x10);
        out_r = -out_r;
    } else if (out_r < 0 && out_l < 0) {
        drive_set_dir(0x11);
        out_r = -out_r;
        out_l = -out_l;
    }

    // PWM出力
    if (MF.FLAG.FAILED) {
        // フェイルセーフ発動の場合，強制停止（PWM=0 ＋ STBY=Low）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        drive_disable_motor();
        drive_fan(0);

        buzzer_beep(1200);

    } else {
        // TIM2 の ARR を取得（0..ARR のカウント幅）
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        const uint16_t min_counts = (uint16_t)((arr * MOTOR_MIN_DUTY_PCT) / 100u);
        const uint16_t boost_counts = (uint16_t)((arr * MOTOR_BOOST_DUTY_PCT) / 100u);

        // コマンド（0..ARR）へクリップ
        uint16_t cmd_l = (uint16_t)fminf(fabsf(out_l), (float)arr);
        uint16_t cmd_r = (uint16_t)fminf(fabsf(out_r), (float)arr);

        // 最小デューティ補償（非ゼロの場合のみ）
        if (cmd_l > 0 && cmd_l < min_counts) cmd_l = min_counts;
        if (cmd_r > 0 && cmd_r < min_counts) cmd_r = min_counts;

        // 起動ブースト（停止 -> 非ゼロの立上りで一定時間ブースト）
        const uint32_t now = HAL_GetTick();
        if (s_prev_cmd_l == 0 && cmd_l > 0) {
            s_boost_until_ms_l = now + MOTOR_BOOST_TIME_MS;
        }
        if (s_prev_cmd_r == 0 && cmd_r > 0) {
            s_boost_until_ms_r = now + MOTOR_BOOST_TIME_MS;
        }
        if (cmd_l > 0 && now < s_boost_until_ms_l && boost_counts > cmd_l) {
            cmd_l = boost_counts;
        }
        if (cmd_r > 0 && now < s_boost_until_ms_r && boost_counts > cmd_r) {
            cmd_r = boost_counts;
        }

        // デューティ適用（DIRが反転対象レベルのとき PWM反転）
        uint32_t ccr_l;
        uint32_t ccr_r;
        const uint32_t arr_apply = arr;
        uint8_t invert_active_l = ((s_dir_pin_high_l ? 1 : 0) == PWM_INVERT_DIR_LEVEL);
        uint8_t invert_active_r = ((s_dir_pin_high_r ? 1 : 0) == PWM_INVERT_DIR_LEVEL);
        if (cmd_l == 0) {
            ccr_l = s_dir_pin_high_l ? arr_apply : 0u; // アイドルはIN1==IN2
        } else if (invert_active_l) {
            ccr_l = arr_apply - cmd_l;
        } else {
            ccr_l = cmd_l;
        }

        if (cmd_r == 0) {
            ccr_r = s_dir_pin_high_r ? arr_apply : 0u; // アイドルはIN1==IN2
        } else if (invert_active_r) {
            ccr_r = arr_apply - cmd_r;
        } else {
            ccr_r = cmd_r;
        }

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_l);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_r);

        // 次回用の履歴
        s_prev_cmd_l = cmd_l;
        s_prev_cmd_r = cmd_r;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_fan
// ファンを回す
// 引数：fan_power (0~1000)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_fan(uint16_t fan_power) {

    if (fan_power > 0) {
        MF.FLAG.SUCTION = 1;
    } else {
        MF.FLAG.SUCTION = 0;
    }

    if (fan_power) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

        for (uint16_t i = 0; i < fan_power; i++) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            HAL_Delay(1);
        }
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        // 停止時刻を記録（同一TIM3を使うブザーとの干渉を避けるためのクールダウン判定用）
        fan_last_off_ms = HAL_GetTick();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// test_run
// テスト走行モード
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_run(void) {
    int mode = 0;
    // drive_enable_motor();

    led_flash(8);

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:
            //----尻当て----
            printf("Mode 4-0 Set Position.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            set_position();

            led_flash(5);
            drive_stop();

            break;
        case 1:
            // ログ確認用 
            printf("Mode 6-1 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.0;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_VELOCITY);
            // log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);
            one_sectionU(0);
            half_sectionD(0);
            log_stop();

            drive_stop();

            break;
        case 2:
            // ログ確認用
            printf("Mode 6-2 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.0;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_DISTANCE);
            // log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);
            one_sectionU(0);
            one_sectionU(0);
            half_sectionD(0);
            log_stop();

            // drive_stop();

            break;
        case 3:
            // ログ確認用
            printf("Mode 6-2 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);

            turn_R90(0);
            // rotate_180();

            half_sectionD(0);
            log_stop();

            drive_stop();

            break;
        case 4:
            //----直進----
            printf("Mode 4-4 straight 2 Sections.\n");

            // 直線
            acceleration_straight = 5444.44;
            acceleration_straight_dash = 8000; // 5000
            velocity_straight = 700;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 43900;
            acceleration_turn = 0;
            dist_offset_in = 5;
            dist_offset_out = 8.47; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.5;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            led_flash(3);
            drive_fan(1000);
            led_flash(3);

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(700);

            turn_R90(0);

            half_sectionD(0);
            log_stop();

            drive_stop();

            led_flash(3);
            drive_fan(0);
            led_flash(3);

            break;

        case 5:
            //----右旋回----
            printf("Mode 4-5 Turn R90.\n");

            // 直線
            acceleration_straight = 11111.11;
            acceleration_straight_dash = 10000; // 5000
            velocity_straight = 1000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 43900;
            acceleration_turn = 0;
            dist_offset_in = 5;
            dist_offset_out = 8.47; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 1000;
            alpha_l_turn_90 = 20400;
            angle_l_turn_90 = 89.5;
            dist_l_turn_out_90 = 8.25;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            led_flash(3);
            drive_fan(1000);
            led_flash(3);

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);

            l_turn_R90();

            half_sectionD(0);
            log_stop();

            drive_stop();

            led_flash(3);
            drive_fan(0);
            led_flash(3);

            break;
        case 6:
            //--------
            printf("Mode 4-6 Rotate R90.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            get_base();

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            first_sectionA();
            for (uint8_t i = 0; i < 7; i++) {
                half_sectionU();
            }
            turn_R90(1);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;
        case 7:

            printf("Mode 4-7.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(velocity_l_turn_90);
            l_turn_R90();
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 8:

            printf("Mode 4-8 (rear set_position).\n");

            // 尻当てによる中央合わせ（前壁がある想定で180度回して実施）
            led_flash(5);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();
            rotate_180();
            set_position();
            rotate_180();
            buzzer_beep(1200);
            drive_stop();

            break;
        
            case 9:

            // 新しいロギングシステムでログを出力
            printf("Mode 4-2 - ロギングデータの出力\n");
            log_print_all();
        }
    }
    drive_disable_motor();
}

void reset_failed(void) {

    drive_stop();
    drive_variable_reset();

    for (int i = 0; i < 5; i++) {
        buzzer_beep(900);
    }

    MF.FLAG.FAILED = 0;
}
