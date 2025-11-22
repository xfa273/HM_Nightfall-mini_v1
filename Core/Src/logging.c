#include "global.h"
#include "interrupt.h"
#include "logging.h"
#include "stdio.h"
#include "sensor_distance.h"

// 現在のログプロファイル（取得内容の切替）
static volatile LogProfile s_log_profile = LOG_PROFILE_OMEGA;

void log_set_profile(LogProfile profile) { s_log_profile = profile; }

// 前壁補正テスト用: CSV出力
void log_print_frontwall_all(void) {
    printf("=== Micromouse Log Data (CSV Format, FRONTWALL) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,d_front_mm,d_front_thr_mm,dist_offset_in_mm,front_entry_mm,real_velocity_mm_s,ad_fr,ad_fl\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,   // d_front_mm
               entry->actual_omega,   // d_front_thr_mm
               entry->p_term_omega,   // dist_offset_in
               entry->i_term_omega,   // FRONT_DIST_AT_CELL_ENTRY_MM
               entry->d_term_omega,   // real_velocity
               entry->motor_out_r,    // ad_fr (整数に近い)
               entry->motor_out_l);   // ad_fl (整数に近い)
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 角速度ログ（log_buffer）をCSV出力
void log_print_omega_all(void) {
    printf("=== Micromouse Log Data (CSV Format, OMEGA) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 角度ログ（log_buffer2）をCSV出力
void log_print_angle_all(void) {
    printf("=== Micromouse Log Data (CSV Format, ANGLE) ===\n");
    printf("Total entries: %d\n", log_buffer2.count);
    printf("CSV Format: timestamp,target_angle,actual_angle,p_term,i_term,d_term,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer2.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer2.count;
    uint16_t start = log_buffer2.count > MAX_LOG_ENTRIES ? log_buffer2.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer2.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer2.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

/**
 * @brief 並進制御ログを8列CSVで出力する（ツール互換）
 * フォーマット（8列）:
 *   timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l
 * 備考:
 *   - target_omega は target_velocity、actual_omega は real_velocity を格納済み
 *   - out_translation_est は (motor_out_r + motor_out_l)/2 で再現可能
 */
void log_print_translation_csv(void) {
    printf("=== Micromouse Log Data (CSV Format, TRANSLATION) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];

        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}
LogProfile log_get_profile(void) { return s_log_profile; }

/**
 * @brief ロギングシステムを初期化する
 */
void log_init(void) {
    // 速度ログバッファ初期化
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 0;
    log_buffer.start_time = 0;

    // 距離ログバッファ初期化
    log_buffer2.head = 0;
    log_buffer2.count = 0;
    log_buffer2.logging_active = 0;
    log_buffer2.start_time = 0;
}

/**
 * @brief 現在のプロファイルに基づいて1サンプル分のログを記録する（割り込みから呼ぶ）
 * @note 取得する変数は後で編集予定のため、現状は既存(角速度系)の内容を各プロファイルで共通に記録
 */
void log_capture_tick(void) {
    // どちらかがアクティブなら記録
    if (!log_buffer.logging_active && !log_buffer2.logging_active) {
        return;
    }

    static float s_angle_i = 0.0f;       // 角度ログ用の仮想I項（制御とは独立）
    static float s_angle_prev_e = 0.0f;  // 角度ログ用の仮想D項

    uint32_t current_time = HAL_GetTick();

    switch (s_log_profile) {
    case LOG_PROFILE_VELOCITY: {
        // 速度ログ（log_buffer）
        if (log_buffer.logging_active && log_buffer.count < MAX_LOG_ENTRIES) {
            uint16_t pos = log_buffer.head;
            log_buffer.entries[pos].count = (uint16_t)log_buffer.count;
            log_buffer.entries[pos].target_omega = velocity_interrupt; // 目標速度
            log_buffer.entries[pos].actual_omega = real_velocity;      // 実速度
            log_buffer.entries[pos].p_term_omega = KP_VELOCITY * velocity_error;
            log_buffer.entries[pos].i_term_omega = KI_VELOCITY * velocity_integral;
            // d_term_omega フィールドには target_velocity を記録（直進速度ログの可視化用）
            log_buffer.entries[pos].d_term_omega = target_velocity;  
            log_buffer.entries[pos].motor_out_r = (float)out_r;
            log_buffer.entries[pos].motor_out_l = (float)out_l;
            log_buffer.entries[pos].timestamp = current_time;
            log_buffer.head = (pos + 1) % MAX_LOG_ENTRIES;
            log_buffer.count++;
        }

        // 距離ログ（log_buffer2）
        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = target_distance; // 目標距離
            log_buffer2.entries[pos2].actual_omega = real_distance;   // 実距離
            log_buffer2.entries[pos2].p_term_omega = KP_DISTANCE * distance_error;
            log_buffer2.entries[pos2].i_term_omega = KI_DISTANCE * distance_integral;
            log_buffer2.entries[pos2].d_term_omega = KD_DISTANCE * distance_error_error;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;
    }
    case LOG_PROFILE_CUSTOM: {
        // 前壁補正テスト用: d_front, d_front_thr, dist_offset_in, FRONT_DIST_AT_CELL_ENTRY_MM, real_velocity, ad_fr, ad_fl
        if (log_buffer.logging_active && log_buffer.count < MAX_LOG_ENTRIES) {
            uint16_t pos = log_buffer.head;
            // 測定
            uint32_t sum = (uint32_t)ad_fl + (uint32_t)ad_fr;
            if (sum > 0xFFFFu) sum = 0xFFFFu;
            float d_front = sensor_distance_from_fsum((uint16_t)sum);
            float d_thr = FRONT_DIST_AT_CELL_ENTRY_MM - dist_offset_in;
            if (d_thr < 0.0f) d_thr = 0.0f;

            log_buffer.entries[pos].count = (uint16_t)log_buffer.count;
            log_buffer.entries[pos].target_omega = d_front;            // d_front_mm
            log_buffer.entries[pos].actual_omega = d_thr;               // d_front_thr_mm
            log_buffer.entries[pos].p_term_omega = dist_offset_in;      // dist_offset_in_mm
            log_buffer.entries[pos].i_term_omega = FRONT_DIST_AT_CELL_ENTRY_MM; // front_entry_mm
            log_buffer.entries[pos].d_term_omega = real_velocity;       // real_velocity_mm_s
            log_buffer.entries[pos].motor_out_r = (float)ad_fr;         // ad_fr
            log_buffer.entries[pos].motor_out_l = (float)ad_fl;         // ad_fl
            log_buffer.entries[pos].timestamp = current_time;
            log_buffer.head = (pos + 1) % MAX_LOG_ENTRIES;
            log_buffer.count++;
        }
        // log_buffer2 は未使用
        break;
    }
    case LOG_PROFILE_OMEGA: {
        // 角速度ログ（log_buffer）
        if (log_buffer.logging_active && log_buffer.count < MAX_LOG_ENTRIES) {
            uint16_t pos = log_buffer.head;
            log_buffer.entries[pos].count = (uint16_t)log_buffer.count;
            log_buffer.entries[pos].target_omega = omega_interrupt; // 目標角速度
            log_buffer.entries[pos].actual_omega = real_omega;      // 実角速度
            // 制御の内部状態をそのまま参照（PID各項の推定）
            log_buffer.entries[pos].p_term_omega = KP_OMEGA * omega_error;
            log_buffer.entries[pos].i_term_omega = KI_OMEGA * omega_integral;
            log_buffer.entries[pos].d_term_omega = KD_OMEGA * omega_error_error;
            log_buffer.entries[pos].motor_out_r = (float)out_r;
            log_buffer.entries[pos].motor_out_l = (float)out_l;
            log_buffer.entries[pos].timestamp = current_time;
            log_buffer.head = (pos + 1) % MAX_LOG_ENTRIES;
            log_buffer.count++;
        }

        // 角度ログ（log_buffer2）: target_angle vs real_angle（独立PI-D推定）
        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            float e_ang = target_angle - real_angle;
            s_angle_i += e_ang;
            float d_ang = e_ang - s_angle_prev_e;
            s_angle_prev_e = e_ang;

            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = target_angle; // 目標角度
            log_buffer2.entries[pos2].actual_omega = real_angle;   // 実角度
            log_buffer2.entries[pos2].p_term_omega = KP_ANGLE * e_ang;
            log_buffer2.entries[pos2].i_term_omega = KI_ANGLE * s_angle_i;
            log_buffer2.entries[pos2].d_term_omega = KD_ANGLE * d_ang;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;
    }
    default: {
        // 既定は速度/距離と同様に扱う
        if (log_buffer.logging_active && log_buffer.count < MAX_LOG_ENTRIES) {
            uint16_t pos = log_buffer.head;
            log_buffer.entries[pos].count = (uint16_t)log_buffer.count;
            log_buffer.entries[pos].target_omega = velocity_interrupt;
            log_buffer.entries[pos].actual_omega = real_velocity;
            log_buffer.entries[pos].p_term_omega = KP_VELOCITY * velocity_error;
            log_buffer.entries[pos].i_term_omega = KI_VELOCITY * velocity_integral;
            log_buffer.entries[pos].d_term_omega = KD_VELOCITY * velocity_error_error;
            log_buffer.entries[pos].motor_out_r = (float)out_r;
            log_buffer.entries[pos].motor_out_l = (float)out_l;
            log_buffer.entries[pos].timestamp = current_time;
            log_buffer.head = (pos + 1) % MAX_LOG_ENTRIES;
            log_buffer.count++;
        }

        if (log_buffer2.logging_active && log_buffer2.count < MAX_LOG_ENTRIES) {
            uint16_t pos2 = log_buffer2.head;
            log_buffer2.entries[pos2].count = (uint16_t)log_buffer2.count;
            log_buffer2.entries[pos2].target_omega = target_distance;
            log_buffer2.entries[pos2].actual_omega = real_distance;
            log_buffer2.entries[pos2].p_term_omega = KP_DISTANCE * distance_error;
            log_buffer2.entries[pos2].i_term_omega = KI_DISTANCE * distance_integral;
            log_buffer2.entries[pos2].d_term_omega = KD_DISTANCE * distance_error_error;
            log_buffer2.entries[pos2].motor_out_r = (float)out_r;
            log_buffer2.entries[pos2].motor_out_l = (float)out_l;
            log_buffer2.entries[pos2].timestamp = current_time;
            log_buffer2.head = (pos2 + 1) % MAX_LOG_ENTRIES;
            log_buffer2.count++;
        }
        break;
    }
    }
}

/**
 * @brief ロギングを開始する
 * @param start_time 開始時間（ms）
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_start(uint32_t start_time) {
    // 速度ログ開始
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 1;
    log_buffer.start_time = start_time;

    // 距離ログ開始
    log_buffer2.head = 0;
    log_buffer2.count = 0;
    log_buffer2.logging_active = 1;
    log_buffer2.start_time = start_time;

    // ロギングフラグを設定
    MF.FLAG.GET_LOG_1 = 1;
    MF.FLAG.GET_LOG_2 = 1;
}

/**
 * @brief ロギングを停止する
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_stop(void) {
    log_buffer.logging_active = 0;
    log_buffer2.logging_active = 0;

    // ロギングフラグをクリア
    MF.FLAG.GET_LOG_1 = 0;
    MF.FLAG.GET_LOG_2 = 0;
}

/**
 * @brief ログにデータを追加する（割り込みハンドラから呼ばれる）
 * @param index サンプルカウント/インデックス
 * @param target_omega 目標角速度
 * @param actual_omega 実際の角速度
 * @param p_term_omega 角速度制御のP項
 * @param i_term_omega 角速度制御のI項
 * @param d_term_omega 角速度制御のD項
 * @param motor_out_r 右モーター出力
 * @param motor_out_l 左モーター出力
 * @param timestamp タイムスタンプ（ms）
 */
void log_add_entry(uint16_t index, float target_omega, float actual_omega,
                  float p_term_omega, float i_term_omega, float d_term_omega,
                  float motor_out_r, float motor_out_l, uint32_t timestamp) {
    
    if (!log_buffer.logging_active || log_buffer.count >= MAX_LOG_ENTRIES) {
        return;
    }
    
    uint16_t position = log_buffer.head;
    
    // データをバッファに格納
    log_buffer.entries[position].count = index;
    log_buffer.entries[position].target_omega = target_omega;
    log_buffer.entries[position].actual_omega = actual_omega;
    log_buffer.entries[position].p_term_omega = p_term_omega;
    log_buffer.entries[position].i_term_omega = i_term_omega;
    log_buffer.entries[position].d_term_omega = d_term_omega;
    log_buffer.entries[position].motor_out_r = motor_out_r;
    log_buffer.entries[position].motor_out_l = motor_out_l;
    log_buffer.entries[position].timestamp = timestamp;
    
    // ヘッドとカウントを更新
    log_buffer.head = (position + 1) % MAX_LOG_ENTRIES;
    log_buffer.count++;
}

/**
 * @brief ログデータをシリアルポートに出力する（micromouse_log_visualizer用CSV形式）
 */
void log_print_all(void) {
    printf("=== Micromouse Log Data (CSV Format) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");
    
    // CSV形式でデータを出力（micromouse_log_visualizer用）
    // 形式: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l
    
    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        
        // CSV形式: timestamp,param1,param2,param3,param4,param5,param6,param7
        // timestamp = entry->timestamp - log_buffer.start_time (相対時間)
        // param1 = target_omega
        // param2 = actual_omega  
        // param3 = p_term_omega
        // param4 = i_term_omega
        // param5 = d_term_omega
        // param6 = motor_out_r
        // param7 = motor_out_l
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }
    
    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 速度ログ（log_buffer）をCSV出力
void log_print_velocity_all(void) {
    printf("=== Micromouse Log Data (CSV Format, VELOCITY) ===\n");
    printf("Total entries: %d\n", log_buffer.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

// 距離ログ（log_buffer2）をCSV出力
void log_print_distance_all(void) {
    printf("=== Micromouse Log Data (CSV Format, DISTANCE) ===\n");
    printf("Total entries: %d\n", log_buffer2.count);
    printf("CSV Format: timestamp,target_omega,actual_omega,p_term_omega,i_term_omega,d_term_omega,motor_out_r,motor_out_l\n");
    printf("--- CSV Data Start ---\n");

    uint16_t count = log_buffer2.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer2.count;
    uint16_t start = log_buffer2.count > MAX_LOG_ENTRIES ? log_buffer2.head : 0;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer2.entries[idx];
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer2.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }

    printf("--- CSV Data End ---\n");
    printf("=== End of Log ===\n");
}

/**
 * @brief ログデータを純粋なCSV形式で出力する（可視化ツール貼り付け用）
 */
void log_print_csv_only(void) {
    // 純粋なCSVデータのみを出力（説明行なし）
    // 可視化ツールのテキストエリアに直接貼り付け可能
    
    uint16_t count = log_buffer.count > MAX_LOG_ENTRIES ? MAX_LOG_ENTRIES : log_buffer.count;
    uint16_t start = log_buffer.count > MAX_LOG_ENTRIES ? log_buffer.head : 0;
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t idx = (start + i) % MAX_LOG_ENTRIES;
        volatile LogEntry *entry = &log_buffer.entries[idx];
        
        // CSV形式: timestamp,param1,param2,param3,param4,param5,param6,param7
        // timestampは相対時間（0始まり）
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               entry->timestamp - log_buffer.start_time,
               entry->target_omega,
               entry->actual_omega,
               entry->p_term_omega,
               entry->i_term_omega,
               entry->d_term_omega,
               entry->motor_out_r,
               entry->motor_out_l);
    }
}
