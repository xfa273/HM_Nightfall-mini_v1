#include "global.h"
#include "interrupt.h"
#include "stdio.h"

/**
 * @brief ロギングシステムを初期化する
 */
void log_init(void) {
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 0;
    log_buffer.start_time = 0;
}

/**
 * @brief ロギングを開始する
 * @param start_time 開始時間（ms）
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_start(uint32_t start_time) {
    log_buffer.head = 0;
    log_buffer.count = 0;
    log_buffer.logging_active = 1;
    log_buffer.start_time = start_time;
    
    // ロギングフラグを設定
    MF.FLAG.GET_LOG_1 = 1;
}

/**
 * @brief ロギングを停止する
 * @note この関数はMF.FLAG.GET_LOG_1フラグも操作します
 */
void log_stop(void) {
    log_buffer.logging_active = 0;
    
    // ロギングフラグをクリア
    MF.FLAG.GET_LOG_1 = 0;
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
