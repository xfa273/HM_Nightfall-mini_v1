#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include "stdint.h"

/**
 * @brief ロギングシステムを初期化する
 */
void log_init(void);

/**
 * @brief ロギングを開始する
 * @param start_time 開始時間（ms）
 */
void log_start(uint32_t start_time);

/**
 * @brief ロギングを停止する
 */
void log_stop(void);

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
                  float motor_out_r, float motor_out_l, uint32_t timestamp);

/**
 * @brief ログデータをシリアルポートに出力する
 */
void log_print_all(void);

/**
 * @brief ログデータを純粋なCSV形式で出力する（可視化ツール貼り付け用）
 */
void log_print_csv_only(void);

#endif /* INC_LOGGING_H_ */
