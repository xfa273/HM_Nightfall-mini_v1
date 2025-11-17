/*
 * interrupt.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "interrupt.h"
#include "logging.h"

// 非同期ADC DMA制御用ステート
// 8相スケジュール（250us刻み）でIR安定待ちを確保
// 0:RL OFF set, 1:RL OFF capture, 2:RL ON set, 3:RL ON capture,
// 4:FR/FL OFF set, 5:FR/FL OFF capture, 6:FR/FL ON set, 7:FR/FL ON capture
static volatile uint8_t s_adc_phase = 0;
static volatile uint8_t s_adc_inflight = 0;  // 0:idle, 1:converting (DMA中)

// センサ差分後の移動平均（SENSOR_MA_TAPSで切替、1で無効）
#if SENSOR_MA_TAPS < 1
#undef SENSOR_MA_TAPS
#define SENSOR_MA_TAPS 1
#endif

#if SENSOR_MA_TAPS > 1
static uint16_t s_ma_buf_r[SENSOR_MA_TAPS];
static uint16_t s_ma_buf_l[SENSOR_MA_TAPS];
static uint16_t s_ma_buf_fr[SENSOR_MA_TAPS];
static uint16_t s_ma_buf_fl[SENSOR_MA_TAPS];
static uint32_t s_ma_sum_r = 0, s_ma_sum_l = 0, s_ma_sum_fr = 0, s_ma_sum_fl = 0;
static uint8_t  s_ma_idx_rl = 0, s_ma_idx_frfl = 0;
static uint8_t  s_ma_count_rl = 0, s_ma_count_frfl = 0; // 立ち上がり中は有効長を短く
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        // TIM1: microsecond timer helper (used by tim1_wait_us)
    }

    if (htim->Instance == htim5.Instance) {
        // TIM5: 1kHz（制御系・IMU・エンコーダ等）

        // センサスケジューラ（DMA駆動）: 8相を1msごとに1ステップ進める
        if (!s_adc_inflight) {
            HAL_StatusTypeDef st = HAL_OK;
            uint8_t start_dma = 0;
            switch (s_adc_phase & 0x07u) {
                case 0: // RL OFF set
                    HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
                    break;
                case 1: // RL OFF capture
                    st = sensor_adc_dma_start(adc_dma_buf_off);
                    start_dma = 1;
                    break;
                case 2: // RL ON set
                    HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
                    break;
                case 3: // RL ON capture
                    st = sensor_adc_dma_start(adc_dma_buf_on);
                    start_dma = 1;
                    break;
                case 4: // FR/FL OFF set
                    HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
                    break;
                case 5: // FR/FL OFF capture
                    st = sensor_adc_dma_start(adc_dma_buf_off);
                    start_dma = 1;
                    break;
                case 6: // FR/FL ON set
                    HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
                    break;
                case 7: // FR/FL ON capture
                    st = sensor_adc_dma_start(adc_dma_buf_on);
                    start_dma = 1;
                    break;
                default:
                    s_adc_phase = 0;
                    break;
            }
            if (start_dma && st == HAL_OK) {
                s_adc_inflight = 1; // DMA進行中
            } else if (!start_dma) {
                // セット相はここで次相へ進める（キャプチャ相はDMA完了CBで進める）
                s_adc_phase = (uint8_t)((s_adc_phase + 1u) & 0x07u);
            }
        }

        // 前壁有無（簡易判定）
        if (ad_fr > WALL_BASE_FR * 1.1 && ad_fl > WALL_BASE_FL * 1.1) {
            MF.FLAG.F_WALL = 1;
        } else {
            MF.FLAG.F_WALL = 0;
        }

        // エンコーダ・IMU（センサDMA処理とは独立）
        read_encoder();
        read_IMU();

        // 横壁の立ち下がりによる壁切れ検知（探索用の壁判断とは独立）
        detect_wall_end();

        if (MF.FLAG.OVERRIDE == 0) {
            // 壁制御
            wall_PID();
            diagonal_CTRL();

            // 目標値の積算計算
            calculate_translation();
            calculate_rotation();
            velocity_PID();

            // 角度→角速度のPID
            // angle_PID();
            omega_PID();

            drive_motor();
        }
    }

    if (wall_end_count > 1) {
        wall_end_count--;
    }

    if (buzzer_count > 1) {
        buzzer_count--;
    } else if (buzzer_count) {
        // ブザーを止める
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        buzzer_count = 0;
    }

    // 新しいロギング機能の実装（プロファイル切替対応）
    if (MF.FLAG.GET_LOG_1) {
        log_capture_tick();
    }


} /* HAL_TIM_PeriodElapsedCallback */

//+++++++++++++++++++++++++++++++++++++++++++++++
// tim1_wait_us
// 1us毎にカウントアップするTIM5を使ってusマイクロ秒処理を止める関数。
// （whileループ中にオーバーフローが起こると機能しないのでTIM5タイマ更新割り込みハンドラ内のみで使用することを推奨する）
// 引数：us …… 処理を止めたいマイクロ秒
// 戻り値：無し
//+++++++++++++++++++++++++++++++++++++++++++++++
void tim1_wait_us(uint32_t us) {
    // TIM1 を 1us タイマとして使用（TIM5 ISR 内での待ち時間測定に同一タイマを使わない）
    // sensor_init() で HAL_TIM_Base_Start_IT(&htim1) 済みである前提
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); // 例: 1000（0..ARRでカウント）
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim1);

    uint32_t elapsed = 0;
    while (elapsed < us) {
        uint32_t now = __HAL_TIM_GET_COUNTER(&htim1);
        if (now >= start) {
            elapsed = now - start;
        } else {
            // ARR を跨いだ場合の経過時間
            elapsed = (arr + 1u - start) + now;
        }
    }
}
