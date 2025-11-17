/*
 * interrupt.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "interrupt.h"
#include "logging.h"
#include "params.h"

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
    if (htim->Instance == htim6.Instance) {
        // TIM6: センサスケジューラ（DMA駆動）: 4kHz tick / 8相 = 500Hz per full cycle
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
    }
    if (htim->Instance == htim5.Instance) {
        // TIM5: 1kHz（制御系・IMU・エンコーダ等）

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

        // 新しいロギング機能の実装（1ms固定。TIM6等の高速タイマでは呼ばない）
        if (MF.FLAG.GET_LOG_1) {
            log_capture_tick();
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

} /* HAL_TIM_PeriodElapsedCallback */

// ADC DMA 完了コールバック（DMA2_Stream0 IRQ内でHALにより呼ばれる）
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    // Rankマップ: 0,2=R; 1,3=L; 4,6=FR; 5,7=FL; 8=BAT
    switch (s_adc_phase & 0x07u) {
        case 3: { // RL ON capture 完了 -> 差分算出
            uint16_t r_off = (uint16_t)(((uint32_t)adc_dma_buf_off[0] + (uint32_t)adc_dma_buf_off[2]) / 2u);
            uint16_t l_off = (uint16_t)(((uint32_t)adc_dma_buf_off[1] + (uint32_t)adc_dma_buf_off[3]) / 2u);
            uint16_t r_on  = (uint16_t)(((uint32_t)adc_dma_buf_on[0]  + (uint32_t)adc_dma_buf_on[2])  / 2u);
            uint16_t l_on  = (uint16_t)(((uint32_t)adc_dma_buf_on[1]  + (uint32_t)adc_dma_buf_on[3])  / 2u);

            ad_r_off = r_off;
            ad_l_off = l_off;
            ad_r_raw = r_on;
            ad_l_raw = l_on;
            ad_r = max((int)ad_r_raw - (int)ad_r_off - (int)wall_offset_r, 0);
            ad_l = max((int)ad_l_raw - (int)ad_l_off - (int)wall_offset_l, 0);

#if SENSOR_MA_TAPS > 1
            // RLグループ移動平均
            uint16_t v_r = (uint16_t)ad_r;
            uint16_t v_l = (uint16_t)ad_l;
            uint16_t old_r = s_ma_buf_r[s_ma_idx_rl];
            uint16_t old_l = s_ma_buf_l[s_ma_idx_rl];
            s_ma_sum_r += v_r - old_r;
            s_ma_sum_l += v_l - old_l;
            s_ma_buf_r[s_ma_idx_rl] = v_r;
            s_ma_buf_l[s_ma_idx_rl] = v_l;
            if (s_ma_count_rl < SENSOR_MA_TAPS) {
                s_ma_count_rl++;
            }
            s_ma_idx_rl++;
            if (s_ma_idx_rl >= SENSOR_MA_TAPS) s_ma_idx_rl = 0;
            ad_r = (int)(s_ma_sum_r / s_ma_count_rl);
            ad_l = (int)(s_ma_sum_l / s_ma_count_rl);
#endif

            // LEDをOFFに戻す
            HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
            
            break;
        }
        case 7: { // FR/FL ON capture 完了 -> 差分算出
            uint16_t fr_off = (uint16_t)(((uint32_t)adc_dma_buf_off[4] + (uint32_t)adc_dma_buf_off[6]) / 2u);
            uint16_t fl_off = (uint16_t)(((uint32_t)adc_dma_buf_off[5] + (uint32_t)adc_dma_buf_off[7]) / 2u);
            uint16_t fr_on  = (uint16_t)(((uint32_t)adc_dma_buf_on[4]  + (uint32_t)adc_dma_buf_on[6])  / 2u);
            uint16_t fl_on  = (uint16_t)(((uint32_t)adc_dma_buf_on[5]  + (uint32_t)adc_dma_buf_on[7])  / 2u);

            ad_fr_off = fr_off;
            ad_fl_off = fl_off;
            ad_fr_raw = fr_on;
            ad_fl_raw = fl_on;
            ad_fr = max((int)ad_fr_raw - (int)ad_fr_off - (int)wall_offset_fr, 0);
            ad_fl = max((int)ad_fl_raw - (int)ad_fl_off - (int)wall_offset_fl, 0);

#if SENSOR_MA_TAPS > 1
            // FR/FLグループ移動平均
            uint16_t v_fr = (uint16_t)ad_fr;
            uint16_t v_fl = (uint16_t)ad_fl;
            uint16_t old_fr = s_ma_buf_fr[s_ma_idx_frfl];
            uint16_t old_fl = s_ma_buf_fl[s_ma_idx_frfl];
            s_ma_sum_fr += v_fr - old_fr;
            s_ma_sum_fl += v_fl - old_fl;
            s_ma_buf_fr[s_ma_idx_frfl] = v_fr;
            s_ma_buf_fl[s_ma_idx_frfl] = v_fl;
            if (s_ma_count_frfl < SENSOR_MA_TAPS) {
                s_ma_count_frfl++;
            }
            s_ma_idx_frfl++;
            if (s_ma_idx_frfl >= SENSOR_MA_TAPS) s_ma_idx_frfl = 0;
            ad_fr = (int)(s_ma_sum_fr / s_ma_count_frfl);
            ad_fl = (int)(s_ma_sum_fl / s_ma_count_frfl);
#endif

            // LEDをOFFに戻す
            HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);

            // バッテリー更新（OFF側の最新値）
            ad_bat = adc_dma_buf_off[8];
            
            break;
        }
        default:
            break;
    }

    // 次回に進める（capture相はCBで進める）
    s_adc_inflight = 0;
    s_adc_phase = (uint8_t)((s_adc_phase + 1u) & 0x07u);
}

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
