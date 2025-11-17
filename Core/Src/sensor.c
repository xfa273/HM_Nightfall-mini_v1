/*
 * sensor.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "flash_params.h"
#include "sensor_distance.h"

// デバッグ出力ヘルパ：壁センサオフセットを表示
static void print_wall_offsets(const char* label)
{
    printf("%s: L=%u, R=%u, FR=%u, FL=%u\r\n",
           label,
           (unsigned)wall_offset_l,
           (unsigned)wall_offset_r,
           (unsigned)wall_offset_fr,
           (unsigned)wall_offset_fl);
}

// Save FL distance-domain warp (3 anchors) to Flash
HAL_StatusTypeDef sensor_front_warp_fl_save_to_flash(const float x_mm_est[3], const float y_mm_true[3])
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        flash_params_defaults(&p);
        // Carry over known globals for convenience
        p.base_l = base_l; p.base_r = base_r; p.base_f = base_f;
        p.wall_offset_r = wall_offset_r; p.wall_offset_l = wall_offset_l;
        p.wall_offset_fr = wall_offset_fr; p.wall_offset_fl = wall_offset_fl;
        p.imu_offset_z = imu_offset_z;
    }
    for (int i = 0; i < 3; ++i) {
        p.front_warp_fl_x_mm_est[i] = x_mm_est[i];
        p.front_warp_fl_y_mm_true[i] = y_mm_true[i];
    }
    p.flags |= FLASH_FLAG_FRONT_WARP_FL;
    HAL_StatusTypeDef st = flash_params_save(&p);
    if (st == HAL_OK) {
        printf("[Flash] Saved FL warp (3 anchors).\r\n");
    } else {
        printf("[Flash] Save FL warp failed. HAL=%d\r\n", st);
    }
    return st;
}

// Save FR distance-domain warp (3 anchors) to Flash
HAL_StatusTypeDef sensor_front_warp_fr_save_to_flash(const float x_mm_est[3], const float y_mm_true[3])
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        flash_params_defaults(&p);
        // Carry over known globals for convenience
        p.base_l = base_l; p.base_r = base_r; p.base_f = base_f;
        p.wall_offset_r = wall_offset_r; p.wall_offset_l = wall_offset_l;
        p.wall_offset_fr = wall_offset_fr; p.wall_offset_fl = wall_offset_fl;
        p.imu_offset_z = imu_offset_z;
    }
    for (int i = 0; i < 3; ++i) {
        p.front_warp_fr_x_mm_est[i] = x_mm_est[i];
        p.front_warp_fr_y_mm_true[i] = y_mm_true[i];
    }
    p.flags |= FLASH_FLAG_FRONT_WARP_FR;
    HAL_StatusTypeDef st = flash_params_save(&p);
    if (st == HAL_OK) {
        printf("[Flash] Saved FR warp (3 anchors).\r\n");
    } else {
        printf("[Flash] Save FR warp failed. HAL=%d\r\n", st);
    }
    return st;
}

// Save front-sum distance-domain warp (3 anchors) to Flash (preserving other parameters)
HAL_StatusTypeDef sensor_front_warp_save_to_flash(const float x_mm_est[3], const float y_mm_true[3])
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        flash_params_defaults(&p);
        // Carry over current known globals for user convenience
        p.base_l = base_l;
        p.base_r = base_r;
        p.base_f = base_f;
        p.wall_offset_r  = wall_offset_r;
        p.wall_offset_l  = wall_offset_l;
        p.wall_offset_fr = wall_offset_fr;
        p.wall_offset_fl = wall_offset_fl;
        p.imu_offset_z = imu_offset_z;
    }

    // Optionally apply distance-domain warp for FRONT SUM (3 anchors)
    if (p.flags & FLASH_FLAG_FRONT_WARP_VALID) {
        const float *x = p.front_warp_x_mm_est;
        const float *y = p.front_warp_y_mm_true;
        if (x[0] < x[1] && x[1] < x[2] && y[0] < y[1] && y[1] < y[2]) {
            sensor_distance_set_warp_front_sum_3pt(x, y);
            printf("[Flash] Applied front warp: x={%.2f,%.2f,%.2f} -> y={%.2f,%.2f,%.2f}\r\n",
                   x[0], x[1], x[2], y[0], y[1], y[2]);
        } else {
            printf("[Flash] Front warp invalid. Ignored.\r\n");
        }
    }

    // Optionally apply FL/FR individual warps
    if (p.flags & FLASH_FLAG_FRONT_WARP_FL) {
        const float *x = p.front_warp_fl_x_mm_est;
        const float *y = p.front_warp_fl_y_mm_true;
        if (x[0] < x[1] && x[1] < x[2] && y[0] < y[1] && y[1] < y[2]) {
            sensor_distance_set_warp_fl_3pt(x, y);
            printf("[Flash] Applied FL warp: x={%.2f,%.2f,%.2f} -> y={%.2f,%.2f,%.2f}\r\n",
                   x[0], x[1], x[2], y[0], y[1], y[2]);
        } else {
            printf("[Flash] FL warp invalid. Ignored.\r\n");
        }
    }
    if (p.flags & FLASH_FLAG_FRONT_WARP_FR) {
        const float *x = p.front_warp_fr_x_mm_est;
        const float *y = p.front_warp_fr_y_mm_true;
        if (x[0] < x[1] && x[1] < x[2] && y[0] < y[1] && y[1] < y[2]) {
            sensor_distance_set_warp_fr_3pt(x, y);
            printf("[Flash] Applied FR warp: x={%.2f,%.2f,%.2f} -> y={%.2f,%.2f,%.2f}\r\n",
                   x[0], x[1], x[2], y[0], y[1], y[2]);
        } else {
            printf("[Flash] FR warp invalid. Ignored.\r\n");
        }
    }

    for (int i = 0; i < 3; ++i) {
        p.front_warp_x_mm_est[i] = x_mm_est[i];
        p.front_warp_y_mm_true[i] = y_mm_true[i];
    }
    p.flags |= FLASH_FLAG_FRONT_WARP_VALID;

    HAL_StatusTypeDef st = flash_params_save(&p);
    if (st == HAL_OK) {
        printf("[Flash] Saved front warp (3 anchors).\r\n");
    } else {
        printf("[Flash] Save front warp failed. HAL=%d\r\n", st);
    }
    return st;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_calibrate_wall_ctrl_base_and_save
// 壁制御の基準値（base_l/base_r/base_f）を一定時間平均して測定し、Flashへ保存
// 引数: duration_ms 測定時間[ms] （例: 1500）
// 前提: 通路の中央に停止し、左右に壁がある状態で実行
// 戻り: HAL_StatusTypeDef（保存の成否。測定自体は常に実施）
//+++++++++++++++++++++++++++++++++++++++++++++++
HAL_StatusTypeDef sensor_calibrate_wall_ctrl_base_and_save(uint32_t duration_ms)
{
    if (duration_ms < 200) duration_ms = 200; // 最低200ms

    uint32_t t0 = HAL_GetTick();
    uint32_t sum_l = 0, sum_r = 0, sum_f = 0;
    uint32_t n = 0;

    while ((HAL_GetTick() - t0) < duration_ms) {
        // ad_* は割り込みで更新済みの差分値（オフセット補正・移動平均後）
        sum_l += (uint32_t)ad_l;
        sum_r += (uint32_t)ad_r;
        sum_f += (uint32_t)(ad_fl + ad_fr);
        n++;
        HAL_Delay(5);
    }

    if (n == 0) n = 1; // ゼロ除算保護

    // 平均値を適用
    base_l = (uint16_t)(sum_l / n);
    base_r = (uint16_t)(sum_r / n);
    base_f = (uint16_t)(sum_f / n);

    printf("[CAL] base_l=%u, base_r=%u, base_f=%u (n=%lu)\r\n",
           (unsigned)base_l, (unsigned)base_r, (unsigned)base_f, (unsigned long)n);

    // 保存
    HAL_StatusTypeDef st = sensor_params_save_to_flash();
    if (st == HAL_OK) {
        printf("[CAL] Saved to Flash successfully.\r\n");
        buzzer_beep(1000);
    } else {
        printf("[CAL] Flash save failed. HAL=%d\r\n", st);
        buzzer_beep(3000);
    }
    return st;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ADC DMA helpers
//+++++++++++++++++++++++++++++++++++++++++++++++
HAL_StatusTypeDef sensor_adc_dma_start(volatile uint16_t *dst)
{
    // Ensure previous DMA is stopped
    (void)HAL_ADC_Stop_DMA(&hadc1);

    // Start regular group conversion with DMA into provided buffer (9 ranks)
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dst, 9);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_init
// センサ系の変数の初期化，ADコンバータの設定とセンサ値取得に使用するタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_init(void) {
    //====センサ系の変数の初期化====
    tp = 0;
    ad_l = ad_r = ad_fr = ad_fl = 0;
    base_l = base_r = base_f = 0;

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);

    // Initialize default distance LUTs for sensors first
    sensor_distance_init();

    // センサのオフセット値をフラッシュから読み込み（有効なら使用）
    // 失敗した場合のみ測定を実施
    bool loaded = sensor_params_load_from_flash();
    if (!loaded) {
        get_sensor_offsets();
        print_wall_offsets("Wall offsets (measured at init)");
    } else {
        print_wall_offsets("Wall offsets (loaded at init)");
    }

    IMU_Init_Auto();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// Flash parameter I/O (separate from maze EEPROM emulation)
// Save/Load sensor baselines and offsets to FLASH_SECTOR_10
//+++++++++++++++++++++++++++++++++++++++++++++++

bool sensor_params_load_from_flash(void)
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        return false;
    }

    // Apply loaded values to globals
    base_l = (uint16_t)p.base_l;
    base_r = (uint16_t)p.base_r;
    base_f = (uint16_t)p.base_f;

    wall_offset_r  = (uint16_t)p.wall_offset_r;
    wall_offset_l  = (uint16_t)p.wall_offset_l;
    wall_offset_fr = (uint16_t)p.wall_offset_fr;
    wall_offset_fl = (uint16_t)p.wall_offset_fl;

    imu_offset_z = p.imu_offset_z;

    // Optionally apply front-distance LUT (3 points) if present
    if (p.flags & FLASH_FLAG_FRONT_LUT_VALID) {
        // Validate monotonicity to satisfy LUT API
        const uint16_t *mm = p.front_lut_mm;
        const uint16_t *fl = p.front_lut_fl;
        const uint16_t *fr = p.front_lut_fr;
        bool ok = true;
        if (!(mm[0] < mm[1] && mm[1] < mm[2])) ok = false;
        if (!(fl[0] > fl[1] && fl[1] > fl[2])) ok = false;
        if (!(fr[0] > fr[1] && fr[1] > fr[2])) ok = false;
        if (ok) {
            (void)sensor_distance_set_lut_fl(mm, fl, 3);
            (void)sensor_distance_set_lut_fr(mm, fr, 3);
            uint16_t sum3[3];
            for (int i = 0; i < 3; ++i) {
                uint32_t s = (uint32_t)fl[i] + (uint32_t)fr[i];
                sum3[i] = (uint16_t)((s > 0xFFFFu) ? 0xFFFFu : s);
            }
            (void)sensor_distance_set_lut_front_sum(mm, sum3, 3);
            printf("[Flash] Applied front LUT: mm={%u,%u,%u}, FL={%u,%u,%u}, FR={%u,%u,%u}\r\n",
                   (unsigned)mm[0], (unsigned)mm[1], (unsigned)mm[2],
                   (unsigned)fl[0], (unsigned)fl[1], (unsigned)fl[2],
                   (unsigned)fr[0], (unsigned)fr[1], (unsigned)fr[2]);
        } else {
            printf("[Flash] Front LUT invalid (monotonicity). Ignored.\r\n");
        }
    }

    return true;
}

HAL_StatusTypeDef sensor_params_save_to_flash(void)
{
    flash_params_t p;
    flash_params_defaults(&p);

    // Capture current values
    p.base_l = base_l;
    p.base_r = base_r;
    p.base_f = base_f;

    p.wall_offset_r  = wall_offset_r;
    p.wall_offset_l  = wall_offset_l;
    p.wall_offset_fr = wall_offset_fr;
    p.wall_offset_fl = wall_offset_fl;

    p.imu_offset_z = imu_offset_z;

    return flash_params_save(&p);
}

// Save 3-point front distance LUT to Flash (preserving other parameters)
HAL_StatusTypeDef sensor_front_lut_save_to_flash(const uint16_t mm[3], const uint16_t fl[3], const uint16_t fr[3])
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        // If nothing stored yet, start from defaults and copy known runtime values
        flash_params_defaults(&p);
        // Carry over current offsets/bases if globals hold them
        p.base_l = base_l;
        p.base_r = base_r;
        p.base_f = base_f;
        p.wall_offset_r  = wall_offset_r;
        p.wall_offset_l  = wall_offset_l;
        p.wall_offset_fr = wall_offset_fr;
        p.wall_offset_fl = wall_offset_fl;
        p.imu_offset_z = imu_offset_z;
    }

    // Copy LUT (3 points)
    for (int i = 0; i < 3; ++i) {
        p.front_lut_mm[i] = mm[i];
        p.front_lut_fl[i] = fl[i];
        p.front_lut_fr[i] = fr[i];
    }
    p.flags |= FLASH_FLAG_FRONT_LUT_VALID;

    HAL_StatusTypeDef st = flash_params_save(&p);
    if (st == HAL_OK) {
        printf("[Flash] Saved front LUT (3pt) successfully.\r\n");
    } else {
        printf("[Flash] Save front LUT failed. HAL=%d\r\n", st);
    }
    return st;
}

// 任意タイミングで再測定してフラッシュへ保存するヘルパ（壁センサのみ）
HAL_StatusTypeDef sensor_recalibrate_and_save(void)
{
    // 壁センサオフセットの再測定（壁無し・静止条件で実施すること）
    get_sensor_offsets();
    // 制御基準値の更新（機体姿勢が適正な状態で）
    (void)get_base();
    // 測定したオフセットを表示
    print_wall_offsets("Wall offsets (measured)");

    return sensor_params_save_to_flash();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_adc_value
// 指定されたチャンネルのアナログ電圧値を取り出す
// 引数1：hadc …… 電圧値を取り出すチャンネルが属すADCのHandler
// 引数2：channel …… 電圧値を取り出すチャンネル
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_adc_value(ADC_HandleTypeDef *hadc, uint32_t channel) {
    (void)channel;
    HAL_ADC_Start(hadc);                  // AD変換を開始する
    HAL_ADC_PollForConversion(hadc, 300); // AD変換終了まで待機する
    return (HAL_ADC_GetValue(hadc) * K_SENSOR); // AD変換結果を取得する
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_r
// Sensor R の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_r(void) { return get_adc_value(&hadc1, ADC_CHANNEL_1); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_fr
// Sensor FR の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_fr(void) { return get_adc_value(&hadc1, ADC_CHANNEL_0); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_fl
// Sensor FL の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_fl(void) { return get_adc_value(&hadc1, ADC_CHANNEL_2); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_l
// Sensor L の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_l(void) { return get_adc_value(&hadc1, ADC_CHANNEL_3); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_battery_value
// VOL Check の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_battery_value(void) { return get_adc_value(&hadc1, ADC_CHANNEL_10); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_base
// 壁制御用の基準値を取得する
// 引数：なし
// 戻り値：理想的な値を取得できたか　1:できた　0:できなかった
//+++++++++++++++++++++++++++++++++++++++++++++++
uint8_t get_base() {
    uint8_t res = 1; // 理想的な値を取得できたか

    //----制御用の基準を取得----
    // 既にフラッシュから読み込まれている（もしくは校正モードで測定済みの）
    // base_l/base_r を優先使用する。未設定(0)の場合は params.h の既定値を利用。
    if (base_l == 0) {
        base_l = WALL_CTRL_BASE_L;
    }
    if (base_r == 0) {
        base_r = WALL_CTRL_BASE_R;
    }
    // 前壁の基準は都度、現在のFR+FLの合計を用いる（主に探索用の参照）
    base_f = ad_fl + ad_fr;

    // printf("base: %d,%d\n", base_l, base_r);

    return res; // 理想的な値を取得できたかを返す
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_wall_info
// 壁情報を取得する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void get_wall_info() {

    // センサ補正値が未定義なら1.0にする
    if (sensor_kx < 0.3 || sensor_kx > 2.0) {
        sensor_kx = 1.0;
    }
    //----壁情報の初期化----
    wall_info = 0x00; // 壁情報を初期化
    //----距離ベースで前壁を見る----
    {
        float d_fr = sensor_distance_from_fr(ad_fr);
        float d_fl = sensor_distance_from_fl(ad_fl);
        // どちらかがしきい値以内なら前壁あり
        if ((d_fr <= FRONT_DETECT_DIST_MM) || (d_fl <= FRONT_DETECT_DIST_MM)) {
            wall_info |= 0x88;
        }
    }
    //----距離ベースで右壁を見る----
    {
        float d_r = sensor_distance_from_r(ad_r);
        if (d_r <= WALL_DETECT_DIST_R_MM) {
            wall_info |= 0x44;
            r_wall = true;
        } else {
            r_wall = false;
        }
    }
    //----距離ベースで左壁を見る----
    {
        float d_l = sensor_distance_from_l(ad_l);
        if (d_l <= WALL_DETECT_DIST_L_MM) {
            wall_info |= 0x11;
            l_wall = true;
        } else {
            l_wall = false;
        }
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ISM330DHCX support
//+++++++++++++++++++++++++++++++++++++++++++++++

// 読み出しヘルパ（リトルエンディアン16bit）
static int16_t read16_le(uint8_t reg_l) {
    uint8_t lo = read_byte(reg_l);
    uint8_t hi = read_byte(reg_l + 1);
    return (int16_t)((int16_t)hi << 8 | lo);
}

// ISM330DHCX 初期化
void ISM330_Init(void) {
    uint8_t who = read_byte(0x0F); // WHO_AM_I
    printf("ISM330 WHO_AM_I = 0x%02x\r\n", who); // 期待値: 0x6B

    if (who != 0x6B) {
        // 必要最小限の表示のみ。失敗時はブザーのみ。
        buzzer_beep(2500);
        // 検出失敗でも継続は可能だが、以降の読み出しは無効
    }

    // CTRL3_C (0x12): BDU=1, IF_INC=1
    // BDU(6)=1, IF_INC(2)=1 -> 0b0100_0100 = 0x44
    write_byte(0x12, 0x44);
    HAL_Delay(10);

    // CTRL2_G (0x11): Gyro設定
    // ODR_G=833Hz(0x7<<4)=0x70
    // FS_4000=1で±4000 dpsに設定（FS_Gは未使用扱い）
    // 推奨: 0x71 (= 0x70 | 0x01)
    write_byte(0x11, 0x71);
    // 以前の設定（参考・ロールバック用）: ±2000 dps
    // write_byte(0x11, 0x7C); // ODR=833Hz, FS_G=2000 dps
    HAL_Delay(10);

    // CTRL1_XL (0x10): ODR_XL=833Hz, FS_XL=±16g -> 0x7C
    write_byte(0x10, 0x7C);
    HAL_Delay(10);

    set_flag = 1;
}

// 感度換算
static inline float ism330_gyro_dps(int16_t raw) {
    // FS=4000 dps => 140 mdps/LSB = 0.14 dps/LSB
    return (float)raw * 0.14f; // [deg/s]
    // 参考（以前の設定）: FS=2000 dps => 70 mdps/LSB
    // return (float)raw * 0.07f; // [deg/s]
}
static inline float ism330_accel_g(int16_t raw) { // FS=16g => 0.488 mg/LSB
    return (float)raw * 0.000488f; // [g]
}

void ISM330_DataUpdate(void) {
    if (set_flag == 1) {
        // Gyro OUT*_G: X 0x22/0x23, Y 0x24/0x25, Z 0x26/0x27
        float gx = ism330_gyro_dps(read16_le(0x22));
        float gy = ism330_gyro_dps(read16_le(0x24));
        float gz = ism330_gyro_dps(read16_le(0x26));

        // Accel OUT*_A: X 0x28/0x29, Y 0x2A/0x2B, Z 0x2C/0x2D
        float ax = ism330_accel_g(read16_le(0x28));
        float ay = ism330_accel_g(read16_le(0x2A));
        float az = ism330_accel_g(read16_le(0x2C));

        // 既存座標系との整合（ICM実装と同様の符号を暫定適用）
        omega_x_raw = -gx;
        omega_y_raw = gy;
        omega_z_raw = gz;

        accel_x_raw = -ax;
        accel_y_raw = ay;
        accel_z_raw = az;

        // オフセット補正
        omega_x_true = omega_x_raw - omega_x_offset;
        omega_y_true = omega_y_raw - omega_y_offset;
        omega_z_true = omega_z_raw - omega_z_offset;
        accel_x_true = accel_x_raw - accel_x_offset;
        accel_y_true = accel_y_raw - accel_y_offset;
        accel_z_true = accel_z_raw - accel_z_offset;
    }
}

// IMU自動検出と初期化
void IMU_Init_Auto(void) {
    // まずICM20689を試す
    uint8_t who_icm = read_byte(0x75);
    if (who_icm == 0x98) {
        imu_model = 1; // ICM20689
        ICM20689_Init();
        return;
    }

    // 次にISM330DHCXを試す
    uint8_t who_ism = read_byte(0x0F);
    if (who_ism == 0x6B) {
        imu_model = 2; // ISM330DHCX
        ISM330_Init();
        return;
    }

    // いずれも検出できない場合はエラー
    imu_model = 0;
    buzzer_beep(3000);
}

// アクティブなIMUから最新値を取得（ラッパー）
void IMU_DataUpdate(void) {
    if (imu_model == 1) {
        ICM20689_DataUpdate();
    } else if (imu_model == 2) {
        ISM330_DataUpdate();
    } else {
        // 未検出時は何もしない
    }
}

uint8_t read_byte(uint8_t reg) {
    uint8_t ret, val;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    ret = reg | 0x80;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Receive(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // cs = 1;
    return val;
}

// SPI3を動的に再初期化（モード/速度切替用）
static void spi3_reinit(uint32_t pol, uint32_t phase, uint32_t presc) {
    HAL_SPI_DeInit(&hspi3);
    hspi3.Init.CLKPolarity = pol;
    hspi3.Init.CLKPhase = phase;
    hspi3.Init.BaudRatePrescaler = presc;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        printf("HAL_SPI_Init failed in debug reinit.\r\n");
    }
}

// WHO_AM_IをSPIモードを切替えつつ読み取って表示（デバッグ用）
void IMU_ProbeWHOAMI_Debug(void) {
    uint8_t icm_reg = 0x75; // ICM20689 WHO_AM_I
    uint8_t ism_reg = 0x0F; // ISM330DHCX WHO_AM_I

    printf("=== IMU WHO_AM_I probe start ===\r\n");

    // 電源投入直後の安定待ち
    HAL_Delay(20);

    // 低速で検証（ライン品質の影響を減らす）
    // Mode 3 (CPOL=1, CPHA=2)
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m3 = read_byte(icm_reg);
    uint8_t ism_m3 = read_byte(ism_reg);
    printf("Mode3/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m3, ism_m3);

    // Mode 0 (CPOL=0, CPHA=1)
    spi3_reinit(SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m0 = read_byte(icm_reg);
    uint8_t ism_m0 = read_byte(ism_reg);
    printf("Mode0/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m0, ism_m0);

    // Mode 1 (CPOL=0, CPHA=2)
    spi3_reinit(SPI_POLARITY_LOW, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m1 = read_byte(icm_reg);
    uint8_t ism_m1 = read_byte(ism_reg);
    printf("Mode1/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m1, ism_m1);

    // Mode 2 (CPOL=1, CPHA=1)
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m2 = read_byte(icm_reg);
    uint8_t ism_m2 = read_byte(ism_reg);
    printf("Mode2/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m2, ism_m2);

    // 既定へ復帰（Mode3/Presc32）
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_32);
    printf("=== IMU WHO_AM_I probe end ===\r\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// write_byte関数
/*SPI Operational Features
1.Data is delivered MSB first and LSB last
2.Data is latched on the rising edge of SCLK
3.Data should be transitioned on the falling edge of SPC
4.The maximum frequency of SPC is 10MHz
5.SPI read and write operations are completed in 16 or more clock cycles(two or
more bytes.) The first byte conains the SPI Adress The following bytes contain
the SPI data The first bit of the first byte contains the Read/Write bit and
indicates the Read(1) or Write(0) operation. The following 7 bits is the
Resister Address.
*/
//+++++++++++++++++++++++++++++++++++++++++++++++

void write_byte(uint8_t reg, uint8_t val) {
    uint8_t ret;
    ret = reg & 0x7F;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Transmit(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // cs = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ICM20602_Init
//+++++++++++++++++++++++++++++++++++++++++++++++

void ICM20689_Init(void) {
    uint8_t who_am_i = 0;
    who_am_i = read_byte(0x75);              // check WHO_AM_I (0x75)
    printf("ICM20689 WHO_AM_I = 0x%02x\r\n", who_am_i); // 期待値: 0x98

    if (who_am_i != 0x98) { // recheck 0x98
        HAL_Delay(100);
        who_am_i = read_byte(0x75);
        if (who_am_i != 0x98) {
            // 必要最小限の表示のみ。失敗時はブザーのみ。
            buzzer_beep(3000);
            buzzer_beep(3000);
        }
    }

    // PWR_MIGHT_1 0x6B
    write_byte(0x6B, 0x00); // Set pwr might
    HAL_Delay(50);
    // PWR_MIGHT_2 0x6C
    write_byte(0x6C, 0x00);
    HAL_Delay(50);
    // set gyro config
    // GYRO_CONFIG 0x1B
    write_byte(0x1B, 0x18); // use 2000 dps
    HAL_Delay(50);
    // ACCEL_CONFIG 0x1C
    write_byte(0x1B, 0x18); // use pm 16g
    HAL_Delay(50);

    set_flag = 1;
}

float ICM20689_GYRO_READ(uint8_t H_reg) {
    int16_t data = (int16_t)(((uint8_t)read_byte(H_reg) << 8) |
                             (uint8_t)read_byte(H_reg + 1));
    float omega =
        (float)(data / 16.4f); //[deg/s] FS_SEL=3-> Scale Factor=16.4[LSB/(dps)]
    return omega;
}

float ICM20689_ACCEL_READ(uint8_t H_reg) {
    int16_t data = (int16_t)(((uint8_t)read_byte(H_reg) << 8) |
                             (uint8_t)read_byte(H_reg + 1));
    float accel = (float)(data / 2048.0f);
    return accel;
}

void ICM20689_DataUpdate(void) {
    if (set_flag == 1) {
        // get yawrate
        omega_x_raw = -1 * ICM20689_GYRO_READ(0x43);
        omega_y_raw = ICM20689_GYRO_READ(0x45);
        omega_z_raw = ICM20689_GYRO_READ(0x47);

        // get accel

        accel_x_raw = -1 * ICM20689_ACCEL_READ(0x3B);
        accel_y_raw = ICM20689_ACCEL_READ(0x3D);
        accel_z_raw = ICM20689_ACCEL_READ(0x3F);

        // True Value(Consider Offset)
        omega_x_true = omega_x_raw - omega_x_offset;
        omega_y_true = omega_y_raw - omega_y_offset;
        omega_z_true = omega_z_raw - omega_z_offset;
        accel_x_true = accel_x_raw - accel_x_offset;
        accel_y_true = accel_y_raw - accel_y_offset;
        accel_z_true = accel_z_raw - accel_z_offset;
    }
}

// センサオフセット値を起動時に複数回測定して平均化する
// 定義: 壁無し・静止状態での LED 同期差分 (ad_on - ad_off) の平均値を
//       LED 由来の漏れ/筐体内反射の定常成分として wall_offset_* に保存する。
// これによりランタイムの ad = (ad_on - ad_off) - wall_offset_* が環境光に頑健になる。
void get_sensor_offsets(void) {
    const int NUM_SAMPLES = 10;  // 測定回数
    int i;
    uint32_t sum_r = 0, sum_l = 0, sum_fr = 0, sum_fl = 0;

    wall_offset_l = 0;
    wall_offset_r = 0;
    wall_offset_fr = 0;
    wall_offset_fl = 0;
    
    // 起動時ログを抑制
    
    // interrupt.c ですでに実装されている ADC の LED ON/OFF 差分更新を使用する
    // 割り込み処理が複数回走るのを待つ
    for (i = 0; i < NUM_SAMPLES; i++) {
        // LED ON/OFF 差分（ad_on - ad_off）のみを加算（負値は0に丸め）
        int32_t dr  = (int32_t)ad_r_raw  - (int32_t)ad_r_off;  if (dr  < 0) dr  = 0;
        int32_t dl  = (int32_t)ad_l_raw  - (int32_t)ad_l_off;  if (dl  < 0) dl  = 0;
        int32_t dfr = (int32_t)ad_fr_raw - (int32_t)ad_fr_off; if (dfr < 0) dfr = 0;
        int32_t dfl = (int32_t)ad_fl_raw - (int32_t)ad_fl_off; if (dfl < 0) dfl = 0;

        sum_r  += (uint32_t)dr;
        sum_l  += (uint32_t)dl;
        sum_fr += (uint32_t)dfr;
        sum_fl += (uint32_t)dfl;

        HAL_Delay(10); // 少し待機して次のサンプルを取得
    }
    
    // 平均値を計算して設定
    // LED 同期差分の基本成分をオフセット値として設定
    // 新しい変数にオフセット値を保存（割り込みで上書きされない）
    wall_offset_r  = (uint16_t)(sum_r  / (uint32_t)NUM_SAMPLES);
    wall_offset_l  = (uint16_t)(sum_l  / (uint32_t)NUM_SAMPLES);
    wall_offset_fr = (uint16_t)(sum_fr / (uint32_t)NUM_SAMPLES);
    wall_offset_fl = (uint16_t)(sum_fl / (uint32_t)NUM_SAMPLES);
    
    // 起動時ログを抑制
}

// 静止状態でIMUのオフセット値を取得
void IMU_GetOffset(void) {
    HAL_Delay(400);
    omega_x_offset = omega_x_raw;
    omega_y_offset = omega_y_raw;
    omega_z_offset = omega_z_raw;
    accel_x_offset = accel_x_raw;
    accel_y_offset = accel_y_raw;
    accel_z_offset = accel_z_raw;

    printf("offset: %f, %f, %f\n", omega_x_offset, omega_y_offset,
           omega_z_offset);
}

// センサからの壁判断をLEDで表示
void indicate_sensor(void) {

    // 左センサ
    if (ad_l > WALL_BASE_L) {
        //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    }

    // 右センサ
    if (ad_r > WALL_BASE_R) {
        //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    // 前壁センサ
    if (ad_fr > WALL_BASE_FR && ad_fl > WALL_BASE_FL) {
        //HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_RESET);
    }
}

// 壁切れの判定（現状は未使用のため空実装）
void wall_end(void) {}

//+++++++++++++++++++++++++++++++++++++++++++++++
// detect_wall_end
// 横壁センサの「有り → 無し」の立ち下がりで壁切れを検出
// 既存の get_wall_info とは独立に動作し、r_wall/l_wall は変更しない
// 検出直後にブザーを鳴らし、R_WALL_END / L_WALL_END を即座にクリア
//+++++++++++++++++++++++++++++++++++++++++++++++
void detect_wall_end(void) {
    // 有効なしきい値係数を決定
    float kx = sensor_kx;
    if (kx < 0.3f || kx > 2.0f) {
        kx = 1.0f;
    }

    // 現在の横壁判定（get_wall_info を呼ばず、ここで独立判定）
    // 壁切れ専用しきい値（WALL_END_THR_R/L）を使用
    bool r_has = (ad_r > (uint16_t)(WALL_END_THR_R * kx));
    bool l_has = (ad_l > (uint16_t)(WALL_END_THR_L * kx));

    // 直前状態（関数ローカルに保持）
    static uint8_t s_inited = 0;
    static bool s_prev_r = false;
    static bool s_prev_l = false;

    if (!s_inited) {
        s_prev_r = r_has;
        s_prev_l = l_has;
        s_inited = 1;
        return;
    }

    // ゲート条件: 最短走行中(SCND)かつWALL_ENDアーム中のみ最終フラグを立てる
    const bool gate_on = (MF.FLAG.SCND && MF.FLAG.WALL_END);

    // 右側の立ち下がり（有→無）
    if (s_prev_r && !r_has) {
        if (gate_on) {
            MF.FLAG.R_WALL_END = 1; // 最終フラグ: 消費側で明示的にクリア
            buzzer_interrupt(2000); // 確認用（SUCTION時は抑制）
        }
    }

    // 左側の立ち下がり（有→無）
    if (s_prev_l && !l_has) {
        if (gate_on) {
            MF.FLAG.L_WALL_END = 1; // 最終フラグ: 消費側で明示的にクリア
            buzzer_interrupt(2400);
        }
    }

    // 状態を更新
    s_prev_r = r_has;
    s_prev_l = l_has;
}
