/*
 * sensor.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"

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
    ADC_task_counter = 0;

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);

    ICM20689_Init();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_adc_value
// 指定されたチャンネルのアナログ電圧値を取り出す
// 引数1：hadc …… 電圧値を取り出すチャンネルが属すADCのHandler
// 引数2：channel …… 電圧値を取り出すチャンネル
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_adc_value(ADC_HandleTypeDef *hadc, uint32_t channel) {
    HAL_ADC_Start(hadc);                  // AD変換を開始する
    HAL_ADC_PollForConversion(hadc, 150); // AD変換終了まで待機する
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
    // base_l = ad_l; // 現在の左側のセンサ値で決定
    // base_r = ad_r; // 現在の右側のセンサ値で決定
    base_l = WALL_CTRL_BASE_L;
    base_r = WALL_CTRL_BASE_R;
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
    //----前壁を見る----
    if (ad_fr > WALL_BASE_FR * sensor_kx || ad_fl > WALL_BASE_FL * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x88; // 壁情報を更新

        if (ad_fr > WALL_BASE_FR * 1.1 && ad_fl > WALL_BASE_FL * 1.1) {
            MF.FLAG.F_WALL = 1;
        } else {
            MF.FLAG.F_WALL = 0;
        }
    }
    //----右壁を見る----
    if (ad_r > WALL_BASE_R * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x44; // 壁情報を更新
        r_wall = true;
    } else {
        r_wall = false;
    }
    //----左壁を見る----
    if (ad_l > WALL_BASE_L * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x11; // 壁情報を更新
        l_wall = true;
    } else {
        l_wall = false;
    }
}

uint8_t read_byte(uint8_t reg) {
    uint8_t ret, val;
    HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    ret = reg | 0x80;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Receive(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_SET); // cs = 1;
    return val;
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
    HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Transmit(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_SET); // cs = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ICM20602_Init
//+++++++++++++++++++++++++++++++++++++++++++++++

void ICM20689_Init(void) {
    uint8_t who_am_i = 0;
    who_am_i = read_byte(0x75);              // check WHO_AM_I (0x75)
    printf("who_am_i = 0x%x\r\n", who_am_i); // Who am I = 0x98

    if (who_am_i != 0x98) { // recheck 0x98
        HAL_Delay(100);
        who_am_i = read_byte(0x98);

        if (who_am_i != 0x98) {
            printf("gyro_error\r\n\n");
            buzzer_beep(3000);
            buzzer_beep(3000);
            while (1) {
            }
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
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    }

    // 右センサ
    if (ad_r > WALL_BASE_R) {
        //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    // 前壁センサ
    if (ad_fr > WALL_BASE_FR && ad_fl > WALL_BASE_FL) {
        //HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_RESET);
    }
}

// 壁切れの判定
void wall_end(void) {
    get_wall_info();

    if (1) {
        // 直前に右壁が有ったとき
        if (MF.FLAG.R_WALL) {
            // 右壁がなくなっていたら
            if (!r_wall && wall_end_count < 2) {
                MF.FLAG.R_WALL_END = 1; // 右壁切れフラグを立てる
                wall_end_count = 3;
            }
        } else {
            MF.FLAG.R_WALL_END = 0;
        }
        if (MF.FLAG.L_WALL) {
            if (!l_wall && wall_end_count < 2) {
                MF.FLAG.L_WALL_END = 1; // 左壁切れフラグを立てる
                wall_end_count = 3;
            }
        } else {
            MF.FLAG.L_WALL_END = 0;
        }

        // 壁情報からフラグを管理
        if (r_wall) {
            MF.FLAG.R_WALL = 1;
        } else {
            MF.FLAG.R_WALL = 0;
        }
        if (l_wall) {
            MF.FLAG.L_WALL = 1;
        } else {
            MF.FLAG.L_WALL = 0;
        }
    } else {
        MF.FLAG.R_WALL = 0;
        MF.FLAG.L_WALL = 0;
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
    }
}