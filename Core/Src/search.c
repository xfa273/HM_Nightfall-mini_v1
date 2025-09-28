/*
 * search.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>

//+++++++++++++++++++++++++++++++++++++++++++++++
// search_init
// 探索系の変数とマップの初期化をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void search_init(void) {
    //====マウスフラグの初期化===
    MF.FLAGS = 0; // フラグクリア

    //====探索系の変数の初期化====
    goal_x = GOAL_X; // GOAL_Xはglobal.hにマクロ定義あり
    goal_y = GOAL_Y; // GOAL_Yはglobal.hにマクロ定義あり
    map_Init();      // マップの初期化
    mouse.x = 0;
    mouse.y = 0;   // 現在地の初期化
    mouse.dir = 0; // マウスの向きの初期化
    search_end = false;
    save_count = 0;
}

/*===========================================================
    探索系関数
===========================================================*/
/*-----------------------------------------------------------
    足立法探索走行A（1区画走行）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// searchA
// 1区画走行でgoal座標に進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchA() {}

/*-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -足立法探索走行B（連続走行）-- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// searchB
// 連続走行でgoal座標に進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchB(uint16_t fan_duty) {
    (void)fan_duty;
}

/*-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -足立法探索走行（全面探索用）-- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// adachi
// 連続走行でgoal座標に進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void adachi(void) {

    drive_start();


    //====スタート位置壁情報取得====
    get_wall_info();    // 壁情報の初期化, 後壁はなくなる
    wall_info &= ~0x88; // 前壁は存在するはずがないので削除する
    write_map();        // 壁情報を地図に記入

    //====前に壁が無い想定で問答無用で前進====
    speed_now = 0;
    first_sectionA();
    half_sectionU();
    // adv_pos();

    write_map();

    //====歩数マップ・経路作成====
    r_cnt = 0;                 // 経路カウンタの初期化
    make_smap(goal_x, goal_y); // 歩数マップ作成
    make_route(); // 最短経路探索（route配列に動作が格納される）

    adv_pos();

    //====探索走行====
    do {

        //----進行----
        switch (
            route
                [r_cnt++]) { // route配列によって進行を決定。経路カウンタを進める

        //----前進----
        case 0x88:

            if (mouse.dir == 0x00 && route[r_cnt] == 0x88) {
                // 北向き
                if (visited[mouse.y + 1][mouse.x]) {
                    known_straight = true;
                } else {
                    known_straight = false;
                }
            } else if (mouse.dir == 0x01 && route[r_cnt] == 0x88) {
                // 東向き
                if (visited[mouse.y][mouse.x + 1]) {
                    known_straight = true;
                } else {
                    known_straight = false;
                }
            } else if (mouse.dir == 0x02 && route[r_cnt] == 0x88) {
                // 南向き
                if (visited[mouse.y - 1][mouse.x]) {
                    known_straight = true;
                } else {
                    known_straight = false;
                }
            } else if (mouse.dir == 0x03 && route[r_cnt] == 0x88) {
                // 西向き
                if (visited[mouse.y][mouse.x - 1]) {
                    known_straight = true;
                } else {
                    known_straight = false;
                }
            } else {
                known_straight = false;
            }

            led_write(1, 1);

            if (known_straight && !acceled) {
                one_sectionA();
                acceled = true;
            } else if (!known_straight && acceled) {
                one_sectionD();
                acceled = false;
            } else if (!acceled && fabsf(latest_wall_error) > WALL_ALIGN_ERR_THR) {
                if (r_wall) {
                    half_sectionD(0);
                    rotate_R90();
                    match_position(0);
                    rotate_L90();
                    half_sectionA(1);
                } else if (l_wall) {
                    half_sectionD(0);
                    rotate_L90();
                    match_position(0);
                    rotate_R90();
                    half_sectionA(1);
                }else{
                    one_sectionU(1);
                }
            } else {
                one_sectionU(1);
            }

            led_write(0, 0);

            break;
        //----右折----
        case 0x44:

            led_write(0, 1);

            turn_R90(1);

            turn_dir(DIR_TURN_R90); // マイクロマウス内部位置情報でも右回転処理

            led_write(0, 0);

            break;
        //----180回転----
        case 0x22:
            half_sectionD(0); // 半区間分減速しながら走行し停止

            if (MF.FLAG.GOALED && save_count == 0) {
                drive_variable_reset();
                alpha_interrupt = 0;
                acceleration_interrupt = 0;
                drive_wait();
                store_map_in_eeprom(); // ROMにマップ情報を書き込む
                buzzer_beep(200);

                save_count++;
                if (save_count > 2) {
                    save_count = 0;
                }
            }

            if (ad_fr > WALL_BASE_FR * 1.5 && ad_fl > WALL_BASE_FL * 1.5) {
                match_position(0);
                if (r_wall) {
                    rotate_R90();
                    match_position(0);
                    rotate_R90();
                } else if (l_wall) {
                    rotate_L90();
                    match_position(0);
                    rotate_L90();
                } else {
                    rotate_180();
                }
                drive_wait();
            } else {
                rotate_180();
                drive_wait();
            }

            turn_dir(
                DIR_TURN_180); // マイクロマウス内部位置情報でも180度回転処理
            half_sectionA(1); // 半区画分加速しながら走行する
            break;
        //----左折----
        case 0x11:

            led_write(1, 0);

            turn_L90(1);

            turn_dir(DIR_TURN_L90); // マイクロマウス内部位置情報でも右回転処理

            led_write(0, 0);

            break;
        }

        adv_pos();

        markVisited(mouse.x, mouse.y); // 探索済区画として記録

        if (mouse.x == GOAL_X && mouse.y == GOAL_Y) {
            MF.FLAG.GOALED = 1;
        }

        conf_route();

    } while (!search_end); // 現在座標とgoal座標が等しくなるまで実行

    half_sectionD(0); // 半区画分減速しながら走行し停止

    drive_stop();

    led_flash(2);

    drive_wait();

    store_map_in_eeprom(); // ROMにマップ情報を書き込む

    drive_stop();

    drive_fan(0);

    led_flash(2);

    for (uint8_t i = 0; i < 3; i++) {

        buzzer_enter(300);

        HAL_Delay(150);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// adv_pos
// マイクロマウス内部位置情報で前進させる
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void adv_pos() {
    switch (mouse.dir) { // マイクロマウスが現在向いている方向で判定
    case 0x00:           // 北方向に向いている場合
        mouse.y++; // Y座標をインクリメント
        break;
    case 0x01:     // 東方向に向いている場合
        mouse.x++; // X座標をインクリメント
        break;
    case 0x02:     // 南方向に向いている場合
        mouse.y--; // Y座標をデクリメント
        break;
    case 0x03:     // 西方向に向いている場合
        mouse.x--; // X座標をデクリメント
        break;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// conf_route
// 進路を判定する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void conf_route() {
    //----壁情報書き込み----
    write_map();

    if (make_smap(goal_x, goal_y) > (16 * 16 - 15)) {
        search_end = true;
    } else {
        make_route(); // 最短経路を更新
        r_cnt = 0;    // 経路カウンタを0に
    }

    // buzzer_interrupt(300);

    /*
    if (wall_info & route[r_cnt]) {
        if (make_smap(goal_x, goal_y) > (16 * 16 - 15)) {
            search_end = true;
        } else {
            make_route(); // 最短経路を更新
            r_cnt = 0;    // 経路カウンタを0に
        }
    }
    */
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// map_Init
// マップ格納配列map[][]の初期化をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void map_Init() {
    //====変数宣言====
    uint8_t x, y; // for文用変数

    //====初期化開始====
    // マップのクリア
    for (y = 0; y < 16; y++) {     // 各Y座標で実行
        for (x = 0; x < 16; x++) { // 各X座標で実行
            map[y][x] =
                0xf0; // 上位4ビット（2次走行時）を壁あり，下位4ビット（1次走行時）を壁なしとする。
        }
    }

    // 探索済区画のクリア
    for (y = 0; y < 16; y++) {     // 各Y座標で実行
        for (x = 0; x < 16; x++) { // 各X座標で実行
            visited[y][x] = false;
        }
    }
    visited[0][0] = true;

    // 確定壁の配置
    for (y = 0; y < MAZE_SIZE; y++) {  // 各Y座標で実行
        map[y][0] |= 0xf1;             // 最西に壁を配置
        map[y][MAZE_SIZE - 1] |= 0xf4; // 最東に壁を配置
    }
    for (x = 0; x < MAZE_SIZE; x++) {  // 各X座標で実行
        map[0][x] |= 0xf2;             // 最南に壁を配置
        map[MAZE_SIZE - 1][x] |= 0xf8; // 最北に壁を配置
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// write_map
// マップデータを書き込む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void write_map() {

    //====変数宣言====
    uint8_t m_temp; // 向きを補正した壁情報

    //====壁情報の補正格納====
    m_temp = (wall_info >> mouse.dir) &
             0x0f; // センサ壁情報をmouse.dirで向きを補正させて下位4bit分を残す
    m_temp |=
        (m_temp
         << 4); // 上位4bitに下位4bitをコピー。この作業でm_tempにNESW順で壁が格納

    //====データの書き込み====
    map[mouse.y][mouse.x] = m_temp; // 現在地に壁情報書き込み
    //----周辺に書き込む----
    // 北側について
    if (mouse.y != 15) {     // 現在最北端でないとき
        if (m_temp & 0x88) { // 北壁がある場合
            map[mouse.y + 1][mouse.x] |=
                0x22; // 北側の区画から見て南壁ありを書き込む
        } else {      // 北壁がない場合
            map[mouse.y + 1][mouse.x] &=
                0xDD; // 北側の区画から見て南壁なしを書き込む
        }
    }
    // 東側について
    if (mouse.x != 15) {     // 現在最東端でないとき
        if (m_temp & 0x44) { // 東壁がある場合
            map[mouse.y][mouse.x + 1] |=
                0x11; // 東側の区画から見て西壁ありを書き込む
        } else {      // 北壁がない場合
            map[mouse.y][mouse.x + 1] &=
                0xEE; // 東側の区画から見て西壁なしを書き込む
        }
    }
    // 南壁について
    if (mouse.y != 0) {      // 現在最南端でないとき
        if (m_temp & 0x22) { // 南壁がある場合
            map[mouse.y - 1][mouse.x] |=
                0x88; // 南側の区画から見て北壁ありを書き込む
        } else {      // 南壁がない場合
            map[mouse.y - 1][mouse.x] &=
                0x77; // 南側の区画から見て北壁なしを書き込む
        }
    }
    // 西側について
    if (mouse.x != 0) {      // 現在最西端でないとき
        if (m_temp & 0x11) { // 西壁がある場合
            map[mouse.y][mouse.x - 1] |=
                0x44; // 西側の区画から見て東壁ありを書き込む
        } else {      // 西壁がない場合
            map[mouse.y][mouse.x - 1] &=
                0xBB; // 西側の区画から見て東側なしを書き込む
        }
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_dir
// マウスの方向を変更する
// 引数1：t_pat …… 回転方向(drive.hでマクロ定義)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_dir(uint8_t t_pat) {
    //====方向を変更====
    mouse.dir = (mouse.dir + t_pat) & 0x03; // 指定された分mouse.dirを回転させる
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// make_smap
// 歩数マップを作成する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
int make_smap(uint8_t target_x, uint8_t target_y) {
    (void)target_x;
    (void)target_y;

    //====変数宣言====
    uint8_t x, y; // for文用変数

    //====歩数マップのクリア====
    for (y = 0; y <= 15; y++) {     // 各Y座標で実行
        for (x = 0; x <= 15; x++) { // 各X座標で実行
            smap[y][x] = 0xffff;    // 未記入部分は歩数最大とする
        }
    }

    //====ゴール座標を0にする====
    uint8_t m_step = 0; // 歩数カウンタを0にする
    // smap[target_y][target_x] = 0;

    for (y = 0; y <= 15; y++) {     // 各Y座標で実行
        for (x = 0; x <= 15; x++) { // 各X座標で実行
            if (visited[y][x] == false) {
                smap[y][x] = 0;
            }
        }
    }

    //====自分の座標にたどり着くまでループ====
    do {
        //----マップ全域を捜索----
        for (y = 0; y <= 15; y++) {     // 各Y座標で実行
            for (x = 0; x <= 15; x++) { // 各X座標で実行
                //----現在最大の歩数を発見したとき----
                if (smap[y][x] ==
                    m_step) { // 歩数カウンタm_stepの値が現在最大の歩数
                    uint8_t m_temp =
                        map[y][x]; // map配列からマップデータを取り出す
                    if (MF.FLAG
                            .SCND) { // 二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
                        m_temp >>=
                            4; // 上位4bitを使うので4bit分右にシフトさせる
                    }
                    //----北壁についての処理----
                    if (!(m_temp & 0x08) &&
                        y != 15) { // 北壁がなく現在最北端でないとき
                        if (smap[y + 1][x] == 0xffff) { // 北側が未記入なら

                            smap[y + 1][x] = m_step + 1; // 次の歩数を書き込む
                        }
                    }
                    //----東壁についての処理----
                    if (!(m_temp & 0x04) &&
                        x != 15) { // 東壁がなく現在最東端でないとき
                        if (smap[y][x + 1] == 0xffff) { // 東側が未記入なら
                            smap[y][x + 1] = m_step + 1; // 次の歩数を書き込む
                        }
                    }
                    //----南壁についての処理----
                    if (!(m_temp & 0x02) &&
                        y != 0) { // 南壁がなく現在最南端でないとき
                        if (smap[y - 1][x] == 0xffff) { // 南側が未記入なら
                            smap[y - 1][x] = m_step + 1; // 次の歩数を書き込む
                        }
                    }
                    //----西壁についての処理----
                    if (!(m_temp & 0x01) &&
                        x != 0) { // 西壁がなく現在最西端でないとき
                        if (smap[y][x - 1] == 0xffff) { // 西側が未記入なら
                            smap[y][x - 1] = m_step + 1; // 次の歩数を書き込む
                        }
                    }
                }
            }
        }
        //====歩数カウンタのインクリメント====
        m_step++;
    } while (smap[mouse.y][mouse.x] == 0xffff &&
             m_step < (16 * 16 - 10)); // 現在座標が未記入ではなくなるまで実行

    return m_step;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// make_route
// 最短経路を導出する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_route() {
    //====変数宣言====
    uint8_t x, y; // X，Y座標
    uint8_t dir_temp =
        mouse.dir; // マウスの方角を表すmouse.dirの値をdir_temp変数に退避させる

    //====最短経路を初期化====
    uint16_t i;
    for (i = 0; i < 256; i++) {
        route[i] = 0xffff; // routeを0xffで初期化
    }

    //====歩数カウンタをセット====
    uint8_t m_step = smap[mouse.y][mouse.x]; // 現在座標の歩数マップ値を取得

    //====x, yに現在座標を書き込み====
    x = mouse.x;
    y = mouse.y;

    //====最短経路を導出====
    i = 0;

    do {
        uint8_t m_temp = map[y][x]; // 比較用マップ情報の格納

        if (MF.FLAG
                .SCND) { // 二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
            m_temp >>= 4; // 上位4bitを使うので4bit分右にシフトさせる
        }

        //----北を見る----
        if (!(m_temp & 0x08) &&
            (smap[y + 1][x] <
             m_step)) { // 北側に壁が無く、現在地より小さい歩数マップ値であれば
            route[i] = (0x00 - mouse.dir) & 0x03; // route配列に進行方向を記録
            m_step = smap[y + 1][x]; // 最大歩数マップ値を更新
            y++; // 北に進んだのでY座標をインクリメント

        }
        //----東を見る----
        else if (
            !(m_temp & 0x04) &&
            (smap[y][x + 1] <
             m_step)) { // 東側に壁が無く、現在地より小さい歩数マップ値であれば
            route[i] = (0x01 - mouse.dir) & 0x03; // route配列に進行方向を記録
            m_step = smap[y][x + 1]; // 最大歩数マップ値を更新
            x++; // 東に進んだのでX座標をインクリメント

        }
        //----南を見る----
        else if (
            !(m_temp & 0x02) &&
            (smap[y - 1][x] <
             m_step)) { // 南側に壁が無く、現在地より小さい歩数マップ値であれば
            route[i] = (0x02 - mouse.dir) & 0x03; // route配列に進行方向を記録
            m_step = smap[y - 1][x]; // 最大歩数マップ値を更新
            y--; // 南に進んだのでY座標をデクリメント

        }
        //----西を見る----
        else if (
            !(m_temp & 0x01) &&
            (smap[y][x - 1] <
             m_step)) { // 西側に壁が無く、現在地より小さい歩数マップ値であれば
            route[i] = (0x03 - mouse.dir) & 0x03; // route配列に進行方向を記録
            m_step = smap[y][x - 1]; // 最大歩数マップ値を更新
            x--; // 西に進んだのでX座標をデクリメント
        }

        //----格納データ形式変更----
        switch (route[i]) {  // route配列に格納した要素値で分岐
        case 0x00:           // 前進する場合
            route[i] = 0x88; // 格納データ形式を変更
            break;
        case 0x01:                  // 右折する場合
            turn_dir(DIR_TURN_R90); // 内部情報の方向を90度右回転
            route[i] = 0x44;        // 格納データ形式を変更

            break;
        case 0x02:                  // Uターンする場合
            turn_dir(DIR_TURN_180); // 内部情報の方向を180度回転
            route[i] = 0x22;        // 格納データ形式を変更

            break;
        case 0x03:                  // 左折する場合
            turn_dir(DIR_TURN_L90); // 内部情報の方向を90度右回転
            route[i] = 0x11;        // 格納データ形式を変更

            break;
        default:             // それ以外の場合
            route[i] = 0x00; // 格納データ形式を変更

            break;
        }
        i++; // カウンタをインクリメント
    } while (smap[y][x] !=
             0); // 進んだ先の歩数マップ値が0（=ゴール）になるまで実行

    mouse.dir = dir_temp; // dir_tempに退避させた値をmouse.dirにリストア
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// markVisited
// 探索済区画を管理する
// 引数：探索区画のXY座標(x,y)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void markVisited(uint8_t x, uint8_t y) { visited[y][x] = true; }

//+++++++++++++++++++++++++++++++++++++++++++++++
// findClosestUnvisitedCell
// 最近の未探索区画を選択
// 引数：探索区画のXY座標(x,y)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void findClosestUnvisitedCell(uint8_t currentX, uint8_t currentY) {
    (void)currentX;
    (void)currentY;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// store_map_in_eeprom
// mapデータをeepromに格納する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void store_map_in_eeprom(void) {
    eeprom_enable_write();

    int i;
    for (i = 0; i < 16; i++) {
        int j;
        for (j = 0; j < 16; j++) {
            eeprom_write_halfword((uint32_t)(i * 16 + j), (uint16_t)map[i][j]);
        }
    }
    eeprom_disable_write();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// load_map_in_eeprom
// mapデータをeepromから取得する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void load_map_from_eeprom(void) {
    int i;
    for (i = 0; i < 16; i++) {
        int j;
        for (j = 0; j < 16; j++) {
            map[i][j] = (uint8_t)eeprom_read_halfword(i * 16 + j);
        }
    }
}
