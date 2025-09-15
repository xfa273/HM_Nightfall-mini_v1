/*
 * path.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"

void simplifyPath(void) {
    int simplifiedPath[256]; // 結果を格納するための配列
    int currentIndex = 0; // simplifiedPathにおける現在のインデックス
    int currentAction = path[0]; // 現在処理している動作
    int count = 1;               // 現在の動作のカウント
    int i = 1;

    while (path[i] != 0) {
        if (path[i] == currentAction && currentAction == STRAIGHT) {
            // 現在の動作が直進で続いている場合、カウントを増やす
            count++;
        } else {
            // 直進以外の場合、または動作が変わった場合
            if (currentAction == STRAIGHT) {
                // 直進の場合、カウントを加えて結果配列に追加
                simplifiedPath[currentIndex++] = currentAction + count * 2;
            } else {
                // 直進以外の場合はそのまま結果配列に追加
                simplifiedPath[currentIndex++] = currentAction;
            }

            // 新しい動作の処理を開始
            currentAction = path[i];
            count = 1;
        }
        i++;
    }

    // 最後の動作を結果配列に追加
    if (currentAction == STRAIGHT) {
        simplifiedPath[currentIndex++] = currentAction + count * 2;
    } else {
        simplifiedPath[currentIndex++] = currentAction;
    }

    // simplifiedPathをpathにコピー
    for (int j = 0; j < currentIndex; j++) {
        path[j] = simplifiedPath[j];
    }

    // pathの残りをクリア
    for (int j = currentIndex; j < 256; j++) {
        path[j] = 0;
    }

    path[0] -= 1;
}

void convertLTurn() {
    int convertedPath[256];
    int i = 0;
    int j = 0;

    while (path[i] != 0) {

        if (path[i] == 300 && path[i + 1] == 300) {
            // 右旋回が2連続

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 2] > 200 &&
                path[i + 2] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 502;
                path[i + 2] -= 1;

                i++;
            } else {
                // 通常右旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400 && path[i + 1] == 400) {
            // 左旋回が2連続

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 2] > 200 &&
                path[i + 2] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 602;
                path[i + 2] -= 1;

                i++;
            } else {
                // 通常左旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 300) {
            // 右旋回

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 1] > 200 &&
                path[i + 1] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 501;
                path[i + 1] -= 1;
            } else {
                // 通常右旋回
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400) {
            // 左旋回

            if (path[i - 1] > 200 && path[i - 1] < 300 && path[i + 1] > 200 &&
                path[i + 1] < 300) {
                // 前後が直進
                // 大回り判定

                convertedPath[j - 1] -= 1;
                convertedPath[j] = 601;
                path[i + 1] -= 1;
            } else {
                // 通常左旋回
                convertedPath[j] = path[i];
            }
        } else {
            // 直進
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    int count = 0;
    int m, n;

    // 値が200の要素を削除し、後ろの要素を前に詰める
    for (m = 0; m < 256 - count; m++) {
        if (path[m] == 200) {
            for (n = m; n < 255 - count; n++) {
                path[n] = path[n + 1];
            }
            count++; // 削除した要素の数をインクリメント
            m--; // 詰めた後の現在の位置にある新しい要素もチェックするためにデクリメント
        }
    }
}

void convertDiagonal(void) {
    int convertedPath[256];
    int i = 0;
    int j = 0;

    /* 小回りの開始の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 && path[i - 1] < 300) {
            // 右小回りの開始

            if (path[i + 1] >= 300 && path[i + 1] < 400) {
                // 右小回りx2

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 901; // 右斜め135°に変換
                } else {
                    convertedPath[j - 1] = 901; // 右斜め135°に変換
                    // 直前の直進を上書きで削除
                }

                i++; // 次の小回りとまとめたので次パスをスキップ
            } else {
                // 右小回りx1

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 701; // 右斜め45°に変換
                } else {
                    convertedPath[j - 1] = 701; // 右斜め45°に変換
                    j--;
                    // 直前の直進を上書きで削除
                }

                // j--;
            }
        } else if (path[i] >= 400 && path[i] < 500 && path[i - 1] < 300) {
            // 左小回りの開始

            if (path[i + 1] >= 400 && path[i + 1] < 500) {
                // 左小回りx2

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 902; // 左斜め135°に変換
                } else {
                    convertedPath[j - 1] = 902; // 左斜め135°に変換
                    // 直前の直進を上書きで削除
                }
                i++; // 次の小回りとまとめたので次パスをスキップ
            } else {
                // 左小回りx1

                if (path[i - 1] - 200 > 1) {
                    convertedPath[j - 1] -= 1; // 直前の直進を半区画縮める
                    convertedPath[j] = 702; // 左斜め45°に変換
                } else {
                    convertedPath[j - 1] = 702; // 左斜め45°に変換
                    j--;
                    // 直前の直進を上書きで削除
                }
                // j--;
            }
        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* 小回りの終了の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 &&
            (path[i + 1] < 300 || path[i + 1] > 500)) {
            // 右小回りの終了

            if (path[i - 1] >= 300 && path[i - 1] < 400) {
                // 右小回りx2

                convertedPath[j - 1] = 903; // 右斜め135°に変換
                // 前の小回りとまとめたので1つ前のパスを上書き
                j--;

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後の直進をスキップして削除
                }
                i++;

            } else {
                // 右小回りx1

                convertedPath[j] = 703; // 右斜め45°に変換

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後の直進をスキップして削除
                }
                i++;
            }
        } else if (path[i] >= 400 && path[i] < 500 &&
                   (path[i + 1] < 300 || path[i + 1] > 500)) {
            // 左小回りの終了

            if (path[i - 1] >= 400 && path[i - 1] < 500) {
                // 左小回りx2

                convertedPath[j - 1] = 904; // 左斜め135°に変換
                // 前の小回りとまとめたので1つ前のパスを上書き
                j--;

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後の直進をスキップして削除
                }
                i++;
            } else {
                // 左小回りx1

                convertedPath[j] = 704; // 左斜め45°に変換

                if (path[i + 1] - 200 > 1) {
                    convertedPath[j + 1] =
                        path[i + 1] - 1; // 直後の直進を半区画縮める
                    j++;

                } else {
                    // 直後の直進をスキップして削除
                }
                i++;
            }
        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* V90の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 400 && path[i + 1] >= 300 &&
            path[i + 1] < 400) {
            // 右小回りの連続

            convertedPath[j] = 801; // 右V90に変換
            i++; // 次の小回りとまとめたので次パスをスキップ

        } else if (path[i] >= 400 && path[i] < 500 && path[i + 1] >= 400 &&
                   path[i + 1] < 500) {
            // 左小回りの連続

            convertedPath[j] = 802; // 左V90に変換
            i++; // 次の小回りとまとめたので次パスをスキップ

        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    /* 斜め直進の処理 */
    while (path[i] != 0) {

        if (path[i] >= 300 && path[i] < 500) {
            // 小回り

            convertedPath[j] = 1001; // 斜め直進に変換

        } else {
            convertedPath[j] = path[i];
        }

        i++;
        j++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < j; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    // カウンタをリセット
    i = 0;
    j = 0;

    int result_index = 0; // convertedPathのインデックス
    int sum = 0;          // 連続した1000以上の要素の積算値
    int in_sequence = 0;  // 1000以上の連続のフラグ

    /* 斜め直進を繋いでまとめる処理 */
    while (path[i] != 0) {
        if (path[i] >= 1000) {
            // 1000以上の連続が始まった場合
            if (!in_sequence) {
                in_sequence = 1;
                sum = 0; // 積算値をリセット
            }
            sum += path[i] - 1000; // 1000を引いた値を積算
        } else {
            // 1000未満の要素に達した場合
            if (in_sequence) {
                // 連続が終わったときに積算値をconvertedPathに格納
                convertedPath[result_index++] = sum + 1000;
                j++;
                in_sequence = 0;
            }
            convertedPath[result_index++] = path[i];
            j++;
        }
        i++;
    }

    // convertedPathをpathにコピー
    for (int k = 0; k < 256; k++) {
        path[k] = convertedPath[k];
    }

    // pathの残りをクリア
    for (int l = j; l < 256; l++) {
        path[l] = 0;
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");
}

void makePath(uint8_t path_type) {
    // 0:小回り 1:大回り 2:斜め

    load_map_from_eeprom();

    uint8_t fixedMap[MAZE_SIZE][MAZE_SIZE];

    // 迷路の初期化
    initializeMaze(fixedMap);

    // 下位4ビットを新しい配列にコピーし、表示する
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {

            // 下位4ビットを抽出し、新しい配列に格納
            fixedMap[i][j] = (map[i][j] >> 4) & 0xF;
        }
    }

    reverseArrayYAxis(fixedMap);
    setMazeWalls(fixedMap);

    // 壁情報の矛盾を修正
    correctWallInconsistencies();

    // 最短経路を見つける
    Node start = {START_X, START_Y}; // スタート地点
    Node goal = {GOAL_X, GOAL_Y};    // ゴール地点

    dijkstra(start, goal);

    simplifyPath();

    if (path_type > 0) {
        convertLTurn();
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    if (path_type > 1) {
        convertDiagonal();
    }

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        // printf("%d ", path[i]);
        if (path[i] < 300) {
            uint8_t str_sec;
            str_sec = path[i] - 200;
            printf("S%d", str_sec);
        } else if (path[i] == 300) {
            printf("S-R90");
        } else if (path[i] == 400) {
            printf("S-L90");
        } else if (path[i] == 501) {
            printf("L-R90");
        } else if (path[i] == 502) {
            printf("L-R180");
        } else if (path[i] == 601) {
            printf("L-L90");
        } else if (path[i] == 602) {
            printf("L-L180");
        } else if (path[i] == 701) {
            printf("R45-in");
        } else if (path[i] == 702) {
            printf("L45-in");
        } else if (path[i] == 703) {
            printf("R45-out");
        } else if (path[i] == 704) {
            printf("L45-out");
        } else if (path[i] == 801) {
            printf("R-V90");
        } else if (path[i] == 802) {
            printf("L-V90");
        } else if (path[i] == 901) {
            printf("R135-in");
        } else if (path[i] == 902) {
            printf("L135-in");
        } else if (path[i] == 903) {
            printf("R135-out");
        } else if (path[i] == 904) {
            printf("L135-out");
        } else if (path[i] > 1000) {
            uint8_t diag_sec;
            diag_sec = path[i] - 1000;
            printf("D-S%d", diag_sec);
        }
        printf(", ");
    }
    printf("Goal\n");

    // 迷路の表示
    printMaze();
}
