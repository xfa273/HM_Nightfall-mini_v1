/*
 * run.h
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#ifndef INC_RUN_H_
#define INC_RUN_H_

#include <stdint.h>

void convertPath(uint8_t);
void run(void);
// 任意のパス配列で run() と同一ロジックを実行するテスト/デバッグ用API
void run_with_path(const uint16_t *path_array);
void run_shortest(uint8_t mode, uint8_t case_index);

#endif /* INC_RUN_H_ */
