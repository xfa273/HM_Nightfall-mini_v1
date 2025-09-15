/*
 * dijkstra.h
 *
 *  Created on: Dec 23, 2023
 *      Author: yuho-
 */

#ifndef INC_DIJKSTRA_H_
#define INC_DIJKSTRA_H_

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

// 区画の壁の情報を表すビットマスク
#define NORTH_WALL 0x8 // 北
#define EAST_WALL  0x4 // 東
#define SOUTH_WALL 0x2 // 南
#define WEST_WALL  0x1 // 西

// #define MAZE_SIZE 8
// #define START_X 0
// #define START_Y 0

#define STRAIGHT_WEIGHT 10
#define DIAGONAL_WEIGHT 10

#define INFINITY INT_MAX
#define NODE_COUNT (MAZE_SIZE * MAZE_SIZE)

// 移動方向
#define MOVE_NORTH 100
#define MOVE_EAST 101
#define MOVE_SOUTH 102
#define MOVE_WEST 103

// 進行方向
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// 走行パス
#define STRAIGHT 200
#define TURN_R 300
#define TURN_L 400

// 迷路のデータを格納する2次元配列
extern uint8_t maze[MAZE_SIZE][MAZE_SIZE];
extern bool path_cell[MAZE_SIZE][MAZE_SIZE];

extern int diagonal_counter;
extern int straight_counter;

typedef struct {
    int x, y;
} Node;

// 迷路のデータを初期化する関数
// void initializeMaze();
void initializeMaze(uint8_t maze[MAZE_SIZE][MAZE_SIZE]);

// 与えられた壁情報で迷路を更新する関数
void setMazeWalls(uint8_t walls[MAZE_SIZE][MAZE_SIZE]);

// 壁情報の矛盾を修正する関数
void correctWallInconsistencies();

// 迷路の情報を表示する関数
void printMaze();

// ダイクストラ法による最短経路を計算する関数
void dijkstra(Node start, Node goal);

void ConvertMapIntoWall();
void PrintWallData();

void reverseArrayYAxis(uint8_t array[MAZE_SIZE][MAZE_SIZE]);

#endif /* INC_DIJKSTRA_H_ */
