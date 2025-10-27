/*
 * dijkstra.c
 *
 *  Created on: Dec 23, 2023
 *      Author: yuho-
 */

#include "global.h"
#include "maze_grid.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*迷路の用意↓*/
// 迷路データと表示系は maze_grid.c に移設

int diagonal_counter = 0;
int straight_counter = 0;

// 直近の dijkstra() 実行での総コストを保持（到達不能時は INFINITY）
int last_dijkstra_cost = INFINITY;

// initializeMaze は maze_grid.c に実装

void initializePath() {
    for (int i = 0; i < 256; i++) {
        path[i] = 0;
    }
}

// 前方宣言
int getDistance(Node a, Node b);

// 経路のトレースを行わず、総コストのみ返す（到達不能時は INFINITY）
int dijkstra_cost_only(Node start, Node goal) {
    // 初期化
    int dist[NODE_COUNT], prev[NODE_COUNT];
    bool visited[NODE_COUNT] = {false};

    for (int i = 0; i < NODE_COUNT; i++) {
        dist[i] = INFINITY;
        prev[i] = -1;
    }

    dist[(MAZE_SIZE - 1 - start.y) * MAZE_SIZE + start.x] = 0;

    for (int i = 0; i < NODE_COUNT - 1; i++) {
        int min = INFINITY, min_index = 0;
        for (int v = 0; v < NODE_COUNT; v++) {
            if (!visited[v] && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
        }

        int u = min_index;
        visited[u] = true;

        for (int v = 0; v < NODE_COUNT; v++) {
            Node uNode = {u % MAZE_SIZE, u / MAZE_SIZE};
            Node vNode = {v % MAZE_SIZE, v / MAZE_SIZE};

            int distance = getDistance(uNode, vNode);

            if (distance > 0) {
                int prevNodeIndex = prev[u];
                if (prevNodeIndex != -1) {
                    if (vNode.x == uNode.x) {
                        if (straight_counter % 10 == 1) {
                            straight_counter += 10;
                        } else {
                            straight_counter = 1;
                        }
                    } else if (vNode.y == uNode.y) {
                        if (straight_counter % 10 == 2) {
                            straight_counter += 10;
                        } else {
                            straight_counter = 2;
                        }
                    } else {
                        straight_counter = 0;
                    }

                    if (straight_counter > 10) {
                        distance -= straight_counter * straight_weight;
                    }

                    if ((vNode.x == uNode.x + 1 && vNode.y == uNode.y + 1)) {
                        if (diagonal_counter % 10 == 1) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 1;
                        }
                    } else if ((vNode.x == uNode.x + 1 && vNode.y == uNode.y - 1)) {
                        if (diagonal_counter % 10 == 2) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 2;
                        }
                    } else if ((vNode.x == uNode.x - 1 && vNode.y == uNode.y + 1)) {
                        if (diagonal_counter % 10 == 3) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 3;
                        }
                    } else if ((vNode.x == uNode.x - 1 && vNode.y == uNode.y - 1)) {
                        if (diagonal_counter % 10 == 4) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 4;
                        }
                    } else {
                        diagonal_counter = 0;
                    }

                    if (diagonal_counter > 10) {
                        distance -= diagonal_counter * diagonal_weight;
                    }
                }
            }

            if (!visited[v] && distance && dist[u] != INFINITY &&
                dist[u] + distance < dist[v]) {
                dist[v] = dist[u] + distance;
                prev[v] = u;
            }
        }
    }

    int goalIndex = (MAZE_SIZE - 1 - goal.y) * MAZE_SIZE + goal.x;
    last_dijkstra_cost = dist[goalIndex];
    return last_dijkstra_cost;
}

void convertPathCellToRun() {
    int8_t Direction = NORTH;
    for (int i = 0; i < 256; i++) {

        if (path[i] - 100 == Direction) {
            // 直進
            // printf("Straight\n");
            path[i] = STRAIGHT;
        } else if (path[i] == MOVE_EAST) {
            // 進行方向が東
            if (Direction == NORTH) {
                // 北向きから右折
                // printf("Turn_R\n");
                path[i] = TURN_R;
            } else if (Direction == SOUTH) {
                // 南向きから左折
                // printf("Turn_L\n");
                path[i] = TURN_L;
            }
            Direction = EAST;
        } else if (path[i] == MOVE_SOUTH) {
            // 進行方向が南
            if (Direction == EAST) {
                // 東向きから右折
                // printf("Turn_R\n");
                path[i] = TURN_R;
            } else if (Direction == WEST) {
                // 西向きから左折
                // printf("Turn_L\n");
                path[i] = TURN_L;
            }
            Direction = SOUTH;
        } else if (path[i] == MOVE_WEST) {
            // 進行方向が西
            if (Direction == SOUTH) {
                // 南向きから右折
                // printf("Turn_R\n");
                path[i] = TURN_R;
            } else if (Direction == NORTH) {
                // 北向きから左折
                // printf("Turn_L\n");
                path[i] = TURN_L;
            }
            Direction = WEST;
        } else if (path[i] == MOVE_NORTH) {
            // 進行方向が北
            if (Direction == WEST) {
                // 西向きから右折
                // printf("Turn_R\n");
                path[i] = TURN_R;
            } else if (Direction == EAST) {
                // 東向きから左折
                // printf("Turn_L\n");
                path[i] = TURN_L;
            }
            Direction = NORTH;
        } else {
            // printf("No Path Found.\n");
        }
    }
}

// setMazeWalls は maze_grid.c に実装

// correctWallInconsistencies は maze_grid.c に実装

// printMaze は maze_grid.c に実装

// ノード間のエッジを表す構造体
typedef struct {
    Node from, to;
    int weight;
} Edge;

// ノード間の距離（エッジの有無）を計算する関数

int getDistance(Node a, Node b) {

    if (a.x == b.x && (a.y == b.y + 1)) {
        return (maze[a.y][a.x] & NORTH_WALL) || (maze[b.y][b.x] & SOUTH_WALL)
                   ? 0
                   : 160;
    } else if (a.x == b.x && (a.y + 1 == b.y)) {
        return (maze[a.y][a.x] & SOUTH_WALL) || (maze[b.y][b.x] & NORTH_WALL)
                   ? 0
                   : 160;
    } else if (a.y == b.y && (a.x == b.x + 1)) {
        return (maze[a.y][a.x] & WEST_WALL) || (maze[b.y][b.x] & EAST_WALL)
                   ? 0
                   : 160;
    } else if (a.y == b.y && (a.x + 1 == b.x)) {
        return (maze[a.y][a.x] & EAST_WALL) || (maze[b.y][b.x] & WEST_WALL)
                   ? 0
                   : 160;
    }

    return 0; // 隣接していないノードの場合
}

// ダイクストラ法による最短経路を計算する関数
void dijkstra(Node start, Node goal) {
    // 初期化
    last_dijkstra_cost = INFINITY;

    int dist[NODE_COUNT], prev[NODE_COUNT];
    bool visited[NODE_COUNT] = {false};

    for (int i = 0; i < NODE_COUNT; i++) {
        dist[i] = INFINITY;
        prev[i] = -1;
    }

    // ノードのインデックス計算時のy座標反転
    dist[(MAZE_SIZE - 1 - start.y) * MAZE_SIZE + start.x] = 0;

    for (int i = 0; i < NODE_COUNT - 1; i++) {
        // 最短距離の未訪問ノードを見つける
        int min = INFINITY, min_index;
        for (int v = 0; v < NODE_COUNT; v++) {
            if (!visited[v] && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
        }

        int u = min_index;
        visited[u] = true;

        // 隣接するノードの距離を更新する
        for (int v = 0; v < NODE_COUNT; v++) {
            // int uy = MAZE_SIZE - 1 - (u / MAZE_SIZE); // y座標反転 - 未使用変数
            Node uNode = {u % MAZE_SIZE, u / MAZE_SIZE};
            // int vy = MAZE_SIZE - 1 - (v / MAZE_SIZE); // y座標反転 - 未使用変数
            Node vNode = {v % MAZE_SIZE, v / MAZE_SIZE};

            int distance = getDistance(uNode, vNode);

            // 直線移動のコスト調整
            if (distance > 0) {
                int prevNodeIndex = prev[u];
                if (prevNodeIndex != -1) {
                    // 変数prevNodeは使用されていないのでコメントアウト
                    // Node prevNode = {prevNodeIndex % MAZE_SIZE,
                    //                  prevNodeIndex / MAZE_SIZE};

                    // 直線の長さに応じてコストを減少させる
                    if (vNode.x == uNode.x) {
                        if (straight_counter % 10 == 1) {
                            straight_counter += 10;
                        } else {
                            straight_counter = 1;
                        }
                    } else if (vNode.y == uNode.y) {
                        if (straight_counter % 10 == 2) {
                            straight_counter += 10;
                        } else {
                            straight_counter = 2;
                        }
                    } else {
                        straight_counter = 0;
                    }

                    if (straight_counter > 10) {
                        distance -= straight_counter * straight_weight;
                    }

                    // 斜めの長さに応じてコストを減少させる
                    if ((vNode.x == uNode.x + 1 && vNode.y == uNode.y + 1)) {
                        if (diagonal_counter % 10 == 1) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 1;
                        }
                    } else if ((vNode.x == uNode.x + 1 &&
                                vNode.y == uNode.y - 1)) {
                        if (diagonal_counter % 10 == 2) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 2;
                        }
                    } else if ((vNode.x == uNode.x - 1 &&
                                vNode.y == uNode.y + 1)) {
                        if (diagonal_counter % 10 == 3) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 3;
                        }
                    } else if ((vNode.x == uNode.x - 1 &&
                                vNode.y == uNode.y - 1)) {
                        if (diagonal_counter % 10 == 4) {
                            diagonal_counter += 10;
                        } else {
                            diagonal_counter = 4;
                        }
                    } else {
                        diagonal_counter = 0;
                    }

                    if (diagonal_counter > 10) {
                        distance -= diagonal_counter * diagonal_weight;
                    }
                }
            }

            if (!visited[v] && distance && dist[u] != INFINITY &&
                dist[u] + distance < dist[v]) {
                dist[v] = dist[u] + distance;
                prev[v] = u;
            }
        }
    }

    // ゴールのインデックスと総コストを取得
    int goalIndex = (MAZE_SIZE - 1 - goal.y) * MAZE_SIZE + goal.x;
    last_dijkstra_cost = dist[goalIndex];

    // ゴールから最短経路をトレースする
    int stack[NODE_COUNT], stackSize = 0;
    int index = goalIndex;
    while (index != -1) {
        int y = MAZE_SIZE - 1 - (index / MAZE_SIZE); // y座標反転
        int x = index % MAZE_SIZE;
        path_cell[y][x] = true; // 経路上のセルをマーク
        stack[stackSize++] = index;
        index = prev[index];
    }

    // 移動方向を出力する
    uint16_t path_count = 0;
    for (int i = stackSize - 1; i > 0; i--) {
        int current = stack[i];
        int next = stack[i - 1];
        int currentX = current % MAZE_SIZE;
        int currentY = MAZE_SIZE - 1 - (current / MAZE_SIZE);
        int nextX = next % MAZE_SIZE;
        int nextY = MAZE_SIZE - 1 - (next / MAZE_SIZE);

        if (nextX > currentX) {
            printf("R ");
            path[path_count] = MOVE_EAST;
        } else if (nextX < currentX) {
            printf("L ");
            path[path_count] = MOVE_WEST;
        } else if (nextY > currentY) {
            printf("F ");
            path[path_count] = MOVE_NORTH;
        } else if (nextY < currentY) {
            printf("B ");
            path[path_count] = MOVE_SOUTH;
        }

        path_count++;
    }
    printf("\n");

    convertPathCellToRun();
}

// reverseArrayYAxis は maze_grid.c に実装
