#include "solver_params.h"
#include <stddef.h>

// move_cost_min は安全のため固定値（リストからは設定しない）
#define SOLVER_MOVE_COST_MIN 0.05f

// 既定値（必要に応じてユーザーが調整）
// 各モードの5ケース（case=3..7に対応）
static const SolverCaseParams_t solverCaseParamsMode2[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.85f, 0.0f, 0.008f, 0.005f, 0.55f},
    {1.0f, 0.76f, 0.85f, 0.0f, 0.010f, 0.005f, 0.55f},
};

static const SolverCaseParams_t solverCaseParamsMode3[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.84f, 0.0f, 0.008f, 0.006f, 0.55f},
    {1.0f, 0.76f, 0.83f, 0.0f, 0.010f, 0.007f, 0.55f},
    {1.0f, 0.74f, 0.82f, 0.0f, 0.012f, 0.008f, 0.60f},
    {1.0f, 0.72f, 0.81f, 0.0f, 0.014f, 0.009f, 0.60f},
};

static const SolverCaseParams_t solverCaseParamsMode4[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.85f, 0.0f, 0.008f, 0.005f, 0.55f},
    {1.0f, 0.76f, 0.85f, 0.0f, 0.010f, 0.005f, 0.55f},
    {1.0f, 0.76f, 0.84f, 0.0f, 0.010f, 0.006f, 0.55f},
    {1.0f, 0.74f, 0.84f, 0.0f, 0.012f, 0.006f, 0.60f},
};

static const SolverCaseParams_t solverCaseParamsMode5[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.84f, 0.0f, 0.008f, 0.006f, 0.55f},
    {1.0f, 0.76f, 0.83f, 0.0f, 0.010f, 0.007f, 0.55f},
    {1.0f, 0.74f, 0.82f, 0.0f, 0.012f, 0.008f, 0.60f},
    {1.0f, 0.72f, 0.81f, 0.0f, 0.014f, 0.009f, 0.60f},
};

static const SolverCaseParams_t solverCaseParamsMode6[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.84f, 0.0f, 0.008f, 0.006f, 0.55f},
    {1.0f, 0.76f, 0.83f, 0.0f, 0.010f, 0.007f, 0.55f},
    {1.0f, 0.74f, 0.82f, 0.0f, 0.012f, 0.008f, 0.60f},
    {1.0f, 0.72f, 0.81f, 0.0f, 0.014f, 0.009f, 0.60f},
};

static const SolverCaseParams_t solverCaseParamsMode7[5] = {
    {1.0f, 0.80f, 0.85f, 0.0f, 0.006f, 0.005f, 0.50f},
    {1.0f, 0.78f, 0.84f, 0.0f, 0.008f, 0.006f, 0.55f},
    {1.0f, 0.76f, 0.83f, 0.0f, 0.010f, 0.007f, 0.55f},
    {1.0f, 0.74f, 0.82f, 0.0f, 0.012f, 0.008f, 0.60f},
    {1.0f, 0.72f, 0.81f, 0.0f, 0.014f, 0.009f, 0.60f},
};

static SolverCaseParams_t g_params;

const SolverCaseParams_t* solver_get_case_params(uint8_t mode, uint8_t case_index){
    // case_index: 3..7 を 0..4 に正規化
    uint8_t idx = 0;
    if (case_index >= 3 && case_index <= 7) idx = (uint8_t)(case_index - 3);

    const SolverCaseParams_t* p = NULL;
    switch(mode){
        case 2: p = &solverCaseParamsMode2[idx]; break;
        case 3: p = &solverCaseParamsMode3[idx]; break;
        case 4: p = &solverCaseParamsMode4[idx]; break;
        case 5: p = &solverCaseParamsMode5[idx]; break;
        case 6: p = &solverCaseParamsMode6[idx]; break;
        case 7: p = &solverCaseParamsMode7[idx]; break;
        default: p = &solverCaseParamsMode2[0]; break;
    }

    // 出力用にコピーして move_cost_min を固定値へ上書き
    g_params = *p;
    g_params.move_cost_min = SOLVER_MOVE_COST_MIN;
    return &g_params;
}
