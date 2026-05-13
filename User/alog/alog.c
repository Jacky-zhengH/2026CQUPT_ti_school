#include "alog.h"
#include "math.h"

#define R_SERIES_INPUT 9100.0f // 信号源串联的分压电阻 (1kΩ)
#define R_TEST_LOAD 1000.0f    // 继电器接入的测试负载 (1kΩ)
#define MIN_VOLTAGE 0.01f      // 极小电压阈值 (10mV)，防除零和底噪

//---------------------------------------------------------
// 第二问算法实现
//---------------------------------------------------------
float Algo_Round_3(float val)
{
    return roundf(val * 1000.0f) / 1000.0f;
}
float Algo_Measure_Gain(float v_in_amp, float v_out_amp)
{
    if (v_in_amp < MIN_VOLTAGE)
        return 0.0f;
    return v_out_amp / v_in_amp;
}

float Algo_Measure_Rin(float v_source, float v_in_amp)
{
    float v_diff = v_source - v_in_amp;
    if (v_diff == 0.000f)
        return 2000000.0f; // 压差极小，认为输入阻抗极大

    return (v_in_amp / v_diff) * R_SERIES_INPUT;
}

float Algo_Measure_Rout(float v_out_open, float v_out_load)
{
    if (v_out_open < MIN_VOLTAGE)
        return 0.0f;
    if (v_out_load < MIN_VOLTAGE)
        return 99999.0f;

    float rout = ((v_out_open / v_out_load) - 1.0f) * R_TEST_LOAD;
    if (rout < 0.0f)
        rout = 0.0f;
    return rout;
}

//---------------------------------------------------------
// 第三问：故障诊断核心算法
//---------------------------------------------------------
//static float base_gain = 10.0f;
//static float base_rin = 1380000.0f;
//Zstatic float base_rout = 180.0f;
static float base_gain = 17.76f;   
static float base_rin  = 360700.0f; 
static float base_rout = 152.0f;  // 保留变量，防止外部报错

void Algo_Set_Baseline(float normal_gain, float normal_rin, float normal_rout)
{
    base_gain = (normal_gain > 1.0f) ? normal_gain : 10.0f;
    base_rin = (normal_rin > 10000.0f) ? normal_rin : 1380000.0f;
    base_rout = (normal_rout > 10.0f) ? normal_rout : 180.0f;
}

// 预估故障字典 (Gain比率 | Rin比率 | Rout比率)
// const float Fault_Dictionary[6][3] = {
//     {0.05f, 1.00f, 1.00f}, // R1 翻倍
//     {0.30f, 1.00f, 1.00f}, // R2 翻倍
//     {1.00f, 1.70f, 1.00f}, // R3 翻倍
//     {1.70f, 1.00f, 2.00f}, // R4 翻倍
//     {0.60f, 1.00f, 1.00f}, // R5 翻倍
//     {1.08f, 1.00f, 1.00f}  // RL 翻倍
// };

const float Fault_Dictionary[6][2] = {
    {0.01f, 6.50f},  // 0: R1 翻倍 (靠极大的 Rin 区分)
    {0.01f, 29.00f}, // 1: R2 翻倍 (靠极其夸张的 Rin 区分)
    {0.98f, 1.22f},  // 2: R3 翻倍
    {0.13f, 3.35f},  // 3: R4 翻倍 (靠增益大幅下降区分)
    {0.81f, 3.35f},  // 4: R5 翻倍 (靠增益轻微下降区分)
    {1.05f, 1.00f}   // 5: RL 翻倍 (靠增益微涨区分)
};

// uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout)
// {
//     float ratio_gain = current_gain / base_gain;
//     float ratio_rin = current_rin / base_rin;
//     float ratio_rout = current_rout / base_rout;

//     // 容差判断：波动在15%以内视为正常
//     if (fabs(ratio_gain - 1.0f) < 0.15f &&
//         fabs(ratio_rin - 1.0f) < 0.15f &&
//         fabs(ratio_rout - 1.0f) < 0.15f)
//     {
//         return 0; // 正常
//     }

//     float min_distance = 99999.0f;
//     uint8_t fault_index = 0;

//     const float W_GAIN = 1.0f;
//     const float W_RIN = 1.5f;
//     const float W_ROUT = 2.0f;

//     if (ratio_gain < 0.1f)
//         ratio_rout = 1.0f; // 增益极小修正

//     for (int i = 0; i < 6; i++)
//     {
//         float dist_sq = W_GAIN * (ratio_gain - Fault_Dictionary[i][0]) * (ratio_gain - Fault_Dictionary[i][0]) +
//                         W_RIN * (ratio_rin - Fault_Dictionary[i][1]) * (ratio_rin - Fault_Dictionary[i][1]) +
//                         W_ROUT * (ratio_rout - Fault_Dictionary[i][2]) * (ratio_rout - Fault_Dictionary[i][2]);

//         if (dist_sq < min_distance)
//         {
//             min_distance = dist_sq;
//             fault_index = i + 1;
//         }
//     }
//     return fault_index;
// }

uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout)
{
    float ratio_gain = current_gain / base_gain;
    float ratio_rin = current_rin / base_rin;

    // 容差范围 15%，只判断 Gain 和 Rin
    if (fabs(ratio_gain - 1.0f) < 0.15f &&
        fabs(ratio_rin - 1.0f) < 0.15f)
    {
        return 0; // 0代表正常
    }

    float min_distance = 99999.0f;
    uint8_t fault_index = 0;

    // 权重调整：既然只剩两个维度，我们把权重都拉高
    const float W_GAIN = 2.0f;
    const float W_RIN = 1.0f;

    for (int i = 0; i < 6; i++)
    {
        float dict_gain = Fault_Dictionary[i][0];
        float dict_rin = Fault_Dictionary[i][1];

        // 核心修改：只计算二维平面的欧氏距离平方
        float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain) +
                        W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin);

        if (dist_sq < min_distance)
        {
            min_distance = dist_sq;
            fault_index = i + 1; // 返回 1~6 的代号
        }
    }

    return fault_index;
}