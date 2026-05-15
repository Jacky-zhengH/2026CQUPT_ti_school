#include "alog.h"
#include "math.h"

#define R_SERIES_INPUT 9100.0f // 信号源串联的分压电阻 (1kΩ)
#define R_TEST_LOAD 1000.0f    // 继电器接入的测试负载 (1kΩ)
#define MIN_VOLTAGE 0.01f      // 极小电压阈值 (10mV)，防除零和底噪

//---------------------------------------------------------
// 第二问算法实现
//---------------------------------------------------------
// 四舍五入保留四位
float Algo_Round_3(float val)
{
    return roundf(val * 10000.0f) / 10000.0f;
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
// static float base_gain = 10.0f;
// static float base_rin = 1380000.0f;
// Zstatic float base_rout = 180.0f;
static float base_gain = 16.6f;
static float base_rin = 366000.0f;
static float base_rout = 144.0f; // 保留变量，防止外部报错

void Algo_Set_Baseline(float normal_gain, float normal_rin, float normal_rout)
{
    base_gain = (normal_gain > 1.0f) ? normal_gain : 16.6f;
    base_rin = (normal_rin > 10000.0f) ? normal_rin : 366000.0f;
    base_rout = (normal_rout > 50.0f) ? normal_rout : 144.0f;
}

// 预估故障字典 (Gain比率 | Rin比率 | Rout比率)
const float Fault_Dictionary[6][3] = {
    {0.004f, 3.7640f, 0.0000f},  // R1 翻倍
    {0.00f, 4.00f, 0.0000f},     // R2 翻倍
    {1.0024f, 1.2022f, 0.9792f}, // R3 翻倍
    {0.7906f, 0.3230f, 0.8542f}, // R4 翻倍
    {0.7965f, 1.5056f, 1.0208f}, // R5 翻倍
    {1.1f, 1.4213f, 1.07f}       // RL 翻倍
};

// const float Fault_Dictionary[6][2] = {
//     {0.01f, 6.50f},  // 0: R1 翻倍 (靠极大的 Rin 区分)
//     {0.01f, 29.00f}, // 1: R2 翻倍 (靠极其夸张的 Rin 区分)
//     {0.98f, 1.22f},  // 2: R3 翻倍
//     {0.13f, 3.35f},  // 3: R4 翻倍 (靠增益大幅下降区分)
//     {0.81f, 3.35f},  // 4: R5 翻倍 (靠增益轻微下降区分)
//     {1.05f, 1.00f}   // 5: RL 翻倍 (靠增益微涨区分)
// };

uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout, float current_uo)
{
    // 提前判断R1/R2
    if (current_uo < 0.001f)
    {
        return 2; // R2 翻倍
    }
    // R1故障: Uo在[0.001, 0.01]之间
    else if (current_uo <= 0.01f)
    {
        return 1; // R1 翻倍
    }

    float ratio_gain = current_gain / base_gain;
    float ratio_rin = current_rin / base_rin;
    float ratio_rout = current_rout / base_rout;

    // 容差判断：波动在15%以内视为正常
    if (fabs(ratio_gain - 1.0f) < 0.05f &&
        fabs(ratio_rin - 1.0f) < 0.05f &&
        fabs(ratio_rout - 1.0f) < 0.05f)
    {
        return 0; // 正常
    }

    // 判断RL：增益和输出电阻同时提高超过5%
    if (ratio_gain > 1.05f && ratio_rout > 1.05f)
    {
        return 6; // RL 翻倍
    }

    float min_distance = 99999.0f;
    uint8_t fault_index = 0;

    const float W_GAIN = 4.0f;
    const float W_RIN = 0.0f; // 波动过大不打算考虑
    const float W_ROUT = 1.0f;

    if (ratio_gain < 0.1f)
        ratio_rout = 1.0f; // 增益极小修正

    for (int i = 0; i < 6; i++)
    {
        float dist_sq = W_GAIN * (ratio_gain - Fault_Dictionary[i][0]) * (ratio_gain - Fault_Dictionary[i][0]) +
                        W_RIN * (ratio_rin - Fault_Dictionary[i][1]) * (ratio_rin - Fault_Dictionary[i][1]) +
                        W_ROUT * (ratio_rout - Fault_Dictionary[i][2]) * (ratio_rout - Fault_Dictionary[i][2]);

        if (dist_sq < min_distance)
        {
            min_distance = dist_sq;
            fault_index = i + 1;
        }
    }
    return fault_index;
}

// uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout)
// {
//     float ratio_gain = current_gain / base_gain;
//     float ratio_rin = current_rin / base_rin;

//     // 容差范围 15%，只判断 Gain 和 Rin
//     if (fabs(ratio_gain - 1.0f) < 0.15f &&
//         fabs(ratio_rin - 1.0f) < 0.15f)
//     {
//         return 0; // 0代表正常
//     }

//     float min_distance = 99999.0f;
//     uint8_t fault_index = 0;

//     // 权重调整：既然只剩两个维度，我们把权重都拉高
//     const float W_GAIN = 2.0f;
//     const float W_RIN = 1.0f;

//     for (int i = 0; i < 6; i++)
//     {
//         float dict_gain = Fault_Dictionary[i][0];
//         float dict_rin = Fault_Dictionary[i][1];
//         //float dict_out = Fault_Dictionary[i][2];

//         //（1）使用一维（增益）判断
//         // float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain);

//         // if (dist_sq < min_distance)
//         // {
//         //     min_distance = dist_sq;
//         //     fault_index = i + 1; // 返回 1~6 的代号
//         // }

//         //（2）使用二维（增益+输入电阻）判断
//         float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain) +
//         W_RIN *(ratio_rin - dict_rin) * (ratio_rin - dict_rin);

//         if (dist_sq < min_distance)
//         {
//             min_distance = dist_sq;
//             fault_index = i + 1; // 返回 1~6 的代号
//         }

//         // （3）使用三维（增益+输入电阻+输出电阻）判断
//         // float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain) +
//         //                 W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin) +
//         //                 W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin) ;

//         // if (dist_sq < min_distance)
//         // {
//         //     min_distance = dist_sq;
//         //     fault_index = i + 1; // 返回 1~6 的代号
//         // }
//     }

//     return fault_index;
// }

//---------------------------------------------------------
// ·使用实际值作差值->比较欧氏距离

// 使用实际值作为比较
const float Fault_real_Dictionary[6][3] = {0};
// uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout)
// {
//     float ratio_gain = current_gain / base_gain;
//     float ratio_rin = current_rin / base_rin;

//     // 容差范围 15%，只判断 Gain 和 Rin
//     if (fabs(ratio_gain - 1.0f) < 0.15f &&
//         fabs(ratio_rin - 1.0f) < 0.15f)
//     {
//         return 0; // 0代表正常
//     }

//     float min_distance = 99999.0f;
//     uint8_t fault_index = 0;

//     // 权重调整：既然只剩两个维度，我们把权重都拉高
//     const float W_GAIN = 2.0f;
//     const float W_RIN = 1.0f;

//     for (int i = 0; i < 6; i++)
//     {
//         float dict_gain = Fault_Dictionary[i][0];
//         float dict_rin = Fault_Dictionary[i][1];
//         // float dict_out = Fault_Dictionary[i][2];

//         // （1）使用一维（增益）判断
//         //  float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain);

//         // if (dist_sq < min_distance)
//         // {
//         //     min_distance = dist_sq;
//         //     fault_index = i + 1; // 返回 1~6 的代号
//         // }

//         // （2）使用二维（增益+输入电阻）判断
//         float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain) +
//                         W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin);

//         if (dist_sq < min_distance)
//         {
//             min_distance = dist_sq;
//             fault_index = i + 1; // 返回 1~6 的代号
//         }

//         // （3）使用三维（增益+输入电阻+输出电阻）判断
//         // float dist_sq = W_GAIN * (ratio_gain - dict_gain) * (ratio_gain - dict_gain) +
//         //                 W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin) +
//         //                 W_RIN * (ratio_rin - dict_rin) * (ratio_rin - dict_rin) ;

//         // if (dist_sq < min_distance)
//         // {
//         //     min_distance = dist_sq;
//         //     fault_index = i + 1; // 返回 1~6 的代号
//         // }
//     }

//     return fault_index;
// }
