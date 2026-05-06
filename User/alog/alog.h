#ifndef __ALGO_H
#define __ALGO_H

#include "stdint.h"

// 第二问：参数计算接口
float Algo_Measure_Gain(float v_in_amp, float v_out_amp);
float Algo_Measure_Rin(float v_source, float v_in_amp);
float Algo_Measure_Rout(float v_out_open, float v_out_load);

// 第三问：故障诊断接口
// 返回值可以是故障类型的代号，例如 0:正常, 1:R1翻倍, 2:R2翻倍...
uint8_t Algo_Diagnosis_Fault(float current_gain, float current_rin, float current_rout, float current_vdc);

#endif /* __ALGO_H */