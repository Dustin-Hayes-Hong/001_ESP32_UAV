#ifndef __filter_h
#define __filter_h

#include <stdio.h>
#include "math.h"

#define IIR_SHIFT         8

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

// 定义一个用于二阶低通滤波器的结构体
typedef struct
{
    float a1;            // 滤波器系数 a1，影响滤波器的频率响应
    float a2;            // 滤波器系数 a2，影响滤波器的频率响应
    float b0;            // 滤波器系数 b0，输入信号的当前采样系数
    float b1;            // 滤波器系数 b1，输入信号的前一次采样系数
    float b2;            // 滤波器系数 b2，输入信号的前两次采样系数
    float delay_element_1;  // 延迟元素1，存储前一次的输出值
    float delay_element_2;  // 延迟元素2，存储前两次的输出值
} lpf2pData;

void lpf2p_init(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2p_set_cutoff_freq(lpf2pData *lpf_data, float sample_freq, float cutoff_freq);
float lpf2p_apply(lpf2pData* lpfData, float sample);
float lpf2p_reset(lpf2pData* lpfData, float sample);



#endif	// __filter_h
