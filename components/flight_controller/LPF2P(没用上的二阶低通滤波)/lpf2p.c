/**
 * @brief 二阶低通滤波，实际无调用
 */
#include "filter.h"

// 全局宏定义
#define M_PI_F      3.14159265f  // 圆周率，定义为浮点常量
#define IIR_SHIFT   8            // IIR滤波器的移位因子，用于定点运算精度控制

/**
 * @brief 单通道IIR低通滤波器（定点运算实现）
 * @param in 输入信号（32位整数）
 * @param attenuation 衰减因子（控制滤波强度，范围 [1, 1<<IIR_SHIFT]）
 * @param filt 滤波器状态（历史值，32位整数指针）
 * @return 滤波后的输出（16位整数）
 */
int16_t iir_lp_filter_single(int32_t in, int32_t attenuation, int32_t *filt) {
    // 限制衰减因子范围
    attenuation = (attenuation > (1 << IIR_SHIFT)) ? (1 << IIR_SHIFT) : 
                  (attenuation < 1) ? 1 : attenuation;

    // 提升输入精度（左移 IIR_SHIFT 位）
    int32_t in_scaled = in << IIR_SHIFT;
    int32_t filt_tmp = *filt;

    // IIR滤波计算：filt_new = filt_old + (in_scaled - filt_old) * attenuation / 2^IIR_SHIFT
    filt_tmp += (((in_scaled - filt_tmp) >> IIR_SHIFT) * attenuation);
    
    // 输出缩放并四舍五入（从32位降到16位）
    int16_t out = (filt_tmp >> IIR_SHIFT) + 
                  ((filt_tmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
    
    // 更新滤波器状态
    *filt = filt_tmp;
    return out;
}

/**
 * @brief 初始化二阶低通滤波器
 * @param lpf_data 滤波器数据结构体指针
 * @param sample_freq 采样频率（Hz）
 * @param cutoff_freq 截止频率（Hz）
 */
void lpf2p_init(lpf2pData *lpf_data, float sample_freq, float cutoff_freq) {
    if (lpf_data == NULL || cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        return; // 输入无效时直接退出
    }
    lpf2p_set_cutoff_freq(lpf_data, sample_freq, cutoff_freq);
}

/**
 * @brief 设置二阶低通滤波器的截止频率并计算滤波系数
 * @param lpf_data 滤波器数据结构体指针
 * @param sample_freq 采样频率（Hz）
 * @param cutoff_freq 截止频率（Hz）
 */
void lpf2p_set_cutoff_freq(lpf2pData *lpf_data, float sample_freq, float cutoff_freq) {
    // 计算频率比和中间变量
    float fr = sample_freq / cutoff_freq;          // 采样频率与截止频率的比值
    float ohm = tanf(M_PI_F / fr);                 // 预畸变频率（Butterworth设计）
    float cos_term = cosf(M_PI_F / 4.0f);          // 预计算 cos(π/4) = √2/2
    float ohm_sq = ohm * ohm;                      // ohm 的平方项
    float c = 1.0f + 2.0f * cos_term * ohm + ohm_sq; // 归一化因子

    // 计算滤波器系数
    lpf_data->b0 = ohm_sq / c;                     // 输入当前采样系数
    lpf_data->b1 = 2.0f * lpf_data->b0;            // 输入前一次采样系数
    lpf_data->b2 = lpf_data->b0;                   // 输入前两次采样系数
    lpf_data->a1 = 2.0f * (ohm_sq - 1.0f) / c;     // 输出前一次反馈系数
    lpf_data->a2 = (1.0f - 2.0f * cos_term * ohm + ohm_sq) / c; // 输出前两次反馈系数

    // 重置延迟单元
    lpf_data->delay_element_1 = 0.0f;
    lpf_data->delay_element_2 = 0.0f;
}

/**
 * @brief 应用二阶低通滤波到单个样本
 * @param lpf_data 滤波器数据结构体指针
 * @param sample 输入样本
 * @return 滤波后的输出值
 */
float lpf2p_apply(lpf2pData *lpf_data, float sample) {
    // 计算当前延迟单元（基于输入和反馈）
    float delay_element_0 = sample - lpf_data->delay_element_1 * lpf_data->a1 
                           - lpf_data->delay_element_2 * lpf_data->a2;

    // 检查数值稳定性，若无效则直接使用输入
    delay_element_0 = isfinite(delay_element_0) ? delay_element_0 : sample;

    // 计算输出（基于输入和前两次延迟）
    float output = delay_element_0 * lpf_data->b0 + 
                   lpf_data->delay_element_1 * lpf_data->b1 + 
                   lpf_data->delay_element_2 * lpf_data->b2;

    // 更新延迟单元（滑动窗口）
    lpf_data->delay_element_2 = lpf_data->delay_element_1;
    lpf_data->delay_element_1 = delay_element_0;

    return output;
}

/**
 * @brief 重置二阶低通滤波器状态并应用滤波
 * @param lpf_data 滤波器数据结构体指针
 * @param sample 输入样本（用于初始化状态）
 * @return 滤波后的初始输出值
 */
float lpf2p_reset(lpf2pData *lpf_data, float sample) {
    // 计算稳态延迟值（假设输入恒定）
    float gain_sum = lpf_data->b0 + lpf_data->b1 + lpf_data->b2;
    float dval = (gain_sum != 0.0f) ? (sample / gain_sum) : sample;

    // 初始化延迟单元
    lpf_data->delay_element_1 = dval;
    lpf_data->delay_element_2 = dval;

    // 返回滤波结果
    return lpf2p_apply(lpf_data, sample);
}



