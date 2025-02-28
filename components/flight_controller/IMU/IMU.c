#include "IMU.h"


// 定义全局常量
#define KP_NEW      0.8f         // 互补滤波当前数据权重
#define KP_OLD      0.01f        // 互补滤波历史数据权重
#define ACC_GAIN    0.0001220f   // 加速度转换因子（满量程±4g，LSB = 8/65535）
#define GYRO_GAIN   0.0609756f   // 角速度转换因子（满量程±2000°/s，LSB = 4000/65535）
#define GYRO_GR     0.0010641f   // 角速度转换为弧度/秒（π/180 * LSB）
#define FILTER_N    12           // 滑动窗口滤波缓存大小
#define M_PI_F      3.1416f      // 圆周率近似值
#define RAD_TO_DEG  57.29578f    // 弧度转角度系数 (180/π)

// 全局变量
state_t state;          //状态结构体

static FLOAT_XYZ Acc_filtold;      // 加速度历史滤波值
static float accb[3];              // 加速度数组（机体坐标系）
float DCMgb[3][3];                 // 方向余弦矩阵（惯性系转机体系）
uint8_t AccbUpdate = 0;            // 加速度更新标志

// 确定元素在数组中的位置（快速排序辅助函数）
static float find_position(float *a, int low, int high) {
    float val = a[low]; // 选取基准值
    while (low < high) {
        while (low < high && a[high] >= val) high--; // 从右向左找小于基准的值
        a[low] = a[high];
        while (low < high && a[low] <= val) low++;  // 从左向右找大于基准的值
        a[high] = a[low];
    }
    a[low] = val; // 放置基准值
    return (float)low;
}

// 快速排序实现
static void quick_sort(float *a, int low, int high) {
    if (low < high) {
        int pos = (int)find_position(a, low, high); // 确定一个位置
        quick_sort(a, low, pos - 1);                // 递归排序左半部分
        quick_sort(a, pos + 1, high);               // 递归排序右半部分
    }
}

// 滑动窗口去极值滤波
void sort_aver_filter_xyz(INT16_XYZ *acc, FLOAT_XYZ *acc_filt, uint8_t n) {
    static float bufx[FILTER_N], bufy[FILTER_N], bufz[FILTER_N]; // 静态缓冲区
    static uint8_t cnt = 0, flag = 1; // 计数器和初始化标志
    float temp_x = 0, temp_y = 0, temp_z = 0;

    // 存入新数据
    bufx[cnt] = acc->X;
    bufy[cnt] = acc->Y;
    bufz[cnt] = acc->Z;
    cnt++;

    // 如果缓冲区未满，则不计算
    if (cnt < n && flag) return;
    flag = 0; // 数据填满后停止检查

    // 对三轴数据排序
    quick_sort(bufx, 0, n - 1);
    quick_sort(bufy, 0, n - 1);
    quick_sort(bufz, 0, n - 1);

    // 去掉最大最小值后求平均
    for (uint8_t i = 1; i < n - 1; i++) {
        temp_x += bufx[i];
        temp_y += bufy[i];
        temp_z += bufz[i];
    }

    acc_filt->X = temp_x / (n - 2);
    acc_filt->Y = temp_y / (n - 2);
    acc_filt->Z = temp_z / (n - 2);

    if (cnt >= n) cnt = 0; // 重置计数器以循环使用缓冲区
}

// 数据预处理（加速度和陀螺仪单位转换及滤波）
void prepare_data(sensorData_t *sensor_data) {
    static uint8_t iir_mode = 1; // IIR滤波开关

    // 对加速度进行滑动窗口去极值滤波
    sort_aver_filter_xyz(&sensor_data->acc_n, &sensor_data->acc_f, FILTER_N);

    // 加速度转换为米/秒² (g * 9.80665)
    sensor_data->acc_f.X *= ACC_GAIN * 9.80665f;
    sensor_data->acc_f.Y *= ACC_GAIN * 9.80665f;
    sensor_data->acc_f.Z *= ACC_GAIN * 9.80665f;

    // 陀螺仪转换为弧度/秒
    sensor_data->gyro_f.X = sensor_data->gyro_n.X * GYRO_GR;
    sensor_data->gyro_f.Y = sensor_data->gyro_n.Y * GYRO_GR;
    sensor_data->gyro_f.Z = sensor_data->gyro_n.Z * GYRO_GR;

    // 可选的IIR互补滤波
    if (iir_mode) {
        sensor_data->acc_f.X = sensor_data->acc_f.X * KP_NEW + Acc_filtold.X * KP_OLD;
        sensor_data->acc_f.Y = sensor_data->acc_f.Y * KP_NEW + Acc_filtold.Y * KP_OLD;
        sensor_data->acc_f.Z = sensor_data->acc_f.Z * KP_NEW + Acc_filtold.Z * KP_OLD;

        // 更新历史值
        Acc_filtold = sensor_data->acc_f;
    }

    // 更新加速度数组
    accb[0] = sensor_data->acc_f.X;
    accb[1] = sensor_data->acc_f.Y;
    accb[2] = sensor_data->acc_f.Z;

    // 检查加速度数据有效性
    if (accb[0] && accb[1] && accb[2]) AccbUpdate = 1;
}

// 快速计算 1/sqrt(x)
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    long i = *(long*)&x;
    i = 0x5f375a86 - (i >> 1); // 魔数优化
    float y = *(float*)&i;
    y *= (1.5f - (halfx * y * y)); // 一次牛顿迭代
    return y;
}

// IMU融合参数
#define KP         1.1f    // 比例增益，控制加速度计收敛速率
#define KI         0.001f  // 积分增益，控制陀螺仪偏差收敛速度
#define HALF_T     0.005f  // 采样周期一半（5ms）

// 四元数和误差积分
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始姿态四元数
static float ex_int = 0, ey_int = 0, ez_int = 0;          // 积分误差

// IMU姿态更新（基于四元数融合）
void imu_update(sensorData_t *sensor_data, state_t *state) {
    float ax = sensor_data->acc_f.X, ay = sensor_data->acc_f.Y, az = sensor_data->acc_f.Z;
    float gx = sensor_data->gyro_f.X, gy = sensor_data->gyro_f.Y, gz = sensor_data->gyro_f.Z;
    float norm, vx, vy, vz, ex, ey, ez;

    // 若加速度数据无效则退出
    if (ax * ay * az == 0) return;

    // 四元数平方项
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

    // 归一化加速度（机体坐标系重力向量）
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 陀螺仪估计的重力向量（机体坐标系）
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 计算误差（加速度测量值与估计值的叉乘）
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 误差积分
    ex_int += ex * KI;
    ey_int += ey * KI;
    ez_int += ez * KI;

    // 陀螺仪数据校正（PI控制）
    gx += KP * ex + ex_int;
    gy += KP * ey + ey_int;
    gz += KP * ez + ez_int;

    // 四元数微分方程更新
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * HALF_T;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * HALF_T;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * HALF_T;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * HALF_T;

    // 四元数归一化
    norm = inv_sqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 更新方向余弦矩阵（DCM）
    DCMgb[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    DCMgb[0][1] = 2 * (q1q2 + q0q3);
    DCMgb[0][2] = 2 * (q1q3 - q0q2);
    DCMgb[1][0] = 2 * (q1q2 - q0q3);
    DCMgb[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    DCMgb[1][2] = 2 * (q2q3 + q0q1);
    DCMgb[2][0] = 2 * (q1q3 + q0q2);
    DCMgb[2][1] = 2 * (q2q3 - q0q1);
    DCMgb[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

    // 计算欧拉角（单位：度）
    float temp = sensor_data->gyro_f.Z * RAD_TO_DEG * 0.01f; // 偏航角增量
    if (temp > 0.03f || temp < -0.01f) state->attitude.yaw -= temp; // 过滤微小漂移
    state->attitude.roll = -asin(2 * (q1q3 - q0q2)) * RAD_TO_DEG;
    state->attitude.pitch = atan2(2 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3) * RAD_TO_DEG;
}

