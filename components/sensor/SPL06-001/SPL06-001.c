#include "SPL06-001.h"
#include "IIC.h"

//https://blog.csdn.net/qq_40598185/article/details/119347845?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522b1b74e98a2848eda046f0182781c7856%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=b1b74e98a2848eda046f0182781c7856&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-119347845-null-null.142%5Ev100%5Epc_search_result_base1&utm_term=spl06&spm=1018.2226.3001.4187
//感觉SPL06-001的代码逻辑有些复杂，MPU6050就是初始化、带校准的读取数据
//SPL06-001在初始化里校准了零偏

/*
 * SPL06-001 气压计驱动程序
 *
 *  该驱动程序实现了对 SPL06-001 气压计的初始化、校准、数据读取和处理等功能。
 *
 *  主要流程如下：
 *
 *  1. 芯片连接检测 (SPL06_test)
 *      - 检测 SPL06-001 是否连接成功，通过读取芯片 ID 进行判断。
 *
 *  2. 校准参数读取 (spl0601_get_calib_param)
 *      - 从 SPL06-001 内部寄存器读取校准系数，用于后续的温度和气压计算。
 *
 *  3. 气压计初始化 (SPL06Init)
 *      - 配置传感器参数：
 *          - 配置压力寄存器：设置采样率和过采样率，例如每秒刷新 4 次，过采样率 64 次（高精度模式）。
 *          - 配置温度寄存器：设置采样率和过采样率，例如使用外部传感器，3.6ms 测量一次，采样 1 次（默认模式）。
 *          - 配置温度和压力结果移位：根据需要进行配置，通常需要移位以提高精度。
 *      - 启动连续测量模式：使传感器进入连续测量压力和温度的模式。
 *      - 进行气压校准零偏：消除由于传感器自身或环境因素导致的零点漂移。
 *
 *  4. 数据读取 (SPL06GetPressure)
 *      - 从 SPL06-001 内部寄存器读取原始压力和温度数据。
 *
 *  5. 数据处理
 *      - 计算实际气压值 (spl0601_get_pressure)：根据原始压力数据、温度数据和校准系数，计算出实际的气压值。
 *      - 计算实际温度值 (spl0601_get_temperature)：根据原始温度数据和校准系数，计算出实际的温度值。
 *      - 滤波 (lowV)：对气压数据进行滤波处理，降低噪声干扰，可以使用一阶低通滤波等算法。
 *      - 压力转海拔高度 (SPL06PressureToAltitude)：根据气压值，计算出对应的海拔高度。
 *
 *  6. 数据返回和零偏校准 (spl06_get_data, Height_Get)
 *      - 将计算得到的温度、气压和海拔高度返回给调用者。
 *      - 在适当的时机（例如初始化后或定期进行），重新进行零偏校准，以保证测量精度。
 */

// SPL06-001 气压计相关参数
// 校准数据
spl06CalibCoefficient_t spl06Calib; // 气压计校准数据，存储从传感器读取的校准系数

// 原始数据
int32_t SPL06RawPressure;      // 气压计原始数据，未经校准和补偿的原始压力值
int32_t SPL06RawTemperature;   // 温度计原始数据，未经校准和补偿的原始温度值

// 校准和比例因子
int AirPressure_calibration;   // 气压计校准标志，用于指示是否需要进行校准
int32_t kp;                   // 压力计比例因子，用于压力计算的比例系数
int32_t kt;                   // 温度计比例因子，用于温度计算的比例系数
float asl_offset;             // 气压计零偏，用于校准海拔高度的偏移量

// 计算结果
float pressure;               // 气压值 (单位：hPa 或 Pa，取决于具体应用)
float temperature;            // 温度值 (单位：摄氏度)
float asl;                    // 海拔高度 (单位：米)
alt_bro ALT_BRO;             // 海拔高度和垂直速度数据

//相关常数
static  const uint8_t num_map[8] = {1,2,4,8,16,32,64,128};  // 2 的幂次方表，用于配置 SPL06 的过采样率或采样率
static  const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};  // SPL06 压力计和温度计比例因子，用于原始数据转换

//检测SPL06-001是否连接
int spl06_test(void)
{
    uint8_t receive[1]={0};
    uint8_t reg_address[1]={SPL06_001_WHO_AM_I_REG_ADDR};
    if(i2c_write_read(dev2_handle,reg_address,1,receive,1)==ESP_OK) printf("Read reg sucess\n");
    printf("receive[0]=%x\n",receive[0]);
    if(receive[0]==SPL06_001_SENSOR_ADDR) {
        printf("SPL06-001 connect sucess\n");
        return ESP_OK;}
    else {
        printf("SPL06-001 connect error\n");
        return ESP_FAIL;}
}

//获取气压计校准参数
int spl06_get_calib_param(void)
{
    uint8_t buffer[SPL06_CALIB_COEFFICIENT_LENGTH] = {0};
    uint8_t reg_address[1] = {SPL06_COEFFICIENT_CALIB_REG};
    i2c_write_read(dev2_handle, reg_address, 1,buffer,SPL06_CALIB_COEFFICIENT_LENGTH);

    spl06Calib.c0 = (int16_t)buffer[0] << 4 | buffer[1] >> 4;
    spl06Calib.c0 = (spl06Calib.c0 & 0x0800) ? (spl06Calib.c0 | 0xF000) : spl06Calib.c0;

    spl06Calib.c1 = (int16_t)(buffer[1] & 0x0F) << 8 | buffer[2];
    spl06Calib.c1 = (spl06Calib.c1 & 0x0800) ? (spl06Calib.c1 | 0xF000) : spl06Calib.c1;

    spl06Calib.c00 = (int32_t)buffer[3] << 12 | (int32_t)buffer[4] << 4 | (int32_t)buffer[5] >> 4;
    spl06Calib.c00 = (spl06Calib.c00 & 0x080000) ? (spl06Calib.c00 | 0xFFF00000) : spl06Calib.c00;

    spl06Calib.c10 = (int32_t)(buffer[5] & 0x0F) << 16 | (int32_t)buffer[6] << 8 | (int32_t)buffer[7];
    spl06Calib.c10 = (spl06Calib.c10 & 0x080000) ? (spl06Calib.c10 | 0xFFF00000) : spl06Calib.c10;

    spl06Calib.c01 = (int16_t)buffer[8] << 8 | buffer[9];
    spl06Calib.c11 = (int16_t)buffer[10] << 8 | buffer[11];
    spl06Calib.c20 = (int16_t)buffer[12] << 8 | buffer[13];
    spl06Calib.c21 = (int16_t)buffer[14] << 8 | buffer[15];
    spl06Calib.c30 = (int16_t)buffer[16] << 8 | buffer[17];

    return ESP_OK;
}

//获取气压计原始数据
void SPL06GetPressure(void)
{
    uint8_t data[SPL06_DATA_FRAME_SIZE];

    i2c_write_read(dev2_handle, SPL06_PRESSURE_MSB_REG,1, data,SPL06_DATA_FRAME_SIZE); // 读取SPL06 ID

    SPL06RawPressure = (int32_t)data[0] << 16 | (int32_t)data[1] << 8 | (int32_t)data[2];
    SPL06RawPressure = (SPL06RawPressure & 0x800000) ? (0xFF000000 | SPL06RawPressure) : SPL06RawPressure;

    SPL06RawTemperature = (int32_t)data[3] << 16 | (int32_t)data[4] << 8 | (int32_t)data[5];
    SPL06RawTemperature = (SPL06RawTemperature & 0x800000) ? (0xFF000000 | SPL06RawTemperature) : SPL06RawTemperature;
}

//计算气压计实际数据
float spl0601_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;

    fTsc = rawTemperature / (float)kt;
    fTCompensate = spl06Calib.c0 * 0.5 + spl06Calib.c1 * fTsc;
    return fTCompensate;
}

//计算温度计实际数据
float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemperature / (float)kt;
    fPsc = rawPressure / (float)kp;
    qua2 = spl06Calib.c10 + fPsc * (spl06Calib.c20 + fPsc * spl06Calib.c30);
    qua3 = fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);
    // qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

    fPCompensate = spl06Calib.c00 + fPsc * qua2 + fTsc * spl06Calib.c01 + qua3;
    // fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
    return fPCompensate;
}

float alt_3, height;//海拔高度

//压力转海拔高度
float SPL06PressureToAltitude(float pressure /*, float* groundPressure, float* groundTemp*/)
{
    alt_3 = (101000 - pressure) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * (101000 - pressure) * 100.0f;
    return height;
}

//低通滤波
float lowV(float com)
{
    static float iLastData;                            // 上一次值
    float iData;                                       // 本次计算值
    float dPower = 0.1;                                // 滤波系数
    iData = (com * dPower) + (1 - dPower) * iLastData; // 计算
    iLastData = iData;                                 // 存贮本次数据
    return iData;                                      // 返回数据
}

//气压计数据处理
void spl06_get_data(float *pressure, float *temperature, float *asl)
{
    static float t;
    static float p;

    SPL06GetPressure();

    t = spl0601_get_temperature(SPL06RawTemperature);
    p = spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);

    //	pressureFilter(&p,pressure);
    *temperature = (float)t; /*单位度*/
    *pressure = (float)p;    /*单位hPa*/

    *pressure = lowV(*pressure);
    *asl = SPL06PressureToAltitude(*pressure); /*转换成海拔*/
}

//气压校准零偏
int spl06_offset(void)
{
    static uint16_t cnt_a=0;
    float pressure, temperature, asl;

    if(cnt_a==0)
    {
    	for (uint16_t i = 0; i < 200; i++)
    	{
    		spl06_get_data(&pressure, &temperature, &asl);
    	}

    }

    spl06_get_data(&pressure, &temperature, &asl);
    asl_offset += asl;

    if(cnt_a==200)
    {
        asl_offset = asl_offset / 200.0f;
        cnt_a = 0;
        return 1;
    }
      cnt_a++;
      return 0;
}

int spl06_init(void)
{
    printf("********************SPL06-001初始化开始********************\n");

	if(spl06_test() == ESP_OK){

		 // 读取校准数据
            spl06_get_calib_param();

		    if(i2c_write_reg(dev2_handle,0x06,Pressure_measurement_rate << 4 | Pressure_oversampling_rate) == ESP_OK) printf("1.配置压力寄存器，每秒刷新%d次，过采样率%d次（高精度）\n",num_map[Pressure_measurement_rate],num_map[Pressure_oversampling_rate]);
		    kp = scaleFactor[Pressure_oversampling_rate];

		    if(i2c_write_reg(dev2_handle,0x07,Temperature_measurement_rate << 4 | Temperature_oversampling_rate | 0x80) == ESP_OK) printf("2.配置温度寄存器，使用外部传感器，3.6ms测量一次，采样1次（默认）\n");
		    kt = scaleFactor[Temperature_oversampling_rate];

			if(i2c_write_reg(dev2_handle,0x09,0x0C) == ESP_OK) printf("3.温度和压力结果移位\n");

			if(i2c_write_reg(dev2_handle,0x08,0x07) == ESP_OK) printf("4.开始连续测量压力和温度\n");

		    if(i2c_write_reg(dev2_handle,0x08,0x07) == ESP_OK) printf("4.开始连续测量压力和温度\n");

		    AirPressure_calibration = 1;
		    while(!spl06_offset()); //校准气压计零偏

	}

    printf("********************SPL06-001初始化完成********************\n");
    return ESP_OK;
}

void Height_Get(void) // 取得高度
{

    // 读取气压计高度
    spl06_get_data(&pressure, &temperature, &asl);
    ALT_BRO.h = asl - asl_offset;

    if(AirPressure_calibration == 1){
    	if(spl06_offset() == 1)  AirPressure_calibration = 2;
    }

}