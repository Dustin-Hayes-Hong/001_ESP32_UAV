#include "MPU6050.h"
#include "IIC.h"

sensorData_t sensorData;

int mpu6050_test(void)
{
    uint8_t receive[1]={0};
    uint8_t reg_address[1]={MPU6050_WHO_AM_I_REG_ADDR};
    if(i2c_write_read(dev1_handle,reg_address,1,receive,1)==ESP_OK) printf("Read reg sucess\n");
    printf("receive[0]=%x\n",receive[0]);
    if(receive[0]==MPU6050_SENSOR_ADDR) {
        printf("MPU6050 connect sucess\n");
        return ESP_OK;}
    else {
        printf("MPU6050 connect error\n");
        return ESP_FAIL;}
}

int mpu6050_init(void)
{
	printf("********************MPU6050初始化开始********************\n");
	if(mpu6050_test() == ESP_OK){//检测传感器成功连接就进行配置

		if(i2c_write_reg(dev1_handle,0x6B,0x80) == ESP_OK) printf("1.复位MPU6050\n");
		vTaskDelay(100 / portTICK_PERIOD_MS);//等待100ms
		if(i2c_write_reg(dev1_handle,0x6B,0x01) == ESP_OK) printf("2.唤醒MPU6050,并选择陀螺仪x轴PLL为时钟源\n");
		if(i2c_write_reg(dev1_handle,0x38,0x00) == ESP_OK) printf("3.禁止中断\n");
		if(i2c_write_reg(dev1_handle,0x1B,0x18) == ESP_OK) printf("4.禁陀螺仪满量程+-2000度/秒(最低分辨率 = 2^15/2000 = 16.4LSB/度/秒)止中断\n");
		if(i2c_write_reg(dev1_handle,0x1C,0x08) == ESP_OK) printf("5.加速度满量程+-4g  (最低分辨率 = 2^15/4g = 8196LSB/g)\n");
		if(i2c_write_reg(dev1_handle,0x1A,0x04) == ESP_OK) printf("6.设置陀螺的输出为1KHZ,DLPF=20HZ\n");
		if(i2c_write_reg(dev1_handle,0x19,0x00) == ESP_OK) printf("7.采样分频(采样频率 = 陀螺仪输出频率 / (1+DIV), 采样频率1000hz)\n");
		if(i2c_write_reg(dev1_handle,0x37,0x02) == ESP_OK) printf("8.MPU 可直接访问MPU6050辅助12C\n");
	}
	printf("********************MPU6050初始化完成********************\n\n");
    return ESP_OK;
}

int mpu6050_read_data(sensorData_t *data)
{
	static uint8_t reg_address[1] = {0x3B};//MPU6050加速度寄存器地址
	static uint8_t test[14] = {0};//接收数据缓冲区
	static uint16_t mpu6050_temp = 0;
	if(i2c_write_read(dev1_handle,reg_address,1,test,14)==ESP_OK)
	{
		
		data->acc_n.X = (test[0]  << 8 | test[1]);
		data->acc_n.Y = (test[2]  << 8 | test[3]);
		data->acc_n.Z = (test[4]  << 8 | test[5]);

		mpu6050_temp  = (test[6]  << 8 | test[7]);

		data->gyro_n.X = (test[8]  << 8 | test[9]);
		data->gyro_n.Y = (test[10] << 8 | test[11]);
		data->gyro_n.Z = (test[12] << 8 | test[13]); 

		printf("MPU6050加速度原始数据: acc.x = %d acc.y = %d acc.z = %d\n",data->acc_n.X,data->acc_n.Y,data->acc_n.Z);
		printf("MPU6050陀螺仪原始数据: gyro.x = %d gyro.y = %d gyro.z = %d\n",data->gyro_n.X,data->gyro_n.Y,data->gyro_n.Z);
		printf("MPU6050陀螺仪原始数据: mpu6050_temp = %d\n",mpu6050_temp);

		return ESP_OK;
	}
	else return ESP_FAIL;
}