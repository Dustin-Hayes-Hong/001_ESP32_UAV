#include "IIC.h"

//I2C用于MPU6050姿态传感器、SPL06气压传感器

#define DATA_LENGTH 128         /*!< Data buffer length */  //数据缓冲区长度

i2c_master_dev_handle_t dev1_handle;
i2c_master_dev_handle_t dev2_handle;

void I2C_Init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    
    //挂载device1
    i2c_device_config_t dev1_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68, //MPU6050_SENSOR_ADDR
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev1_cfg, &dev1_handle));

    //挂载device2
    i2c_device_config_t dev2_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x10, //SPL06_001_SENSOR_ADDR
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev2_cfg, &dev2_handle));
}

int i2c_write(i2c_master_dev_handle_t handle,uint8_t *write_buffer,size_t write_size)
{
    i2c_master_transmit(handle, write_buffer, write_size, -1);
    return ESP_OK;
}

int i2c_read(i2c_master_dev_handle_t handle,uint8_t *read_buffer,size_t read_size)
{
    i2c_master_receive(handle, read_buffer, read_size, -1);
    return ESP_OK;
}

int i2c_write_read(i2c_master_dev_handle_t handle,uint8_t *write_buffer,size_t write_size,uint8_t *read_buffer,size_t read_size)
{
    i2c_master_transmit_receive(handle, write_buffer, write_size, read_buffer, read_size, -1);
    return ESP_OK;
}

int i2c_write_reg(i2c_master_dev_handle_t handle,uint8_t reg_addr,uint8_t data)
{
    uint8_t reg_addr_buffer = reg_addr;
    uint8_t data_buffer = data;
    i2c_master_transmit_multi_buffer_info_t write_buffer[2] = {
        {.write_buffer = &reg_addr_buffer, .buffer_size=1},
        {.write_buffer = &data_buffer, .buffer_size=1}
    };
    i2c_master_multi_buffer_transmit(handle, write_buffer, 2, -1);
    return ESP_OK;
}