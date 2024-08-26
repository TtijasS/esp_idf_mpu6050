// i2c_config.c
#include "my_i2c_com.h"

// Define the I2C master bus configuration
const i2c_master_bus_config_t i2c_master_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA_IO,
    .scl_io_num = I2C_SCL_IO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

// Define the I2C device configuration
const i2c_device_config_t i2c_master_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU_ADR_REG,
    .scl_speed_hz = I2C_FREQ_HZ
};

// Initialize handles (can be allocated or further defined elsewhere)
i2c_master_bus_handle_t i2c_master_bus_handle;
i2c_master_dev_handle_t i2c_master_dev_handle;

/**
 * @brief Initialize the I2C bus
 *
 * Add new master bus and MPU6050 as a slave device
 * Slave address of the MPU6050 sensor is 0x68
 *
 * @param void
 * @return 0 OK
 * @return -1 failed to create new master bus
 * @return -2 failed to add new device to master bus
 * @return -3 failed to probe the added device
 */
int i2c_init()
{
    int error_code = 0;
    // Initialize the I2C bus
    if((error_code = i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus_handle)) != ESP_OK)
    {
        return -1;
    }
    if((error_code = i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_master_device_config, &i2c_master_dev_handle)) != ESP_OK)
    {
        return -2;
    }
    // Probe the slave device
    if((error_code = i2c_master_probe(i2c_master_bus_handle, MPU_ADR_REG, I2C_TIMEOUT_MS)) != ESP_OK)
    {
        return -3;
    }
    return 0;
}