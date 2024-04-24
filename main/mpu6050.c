#include "mpu6050.h"


/**
 * @brief Initial MPU6050 setup.
 *
 * Set up the MPU6050 registers for the first time. MPU is set to wake up, filter freq is set to 5Hz,
 * accelerometer to +-8g full scale range and gyroscope to +-1000 deg/s full scale range.
 * The FIFO buffer is set to store accelerometer and gyroscope data.
 *
 * @param i2c_buffer: struct with write_buffer, read_buffer
 * @return void
 */
void mpu_initial_setup(i2c_buffer_type *i2c_buffer)
{

    // Wake up the MPU6050
    i2c_buffer->write_buffer[0] = MPU_PWR_REG;
    i2c_buffer->write_buffer[1] = 0x00; // wake up signal
    mpu_transmit(i2c_buffer, 2);        // write

    // Set filter freq
    i2c_buffer->write_buffer[0] = MPU_FILTER_FREQ_REG;     // Filter freq register
    i2c_buffer->write_buffer[1] = MPU_FILTER_FREQ_REG_SET; // Filter freq
    mpu_transmit(i2c_buffer, 2);

    // Set accelerometer to +-8g full scale range
    i2c_buffer->write_buffer[0] = MPU_ACCEL_CFG_REG; // Accel register
    i2c_buffer->write_buffer[1] = MPU_ACCEL_REG_SET; //  Accel register setting
    mpu_transmit(i2c_buffer, 2);

    // Set gyroscope to +-1000 deg/s full scale range
    i2c_buffer->write_buffer[0] = MPU_GYRO_CFG_REG; // Gyro register
    i2c_buffer->write_buffer[1] = MPU_GYRO_REG_SET; // Gyro register setting
    mpu_transmit(i2c_buffer, 2);

    // Set what data is stored in the FIFO buffer
    i2c_buffer->write_buffer[0] = MPU_FIFO_ENBL_MASK_REG;  // Fifo enable register
    i2c_buffer->write_buffer[1] = MPU_FIFO_ENABLE_REG_SET; // Fifo enable gyro and accel data setting
    mpu_transmit(i2c_buffer, 2);

    // Check the integrity of the configured registers
    i2c_buffer->write_buffer[0] = MPU_PWR_REG;
    mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x00h (0d)

    i2c_buffer->write_buffer[0] = MPU_FILTER_FREQ_REG;
    mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x06h (6d)

    i2c_buffer->write_buffer[0] = MPU_ACCEL_CFG_REG;
    mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x10h (16d)

    i2c_buffer->write_buffer[0] = MPU_GYRO_CFG_REG;
    mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x10h (16d)

    i2c_buffer->write_buffer[0] = MPU_FIFO_ENBL_MASK_REG;
    mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x78h (120d)
}

/**
 * @brief Transmit MPU register data and read it's value.
 *
 * MCU_write -> REG_ADDR -> MCU_read <- REG_VALUE(s)
 *
 * Before transmision, you have to first setup the write_buffer
 * i2c_buffer.write_buffer[write_buf_size]
 * [0] - register address
 *
 * To read multiple values, set the read_buf_size to greater than 1.
 * Reading multiple values usually means reading subsequent registers.
 *
 * @param i2c_buffer: struct with read_buffer, write_buffer
 * @param write_buf_size: i2c_buffer.wirte_buffer size (how many values to transmit)
 * @param read_buf_size: i2c_buffer.read_buffer size (how many values to receive)
 *
 * @return void
 */
void mpu_transmit_receive(i2c_buffer_type *i2c_buffer, uint8_t write_buf_size, uint8_t read_buf_size)
{
    if (sizeof(i2c_buffer->write_buffer) < write_buf_size)
    {
        ESP_LOGI(TAG, "The allocated i2c_buffer.write_buffer size is smaller than %d", write_buf_size);
        return;
    }
    if (sizeof(i2c_buffer->read_buffer) < read_buf_size)
    {
        ESP_LOGI(TAG, "The allocated i2c_buffer.read_buffer size is smaller than %d", read_buf_size);
        return;
    }
    ESP_ERROR_CHECK(i2c_master_transmit_receive(master_dev_handle, i2c_buffer->write_buffer, write_buf_size, i2c_buffer->read_buffer, read_buf_size, I2C_TIMEOUT_MS));
}

/**
 * @brief Write data to specific MPU6050 register
 *
 * MCU_write -> REG_ADDR -> REG_VALUE
 *
 * Before transmision, you have to first setup the write_buffer
 * i2c_buffer->write_buffer[I2C_WRITE_BUFF_SIZE]
 * [0] - register address
 * [1] - data to write
 *
 * @param i2c_buffer: struct with read_buffer, write_buffer
 * @param write_buf_size: i2c_buffer.wirte_buffer size (how many values to transmit)
 *
 * @return void
 */
void mpu_transmit(i2c_buffer_type *i2c_buffer, uint8_t write_buf_size)
{
    if (sizeof(i2c_buffer->write_buffer) < write_buf_size)
    {
        ESP_LOGI(TAG, "The allocated i2c_buffer.write_buffer size is smaller than %d", write_buf_size);
        return;
    }
    ESP_ERROR_CHECK(i2c_master_transmit(master_dev_handle, i2c_buffer->write_buffer, write_buf_size, I2C_TIMEOUT_MS));
}

/**
 * @brief Enable the FIFO buffer of the MPU6050 sensor
 *
 * FIFO buffer automatically reads the data from the MPU6050 sensor and stores it in the buffer.
 *
 * @param i2c_buffer: struct with write_buffer, read_buffer
 * @return void
 */
void mpu_fifo_enable(i2c_buffer_type *i2c_buffer)
{
    i2c_buffer->write_buffer[0] = MPU_FIFO_CTRL_REG;
    i2c_buffer->write_buffer[1] = 0x40; // Enable FIFO
    mpu_transmit(i2c_buffer, 2);
}

/**
 * @brief Reset the FIFO buffer of the MPU6050 sensor
 *
 * @param i2c_buffer: struct with write_buffer, read_buffer
 * @return void
 */
void mpu_fifo_reset(i2c_buffer_type *i2c_buffer)
{
    // Fifo reset bit = 0x04 and fifo enable bit = 0x40
    // Therefore reset and enable FIFO = 0x44
    i2c_buffer->write_buffer[0] = MPU_FIFO_CTRL_REG;
    i2c_buffer->write_buffer[1] = 0x44;
    mpu_transmit(i2c_buffer, 2);
}

/**
 * @brief Read FIFO count register and return pointer to the value
 *
 * Functions is usefull when we need to check if the FIFO buffer is filled with enough bytes
 *
 * @param i2c_buffer: struct with read_buffer, write_buffer
 * @return uint16_t* pointer to the FIFO count value
 */
uint16_t *mpu_fifo_count(i2c_buffer_type *i2c_buffer)
{
    static uint16_t fifo_count;
    i2c_buffer->write_buffer[0] = MPU_FIFO_COUNT_H_REG;
    mpu_transmit_receive(i2c_buffer, 1, 2);

    fifo_count = (i2c_buffer->read_buffer[0] << 8) | i2c_buffer->read_buffer[1];
    return &fifo_count;
}

/**
 * @brief Read accel and gyro data from the FIFO buffer and extract it mpu_data.accel_gyro_raw
 *
 * If FIFO count is at least 12 bytes long, read the FIFO buffer into i2c_buffer.read_buffer
 *
 * @param fifo_bytes: number of bytes to read from the FIFO buffer
 * @param reset_fifo: reset the FIFO buffer after reading
 *
 * @return void
 */
bool mpu_fifo_read_extract(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data)
{
    // First read FIFO count to make sure it is at least 12 bytes long (accel and gyro high and low registers)
    if (I2C_READ_BUFF_SIZE < 12)
    {
        ESP_LOGI(TAG, "The allocated i2c_buffer.read_buffer size is smaller than 12");
        return false;
    }
    uint16_t *fifo_count = mpu_fifo_count(i2c_buffer);
    if (*fifo_count >= 12)
    {
        i2c_buffer->write_buffer[0] = MPU_FIFO_DATA_REG;
        mpu_transmit_receive(i2c_buffer, 1, 12);
    }

    mpu_fifo_extract_buffer(i2c_buffer, mpu_data);
    return true;
}

/**
 * @brief Extract gyro and accel data
 *
 * After reading FIFO buffer, the data has to be extracted from buffer
 * and converted to uint16_t values. Subsequent pairs of registers are
 * combined to form the final value.
 */
void mpu_fifo_extract_buffer(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data)
{
    for (int i = 0; i < 12; i += 2)
    {
        mpu_data->accel_gyro_raw[i] = (i2c_buffer->read_buffer[i] << 8) | i2c_buffer->read_buffer[i + 1];
    }
}

/**
 * @brief Scale mpu accel and gyro reading to preset full scale range
 *
 * @param mpu_data
 */
void mpu_readings_scale(mpu_data_type *mpu_data)
{
    mpu_data->accel_gyro_g[0] = (double)mpu_data->accel_gyro_raw[0] / MPU_ACCEL_FS;
    mpu_data->accel_gyro_g[1] = (double)mpu_data->accel_gyro_raw[1] / MPU_ACCEL_FS;
    mpu_data->accel_gyro_g[2] = (double)mpu_data->accel_gyro_raw[2] / MPU_ACCEL_FS;
    mpu_data->accel_gyro_g[3] = (double)mpu_data->accel_gyro_raw[3] / MPU_GYRO_FS;
    mpu_data->accel_gyro_g[4] = (double)mpu_data->accel_gyro_raw[4] / MPU_GYRO_FS;
    mpu_data->accel_gyro_g[5] = (double)mpu_data->accel_gyro_raw[5] / MPU_GYRO_FS;
}

void mpu_readings_avg_err(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data, uint16_t repetitions)
{
    // Calculate average deviation
    uint64_t error_sum[6] = {0};
    uint16_t i = 0;
    while (i < repetitions)
    {
        // Check if the FIFO buffer is filled with at least 12 bytes
        if (mpu_fifo_read_extract(i2c_buffer, mpu_data))
        {
            error_sum[0] += mpu_data->accel_gyro_raw[0];
            error_sum[1] += mpu_data->accel_gyro_raw[1];
            error_sum[2] += mpu_data->accel_gyro_raw[2];
            error_sum[3] += mpu_data->accel_gyro_raw[3];
            error_sum[4] += mpu_data->accel_gyro_raw[4];
            error_sum[5] += mpu_data->accel_gyro_raw[5];
            ++i;
        }
    }
    mpu_data->accel_gyro_avg_deviation[0] = error_sum[0] / repetitions;
    mpu_data->accel_gyro_avg_deviation[1] = error_sum[1] / repetitions;
    mpu_data->accel_gyro_avg_deviation[2] = error_sum[2] / repetitions;
    mpu_data->accel_gyro_avg_deviation[3] = error_sum[3] / repetitions;
    mpu_data->accel_gyro_avg_deviation[4] = error_sum[4] / repetitions;
    mpu_data->accel_gyro_avg_deviation[5] = error_sum[5] / repetitions;
}