#include "mpu6050.h"

/**
 * @brief Initial MPU6050 setup.
 *
 * Set up the MPU6050 registers for the first time. MPU is set to wake up, filter freq is set to 5Hz,
 * accelerometer to +-8g full scale range and gyroscope to +-1000 deg/s full scale range.
 * The FIFO buffer is set to store accelerometer and gyroscope data.
 *
 * @param i2c_buffer_t: struct with write_buffer, read_buffer
 * @return 0 OK
 * @return -1 failed to set power settings
 * @return -2 failed to set filter frequency
 * @return -3 failed to set accel register
 * @return -4 failed to set gyro register
 * @return -5 failed to dissable fifo
 */
int mpu_initial_setup(i2cBufferType *i2c_buffer_t)
{
    // const char *TAG = "mpu_initial_setup";

    if (mpu_wake_up_senzor(i2c_buffer_t) != 0)
        return -1;

    if (mpu_set_filter_freq(i2c_buffer_t) != 0)
        return -2;

    if (mpu_set_accel_fullscale(i2c_buffer_t) != 0)
        return -3;

    if (mpu_set_gyro_fullscale(i2c_buffer_t) != 0)
        return -4;

    if (mpu_disable_fifo(i2c_buffer_t) != 0)
        return -5;

    if (mpu_wake_up_senzor(i2c_buffer_t) != 0)
        return -6;

    // // Wake up the MPU6050
    // i2c_buffer_t->write_buffer[0] = MPU_PWR_REG; // Power settings register
    // i2c_buffer_t->write_buffer[1] = 0x00;        // wake up signal
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -1;
    // }

    // // Set filter freq
    // i2c_buffer_t->write_buffer[0] = MPU_FILTER_FREQ_REG;  // Filter freq register
    // i2c_buffer_t->write_buffer[1] = MPU_FILTER_FREQ_MASK; // Filter freq
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -2;
    // }

    // // Set accelerometer full scale range
    // i2c_buffer_t->write_buffer[0] = MPU_ACCEL_CFG_REG; // Accel register
    // i2c_buffer_t->write_buffer[1] = MPU_ACCEL_FS_MASK; //  Accel register setting
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -3;
    // }
    // // Set gyroscope full scale range
    // i2c_buffer_t->write_buffer[0] = MPU_GYRO_CFG_REG; // Gyro register
    // i2c_buffer_t->write_buffer[1] = MPU_GYRO_FS_MASK; // Gyro register setting
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -4;
    // }

    // // Dissable FIFO buffer
    // i2c_buffer_t->write_buffer[0] = MPU_USER_CTRL_REG;
    // i2c_buffer_t->write_buffer[1] = MPU_FIFO_DISSABLE;
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -5;
    // }

    // // Set what data is stored in the FIFO buffer
    // i2c_buffer_t->write_buffer[0] = MPU_FIFO_EN_REG;  // Fifo enable register
    // i2c_buffer_t->write_buffer[1] = MPU_FIFO_EN_MASK; // Fifo enable gyro and accel data setting
    // if (mpu_transmit(i2c_buffer_t, 2) != 0)
    // {
    //     return -6;
    // }
    // ESP_LOGI(TAG, "Configured MPU registers checkup:");
    // // // Check the integrity of the configured registers
    // i2c_buffer_t->write_buffer[0] = MPU_PWR_REG;
    // mpu_transmit_receive(i2c_buffer_t, 1, 1); // Should be 0x00h (0d)
    // ESP_LOGI(TAG, "Power reg: %i", i2c_buffer_t->read_buffer[0]);

    // i2c_buffer_t->write_buffer[0] = MPU_FILTER_FREQ_REG;
    // mpu_transmit_receive(i2c_buffer_t, 1, 1); // Should equal to MPU_FILTER_FREQ_MASK
    // ESP_LOGI(TAG, "DLPF reg: %i", i2c_buffer_t->read_buffer[0]);

    // i2c_buffer_t->write_buffer[0] = MPU_ACCEL_CFG_REG;
    // mpu_transmit_receive(i2c_buffer_t, 1, 1); // Should equal to MPU_ACCEL_FS_MASK
    // ESP_LOGI(TAG, "Accel reg: %i", i2c_buffer_t->read_buffer[0]);

    // i2c_buffer_t->write_buffer[0] = MPU_GYRO_CFG_REG;
    // mpu_transmit_receive(i2c_buffer_t, 1, 1); // Should equal to MPU_GYRO_FS_MASK
    // ESP_LOGI(TAG, "Gyro reg: %i", i2c_buffer_t->read_buffer[0]);

    // i2c_buffer_t->write_buffer[0] = MPU_FIFO_EN_REG;
    // mpu_transmit_receive(i2c_buffer_t, 1, 1); // Should equal to MPU_FIFO_EN_MASK
    // ESP_LOGI(TAG, "FIFO reg: %i", i2c_buffer_t->read_buffer[0]);
    
    return 0;
}

int mpu_wake_up_senzor(i2cBufferType *i2c_buffer_t)
{
    i2c_buffer_t->write_buffer[0] = MPU_PWR_REG; // Power settings register
    i2c_buffer_t->write_buffer[1] = 0x00;        // wake up signal
    if (mpu_transmit(i2c_buffer_t, 2) != 0)
    {
        return -1;
    }
    return 0;
}

int mpu_set_filter_freq(i2cBufferType *i2c_buffer_t)
{
    i2c_buffer_t->write_buffer[0] = MPU_FILTER_FREQ_REG;  // Filter freq register
    i2c_buffer_t->write_buffer[1] = MPU_FILTER_FREQ_MASK; // Filter freq
    if (mpu_transmit(i2c_buffer_t, 2) != 0)
    {
        return -1;
    }
    return 0;
}

int mpu_set_accel_fullscale(i2cBufferType *i2c_buffer_t)
{
    // Set accelerometer full scale range
    i2c_buffer_t->write_buffer[0] = MPU_ACCEL_CFG_REG; // Accel register
    i2c_buffer_t->write_buffer[1] = MPU_ACCEL_FS_MASK; //  Accel register setting
    if (mpu_transmit(i2c_buffer_t, 2) != 0)
    {
        return -1;
    }
    return 0;
}

int mpu_set_gyro_fullscale(i2cBufferType *i2c_buffer_t)
{
    // Set gyroscope full scale range
    i2c_buffer_t->write_buffer[0] = MPU_GYRO_CFG_REG; // Gyro register
    i2c_buffer_t->write_buffer[1] = MPU_GYRO_FS_MASK; // Gyro register setting
    if (mpu_transmit(i2c_buffer_t, 2) != 0)
    {
        return -1;
    }
    return 0;
}

int mpu_disable_fifo(i2cBufferType *i2c_buffer_t)
{
    // Dissable FIFO buffer
    i2c_buffer_t->write_buffer[0] = MPU_USER_CTRL_REG;
    i2c_buffer_t->write_buffer[1] = MPU_FIFO_DISSABLE;
    if (mpu_transmit(i2c_buffer_t, 2) != 0)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Transmit MPU register data and read it's value.
 *
 * MCU_write -> REG_ADDR -> MCU_read <- REG_VALUE(s)
 *
 * Before transmision, you have to first setup the write_buffer
 * i2c_buffer_t.write_buffer[write_buf_size]
 * [0] - register address
 *
 * To read multiple values, set the read_buf_size to greater than 1.
 * Reading multiple values usually means reading subsequent registers.
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param write_buf_size: i2c_buffer_t.wirte_buffer size (how many values to transmit)
 * @param read_buf_size: i2c_buffer_t.read_buffer size (how many values to receive)
 *
 * @return 0 OK
 * @return -1 i2c write buffer size is too small
 * @return -2 i2c read buffer is too small
 * @return -3 i2c master failed to transmit receive data
 */
int mpu_transmit_receive(i2cBufferType *i2c_buffer_t, uint8_t write_buf_size, uint8_t read_buf_size)
{
    const char *TAG = "MPU TRANSMIT RECEIVE";

    int error_code = 0;
    if (sizeof(i2c_buffer_t->write_buffer) < write_buf_size)
    {
        ESP_LOGE(TAG, "The allocated i2c_buffer_t.write_buffer size is smaller than %d", write_buf_size);
        return -1;
    }
    if (sizeof(i2c_buffer_t->read_buffer) < read_buf_size)
    {
        ESP_LOGE(TAG, "The allocated i2c_buffer_t.read_buffer size is smaller than %d", read_buf_size);
        return -2;
    }
    if ((error_code = i2c_master_transmit_receive(i2c_master_dev_handle, i2c_buffer_t->write_buffer, write_buf_size, i2c_buffer_t->read_buffer, read_buf_size, I2C_TIMEOUT_MS)) != 0)
    {
        ESP_LOGE(TAG, "Falled to i2c master transmit receive, error %d", error_code);
        return -3;
    }
    return 0;
}

/**
 * @brief Write data to specific MPU6050 register
 *
 * MCU_write -> REG_ADDR -> REG_VALUE
 *
 * Before transmision, you have to first setup the write_buffer
 * i2c_buffer_t->write_buffer[I2C_WRITE_BUFF_SIZE]
 * [0] - register address
 * [1] - data to write
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param write_buf_size: i2c_buffer_t.wirte_buffer size (how many values to transmit)
 *
 * @return 0 OK
 * @return -1 Buffer size smaller than expected
 * @return -2 i2c master trasmit failed
 */
int mpu_transmit(i2cBufferType *i2c_buffer_t, uint8_t write_buf_size)
{
    const char *TAG = "MPU TRANSMIT";
    int error_code = 0;
    if (sizeof(i2c_buffer_t->write_buffer) < write_buf_size)
    {
        ESP_LOGE(TAG, "The allocated i2c_buffer_t.write_buffer size is smaller than %d", write_buf_size);
        return -1;
    }
    if ((error_code = i2c_master_transmit(i2c_master_dev_handle, i2c_buffer_t->write_buffer, write_buf_size, I2C_TIMEOUT_MS)) != ESP_OK)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief Enable the FIFO buffer of the MPU6050 sensor
 *
 * FIFO buffer automatically reads the data from the MPU6050 sensor and stores it in the buffer.
 *
 * @param i2c_buffer_t: struct with write_buffer, read_buffer
 * @return void
 */
void mpu_fifo_enable(i2cBufferType *i2c_buffer_t)
{
    i2c_buffer_t->write_buffer[0] = MPU_USER_CTRL_REG;
    i2c_buffer_t->write_buffer[1] = 0x40; // Enable FIFO
    mpu_transmit(i2c_buffer_t, 2);
}

/**
 * @brief Reset the FIFO buffer of the MPU6050 sensor
 *
 * @param i2c_buffer_t: struct with write_buffer, read_buffer
 * @return void
 */
void mpu_fifo_reset(i2cBufferType *i2c_buffer_t)
{
    // Fifo reset bit = 0x04 and fifo enable bit = 0x40
    // Therefore reset and enable FIFO = 0x44
    i2c_buffer_t->write_buffer[0] = MPU_USER_CTRL_REG;
    i2c_buffer_t->write_buffer[1] = MPU_FIFO_RESET_MASK;
    mpu_transmit(i2c_buffer_t, 2);
}

/**
 * @brief Read FIFO count register and return the value
 *
 * Functions is usefull when we need to check if the FIFO buffer is filled with enough bytes
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @return fifo_count: uint16_t value
 */
uint16_t mpu_fifo_count(i2cBufferType *i2c_buffer_t)
{
    static uint16_t fifo_count;
    // Read MSB
    i2c_buffer_t->write_buffer[0] = MPU_FIFO_COUNT_H_REG;
    mpu_transmit_receive(i2c_buffer_t, 1, 1);
    fifo_count = (i2c_buffer_t->read_buffer[0] << 8);
    // Read LSB
    i2c_buffer_t->write_buffer[0] = MPU_FIFO_COUNT_L_REG;
    mpu_transmit_receive(i2c_buffer_t, 1, 1);
    fifo_count |= i2c_buffer_t->read_buffer[0];

    return fifo_count;
}

/**
 * @brief Check if FIFO buffer has overflowed
 *
 * @param i2c_buffer_t struct with write_buffer, read_buffer
 * @return true
 * @return false
 */
bool mpu_fifo_overflow_check(i2cBufferType *i2c_buffer_t)
{
    i2c_buffer_t->write_buffer[0] = MPU_FIFO_OVERFLOW;
    mpu_transmit_receive(i2c_buffer_t, 1, 1);
    if (i2c_buffer_t->read_buffer[0] & 0x10) // check if fifo overflow bit is 1
    {
        return true;
    }
    return false;
}

/**
 * @brief Read accel and gyro data from the FIFO buffer and extract it mpu_data_t.accel_gyro_raw
 *
 * If FIFO count is at least 12 bytes long, read the FIFO buffer into i2c_buffer_t.read_buffer
 *
 * @param fifo_bytes: number of bytes to read from the FIFO buffer
 * @param reset_fifo: reset the FIFO buffer after reading
 *
 * @return true if FIFO was read and extracted successfully
 */
bool mpu_fifo_read_extract(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t)
{
    const char *TAG = "MPU FIFO READ EXTRACT";
    // First read FIFO count to make sure it is at least 12 bytes long (accel and gyro high and low registers)
    if (I2C_READ_BUFF_SIZE < 12)
    {
        ESP_LOGW(TAG, "The allocated i2c_buffer_t.read_buffer size is smaller than 12");
        return false;
    }
    mpu_fifo_reset(i2c_buffer_t);
    for (int i = 0; i < 10; ++i)
    {
        uint16_t fifo_count = mpu_fifo_count(i2c_buffer_t);
        // ESP_LOGI(TAG, "(%d) FIFO count: %d", i, fifo_count);
        if (fifo_count % 12 != 0)
            mpu_fifo_reset(i2c_buffer_t);
        if (fifo_count >= 12 && fifo_count % 12 == 0)
        {
            ESP_LOGD(TAG, "(%d) FIFO %% 12 == 0 (%d -> %d)", i, fifo_count, fifo_count % 12);
            i2c_buffer_t->write_buffer[0] = MPU_FIFO_DATA_REG;
            mpu_transmit_receive(i2c_buffer_t, 1, 12);
            mpu_fifo_extract_buffer(i2c_buffer_t, mpu_data_t);
            return true;
        }
    }
    return false;
}

/**
 * @brief Read and extract accel and gyro data directly from sencor registers (not from fifo)
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param mpu_data_t: struct with accel_gyro_raw
 * @return true
 * @return false
 */
bool mpu_data_read_extract(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t)
{
    const char *TAG = "MPU DATA READ EXTRACT";
    if (I2C_READ_BUFF_SIZE < 14)
    {
        ESP_LOGW(TAG, "The allocated i2c_buffer_t.read_buffer size is smaller than 14");
        return false;
    }
    i2c_buffer_t->write_buffer[0] = MPU_ACCEL_X_H_REG;
    mpu_transmit_receive(i2c_buffer_t, 1, 14);
    mpu_data_t->accel_gyro_raw[0] = (i2c_buffer_t->read_buffer[0] << 8) | i2c_buffer_t->read_buffer[1];
    mpu_data_t->accel_gyro_raw[1] = (i2c_buffer_t->read_buffer[2] << 8) | i2c_buffer_t->read_buffer[3];
    mpu_data_t->accel_gyro_raw[2] = (i2c_buffer_t->read_buffer[4] << 8) | i2c_buffer_t->read_buffer[5];
    mpu_data_t->accel_gyro_raw[3] = (i2c_buffer_t->read_buffer[8] << 8) | i2c_buffer_t->read_buffer[9];
    mpu_data_t->accel_gyro_raw[4] = (i2c_buffer_t->read_buffer[10] << 8) | i2c_buffer_t->read_buffer[11];
    mpu_data_t->accel_gyro_raw[5] = (i2c_buffer_t->read_buffer[12] << 8) | i2c_buffer_t->read_buffer[13];
    return true;
}

/**
 * @brief Read and extract accel data from the sensor registers (not from fifo)
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param mpu_data_t: struct with accel_gyro_raw
 * @return true
 * @return false
 */
bool mpu_data_read_extract_accel(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t)
{
    const char *TAG = "MPU DATA READ EXTRACT ACCEL";

    if (I2C_READ_BUFF_SIZE < 6)
    {
        ESP_LOGW(TAG, "The allocated i2c_buffer_t.read_buffer size is smaller than 6");
        return false;
    }
    i2c_buffer_t->write_buffer[0] = MPU_ACCEL_X_H_REG;
    mpu_transmit_receive(i2c_buffer_t, 1, 6);
    mpu_data_t->accel_gyro_raw[0] = (i2c_buffer_t->read_buffer[0] << 8) | i2c_buffer_t->read_buffer[1];
    mpu_data_t->accel_gyro_raw[1] = (i2c_buffer_t->read_buffer[2] << 8) | i2c_buffer_t->read_buffer[3];
    mpu_data_t->accel_gyro_raw[2] = (i2c_buffer_t->read_buffer[4] << 8) | i2c_buffer_t->read_buffer[5];
    return true;
}

/**
 * @brief Used for reading and averaging out the raw reading into a mpu_data_t.avg_err array
 *
 * The function simply adds the mpu_data_t.accel_gyro_raw array to the mpu_data_t.avg_err array.
 * If average_out is set to true, the result is divided by 2.
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param mpu_data_t: struct with accel_gyro_raw, avg_err
 * @param average_out: true if you want to average out the values (a_0+b_0)/2
 * @return true if successful
 */
bool mpu_data_sum_error(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t, bool average_out)
{
    const char *TAG = "MPU DATA SUM ERR";
    if (sizeof(mpu_data_t->accel_gyro_raw) < 6 || sizeof(mpu_data_t->avg_err) < 6)
    {
        ESP_LOGW(TAG, "The allocated mpu_data_t.accel_gyro_raw or mpu_data_t.avg_err size is smaller than 6");
        return false;
    }
    if (!average_out)
        mpu_data_reset(mpu_data_t);

    for (int i = 0; i < 6; ++i)
    {
        mpu_data_t->avg_err[i] += mpu_data_t->accel_gyro_raw[i];
        if (average_out)
            mpu_data_t->avg_err[i] /= 2;
    }
    return true;
}

/**
 * @brief Calibrate the MPU6050 sensor using direct register access
 *
 * The function reads the accel and gyro data straight from the accel and gyro registers and averages out the errors.
 * Error values are store into the mpu_data_t.avg_err array.
 *
 * @param i2c_buffer_t: struct with read_buffer, write_buffer
 * @param mpu_data_t: struct with accel_gyro_raw, avg_err
 * @param cycles: number of cycles to average out the errors (cycles * 100 readings)
 * @return true
 * @return false
 */
bool mpu_data_calibrate(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t, uint8_t cycles)
{
    const char *TAG = "MPU DATA CALIBRATE";
    // First data reading
    if (!mpu_data_read_extract(i2c_buffer_t, mpu_data_t))
    {
        ESP_LOGE(TAG, "Failed to read FIFO buffer. Aborting calibration");
        return false;
    }
    if (!mpu_data_sum_error(i2c_buffer_t, mpu_data_t, false))
    {
        ESP_LOGE(TAG, "Failed to sum the errors. Aborting calibration");
        return false;
    }

    // Start averaging out the errors
    for (int cycle = 0; cycle < cycles; ++cycle)
    {
        for (int reading = 0; reading < 100; ++reading)
        {
            mpu_data_read_extract(i2c_buffer_t, mpu_data_t);
            mpu_data_sum_error(i2c_buffer_t, mpu_data_t, true);
        }
        if (cycle % 5 == 0)
        {
            ESP_LOGD(TAG, "Calibration cycle %d (%d readings)", cycle, cycle * 100);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    ESP_LOGD(TAG, "Calibration finished with %d cycles (%d readings)", cycles, cycles * 100);
    return true;
}

/**
 * @brief Extract gyro and accel data
 *
 * After reading FIFO buffer, the data has to be extracted from buffer
 * and converted to uint16_t values. Subsequent pairs of registers are
 * combined to form the final value.
 */
void mpu_fifo_extract_buffer(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t)
{
    int msb = 0;
    for (int i = 0; i < 6; ++i)
    {
        msb = i * 2;
        mpu_data_t->accel_gyro_raw[i] = (i2c_buffer_t->read_buffer[msb] << 8) | i2c_buffer_t->read_buffer[msb + 1];
    }
}

/**
 * @brief Reset mpu_data_t buffers
 *
 * @param mpu_data_t
 */
void mpu_data_reset(mpuDataType *mpu_data_t)
{
    for (int i = 0; i < 6; ++i)
    {
        mpu_data_t->accel_gyro_raw[i] = 0;
        mpu_data_t->avg_err[i] = 0;
        mpu_data_t->accel_gyro_g[i] = 0;
    }
}

/**
 * @brief Scale mpu_data_t.accel_gyro_g to full scale
 *
 * Divide accel_gyro_g array with accel full scale and gyro full scale values
 *
 * @param mpu_data_t
 * @param accel_gyro_raw
 * @return void
 */
void mpu_data_to_fs(mpuDataType *mpu_data_t, bool accel_only)
{
    mpu_data_t->accel_gyro_g[0] = (float)mpu_data_t->accel_gyro_raw[0] / MPU_ACCEL_FS; //  - mpu_data_t->avg_err[0]
    mpu_data_t->accel_gyro_g[1] = (float)mpu_data_t->accel_gyro_raw[1] / MPU_ACCEL_FS; //  - mpu_data_t->avg_err[1]
    mpu_data_t->accel_gyro_g[2] = (float)mpu_data_t->accel_gyro_raw[2] / MPU_ACCEL_FS; //  - mpu_data_t->avg_err[2]
    if (accel_only)
        return;
    mpu_data_t->accel_gyro_g[3] = (float)mpu_data_t->accel_gyro_raw[3] / MPU_GYRO_FS; //  - mpu_data_t->avg_err[3]
    mpu_data_t->accel_gyro_g[4] = (float)mpu_data_t->accel_gyro_raw[4] / MPU_GYRO_FS; //  - mpu_data_t->avg_err[4]
    mpu_data_t->accel_gyro_g[5] = (float)mpu_data_t->accel_gyro_raw[5] / MPU_GYRO_FS; //  - mpu_data_t->avg_err[5]
}

/**
 * @brief Calculate average error of the MPU6050 sensor using FIFO buffer
 *
 * @param i2c_buffer_t struct with write_buffer, read_buffer
 * @param mpu_data_t struct with avg_errors
 * @param cycles number of cycles to average out (cycles * 100 readings)
 * @param substract_err substract the average error from the raw data during calibration
 */
bool mpu_calibrate(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t, uint8_t cycles, bool substract_err)
{
    const char *TAG = "MPU CALIBRATE";
    float avg_errors[6] = {0};
    for (int i = 0; i < 10; ++i)
    {
        if (i == 10)
        {
            ESP_LOGE(TAG, "Failed to read FIFO buffer 10 times. Aborting calibration");
            return false;
        }
        if (mpu_fifo_read_to_array(i2c_buffer_t, mpu_data_t, avg_errors, 6, substract_err, false) != NULL)
            break;
        vTaskDelay(10 / portTICK_PERIOD_MS); // wait 5 ms for watchdog reasons
    }

    // Number of subsequent failed readings
    uint8_t failed_readings = 0;

    // cycles * 100 readings are used for calibration
    for (int cycle = 1; cycle <= cycles; ++cycle)
    {
        if (cycle % 5 == 0)
        {
            vTaskDelay(20 / portTICK_PERIOD_MS); // wait for watchdog reasons
            ESP_LOGD(TAG, "Calibration cycle %d (%d readings)", cycle, cycle * 100);
        }
        for (int readings = 0; readings < 100; ++readings)
        {
            // Read extract FIFO and check if it was successful
            if (mpu_fifo_read_to_array(i2c_buffer_t, mpu_data_t, avg_errors, 6, substract_err, true) != NULL)
            {
                failed_readings = 0; // Reset failed count on a successful read
            }
            else
            {
                ++failed_readings;
                if (failed_readings >= 20)
                {
                    ESP_LOGE(TAG, "Aborting the calibration process. Failed to read %d subsequent readings!", failed_readings);
                    return false;
                }
            }
        }
    }

    // update mpu_data_t avg_errors array
    for (int i = 0; i < 6; ++i)
    {
        if (substract_err)
            mpu_data_t->avg_err[i] += avg_errors[i];
        else
            mpu_data_t->avg_err[i] = avg_errors[i];
    }

    ESP_LOGD(TAG, "Calibration finished with %d cycles (%d readings)", cycles, cycles * 100);
    return true;
}

/**
 * @brief Read and extract FIFO buffer, then add the values to desired readings_array
 *
 * The average error is substracted from the raw data, if substract_err is set to true.
 * If the average_out is set to true, the readings_array values averaged out (a_0+b_0)/2.
 *
 * @param i2c_buffer_t i2c buffer struct with read_buffer, write_buffer
 * @param mpu_data_t mpu data struct with accel_gyro_raw, avg_errors, accel_gyro_g
 * @param readings_array array to which FIFO data will be added
 * @param array_size size of the readings_array (must be at least 6)
 * @param substract_err substract the average error from the raw data
 * @param average_out average the readings_array values
 * @return float*
 */
float *mpu_fifo_read_to_array(i2cBufferType *i2c_buffer_t, mpuDataType *mpu_data_t, float *readings_array, uint8_t array_size, bool subtract_err, bool average_out)
{
    const char *TAG = "MPU FIFO READ TO ARR";
    if (array_size < 6)
    {
        ESP_LOGW(TAG, "The readings_array size must be at least 6\n");
        return NULL;
    }

    if (i2c_buffer_t == NULL || mpu_data_t == NULL || readings_array == NULL)
    {
        ESP_LOGW(TAG, "Invalid pointer(s) passed to function\n");
        return NULL;
    }

    // Read and extract FIFO and check if it was successful
    if (mpu_fifo_read_extract(i2c_buffer_t, mpu_data_t))
    {
        // Subtract average error from the raw data (useful if the error was calculated before)
        if (subtract_err)
            mpu_data_substract_err(mpu_data_t, false);

        // Sum the raw data to the readings_array
        for (int i = 0; i < 6; ++i)
        {
            readings_array[i] += mpu_data_t->accel_gyro_raw[i];
            if (average_out)
                readings_array[i] /= 2;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read and extract data from FIFO");
        return NULL;
    }

    return readings_array;
}

/**
 * @brief Dvidide avg error array by divisor. Divisor is the number
 * of cycles, that got summed up.
 *
 * @param mpu_data_t struct with avg_errors
 * @param divisor number to divide the avg_errors array with (must be != 0)
 */
void mpu_avg_err_divide(mpuDataType *mpu_data_t, uint16_t divisor)
{
    const char *TAG = "MPU AVG ERR DIV";
    if (divisor == 0)
    {
        ESP_LOGW(TAG, "Divisor can't be 0");
        return;
    }

    for (int i = 0; i < 6; ++i)
    {
        mpu_data_t->avg_err[i] /= divisor;
    }
}

/**
 * @brief Substract the average error from the MPU6050 cycles
 *
 * @param mpu_data_t: mpu data struct with accel_gyro_raw, avg_errors, accel_gyro_g
 * @param accel_only: if true, only the accelerometer data is substracted
 */
void mpu_data_substract_err(mpuDataType *mpu_data_t, bool accel_only)
{
    static uint8_t i_limit = 6;
    if (accel_only)
        i_limit = 3;
    for (int i = 0; i < i_limit; ++i)
    {
        int32_t temp = (int32_t)mpu_data_t->accel_gyro_raw[i] - (int32_t)mpu_data_t->avg_err[i];
        // if temp is larger than max int16_t value, set it to max int16_t value or smaller than min int16_t value, set it to min int16_t value
        if (temp > INT16_MAX)
            mpu_data_t->accel_gyro_raw[i] = INT16_MAX;
        else if (temp < INT16_MIN)
            mpu_data_t->accel_gyro_raw[i] = INT16_MIN;
        else
            mpu_data_t->accel_gyro_raw[i] = (int16_t)temp;
    }
}

void mpu_avg_err_print(mpuDataType *mpu_data_t)
{
    for (int i = 0; i < 6; ++i)
    {
        printf("%.6f, ", mpu_data_t->avg_err[i]);
    }
    printf("\n");
}