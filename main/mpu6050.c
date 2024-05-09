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
    i2c_buffer->write_buffer[0] = MPU_PWR_REG; // Power settings register
    i2c_buffer->write_buffer[1] = 0x00;        // wake up signal
    mpu_transmit(i2c_buffer, 2);

    // Set filter freq
    i2c_buffer->write_buffer[0] = MPU_FILTER_FREQ_REG;  // Filter freq register
    i2c_buffer->write_buffer[1] = MPU_FILTER_FREQ_MASK; // Filter freq
    mpu_transmit(i2c_buffer, 2);

    // Set accelerometer full scale range
    i2c_buffer->write_buffer[0] = MPU_ACCEL_CFG_REG; // Accel register
    i2c_buffer->write_buffer[1] = MPU_ACCEL_FS_MASK; //  Accel register setting
    mpu_transmit(i2c_buffer, 2);

    // Set gyroscope full scale range
    i2c_buffer->write_buffer[0] = MPU_GYRO_CFG_REG; // Gyro register
    i2c_buffer->write_buffer[1] = MPU_GYRO_FS_MASK; // Gyro register setting
    mpu_transmit(i2c_buffer, 2);

    // Set what data is stored in the FIFO buffer
    i2c_buffer->write_buffer[0] = MPU_FIFO_EN_REG;     // Fifo enable register
    i2c_buffer->write_buffer[1] = MPU_FIFO_EN_MASK; // Fifo enable gyro and accel data setting
    mpu_transmit(i2c_buffer, 2);

    // // Check the integrity of the configured registers
    // i2c_buffer->write_buffer[0] = MPU_PWR_REG;
    // mpu_transmit_receive(i2c_buffer, 1, 1); // Should be 0x00h (0d)

    // i2c_buffer->write_buffer[0] = MPU_FILTER_FREQ_REG;
    // mpu_transmit_receive(i2c_buffer, 1, 1); // Should equal to MPU_FILTER_FREQ_MASK

    // i2c_buffer->write_buffer[0] = MPU_ACCEL_CFG_REG;
    // mpu_transmit_receive(i2c_buffer, 1, 1); // Should equal to MPU_ACCEL_FS_MASK

    // i2c_buffer->write_buffer[0] = MPU_GYRO_CFG_REG;
    // mpu_transmit_receive(i2c_buffer, 1, 1); // Should equal to MPU_GYRO_FS_MASK

    // i2c_buffer->write_buffer[0] = MPU_FIFO_EN_REG;
    // mpu_transmit_receive(i2c_buffer, 1, 1); // Should equal to MPU_FIFO_EN_MASK
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
    i2c_buffer->write_buffer[0] = MPU_USER_CTRL_REG;
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
    i2c_buffer->write_buffer[0] = MPU_USER_CTRL_REG;
    i2c_buffer->write_buffer[1] = MPU_FIFO_RESET_MASK;
    mpu_transmit(i2c_buffer, 2);
}

/**
 * @brief Read FIFO count register and return pointer to the value
 *
 * Functions is usefull when we need to check if the FIFO buffer is filled with enough bytes
 *
 * @param i2c_buffer: struct with read_buffer, write_buffer
 * @return &fifo_count: pointer to the FIFO count value
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
 * @return true if FIFO was read and extracted successfully
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
    int j = 0;
    for (int i = 0; i < 6; ++i)
    {
        j = i * 2;
        mpu_data->accel_gyro_raw[i] = (i2c_buffer->read_buffer[j] << 8) | i2c_buffer->read_buffer[j + 1];
    }
}

/**
 * @brief Reset mpu_data buffers
 *
 * @param mpu_data
 */
void mpu_data_reset(mpu_data_type *mpu_data)
{
    for (int i = 0; i < 6; ++i)
    {
        mpu_data->accel_gyro_raw[i] = 0;
        mpu_data->avg_err[i] = 0;
        mpu_data->accel_gyro_g[i] = 0;
    }
}

/**
 * @brief Scale mpu_data.accel_gyro_g to full scale
 *
 * Divide accel_gyro_g array with accel full scale and gyro full scale values
 *
 * @param mpu_data
 * @param accel_gyro_raw
 * @return void
 */
void mpu_data_to_fs(mpu_data_type *mpu_data)
{
    mpu_data->accel_gyro_g[0] = (double)mpu_data->accel_gyro_raw[0] / MPU_ACCEL_FS; //  - mpu_data->avg_err[0]
    mpu_data->accel_gyro_g[1] = (double)mpu_data->accel_gyro_raw[1] / MPU_ACCEL_FS; //  - mpu_data->avg_err[1]
    mpu_data->accel_gyro_g[2] = (double)mpu_data->accel_gyro_raw[2] / MPU_ACCEL_FS; //  - mpu_data->avg_err[2]
    mpu_data->accel_gyro_g[3] = (double)mpu_data->accel_gyro_raw[3] / MPU_GYRO_FS;  //  - mpu_data->avg_err[3]
    mpu_data->accel_gyro_g[4] = (double)mpu_data->accel_gyro_raw[4] / MPU_GYRO_FS;  //  - mpu_data->avg_err[4]
    mpu_data->accel_gyro_g[5] = (double)mpu_data->accel_gyro_raw[5] / MPU_GYRO_FS;  //  - mpu_data->avg_err[5]
}

/**
 * @brief Calculate average error of the MPU6050 sensor
 *
 * @param i2c_buffer struct with write_buffer, read_buffer
 * @param mpu_data struct with avg_errors
 * @param cycles number of cycles to average out (cycles * 100 readings)
 * @param substract_err substract the average error from the raw data during calibration
 */
bool mpu_calibrate(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data, uint8_t cycles, bool substract_err)
{
    double avg_errors[6] = {0};
    for (int i = 0; i < 10; ++i)
    {
        if (i == 10)
        {
            ESP_LOGI(TAG, "Failed to read FIFO buffer 10 times. Aborting calibration");
            return false;
        }
        if (mpu_fifo_read_to_array(i2c_buffer, mpu_data, avg_errors, 6, substract_err, false) != NULL)
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
            ESP_LOGI(TAG, "Calibration cycle %d (%d readings)", cycle, cycle * 100);
        }
        for (int readings = 0; readings < 100; ++readings)
        {
            // Read extract FIFO and check if it was successful
            if (mpu_fifo_read_to_array(i2c_buffer, mpu_data, avg_errors, 6, substract_err, true) != NULL)
            {
                failed_readings = 0; // Reset failed count on a successful read
            }
            else
            {
                ++failed_readings;
                if (failed_readings >= 20)
                {
                    ESP_LOGI(TAG, "Aborting the calibration process. Failed to read %d subsequent readings!", failed_readings);
                    return false;
                }
            }
        }
    }

    // update mpu_data avg_errors array
    for (int i = 0; i < 6; ++i)
    {
        if (substract_err)
            mpu_data->avg_err[i] += avg_errors[i];
        else
            mpu_data->avg_err[i] = avg_errors[i];
    }

    ESP_LOGI(TAG, "Calibration finished with %d cycles (%d readings)", cycles, cycles * 100);
    return true;
}

/**
 * @brief Read and extract FIFO buffer, then add the values to desired readings_array
 *
 * The average error is substracted from the raw data, if substract_err is set to true.
 * If the average_out is set to true, the readings_array values averaged out (a_0+b_0)/2.
 *
 * @param i2c_buffer i2c buffer struct with read_buffer, write_buffer
 * @param mpu_data mpu data struct with accel_gyro_raw, avg_errors, accel_gyro_g
 * @param readings_array array to which FIFO data will be added
 * @param array_size size of the readings_array (must be at least 6)
 * @param substract_err substract the average error from the raw data
 * @param average_out average the readings_array values
 * @return double*
 */
double *mpu_fifo_read_to_array(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data, double *readings_array, uint8_t array_size, bool substract_err, bool average_out)
{
    if (array_size < 6)
    {
        ESP_LOGI(TAG, "The readings_array size must be at least 6\n");
        return NULL;
    }

    if (i2c_buffer == NULL || mpu_data == NULL || readings_array == NULL)
    {
        ESP_LOGI(TAG, "Invalid pointer(s) passed to function\n");
        return NULL;
    }

    // Read and extract FIFO and check if it was successful
    if (mpu_fifo_read_extract(i2c_buffer, mpu_data))
    {
        // Subtract average error from the raw data (useful if the error was calculated before)
        if (substract_err)
            mpu_data_substract_err(mpu_data);

        // Sum the raw data to the readings_array
        for (int i = 0; i < 6; ++i)
        {
            readings_array[i] += mpu_data->accel_gyro_raw[i];
            if (average_out)
                readings_array[i] /= 2;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Failed to read and extract data from FIFO");
        return NULL;
    }

    return readings_array;
}

/**
 * @brief Dvidide avg error array by divisor. Divisor is the number
 * of cycles, that got summed up.
 *
 * @param mpu_data struct with avg_errors
 * @param divisor number to divide the avg_errors array with (must be != 0)
 */
void mpu_avg_err_divide(mpu_data_type *mpu_data, uint16_t divisor)
{
    if (divisor == 0)
    {
        ESP_LOGI(TAG, "Divisor can't be 0");
        return;
    }

    for (int i = 0; i < 6; ++i)
    {
        mpu_data->avg_err[i] /= divisor;
    }
}

/**
 * @brief Substract the average error from the MPU6050 cycles
 *
 * @param mpu_data mpu data struct with accel_gyro_raw, avg_errors, accel_gyro_g
 */
void mpu_data_substract_err(mpu_data_type *mpu_data)
{
    for (int i = 0; i < 6; ++i)
    {
        int32_t temp = (int32_t)mpu_data->accel_gyro_raw[i] - (int32_t)mpu_data->avg_err[i];
        // if temp is larger than max int16_t value, set it to max int16_t value or smaller than min int16_t value, set it to min int16_t value
        if (temp > INT16_MAX)
            mpu_data->accel_gyro_raw[i] = INT16_MAX;
        else if (temp < INT16_MIN)
            mpu_data->accel_gyro_raw[i] = INT16_MIN;
        else
            mpu_data->accel_gyro_raw[i] = (int16_t)temp;
    }
}