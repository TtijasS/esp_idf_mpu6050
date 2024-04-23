#include <stdio.h>
#include "esp_log.h"
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include "constants.h"

/**
 * @brief Data structure for storing the MPU6050 sensor data
 *
 * The data structure stores the raw accelerometer and gyroscope data from the MPU6050 sensor.
 *
 */
typedef struct mpu_data_type
{
    // accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw
    int16_t accel_gyro_raw[6];

    // accel_x_er, accel_y_err, accel_z_err, gyro_x_err, gyro_y_err, gyro_z_err
    int32_t accel_gyro_avg_deviation[6];

    // accel_x_g, accel_y_g, accel_z_g, gyro_x_g, gyro_y_g, gyro_z_g
    double accel_gyro_g[6];
} mpu_data_type;

/**
 * @brief Data structure for storing the I2C read and write buffers
 */
typedef struct i2c_buffer_type
{
    // Read buffer array of size I2C_READ_BUFF_SIZE
    uint8_t read_buffer[I2C_READ_BUFF_SIZE]; // read buffer

    // Write buffer array of size I2C_WRITE_BUFF_SIZE
    uint8_t write_buffer[I2C_WRITE_BUFF_SIZE]; // write buffer
} i2c_buffer_type;

// Init functions
void print_binary(uint8_t *);
void init_i2c(void);
void mpu_initial_setup(i2c_buffer_type *);
void mpu_transmit_receive(i2c_buffer_type *, uint8_t, uint8_t);
void mpu_transmit(i2c_buffer_type *, uint8_t);

mpu_data_type mpu_data = {
    .accel_gyro_raw = {0},
    .accel_gyro_avg_deviation = {0},
    .accel_gyro_g = {0},
};

i2c_buffer_type i2c_buffer = {
    .read_buffer = {0},
    .write_buffer = {0},
};

/**
 * @brief I2C master bus configuration
 */
const i2c_master_bus_config_t master_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA_IO,
    .scl_io_num = I2C_SCL_IO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true};

/**
 * @brief I2C device configuration
 */
const i2c_device_config_t MPU6050_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU_ADR_REG,
    .scl_speed_hz = I2C_FREQ_HZ};

// Master device handle struct
i2c_master_bus_handle_t master_bus_handle;
// Slave device handle struct
i2c_master_dev_handle_t master_dev_handle;

void app_main(void)
{
    // Initialize the I2C bus
    init_i2c();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    // Enable the FIFO buffer (otherwise the data is not stored in the MPU FIFO)
    mpu_fifo_enable(&i2c_buffer);

    // Read MPU data registers
    // Clear the FIFO buffer to start fresh
    mpu_fifo_reset(&i2c_buffer);

    // Delay for at least 1 ms to allow the FIFO buffer to fill up
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait 5 ms

    // Read FIFO buffer
    mpu_fifo_read_extract(&i2c_buffer, &mpu_data);

    // printf("%.1f, %.1f, %.1f", accel_x_g, accel_y_g, accel_z_g);
    // printf("%.2f; %.2f; %.2f\n", accel_x_g, accel_y_g, accel_z_g);
    // printf("%.2f; %.2f; %.2f\n", mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g);

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}

/**
 * @brief Extract gyro, temperature and accel data
 *
 * @param n: number to be printed in binary
 * @return void
 * */
void print_binary(uint8_t *n)
{
    printf("binary: ");
    for (int i = 7; i >= 0; --i)
    {
        uint8_t mask = 1 << i;
        printf("%u", (*n & mask) ? 1 : 0);
    }
    printf("\n");
}

/**
 * @brief Initialize the I2C bus
 *
 * Add new master bus and MPU6050 as a slave device
 * Slave address of the MPU6050 sensor is 0x68
 *
 * @param void
 * @return void
 */
void init_i2c()
{
    // Initialize the I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_bus_config, &master_bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus_handle, &MPU6050_device_config, &master_dev_handle));
    // Probe the slave device
    ESP_ERROR_CHECK(i2c_master_probe(master_bus_handle, MPU_ADR_REG, I2C_TIMEOUT_MS));
    ESP_LOGI(TAG, "I2C initialized successfully");
}

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
 * @param write_buffer: buffer with the register address at [0] and data at [1]
 * @param write_buf_size: Write buffer size (how many values to transmit)
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
    static uint16_t *fifo_count;
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
    if (&fifo_count >= 12)
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

void mpu_readings_avg_err(mpu_data_type *mpu_data, uint16_t repetitions)
{
    // Calculate average deviation
    uint64_t error_sum[6] = {0};
    uint16_t i = 0;
    while (i < repetitions)
    {
        // Check if the FIFO buffer is filled with at least 12 bytes
        if (mpu_fifo_read_extract(&i2c_buffer, &mpu_data))
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
