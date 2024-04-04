#include <stdio.h>
#include "esp_log.h"
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

static const char *TAG = "I2C Example";
#define I2C_SCL_IO CONFIG_I2C_MASTER_SCL // GPIO number used for I2C master clock
#define I2C_SDA_IO CONFIG_I2C_MASTER_SDA // GPIO number used for I2C master data
#define I2C_PORT_NUM I2C_NUM_0           // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
#define I2C_FREQ_HZ 100000               // I2C master clock frequency
#define I2C_TIMEOUT_MS 1000              // I2C timeout in milliseconds

// Setup I2C buffer sizes
#define I2C_READ_BUFF_SIZE 14 // I2C max read buffer size
#define I2C_WRITE_BUFF_SIZE 2 // i2c max write buffer size

// MPU config registers
#define MPU_ADR_REG 0x68            // Slave address of the MPU6050 sensor
#define MPU_WHO_AM_I_REG 0x75       // Register addresses of the "who am I"
#define MPU_FILTER_FREQ_REG 0x1A    // Register address for digital filter frequency range settings
#define MPU_MSTR_CTRL_REG 0x24      // Master control register
#define MPU_ACCEL_CFG_REG 0x1C      // Accelerometer configuration register
#define MPU_GYRO_CFG_REG 0x1B       // Gyroscope configuration register
#define MPU_PWR_REG 0x6B            // Wake up register
#define MPU_FIFO_CTRL_REG 0x6A      // User ctrl reg: 0x04 - fifo reset; 0x40 - fifo enable
#define MPU_FIFO_ENBL_MASK_REG 0x23 // FIFO register for controlling which data is stored in the FIFO buffer
#define MPU_FIFO_COUNT_H_REG 0x72   // FIFO count register
#define MPU_FIFO_COUNT_L_REG 0x73   // FIFO count register
#define MPU_FIFO_DATA_REG 0x74      // FIFO data register

#define MPU_FIFO_ENBL_ACCEL_GYRO 0x78 // Enable gyro and accelerometer data in FIFO buffer
#define MPU_ACCEL_X_H 0x3B            // Accelerometer X-axis data high byte register
#define MPU_TEMP_H 0x41               // Temperature data high byte register
#define MPU_GYRO_X_H 0x43             // Gyroscope X-axis data high byte register

#define FIFO_ENABLE_RESET 0x44 // FIFO enable and reset mask

/*
    ACCEL AND GYRO LOW PASS FILTER SETTINGS
    hex,      Gyroscope freq, delay        Accelerometer freq, delay
    0x00      260Hz, 0ms delay             256Hz, 0.98ms delay
    0x01      188Hz, 1.9ms delay           188Hz, 1.9ms delay
    0x02      98Hz, 2.8ms delay            98Hz, 2.8ms delay
    0x03      42Hz, 4.8ms delay            42Hz, 4.8ms delay
    0x04      20Hz, 8.3ms delay            20Hz, 8.3ms delay
    0x05      10Hz, 13.8ms delay           10Hz, 13.4ms delay
    0x06      5Hz, 19ms delay              5Hz, 18.6ms delay
*/
#define MPU_FILTER_FREQ_260Hz 0x00
#define MPU_FILTER_FREQ_188Hz 0x01
#define MPU_FILTER_FREQ_98Hz 0x02
#define MPU_FILTER_FREQ_42Hz 0x03
#define MPU_FILTER_FREQ_20Hz 0x04
#define MPU_FILTER_FREQ_10Hz 0x05
#define MPU_FILTER_FREQ_5Hz 0x06

/*
ACCEL FULL SCALE SETTINGS
AFS_SEL Full Scale Range LSB Sensitivity
0 ±2g 16384 LSB/g
1 ±4g 8192 LSB/g
2 ±8g 4096 LSB/g
3 ±16g 2048 LSB/g
GYRO FULL SCALE SETTINGS
FS_SEL Full Scale Range LSB Sensitivity
0 ± 250 °/s 131 LSB/°/s
1 ± 500 °/s 65.5 LSB/°/s
2 ± 1000 °/s 32.8 LSB/°/s
3 ± 2000 °/s 16.4 LSB/°/s
*/
#define MPU_ACCEL_FS_2g 16384
#define MPU_ACCEL_FS_4g 8192
#define MPU_ACCEL_FS_8g 4096
#define MPU_ACCEL_FS_16g 2048
#define MPU_GYRO_FS_250deg 131
#define MPU_GYRO_FS_500deg 65.5
#define MPU_GYRO_FS_1000deg 32.8
#define MPU_GYRO_FS_2000deg 16.4

/*
GYRO Bit4 Bit3 setting
FS_SEL Full Scale Range
0 ± 250 °/s
1 ± 500 °/s
2 ± 1000 °/s
3 ± 2000 °/s
*/
#define MPU_GYRO_SET_FS_250deg 0x0
#define MPU_GYRO_SET_FS_500deg 0x08
#define MPU_GYRO_SET_FS_1000deg 0x10
#define MPU_GYRO_SET_FS_2000deg 0x18

/*
ACCEL Bit4 Bit3 setting
AFS_SEL Full Scale Range
0 ± 2g
1 ± 4g
2 ± 8g
3 ± 16g
*/
#define MPU_ACCEL_SET_FS_2g 0x0
#define MPU_ACCEL_SET_FS_4g 0x08
#define MPU_ACCEL_SET_FS_8g 0x10
#define MPU_ACCEL_SET_FS_16g 0x18

// Custom typedefs
typedef struct mpu_data_type
{
    int16_t accel_gyro_raw[12];          // for reading raw accel and gyro data from MPU6050
    int32_t accel_gyro_avg_deviation[6]; // average deviation of the accel and gyro
    double accel_gyro_g[6];              // accel gyro converted to g and deg/s
} mpu_data_type;

typedef struct i2c_buffer_type
{
    uint8_t read_buffer[I2C_READ_BUFF_SIZE];   // read buffer
    uint8_t write_buffer[I2C_WRITE_BUFF_SIZE]; // write buffer
} i2c_buffer_type;

// Init functions
void print_binary(uint8_t *);
void init_i2c(void);
void mpu_initial_setup(i2c_buffer_type *);
void mpu_transmit_receive(i2c_buffer_type *, uint8_t, uint8_t);
void mpu_transmit(i2c_buffer_type *, uint8_t);

// Global variables
mpu_data_type mpu_data = {
    .accel_gyro_raw = {0},
    .accel_gyro_avg_deviation = {0},
};

i2c_buffer_type i2c_buffer = {
    .read_buffer = {0},
    .write_buffer = {0},
};

// Master configuration
const i2c_master_bus_config_t master_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA_IO,
    .scl_io_num = I2C_SCL_IO,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true};

// Slave device configuration
const i2c_device_config_t MPU6050_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU_ADR_REG,
    .scl_speed_hz = I2C_FREQ_HZ};

// Data handle structs to be used by the driver
i2c_master_bus_handle_t master_bus_handle;
i2c_master_dev_handle_t master_dev_handle;

void app_main(void)
{
    // Initialize the I2C bus
    init_i2c();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    // Enable the FIFO buffer (otherwise the data is not stored in the MPU FIFO)
    mpu_enable_fifo(&i2c_buffer);

    // Read MPU data registers
    while (true)
    {
        // Clear the FIFO buffer to start fresh
        mpu_reset_fifo(&i2c_buffer);

        // Delay for at least 1 ms to allow the FIFO buffer to fill up
        vTaskDelay(10 / portTICK_PERIOD_MS); // wait 5 ms

        // Read FIFO buffer
        mpu_read_fifo_buffer(12, true);
        mpu_extract_buffer(false);
        mpu_scale_accel_gyro();

        // printf("%.1f, %.1f, %.1f", accel_x_g, accel_y_g, accel_z_g);
        // printf("%.2f; %.2f; %.2f\n", accel_x_g, accel_y_g, accel_z_g);
        printf("%.2f; %.2f; %.2f\n", mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g);
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}

/*
 * Print binary representation of a number
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

/*
 * Initialize the I2C bus
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

/*
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
    i2c_buffer->write_buffer[0] = MPU_FILTER_FREQ_REG;
    i2c_buffer->write_buffer[1] = MPU_FILTER_FREQ_5Hz;
    mpu_transmit(i2c_buffer, 2);

    // Set accelerometer to +-8g full scale range
    i2c_buffer->write_buffer[0] = MPU_ACCEL_CFG_REG;
    i2c_buffer->write_buffer[1] = MPU_ACCEL_SET_FS_8g;
    mpu_transmit(i2c_buffer, 2);

    // Set gyroscope to +-1000 deg/s full scale range
    i2c_buffer->write_buffer[0] = MPU_GYRO_CFG_REG;
    i2c_buffer->write_buffer[1] = MPU_GYRO_SET_FS_1000deg;
    mpu_transmit(i2c_buffer, 2);

    // Set what data is stored in the FIFO buffer
    i2c_buffer->write_buffer[0] = MPU_FIFO_ENBL_MASK_REG;
    i2c_buffer->write_buffer[1] = MPU_FIFO_ENBL_ACCEL_GYRO; // Enable gyro and accelerometer data (no tamp and slave)
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

/*
* Enable the FIFO buffer of the MPU6050 sensor
*
* @param i2c_buffer: struct with write_buffer, read_buffer
* @return void
*/
void mpu_enable_fifo(i2c_buffer_type *i2c_buffer)
{
    i2c_buffer->write_buffer[0] = MPU_FIFO_CTRL_REG;
    i2c_buffer->write_buffer[1] = 0x40; // Enable FIFO
    mpu_transmit(i2c_buffer, 2);
}

/*
* Reset the FIFO buffer of the MPU6050 sensor
*
* @param i2c_buffer: struct with write_buffer, read_buffer
* @return void
*/
void mpu_reset_fifo(i2c_buffer_type *i2c_buffer)
{
    // Fifo reset bit = 0x04 and fifo enable bit = 0x40
    // Therefore reset and enable FIFO = 0x44
    i2c_buffer->write_buffer[0] = MPU_FIFO_CTRL_REG;
    i2c_buffer->write_buffer[1] = 0x44;
    mpu_transmit(i2c_buffer, 2);
}

/*
 * Transmit MPU register data and read it's value.
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

/*
 * Write data to specific MPU6050 register
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

/*
 * Read data from the FIFO buffer of the MPU6050 sensor
 *
 * You first have to set up the FIFO buffer to store just the
 * accel and gyro data.
 *
 * @param fifo_bytes: number of bytes to read from the FIFO buffer
 * @param reset_fifo: reset the FIFO buffer after reading
 *
 * @return void
 */
void mpu_read_fifo_buffer(i2c_buffer_type *i2c_buffer, mpu_data_type *mpu_data)
{
    // First read FIFO count to make sure it is at least 12 bytes long (accel and gyro high and low registers)
    i2c_buffer->write_buffer[0] = MPU_FIFO_COUNT_H_REG;
    mpu_transmit_receive(i2c_buffer, 1, 2);
    uint16_t fifo_count = (i2c_buffer->read_buffer[0] << 8) | i2c_buffer->read_buffer[1];
    if (fifo_count >= 12)
    {
        i2c_buffer->write_buffer[0] = MPU_FIFO_DATA_REG;
        mpu_transmit_receive(write_buffer, 1, read_buffer, 12);

        if (reset_fifo)
            mpu_reset_fifo();
    }
}

// /*
//  * Read accelerometer and gyroscope data from the FIFO buffer
//  *
//  * @param void
//  * @return void
//  */
// void mpu_read_accel_and_gyro()
// {
//     write_buffer[0] = MPU_ACCEL_X_H; // 0x3B
//     mpu_transmit_receive(write_buffer, 1, read_buffer, 6);

//     write_buffer[0] = MPU_GYRO_X_H; // 0x43
//     mpu_transmit_receive(write_buffer, 1, &read_buffer[6], 6);
// }

// /*
//  * Extract gyro, temperature and accel data
//  *
//  * After reading FIFO buffer, the data has to be extracted from buffer
//  * and converted to uint16_t values. Subsequent pairs of registers are
//  * combined to form the final value.
//  */
// void mpu_extract_buffer(bool with_temp)
// {
//     uint8_t shift = 0;
//     // Extract data from the read buffer
//     accel_x = (read_buffer[0] << 8) | read_buffer[1];
//     accel_y = (read_buffer[2] << 8) | read_buffer[3];
//     accel_z = (read_buffer[4] << 8) | read_buffer[5];
//     if (with_temp)
//     {
//         temperature = (read_buffer[6] << 8) | read_buffer[7]; // Not used
//         ++shift;
//     }
//     gyro_x = (read_buffer[6 + shift] << 8) | read_buffer[7 + shift];
//     gyro_y = (read_buffer[8 + shift] << 8) | read_buffer[9 + shift];
//     gyro_z = (read_buffer[10 + shift] << 8) | read_buffer[11 + shift];
// }

// /*
//  * Scale mpu readings to full scale range
//  * Accel full scale range: +-8g
//  * Gyro full scale range: +-1000 deg/s
//  *
//  * @param void
//  * @return void
//  */
// void mpu_scale_accel_gyro()
// {
//     accel_x_g = (double)accel_x / MPU_ACCEL_FULL_SCALE_8g;
//     accel_y_g = (double)accel_y / MPU_ACCEL_FULL_SCALE_8g;
//     accel_z_g = (double)accel_z / MPU_ACCEL_FULL_SCALE_8g;
//     gyro_x_g = (double)gyro_x / MPU_GYRO_FULL_SCALE_1000deg;
//     gyro_y_g = (double)gyro_y / MPU_GYRO_FULL_SCALE_1000deg;
//     gyro_z_g = (double)gyro_z / MPU_GYRO_FULL_SCALE_1000deg;
// }

// /*
//  * Resets the FIFO buffer of the MPU6050 sensor
//  *
//  * To avoid FIFO overflow you have to reset the buffer after reading
//  *
//  * @param void
//  * @return void
//  */
// void mpu_reset_fifo()
// {
//     mpu_transmit(fifo_reset_buffer, 2);
// }

// /*
//  * Calculate average error of the MPU6050 sensor
//  *
//  * Calculate average accel and gyro output error
//  */
// void mpu_calculate_avg_error(uint32_t *accel_x_err, uint32_t *accel_y_err, uint32_t *accel_z_err, uint32_t *gyro_x_err, uint32_t *gyro_y_err, uint32_t *gyro_z_err, int samples)
// {
//     // jutr moram uint32_t error vrednostim prištet n meritev in v vsakem runu delit z 2. Tko pride povprečna napaka
//     mpu_read_accel_and_gyro();
//     mpu_extract_buffer(false);
//     for (int i = 0; i < samples; ++i)
//     {

//         mpu_read_accel_and_gyro();
//         mpu_extract_buffer(false);

//         *accel_x_err += accel_x;
//         *accel_y_err += accel_y;
//         *accel_z_err += accel_z;
//         *gyro_x_err += gyro_x;
//         *gyro_y_err += gyro_y;
//         *gyro_z_err += gyro_z;
//     }
// }
