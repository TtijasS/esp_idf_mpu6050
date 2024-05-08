#ifndef CONSTANTS_H
#define CONSTANTS_H

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

#define MPU_FIFO_ENABLE_REG_SET 0x78 // Enable gyro and accelerometer data in FIFO buffer
#define MPU_ACCEL_X_H 0x3B            // Accelerometer X-axis data high byte register
#define MPU_TEMP_H 0x41               // Temperature data high byte register
#define MPU_GYRO_X_H 0x43             // Gyroscope X-axis data high byte register

#define FIFO_ENABLE_RESET 0x44 // FIFO enable and reset mask

// #define KAMERA 0x33 // Kamera register
/**
 * @brief MPU FILTER FREQUENCY SETTINGS
 * @attention Uncomment one of the filter frequency settings
 * 
 *   ACCEL AND GYRO LOW PASS FILTER SETTINGS
 *   hex,      Gyroscope freq, delay        Accelerometer freq, delay
 *   0x00      260Hz, 0ms delay             256Hz, 0.98ms delay
 *   0x01      188Hz, 1.9ms delay           188Hz, 1.9ms delay
 *   0x02      98Hz, 2.8ms delay            98Hz, 2.8ms delay
 *   0x03      42Hz, 4.8ms delay            42Hz, 4.8ms delay
 *   0x04      20Hz, 8.3ms delay            20Hz, 8.3ms delay
 *   0x05      10Hz, 13.8ms delay           10Hz, 13.4ms delay
 *   0x06      5Hz, 19ms delay              5Hz, 18.6ms delay
 */
// #define MPU_FILTER_FREQ_REG_SET_260Hz 0x00 // 260Hz, 0ms delay
#define MPU_FILTER_FREQ_REG_SET_188Hz 0x01 // 188Hz, 1.9ms delay
// #define MPU_FILTER_FREQ_REG_SET_98Hz 0x02 // 98Hz, 2.8ms delay
// #define MPU_FILTER_FREQ_REG_SET_42Hz 0x03 // 42Hz, 4.8ms delay
// #define MPU_FILTER_FREQ_REG_SET_20Hz 0x04 // 20Hz, 8.3ms delay
// #define MPU_FILTER_FREQ_REG_SET_10Hz 0x05 // 10Hz, 13.8ms delay
// #define MPU_FILTER_FREQ_REG_SET_5Hz 0x06 // 5Hz, 19ms delay
// ---------- ifdef blocks for setting the filter frequency
#ifdef MPU_FILTER_FREQ_REG_SET_260Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_260Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_188Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_188Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_98Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_98Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_42Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_42Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_20Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_20Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_10Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_10Hz
#endif
#ifdef MPU_FILTER_FREQ_REG_SET_5Hz
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_5Hz
#endif
#ifndef MPU_FILTER_FREQ_REG_SET
#define MPU_FILTER_FREQ_REG_SET MPU_FILTER_FREQ_REG_SET_260Hz
#endif
// ----------


/**
 * @brief ACCEL FULL SCALE SETTINGS
 * @attention Uncomment one of the accel FS settings
 * 
 * AFS_SEL Full Scale Range LSB Sensitivity
 * - 0 ±2g 16384 LSB/g
 * - 1 ±4g 8192 LSB/g
 * - 2 ±8g 4096 LSB/g
 * - 3 ±16g 2048 LSB/g
 */
#define MPU_ACCEL_FS_2g 16384 // 2g, 16384 LSB/g
// #define MPU_ACCEL_FS_4g 8192 // 4g, 8192 LSB/g
// #define MPU_ACCEL_FS_8g 4096 // 8g, 4096 LSB/g
// #define MPU_ACCEL_FS_16g 2048 // 16g, 2048 LSB/g
/**
 * ACCEL REGISTER SETTINGS (bit4, bit3)
 * AFS_SEL Full Scale Range
 * 0 ± 2g
 * 1 ± 4g
 * 2 ± 8g
 * 3 ± 16g
 */
#define MPU_ACCEL_REG_SET_2g 0x0 // don't comment this line
#define MPU_ACCEL_REG_SET_4g 0x08 // don't comment this line
#define MPU_ACCEL_REG_SET_8g 0x10 // don't comment this line
#define MPU_ACCEL_REG_SET_16g 0x18 // don't comment this line
// ---------- ifdef blocks for setting the accel full scale
#ifdef MPU_ACCEL_FS_2g
#define MPU_ACCEL_FS MPU_ACCEL_FS_2g
#define MPU_ACCEL_REG_SET MPU_ACCEL_REG_SET_2g
#endif
#ifdef MPU_ACCEL_FS_4g
#define MPU_ACCEL_FS MPU_ACCEL_FS_4g
#define MPU_ACCEL_REG_SET MPU_ACCEL_REG_SET_4g
#endif
#ifdef MPU_ACCEL_FS_8g
#define MPU_ACCEL_FS MPU_ACCEL_FS_8g
#define MPU_ACCEL_REG_SET MPU_ACCEL_REG_SET_8g
#endif
#ifdef MPU_ACCEL_FS_16g
#define MPU_ACCEL_FS MPU_ACCEL_FS_16g
#define MPU_ACCEL_REG_SET MPU_ACCEL_REG_SET_16g
#endif
#ifndef MPU_ACCEL_FS
#define MPU_ACCEL_FS MPU_ACCEL_FS_2g
#define MPU_ACCEL_REG_SET MPU_ACCEL_REG_SET_2g
#endif

/**
 * @brief GYRO FULL SCALE SETTINGS
 * @attention Uncomment one of the gyro FS settings
 * 
 * FS_SEL Full Scale Range LSB Sensitivity
 * 0 ± 250 °/s 131 LSB/°/s
 * 1 ± 500 °/s 65.5 LSB/°/s
 * 2 ± 1000 °/s 32.8 LSB/°/s
 * 3 ± 2000 °/s 16.4 LSB/°/s
 */
#define MPU_GYRO_FS_250deg 131 // 250deg, 131 LSB/°/s
// #define MPU_GYRO_FS_500deg 65.5 // 500deg, 65.5 LSB/°/s
// #define MPU_GYRO_FS_1000deg 32.8 // 1000deg, 32.8 LSB/°/s
// #define MPU_GYRO_FS_2000deg 16.4 // 2000deg, 16.4 LSB/°/s
/**
 * @brief GYRO FULL SCALE REGISTER SETTINGS
 * FS_SEL Full Scale Range
 * 0 ± 250 °/s
 * 1 ± 500 °/s
 * 2 ± 1000 °/s
 * 3 ± 2000 °/s
 */
#define MPU_GYRO_REG_SET_250deg 0x0 // don't comment this line
#define MPU_GYRO_REG_SET_500deg 0x08 // don't comment this line
#define MPU_GYRO_REG_SET_1000deg 0x10 // don't comment this line
#define MPU_GYRO_REG_SET_2000deg 0x18 // don't comment this line
// ---------- ifdef blocks for setting the gyro full scale
#ifdef MPU_GYRO_FS_250deg
#define MPU_GYRO_FS MPU_GYRO_FS_250deg
#define MPU_GYRO_REG_SET MPU_GYRO_REG_SET_250deg
#endif
#ifdef MPU_GYRO_FS_500deg
#define MPU_GYRO_FS MPU_GYRO_FS_500deg
#define MPU_GYRO_REG_SET MPU_GYRO_REG_SET_500deg
#endif
#ifdef MPU_GYRO_FS_1000deg
#define MPU_GYRO_FS MPU_GYRO_FS_1000deg
#define MPU_GYRO_REG_SET MPU_GYRO_REG_SET_1000deg
#endif
#ifdef MPU_GYRO_FS_2000deg
#define MPU_GYRO_FS MPU_GYRO_FS_2000deg
#define MPU_GYRO_REG_SET MPU_GYRO_REG_SET_2000deg
#endif
#ifndef MPU_GYRO_FS
#define MPU_GYRO_FS MPU_GYRO_FS_250deg
#define MPU_GYRO_REG_SET MPU_GYRO_REG_SET_250deg
#endif

#endif
