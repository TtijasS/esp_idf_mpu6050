#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <freertos/FreeRTOS.h>
#include "constants.h"

/**
 * @brief Data structure for storing the MPU6050 sensor data
 *
 * The data structure stores the raw accelerometer and gyroscope data from the MPU6050 sensor.
 *
 */
typedef struct mpuDataType
{
    // accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw
    int16_t accel_gyro_raw[6];

    // accel_x_er, accel_y_err, accel_z_err, gyro_x_err, gyro_y_err, gyro_z_err
    float avg_err[6];

    // accel_x_g, accel_y_g, accel_z_g, gyro_x_g, gyro_y_g, gyro_z_g
    float accel_gyro_g[6];
} mpuDataType;

/**
 * @brief Data structure for storing the I2C read and write buffers
 */
typedef struct i2cBufferType
{
    // Read buffer array of size I2C_READ_BUFF_SIZE
    uint8_t read_buffer[I2C_READ_BUFF_SIZE]; // read buffer

    // Write buffer array of size I2C_WRITE_BUFF_SIZE
    uint8_t write_buffer[I2C_WRITE_BUFF_SIZE]; // write buffer
} i2cBufferType;

// Declare the variables as extern
extern mpuDataType mpu_data_t;
extern i2cBufferType i2c_buffer_t;

typedef struct indexed_float_type
{
    uint32_t index;
    float value;

} indexed_float_type;

#endif // DATA_STRUCTS_H