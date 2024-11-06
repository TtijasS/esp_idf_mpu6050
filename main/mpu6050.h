#ifndef MPU6050_H
#define MPU6050_H

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include "constants.h"
#include "my_i2c_com.h"
#include "data_structs.h"
#include <esp_log.h>

// Functions prototypes
int mpu_initial_setup(i2cBufferType *);
int mpu_wake_up_senzor(i2cBufferType *);
int mpu_set_filter_freq(i2cBufferType *);
int mpu_wake_up_senzor(i2cBufferType *);
int mpu_set_accel_fullscale(i2cBufferType *);
int mpu_set_gyro_fullscale(i2cBufferType *);
int mpu_disable_fifo(i2cBufferType *);
int mpu_transmit_receive(i2cBufferType *, uint8_t, uint8_t);
int mpu_transmit(i2cBufferType *, uint8_t);
void mpu_fifo_enable(i2cBufferType *);
void mpu_fifo_reset(i2cBufferType *);
uint16_t mpu_fifo_count(i2cBufferType *);
bool mpu_fifo_overflow_check(i2cBufferType *);
bool mpu_fifo_read_extract(i2cBufferType *, mpuDataType *);
void mpu_fifo_extract_buffer(i2cBufferType *, mpuDataType *);
void mpu_data_to_fs(mpuDataType *mpu_data_t, bool accel_only);
bool mpu_calibrate(i2cBufferType*, mpuDataType *, uint8_t, bool);
void mpu_data_substract_err(mpuDataType *mpu_data_t, bool accel_only);
void mpu_avg_err_divide(mpuDataType *, uint16_t);
float *mpu_fifo_read_to_array(i2cBufferType *, mpuDataType *, float *, uint8_t, bool, bool);
bool mpu_data_read_extract(i2cBufferType *, mpuDataType *);
bool mpu_data_read_extract_accel(i2cBufferType *, mpuDataType *);
bool mpu_data_sum_error(i2cBufferType *, mpuDataType *, bool);
bool mpu_data_calibrate(i2cBufferType *, mpuDataType *, uint8_t);
void mpu_data_reset(mpuDataType *);
void mpu_avg_err_print(mpuDataType *);

#endif // MPU6050_H