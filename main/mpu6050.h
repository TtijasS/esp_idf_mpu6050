#ifndef MPU6050_H
#define MPU6050_H

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include "constants.h"
#include "my_i2c_config.h"
#include "data_structs.h"
#include <esp_log.h>

// Functions prototypes
void mpu_initial_setup(i2c_buffer_type *);
void mpu_transmit_receive(i2c_buffer_type *, uint8_t, uint8_t);
void mpu_transmit(i2c_buffer_type *, uint8_t);
void mpu_fifo_enable(i2c_buffer_type *);
void mpu_fifo_reset(i2c_buffer_type *);
uint16_t *mpu_fifo_count(i2c_buffer_type *);
bool mpu_fifo_read_extract(i2c_buffer_type *, mpu_data_type *);
void mpu_fifo_extract_buffer(i2c_buffer_type *, mpu_data_type *);
void mpu_readings_to_fs(mpu_data_type *);
bool mpu_readings_avg_err(i2c_buffer_type*, mpu_data_type *, uint16_t repetitions);


#endif // MPU6050_H