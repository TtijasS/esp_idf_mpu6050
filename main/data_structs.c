#include "data_structs.h"

mpu_data_type mpu_data = {
	.accel_gyro_raw = {0},
	.avg_err = {0},
	.accel_gyro_g = {0},
};

i2c_buffer_type i2c_buffer = {
	.read_buffer = {0},
	.write_buffer = {0},
};