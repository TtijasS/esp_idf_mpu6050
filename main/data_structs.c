#include "data_structs.h"

mpu_data_type mpu_data = {
	.accel_gyro_raw = {0},
	.avg_err = {7.13768747, 80.83285601, 19749.88091774, -627.37748184, -373.46484596, 10.40970808},
	// .avg_err = {0},
	.accel_gyro_g = {0},
};

i2c_buffer_type i2c_buffer = {
	.read_buffer = {0},
	.write_buffer = {0},
};