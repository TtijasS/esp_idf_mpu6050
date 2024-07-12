#include "data_structs.h"

mpu_data_type mpu_data = {
	.accel_gyro_raw = {0},
	.avg_err = {-1496.185791, 147.318146, 19547.138672, -667.104980, -361.012207, -4.112995},
	// .avg_err = {0},
	.accel_gyro_g = {0},
};

i2c_buffer_type i2c_buffer = {
	.read_buffer = {0},
	.write_buffer = {0},
};