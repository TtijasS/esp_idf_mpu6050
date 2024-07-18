#include "data_structs.h"

mpuDataType mpu_data_t = {
	.accel_gyro_raw = {0},
	.avg_err = {-1165.642822, 518.381592, 16480.429688, 225.682465, -145.626221, 109.899750},
	// .avg_err = {0},
	.accel_gyro_g = {0},
};

i2cBufferType i2c_buffer_t = {
	.read_buffer = {0},
	.write_buffer = {0},
};