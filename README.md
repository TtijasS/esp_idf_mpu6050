This is a MPU6050 custom driver that reads accel and gyro values directly from registers using ESP-IDF.
The FIFO buffer was ommited because of the synchronization problems that will cause byte missalignment, which will combine to veird readings.
