	#ifndef CONSTANTS_H
	#define CONSTANTS_H

	static const char *TAG = "I2C Example";

	#define TASK_PRIORITY_SAMPLING 2
	#define TASK_PRIORITY_FFT 2
	#define TASK_PRIORITY_SORT 1
	#define TASK_PRIORITY_UART 1

	#define TASK_STACK_MPU_SAMPLING 2048 // 2048 words (8192 bytes)
	#define TASK_STACK_FFT 4096			 // 4096 words (16384 bytes)
	#define TASK_STACK_UART 2048		 // 2048 words (8192 bytes)
	#define TASK_STACK_SORT_MAG 1024	 // 1024 words

	#define N_SAMPLES 1024					  // Number of samples to run FFT on
	#define FFT_COMPONENTS_SIZE N_SAMPLES * 2 // Size of ffc components array size
	#define MAGNITUDES_SIZE N_SAMPLES / 2	  // size of magnitudes struct array size

	// I2C CONFIGURATION
	#define I2C_SCL_IO CONFIG_I2C_MASTER_SCL // GPIO number used for I2C master clock
	#define I2C_SDA_IO CONFIG_I2C_MASTER_SDA // GPIO number used for I2C master data
	#define I2C_PORT_NUM I2C_NUM_0			 // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
	#define I2C_FREQ_HZ 400000				 // I2C master clock frequency
	#define I2C_TIMEOUT_MS 1000				 // I2C timeout in milliseconds
	// buffer sizes
	#define I2C_READ_BUFF_SIZE 14 // I2C max read buffer size
	#define I2C_WRITE_BUFF_SIZE 2 // i2c max write buffer size

	// UART CONFIGURATION
	#define UART_BAUD_RATE 3000000 // UART baud rate
	#define UART_BUFFER_SIZE 1024 // UART buffer size

	// MPU REGISTERS FOR COMMUNICATION
	#define MPU_ADR_REG 0x68	  // MPU6050 slave address
	#define MPU_WHO_AM_I_REG 0x75 // Reg adr - get "who am I" response

	// MPU REGISTERS FOR CONFIGURATION
	#define MPU_FILTER_FREQ_REG 0x1A // Reg adr - low pass filter frequency config
	#define MPU_MSTR_CTRL_REG 0x24	 // Reg adr - master control
	#define MPU_PWR_REG 0x6B		 // Reg adr - wake up the MPU6050
	#define MPU_USER_CTRL_REG 0x6A	 // Reg adr - FIFO buffer enable, reset (and more)
	#define MPU_FIFO_EN_REG 0x23	 // Reg adr - config which data is stored in FIFO buffer
	#define MPU_GYRO_CFG_REG 0x1B	 // Reg adr - gyro full scale configuration
	#define MPU_ACCEL_CFG_REG 0x1C	 // Reg adr - accel configuration

	// MPU DATA REGISTERS
	#define MPU_FIFO_DATA_REG 0x74	  // Reg adr - FIFO data register
	#define MPU_ACCEL_X_H_REG 0x3B	  // Reg adr - accel X-axis data high byte register (MSB)
	#define MPU_TEMP_H_REG 0x41		  // Reg adr - temp data high byte register (MSB)
	#define MPU_GYRO_X_H_REG 0x43	  // Reg adr - gyro X-axis data high byte register (MSB)
	#define MPU_FIFO_COUNT_H_REG 0x72 // Reg adr - FIFO count MSB byte
	#define MPU_FIFO_COUNT_L_REG 0x73 // Reg adr - FIFO count LSB byte
	#define MPU_FIFO_OVERFLOW 0x3A	  // Reg adr - FIFO overflow register

	// MPU SETTING MASKS
	#define MPU_FIFO_EN_MASK 0x78	 // Reg mask - use with MPU_FIFO_EN_REG; Enable gyro and acel in FIFO)
	#define MPU_FIFO_RESET_MASK 0x44 // Reg mask - use with MPU_USER_CTRL_REG; Reset FIFO and keep it enabled: 0x04 - fifo reset; 0x40 - fifo enable)

	/**
	 * @brief MPU FILTER FREQUENCY SETTINGS
	 * @attention Uncomment one of the filter frequency settings
	 *
	 * Preprocessor macros to select the desired filter frequency.
	 * If none are defined, it defaults to 260Hz.
	 */
	// #define MPU_FILTER_FREQ_188Hz
	// #define MPU_FILTER_FREQ_98Hz
	// #define MPU_FILTER_FREQ_42Hz
	// #define MPU_FILTER_FREQ_20Hz
	// #define MPU_FILTER_FREQ_10Hz
	// #define MPU_FILTER_FREQ_5Hz

	// Mask definitions for MPU filter frequencies
	#define MPU_FILTER_FREQ_MASK_260Hz 0x00 // 260Hz, 0ms delay
	#define MPU_FILTER_FREQ_MASK_188Hz 0x01 // 188Hz, 1.9ms delay
	#define MPU_FILTER_FREQ_MASK_98Hz 0x02	// 98Hz, 2.8ms delay
	#define MPU_FILTER_FREQ_MASK_42Hz 0x03	// 42Hz, 4.8ms delay
	#define MPU_FILTER_FREQ_MASK_20Hz 0x04	// 20Hz, 8.3ms delay
	#define MPU_FILTER_FREQ_MASK_10Hz 0x05	// 10Hz, 13.8ms delay
	#define MPU_FILTER_FREQ_MASK_5Hz 0x06	// 5Hz, 19ms delay

	// Determine which filter frequency mask to use based on the defined preprocessor macro
	#if defined(MPU_FILTER_FREQ_188Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_188Hz
	#elif defined(MPU_FILTER_FREQ_98Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_98Hz
	#elif defined(MPU_FILTER_FREQ_42Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_42Hz
	#elif defined(MPU_FILTER_FREQ_20Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_20Hz
	#elif defined(MPU_FILTER_FREQ_10Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_10Hz
	#elif defined(MPU_FILTER_FREQ_5Hz)
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_5Hz
	#else
	#define MPU_FILTER_FREQ_MASK MPU_FILTER_FREQ_MASK_260Hz
	#endif

	// ---------- ACCELEROMETER FULL SCALE SETTINGS ----------
	/**
	 * @brief ACCEL FULL SCALE SETTINGS
	 * @attention Uncomment one of the accel FS_RANGE settings
	 * Default is ±2g, 16384 LSB/g
	 */
	// #define MPU_ACCEL_FS_RANGE_4G // 0 ± 4g, 8192 LSB/g
	// #define MPU_ACCEL_FS_RANGE_8G // 1 ± 8g, 4096 LSB/g
	// #define MPU_ACCEL_FS_RANGE_16G // 2 ± 16g, 2048 LSB/g

	// Accelerometer full scale ranges in LSB/g
	#define MPU_ACCEL_FS_2g 16384 // 2g, 16384 LSB/g
	#define MPU_ACCEL_FS_4g 8192  // 4g, 8192 LSB/g
	#define MPU_ACCEL_FS_8g 4096  // 8g, 4096 LSB/g
	#define MPU_ACCEL_FS_16g 2048 // 16g, 2048 LSB/g
	// Accelerometer full scale range masks
	#define MPU_ACCEL_FS_MASK_2g 0x00  // 2g
	#define MPU_ACCEL_FS_MASK_4g 0x08  // 4g
	#define MPU_ACCEL_FS_MASK_8g 0x10  // 8g
	#define MPU_ACCEL_FS_MASK_16g 0x18 // 16g

	// ---------- ifdef blocks for setting the accel full scale ----------
	#if defined(MPU_ACCEL_FS_RANGE_4G)
	#define MPU_ACCEL_FS MPU_ACCEL_FS_4g
	#define MPU_ACCEL_FS_MASK MPU_ACCEL_FS_MASK_4g
	#elif defined(MPU_ACCEL_FS_RANGE_8G)
	#define MPU_ACCEL_FS MPU_ACCEL_FS_8g
	#define MPU_ACCEL_FS_MASK MPU_ACCEL_FS_MASK_8g
	#elif defined(MPU_ACCEL_FS_RANGE_16G)
	#define MPU_ACCEL_FS MPU_ACCEL_FS_16g
	#define MPU_ACCEL_FS_MASK MPU_ACCEL_FS_MASK_16g
	#else
	// Default to 2g
	#define MPU_ACCEL_FS MPU_ACCEL_FS_2g
	#define MPU_ACCEL_FS_MASK MPU_ACCEL_FS_MASK_2g
	#endif

	/**
	 * @brief GYROSCOPE FULL SCALE SETTINGS
	 * @attention Uncomment one of the gyro FS_RANGE settings
	 * Default is 250 °/s
	 */
	// #define MPU_GYRO_FS_RANGE_500DEG // 0 ± 500 °/s
	// #define MPU_GYRO_FS_RANGE_1000DEG // 1 ± 1000 °/s
	// #define MPU_GYRO_FS_RANGE_2000DEG // 2 ± 2000 °/s

	// Gyroscope full scale ranges in LSB/°/s
	#define MPU_GYRO_FS_250deg 131	 // 250deg, 131 LSB/°/s
	#define MPU_GYRO_FS_500deg 65.5	 // 500deg, 65.5 LSB/°/s
	#define MPU_GYRO_FS_1000deg 32.8 // 1000deg, 32.8 LSB/°/s
	#define MPU_GYRO_FS_2000deg 16.4 // 2000deg, 16.4 LSB/°/s
	// Gyroscope full scale range masks
	#define MPU_GYRO_FS_MASK_250deg 0x00  // 250 °/s
	#define MPU_GYRO_FS_MASK_500deg 0x08  // 500 °/s
	#define MPU_GYRO_FS_MASK_1000deg 0x10 // 1000 °/s
	#define MPU_GYRO_FS_MASK_2000deg 0x18 // 2000 °/s

	// ---------- ifdef blocks for setting the gyro full scale ----------
	#if defined(MPU_GYRO_FS_RANGE_500DEG)
	#define MPU_GYRO_FS MPU_GYRO_FS_500deg
	#define MPU_GYRO_FS_MASK MPU_GYRO_FS_MASK_500deg
	#elif defined(MPU_GYRO_FS_RANGE_1000DEG)
	#define MPU_GYRO_FS MPU_GYRO_FS_1000deg
	#define MPU_GYRO_FS_MASK MPU_GYRO_FS_MASK_1000deg
	#elif defined(MPU_GYRO_FS_RANGE_2000DEG)
	#define MPU_GYRO_FS MPU_GYRO_FS_2000deg
	#define MPU_GYRO_FS_MASK MPU_GYRO_FS_MASK_2000deg
	#else
	#define MPU_GYRO_FS MPU_GYRO_FS_250deg
	#define MPU_GYRO_FS_MASK MPU_GYRO_FS_MASK_250deg
	#endif

	#endif // CONSTANTS_H
