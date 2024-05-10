#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_config.h"
#include "data_structs.h"
#include "custom_functions.h"

void print_avg_errors(mpu_data_type *mpu_data);

void app_main(void)
{
    // Setup master handles and initialize I2C
    init_i2c();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    // Enable the FIFO buffer (otherwise the data is not stored in the MPU FIFO)
    mpu_fifo_enable(&i2c_buffer);

    // ESP_LOGI(TAG, "Calculating average error");
    // mpu_calibrate(&i2c_buffer, &mpu_data, 10, false);

    // printf("Calibration done with calibration numbers:\n");
    // print_avg_errors(&mpu_data);

    // printf("Starting main loop in 5s\n");
    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "Calculating average error with substracting error");
    // mpu_calibrate(&i2c_buffer, &mpu_data, 20, true);

    // mpu_fifo_reset(&i2c_buffer);
    while (1)
    {
        mpu_fifo_read_extract(&i2c_buffer, &mpu_data);
        printf("%d;\n", mpu_data.accel_gyro_raw[1]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}

void print_avg_errors(mpu_data_type *mpu_data)
{
    printf("< ");
    for (int i = 0; i < 5; ++i)
    {
        printf("%g, ", mpu_data->avg_err[i]);
    }
    printf("%g >\n", mpu_data->avg_err[5]);
}