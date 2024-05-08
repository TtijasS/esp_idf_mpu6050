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

    // Read MPU data registers
    // Clear the FIFO buffer to start fresh

    // Delay for at least 1 ms to allow the FIFO buffer to fill up
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait x ms (x10 == 1ms)
    mpu_fifo_reset(&i2c_buffer);

    ESP_LOGI(TAG, "Calculating average error");
    mpu_calibrate(&i2c_buffer, &mpu_data, 40, false);
    print_avg_errors(&mpu_data);

    // ESP_LOGI(TAG, "Calculating average error with substracting error");
    // mpu_calibrate(&i2c_buffer, &mpu_data, 20, true);
    // print_avg_errors(&mpu_data);


    mpu_fifo_reset(&i2c_buffer);
    uint16_t print_count = 0;
    while (1)
    {
        print_count++;
        // Read the FIFO buffer
        if(mpu_fifo_read_extract(&i2c_buffer, &mpu_data)){
            // printf("%i; ", mpu_data.accel_gyro_raw[0]);
            mpu_data_substract_err(&mpu_data);
            // printf("%i; ", mpu_data.accel_gyro_raw[0]);
            mpu_data_to_fs(&mpu_data);
        }

        if (print_count >= 500){
            printf("%g\n", mpu_data.accel_gyro_g[0]);
            mpu_fifo_reset(&i2c_buffer);
            print_count = 0;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // vTaskDelay(5 / portTICK_PERIOD_MS);
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