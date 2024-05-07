#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_config.h"
#include "data_structs.h"
#include "custom_functions.h"

void print_mpu_data_raw(mpu_data_type *mpu_data);
void print_mpu_data_avg_err(mpu_data_type *mpu_data);

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
    mpu_data_calculate_avg_err(&i2c_buffer, &mpu_data, 1000);
    print_mpu_data_avg_err(&mpu_data);
    printf("\n");

    mpu_fifo_reset(&i2c_buffer);
    // for (int i = 0; i < 10000; ++i)
    // {
    //     // Read the FIFO buffer
    //     mpu_fifo_read_extract(&i2c_buffer, &mpu_data);
    //     mpu_data_substract_err(&mpu_data);
    //     mpu_data_to_fs(&mpu_data);

    //     ESP_LOGI(TAG, "%g", mpu_data.accel_gyro_g[0]);

    //     vTaskDelay(100 / portTICK_PERIOD_MS); // wait x ms (x10 == 1ms)
    // }
    // mpu_fifo_read_extract(&i2c_buffer, &mpu_data);
    // mpu_data_substract_err(&mpu_data);
    // mpu_data_to_fs(&mpu_data);

    // ESP_LOGI(TAG, "Testing data");
    // mpu_fifo_read_extract(&i2c_buffer, &mpu_data);
    // printf("Raw data: ");
    // print_mpu_data_raw(&mpu_data);
    // printf("\n");
    // printf("Err data:");
    // print_mpu_data_avg_err(&mpu_data);

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}

void print_mpu_data_raw(mpu_data_type *mpu_data)
{
    for (int i = 0; i < 6; i++)
    {
        printf("%d; ", mpu_data->accel_gyro_raw[i]);
    }
}

/**
 * @brief
 *
 * @param mpu_data
 */
void print_mpu_data_avg_err(mpu_data_type *mpu_data)
{
    for (int i = 0; i < 6; i++)
    {
        printf("%.2f; ", mpu_data->avg_err[i]);
    }
}