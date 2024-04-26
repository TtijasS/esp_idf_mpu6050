#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_config.h"
#include "data_structs.h"
#include "custom_functions.h"

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
    mpu_fifo_reset(&i2c_buffer);

    // Delay for at least 1 ms to allow the FIFO buffer to fill up
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait x ms (x10 == 1ms)

    // Read FIFO buffer
    mpu_fifo_read_extract(&i2c_buffer, &mpu_data);
    mpu_readings_to_fs(&mpu_data);

    mpu_readings_avg_err(&i2c_buffer, &mpu_data, 1000);



    // printf("%.1f, %.1f, %.1f", accel_x_g, accel_y_g, accel_z_g);
    // printf("%.2f; %.2f; %.2f\n", accel_x_g, accel_y_g, accel_z_g);
    // printf("%.2f; %.2f; %.2f\n", mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g);

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}
