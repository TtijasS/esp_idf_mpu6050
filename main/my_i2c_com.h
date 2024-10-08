// i2c_config.h
#ifndef MY_I2C_COM_H
#define MY_I2C_COM_H

#include <driver/i2c_master.h>
#include <esp_log.h>
#include "constants.h"


// Extern declarations for global configurations and handles
extern const i2c_master_bus_config_t i2c_master_bus_config;
extern const i2c_device_config_t i2c_master_device_config;
extern i2c_master_bus_handle_t i2c_master_bus_handle;
extern i2c_master_dev_handle_t i2c_master_dev_handle;
int i2c_init();

#endif // MY_I2C_COM_H
