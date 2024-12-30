#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"
#include <driver/i2c.h>

#ifndef ICM42688P_H
#define ICM42688P_H

#define ICM42688P_TAG "ICM42688P"

#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO
#define I2C_MASTER_BITRATE CONFIG_I2C_MASTER_BITRATE

// https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
#define ICM42688P_I2C_ADDRESS               0x80   // ToDo: Correct?

#define ICM42688P_W_CONFIG_RESET            0x1101
#define ICM42688P_W_CONFIG_ODR              0x5049
#define ICM42688P_W_CONFIG_MODE             0x4E02
#define ICM42688P_W_CONFIG_APEX             0x5602
#define ICM42688P_W_CONFIG_DMP              0x4B20
#define ICM42688P_W_CONFIG_DMP_INIT         0x4B40
#define ICM42688P_W_CONFIG_BANK             0x4E20

#define ICM42688P_W_PEDOMETER_ENABLE        0x5622
#define ICM42688P_W_PEDOMETER_DISABLE       0x5602
#define ICM42688P_R_PEDOMETER_READSTEPS     0x31

#define ICM42688P_R_ACCELEROMETER_READX     0x1F
#define ICM42688P_R_ACCELEROMETER_READY     0x21
#define ICM42688P_R_ACCELEROMETER_READZ     0x23

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} movement_t;
typedef struct {
    uint16_t steps;
    movement_t movement;
} measurement_t;

void ICM42688P_reset();
void configure_accelerometer();
measurement_t ICM42688P_read_all();
void ICM42688P_start_measurement(void);
void ICM42688P_stop_measurement(void);

#endif // ICM42688P_H