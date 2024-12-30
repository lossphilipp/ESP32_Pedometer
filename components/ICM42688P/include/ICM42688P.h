#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"

#ifndef ICM42688P_H
#define ICM42688P_H

#define ICM42688P_TAG "ICM42688P"

// https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
#define ICM42688P_CONFIG_RESET            0x1101
#define ICM42688P_CONFIG_ODR              0x5049
#define ICM42688P_CONFIG_MODE             0x4E02
#define ICM42688P_CONFIG_APEX             0x5602
#define ICM42688P_CONFIG_DMP_RESET        0x4B20
#define ICM42688P_CONFIG_DMP_INIT         0x4B40
#define ICM42688P_CONFIG_GYRO             0x5000
#define ICM42688P_CONFIG_BANK             0x4E20

#define ICM42688P_PEDOMETER_ENABLE        0x5622
#define ICM42688P_PEDOMETER_DISABLE       0x5602

#define ICM42688P_ACCEL_XOUT_H    0x1F
#define ICM42688P_ACCEL_XOUT_L    0x20
#define ICM42688P_ACCEL_YOUT_H    0x21
#define ICM42688P_ACCEL_YOUT_L    0x22
#define ICM42688P_ACCEL_ZOUT_H    0x23
#define ICM42688P_ACCEL_ZOUT_L    0x24
#define ICM42688P_STEPS_OUT_L     0x31
#define ICM42688P_STEPS_OUT_H     0x32

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