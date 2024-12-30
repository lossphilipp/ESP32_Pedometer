#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"

#ifndef ICM42688P_H
#define ICM42688P_H

#define ICM42688P_TAG "ICM42688P"

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