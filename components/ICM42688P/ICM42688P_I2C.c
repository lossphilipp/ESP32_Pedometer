#include <driver/i2c.h>

#include "ICM42688P.h"

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

i2c_port_t i2c_port = I2C_NUM_0;

static void initI2C(i2c_port_t i2c_num) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_BITRATE
    };

    i2c_param_config(i2c_num, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));

    ESP_LOGD(ICM42688P_TAG, "I2C connection initialized"); 
}

static void ICM42688P_writeRegister(uint16_t command) {
    uint8_t writeCmd[2] = {command >> 8, command & 0xFF};
    //ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50)));
    esp_err_t err = i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "Failed to write to register: %s", esp_err_to_name(err));
    }
}

static void ICM42688P_readRegister(uint8_t readBuffer[6]) {
    //ESP_ERROR_CHECK(i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50)));
    esp_err_t err = i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "Failed to read from register: %s", esp_err_to_name(err));
    }
}

void ICM42688P_reset() {
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_RESET);
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_ODR);
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_MODE);
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_APEX);
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_DMP);
    vTaskDelay(1 / portTICK_PERIOD_MS); // According to datasheet
    ICM42688P_writeRegister(ICM42688P_W_CONFIG_DMP_INIT);
    // ICM42688P_writeRegister(ICM42688P_W_CONFIG_BANK); // Would create interrupt on step
}

void configure_accelerometer() {
    initI2C(i2c_port);
    ICM42688P_reset();

    ESP_LOGD(ICM42688P_TAG, "Accelerometer configured");
}

uint16_t ICM42688P_read_steps() {
    uint8_t readBuffer[6];
    ICM42688P_writeRegister(ICM42688P_R_PEDOMETER_READSTEPS);
    ICM42688P_readRegister(readBuffer);
    return (readBuffer[0] << 8) | readBuffer[1];
}

uint16_t ICM42688P_read_movementX() {
    uint8_t readBuffer[6];
    ICM42688P_writeRegister(ICM42688P_R_ACCELEROMETER_READX);
    ICM42688P_readRegister(readBuffer);
    return (readBuffer[0] << 8) | readBuffer[1];
}

uint16_t ICM42688P_read_movementY() {
    uint8_t readBuffer[6];
    ICM42688P_writeRegister(ICM42688P_R_ACCELEROMETER_READY);
    ICM42688P_readRegister(readBuffer);
    return (readBuffer[0] << 8) | readBuffer[1];
}

uint16_t ICM42688P_read_movementZ() {
    uint8_t readBuffer[6];
    ICM42688P_writeRegister(ICM42688P_R_ACCELEROMETER_READZ);
    ICM42688P_readRegister(readBuffer);
    return (readBuffer[0] << 8) | readBuffer[1];
}

void ICM42688P_read_movement(measurement_t *measurement) {
    measurement->movement.x = ICM42688P_read_movementX();
    measurement->movement.y = ICM42688P_read_movementY();
    measurement->movement.z = ICM42688P_read_movementZ();
}

measurement_t ICM42688P_read_all() {
    measurement_t measurement;
    
    measurement.steps = ICM42688P_read_steps();
    ICM42688P_read_movement(&measurement);

    ESP_LOGD(ICM42688P_TAG, "\nSteps: %d\nX: %d\nY: %d\nZ: %d", measurement.steps, measurement.movement.x, measurement.movement.y, measurement.movement.z); 

    return measurement;
}

void ICM42688P_start_measurement() {
    ICM42688P_writeRegister(ICM42688P_W_PEDOMETER_ENABLE);
}

void ICM42688P_stop_measurement() {
    ICM42688P_writeRegister(ICM42688P_W_PEDOMETER_DISABLE);
}