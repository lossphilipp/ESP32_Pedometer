#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "ICM42688P.h"

#define ICM42688P_SPI_ADDRESS     0x80
#define ICM42688P_SPI_MSB_CLEAR   0x7F

spi_device_interface_config_t spiDeviceConfig = { 0 };
spi_device_handle_t spiDeviceHandle;

void initSPI(spi_host_device_t spiHost) {
    printf("init SPI\n");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 4,
        .miso_io_num = 7,
        .sclk_io_num = 10,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 0 // DMA default
    };
    esp_err_t res = spi_bus_initialize(spiHost, &bus_cfg, SPI_DMA_CH_AUTO);
    if (res != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "spi_bus_initialize() FAILED: %d!", res);
        return;
    } else {
        ESP_LOGD(ICM42688P_TAG, "spi_bus_initialize() OK");
    }

    spiDeviceConfig.command_bits = 0;
    spiDeviceConfig.address_bits = 0;
    spiDeviceConfig.dummy_bits = 0;
    spiDeviceConfig.duty_cycle_pos = 128;
    spiDeviceConfig.mode = 0; // CPOL = 0, CPHA = 0
    spiDeviceConfig.clock_source = SPI_CLK_SRC_DEFAULT;
    spiDeviceConfig.clock_speed_hz = 500000; // 500 kbit/s
    spiDeviceConfig.spics_io_num = GPIO_NUM_1;
    spiDeviceConfig.flags = 0; //SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    spiDeviceConfig.queue_size = 3;
    spiDeviceConfig.cs_ena_posttrans = 3;
    spi_bus_add_device(spiHost, &spiDeviceConfig, &spiDeviceHandle);
}

bool read_who_am_i(){
    spi_transaction_t spiTransaction = { 0 };
    uint8_t cmd[16];
    cmd[0] = 0x75 | 0x80; // WHO AM I, Read
    cmd[1] = 0xFF; // second byte, for response transmit
    uint8_t rcv[16];
    rcv[0] = 0xCC;
    rcv[1] = 0xCC;
    spiTransaction.length = 8 * 2; // 8 bit send, 8 bit receive, only the register
    spiTransaction.rxlength = 0;
    spiTransaction.flags = 0; // SPI_TRANS_CS_KEEP_ACTIVE;
    spiTransaction.tx_buffer = &cmd;
    spiTransaction.rx_buffer = &rcv;

    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);

    if (ret == ESP_OK) {
        ESP_LOGD(ICM42688P_TAG, "got data from ICM42688: %X %X", rcv[0], rcv[1]);
    }

    if (rcv[1] == 0x47) { // see datasheet
        ESP_LOGD(ICM42688P_TAG, "gyro responds, ID ok");
        return true;
    } else {
        ESP_LOGD(ICM42688P_TAG, "gyro responds, but wrong ID! ID: %X", rcv[1]);
    }

    return false;
}

uint8_t ICM42688P_read_single_reg(uint8_t reg) {
    spi_transaction_t spiTransaction = { 0 };

    uint8_t cmd[2];
    cmd[0] = reg | ICM42688P_SPI_ADDRESS;
    cmd[1] = 0xFF; // second byte, for response transmit

    uint8_t rcv[2];

    rcv[0] = 0xCC;
    rcv[1] = 0xCC;

    spiTransaction.length = 8 * 2; // 8 bit send, 8 bit receive, only the register
    spiTransaction.rxlength = 0;
    spiTransaction.flags = 0; // SPI_TRANS_CS_KEEP_ACTIVE;
    spiTransaction.tx_buffer = &cmd;
    spiTransaction.rx_buffer = &rcv;

    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "SPI transmission failed: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGV(ICM42688P_TAG, "got data from ICM42688: %X %X", rcv[0], rcv[1]);
    return rcv[1];
}

esp_err_t ICM42688P_read_data(uint8_t start_reg, uint8_t* pData, size_t length) {
    spi_transaction_t spiTransaction = { 0 };

    uint8_t cmd[2];
    cmd[0] = start_reg | ICM42688P_SPI_ADDRESS;
    cmd[1] = 0xFF; // second byte, for response transmit

    spiTransaction.length = 8 * (1 + length); // 8 bits for command + 8 bits for each byte to read
    spiTransaction.rxlength = 0; // 0 defaults to the length parameter
    spiTransaction.flags = 0; // SPI_TRANS_CS_KEEP_ACTIVE;
    spiTransaction.tx_buffer = &cmd;
    spiTransaction.rx_buffer = pData;

    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "SPI transmission failed: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGV(ICM42688P_TAG, "got data from ICM42688! start reg: %X", start_reg);
    return ESP_OK;
}

esp_err_t ICM42688P_write_data(uint16_t reg_value) {
    spi_transaction_t spiTransaction = { 0 };

    uint8_t cmd[2];
    cmd[0] = (reg_value >> 8) & ICM42688P_SPI_MSB_CLEAR;
    cmd[1] = reg_value & 0xFF;

    spiTransaction.length = 8 * 2;         // Two bytes: 1 address, 1 data
    spiTransaction.tx_buffer = cmd;        // Send the command and data buffer
    spiTransaction.rx_buffer = NULL;       // No data expected to be received

    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret != ESP_OK) {
        ESP_LOGE(ICM42688P_TAG, "SPI transmission failed: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGD(ICM42688P_TAG, "Wrote 0x%02X to register 0x%02X", (reg_value & 0xFF), (reg_value >> 8));
    return ESP_OK;
}

void ICM42688P_reset() {
    ICM42688P_write_data(ICM42688P_CONFIG_RESET);
    ICM42688P_write_data(ICM42688P_CONFIG_ODR);
    ICM42688P_write_data(ICM42688P_CONFIG_MODE);
    ICM42688P_write_data(ICM42688P_CONFIG_APEX);
    ICM42688P_write_data(ICM42688P_CONFIG_GYRO);
    ICM42688P_write_data(ICM42688P_CONFIG_DMP_RESET);
    vTaskDelay(1 / portTICK_PERIOD_MS); // According to datasheet
    ICM42688P_write_data(ICM42688P_CONFIG_DMP_INIT);
    // ICM42688P_write_data(ICM42688P_CONFIG_BANK); // Would create interrupt on step
}

void configure_accelerometer() {
    initSPI(SPI2_HOST);
    ICM42688P_reset();

    ESP_LOGD(ICM42688P_TAG, "Accelerometer configured, trying to read WHO AM I...");

    bool gyroscopeWorking = read_who_am_i();
    if (!gyroscopeWorking) {
        ESP_LOGE(ICM42688P_TAG, "Gyroscope configuration failed, exiting...");
        return;
    }

    ESP_LOGI(ICM42688P_TAG, "Accelerometer successfully configured!");
}

uint16_t ICM42688P_read_steps() {
    uint8_t data[2];
    ICM42688P_read_data(ICM42688P_STEPS_OUT_L, data, 2);

    return ((data[0] << 8) | data[1]);
}

// ToDo: This does suddenly not work anymore! WTF?
void ICM42688P_read_movement(measurement_t *measurement) {
    uint8_t upper_x = ICM42688P_read_single_reg(ICM42688P_ACCEL_XOUT_H);
    uint8_t lower_x = ICM42688P_read_single_reg(ICM42688P_ACCEL_XOUT_L);
    measurement->movement.x = ((upper_x << 8) | lower_x);

    uint8_t upper_y = ICM42688P_read_single_reg(ICM42688P_ACCEL_YOUT_H);
    uint8_t lower_y = ICM42688P_read_single_reg(ICM42688P_ACCEL_YOUT_L);
    measurement->movement.y = ((upper_y << 8) | lower_y);

    uint8_t upper_z = ICM42688P_read_single_reg(ICM42688P_ACCEL_ZOUT_H);
    uint8_t lower_z = ICM42688P_read_single_reg(ICM42688P_ACCEL_ZOUT_L);
    measurement->movement.z = ((upper_z << 8) | lower_z);
}

// ToDo: This would be the more elegant solution bit it emits weird values.
// void ICM42688P_read_movement(measurement_t *measurement) {
//     uint8_t data[6];
//     ICM42688P_read_data(ICM42688P_ACCEL_XOUT_H, data, 6);

//     uint16_t x = ((data[0] << 8) | data[1]);
//     uint16_t y = ((data[2] << 8) | data[3]);
//     uint16_t z = ((data[4] << 8) | data[5]);

//     measurement->movement.x = x;
//     measurement->movement.y = y;
//     measurement->movement.z = z;
// }

measurement_t ICM42688P_read_all() {
    measurement_t measurement;
    
    measurement.steps = ICM42688P_read_steps();
    ICM42688P_read_movement(&measurement);

    ESP_LOGD(ICM42688P_TAG, "Read the following data:\nSteps: %d\nX: %d\nY: %d\nZ: %d", measurement.steps, measurement.movement.x, measurement.movement.y, measurement.movement.z); 

    return measurement;
}

void ICM42688P_start_measurement() {
    ICM42688P_write_data(ICM42688P_PEDOMETER_ENABLE);
}

void ICM42688P_stop_measurement() {
    ICM42688P_write_data(ICM42688P_PEDOMETER_DISABLE);
}