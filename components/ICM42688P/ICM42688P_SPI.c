#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "ICM42688P.h"

#define ICM42688P_SPI_ADDRESS               0x80
#define ICM42688P_SPI_MSB_CLEAR             0x7F

#define ICM42688P_ACCEL_XOUT_H 0x1F | ICM42688P_SPI_ADDRESS
#define ICM42688P_ACCEL_XOUT_L 0x20 | ICM42688P_SPI_ADDRESS
#define ICM42688P_ACCEL_YOUT_H 0x21 | ICM42688P_SPI_ADDRESS
#define ICM42688P_ACCEL_YOUT_L 0x22 | ICM42688P_SPI_ADDRESS
#define ICM42688P_ACCEL_ZOUT_H 0x23 | ICM42688P_SPI_ADDRESS
#define ICM42688P_ACCEL_ZOUT_L 0x24 | ICM42688P_SPI_ADDRESS

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
		printf("spi_bus_initialize() FAILED\n");
		return;
	} else {
		printf("spi_bus_initialize() ok\n");
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

void ICM42688P_reset() {

}

void configure_accelerometer() {
    initSPI(SPI2_HOST);
    ICM42688P_reset();

    ESP_LOGD(ICM42688P_TAG, "Accelerometer configured");
}

uint16_t ICM42688P_read_steps() {
    return 0;
}

uint16_t ICM42688P_read_movementX() {
    return 0;
}

uint16_t ICM42688P_read_movementY() {
    return 0;
}

uint16_t ICM42688P_read_movementZ() {
    return 0;
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

}

void ICM42688P_stop_measurement() {

}