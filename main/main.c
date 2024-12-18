#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include <driver/i2c.h>

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_PERIOD CONFIG_BLINK_PERIOD

#define BUTTON_GPIO_LEFT CONFIG_BUTTON_GPIO_LEFT
#define BUTTON_GPIO_RIGHT CONFIG_BUTTON_GPIO_RIGHT

#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO
#define I2C_MASTER_BITRATE CONFIG_I2C_MASTER_BITRATE

// https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
// ToDo: Adjust to correct values
#define ICM42688P_I2C_ADDRESS 0x00
#define ICM42688P_CMD_WAKEUP 0x0000
#define ICM42688P_CMD_SLEEP 0x0000

#define OUTPUT_TYPE_READ 0
#define OUTPUT_TYPE_CALCULATED 1
#define OUTPUT_TYPE_COMPARISON 2

typedef struct {
    uint8_t steps;
    uint8_t movement;
} measurement_t; 

uint8_t output_type = OUTPUT_TYPE_COMPARISON; // magic number
measurement_t current_measurement = {
    .steps = 0,
    .movement = 0
};
uint16_t calculated_steps = 0;
bool measurement_running = false;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 25,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);

    ESP_LOGD("LED", "LED configured"); 
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    ESP_LOGD("LED", "LED configured"); 
}

#else
#error "unsupported LED type"
#endif

#ifdef ICM42688P_I2C_ADDRESS

i2c_port_t i2c_port = I2C_NUM_0;

void initI2C(i2c_port_t i2c_num)
{
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

    ESP_LOGD("I2C", "I2C connection initialized"); 
}

void configure_accelerometer()
{
    initI2C(i2c_port);

    ESP_LOGD("SENSOR", "Accelerometer configured"); 
}

void ICM42688P_writeRegister(uint16_t command)
{
    uint8_t writeCmd[2] = {command >> 8, command & 0xFF};
    //ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50)));
    i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50));
}

void ICM42688P_readRegister(uint8_t readBuffer[6])
{
    //ESP_ERROR_CHECK(i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50)));
    i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50));
}

uint8_t ICM42688P_CalculateChecksum(uint16_t readValue)
{
    // ToDo: Implement checksum calculation
    return 0;
}

uint8_t ICM42688P_read_steps(){
    // ToDo: Implement reading step register
    return 0;
}

uint8_t ICM42688P_read_movement(){
    // ToDo: Implement reading movement register
    return 0;
}

uint8_t ICM42688P_start_measurement(){
    // ToDo: Implement start measurement
    return 0;
}

uint8_t ICM42688P_stop_measurement(){
    // ToDo: Implement start measurement
    return 0;
}

uint8_t ICM42688P_reset_steps(){
    // ToDo: Implement reset step register
    return 0;
}

measurement_t ICM42688P_read_all()
{
    uint8_t steps = ICM42688P_read_steps();
    uint8_t movement = ICM42688P_read_movement();

    measurement_t measurement = {
        .steps = steps,
        .movement = movement
    };

    return measurement;
}
#endif

#if defined(BUTTON_GPIO_LEFT) || defined(BUTTON_GPIO_RIGHT)

void switch_mode()
{
    output_type = (output_type + 1) % 3;
    ESP_LOGI("CONFIGURATION", "Switching mode to %d", output_type);
}

void toggle_measurement() {
    if (measurement_running) {
        ICM42688P_stop_measurement();
    } else {
        ICM42688P_start_measurement();
    }
    measurement_running = !measurement_running;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_num == BUTTON_GPIO_LEFT) {
        switch_mode();
    } else if (gpio_num == BUTTON_GPIO_RIGHT) {
        toggle_measurement();
    }
}

void configure_buttons()
{
    gpio_config_t gpioConfigIn = {
        .pin_bit_mask = (1 << BUTTON_GPIO_LEFT) | (1 << BUTTON_GPIO_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&gpioConfigIn);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO_LEFT, gpio_isr_handler, (void*) BUTTON_GPIO_LEFT);
    gpio_isr_handler_add(BUTTON_GPIO_RIGHT, gpio_isr_handler, (void*) BUTTON_GPIO_RIGHT);

    ESP_LOGD("CONFIGURATION", "Buttons configured"); 
}

#endif

// ######################### Program #########################

void read_accelerometer_data()
{
    current_measurement = ICM42688P_read_all();
}

void reset_accelerometer_steps()
{
    ICM42688P_reset_steps();
}

void calc_steps_from_movement() {
    // ToDo: Implement calculation of steps. We most likely need to save multiple measurements and calculate the difference
    calculated_steps = 0;
}

uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void calculate_binary(float number, uint8_t binary_array[10]) {
    int integer_part = (int)number;
    float fractional_part = number - integer_part;

    for (int8_t i = 4; i >= 0; i--) {
        binary_array[i] = integer_part % 2;
        integer_part /= 2;
    }

    for (int8_t i = 5; i < 10; i++) {
        fractional_part *= 2;
        if (fractional_part >= 1.0) {
            binary_array[i] = 1;
            fractional_part -= 1.0;
        } else {
            binary_array[i] = 0;
        }
    }
}

void draw_binary(uint8_t binary_array[10], int rowLower, int rowUpper) {
    if (rowLower < 0 || rowLower > 4 || rowUpper < 0 || rowUpper > 4) {
        ESP_LOGE("DRAW", "rowLower and rowUpper must be between 0 and 4");
    }
    if (rowLower > rowUpper) {
        ESP_LOGE("DRAW", "rowLower must be smaller than rowUpper");
    }

    for (int8_t i = (rowLower * 5); i < (rowUpper * 5); i++) {
        if (binary_array[i] == 1) {
            led_strip_set_pixel(led_strip, i, 255, 255, 255);
        } else {
            led_strip_set_pixel(led_strip, i, 0, 0, 0);
        }
    }
}

void draw_data()
{
    uint8_t steps_read_binary[10];
    uint8_t steps_calculated_binary[10];

    if (output_type != OUTPUT_TYPE_READ) {
        calc_steps_from_movement();
        calculate_binary(current_measurement.movement, steps_calculated_binary);
    }
    if (output_type != OUTPUT_TYPE_CALCULATED){
        calculate_binary(current_measurement.steps, steps_read_binary);
    }

    led_strip_clear(led_strip);

    if (output_type == OUTPUT_TYPE_READ) {
        draw_binary(steps_calculated_binary, 0, 4);
    } else if (output_type == OUTPUT_TYPE_CALCULATED) {
        draw_binary(steps_read_binary, 0, 4);
    } else if (output_type == OUTPUT_TYPE_COMPARISON) {
        draw_binary(steps_calculated_binary, 0, 1);
        draw_binary(steps_read_binary, 3, 4);
    } else {
        ESP_LOGE("DRAW", "Invalid output type");
    }

    led_strip_refresh(led_strip);
}

void reset_steps()
{
    // ToDo: Call this when both buttons are pressed
    calculated_steps = 0;
    reset_accelerometer_steps();
}

void app_main(void)
{
    configure_led();
    configure_buttons();
    configure_accelerometer();

    ESP_LOGI("CONFIGURATION", "Everything configured, program start...");

    while (measurement_running)
    {
        read_accelerometer_data();
        draw_data();
        // 25 Hz = Pedometer Low Power Sampling
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}