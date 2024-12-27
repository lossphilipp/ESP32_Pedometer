#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
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
#define ICM42688P_I2C_ADDRESS 0x80      // ToDo: Correct?

#define ICM42688P_W_CONFIG_RESET 0x1101
#define ICM42688P_W_CONFIG_ODR 0x5049
#define ICM42688P_W_CONFIG_MODE 0x4E02
#define ICM42688P_W_CONFIG_APEX 0x5602
#define ICM42688P_W_CONFIG_DMP 0x4B20
#define ICM42688P_W_CONFIG_DMP_INIT 0x4B40
#define ICM42688P_W_CONFIG_BANK 0x4E20

#define ICM42688P_W_PEDOMETER_ENABLE 0x5622
#define ICM42688P_W_PEDOMETER_DISABLE 0x5602
#define ICM42688P_R_PEDOMETER_READSTEPS 0x31

#define ICM42688P_R_ACCELEROMETER_READX 0x1F
#define ICM42688P_R_ACCELEROMETER_READY 0x21
#define ICM42688P_R_ACCELEROMETER_READZ 0x23

#define OUTPUT_TYPE_READ 0
#define OUTPUT_TYPE_CALCULATED 1
#define OUTPUT_TYPE_COMPARISON 2

#define STEP_VECTOR_MAGNITUDE_THRESHOLD 1000
#define STEP_SAMPLE_ERROR_TRESHOLD 5      // Minimum number of consecutive measurements indicating a direction change
#define STEP_CALCULATION_BUFFER_SIZE 25*2 // 25 Hz * 2 seconds

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} movement_t;
typedef struct {
    uint16_t steps;
    movement_t movement;
} measurement_t;


uint16_t calculated_steps = 0;
uint8_t output_type = OUTPUT_TYPE_COMPARISON;
bool measurement_running = false;
volatile uint8_t pressed_button = 0;

volatile measurement_t current_measurement = {
    .steps = 0,
    .movement.x = 0,
    .movement.y = 0,
    .movement.z = 0
};
movement_t calculation_buffer[STEP_CALCULATION_BUFFER_SIZE]; // Ringbuffer
uint8_t calculation_buffer_index = 0;

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

void initI2C(i2c_port_t i2c_num) {
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

void ICM42688P_writeRegister(uint16_t command) {
    uint8_t writeCmd[2] = {command >> 8, command & 0xFF};
    //ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50)));
    esp_err_t err = i2c_master_write_to_device(i2c_port, ICM42688P_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to write to register: %s", esp_err_to_name(err));
    }
}

void ICM42688P_readRegister(uint8_t readBuffer[6]) {
    //ESP_ERROR_CHECK(i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50)));
    esp_err_t err = i2c_master_read_from_device(i2c_port, ICM42688P_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read from register: %s", esp_err_to_name(err));
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

    ESP_LOGD("SENSOR", "Accelerometer configured");
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

    ESP_LOGD("MEASUREMENT", "Steps: %d\nX: %d\nY: %d\nZ: %d", measurement.steps, measurement.movement.x, measurement.movement.y, measurement.movement.z); 

    return measurement;
}

void ICM42688P_start_measurement() {
    ICM42688P_writeRegister(ICM42688P_W_PEDOMETER_ENABLE);
}

void ICM42688P_stop_measurement() {
    ICM42688P_writeRegister(ICM42688P_W_PEDOMETER_DISABLE);
}
#endif

#if defined(BUTTON_GPIO_LEFT) || defined(BUTTON_GPIO_RIGHT)

void switch_mode() {
    output_type = (output_type + 1) % 3;
    ESP_LOGI("CONFIGURATION", "Switching mode to %d", output_type);
}

void toggle_measurement() {
    if (measurement_running) {
        ICM42688P_stop_measurement();
        ESP_LOGI("MEASUREMENT", "Measurement stoped");
    } else {
        ICM42688P_start_measurement();
        ESP_LOGI("MEASUREMENT", "Measurement started"); 
    }
    measurement_running = !measurement_running;
}

void handle_button_press() {
    if (pressed_button == BUTTON_GPIO_LEFT) {
        switch_mode();
    } else if (pressed_button == BUTTON_GPIO_RIGHT) {
        toggle_measurement();
    }
    pressed_button = 0;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint8_t gpio_num = (uint32_t) arg;
    pressed_button = gpio_num;
    ESP_LOGD("BUTTON", "Interrupt of button %d triggered", gpio_num); 
}

void configure_buttons() {
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

bool is_vector_above_threshold(movement_t movement) {
    float_t magnitude = sqrt(movement.x * movement.x + movement.y * movement.y + movement.z * movement.z);
    return magnitude > STEP_VECTOR_MAGNITUDE_THRESHOLD;
}

int32_t calculate_dot_product(movement_t movement1, movement_t movement2) {
    return movement1.x * movement2.x + movement1.y * movement2.y + movement1.z * movement2.z;
}

bool is_vector_direction_same(movement_t movement1, movement_t movement2) {
    return calculate_dot_product(movement1, movement2) > 0.5; // 1 = same direction, 0 = orthogonal, -1 = opposite direction
}

bool is_vector_direction_opposite(movement_t movement1, movement_t movement2) {
    return calculate_dot_product(movement1, movement2) < -0.5; // 1 = same direction, 0 = orthogonal, -1 = opposite direction
}

// ToDo: Review logic and refactor (method way to big)
bool detect_step(movement_t current_movement) {
    if (!is_vector_above_threshold(current_movement)) {
        return false;
    }

    // Check the last measurements for the same direction
    int8_t same_direction_count = 0;
    for (int8_t i = 0; i < STEP_SAMPLE_ERROR_TRESHOLD; i++) {
        int8_t index = (calculation_buffer_index - 1 - i + STEP_CALCULATION_BUFFER_SIZE) % STEP_CALCULATION_BUFFER_SIZE;
        movement_t previous_movement = calculation_buffer[index];

        if (is_vector_direction_same(current_movement, previous_movement)) {
            same_direction_count++;
        } else {
            same_direction_count = 0;
            break;
        }
    }
    if (same_direction_count < STEP_SAMPLE_ERROR_TRESHOLD) {
        return false;
    }

    // Check the previous measurements for the opposite direction
    int8_t opposite_direction_count = 0;
    for (int8_t i = STEP_SAMPLE_ERROR_TRESHOLD; i < 2 * STEP_SAMPLE_ERROR_TRESHOLD; i++) {
        int8_t index = (calculation_buffer_index - 1 - i + STEP_CALCULATION_BUFFER_SIZE) % STEP_CALCULATION_BUFFER_SIZE;
        movement_t previous_movement = calculation_buffer[index];

        if (is_vector_direction_opposite(current_movement, previous_movement)) {
            opposite_direction_count++;
        } else {
            opposite_direction_count = 0;
            break;
        }
    }

    return opposite_direction_count >= STEP_SAMPLE_ERROR_TRESHOLD;
}

void calc_steps_from_movement() {
    movement_t current_movement = current_measurement.movement;

    if (detect_step(current_movement)) {
        calculated_steps++;
    }

    calculation_buffer[calculation_buffer_index] = current_movement;
    calculation_buffer_index = (calculation_buffer_index + 1) % STEP_CALCULATION_BUFFER_SIZE;
}

void read_accelerometer_data() {
    current_measurement = ICM42688P_read_all();
    calc_steps_from_movement();
}

void reset_accelerometer_steps() {
    ICM42688P_reset();
}

uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void calculate_binary(uint16_t number, uint8_t binary_array[10]) {
    for (int8_t i = 9; i >= 0; i--) {
        binary_array[i] = number % 2;
        number /= 2;
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

void draw_data() {
    uint8_t steps_read_binary[10];
    uint8_t steps_calculated_binary[10];

    if (output_type != OUTPUT_TYPE_READ) {
        calculate_binary(calculated_steps, steps_calculated_binary);
    }
    if (output_type != OUTPUT_TYPE_CALCULATED){
        calculate_binary(current_measurement.steps, steps_read_binary);
    }

    led_strip_clear(led_strip);

    if (output_type == OUTPUT_TYPE_READ) {
        draw_binary(steps_calculated_binary, 0, 1);
    } else if (output_type == OUTPUT_TYPE_CALCULATED) {
        draw_binary(steps_read_binary, 3, 4);
    } else if (output_type == OUTPUT_TYPE_COMPARISON) {
        draw_binary(steps_calculated_binary, 0, 1);
        draw_binary(steps_read_binary, 3, 4);
    } else {
        ESP_LOGE("DRAW", "Invalid output type");
    }

    led_strip_refresh(led_strip);
}

void reset_steps() {
    // ToDo: Call this when both buttons are pressed
    calculated_steps = 0;
    reset_accelerometer_steps();
}

void configure_system() {
    configure_led();
    configure_buttons();
    configure_accelerometer();

    // Init time according to datasheet
    vTaskDelay(50 / portTICK_PERIOD_MS);

    ESP_LOGI("CONFIGURATION", "Everything configured, program start...");
}

void app_main(void) {
    configure_system();

    while (true) {
        if (pressed_button != 0) {
            handle_button_press();
        }
        
        if (measurement_running)
        {
            read_accelerometer_data();
            draw_data();
        }
        
        // 25 Hz = Pedometer Low Power Sampling
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}