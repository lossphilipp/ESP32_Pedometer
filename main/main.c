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

#include "ICM42688P.h"
#include "ringbuffer.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_PERIOD CONFIG_BLINK_PERIOD

#define BUTTON_GPIO_LEFT CONFIG_BUTTON_GPIO_LEFT
#define BUTTON_GPIO_RIGHT CONFIG_BUTTON_GPIO_RIGHT

#define OUTPUT_TYPE_READ 0
#define OUTPUT_TYPE_CALCULATED 1
#define OUTPUT_TYPE_COMPARISON 2

#define STEP_VECTOR_MAGNITUDE_THRESHOLD 1000
#define STEP_SAMPLE_ERROR_TRESHOLD 5      // Minimum number of consecutive measurements indicating a direction change
#define STEP_CALCULATION_BUFFER_SIZE 25*2 // 25 Hz * 2 seconds

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

RingbufferHandle calculation_buffer;

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

#if defined(BUTTON_GPIO_LEFT) || defined(BUTTON_GPIO_RIGHT)
// Trying to do this stuff within the interrupt always leads to a kernel panic

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
        ESP_LOGD("BUTTON", "Handling of left button triggered");
        switch_mode();
    } else if (pressed_button == BUTTON_GPIO_RIGHT) {
        ESP_LOGD("BUTTON", "Handling of right button triggered");
        toggle_measurement();
    } else {
        ESP_LOGE("BUTTON", "Invalid GPIO number of pressed button! Number: %d", pressed_button);
    }
    pressed_button = 0;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint8_t gpio_num = (uint32_t) arg;
    pressed_button = gpio_num;
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

// ToDo: Review logic and refactor (method way to big & not working!)
bool detect_step(movement_t current_movement) {
    if (!is_vector_above_threshold(current_movement)) {
        return false;
    }

    // Check the last measurements for the same direction
    int8_t same_direction_count = 0;
    for (int8_t i = 0; i < STEP_SAMPLE_ERROR_TRESHOLD; i++) {
        movement_t previous_movement;
        if (!ringbuffer_get(calculation_buffer, &previous_movement, -1 - i)) {
            break;
        }

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
        movement_t previous_movement;
        if (!ringbuffer_get(calculation_buffer, &previous_movement, -1 - i)) {
            break;
        }

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

    ringbuffer_add(calculation_buffer, &current_movement);
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

void draw_binary(uint8_t binary_array[10], uint8_t rowLower, uint8_t rowUpper, uint8_t color[3]) {
    if (rowLower > 4 || rowUpper > 4) {
        ESP_LOGE("DRAW", "rowLower and rowUpper must be between 0 and 4");
        return;
    }
    if (rowLower > rowUpper) {
        ESP_LOGE("DRAW", "rowLower must be smaller than rowUpper");
        return;
    }

    char log_message[128];
    int offset = snprintf(log_message, sizeof(log_message), "Drawing binary array from row %d to %d: ", rowLower, rowUpper);
    for (int i = 0; i < 10; i++) {
        offset += snprintf(log_message + offset, sizeof(log_message) - offset, "%d", binary_array[i]);
    }
    ESP_LOGD("DRAW", "%s", log_message);

    for (int8_t i = rowLower; i <= rowUpper; i++) {
        for (int8_t j = 0; j < 5; j++) {
            int8_t led_index = i * 5 + j;
            int8_t array_index = led_index - rowLower * 5;
            if (binary_array[array_index] == 1) {
                led_strip_set_pixel(led_strip, led_index, color[0], color[1], color[2]);
            } else {
                led_strip_set_pixel(led_strip, led_index, 0, 0, 0);
            }
        }
    }
}

void draw_calculated_steps() {
    uint8_t steps_calculated_binary[10];
    calculate_binary(calculated_steps, steps_calculated_binary);
    uint8_t color[3] = {50, 0, 0}; // Red
    draw_binary(steps_calculated_binary, 0, 1, color);
}

void draw_read_steps() {
    uint8_t steps_read_binary[10];
    calculate_binary(current_measurement.steps, steps_read_binary);
    uint8_t color[3] = {0, 50, 0}; // Green
    draw_binary(steps_read_binary, 3, 4, color);
}

void draw_data() {
    //led_strip_clear(led_strip);

    if (output_type == OUTPUT_TYPE_READ) {
        draw_read_steps();
    } else if (output_type == OUTPUT_TYPE_CALCULATED) {
        draw_calculated_steps();
    } else if (output_type == OUTPUT_TYPE_COMPARISON) {
        draw_read_steps();
        draw_calculated_steps();
    } else {
        ESP_LOGE("DRAW", "Invalid output type");
    }

    led_strip_refresh(led_strip);
}

void draw_separator() {
    for (int8_t i = 10; i < 15; i++) {
        led_strip_set_pixel(led_strip, i, 50, 50, 50);
    }
    led_strip_refresh(led_strip);
}

void reset_steps() {
    calculated_steps = 0;
    reset_accelerometer_steps();
}

void check_reset_initiated() {
    uint32_t left = gpio_get_level(BUTTON_GPIO_LEFT);
    uint32_t right = gpio_get_level(BUTTON_GPIO_RIGHT);

    if (left == 0 && right == 0) {
        bool reset_initiated = true;
        for (uint8_t i = 0; i < 5; i++) {
            vTaskDelay(250 / portTICK_PERIOD_MS);
            left = gpio_get_level(BUTTON_GPIO_LEFT);
            right = gpio_get_level(BUTTON_GPIO_RIGHT);
            if (left != 0 || right != 0) {
                reset_initiated = false;
                break;
            }
        }

        if (reset_initiated) {
            ESP_LOGI("RESET", "Reset initiated");
            reset_steps();
            // ToDo: Create visual feedback of reset and att delay
        } else {
            ESP_LOGI("RESET", "Reset canceled");
        }
    }
}

void configure_system() {
    calculation_buffer = ringbuffer_create(STEP_CALCULATION_BUFFER_SIZE, sizeof(movement_t));
    if (calculation_buffer == RINGBUFFER_ERROR_OUTOFMEMORY) {
        ESP_LOGE(RINGBUFFER_TAG, "Could not allocate memory for calculation buffer");
    }

    configure_led();
    configure_buttons();
    configure_accelerometer();

    // Init time according to datasheet
    vTaskDelay(50 / portTICK_PERIOD_MS);

    draw_separator();
    ESP_LOGI("CONFIGURATION", "Everything configured, program start...");
}

void app_main(void) {
    configure_system();

    uint8_t run_counter = 0;
    while (true) {
        check_reset_initiated();
        
        if (pressed_button != 0) {
            handle_button_press();
        }
        
        if (measurement_running)
        {
            read_accelerometer_data();
            run_counter++;

            // Update the drawing every 200ms to save resources
            if (run_counter % 5 == 0) {
                draw_data();
            }
        }
        
        // 25 Hz = Pedometer Low Power Sampling
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}