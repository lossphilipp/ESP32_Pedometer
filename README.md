# ESP32 Pedometer
This project implements a pedometer using an ESP32 microcontroller. The pedometer reads data from an ICM-42688-P accelerometer, processes the data to detect steps, and displays the step count using an LED strip. The project also includes button controls to switch modes and start/stop measurements.

Per default the data is read via SPI, but there is also a (non-functional) implementation of I<sup>2</sup>C provided. To change this, the file `/components/ICM42688P/CMakeList.txt` has to be adjusted, by exchanging the value of "`SRCS`" from "`ICM42688P_SPI.c`" to "`ICM42688P_I2C.c`".

## Table of Contents
* Features
* Hardware Requirements
* Software Requirements
* Installation
* Configuration
* Usage
* Code Overview
* License

## Features
* Reads accelerometer data from ICM-42688-P
* Detects steps based on accelerometer data
* Read internal step count of ICM-42688-P
* Displays step count on an LED strip (red = manually calculated, green = red from pedometer)
* Button controls for switching modes and starting/stopping measurements

## Hardware Requirements
* ESP32 microcontroller
* ICM-42688-P accelerometer
* LED strip (compatible with ESP32)
* Two buttons for control

## Software Requirements
* ESP-IDF (Espressif IoT Development Framework)
* CMake
* Ninja build system

## Installation
1. Clone the repository:
``` bash
git clone https://github.com/lossphilipp/ESP32_Pedometer.git
cd esp32-pedometer
```

2. Set up the ESP-IDF environment:
``` bash
. $HOME/esp/esp-idf/export.sh
```

3. Configure the project:
``` bash
idf.py menuconfig
```

4. Build the project:
``` bash
idf.py build
```

5. Flash the project to the ESP32:
``` bash
idf.py flash
```

## Configuration
The project can be configured using the `menuconfig` tool provided by ESP-IDF. Key configuration options include:
* GPIO pins for the LED strip and buttons
* SPI/I2C settings for the accelerometer
* LED strip backend (RMT or SPI)

## Usage
1. Power on the ESP32.
2. The system will initialize and start in the default mode.
3. Use the left button to switch between modes:
    * Read steps from the accelerometer
    * Calculate steps based on movement data
    * Compare read and calculated steps
4. Use the right button to start/stop measurements.
5. The LED strip will display the step count in bindary based on the selected mode.

## Code Overview
* main.c: The main application code.
    * Initializes the system and configures peripherals.
    * Reads data from the accelerometer and processes it to detect steps.
    * Displays the step count on the LED strip.
    * Handles button inputs to switch modes and start/stop measurements.

### Key Functions
* `configure_led()`: Configures the LED strip.
* `configure_buttons()`: Configures the buttons.
* `switch_mode()`: Switches between different output modes.
* `toggle_measurement()`: Starts or stops the measurement.
* `handle_button_press()`: Handles button press events.
* `calculate_binary()`: Converts a number to a binary array.
* `draw_binary()`: Draws a binary array on the LED strip.
* `draw_data()`: Draws the step count data on the LED strip.
* `calculate_average_movement()`: Calculates the average movement of the last few measurements
* `detect_step()`: Detects a step based on movement data.
* `read_accelerometer_data()`: Reads data from the accelerometer.
* `reset_steps()`: Resets the step count.
* `check_reset_initiated()`: Initiates a reset if both buttons are pressed for longer than a second.
* `configure_system()`: Configures the entire system.
* `app_main()`: The main application entry point.

## License
This project is licensed under the MIT License. See the LICENSE file for details.