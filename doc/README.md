# ESP32 Pedometer
This project implements a pedometer using an ESP32 microcontroller. The pedometer reads data from an ICM-42688-P accelerometer via I2C, processes the data to detect steps, and displays the step count using an LED strip. The project also includes button controls to switch modes and start/stop measurements.

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
* Reads accelerometer data from ICM-42688-P via I2C
* Detects steps based on accelerometer data
* Read internal step count of ICM-42688-P
* Displays step count on an LED strip
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
git clone https://github.com/yourusername/esp32-pedometer.git
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
* I2C settings for the accelerometer
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
`configure_led()`: Configures the LED strip.
`configure_buttons()`: Configures the buttons and sets up interrupt handlers.
`configure_accelerometer()`: Initializes the I2C connection and configures the accelerometer.
`read_accelerometer_data()`: Reads data from the accelerometer and processes it to detect steps.
`draw_data()`: Displays the step count on the LED strip.
`app_main()`: The main application loop.

## License
This project is licensed under the MIT License. See the LICENSE file for details.