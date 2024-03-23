| Supported Targets | ESP32-C3 |
| ----------------- | -------- |

# AirQM_ESP32C3

Starts a FreeRTOS task to print "Hello World".

(See the README.md file in the upper level 'examples' directory for more information about examples.)

## Components used

|Item                       |Buy link                                                               |
|---------------------------|-----------------------------------------------------------------------|
| ESP32-C3 SuperMini        | [Aliexpress](https://www.aliexpress.com/item/1005006451672646.html)   |
| senseAir S8 004-0-0053    | [Aliexpress](https://www.aliexpress.com/item/1005005225848074.html)   |
| PMS5003                   | [Aliexpress](https://www.aliexpress.com/item/1005005967735332.html)   |
| SHT45                     | [Aliexpress](https://www.aliexpress.com/item/1005003742319869.html)   |
| SGP41                     | [Aliexpress](https://www.aliexpress.com/item/4000037083952.html)      |
| Oled Display              | [Aliexpress](https://www.aliexpress.com/item/32950307344.html)        |

## Pin connection matrix

```
| ESP32-C3 | OLED    | SGP41   | SHT45   | Senseair S8 | PMS5003  |
|----------|---------|---------|---------|-------------|----------|
| GND      | Pin GND | Pin GND | Pin GND | G0          | PIN2 GND |
| 3V3      | Pin 3V3 | Pin 3V3 | Pin 3V3 |             |          |
| 5V       |         |         |         | G+          | PIN1 VCC |
| GPIO_0   |         |         |         |             | PIN4 RX  |
| GPIO_1   |         |         |         |             | PIN5 TX  |
| GPIO_2   |         |         |         |             |          |
| GPIO_3   |         |         |         |             |          |
| GPIO_4   | Pin SCK |         |         |             |          |
| GPIO_5   | Pin DC  |         |         |             |          |
| GPIO_6   | Pin SDA |         |         |             |          |
| GPIO_7   | Pin CS  |         |         |             |          |
| GPIO_8   |         | Pin SDA | Pin SDA |             |          |
| GPIO_9   |         | Pin SCL | Pin SCL |             |          |
| GPIO_10  | Pin DC  |         |         |             |          |
| GPIO_20  |         |         |         | UART_TxD    |          |
| GPIO_21  |         |         |         | UART_RxD    |          |
```

## How to use example

Follow detailed instructions provided specifically for this example.

Select the instructions depending on Espressif chip installed on your development board:

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-S2 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)


## Folder contents

The project **AirQM_ESP32C3** contains one primary source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both).

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── pytest_main.py      Python script used for automated testing
├── components
│   ├── oled_display
│   │   ├── include
│   │   │   └── oled_display.h
│   │   ├── CMakeLists.txt
│   │   ├── fonts.c
│   │   └── oled_display.c
│   ├── senseair_s8
│   │   ├── include
│   │   │   └── senseair_s8.h
│   │   ├── CMakeLists.txt
│   │   └── senseair_s8.c
│   ├── sgp41
│   │   ├── include
│   │   │   ├── sensirion_gas_index_algorithm.h
│   │   │   └── sgp41.h
│   │   ├── CMakeLists.txt
│   │   ├── sensirion_gas_index_algorithm.c
│   │   └── sgp41.c
│   ├── sht45
│   │   ├── include
│   │   │   └── sht45.h
│   │   ├── CMakeLists.txt
│   │   └── sht45.c
├── main
│   ├── include
│   │   └── main.h
│   ├── CMakeLists.txt
│   └── main.c
├── esp32.svd
├── sdkconfig
└── README.md                  This is the file you are currently reading
```

For more information on structure and contents of ESP-IDF projects, please refer to Section [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) of the ESP-IDF Programming Guide.

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
