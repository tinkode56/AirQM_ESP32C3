| Supported Targets | ESP32-C3 |
| ----------------- | -------- |

# AirQM_ESP32C3

Air quality monitor using ESP32-C3 and sensors to measure temperature, relative humidity, dust particles, CO2, VOC and NOx.

## Components used

|Item                       |Buy link                                                               |
|---------------------------|-----------------------------------------------------------------------|
| ESP32-C3 SuperMini        | [Aliexpress](https://www.aliexpress.com/item/1005006451672646.html)   |
| senseAir S8 004-0-0053    | [Aliexpress](https://www.aliexpress.com/item/1005005225848074.html)   |
| PMS5003                   | [Aliexpress](https://www.aliexpress.com/item/1005005967735332.html)   |
| SHT45                     | [Aliexpress](https://www.aliexpress.com/item/1005003742319869.html)   |
| SGP41                     | [Aliexpress](https://www.aliexpress.com/item/4000037083952.html)      |
| Oled Display              | [Aliexpress](https://www.aliexpress.com/item/32950307344.html)        |

## Hardware
```
Neodim magnet: 2.5mm diameter, 1mm height
Heat insert: M2 x 3 x 3.2 for main board
Screws: M2 for PMS5003
```

## Pin connection matrix

```
| ESP32-C3 | OLED    | SGP41   | SHT45   | Senseair S8 | PMS5003  |
|----------|---------|---------|---------|-------------|----------|
| GND      | Pin GND | Pin GND | Pin GND | G0          | PIN2 GND |
| 3V3      | Pin 3V3 | Pin 3V3 | Pin 3V3 |             |          |
| 5V       |         |         |         | G+          | PIN1 VCC |
| GPIO_0   |         |         |         |             | PIN4 TX  |
| GPIO_1   |         |         |         |             | PIN5 RX  |
| GPIO_2   |         |         |         |             |          |
| GPIO_3   |         |         |         |             |          |
| GPIO_4   | Pin SCK |         |         |             |          |
| GPIO_5   | Pin DC  |         |         |             |          |
| GPIO_6   | Pin SDA |         |         |             |          |
| GPIO_7   | Pin CS  |         |         |             |          |
| GPIO_8   |         | Pin SDA | Pin SDA |             |          |
| GPIO_9   |         | Pin SCL | Pin SCL |             |          |
| GPIO_10  | Pin RES |         |         |             |          |
| GPIO_20  |         |         |         | UART_TxD    |          |
| GPIO_21  |         |         |         | UART_RxD    |          |
```

## How to use example
