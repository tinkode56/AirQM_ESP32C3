/*
 * @Author: calin.acr 
 * @Date: 2024-07-09 20:23:52 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-07-09 20:41:47
 */

#if !defined(_BOARD_SUPPORT_H)
#define _BOARD_SUPPORT_H

#include "driver/gpio.h"

/* OLED PINS */
#define AIRQM_OLED_SCK_PIN      GPIO_NUM_4
#define AIRQM_OLED_MOSI_PIN     GPIO_NUM_6
#define AIRQM_OLED_CS_PIN       GPIO_NUM_7
#define AIRQM_OLED_DC_PIN       GPIO_NUM_5
#define AIRQM_OLED_RESET_PIN    GPIO_NUM_10

/* PMS5003 PINS */
#define AIRQM_PMS_UART_TX_PIN   GPIO_NUM_1
#define AIRQM_PMS_UART_RX_PIN   GPIO_NUM_0

/* SenseairS8 PINS */
#define AIRQM_SENSEAIR_TX_PIN   GPIO_NUM_21
#define AIRQM_SENSEAIR_RX_PIN   GPIO_NUM_20

/* SGP41 and SHT45 PINS */
#define AIRQM_I2C_SCL   GPIO_NUM_9
#define AIRQM_I2C_SDA   GPIO_NUM_8

/* NEOPIXEL PIN */
#define AIRQM_NEOPIXEL_PIN  GPIO_NUM_3

#endif // _BOARD_SUPPORT_H
