/*
 * @Author: calin.acr 
 * @Date: 2024-03-22 20:42:39 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-22 21:24:17
 */

#include <stdio.h>
#include "stdint.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define PMS_UART_PORT UART_NUM_1
#define PMS_UART_TX GPIO_NUM_0
#define PMS_UART_RX GPIO_NUM_1

#define PMS_UART_BUF_SIZE 256

extern uart_config_t pms_uart_cfg;

void PMS_Init(void);
void PMS_MainTask(void *pvParameters);
void PMS_Le2Be_PMSData(void);
