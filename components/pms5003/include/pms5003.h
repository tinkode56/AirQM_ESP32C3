/*
 * @Author: calin.acr 
 * @Date: 2024-03-22 20:42:39 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-07-09 20:18:20
 */

#include <stdio.h>
#include "stdint.h"
#include "driver/uart.h"
#include "board_support.h"

#define PMS_UART_PORT UART_NUM_1
#define PMS_UART_TX AIRQM_PMS_UART_TX_PIN
#define PMS_UART_RX AIRQM_PMS_UART_RX_PIN

#define PMS_UART_BUF_SIZE 256

extern uart_config_t pms_uart_cfg;

void PMS_Init(void);
void PMS_MainTask(void *pvParameters);
void PMS_Le2Be_PMSData(void);
