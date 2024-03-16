/*
 * @Author: calin.acr 
 * @Date: 2024-03-14 18:20:49 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-16 02:01:06
 */
#include <stdio.h>
#include "sgp41.h"
#include "sht45.h"
#include "sensirion_gas_index_algorithm.h"
#include "string.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

/**
 * Sensirion algo usage
 * example at
 * https://github.com/Sensirion/gas-index-algorithm/blob/master/examples/raspberry-pi/algorithm_example_usage.c
 * 
 * initialize gas index parameters
 * 
 * 1. Sleep: Measure every second (1Hz), as defined by the Gas Index
 * 2. Measure SHT4x  RH and T signals and convert to SGP41 ticks
 * 3. Measure SGP4x signals
 * 4. Process raw signals by Gas Index Algorithm to get the VOC and NOx
 * 
*/

i2c_master_dev_handle_t sgp_device_handle = NULL;

void SGP41_Init(i2c_master_bus_handle_t *mbh)
{
    i2c_device_config_t sgp_i2c_device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x59,
        .scl_speed_hz = 400000,
    };
    // Register SGP41 sensor to the I2C master bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*mbh, &sgp_i2c_device, &sgp_device_handle));
}

void SGP41_Read(uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay)
{
    uint8_t buf[2];
    memset(buf, 0, sizeof(uint8_t) * 2);

    buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    buf[1] = (uint8_t)(reg & 0xFF);

    ESP_ERROR_CHECK(i2c_master_transmit(sgp_device_handle, (uint8_t *)buf, 2, -1));

    vTaskDelay(pdMS_TO_TICKS(delay));

    ESP_ERROR_CHECK(i2c_master_receive(sgp_device_handle, data, len, -1));
}

void SGP41_Write(uint16_t reg, uint8_t *data, uint16_t len)
{
    uint8_t buf[16];
    uint16_t i;

    memset(buf, 0, sizeof(uint8_t) * 16);
    buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (i = 0; i < len; i++)
    {
        buf[i + 2] = data[i];
    }

    ESP_ERROR_CHECK(i2c_master_transmit(sgp_device_handle, (uint8_t *)buf, len + 2, -1));
}

void SGP41_ReadWithParam(uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay, uint8_t *output, uint16_t output_len)
{
    uint8_t buf[16];
    uint16_t i;

    memset(buf, 0, sizeof(uint8_t) * 16);
    buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (i = 0; i < len; i++)
    {
        buf[i + 2] = data[i];
    }
    ESP_ERROR_CHECK(i2c_master_transmit(sgp_device_handle, (uint8_t *)buf, len + 2, -1));
    vTaskDelay(pdMS_TO_TICKS(delay));
    ESP_ERROR_CHECK(i2c_master_receive(sgp_device_handle, output, output_len, -1));
}

void SGP41_ExecuteConditioning(uint16_t *voc_raw)
{
    uint8_t input[6];
    uint8_t buf[3];

    memset(buf, 0, sizeof(uint8_t) * 3);
    input[0] = 0x80;
    input[1] = 0x00;
    input[2] = 0xA2;
    input[3] = 0x66;
    input[4] = 0x66;
    input[5] = 0x93;

    SGP41_ReadWithParam(SGP41_COMMAND_EXECUTE_CONDITIONING, input, 6, 50, buf, 3);

    *voc_raw = (uint16_t)(((uint16_t)buf[0]) << 8 | buf[1]);
}

void SGP41_MeasureRawSignals(uint16_t temp_raw, uint16_t hum_raw, uint16_t *voc_raw, uint16_t *nox_raw)
{
    uint8_t input[6];
    uint8_t buf[6];

    memset(buf, 0, sizeof(uint8_t) * 6);
    input[0] = (hum_raw >> 8) & 0xFF;
    input[1] = (hum_raw >> 0) & 0xFF;
    input[2] = SHT45_CRC8(&input[0], 2);
    input[3] = (temp_raw >> 8) & 0xFF;
    input[4] = (temp_raw >> 0) & 0xFF;
    input[5] = SHT45_CRC8(&input[3], 2);

    SGP41_ReadWithParam(SGP41_COMMAND_MEASURE_RAW, input, 6, 50, buf, 6);

    *voc_raw = (uint16_t)(((uint16_t)buf[0]) << 8 | buf[1]);
    *nox_raw = (uint16_t)(((uint16_t)buf[3]) << 8 | buf[4]);
}
