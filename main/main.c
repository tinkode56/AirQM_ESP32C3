/*
 * @Author: calin.acr 
 * @Date: 2024-03-15 14:00:51 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-16 01:34:45
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "oled_display.h"
#include "sgp41.h"
#include "sht45.h"
#include "string.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "sensirion_gas_index_algorithm.h"
#include "main.h"

void vOledTask(void *pvParameters);
void vI2CDevTask(void *pvParameters);

QueueHandle_t i2cdev_queue;

TaskHandle_t xOledTaskHandle = NULL;
TaskHandle_t xI2CDevTaskHandle = NULL;


void app_main(void)
{
    i2cdev_queue = xQueueCreate(3, sizeof(struct SHT45Message *));
    
    // Initialize the I2C master bus
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_2,
        .scl_io_num = GPIO_NUM_3,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));

    xTaskCreate(vI2CDevTask, "I2CDev TASK", 8192, (void *)bus_handle, tskIDLE_PRIORITY, &xI2CDevTaskHandle);
    configASSERT(xI2CDevTaskHandle);

    xTaskCreate(vOledTask, "OLED TASK", 8192, NULL, tskIDLE_PRIORITY, &xOledTaskHandle);
    configASSERT(xOledTaskHandle);

    uint8_t data[] = { 0x31, 0x75, 0x66 };
    ESP_LOGI("SHT45_CRC", "CRC is: %d", SHT45_CRC8(data, 3));

}

void vOledTask(void *pvParameters)
{
    ESP_LOGW("OLED_TASK", "Task started");
    
    char temp_buff[7] = "wait  ";
    char hum_buff[7] = "wait  ";
    char co2_buff[6] = "wait ";
    char pm25_buff[6] = "wait ";
    char tvoc_buff[6] = "wait ";
    char nox_buff[6] = "wait ";

    struct I2CDevMessage *pxMessage;

    oled_init();
    
    oled_draw_text("AirQM -> Meow TECH", 0, 1);

    oled_draw_text(temp_buff, 1, 0);
    oled_draw_text("~C", 1, 6);
    oled_draw_text(hum_buff, 1, 12);
    oled_draw_text("%RH", 1, 18);

    oled_draw_text("---------------------", 2, 0);
    
    oled_draw_text("CO2", 3, 0);
    oled_draw_text(co2_buff, 3, 7);
    oled_draw_text("ppm", 3, 14);

    oled_draw_text("PM2.5", 4, 0);
    oled_draw_text(pm25_buff, 4, 7);
    oled_draw_text("ug/m3", 4, 14);

    oled_draw_text("---------------------", 5, 0);

    oled_draw_text("TVOC", 6, 0);
    oled_draw_text(tvoc_buff, 6, 7);

    oled_draw_text("NOx", 7, 0);
    oled_draw_text(nox_buff, 7, 7);

    oled_flush();

    for( ;; )
    {
        // Task body

        if (i2cdev_queue != 0)
        {
            if (xQueueReceive(i2cdev_queue, &(pxMessage), (TickType_t) 10))
            {
                sprintf(temp_buff, "%6.2f", pxMessage->temp_phy);
                sprintf(hum_buff, "%6.2f", pxMessage->hum_phy);
                sprintf(tvoc_buff, "%5ld", pxMessage->voc_index_value);
                sprintf(nox_buff, "%5ld", pxMessage->nox_index_value);
            }
        }

        sprintf(co2_buff, "%5d", 0);
        sprintf(pm25_buff, "%5d", 0);

        oled_draw_text(temp_buff, 1, 0);
        oled_draw_text(hum_buff, 1, 12);
        oled_draw_text(co2_buff, 3, 7);
        oled_draw_text(pm25_buff, 4, 7);
        oled_draw_text(tvoc_buff, 6, 7);
        oled_draw_text(nox_buff, 7, 7);

        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
        // taskYIELD();
    }
}

void vI2CDevTask(void *pvParameters)
{
    ESP_LOGW("SHT45_TASK", "Task started");

    struct I2CDevMessage *pxMessage;
    i2c_master_bus_handle_t bus_handle;

    bus_handle = (i2c_master_bus_handle_t) pvParameters;
    pxMessage = &xI2CDevMessage;

    uint16_t sht45_temp_raw;
    uint16_t sht45_hum_raw;
    uint16_t sgp41_voc_raw;
    uint16_t sgp41_nox_raw;

    GasIndexAlgorithmParams voc_params;
    GasIndexAlgorithmParams nox_params;
    GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

    // SHT45 init and reset
    SHT45_Init(&bus_handle);
    SHT45_SoftReset();

    // SGP41 init
    SGP41_Init(&bus_handle);

    // SGP41 conditioning (1 trigger every 1 second, max 10 seconds)
    for (uint8_t i = 0; i < 10; i++)
    {
        SGP41_ExecuteConditioning(&sgp41_voc_raw);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    for( ;; )
    {
        // Task body
        // Read data from I2C sensors
        SHT45_Read(SHT45_MODE_HIGH_PRECISION_NO_HEATER, &sht45_temp_raw, &pxMessage->temp_phy, &sht45_hum_raw, &pxMessage->hum_phy);
        SGP41_MeasureRawSignals(sht45_temp_raw, sht45_hum_raw, &sgp41_voc_raw, &sgp41_nox_raw);

        ESP_LOGI("SGP", "RAW VOC %d, NOx %d", sgp41_voc_raw, sgp41_nox_raw);

        // Use Sensirion Algorithm to get VOC and NOx indices
        GasIndexAlgorithm_process(&voc_params, sgp41_voc_raw, &pxMessage->voc_index_value);
        GasIndexAlgorithm_process(&nox_params, sgp41_nox_raw, &pxMessage->nox_index_value);
        
        ESP_LOGI("SGP", "VOC %ld, NOx %ld", pxMessage->voc_index_value, pxMessage->nox_index_value);

        if (i2cdev_queue != 0)
        {
            xQueueSendToBack(i2cdev_queue, (void *) &pxMessage, (TickType_t) 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // taskYIELD();
    }
}

