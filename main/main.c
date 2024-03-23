/*
 * @Author: calin.acr 
 * @Date: 2024-03-15 14:00:51 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-21 01:53:02
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
#include "senseair_s8.h"
#include "main.h"
#include "pms5003.h"

void vOledTask(void *pvParameters);
void vI2CDevTask(void *pvParameters);
void vI2CDummyTask(void *pvParameters);
void vSenseairTask(void *pvParameters);

QueueHandle_t i2cdev_queue;
QueueHandle_t s8_queue;
QueueHandle_t pms_queue;

TaskHandle_t xOledTaskHandle = NULL;
TaskHandle_t xI2CDevTaskHandle = NULL;
TaskHandle_t xI2CDummyTaskHandle = NULL;
TaskHandle_t xSenseairTaskHandle = NULL;
TaskHandle_t xPMSTaskHandle = NULL;

SemaphoreHandle_t xI2CMutex = NULL;


void app_main(void)
{
    // Create queue between i2c and oled task
    i2cdev_queue = xQueueCreate(3, sizeof(struct I2CDevMessage *));
    s8_queue = xQueueCreate(3, sizeof(uint16_t));
    pms_queue = xQueueCreate(3, sizeof(struct PMSData *));
    // create mutex for i2c hardware sharing
    xI2CMutex = xSemaphoreCreateMutex();
    
    // Initialize the I2C master bus
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));

    xTaskCreate(vI2CDevTask, "I2CDev TASK", 2560, (void *)bus_handle, 11, &xI2CDevTaskHandle);
    configASSERT(xI2CDevTaskHandle);

    xTaskCreate(vOledTask, "OLED TASK", 2560, NULL, 11, &xOledTaskHandle);
    configASSERT(xOledTaskHandle);

    // xTaskCreate(vI2CDummyTask, "DUMMY TASK", 2560, NULL, 11, &xI2CDummyTaskHandle);
    // configASSERT(xI2CDummyTaskHandle);

    // xTaskCreate(vSenseairTask, "SENSEAIR TASK", 2560, NULL, tskIDLE_PRIORITY, &xSenseairTaskHandle);
    // configASSERT(xSenseairTaskHandle);

    xTaskCreate(PMS_MainTask, "PMS5003 TASK", 2560, (void *)pms_queue, tskIDLE_PRIORITY, &xPMSTaskHandle);
    configASSERT(xPMSTaskHandle);

    // for ( ;; )
    // {
        /* Stack size measurements */
        // ESP_LOGE("APP_MAIN", "Oled task stack unused: %d", uxTaskGetStackHighWaterMark(xOledTaskHandle));
        // ESP_LOGE("APP_MAIN", "I2C task stack unused: %d", uxTaskGetStackHighWaterMark(xI2CDevTaskHandle));
        // ESP_LOGE("APP_MAIN", "Dummy task stack unused: %d", uxTaskGetStackHighWaterMark(xI2CDummyTaskHandle));
        // ESP_LOGE("APP_MAIN", "Senseair task stack unused: %d", uxTaskGetStackHighWaterMark(xSenseairTaskHandle));
        // ESP_LOGE("APP_MAIN", "PMS5003 task stack unused: %d", uxTaskGetStackHighWaterMark(xPMSTaskHandle));
        // vTaskDelay(pdMS_TO_TICKS(5000));
    // }
}

void vOledTask(void *pvParameters)
{
    ESP_LOGW("OLED_TASK", "Task started");
    
    char temp_buff[7] = "wait  ";
    char hum_buff[7] = "wait  ";
    char co2_buff[6] = "wait ";
    char pm10_buff[6] = "wait ";
    char pm25_buff[6] = "wait ";
    char pm100_buff[6] = "wait ";
    char tvoc_buff[6] = "wait ";
    char nox_buff[6] = "wait ";

    struct I2CDevMessage *pxMessage;
    uint16_t co2_ppm;
    struct PMSRecvMessage *pxPMSMessage;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(60000);

    xLastWakeTime = xTaskGetTickCount();

    oled_init();
    
    oled_draw_text("AirQM -> Meow TECH", 0, 1);

    oled_draw_text(temp_buff, 1, 0);
    oled_draw_text("~C", 1, 6);
    oled_draw_text(hum_buff, 1, 12);
    oled_draw_text("%RH", 1, 18);

    
    oled_draw_text("CO2", 2, 0);
    oled_draw_text(co2_buff, 2, 7);
    oled_draw_text("ppm", 2, 14);

    oled_draw_text("PM1.0", 3, 0);
    oled_draw_text(pm10_buff, 3, 7);
    oled_draw_text("ug/m3", 3, 14);

    oled_draw_text("PM2.5", 4, 0);
    oled_draw_text(pm25_buff, 4, 7);
    oled_draw_text("ug/m3", 4, 14);

    oled_draw_text("PM10", 5, 0);
    oled_draw_text(pm100_buff, 5, 7);
    oled_draw_text("ug/m3", 5, 14);

    oled_draw_text("VOC", 6, 0);
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
                // Convert values to strings
                sprintf(temp_buff, "%6.2f", pxMessage->temp_phy);
                sprintf(hum_buff, "%6.2f", pxMessage->hum_phy);
                sprintf(tvoc_buff, "%5ld", pxMessage->voc_index_value);
                sprintf(nox_buff, "%5ld", pxMessage->nox_index_value);
                // Write to GDDRAM
                oled_draw_text(temp_buff, 1, 0);
                oled_draw_text(hum_buff, 1, 12);
                oled_draw_text(tvoc_buff, 6, 7);
                oled_draw_text(nox_buff, 7, 7);
            }
        }

        if (s8_queue != 0)
        {
            if (xQueueReceive(s8_queue, &co2_ppm, (TickType_t) 10))
            {
                // Convert values to strings
                sprintf(co2_buff, "%5d", co2_ppm);
                // Write to GDDRAM
                oled_draw_text(co2_buff, 2, 7); 
            }
        }

        if (pms_queue != 0)
        {
            if (xQueueReceive(pms_queue, &(pxPMSMessage), (TickType_t) 10))
            {
                // Convert values to strings
                sprintf(pm10_buff, "%5d", (pxPMSMessage->pm10_std));
                sprintf(pm25_buff, "%5d", (pxPMSMessage->pm25_std));
                sprintf(pm100_buff, "%5d", (pxPMSMessage->pm100_std));
                // Write to GDDRAM
                oled_draw_text(pm10_buff, 3, 7);
                oled_draw_text(pm25_buff, 4, 7);
                oled_draw_text(pm100_buff, 5, 7);
            }
        }

        if (xTaskGetTickCount() - xLastWakeTime >= xFrequency)
        {
            ESP_LOGE("OLED->WIFI", "Sending data to wifi task");
            xLastWakeTime = xTaskGetTickCount();
        }

        oled_flush();
        // vTaskDelay(pdMS_TO_TICKS(500));
        // taskYIELD();
    }
}

void vI2CDevTask(void *pvParameters)
{
    ESP_LOGW("I2C TASK", "Task started");

    struct I2CDevMessage *pxMessage;
    i2c_master_bus_handle_t bus_handle;

    bus_handle = (i2c_master_bus_handle_t) pvParameters;
    pxMessage = &xI2CDevMessage;

    uint8_t conditioning_counter = 0;

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

    // SGP41 conditioning (1 command every second, max 10 seconds)
    while (conditioning_counter < 10)
    {
        SGP41_ExecuteConditioning(&sgp41_voc_raw);
        // Count as conditioning iteration done
        conditioning_counter++;
        // Wait one second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    for ( ;; )
    {
        // Task body
        // Read data from I2C sensors
        SHT45_Read(SHT45_MODE_HIGH_PRECISION_NO_HEATER, &sht45_temp_raw, &pxMessage->temp_phy, &sht45_hum_raw, &pxMessage->hum_phy);
        SGP41_MeasureRawSignals(sht45_temp_raw, sht45_hum_raw, &sgp41_voc_raw, &sgp41_nox_raw);

        // ESP_LOGI("SGP", "RAW VOC %d, NOx %d", sgp41_voc_raw, sgp41_nox_raw);

        // Use Sensirion Algorithm to get VOC and NOx indices
        GasIndexAlgorithm_process(&voc_params, sgp41_voc_raw, &pxMessage->voc_index_value);
        GasIndexAlgorithm_process(&nox_params, sgp41_nox_raw, &pxMessage->nox_index_value);
        
        // ESP_LOGI("SGP", "VOC %ld, NOx %ld", pxMessage->voc_index_value, pxMessage->nox_index_value);

        if (i2cdev_queue != 0)
        {
            if(xQueueSendToBack(i2cdev_queue, (void *) &pxMessage, (TickType_t) 0) != pdPASS)
            {
                ESP_LOGE("I2C TASK", "Failed to post the message");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        

        // taskYIELD();
    }
}


void vSenseairTask(void *pvParameters)
{
    ESP_LOGW("S8 TASK", "Task started");
    // uint8_t cal_status;
    uint16_t co2_value, status;
    uint16_t abc, memmap_ver, fw_ver;
    uint32_t sens_type_id, sens_id;
    
    SenseairS8_Init();
    
    // cal_status = S8_BgCalibration(CO2_BG_CAL);
    // ESP_LOGI("S8 TASK", "Calibration status %d", cal_status);
    
    S8_ReadSensorTypeID(&sens_type_id);
    ESP_LOGI("S8 TASK", "Sensor Type ID 0x%08jx", (uintmax_t)sens_type_id);

    S8_ReadABC(&abc);
    ESP_LOGI("S8 TASK", "ABC period 0x%04x, %d hours", abc, abc);

    S8_ReadMemMapVers(&memmap_ver);
    ESP_LOGI("S8 TASK", "Memory Mapping ver 0x%04x", memmap_ver);

    S8_ReadFWVers(&fw_ver);
    ESP_LOGI("S8 TASK", "Firmware ver 0x%04x", fw_ver);

    S8_ReadSensorID(&sens_id);
    ESP_LOGI("S8 TASK", "Sensor ID (SN) 0x%08jx", (uintmax_t)sens_id);
    
    for( ;; )
    {
        vTaskDelay(pdMS_TO_TICKS(4000));
        S8_ReadCO2andStatus(&co2_value, &status);
        
        if ((s8_queue != 0) && (!CHECK_ERROR_ANY(status)))
        {
            if (xQueueSendToBack(s8_queue, (void *) &co2_value, (TickType_t) 0) != pdTRUE)
            {
                ESP_LOGE("S8 TASK", "Failed to post the message");
            }
        }
    }
}

