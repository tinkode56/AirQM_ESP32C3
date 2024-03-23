/*
 * @Author: calin.acr 
 * @Date: 2024-03-22 20:42:47 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-22 21:33:58
 */

#include "pms5003.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"

struct PMSMessage
{
    uint16_t frame_len;
    uint16_t pm10_std;
    uint16_t pm25_std;
    uint16_t pm100_std;
    uint16_t pm10_env;
    uint16_t pm25_env;
    uint16_t pm100_env;
    uint16_t particles_03um;
    uint16_t particles_05um;
    uint16_t particles_10um;
    uint16_t particles_25um;
    uint16_t particles_50um;
    uint16_t particles_100um;
    uint16_t dummy;
    uint16_t checksum;
} xPMSMessage;

uart_config_t pms_uart_cfg = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

static QueueHandle_t pms_uart_queue;
struct PMSMessage *pxPMSMessage;


void PMS_Init(void)
{
    //Install UART driver, and get the queue.
    uart_driver_install(PMS_UART_PORT, PMS_UART_BUF_SIZE, PMS_UART_BUF_SIZE, 5, &pms_uart_queue, 0);
    uart_param_config(PMS_UART_PORT, &pms_uart_cfg);

    //Set UART pins
    uart_set_pin(PMS_UART_PORT, PMS_UART_TX, PMS_UART_RX, GPIO_NUM_NC, GPIO_NUM_NC);
    // pxPMSData = &pms5003_data;
}

void PMS_MainTask(void *pvParameters)
{
    ESP_LOGW("PMS TASK", "Task started");
    pxPMSMessage = &xPMSMessage;

    QueueHandle_t pms_queue = (QueueHandle_t) pvParameters;
    
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(PMS_UART_BUF_SIZE);

    PMS_Init();
    
    for( ;; )
    {
        if(xQueueReceive(pms_uart_queue, (void *)&event, pdMS_TO_TICKS(2300)))
        {
            bzero(dtmp, PMS_UART_BUF_SIZE);
            // ESP_LOGI("PMS TASK", "uart[%d] event:", PMS_UART_PORT);

            switch (event.type)
            {
            case UART_DATA:
                // ESP_LOGI("PMS", "[UART DATA]: %d", event.size);
                uart_read_bytes(PMS_UART_PORT, dtmp, event.size, portMAX_DELAY);
                break;

            case UART_FIFO_OVF:
                ESP_LOGI("PMS TASK", "hw fifo overflow");
                uart_flush_input(PMS_UART_PORT);
                xQueueReset(pms_uart_queue);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGI("PMS TASK", "ring buffer full");
                uart_flush_input(PMS_UART_PORT);
                xQueueReset(pms_uart_queue);
                break;

            default:
                ESP_LOGI("PMS TASK", "uart event type: %d", event.type);
                break;
            }

            if (event.type == UART_DATA)
            {
                // Populate struct with received data
                memcpy(pxPMSMessage, &dtmp[2], sizeof(struct PMSMessage));
                // Convert to big endian
                PMS_Le2Be_PMSData();

                ESP_LOGI("PMS", "PM 1.0 %5d ug/m3", pxPMSMessage->pm10_std);
                ESP_LOGI("PMS", "PM 2.5 %5d ug/m3", pxPMSMessage->pm25_std);
                ESP_LOGI("PMS", "PM 10  %5d ug/m3", pxPMSMessage->pm100_std);

                if (pms_queue != 0)
                {
                    if(xQueueSendToBack(pms_queue, (void *)&pxPMSMessage, (TickType_t) 0) != pdTRUE)
                    {
                        ESP_LOGE("PMS TASK", "Failed to post the message");
                    }
                }
            }
            
        }
    }
}

void PMS_Le2Be_PMSData(void)
{
    // // Convert struct elements from little endian to big endian
    pxPMSMessage->frame_len = __htons(pxPMSMessage->frame_len);
    pxPMSMessage->pm10_std = __htons(pxPMSMessage->pm10_std);
    pxPMSMessage->pm25_std = __htons(pxPMSMessage->pm25_std);
    pxPMSMessage->pm100_std = __htons(pxPMSMessage->pm100_std);
    pxPMSMessage->pm10_env = __htons(pxPMSMessage->pm10_env);
    pxPMSMessage->pm25_env = __htons(pxPMSMessage->pm25_env);
    pxPMSMessage->pm100_env = __htons(pxPMSMessage->pm100_env);
    pxPMSMessage->particles_03um = __htons(pxPMSMessage->particles_03um);
    pxPMSMessage->particles_05um = __htons(pxPMSMessage->particles_05um);
    pxPMSMessage->particles_10um = __htons(pxPMSMessage->particles_10um);
    pxPMSMessage->particles_25um = __htons(pxPMSMessage->particles_25um);
    pxPMSMessage->particles_50um = __htons(pxPMSMessage->particles_50um);
    pxPMSMessage->particles_100um = __htons(pxPMSMessage->particles_100um);
    pxPMSMessage->dummy = __htons(pxPMSMessage->dummy);
    pxPMSMessage->checksum = __htons(pxPMSMessage->checksum);
}
