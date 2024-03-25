/*
 * @Author: calin.acr 
 * @Date: 2024-03-25 18:34:53 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-25 18:37:23
 */

#include "airqm_influxdb.h"
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_http_client.h"

void write_influxdb(AirQM_metrics *metrics)
{
    char query_buffer[64] = { '\0' };
    snprintf(query_buffer, 64, "db=%s&precision=s", CONFIG_AIRQM_INFLUXDB_BUCKET);

    esp_http_client_config_t influxdb_config = {
        .host = CONFIG_AIRQM_INFLUXDB_HOST,
        .path = CONFIG_AIRQM_INFLUXDB_PATH,
        .port = CONFIG_AIRQM_INFLUXDB_PORT,
        .query = query_buffer,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
    };
    esp_http_client_handle_t influxdb_client = esp_http_client_init(&influxdb_config);

    // char auth_buffer[64] = { '\0' };
    // snprintf(auth_buffer, 512, "Token %s:%s", CONFIG_AIRQM_INFLUXDB_USERNAME, CONFIG_AIRQM_INFLUXDB_PASSWORD);
    // esp_http_client_set_header(influxdb_client, "Authorization", auth_buffer);


    // INSERT airquality,metric=temperature value=27.07 111111111111111
    //      measurement        tag            value      timestamp
    time_t now = 0;
    time(&now);
    char post_buffer[1024] = { '\0' };
    snprintf(post_buffer, 1024, "airquality,metric=temperature value=%f %lld\n"
        "airquality,metric=humidity value=%f %lld\n"
        "airquality,metric=co2 value=%d %lld\n"
        "airquality,metric=pm10std value=%d %lld\n"
        "airquality,metric=pm25std value=%d %lld\n"
        "airquality,metric=pm100std value=%d %lld\n"
        "airquality,metric=pm10env value=%d %lld\n"
        "airquality,metric=pm25env value=%d %lld\n"
        "airquality,metric=pm100env value=%d %lld\n"
        "airquality,metric=particles_03um value=%d %lld\n"
        "airquality,metric=particles_05um value=%d %lld\n"
        "airquality,metric=particles_10um value=%d %lld\n"
        "airquality,metric=particles_25um value=%d %lld\n"
        "airquality,metric=particles_50um value=%d %lld\n"
        "airquality,metric=particles_100um value=%d %lld\n"
        "airquality,metric=nox_index value=%ld %lld\n"
        "airquality,metric=voc_index value=%ld %lld",
        metrics->temp_phy, now,
        metrics->hum_phy, now,
        metrics->co2_val, now,
        metrics->pm10_std, now,
        metrics->pm25_std, now,
        metrics->pm100_std, now,
        metrics->pm10_env, now,
        metrics->pm25_env, now,
        metrics->pm100_env, now,
        metrics->particles_03um, now,
        metrics->particles_05um, now,
        metrics->particles_10um, now,
        metrics->particles_25um, now,
        metrics->particles_50um, now,
        metrics->particles_100um, now,
        metrics->nox_index_value, now,
        metrics->voc_index_value, now);

    ESP_LOGI("InfluxDB", "Posting data to http://%s:%d%s?%s: %s", CONFIG_AIRQM_INFLUXDB_HOST, CONFIG_AIRQM_INFLUXDB_PORT, CONFIG_AIRQM_INFLUXDB_PATH, query_buffer, post_buffer);

    esp_err_t err; 
    esp_http_client_set_method(influxdb_client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(influxdb_client, post_buffer, strlen(post_buffer));
    err = esp_http_client_perform(influxdb_client);
    if (err == ESP_OK) {
        ESP_LOGI("InfluxDB", "HTTP POST Status = %d, content_length = %"PRIu64,
                esp_http_client_get_status_code(influxdb_client),
                esp_http_client_get_content_length(influxdb_client));
    } else {
        ESP_LOGE("InfluxDB", "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(influxdb_client);
}
