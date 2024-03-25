/*
 * @Author: calin.acr 
 * @Date: 2024-03-25 18:34:17 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-25 18:40:43
 */

#if !defined(_AIRQM_INFLUXDB_H)
#define _AIRQM_INFLUXDB_H

#include "stdint.h"

typedef struct {
    uint16_t co2_val;
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
    int32_t voc_index_value;
    int32_t nox_index_value;
    float temp_phy;
    float hum_phy;
} AirQM_metrics;

void write_influxdb(AirQM_metrics *metrics);

#endif // _AIRQM_INFLUXDB_H
