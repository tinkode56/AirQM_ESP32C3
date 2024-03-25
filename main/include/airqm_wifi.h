/*
 * @Author: calin.acr 
 * @Date: 2024-03-25 17:27:24 
 * @Last Modified by: calin.acr
 * @Last Modified time: 2024-03-25 17:38:10
 */

#if !defined(_AIRQM_WIFI_H)
#define _AIRQM_WIFI_H

#define PROV_QR_VERSION         "v1"
#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"
#define QRCODE_BASE_URL         "https://espressif.github.io/esp-jumpstart/qrcode.html"

void airqm_wifiprov_init(void);

#endif // _AIRQM_WIFI_H
