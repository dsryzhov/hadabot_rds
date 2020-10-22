#include "uxr/client/config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/uart.h>
#include <driver/gpio.h>

#include <string.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
//#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "WiFi.h"

extern "C" void appMain(void *argument);

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY



void wifi_init_sta()
{
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ESP_WIFI_SSID);

    WiFi.begin(ESP_WIFI_SSID, ESP_WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

extern "C" void app_main(void)
{   
    // Start networkign if required
#ifdef UCLIENT_PROFILE_UDP
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

//    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    Serial.begin(115200);
    wifi_init_sta();
#endif  // UCLIENT_PROFILE_UDP

    // start microROS task
    xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);
}
