#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_server.hpp"
#include "imu_manager.hpp"

static const char *TAG = "MAIN";

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    auto& ble = BLEServer::getInstance();
    auto& imu = IMUManager::getInstance();

    ble.init();
    imu.init();

    xTaskCreate([](void* p){
        auto& ble = BLEServer::getInstance();
        auto& imu = IMUManager::getInstance();
        while(1) {
            Orientation o = imu.update();
            if (ble.isConnected()) {
                ble.notifyOrientation(o);
            }
            // Reduced log frequency to avoid flooding console at 100Hz
            static int count = 0;
            if (count++ % 10 == 0) {
                ESP_LOGI("MPU_DATA", "Roll: %.2f | Pitch: %.2f | Yaw: %.2f", o.roll, o.pitch, o.yaw);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "imu_task", 4096, NULL, 5, NULL);
}
