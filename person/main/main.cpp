#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_server.hpp"
#include "imu_manager.hpp"
#include "calibration.h"

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
        
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

        while(1) {
            MultiOrientation mo = imu.update();
            if (ble.isConnected()) {
                ble.notifyMultiOrientation(mo);
            }
            
            // Minimal diagnostic log
            static int count = 0;
            if (count++ % 100 == 0) {
                ESP_LOGI("APP", "Posture Update [N, L, R] | Neck Pitch: %.1f | L Shoulder: %.1f | R Shoulder: %.1f", 
                         mo.sensors[SENSOR_NECK].pitch,
                         mo.sensors[SENSOR_SHOULDER_L].pitch,
                         mo.sensors[SENSOR_SHOULDER_R].pitch);
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }, "imu_task", 4096, NULL, 5, NULL);
}
