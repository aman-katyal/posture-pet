#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"
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

    // Set static MAC address (optional, but ensures it stays the same across reboots/devices)
    // Note: This must be called before initializing any network interfaces.
    uint8_t static_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01}; 
    esp_base_mac_addr_set(static_mac);
    
    uint8_t derived_mac[6];
    esp_read_mac(derived_mac, ESP_MAC_BT);
    ESP_LOGI("APP", "Static MAC set. Current BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             derived_mac[0], derived_mac[1], derived_mac[2],
             derived_mac[3], derived_mac[4], derived_mac[5]);

    auto& ble = BLEServer::getInstance();
    auto& imu = IMUManager::getInstance();

    ble.init();
    imu.init();

    xTaskCreate([](void* p){
        auto& ble = BLEServer::getInstance();
        auto& imu = IMUManager::getInstance();
        
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz

        while(1) {
            MultiOrientation mo = imu.update();
            if (ble.isConnected()) {
                ble.notifyMultiOrientation(mo);
            }
            
            // Minimal diagnostic log
            static int count = 0;
            if (count++ % 100 == 0) {
                ESP_LOGI("APP", "Posture Update [N, L, R] | Neck QW: %.3f | L Shoulder QW: %.3f | R Shoulder QW: %.3f", 
                         mo.sensors[SENSOR_NECK].qw,
                         mo.sensors[SENSOR_SHOULDER_L].qw,
                         mo.sensors[SENSOR_SHOULDER_R].qw);
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }, "imu_task", 4096, NULL, 5, NULL);
}
