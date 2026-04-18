set(srcs "main.cpp" "ble_server.cpp" "imu_manager.cpp")
idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES nvs_flash bt MPUdriver I2Cbus esp_driver_i2c esp_driver_gpio esp_timer)
