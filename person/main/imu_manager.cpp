#include "imu_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

#define I2C_SCL 18
#define I2C_SDA 17
#define I2C_PORT I2C_NUM_0

static const char *TAG = "IMU_MANAGER";

void IMUManager::init() {
    // Initialize I2C bus
    i2c0.begin((gpio_num_t)I2C_SDA, (gpio_num_t)I2C_SCL, 400000);
    
    mpu.setBus(i2c0);
    mpu.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    
    esp_err_t ret = mpu.testConnection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to MPU9250: %s", esp_err_to_name(ret));
        return;
    }

    mpu.initialize(); // Sets Accel 4G, Gyro 500DPS, 100Hz, DLPF 42Hz, and Compass
    
    quaternion_init(&qState);
    lastTime = esp_timer_get_time();
}

Orientation IMUManager::update() {
    mpud::raw_axes_t accelRaw, gyroRaw, magRaw;
    Orientation o = {0, 0, 0};
    
    if (mpu.motion(&accelRaw, &gyroRaw, &magRaw) == ESP_OK) {
        // Convert to physical units
        mpud::float_axes_t accel = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        mpud::float_axes_t gyro = mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        
        // MPU9250 Mag axes are different from Accel/Gyro
        // Typically: Accel X, Y, Z -> Mag Y, X, -Z or similar depending on mounting
        // For standard MPU9250 orientation:
        float mx = (float)magRaw.y; 
        float my = (float)magRaw.x;
        float mz = -(float)magRaw.z;

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - lastTime) / 1000000.0f;
        lastTime = now;

        quaternion_update(&qState, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mx, my, mz, dt);
        
        o.roll = quaternion_get_roll(&qState);
        o.pitch = quaternion_get_pitch(&qState);
        o.yaw = quaternion_get_yaw(&qState);
    }
    return o;
}
