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
    
    // Read Magnetometer Sensitivity Adjustment values
    mpu.compassGetAdjustment(&magAsa[0], &magAsa[1], &magAsa[2]);

    startTime = esp_timer_get_time();
    calib_init();
    mahony_init(&qState);
    lastTime = startTime;
}

Orientation IMUManager::update() {
    mpud::raw_axes_t accelRaw, gyroRaw, magRaw;
    Orientation o = {0, 0, 0};
    
    if (mpu.motion(&accelRaw, &gyroRaw, &magRaw) == ESP_OK) {
        // Convert to physical units
        mpud::float_axes_t accel = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        mpud::float_axes_t gyro_raw = mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        
        // Background calibration update
        calib_update(accel.x, accel.y, accel.z, gyro_raw.x, gyro_raw.y, gyro_raw.z);
        Bias bias = calib_get_gyro_bias();
        
        // Correct gyro
        float gx = gyro_raw.x - bias.x;
        float gy = gyro_raw.y - bias.y;
        float gz = gyro_raw.z - bias.z;

        // MPU9250 Mag axes are different from Accel/Gyro
        // Typically: Accel X, Y, Z -> Mag Y, X, -Z for standard mounting
        float mx = (float)mpud::math::magAdjust(magRaw.y, magAsa[1]); 
        float my = (float)mpud::math::magAdjust(magRaw.x, magAsa[0]);
        float mz = -(float)mpud::math::magAdjust(magRaw.z, magAsa[2]);

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - lastTime) / 1000000.0f;
        float elapsed = (float)(now - startTime) / 1000000.0f;
        lastTime = now;

        // Adaptive Gain
        float kp = (elapsed < 10.0f) ? 10.0f : 0.5f;
        float ki = (elapsed < 10.0f) ? 0.0f : 0.1f;

        mahony_update(&qState, gx, gy, gz, accel.x, accel.y, accel.z, mx, my, mz, dt, kp, ki);
        
        o.roll = mahony_get_roll(&qState);
        o.pitch = mahony_get_pitch(&qState);
        o.yaw = mahony_get_yaw(&qState);
    }
    return o;
}
