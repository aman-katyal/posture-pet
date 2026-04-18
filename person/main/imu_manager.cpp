#include "imu_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include "mpu/math.hpp"

// Bus 0 Pins (Shoulders)
#define I2C0_SCL 18
#define I2C0_SDA 17

// Bus 1 Pins (Neck - User will wire these soon)
#define I2C1_SCL 22 
#define I2C1_SDA 21

static const char *TAG = "IMU_MANAGER";

// Track which sensors are actually plugged in
static bool sensorActive[NUM_SENSORS] = {false, false, false};

void IMUManager::init() {
    // Initialize Bus 0 (Shoulders)
    ESP_LOGI(TAG, "Initializing I2C Bus 0...");
    i2c0.begin((gpio_num_t)I2C0_SDA, (gpio_num_t)I2C0_SCL, 400000);

    // Initialize Bus 1 (Neck)
    ESP_LOGI(TAG, "Initializing I2C Bus 1...");
    i2c1.begin((gpio_num_t)I2C1_SDA, (gpio_num_t)I2C1_SCL, 400000);

    // Setup Shoulder L (Bus 0, 0x68)
    mpuL.setBus(i2c0);
    mpuL.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    if (mpuL.testConnection() == ESP_OK) {
        ESP_LOGI(TAG, "Shoulder L connected at 0x68");
        mpuL.initialize();
        sensorActive[SENSOR_SHOULDER_L] = true;
    }

    // Setup Shoulder R (Bus 0, 0x69)
    mpuR.setBus(i2c0);
    mpuR.setAddr(mpud::MPU_I2CADDRESS_AD0_HIGH);
    if (mpuR.testConnection() == ESP_OK) {
        ESP_LOGI(TAG, "Shoulder R connected at 0x69");
        mpuR.initialize();
        sensorActive[SENSOR_SHOULDER_R] = true;
    }

    // Setup Neck (Bus 1, 0x68)
    mpuNeck.setBus(i2c1);
    mpuNeck.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    if (mpuNeck.testConnection() == ESP_OK) {
        ESP_LOGI(TAG, "Neck sensor connected on Bus 1");
        mpuNeck.initialize();
        sensorActive[SENSOR_NECK] = true;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        mahony_init(&qStates[i]);
    }
    
    lastTime = esp_timer_get_time();
}

MultiOrientation IMUManager::update() {
    MultiOrientation mo = {};
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - lastTime) / 1000000.0f;
    lastTime = now;

    MPU_t* mpus[NUM_SENSORS] = {&mpuNeck, &mpuL, &mpuR};

    for (int i = 0; i < NUM_SENSORS; i++) {
        // Only attempt update if sensor was found during init
        if (!sensorActive[i]) continue;

        mpud::raw_axes_t accelRaw, gyroRaw, magRaw;
        if (mpus[i]->motion(&accelRaw, &gyroRaw, &magRaw) == ESP_OK) {
            mpud::float_axes_t accel = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
            mpud::float_axes_t gyro = mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);

            mahony_update(&qStates[i], gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, 0, 0, 0, dt, 10.0f, 0.0f);

            mo.sensors[i].roll = mahony_get_roll(&qStates[i]);
            mo.sensors[i].pitch = mahony_get_pitch(&qStates[i]);
            mo.sensors[i].yaw = mahony_get_yaw(&qStates[i]);
            mo.sensors[i].qw = qStates[i].w;
            mo.sensors[i].qx = qStates[i].x;
            mo.sensors[i].qy = qStates[i].y;
            mo.sensors[i].qz = qStates[i].z;
        }
    }
    return mo;
}
