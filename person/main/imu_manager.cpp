#include "imu_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu/math.hpp"
#include <math.h>
#include <string.h>

// Bus 0 Pins (Shoulders)
#define I2C0_SCL 18
#define I2C0_SDA 17

// Bus 1 Pins (Neck)
#define I2C1_SCL 4
#define I2C1_SDA 5

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

  // Static Calibration
  ESP_LOGI(TAG, "Calibrating sensors (1s)... keep device still.");
  MPU_t *mpus[NUM_SENSORS] = {&mpuNeck, &mpuL, &mpuR};
  const int samples = 50;
  for (int i = 0; i < NUM_SENSORS; i++) {
    gyroBias[i] = {0, 0, 0};
    if (!sensorActive[i]) continue;
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int s = 0; s < samples; s++) {
      mpud::raw_axes_t accelRaw, gyroRaw;
      if (mpus[i]->motion(&accelRaw, &gyroRaw) == ESP_OK) {
        mpud::float_axes_t gyro =
            mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    gyroBias[i] = {sumX / samples, sumY / samples, sumZ / samples};
    ESP_LOGI(TAG, "Sensor %d Gyro Bias: %.4f, %.4f, %.4f", i, gyroBias[i].x,
             gyroBias[i].y, gyroBias[i].z);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    mahony_init(&qStates[i]);
  }

  startTime = esp_timer_get_time();
  lastTime = startTime;
}

MultiOrientation IMUManager::update() {
  MultiOrientation mo = {};
  int64_t now = esp_timer_get_time();
  float dt = (float)(now - lastTime) / 1000000.0f;
  float elapsed = (float)(now - startTime) / 1000000.0f;
  lastTime = now;

  if (dt <= 0.0f)
    return mo;

  MPU_t *mpus[NUM_SENSORS] = {&mpuNeck, &mpuL, &mpuR};

  // Filter gains: High gain initially to settle, then low gain for stability
  float kp = (elapsed < 5.0f) ? 10.0f : 1.0f;
  float ki = (elapsed < 5.0f) ? 0.0f : 0.05f;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorActive[i]) {
      mo.sensors[i].qw = 1.0f;
      continue;
    }

    mpud::raw_axes_t accelRaw, gyroRaw, magRaw;
    if (mpus[i]->motion(&accelRaw, &gyroRaw, &magRaw) == ESP_OK) {
      mpud::float_axes_t accel =
          mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
      mpud::float_axes_t gyro =
          mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);

      // Subtract bias
      gyro.x -= gyroBias[i].x;
      gyro.y -= gyroBias[i].y;
      gyro.z -= gyroBias[i].z;

      mahony_update(&qStates[i], gyro.x, gyro.y, gyro.z, accel.x, accel.y,
                    accel.z, 0, 0, 0, dt, kp, ki);

      mo.sensors[i].qw = qStates[i].w;
      mo.sensors[i].qx = qStates[i].x;
      mo.sensors[i].qy = qStates[i].y;
      mo.sensors[i].qz = qStates[i].z;
    } else {
      mo.sensors[i].qw = 1.0f;
    }
  }
  return mo;
}
