#include "imu_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu/math.hpp"
#include <math.h>
#include <string.h>

// Bus 0 Pins (Shoulders)
#define I2C0_SCL 1
#define I2C0_SDA 2

// Bus 1 Pins (Neck)
#define I2C1_SCL 47
#define I2C1_SDA 48

static const char *TAG = "IMU_MANAGER";

// Track which sensors are actually plugged in
static bool sensorActive[NUM_SENSORS] = {false, false, false};

void IMUManager::init() {
  // Initialize Bus 0 (Shoulders) - Using 100kHz for stability
  ESP_LOGI(TAG, "Initializing I2C Bus 0 (SDA:%d, SCL:%d) at 100kHz...", I2C0_SDA, I2C0_SCL);
  esp_err_t err0 = i2c0.begin((gpio_num_t)I2C0_SDA, (gpio_num_t)I2C0_SCL, 100000);
  if (err0 != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C Bus 0: %s", esp_err_to_name(err0));
  } else {
    i2c0.scanner();
  }

  // Initialize Bus 1 (Neck) - Using 100kHz for stability
  ESP_LOGI(TAG, "Initializing I2C Bus 1 (SDA:%d, SCL:%d) at 100kHz...", I2C1_SDA, I2C1_SCL);
  esp_err_t err1 = i2c1.begin((gpio_num_t)I2C1_SDA, (gpio_num_t)I2C1_SCL, 100000);
  if (err1 != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C Bus 1: %s", esp_err_to_name(err1));
  } else {
    i2c1.scanner();
  }

  auto setupSensor = [&](MPU_t &mpu, SensorIdx idx, const char *name) {
    uint8_t wai = mpu.whoAmI();
    esp_err_t ret = mpu.testConnection();
    
    ESP_LOGI(TAG, "Checking %s... WHO_AM_I: 0x%02X, Connection: %s", 
             name, wai, esp_err_to_name(ret));

    // For MPU6500, WHO_AM_I should be 0x70
    if (ret == ESP_OK || wai == 0x70) {
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "%s: Connection test failed but ID 0x%02X matches MPU-6500. Initializing...", name, wai);
      }
      
      mpu.initialize();
      mpu.setSleep(false); // ENSURE AWAKE
      
      // Verification Read
      mpud::raw_axes_t acc, gyro;
      if (mpu.motion(&acc, &gyro) == ESP_OK) {
        ESP_LOGI(TAG, "%s ONLINE. Accel: [%d, %d, %d]", name, acc.x, acc.y, acc.z);
        sensorActive[idx] = true;
      } else {
        ESP_LOGE(TAG, "%s failed to provide initial data", name);
      }
    }
  };

  // Setup Shoulder L (Bus 0, 0x68)
  mpuL.setBus(i2c0);
  mpuL.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
  setupSensor(mpuL, SENSOR_SHOULDER_L, "Shoulder L (0x68)");

  // Setup Shoulder R (Bus 0, 0x69)
  mpuR.setBus(i2c0);
  mpuR.setAddr(mpud::MPU_I2CADDRESS_AD0_HIGH);
  setupSensor(mpuR, SENSOR_SHOULDER_R, "Shoulder R (0x69)");

  // Setup Neck (Bus 1, 0x69 - Swapped and moved from 0x68)
  mpuNeck.setBus(i2c1);
  mpuNeck.setAddr(mpud::MPU_I2CADDRESS_AD0_HIGH);
  setupSensor(mpuNeck, SENSOR_NECK, "Neck (Bus 1, 0x69)");

  // Static Calibration
  ESP_LOGI(TAG, "Calibrating active sensors (1s)... keep device still.");
  MPU_t *mpus[NUM_SENSORS] = {&mpuNeck, &mpuL, &mpuR};
  const int samples = 50;
  for (int i = 0; i < NUM_SENSORS; i++) {
    gyroBias[i] = {0, 0, 0};
    if (!sensorActive[i]) continue;
    float sumX = 0, sumY = 0, sumZ = 0;
    int success_count = 0;
    for (int s = 0; s < samples; s++) {
      mpud::raw_axes_t accelRaw, gyroRaw;
      if (mpus[i]->motion(&accelRaw, &gyroRaw) == ESP_OK) {
        mpud::float_axes_t gyro =
            mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;
        success_count++;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (success_count > 0) {
      gyroBias[i] = {sumX / success_count, sumY / success_count, sumZ / success_count};
      ESP_LOGI(TAG, "Sensor %d Bias Calibrated (%d samples)", i, success_count);
    }
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

    mpud::raw_axes_t accelRaw, gyroRaw;
    esp_err_t res = mpus[i]->motion(&accelRaw, &gyroRaw);
    if (res == ESP_OK) {
      mpud::float_axes_t accel =
          mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
      mpud::float_axes_t gyro =
          mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);

      // Subtract bias
      gyro.x -= gyroBias[i].x;
      gyro.y -= gyroBias[i].y;
      gyro.z -= gyroBias[i].z;

      // Filter skip check
      if (accel.x == 0 && accel.y == 0 && accel.z == 0) {
        accel.z = 1.0f; 
      }

      mahony_update(&qStates[i], gyro.x, gyro.y, gyro.z, accel.x, accel.y,
                    accel.z, 0, 0, 0, dt, kp, ki);

      mo.sensors[i].qw = qStates[i].w;
      mo.sensors[i].qx = qStates[i].x;
      mo.sensors[i].qy = qStates[i].y;
      mo.sensors[i].qz = qStates[i].z;
    } else {
      mo.sensors[i].qw = 1.0f;
      static int errCount[NUM_SENSORS] = {0};
      if (errCount[i]++ % 100 == 0) {
        ESP_LOGE(TAG, "Sensor %d motion fail: %s", i, esp_err_to_name(res));
      }
    }
  }
  return mo;
}
