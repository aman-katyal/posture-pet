#pragma once
#include "MPU.hpp"
#include "I2Cbus.hpp"
#include "ble_server.hpp"
#include "quaternions.h"

class IMUManager {
public:
    static IMUManager& getInstance() { static IMUManager instance; return instance; }
    void init();
    Orientation update();
private:
    IMUManager() : mpu(i2c0) {}
    MPU_t mpu;
    Quaternion qState;
    int64_t lastTime = 0;
};
