#pragma once
#include "MPU.hpp"
#include "I2Cbus.hpp"
#include "ble_server.hpp"
#include "quaternions.h"
#include "calibration.h"

class IMUManager {
public:
    static IMUManager& getInstance() { static IMUManager instance; return instance; }
    void init();
    Orientation update();
private:
    IMUManager() : mpu(i2c0) {}
    MPU_t mpu;
    uint8_t magAsa[3];
    MahonyFilter qState;
    int64_t lastTime = 0;
    int64_t startTime = 0;
};
