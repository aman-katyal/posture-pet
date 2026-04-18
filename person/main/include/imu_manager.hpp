#pragma once
#include "MPU.hpp"
#include "I2Cbus.hpp"
#include "ble_server.hpp"
#include "quaternions.h"
#include "calibration.h"

enum SensorIdx {
    SENSOR_NECK = 0,
    SENSOR_SHOULDER_L = 1,
    SENSOR_SHOULDER_R = 2,
    NUM_SENSORS = 3
};

class IMUManager {
public:
    static IMUManager& getInstance() { static IMUManager instance; return instance; }
    void init();
    MultiOrientation update();
private:
    IMUManager() : mpuNeck(i2c1), mpuL(i2c0), mpuR(i2c0) {}
    
    // MPU Instances
    MPU_t mpuNeck; // Bus 1
    MPU_t mpuL;    // Bus 0, Addr 0x68
    MPU_t mpuR;    // Bus 0, Addr 0x69
    
    MahonyFilter qStates[NUM_SENSORS];
    int64_t lastTime = 0;
};
