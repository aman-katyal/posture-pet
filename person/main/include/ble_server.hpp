#pragma once

#include <stdint.h>
#include "host/ble_hs.h"

struct Orientation {
    float roll;
    float pitch;
    float yaw;
    float qw;
    float qx;
    float qy;
    float qz;
};

struct MultiOrientation {
    Orientation sensors[3];
};

class BLEServer {
public:
    static BLEServer& getInstance();
    void init();
    void startAdvertising();
    void notifyMultiOrientation(const MultiOrientation& mo);
    bool isConnected() const { return connActive; }

private:
    BLEServer() = default;
    static int gapEventCb(struct ble_gap_event *event, void *arg);
    static void onSyncCb();
    
    bool connActive = false;
    uint16_t connHandle = 0;
    uint16_t charHandle = 0;
};
