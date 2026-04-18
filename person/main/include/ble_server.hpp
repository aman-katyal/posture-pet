#pragma once

#include <stdint.h>
#include "host/ble_hs.h"

struct Orientation {
    float roll;
    float pitch;
    float yaw;
};

class BLEServer {
public:
    static BLEServer& getInstance();
    void init();
    void startAdvertising();
    void notifyOrientation(const Orientation& orientation);
    bool isConnected() const { return connActive; }

private:
    BLEServer() = default;
    static int gapEventCb(struct ble_gap_event *event, void *arg);
    static void onSyncCb();
    
    bool connActive = false;
    uint16_t connHandle = 0;
    uint16_t charHandle = 0;
};
