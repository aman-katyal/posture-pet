#include "ble_server.hpp"
#include <string.h>
#include "esp_log.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "BLE_SERVER";

static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);

BLEServer& BLEServer::getInstance() {
    static BLEServer instance;
    return instance;
}

static int gattSvrAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

void BLEServer::init() {
    nimble_port_init();
    ble_hs_cfg.sync_cb = BLEServer::onSyncCb;
    ble_hs_cfg.gatts_register_cb = NULL;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Create zero-initialized definitions
    static struct ble_gatt_chr_def chrs[2];
    memset(chrs, 0, sizeof(chrs));
    chrs[0].uuid = &chr_uuid.u;
    chrs[0].access_cb = gattSvrAccessCb;
    chrs[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    chrs[0].val_handle = &charHandle;

    static struct ble_gatt_svc_def svcs[2];
    memset(svcs, 0, sizeof(svcs));
    svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    svcs[0].uuid = &svc_uuid.u;
    svcs[0].characteristics = chrs;

    ble_gatts_count_cfg(svcs);
    ble_gatts_add_svcs(svcs);
    ble_svc_gap_device_name_set("ESP32S3_PERSON");

    nimble_port_freertos_init([](void* p){
        nimble_port_run();
        nimble_port_freertos_deinit();
    });
}

void BLEServer::startAdvertising() {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESP32S3_PERSON";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 160; // 100ms (160 * 0.625ms)
    adv_params.itvl_max = 160; // 100ms
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, BLEServer::gapEventCb, NULL);
}

int BLEServer::gapEventCb(struct ble_gap_event *event, void *arg) {
    auto& self = BLEServer::getInstance();
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connected; status=%d", event->connect.status);
        if (event->connect.status == 0) {
            self.connActive = true;
            self.connHandle = event->connect.conn_handle;
        } else {
            self.startAdvertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        self.connActive = false;
        self.startAdvertising();
        break;
    }
    return 0;
}

void BLEServer::onSyncCb() {
    BLEServer::getInstance().startAdvertising();
}

void BLEServer::notifyMultiOrientation(const MultiOrientation& mo) {
    if (!connActive) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&mo, sizeof(mo));
    ble_gatts_notify_custom(connHandle, charHandle, om);
}
