/*
 * TeslaCam — ESP32-S3 Camera (OV5640)
 * BLE (NimBLE) JPEG streaming to ESP32-S3 Display
 *
 * Captures JPEG frames at HVGA (480×320) and sends them
 * via BLE GATT notifications to the connected display.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_camera.h"

// ── BLE ─────────────────────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME  "TeslaCam"
#define SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define FRAME_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CTRL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ── Protocol ────────────────────────────────────────────────────────────────────
#define HEADER_SIZE      8
#define CHUNK_PAYLOAD    240        // Fits in single LL packet (240+8+3+4 < 255)
#define CTRL_TIMEOUT_MS  5000       // Stop streaming if no heartbeat for 5s

// ── OV5640 pin mapping (ESP32-S3 WROOM / Freenove) ─────────────────────────────
#define PWDN_GPIO   -1
#define RESET_GPIO  -1
#define XCLK_GPIO   15
#define SIOD_GPIO    4
#define SIOC_GPIO    5
#define Y9_GPIO     16
#define Y8_GPIO     17
#define Y7_GPIO     18
#define Y6_GPIO     12
#define Y5_GPIO     10
#define Y4_GPIO      8
#define Y3_GPIO      9
#define Y2_GPIO     11
#define VSYNC_GPIO   6
#define HREF_GPIO    7
#define PCLK_GPIO   13

// ── BLE state ───────────────────────────────────────────────────────────────────
static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *frameChar = nullptr;
static NimBLECharacteristic *ctrlChar = nullptr;
static volatile bool deviceConnected = false;
static volatile bool streamEnabled = false;
static volatile unsigned long lastCtrlTime = 0;
static volatile uint16_t negotiatedMTU = 23;
static uint16_t frame_id = 0;

// ── BLE Callbacks ───────────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *s, ble_gap_conn_desc *desc) override {
        deviceConnected = true;
        /* Request fast connection params for throughput:
           min=6 (7.5ms), max=6 (7.5ms), latency=0, timeout=200 (2s) */
        s->updateConnParams(desc->conn_handle, 6, 6, 0, 200);
        /* Request 2M PHY for double throughput */
        ble_gap_set_prefered_le_phy(desc->conn_handle,
                                    BLE_GAP_LE_PHY_2M_MASK,
                                    BLE_GAP_LE_PHY_2M_MASK,
                                    BLE_HCI_LE_PHY_CODED_ANY);
        Serial.printf("[BLE] Client connected, handle=%u\n", desc->conn_handle);
    }
    void onDisconnect(NimBLEServer *s) override {
        deviceConnected = false;
        streamEnabled = false;
        NimBLEDevice::startAdvertising();
        Serial.println("[BLE] Client disconnected — re-advertising");
    }
    void onMTUChange(uint16_t mtu, ble_gap_conn_desc *desc) override {
        negotiatedMTU = mtu;
        Serial.printf("[BLE] MTU: %u (payload: %u)\n", mtu, mtu - 3);
    }
};

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar) override {
        std::string val = pChar->getValue();
        if (val.length() >= 4) {
            if (memcmp(val.data(), "STRT", 4) == 0) {
                if (!streamEnabled) Serial.println("[CTRL] START");
                streamEnabled = true;
                lastCtrlTime = millis();
            } else if (memcmp(val.data(), "STOP", 4) == 0) {
                if (streamEnabled) Serial.println("[CTRL] STOP");
                streamEnabled = false;
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────────
//  Camera init
// ─────────────────────────────────────────────────────────────────────────────────
bool initCamera() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO;
    cfg.pin_d1       = Y3_GPIO;
    cfg.pin_d2       = Y4_GPIO;
    cfg.pin_d3       = Y5_GPIO;
    cfg.pin_d4       = Y6_GPIO;
    cfg.pin_d5       = Y7_GPIO;
    cfg.pin_d6       = Y8_GPIO;
    cfg.pin_d7       = Y9_GPIO;
    cfg.pin_xclk     = XCLK_GPIO;
    cfg.pin_pclk     = PCLK_GPIO;
    cfg.pin_vsync    = VSYNC_GPIO;
    cfg.pin_href     = HREF_GPIO;
    cfg.pin_sccb_sda = SIOD_GPIO;
    cfg.pin_sccb_scl = SIOC_GPIO;
    cfg.pin_pwdn     = PWDN_GPIO;
    cfg.pin_reset    = RESET_GPIO;
    cfg.xclk_freq_hz = 24000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    if (psramFound()) {
        cfg.frame_size   = FRAMESIZE_QVGA;   // 320×240 – smaller for BLE throughput
        cfg.jpeg_quality = 12;                 // Standard quality
        cfg.fb_count     = 1;
        cfg.fb_location  = CAMERA_FB_IN_PSRAM;
        Serial.println("[CAM] PSRAM – QVGA, 1 buffer");
    } else {
        cfg.frame_size   = FRAMESIZE_QVGA;
        cfg.jpeg_quality = 12;
        cfg.fb_count     = 1;
        cfg.fb_location  = CAMERA_FB_IN_DRAM;
        Serial.println("[CAM] No PSRAM – QVGA, 1 buffer");
    }

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        Serial.printf("[CAM] Sensor PID: 0x%04X\n", s->id.PID);
        s->set_vflip(s, 1);     // Flip vertical (caméra de recul)
        s->set_hmirror(s, 0);
    }
    Serial.println("[CAM] Camera ready");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────────
//  BLE Server init
// ─────────────────────────────────────────────────────────────────────────────────
void initBLE() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(517);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Max TX power

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    /* Frame data characteristic — Notify, camera pushes JPEG chunks */
    frameChar = pService->createCharacteristic(
        FRAME_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    /* Control characteristic — Write, display sends STRT/STOP */
    ctrlChar = pService->createCharacteristic(
        CTRL_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    ctrlChar->setCallbacks(new CtrlCallbacks());

    pService->start();

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->start();

    Serial.printf("[BLE] Server started — advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Send one JPEG frame via BLE notifications
// ─────────────────────────────────────────────────────────────────────────────────
void sendFrame(camera_fb_t *fb) {
    if (!deviceConnected || negotiatedMTU < CHUNK_PAYLOAD + HEADER_SIZE + 3) return;

    uint16_t total_chunks = (fb->len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
    uint8_t packet[HEADER_SIZE + CHUNK_PAYLOAD];

    for (uint16_t i = 0; i < total_chunks; i++) {
        if (!deviceConnected) return;  // Bail if disconnected mid-frame

        uint32_t offset = (uint32_t)i * CHUNK_PAYLOAD;
        uint16_t size = (uint16_t)min((size_t)CHUNK_PAYLOAD, fb->len - (size_t)offset);

        // Header: frame_id(2) + chunk_id(2) + total_chunks(2) + chunk_size(2)
        memcpy(packet + 0, &frame_id,      2);
        memcpy(packet + 2, &i,             2);
        memcpy(packet + 4, &total_chunks,  2);
        memcpy(packet + 6, &size,          2);
        memcpy(packet + HEADER_SIZE, fb->buf + offset, size);

        frameChar->setValue(packet, HEADER_SIZE + size);
        frameChar->notify();

        // Pacing between notifications — 5ms balances throughput vs reliability
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // End-of-frame marker: chunkId = 0xFFFF
    {
        uint16_t endMarker = 0xFFFF;
        uint16_t zero = 0;
        memcpy(packet + 0, &frame_id,      2);
        memcpy(packet + 2, &endMarker,     2);
        memcpy(packet + 4, &total_chunks,  2);
        memcpy(packet + 6, &zero,          2);
        frameChar->setValue(packet, HEADER_SIZE);
        frameChar->notify();
    }

    if (frame_id < 5 || frame_id % 60 == 0) {
        uint32_t cksum = 0;
        for (size_t i = 0; i < fb->len; i++) cksum ^= ((uint32_t)fb->buf[i]) << ((i & 3) * 8);
        Serial.printf("[TX] fid=%u chunks=%u len=%u cksum=%08X MTU=%u\n",
                      frame_id, total_chunks, fb->len, cksum, negotiatedMTU);
    }
    frame_id++;
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Sensor sleep / wake (OV5640 software standby via register 0x3008 bit 6)
// ─────────────────────────────────────────────────────────────────────────────────
static void setSensorSleep(bool sleep) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    s->set_reg(s, 0x3008, 0x40, sleep ? 0x40 : 0x00);
    Serial.printf("[CAM] Sensor %s\n", sleep ? "sleeping" : "awake");
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Streaming task — core 1 (core 0 free for NimBLE host)
// ─────────────────────────────────────────────────────────────────────────────────
void streamTask(void *pvParameters) {
    Serial.println("[STREAM] Waiting for BLE client...");
    while (!deviceConnected) vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("[STREAM] Client connected — waiting for STRT");
    setSensorSleep(true);
    lastCtrlTime = millis();

    unsigned long frames = 0;
    unsigned long t0 = millis();
    bool wasStreaming = false;

    while (true) {
        /* Handle disconnect — wait for reconnection */
        if (!deviceConnected) {
            if (wasStreaming) {
                Serial.println("[STREAM] BLE disconnected — pausing");
                wasStreaming = false;
                setSensorSleep(true);
                neopixelWrite(48, 0, 0, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Wait for stream to be enabled via STRT command */
        if (!streamEnabled) {
            if (wasStreaming) {
                Serial.println("[STREAM] Paused — STOP received");
                wasStreaming = false;
                setSensorSleep(true);
                neopixelWrite(48, 0, 0, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Heartbeat timeout check */
        if (millis() - lastCtrlTime > CTRL_TIMEOUT_MS) {
            streamEnabled = false;
            neopixelWrite(48, 0, 0, 0);
            Serial.println("[CTRL] Heartbeat timeout — streaming disabled");
            continue;
        }

        /* Start streaming */
        if (!wasStreaming) {
            setSensorSleep(false);
            neopixelWrite(48, 0, 20, 0);  // Dim green
            vTaskDelay(pdMS_TO_TICKS(200));
            Serial.println("[STREAM] Streaming started (BLE)");
            wasStreaming = true;
            frames = 0;
            t0 = millis();
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Capture fail");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        /* No cache invalidation — ESP32-S3 GDMA writes PSRAM through cache,
           so the cache already has fresh data after fb_get() returns. */

        sendFrame(fb);
        esp_camera_fb_return(fb);

        frames++;
        if (frames % 60 == 0) {
            float fps = frames * 1000.0f / (millis() - t0);
            Serial.printf("[STREAM] %lu frames, %.1f fps\n", frames, fps);
        }

        // Brief yield between frames for BLE stack
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Setup & Loop
// ─────────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TeslaCam – Camera (BLE NimBLE) ===");

    pinMode(48, OUTPUT);
    neopixelWrite(48, 0, 0, 0);
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed – halting");
        while (true) delay(1000);
    }

    initBLE();

    // Launch streaming task on core 1 (core 0 for NimBLE host)
    xTaskCreatePinnedToCore(streamTask, "stream", 8192, NULL, 1, NULL, 1);
    Serial.println("[STREAM] Task started on core 1");
}

void loop() {
    delay(1000);
}
