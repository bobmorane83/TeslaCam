/*
 * TeslaCam — ESP32-S3 Display (JC3636W518C, ST77916, 360×360 round)
 * Wi-Fi Station + UDP receiver — decodes JPEG and displays on screen.
 *
 * Architecture:
 *   Core 0: UDP recv task (high priority, blocking recv, never misses packets)
 *   Core 1: Main loop — JPEG decode to screen buffer, then batch draw
 *
 * Triple-buffered JPEG reception prevents contention between recv and display.
 * Screen buffer in PSRAM allows one big QSPI transfer per frame instead of
 * thousands of tiny per-line draws.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <JPEGDEC.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <freertos/semphr.h>

/* ── Display hardware ────────────────────────────────────────────────── */
#define SCREEN_W 360
#define SCREEN_H 360
#define TFT_BL   15
#define TFT_RST  47

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    10 /* CS */, 9 /* SCK */, 11 /* D0 */, 12 /* D1 */, 13 /* D2 */, 14 /* D3 */);

Arduino_GFX *gfx = new Arduino_ST77916(
    bus, TFT_RST, 0 /* rotation */, true /* IPS */,
    SCREEN_W, SCREEN_H, 0, 0, 0, 0,
    st77916_150_init_operations,
    sizeof(st77916_150_init_operations));

/* ── WiFi ────────────────────────────────────────────────────────────── */
static const char *AP_SSID = "TeslaCam";
static const IPAddress STATIC_IP(192, 168, 4, 2);
static const IPAddress GATEWAY(192, 168, 4, 1);
static const IPAddress SUBNET(255, 255, 255, 0);

/* ── UDP ─────────────────────────────────────────────────────────────── */
#define UDP_PORT       5000
#define HEADER_SIZE    8
#define CHUNK_PAYLOAD  1400
#define MAX_FRAME_SIZE 65000

static int udp_sock = -1;

/* ── Triple-buffered JPEG reception ──────────────────────────────────── */
static uint8_t *jpegBuf[3];               // 3 JPEG buffers in PSRAM
static SemaphoreHandle_t tbMutex;
static int tbWrite   = 0;                 // Recv task writes here
static int tbReady   = -1;                // Last completed frame (-1 = none)
static int tbDisplay = -1;                // Display task reading (-1 = none)
static size_t tbReadyLen = 0;

// Recv-task-only state (no mutex needed)
static uint16_t currentFrameId  = 0;
static uint16_t expectedChunks  = 0;
static uint16_t receivedChunks  = 0;
static size_t   recvFrameLen    = 0;

/* ── Screen buffer for batch display ─────────────────────────────────── */
static uint16_t *screenBuf;               // SCREEN_W × SCREEN_H in PSRAM

/* ── Timeout ─────────────────────────────────────────────────────────── */
static volatile unsigned long lastFrameTime = 0;
#define FRAME_TIMEOUT_MS 500
static bool screenBlank = true;

/* ── Crop / center parameters ────────────────────────────────────────── */
static int cropX    = 0;   // Source image crop offset X
static int cropY    = 0;   // Source image crop offset Y
static int screenOX = 0;   // Screen draw offset X (centering)
static int screenOY = 0;   // Screen draw offset Y (centering)
static int drawW    = SCREEN_W;  // Actual drawn width
static int drawH    = SCREEN_H;  // Actual drawn height

/* ── JPEGDEC draw callback — writes to screenBuf (fast memcpy) ───── */
static int jpegDrawCallback(JPEGDRAW *pDraw) {
    int srcX1 = pDraw->x;
    int srcY1 = pDraw->y;
    int srcX2 = pDraw->x + pDraw->iWidth;
    int srcY2 = pDraw->y + pDraw->iHeight;

    int cX2 = cropX + drawW;
    int cY2 = cropY + drawH;

    int ix1 = max(srcX1, cropX);
    int iy1 = max(srcY1, cropY);
    int ix2 = min(srcX2, cX2);
    int iy2 = min(srcY2, cY2);

    if (ix1 >= ix2 || iy1 >= iy2) return 1;

    int sx = ix1 - cropX + screenOX;
    int sy = iy1 - cropY + screenOY;
    int w  = ix2 - ix1;
    int h  = iy2 - iy1;
    int offX = ix1 - srcX1;
    int offY = iy1 - srcY1;

    for (int row = 0; row < h; row++) {
        uint16_t *src = pDraw->pPixels + (offY + row) * pDraw->iWidth + offX;
        uint16_t *dst = screenBuf + (sy + row) * SCREEN_W + sx;
        memcpy(dst, src, w * 2);
    }
    return 1;
}

/* ── Connect to Camera SoftAP ────────────────────────────────────────── */
void connectToCamera() {
    WiFi.mode(WIFI_STA);
    WiFi.config(STATIC_IP, GATEWAY, SUBNET);
    WiFi.begin(AP_SSID);

    Serial.printf("[WIFI] Connecting to %s", AP_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > 15000) {
            Serial.println("\n[WIFI] Timeout – restarting");
            ESP.restart();
        }
        Serial.print(".");
        delay(500);
    }
    Serial.printf("\n[WIFI] Connected – IP: %s\n", WiFi.localIP().toString().c_str());
}

/* ── Process one incoming UDP packet (called from recv task only) ───── */
void processPacket(uint8_t *data, size_t len) {
    if (len < HEADER_SIZE) return;

    uint16_t frameId, chunkId, totalChunks, chunkSize;
    memcpy(&frameId,     data + 0, 2);
    memcpy(&chunkId,     data + 2, 2);
    memcpy(&totalChunks, data + 4, 2);
    memcpy(&chunkSize,   data + 6, 2);

    if (chunkSize > CHUNK_PAYLOAD || chunkSize + HEADER_SIZE > len) return;

    // New frame detected — reset receiver state
    if (frameId != currentFrameId) {
        currentFrameId = frameId;
        expectedChunks = totalChunks;
        receivedChunks = 0;
        recvFrameLen   = 0;
    }

    uint32_t offset = (uint32_t)chunkId * CHUNK_PAYLOAD;
    if (offset + chunkSize > MAX_FRAME_SIZE) return;

    memcpy(jpegBuf[tbWrite] + offset, data + HEADER_SIZE, chunkSize);

    size_t endPos = offset + chunkSize;
    if (endPos > recvFrameLen) recvFrameLen = endPos;
    receivedChunks++;

    // Frame complete — publish to triple buffer
    if (receivedChunks == expectedChunks) {
        xSemaphoreTake(tbMutex, portMAX_DELAY);
        tbReady    = tbWrite;
        tbReadyLen = recvFrameLen;
        // Pick next write slot (not ready, not display)
        for (int i = 0; i < 3; i++) {
            if (i != tbReady && i != tbDisplay) { tbWrite = i; break; }
        }
        xSemaphoreGive(tbMutex);

        lastFrameTime = millis();
        receivedChunks = 0;
        recvFrameLen   = 0;
    }
}

/* ── UDP receive task — runs on core 0, blocking recv ────────────────── */
void udpRecvTask(void *pvParameters) {
    uint8_t buf[HEADER_SIZE + CHUNK_PAYLOAD];
    while (true) {
        int len = recvfrom(udp_sock, buf, sizeof(buf), 0, NULL, NULL); // blocking
        if (len > 0) processPacket(buf, len);
    }
}

/* ── Grab latest frame for display (returns buf index or -1) ─────────── */
int grabReadyFrame(size_t *outLen) {
    xSemaphoreTake(tbMutex, portMAX_DELAY);
    if (tbReady < 0) {
        xSemaphoreGive(tbMutex);
        return -1;
    }
    tbDisplay = tbReady;
    tbReady   = -1;
    *outLen   = tbReadyLen;
    xSemaphoreGive(tbMutex);
    return tbDisplay;
}

/* ── Release display buffer ──────────────────────────────────────────── */
void releaseDisplayBuffer() {
    xSemaphoreTake(tbMutex, portMAX_DELAY);
    tbDisplay = -1;
    xSemaphoreGive(tbMutex);
}

/* ── Decode JPEG into screenBuf, then batch-draw to display ──────────── */
void decodeAndDisplay(int bufIdx, size_t len) {
    static JPEGDEC jpeg;

    if (!jpeg.openRAM(jpegBuf[bufIdx], len, jpegDrawCallback)) {
        Serial.println("[JPEG] Decode failed");
        releaseDisplayBuffer();
        return;
    }

    int imgW = jpeg.getWidth();
    int imgH = jpeg.getHeight();

    // Compute crop + centering
    if (imgW >= SCREEN_W) {
        cropX    = (imgW - SCREEN_W) / 2;
        screenOX = 0;
        drawW    = SCREEN_W;
    } else {
        cropX    = 0;
        screenOX = (SCREEN_W - imgW) / 2;
        drawW    = imgW;
    }
    if (imgH >= SCREEN_H) {
        cropY    = (imgH - SCREEN_H) / 2;
        screenOY = 0;
        drawH    = SCREEN_H;
    } else {
        cropY    = 0;
        screenOY = (SCREEN_H - imgH) / 2;
        drawH    = imgH;
    }

    jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
    jpeg.decode(0, 0, 0);
    jpeg.close();

    releaseDisplayBuffer();

    // One batch draw — the entire 360×360 screen in one QSPI transfer
    gfx->draw16bitRGBBitmap(0, 0, screenBuf, SCREEN_W, SCREEN_H);

    if (screenBlank) screenBlank = false;
}

/* ── Setup ───────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TeslaCam – Display (Station + UDP) ===");

    // Allocate triple JPEG buffers in PSRAM
    for (int i = 0; i < 3; i++) {
        jpegBuf[i] = (uint8_t *)heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
        if (!jpegBuf[i]) {
            Serial.printf("[FATAL] PSRAM alloc failed for jpegBuf[%d]\n", i);
            while (true) delay(1000);
        }
    }
    // Screen buffer: 360×360 × 2 bytes = 259,200 bytes in PSRAM
    screenBuf = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM);
    if (!screenBuf) {
        Serial.println("[FATAL] Screen buffer alloc failed");
        while (true) delay(1000);
    }
    memset(screenBuf, 0, SCREEN_W * SCREEN_H * 2);
    Serial.println("[MEM] 3×65KB JPEG + 259KB screen buffer allocated");

    tbMutex = xSemaphoreCreateMutex();

    // Display init
    if (!gfx->begin()) {
        Serial.println("[FATAL] GFX begin failed");
        while (true) delay(1000);
    }
    gfx->fillScreen(0x0000);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    Serial.println("[GFX] Display OK");

    // Connect to Camera AP
    connectToCamera();
    WiFi.setSleep(false);

    // Create raw lwip UDP socket (blocking mode for recv task)
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        Serial.println("[FATAL] UDP socket creation failed");
        while (true) delay(1000);
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        Serial.println("[FATAL] UDP bind failed");
        while (true) delay(1000);
    }

    // Increase receive buffer
    int rcvbuf = 65536;
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    Serial.printf("[UDP] Listening on port %d (blocking)\n", UDP_PORT);

    // Launch UDP recv task on core 0, high priority
    xTaskCreatePinnedToCore(udpRecvTask, "udpRecv", 4096, NULL, 2, NULL, 0);
    Serial.println("[RECV] UDP recv task started on core 0");

    lastFrameTime = millis();
}

/* ── Loop (core 1) — decode + display ────────────────────────────────── */
static unsigned long dispFrames = 0;
static unsigned long dispT0 = 0;

void loop() {
    size_t len;
    int idx = grabReadyFrame(&len);

    if (idx >= 0) {
        decodeAndDisplay(idx, len);

        dispFrames++;
        if (dispFrames == 1) dispT0 = millis();
        if (dispFrames % 60 == 0) {
            float fps = dispFrames * 1000.0f / (millis() - dispT0);
            Serial.printf("[DISPLAY] %lu frames, %.1f fps\n", dispFrames, fps);
        }
    } else {
        // No frame ready — yield briefly to avoid busy-spin
        taskYIELD();
    }

    // Timeout: black screen if no frame for 500ms
    if (!screenBlank && (millis() - lastFrameTime > FRAME_TIMEOUT_MS)) {
        gfx->fillScreen(0x0000);
        screenBlank = true;
        Serial.println("[TIMEOUT] No frame – screen blanked");
    }
}
