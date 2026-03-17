/*
 * TeslaCam — ESP32-S3 Display (JC3636W518C, ST77916, 360×360 round)
 * Wi-Fi Station + UDP receiver — decodes JPEG and displays on screen.
 *
 * Architecture:
 *   Core 0: UDP recv task (high priority, blocking recv, never misses packets)
 *   Core 1: Main loop — touch polling, JPEG decode to screen buffer, batch draw
 *
 * Touch on screen toggles streaming START/STOP via UDP control channel (port 5001).
 * Camera is in sleep mode by default — only captures on STRT command.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <JPEGDEC.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <freertos/semphr.h>
#include <Wire.h>

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

/* ── UDP video (port 5000) ───────────────────────────────────────────── */
#define UDP_PORT       5000
#define HEADER_SIZE    8
#define CHUNK_PAYLOAD  1400
#define MAX_FRAME_SIZE 65000

static int udp_sock = -1;

/* ── UDP control (port 5001) — Écran → Camera ────────────────────────── */
#define CTRL_PORT      5001
#define CTRL_RESEND_MS 2000        // Resend STRT every 2s as heartbeat
static int ctrl_sock = -1;
static struct sockaddr_in ctrl_dest;

/* ── Touch controller CST816S (I2C 0x15, SDA=7, SCL=8) ──────────────── */
#define TOUCH_SDA  7
#define TOUCH_SCL  8
#define TOUCH_ADDR 0x15
#define TOUCH_DEBOUNCE_MS 300      // Ignore touches within 300ms of last toggle

static bool touchedPrev       = false;   // Previous touch state (for edge detect)
static unsigned long lastToggleMs = 0;   // Debounce timestamp

/* ── Streaming state ─────────────────────────────────────────────────── */
static bool streamActive      = false;   // Are we requesting streaming?
static unsigned long lastCtrlSendMs = 0; // Last STRT/STOP sent timestamp
static bool idleScreenDrawn   = false;   // Have we drawn the idle screen?

/* ── Triple-buffered JPEG reception ──────────────────────────────────── */
static uint8_t *jpegBuf[3];
static SemaphoreHandle_t tbMutex;
static int tbWrite   = 0;
static int tbReady   = -1;
static int tbDisplay = -1;
static size_t tbReadyLen = 0;

static uint16_t currentFrameId  = 0;
static uint16_t expectedChunks  = 0;
static uint16_t receivedChunks  = 0;
static size_t   recvFrameLen    = 0;

/* ── Screen buffer for batch display ─────────────────────────────────── */
static uint16_t *screenBuf;

/* ── Timeout ─────────────────────────────────────────────────────────── */
static volatile unsigned long lastFrameTime = 0;
#define FRAME_TIMEOUT_MS 2000   // 2s — allows Camera 2s warm-up after STRT
static bool screenBlank = true;
static bool firstFrameReceived = false;  // True after first frame since last STRT

/* ── Crop / center parameters ────────────────────────────────────────── */
static int cropX    = 0;
static int cropY    = 0;
static int screenOX = 0;
static int screenOY = 0;
static int drawW    = SCREEN_W;
static int drawH    = SCREEN_H;

/* ── Read CST816S touch state (true = finger touching) ───────────────── */
bool isTouched() {
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(0x02);  // Register: number of touch points
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)1) != 1) return false;
    uint8_t numPoints = Wire.read();
    return (numPoints > 0 && numPoints <= 5);
}

/* ── Send control command to Camera ──────────────────────────────────── */
void sendCtrlCommand(const char *cmd) {
    if (ctrl_sock < 0) return;
    sendto(ctrl_sock, cmd, 4, 0,
           (struct sockaddr *)&ctrl_dest, sizeof(ctrl_dest));
}

/* ── Draw idle screen (camera off) ───────────────────────────────────── */
void drawIdleScreen() {
    gfx->fillScreen(0x0000);
    gfx->setTextColor(0xFFFF);
    gfx->setTextSize(2);

    // Center "Appuyez" on screen
    const char *line1 = "Appuyez pour";
    const char *line2 = "activer la camera";
    int16_t x1, y1;
    uint16_t tw, th;
    gfx->getTextBounds(line1, 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor((SCREEN_W - tw) / 2, SCREEN_H / 2 - th - 4);
    gfx->print(line1);
    gfx->getTextBounds(line2, 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor((SCREEN_W - tw) / 2, SCREEN_H / 2 + 4);
    gfx->print(line2);

    idleScreenDrawn = true;
    screenBlank = false;
}

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

    if (receivedChunks == expectedChunks) {
        xSemaphoreTake(tbMutex, portMAX_DELAY);
        tbReady    = tbWrite;
        tbReadyLen = recvFrameLen;
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
        int len = recvfrom(udp_sock, buf, sizeof(buf), 0, NULL, NULL);
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

    gfx->draw16bitRGBBitmap(0, 0, screenBuf, SCREEN_W, SCREEN_H);

    if (screenBlank) screenBlank = false;
    idleScreenDrawn = false;
    firstFrameReceived = true;
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

    // Touch controller init (CST816S on I2C 0x15)
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    delay(50);
    Serial.println("[TOUCH] CST816S init on SDA=7, SCL=8");

    // Connect to Camera AP
    connectToCamera();
    WiFi.setSleep(false);

    // Video UDP socket (blocking mode for recv task)
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

    int rcvbuf = 65536;
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    Serial.printf("[UDP] Video listening on port %d\n", UDP_PORT);

    // Control UDP socket (for sending STRT/STOP to Camera)
    ctrl_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ctrl_sock < 0) {
        Serial.println("[WARN] Control socket creation failed");
    }
    memset(&ctrl_dest, 0, sizeof(ctrl_dest));
    ctrl_dest.sin_family = AF_INET;
    ctrl_dest.sin_port = htons(CTRL_PORT);
    ctrl_dest.sin_addr.s_addr = inet_addr("192.168.4.1");
    Serial.printf("[CTRL] Control socket → 192.168.4.1:%d\n", CTRL_PORT);

    // Launch UDP recv task on core 0
    xTaskCreatePinnedToCore(udpRecvTask, "udpRecv", 4096, NULL, 2, NULL, 0);
    Serial.println("[RECV] UDP recv task started on core 0");

    lastFrameTime = millis();

    // Draw idle screen on startup
    drawIdleScreen();
    Serial.println("[UI] Idle screen — touch to activate");
}

/* ── Loop (core 1) — touch + decode + display ────────────────────────── */
static unsigned long dispFrames = 0;
static unsigned long dispT0 = 0;

void loop() {
    unsigned long now = millis();

    // ── Touch polling with edge detection (released = toggle) ────────
    bool touchedNow = isTouched();
    if (touchedPrev && !touchedNow && (now - lastToggleMs > TOUCH_DEBOUNCE_MS)) {
        // Finger just released → toggle
        streamActive = !streamActive;
        lastToggleMs = now;

        if (streamActive) {
            sendCtrlCommand("STRT");
            lastCtrlSendMs = now;
            lastFrameTime = now;      // Reset timeout baseline on activation
            firstFrameReceived = false;
            Serial.println("[TOUCH] Activated → STRT sent");
        } else {
            sendCtrlCommand("STOP");
            lastCtrlSendMs = now;
            firstFrameReceived = false;
            Serial.println("[TOUCH] Deactivated → STOP sent");
            // Draw idle screen after short delay to let last frame clear
            drawIdleScreen();
        }
    }
    touchedPrev = touchedNow;

    // ── Heartbeat: resend STRT every 2s while active ─────────────────
    if (streamActive && (now - lastCtrlSendMs >= CTRL_RESEND_MS)) {
        sendCtrlCommand("STRT");
        lastCtrlSendMs = now;
    }

    // ── Display frames only when streaming is active ─────────────────
    if (streamActive) {
        size_t len;
        int idx = grabReadyFrame(&len);

        if (idx >= 0) {
            decodeAndDisplay(idx, len);

            dispFrames++;
            if (dispFrames == 1) dispT0 = now;
            if (dispFrames % 60 == 0) {
                float fps = dispFrames * 1000.0f / (now - dispT0);
                Serial.printf("[DISPLAY] %lu frames, %.1f fps\n", dispFrames, fps);
            }
        } else {
            taskYIELD();
        }

        // Timeout: show idle only after first frame has arrived and then gone silent
        if (firstFrameReceived && !idleScreenDrawn &&
            (now - lastFrameTime > FRAME_TIMEOUT_MS)) {
            drawIdleScreen();
            Serial.println("[TIMEOUT] No frame – idle screen");
        }
    } else {
        // Not streaming — draw idle if not already shown
        if (!idleScreenDrawn) {
            drawIdleScreen();
        }
        // Drain any stale frames from the triple buffer
        size_t dummyLen;
        int idx = grabReadyFrame(&dummyLen);
        if (idx >= 0) releaseDisplayBuffer();

        delay(20);  // Save CPU when idle
    }
}
