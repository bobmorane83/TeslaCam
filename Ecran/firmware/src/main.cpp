/*
 * TeslaCam — ESP32-S3 Display (JC3636W518C, ST77916, 360×360 round)
 * Wi-Fi Station + UDP receiver — decodes JPEG and displays on screen.
 *
 * Connects to the Camera SoftAP "TeslaCam" and receives JPEG frames
 * via UDP chunks on port 5000. Decodes with JPEGDEC and displays via
 * Arduino_GFX with center-crop from VGA (640×480) to 360×360.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <JPEGDEC.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"

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

/* ── Double-buffered frame reception ─────────────────────────────────── */
static uint8_t *frameBuf[2];       // Two JPEG buffers in PSRAM
static uint8_t  recvIdx    = 0;    // Buffer being filled
static uint8_t  dispIdx    = 1;    // Buffer ready for display
static size_t   dispLen    = 0;    // Size of last complete frame
static volatile bool frameReady = false;

static uint16_t currentFrameId  = 0;
static uint16_t expectedChunks  = 0;
static uint16_t receivedChunks  = 0;
static size_t   recvFrameLen    = 0;

/* ── Timeout ─────────────────────────────────────────────────────────── */
static unsigned long lastFrameTime = 0;
#define FRAME_TIMEOUT_MS 500
static bool screenBlank = true;

/* ── Crop parameters (VGA 640×480 → 360×360 center crop) ────────────── */
static int cropX = 0;   // Computed after first JPEG decode
static int cropY = 0;

/* ── JPEGDEC draw callback ───────────────────────────────────────────── */
static int jpegDrawCallback(JPEGDRAW *pDraw) {
    // pDraw->x, pDraw->y: position in the source image
    // pDraw->iWidth, pDraw->iHeight: block size
    // pDraw->pPixels: RGB565 pixel data

    // Compute intersection with crop region
    int srcX1 = pDraw->x;
    int srcY1 = pDraw->y;
    int srcX2 = pDraw->x + pDraw->iWidth;
    int srcY2 = pDraw->y + pDraw->iHeight;

    int cX1 = cropX;
    int cY1 = cropY;
    int cX2 = cropX + SCREEN_W;
    int cY2 = cropY + SCREEN_H;

    // Clip to crop region
    int ix1 = max(srcX1, cX1);
    int iy1 = max(srcY1, cY1);
    int ix2 = min(srcX2, cX2);
    int iy2 = min(srcY2, cY2);

    if (ix1 >= ix2 || iy1 >= iy2) return 1; // No overlap

    // Screen coordinates
    int screenX = ix1 - cropX;
    int screenY = iy1 - cropY;
    int drawW   = ix2 - ix1;
    int drawH   = iy2 - iy1;

    // Offset into the MCU block data
    int offX = ix1 - srcX1;
    int offY = iy1 - srcY1;

    // Draw line by line from the block
    for (int row = 0; row < drawH; row++) {
        uint16_t *src = pDraw->pPixels + (offY + row) * pDraw->iWidth + offX;
        gfx->draw16bitRGBBitmap(screenX, screenY + row, src, drawW, 1);
    }

    return 1; // Continue decoding
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

/* ── Process one incoming UDP packet ─────────────────────────────────── */
void processPacket(uint8_t *data, size_t len) {
    if (len < HEADER_SIZE) return;

    uint16_t frameId, chunkId, totalChunks, chunkSize;
    memcpy(&frameId,     data + 0, 2);
    memcpy(&chunkId,     data + 2, 2);
    memcpy(&totalChunks, data + 4, 2);
    memcpy(&chunkSize,   data + 6, 2);

    // Validate chunk size
    if (chunkSize > CHUNK_PAYLOAD || chunkSize + HEADER_SIZE > len) return;

    // New frame detected — reset receiver state
    if (frameId != currentFrameId) {
        currentFrameId = frameId;
        expectedChunks = totalChunks;
        receivedChunks = 0;
        recvFrameLen   = 0;
    }

    // Copy chunk payload into receive buffer
    uint32_t offset = (uint32_t)chunkId * CHUNK_PAYLOAD;
    if (offset + chunkSize > MAX_FRAME_SIZE) return;  // Safety check

    memcpy(frameBuf[recvIdx] + offset, data + HEADER_SIZE, chunkSize);

    // Track the total size (last chunk determines actual end)
    size_t endPos = offset + chunkSize;
    if (endPos > recvFrameLen) recvFrameLen = endPos;

    receivedChunks++;

    // Frame complete?
    if (receivedChunks == expectedChunks) {
        // Swap buffers
        dispLen = recvFrameLen;
        uint8_t tmp = recvIdx;
        recvIdx = dispIdx;
        dispIdx = tmp;
        frameReady = true;
        lastFrameTime = millis();

        receivedChunks = 0;
        recvFrameLen = 0;
    }
}

/* ── Decode and display a complete JPEG frame ────────────────────────── */
void decodeAndDisplay() {
    static JPEGDEC jpeg;

    if (jpeg.openRAM(frameBuf[dispIdx], dispLen, jpegDrawCallback)) {
        int imgW = jpeg.getWidth();
        int imgH = jpeg.getHeight();

        // Compute center crop offsets
        cropX = max(0, (imgW - SCREEN_W) / 2);
        cropY = max(0, (imgH - SCREEN_H) / 2);

        jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
        jpeg.decode(0, 0, 0); // Full decode, no scaling
        jpeg.close();

        if (screenBlank) screenBlank = false;
    } else {
        Serial.println("[JPEG] Decode failed");
    }
}

/* ── Setup ───────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TeslaCam – Display (Station + UDP) ===");

    // Allocate double buffers in PSRAM
    frameBuf[0] = (uint8_t *)heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    frameBuf[1] = (uint8_t *)heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    if (!frameBuf[0] || !frameBuf[1]) {
        Serial.println("[FATAL] PSRAM alloc failed");
        while (true) delay(1000);
    }
    Serial.println("[MEM] Frame buffers allocated (2 × 65 KB PSRAM)");

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

    // Create raw lwip UDP socket
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

    // Set receive timeout to avoid blocking forever
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms timeout
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Increase receive buffer
    int rcvbuf = 65536;
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);

    lastFrameTime = millis();
}

/* ── Loop ────────────────────────────────────────────────────────────── */
void loop() {
    // Receive pending UDP packets via raw lwip socket
    uint8_t buf[HEADER_SIZE + CHUNK_PAYLOAD];
    int maxPackets = 50;

    while (maxPackets-- > 0) {
        int len = recvfrom(udp_sock, buf, sizeof(buf), 0, NULL, NULL);
        if (len <= 0) break;  // No more packets or timeout
        processPacket(buf, len);
    }

    // Display frame if ready
    if (frameReady) {
        frameReady = false;
        decodeAndDisplay();
    }

    // Timeout: black screen if no frame for 500ms
    if (!screenBlank && (millis() - lastFrameTime > FRAME_TIMEOUT_MS)) {
        gfx->fillScreen(0x0000);
        screenBlank = true;
        Serial.println("[TIMEOUT] No frame – screen blanked");
    }

    delay(1);  // Yield to FreeRTOS scheduler / WDT
}
