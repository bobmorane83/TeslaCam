/*
 * TeslaCam -- ESP32-S3 Display (JC3636W518C, ST77916, 360x360 round)
 * Wi-Fi Station + UDP receiver -- decodes JPEG and displays on screen.
 * Dashboard with CAN data from Bridge via ESP-NOW.
 *
 * Architecture:
 *   Core 0: UDP recv task (high priority, blocking recv, never misses packets)
 *   Core 1: Main loop -- touch polling, dashboard render or JPEG decode + display
 *
 * Touch toggles between dashboard (with CAN data) and camera stream.
 * Camera is in sleep mode by default -- only captures on STRT command.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <JPEGDEC.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <freertos/semphr.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

/* ======================================================================
 *  DISPLAY HARDWARE
 * ====================================================================== */
#define SCREEN_W 360
#define SCREEN_H 360
#define TFT_BL   15
#define TFT_RST  47
#define CX       180
#define CY       180

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    10, 9, 11, 12, 13, 14);

Arduino_GFX *gfx = new Arduino_ST77916(
    bus, TFT_RST, 0, true,
    SCREEN_W, SCREEN_H, 0, 0, 0, 0,
    st77916_150_init_operations,
    sizeof(st77916_150_init_operations));

/* ======================================================================
 *  WiFi / UDP / TOUCH
 * ====================================================================== */
static const char *AP_SSID = "TeslaCam";
static const IPAddress STATIC_IP(192, 168, 4, 2);
static const IPAddress GATEWAY(192, 168, 4, 1);
static const IPAddress SUBNET(255, 255, 255, 0);

#define UDP_PORT       5000
#define HEADER_SIZE    8
#define CHUNK_PAYLOAD  1400
#define MAX_FRAME_SIZE 65000
static int udp_sock = -1;

#define CTRL_PORT      5001
#define CTRL_RESEND_MS 2000
static int ctrl_sock = -1;
static struct sockaddr_in ctrl_dest;

#define TOUCH_SDA  7
#define TOUCH_SCL  8
#define TOUCH_ADDR 0x15
#define TOUCH_DEBOUNCE_MS 300

static bool touchedPrev       = false;
static unsigned long lastToggleMs = 0;
static bool streamActive      = false;
static unsigned long lastCtrlSendMs = 0;
static bool idleScreenDrawn   = false;
static bool wifiConnected     = false;
static bool networkReady      = false;

/* ======================================================================
 *  TRIPLE-BUFFERED JPEG RECEPTION
 * ====================================================================== */
static uint8_t *jpegBuf[3];
static SemaphoreHandle_t tbMutex;
static int tbWrite = 0, tbReady = -1, tbDisplay = -1;
static size_t tbReadyLen = 0;
static uint16_t currentFrameId = 0, expectedChunks = 0, receivedChunks = 0;
static size_t recvFrameLen = 0;

/* ======================================================================
 *  SCREEN BUFFERS
 * ====================================================================== */
static uint16_t *screenBuf;   // 360x360 working buffer (PSRAM)
static uint16_t *bezelBuf;    // 360x360 pre-rendered static bezel (PSRAM)

static volatile unsigned long lastFrameTime = 0;
#define FRAME_TIMEOUT_MS 2000
static bool screenBlank = true;
static bool firstFrameReceived = false;

static int cropX = 0, cropY = 0, screenOX = 0, screenOY = 0;
static int drawW = SCREEN_W, drawH = SCREEN_H;

/* ======================================================================
 *  CAN SIGNAL DATA (from Bridge via ESP-NOW)
 * ====================================================================== */
typedef struct __attribute__((packed)) {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint32_t timestamp;
    uint8_t  is_extended;
} ESP_CAN_Message_t;

#define CAN_ID_SPEED     0x257   // DI_uiSpeed
#define CAN_ID_GEAR      0x118   // DI_gear
#define CAN_ID_RANGE_SOC 0x33A   // UI_Range, UI_SOC
#define CAN_ID_BATT_TEMP 0x312   // BMSmaxPackTemperature
#define CAN_ID_REAR_PWR  0x266   // RearPower266
#define CAN_ID_FRONT_PWR 0x2E5   // FrontPower2E5
#define CAN_ID_BMS_SOC   0x292   // SOCUI292
#define CAN_ID_HEARTBEAT 0xFFF

#define GEAR_INVALID 0
#define GEAR_P       1
#define GEAR_R       2
#define GEAR_N       3
#define GEAR_D       4
#define GEAR_SNA     7

static volatile struct {
    uint16_t uiSpeed;
    uint8_t  gear;
    uint8_t  soc;
    uint16_t rangeKm;
    float    battTempMax;
    float    rearPowerKw;
    float    frontPowerKw;
} canData = { 0, GEAR_P, 0, 0, 25.0f, 0.0f, 0.0f };

#define BRIDGE_TIMEOUT_MS 5000
static volatile unsigned long lastBridgeMsg = 0;
static volatile bool bridgeEverSeen = false;

static float regenKwSmooth = 0;
#define MAX_REGEN_KW 50.0f

/* ======================================================================
 *  COLOR PALETTE (RGB565)
 * ====================================================================== */
#define COL_BG        0x0000
#define COL_TEAL      0x0738   // #00e5c8
#define COL_TEAL_DIM  0x02B2
#define COL_TEAL_VDIM 0x0151
#define COL_AMBER     0xFD40   // #ffaa00
#define COL_RED       0xF9A6   // #ff3b3b
#define COL_BLUE      0x1C7F   // #1a8fff
#define COL_WHITE     0xFFFF
#define COL_TEXTDIM   0x3B0E   // #3a5a70
#define COL_GREEN     0x07E0

static uint16_t speedColor(int speed) {
    float t = speed / 180.0f;
    if (t > 1.0f) t = 1.0f;
    uint8_t r, g, b;
    if (t < 0.55f) {
        float f = t / 0.55f;
        r = (uint8_t)(f * 255);
        g = 230 + (uint8_t)((1.0f - f) * 25);
        b = (uint8_t)((1.0f - f) * 200);
    } else {
        float f = (t - 0.55f) / 0.45f;
        r = 255;
        g = (uint8_t)((1.0f - f) * 200);
        b = 0;
    }
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

static uint16_t socColor(uint8_t soc) {
    if (soc > 50) return COL_TEAL;
    if (soc > 20) return COL_AMBER;
    return COL_RED;
}

static uint16_t tempColor(float t) {
    if (t < 20.0f) return COL_BLUE;
    if (t < 42.0f) return COL_AMBER;
    return COL_RED;
}

/* ======================================================================
 *  SCREEN BUFFER DRAWING PRIMITIVES
 * ====================================================================== */
static inline void bufPixel(int x, int y, uint16_t c) {
    if ((unsigned)x < SCREEN_W && (unsigned)y < SCREEN_H)
        screenBuf[y * SCREEN_W + x] = c;
}

static void bufFillRect(int x, int y, int w, int h, uint16_t c) {
    int x0 = max(x, 0), y0 = max(y, 0);
    int x1 = min(x + w, SCREEN_W), y1 = min(y + h, SCREEN_H);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            screenBuf[j * SCREEN_W + i] = c;
}

static void bufFillCircle(int cx, int cy, int r, uint16_t c) {
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrtf((float)(r2 - dy * dy));
        int y = cy + dy;
        if ((unsigned)y >= SCREEN_H) continue;
        int x0 = max(cx - dx, 0);
        int x1 = min(cx + dx, SCREEN_W - 1);
        for (int x = x0; x <= x1; x++)
            screenBuf[y * SCREEN_W + x] = c;
    }
}

/* Draw a thick arc (ring segment) from angle a0 to a1 (radians) */
static void bufFillArc(int cx, int cy, int ri, int ro, float a0, float a1, uint16_t c) {
    if (a1 < a0) a1 += 2.0f * M_PI;
    int ri2 = ri * ri, ro2 = ro * ro;
    for (int y = cy - ro - 1; y <= cy + ro + 1; y++) {
        if ((unsigned)y >= SCREEN_H) continue;
        for (int x = cx - ro - 1; x <= cx + ro + 1; x++) {
            if ((unsigned)x >= SCREEN_W) continue;
            int dx = x - cx, dy = y - cy;
            int d2 = dx * dx + dy * dy;
            if (d2 < ri2 || d2 > ro2) continue;
            float a = atan2f((float)dy, (float)dx);
            if (a < a0) a += 2.0f * M_PI;
            if (a >= a0 && a <= a1)
                screenBuf[y * SCREEN_W + x] = c;
        }
    }
}

/* Bresenham line */
static void bufDrawLine(int x0, int y0, int x1, int y1, uint16_t c) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        bufPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ======================================================================
 *  GFXfont TEXT RENDERER (draws into screenBuf)
 * ====================================================================== */
static void bufDrawChar(int cx, int cy, char ch, const GFXfont *font, uint8_t sz, uint16_t col) {
    if (ch < (char)font->first || ch > (char)font->last) return;
    GFXglyph *g = &font->glyph[ch - font->first];
    uint8_t *bmp = font->bitmap;
    uint16_t bo = g->bitmapOffset;
    uint8_t w = g->width, h = g->height;
    int8_t xo = g->xOffset, yo = g->yOffset;
    uint8_t bit = 0, bits = 0;
    for (uint8_t yy = 0; yy < h; yy++) {
        for (uint8_t xx = 0; xx < w; xx++) {
            if (!(bit++ & 7)) bits = pgm_read_byte(&bmp[bo++]);
            if (bits & 0x80) {
                if (sz == 1) {
                    bufPixel(cx + xo + xx, cy + yo + yy, col);
                } else {
                    bufFillRect(cx + (xo + xx) * sz, cy + (yo + yy) * sz, sz, sz, col);
                }
            }
            bits <<= 1;
        }
    }
}

static int bufTextWidth(const char *s, const GFXfont *font, uint8_t sz) {
    int w = 0;
    while (*s) {
        if (*s >= (char)font->first && *s <= (char)font->last) {
            GFXglyph *g = &font->glyph[*s - font->first];
            w += g->xAdvance * sz;
        }
        s++;
    }
    return w;
}

static void bufDrawTextCentered(int cx, int cy, const char *s, const GFXfont *font, uint8_t sz, uint16_t col) {
    int tw = bufTextWidth(s, font, sz);
    int x = cx - tw / 2;
    while (*s) {
        if (*s >= (char)font->first && *s <= (char)font->last) {
            GFXglyph *g = &font->glyph[*s - font->first];
            bufDrawChar(x, cy, *s, font, sz, col);
            x += g->xAdvance * sz;
        }
        s++;
    }
}

static void bufDrawTextRight(int rx, int y, const char *s, const GFXfont *f, uint8_t sz, uint16_t c) {
    int tw = bufTextWidth(s, f, sz);
    int x = rx - tw;
    while (*s) {
        if (*s >= (char)f->first && *s <= (char)f->last) {
            GFXglyph *g = &f->glyph[*s - f->first];
            bufDrawChar(x, y, *s, f, sz, c);
            x += g->xAdvance * sz;
        }
        s++;
    }
}

static void bufDrawText(int lx, int y, const char *s, const GFXfont *f, uint8_t sz, uint16_t c) {
    int x = lx;
    while (*s) {
        if (*s >= (char)f->first && *s <= (char)f->last) {
            GFXglyph *g = &f->glyph[*s - f->first];
            bufDrawChar(x, y, *s, f, sz, c);
            x += g->xAdvance * sz;
        }
        s++;
    }
}

/* ======================================================================
 *  BEZEL RENDERING (static elements, drawn once at startup)
 * ====================================================================== */
/* Speed arc: 135 deg CW to 405 deg (bottom-left to bottom-right, 270 deg sweep) */
#define SPEED_ANG_START  (135.0f * M_PI / 180.0f)
#define SPEED_ANG_END    (405.0f * M_PI / 180.0f)
#define SPEED_ANG_RANGE  (270.0f * M_PI / 180.0f)

/* Battery arc: 148 deg to 392 deg (bottom portion, 244 deg sweep) */
#define BATT_ANG_START   (148.0f * M_PI / 180.0f)
#define BATT_ANG_END     (392.0f * M_PI / 180.0f)
#define BATT_ANG_RANGE   (BATT_ANG_END - BATT_ANG_START)

#define SPEED_R_TICK     168  // Tick outer radius
#define SPEED_R_ARC_OUT  158  // Speed arc outer
#define SPEED_R_ARC_IN   152  // Speed arc inner
#define BATT_R_ARC_OUT   121  // Battery arc outer
#define BATT_R_ARC_IN    117  // Battery arc inner
#define SPEED_R_NUM      128  // Number label radius

static void renderBezel() {
    memset(bezelBuf, 0, SCREEN_W * SCREEN_H * 2);
    uint16_t *savedBuf = screenBuf;
    screenBuf = bezelBuf;

    // Speed tick marks: 37 ticks (0..36), every 5 km/h, major every 20
    int numTicks = 36;
    for (int i = 0; i <= numTicks; i++) {
        float a = SPEED_ANG_START + ((float)i / numTicks) * SPEED_ANG_RANGE;
        bool major = (i % 4 == 0);   // Every 4 ticks = 20 km/h
        int len = major ? 16 : 7;
        int rO = SPEED_R_TICK;
        int x0 = CX + (int)(rO * cosf(a));
        int y0 = CY + (int)(rO * sinf(a));
        int x1 = CX + (int)((rO - len) * cosf(a));
        int y1 = CY + (int)((rO - len) * sinf(a));
        uint16_t col = major ? COL_TEAL_DIM : COL_TEAL_VDIM;
        bufDrawLine(x0, y0, x1, y1, col);

        if (major) {
            int spd = (int)(((float)i / numTicks) * 180 + 0.5f);
            char buf[4];
            snprintf(buf, sizeof(buf), "%d", spd);
            int rL = SPEED_R_NUM;
            int tx = CX + (int)(rL * cosf(a));
            int ty = CY + (int)(rL * sinf(a));
            int tw = bufTextWidth(buf, &FreeSans9pt7b, 1);
            bufDrawText(tx - tw / 2, ty + 5, buf, &FreeSans9pt7b, 1, COL_TEAL_DIM);
        }
    }

    // Speed arc track (very dim ring)
    bufFillArc(CX, CY, SPEED_R_ARC_IN, SPEED_R_ARC_OUT, SPEED_ANG_START, SPEED_ANG_END, COL_TEAL_VDIM);

    // Battery arc track (very dim ring)
    bufFillArc(CX, CY, BATT_R_ARC_IN, BATT_R_ARC_OUT, BATT_ANG_START, BATT_ANG_END, COL_TEAL_VDIM);

    // Static labels
    bufDrawTextCentered(52, CY + 18, "AUTO-", &FreeSans9pt7b, 1, COL_TEXTDIM);
    bufDrawTextCentered(52, CY + 34, "NOMIE", &FreeSans9pt7b, 1, COL_TEXTDIM);
    bufDrawTextCentered(308, CY + 18, "TEMP", &FreeSans9pt7b, 1, COL_TEXTDIM);
    bufDrawTextCentered(308, CY + 34, "BATT.", &FreeSans9pt7b, 1, COL_TEXTDIM);
    bufDrawTextCentered(CX, 316, "BATTERIE", &FreeSans9pt7b, 1, COL_TEXTDIM);

    screenBuf = savedBuf;
    Serial.println("[DASH] Bezel rendered");
}

/* ======================================================================
 *  DASHBOARD RENDERING (dynamic elements on top of bezel)
 * ====================================================================== */
static void renderDashboard() {
    // 1. Copy static bezel as background
    memcpy(screenBuf, bezelBuf, SCREEN_W * SCREEN_H * 2);

    int speed = canData.uiSpeed;
    if (speed > 180) speed = 180;
    uint8_t soc = canData.soc;
    uint8_t gear = canData.gear;

    // 2. Speed arc (colored, over the dim track)
    if (speed > 0) {
        float endA = SPEED_ANG_START + (speed / 180.0f) * SPEED_ANG_RANGE;
        uint16_t sCol = speedColor(speed);
        // Glow (wider, dimmer)
        bufFillArc(CX, CY, SPEED_R_ARC_IN - 3, SPEED_R_ARC_OUT + 3,
                   SPEED_ANG_START, endA, COL_TEAL_VDIM);
        // Main colored arc
        bufFillArc(CX, CY, SPEED_R_ARC_IN, SPEED_R_ARC_OUT,
                   SPEED_ANG_START, endA, sCol);
        // Tip dot
        float midR = (SPEED_R_ARC_IN + SPEED_R_ARC_OUT) / 2.0f;
        int tx = CX + (int)(midR * cosf(endA));
        int ty = CY + (int)(midR * sinf(endA));
        bufFillCircle(tx, ty, 5, sCol);
    }

    // 3. Battery SoC arc (colored)
    if (soc > 0) {
        float endA = BATT_ANG_START + (soc / 100.0f) * BATT_ANG_RANGE;
        uint16_t bCol = socColor(soc);
        bufFillArc(CX, CY, BATT_R_ARC_IN - 2, BATT_R_ARC_OUT + 2,
                   BATT_ANG_START, endA, COL_TEAL_VDIM);
        bufFillArc(CX, CY, BATT_R_ARC_IN, BATT_R_ARC_OUT,
                   BATT_ANG_START, endA, bCol);
        float midR = (BATT_R_ARC_IN + BATT_R_ARC_OUT) / 2.0f;
        int tx = CX + (int)(midR * cosf(endA));
        int ty = CY + (int)(midR * sinf(endA));
        bufFillCircle(tx, ty, 3, bCol);
    }

    // 4. Gear selector (P R N D) at top
    {
        const char gLabels[] = "PRND";
        const uint8_t gVals[] = { GEAR_P, GEAR_R, GEAR_N, GEAR_D };
        const uint16_t gCols[] = { COL_BLUE, COL_RED, COL_AMBER, COL_TEAL };
        int boxW = 28, boxH = 24, gap = 6;
        int totalW = 4 * boxW + 3 * gap;
        int gx0 = CX - totalW / 2;
        int gy = 88;
        for (int i = 0; i < 4; i++) {
            int bx = gx0 + i * (boxW + gap);
            bool active = (gear == gVals[i]);
            uint16_t brdCol = active ? gCols[i] : COL_TEAL_VDIM;
            uint16_t txtCol = active ? gCols[i] : COL_TEAL_VDIM;
            // Border
            bufFillRect(bx, gy, boxW, 1, brdCol);
            bufFillRect(bx, gy + boxH - 1, boxW, 1, brdCol);
            bufFillRect(bx, gy, 1, boxH, brdCol);
            bufFillRect(bx + boxW - 1, gy, 1, boxH, brdCol);
            // Active tint fill
            if (active) {
                for (int jj = gy + 1; jj < gy + boxH - 1; jj++)
                    for (int ii = bx + 1; ii < bx + boxW - 1; ii++)
                        bufPixel(ii, jj, COL_TEAL_VDIM);
            }
            // Letter
            char gs[2] = { gLabels[i], 0 };
            int tw = bufTextWidth(gs, &FreeSans9pt7b, 1);
            bufDrawText(bx + (boxW - tw) / 2, gy + 18, gs, &FreeSans9pt7b, 1, txtCol);
        }
    }

    // 5. Speed number (large, centered)
    {
        char spdStr[4];
        snprintf(spdStr, sizeof(spdStr), "%d", (int)canData.uiSpeed);
        bufDrawTextCentered(CX, CY + 12, spdStr, &FreeSansBold24pt7b, 3, COL_WHITE);
        bufDrawTextCentered(CX, CY + 46, "km/h", &FreeSans9pt7b, 1, COL_TEAL);
    }

    // 6. Range (left widget)
    {
        char rStr[6];
        snprintf(rStr, sizeof(rStr), "%d", (int)canData.rangeKm);
        bufDrawTextCentered(52, CY - 6, rStr, &FreeSansBold24pt7b, 1, COL_BLUE);
        bufDrawTextCentered(52, CY + 8, "km", &FreeSans9pt7b, 1, COL_BLUE);
    }

    // 7. Battery temp (right widget)
    {
        char tStr[6];
        snprintf(tStr, sizeof(tStr), "%d", (int)(canData.battTempMax + 0.5f));
        uint16_t tc = tempColor(canData.battTempMax);
        bufDrawTextCentered(308, CY - 6, tStr, &FreeSansBold24pt7b, 1, tc);
        bufDrawTextCentered(308, CY + 8, "C", &FreeSans9pt7b, 1, tc);
    }

    // 8. SoC percentage (bottom center)
    {
        char socStr[5];
        snprintf(socStr, sizeof(socStr), "%d%%", soc);
        uint16_t sc = socColor(soc);
        bufDrawTextCentered(CX, 298, socStr, &FreeSansBold24pt7b, 1, sc);
    }

    // 9. Regen bar (bottom)
    {
        float totalPower = canData.rearPowerKw + canData.frontPowerKw;
        float regen = (totalPower < 0) ? -totalPower : 0;
        regenKwSmooth = regenKwSmooth * 0.72f + regen * 0.28f;
        if (regenKwSmooth < 0.2f) regenKwSmooth = 0;

        int barY = 338, barW = 100, barH = 4;
        int barX = CX - barW / 2;
        bufFillRect(barX, barY, barW, barH, COL_TEAL_VDIM);
        float pct = regenKwSmooth / MAX_REGEN_KW;
        if (pct > 1.0f) pct = 1.0f;
        int fillW = (int)(barW * pct);
        if (fillW > 0)
            bufFillRect(barX, barY, fillW, barH, COL_TEAL);
        // Labels
        bufDrawText(barX, barY - 8, "REGEN", &FreeSans9pt7b, 1, COL_TEXTDIM);
        char kwStr[10];
        snprintf(kwStr, sizeof(kwStr), "%.0f kW", regenKwSmooth);
        uint16_t kwCol = (regenKwSmooth < 0.2f) ? COL_TEXTDIM : COL_TEAL;
        bufDrawTextRight(barX + barW, barY - 8, kwStr, &FreeSans9pt7b, 1, kwCol);
    }

    // 10. Connection status dots
    {
        unsigned long now = millis();
        bool blinkOn = (now / 500) % 2 == 0;
        // Bridge (left)
        uint16_t bCol;
        if (bridgeEverSeen && (now - lastBridgeMsg < BRIDGE_TIMEOUT_MS))
            bCol = COL_GREEN;
        else if (!bridgeEverSeen)
            bCol = blinkOn ? COL_AMBER : COL_BG;
        else
            bCol = COL_RED;
        bufFillCircle(CX - 25, 20, 6, bCol);
        // Camera (right)
        uint16_t cCol = (WiFi.status() == WL_CONNECTED) ? COL_GREEN : COL_RED;
        bufFillCircle(CX + 25, 20, 6, cCol);
    }

    // 11. Flush to display
    gfx->draw16bitRGBBitmap(0, 0, screenBuf, SCREEN_W, SCREEN_H);
    idleScreenDrawn = true;
    screenBlank = false;
}

/* ======================================================================
 *  ESP-NOW RECEIVE CALLBACK
 * ====================================================================== */
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if ((size_t)len != sizeof(ESP_CAN_Message_t)) return;
    const ESP_CAN_Message_t *m = (const ESP_CAN_Message_t *)data;

    lastBridgeMsg = millis();
    bridgeEverSeen = true;

    switch (m->can_id) {

    case CAN_ID_SPEED: // 0x257 -- DI_uiSpeed (bit 24, 9 bits)
        if (m->dlc >= 4) {
            uint16_t raw = m->data[3] | ((uint16_t)(m->data[4] & 0x01) << 8);
            canData.uiSpeed = raw;
        }
        break;

    case CAN_ID_GEAR: // 0x118 -- DI_gear (bit 21, 3 bits)
        if (m->dlc >= 3) {
            uint8_t raw = (m->data[2] >> 5) & 0x07;
            if (raw != GEAR_SNA) canData.gear = raw;
        }
        break;

    case CAN_ID_RANGE_SOC: // 0x33A -- UI_Range (bit 0, 10 bits), UI_SOC (bit 48, 7 bits)
        if (m->dlc >= 7) {
            uint16_t rangeMi = m->data[0] | ((uint16_t)(m->data[1] & 0x03) << 8);
            canData.rangeKm = (uint16_t)(rangeMi * 1.609f + 0.5f);
            uint8_t socRaw = (m->data[6]) & 0x7F;
            if (socRaw <= 100) canData.soc = socRaw;
        }
        break;

    case CAN_ID_BMS_SOC: // 0x292 -- SOCUI292 (bit 10, 10 bits, x0.1)
        if (m->dlc >= 3) {
            uint16_t raw = ((m->data[1] >> 2) & 0x3F) | ((uint16_t)m->data[2] << 6);
            raw &= 0x3FF;
            float socF = raw * 0.1f;
            if (socF <= 100.0f) canData.soc = (uint8_t)(socF + 0.5f);
        }
        break;

    case CAN_ID_BATT_TEMP: { // 0x312 -- BMSmaxPackTemperature (bit 53, 9 bits, x0.25 -25)
        if (m->dlc >= 8) {
            uint16_t raw = ((m->data[6] >> 5) & 0x07) | ((uint16_t)m->data[7] << 3);
            raw &= 0x1FF;
            canData.battTempMax = raw * 0.25f - 25.0f;
        }
        break;
    }

    case CAN_ID_REAR_PWR: // 0x266 -- RearPower266 (bit 0, 11 bits, signed, x0.5)
        if (m->dlc >= 2) {
            uint16_t raw = m->data[0] | ((uint16_t)(m->data[1] & 0x07) << 8);
            int16_t val = (raw & 0x400) ? (raw | 0xF800) : raw;
            canData.rearPowerKw = val * 0.5f;
        }
        break;

    case CAN_ID_FRONT_PWR: // 0x2E5 -- FrontPower2E5 (bit 0, 11 bits, signed, x0.5)
        if (m->dlc >= 2) {
            uint16_t raw = m->data[0] | ((uint16_t)(m->data[1] & 0x07) << 8);
            int16_t val = (raw & 0x400) ? (raw | 0xF800) : raw;
            canData.frontPowerKw = val * 0.5f;
        }
        break;

    case CAN_ID_HEARTBEAT:
        break;
    }
}

static void initESPNOW() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed");
        return;
    }
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("[ESP-NOW] Initialized");
}

/* ======================================================================
 *  TOUCH / CAMERA CONTROL
 * ====================================================================== */
bool isTouched() {
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)1) != 1) return false;
    uint8_t n = Wire.read();
    return (n > 0 && n <= 5);
}

void sendCtrlCommand(const char *cmd) {
    if (ctrl_sock < 0) return;
    sendto(ctrl_sock, cmd, 4, 0,
           (struct sockaddr *)&ctrl_dest, sizeof(ctrl_dest));
}

/* ======================================================================
 *  CAMERA STREAM (JPEG)
 * ====================================================================== */
void udpRecvTask(void *pvParameters);  // forward declaration

void startWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.config(STATIC_IP, GATEWAY, SUBNET);
    WiFi.begin(AP_SSID);
    WiFi.setSleep(false);
    Serial.printf("[WIFI] Connecting to %s (non-blocking)\n", AP_SSID);
}

void setupNetwork() {
    if (networkReady) return;
    initESPNOW();

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock >= 0) {
        struct sockaddr_in bind_addr = {};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(UDP_PORT);
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind(udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
        int rcvbuf = 65536;
        setsockopt(udp_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }

    ctrl_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&ctrl_dest, 0, sizeof(ctrl_dest));
    ctrl_dest.sin_family = AF_INET;
    ctrl_dest.sin_port = htons(CTRL_PORT);
    ctrl_dest.sin_addr.s_addr = inet_addr("192.168.4.1");

    xTaskCreatePinnedToCore(udpRecvTask, "udpRecv", 4096, NULL, 2, NULL, 0);
    Serial.println("[NET] Network stack ready");
    networkReady = true;
}

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
        tbReady = tbWrite;
        tbReadyLen = recvFrameLen;
        for (int i = 0; i < 3; i++)
            if (i != tbReady && i != tbDisplay) { tbWrite = i; break; }
        xSemaphoreGive(tbMutex);
        lastFrameTime = millis();
        receivedChunks = 0;
        recvFrameLen = 0;
    }
}

void udpRecvTask(void *pvParameters) {
    uint8_t buf[HEADER_SIZE + CHUNK_PAYLOAD];
    while (true) {
        int len = recvfrom(udp_sock, buf, sizeof(buf), 0, NULL, NULL);
        if (len > 0) processPacket(buf, len);
    }
}

int grabReadyFrame(size_t *outLen) {
    xSemaphoreTake(tbMutex, portMAX_DELAY);
    if (tbReady < 0) { xSemaphoreGive(tbMutex); return -1; }
    tbDisplay = tbReady;
    tbReady = -1;
    *outLen = tbReadyLen;
    xSemaphoreGive(tbMutex);
    return tbDisplay;
}

void releaseDisplayBuffer() {
    xSemaphoreTake(tbMutex, portMAX_DELAY);
    tbDisplay = -1;
    xSemaphoreGive(tbMutex);
}

static int jpegDrawCallback(JPEGDRAW *pDraw) {
    int srcX2 = pDraw->x + pDraw->iWidth;
    int srcY2 = pDraw->y + pDraw->iHeight;
    int ix1 = max(pDraw->x, cropX);
    int iy1 = max(pDraw->y, cropY);
    int ix2 = min(srcX2, cropX + drawW);
    int iy2 = min(srcY2, cropY + drawH);
    if (ix1 >= ix2 || iy1 >= iy2) return 1;
    int sx = ix1 - cropX + screenOX, sy = iy1 - cropY + screenOY;
    int w = ix2 - ix1, h = iy2 - iy1;
    int offX = ix1 - pDraw->x, offY = iy1 - pDraw->y;
    for (int row = 0; row < h; row++) {
        uint16_t *src = pDraw->pPixels + (offY + row) * pDraw->iWidth + offX;
        uint16_t *dst = screenBuf + (sy + row) * SCREEN_W + sx;
        memcpy(dst, src, w * 2);
    }
    return 1;
}

void decodeAndDisplay(int bufIdx, size_t len) {
    static JPEGDEC jpeg;
    if (!jpeg.openRAM(jpegBuf[bufIdx], len, jpegDrawCallback)) {
        releaseDisplayBuffer();
        return;
    }
    int imgW = jpeg.getWidth(), imgH = jpeg.getHeight();
    cropX = (imgW >= SCREEN_W) ? (imgW - SCREEN_W) / 2 : 0;
    screenOX = (imgW >= SCREEN_W) ? 0 : (SCREEN_W - imgW) / 2;
    drawW = (imgW >= SCREEN_W) ? SCREEN_W : imgW;
    cropY = (imgH >= SCREEN_H) ? (imgH - SCREEN_H) / 2 : 0;
    screenOY = (imgH >= SCREEN_H) ? 0 : (SCREEN_H - imgH) / 2;
    drawH = (imgH >= SCREEN_H) ? SCREEN_H : imgH;
    jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
    jpeg.decode(0, 0, 0);
    jpeg.close();
    releaseDisplayBuffer();
    gfx->draw16bitRGBBitmap(0, 0, screenBuf, SCREEN_W, SCREEN_H);
    screenBlank = false;
    idleScreenDrawn = false;
    firstFrameReceived = true;
}

/* Draw status dots overlay on camera view */
static void drawStatusDotsOverlay() {
    unsigned long now = millis();
    bool blinkOn = (now / 500) % 2 == 0;
    uint16_t bCol;
    if (bridgeEverSeen && (now - lastBridgeMsg < BRIDGE_TIMEOUT_MS))
        bCol = COL_GREEN;
    else if (!bridgeEverSeen)
        bCol = blinkOn ? COL_AMBER : COL_BG;
    else
        bCol = COL_RED;
    gfx->fillCircle(CX - 25, 20, 6, bCol);
    uint16_t cCol = (WiFi.status() == WL_CONNECTED) ? COL_GREEN : COL_RED;
    gfx->fillCircle(CX + 25, 20, 6, cCol);
}

/* ======================================================================
 *  SETUP
 * ====================================================================== */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TeslaCam - Display (Dashboard + Camera) ===");

    // Allocate PSRAM buffers
    for (int i = 0; i < 3; i++) {
        jpegBuf[i] = (uint8_t *)heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
        if (!jpegBuf[i]) {
            Serial.printf("[FATAL] PSRAM alloc jpegBuf[%d]\n", i);
            while (true) delay(1000);
        }
    }
    screenBuf = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM);
    bezelBuf  = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM);
    if (!screenBuf || !bezelBuf) {
        Serial.println("[FATAL] Screen buffer alloc failed");
        while (true) delay(1000);
    }
    memset(screenBuf, 0, SCREEN_W * SCREEN_H * 2);
    memset(bezelBuf,  0, SCREEN_W * SCREEN_H * 2);
    Serial.println("[MEM] 3x65KB JPEG + 2x259KB screen buffers allocated");
    Serial.printf("[MEM] Free PSRAM: %u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

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

    // Render static bezel
    renderBezel();

    // Touch
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    delay(50);
    Serial.println("[TOUCH] CST816S init");

    // Start WiFi (non-blocking)
    startWiFi();

    lastFrameTime = millis();

    // Initial dashboard render
    renderDashboard();
    Serial.println("[DASH] Dashboard ready - touch to activate camera");
}

/* ======================================================================
 *  MAIN LOOP (core 1)
 * ====================================================================== */
static unsigned long dispFrames = 0;
static unsigned long dispT0 = 0;
static unsigned long lastDashRedraw = 0;
#define DASH_REDRAW_MS 100

void loop() {
    unsigned long now = millis();

    // -- WiFi connection management (non-blocking) --
    if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("[WIFI] Connected - IP: %s\n", WiFi.localIP().toString().c_str());
        setupNetwork();
    } else if (wifiConnected && WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        Serial.println("[WIFI] Disconnected - will auto-reconnect");
    }

    // -- Touch toggle --
    bool touchedNow = isTouched();
    if (touchedPrev && !touchedNow && (now - lastToggleMs > TOUCH_DEBOUNCE_MS)) {
        streamActive = !streamActive;
        lastToggleMs = now;
        if (streamActive) {
            sendCtrlCommand("STRT");
            lastCtrlSendMs = now;
            lastFrameTime = now;
            firstFrameReceived = false;
            idleScreenDrawn = false;
            Serial.println("[TOUCH] Camera ON");
        } else {
            sendCtrlCommand("STOP");
            lastCtrlSendMs = now;
            firstFrameReceived = false;
            idleScreenDrawn = false;
            Serial.println("[TOUCH] Camera OFF");
        }
    }
    touchedPrev = touchedNow;

    // -- Heartbeat STRT --
    if (streamActive && (now - lastCtrlSendMs >= CTRL_RESEND_MS)) {
        sendCtrlCommand("STRT");
        lastCtrlSendMs = now;
    }

    // -- Camera mode --
    if (streamActive) {
        size_t len;
        int idx = grabReadyFrame(&len);
        if (idx >= 0) {
            decodeAndDisplay(idx, len);
            drawStatusDotsOverlay();
            dispFrames++;
            if (dispFrames == 1) dispT0 = now;
            if (dispFrames % 60 == 0) {
                float fps = dispFrames * 1000.0f / (now - dispT0);
                Serial.printf("[CAM] %lu frames, %.1f fps\n", dispFrames, fps);
            }
        } else {
            taskYIELD();
        }
    }
    // -- Dashboard mode --
    else {
        size_t dummyLen;
        int idx = grabReadyFrame(&dummyLen);
        if (idx >= 0) releaseDisplayBuffer();

        if (now - lastDashRedraw >= DASH_REDRAW_MS) {
            renderDashboard();
            lastDashRedraw = now;
        } else {
            delay(5);
        }
    }
}
