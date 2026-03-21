/*
 * TeslaCam -- ESP32-S3 Display (JC3636W518C, ST77916, 360x360 round)
 * LVGL-based dashboard with CAN data from Bridge via ESP-NOW.
 * Camera stream via direct JPEG decode (bypasses LVGL).
 *
 * Architecture:
 *   Core 0: UDP recv task (high priority, blocking recv)
 *   Core 1: Main loop -- LVGL timer handler, touch, dashboard / camera
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <JPEGDEC.h>
#include <lvgl.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <freertos/semphr.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

LV_FONT_DECLARE(font_montserrat_72);

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
#define WIFI_CHANNEL 1  // WiFi channel (must match Camera + Bridge)

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
static unsigned long frameStartMs = 0;
#define FRAME_ASSEMBLY_TIMEOUT_MS 200

/* ======================================================================
 *  SCREEN BUFFERS (for camera JPEG mode)
 * ====================================================================== */
static uint16_t *screenBuf;   // 360x360 for JPEG decode
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

#define CAN_ID_SPEED     0x257
#define CAN_ID_GEAR      0x118
#define CAN_ID_RANGE_SOC 0x33A
#define CAN_ID_BATT_TEMP   0x312
#define CAN_ID_COOLANT_TEMP 0x321
#define CAN_ID_REAR_PWR    0x266
#define CAN_ID_FRONT_PWR 0x2E5
#define CAN_ID_BMS_SOC   0x292
#define CAN_ID_UTC_TIME  0x318
#define CAN_ID_HVAC_STATUS 0x243
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
    float    battTempMin;
    float    coolantTemp;
    bool     battTempReceived;
    float    rearPowerKw;
    float    frontPowerKw;
    uint8_t  utcHour;
    uint8_t  utcMinute;
    bool     timeReceived;
    float    outdoorTemp;
    bool     outdoorTempReceived;
    float    cabinTemp;
    bool     cabinTempReceived;
} canData = { 0, GEAR_P, 0, 0, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f, 0, 0, false, 0.0f, false, 0.0f, false };

#define BRIDGE_TIMEOUT_MS 5000
static volatile unsigned long lastBridgeMsg = 0;
static volatile bool bridgeEverSeen = false;
static volatile bool espNowPaused = false;

static float regenKwSmooth = 0;
#define MAX_REGEN_KW 50.0f

/* ======================================================================
 *  LVGL DISPLAY DRIVER
 * ====================================================================== */
static lv_disp_draw_buf_t lvDrawBuf;
static lv_disp_drv_t      lvDispDrv;
static lv_indev_drv_t     lvTouchDrv;
#define LV_BUF_LINES 40
static lv_color_t *lvBuf1;
static lv_color_t *lvBuf2;

static void lvFlushCb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *buf) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)buf, w, h);
    lv_disp_flush_ready(drv);
}

/* ======================================================================
 *  LVGL COLORS (lv_color_t palette)
 * ====================================================================== */
#define MKCOL(r,g,b) lv_color_make(r,g,b)
static const lv_color_t COL_BG       = MKCOL(5, 10, 15);
static const lv_color_t COL_TEAL     = MKCOL(0, 229, 200);
static const lv_color_t COL_TEAL_DIM = MKCOL(0, 90, 80);
static const lv_color_t COL_TEAL_VDIM= MKCOL(0, 42, 38);
static const lv_color_t COL_AMBER    = MKCOL(255, 170, 0);
static const lv_color_t COL_RED      = MKCOL(255, 59, 59);
static const lv_color_t COL_BLUE     = MKCOL(26, 143, 255);
static const lv_color_t COL_WHITE    = MKCOL(255, 255, 255);
static const lv_color_t COL_TEXTDIM  = MKCOL(58, 90, 112);
static const lv_color_t COL_GREEN    = MKCOL(0, 255, 0);

static const lv_color_t COL_GREY     = MKCOL(80, 80, 80);

static lv_color_t lvSpeedColor(int speed) {
    (void)speed;
    return COL_GREEN;
}

static lv_color_t lvSocColor(uint8_t soc) {
    if (soc > 20) return COL_GREEN;
    if (soc > 10) return COL_AMBER;
    return COL_RED;
}

static lv_color_t lvTempColor(float t) {
    if (t < 20.0f) return COL_BLUE;
    if (t < 42.0f) return COL_AMBER;
    return COL_RED;
}

/* ======================================================================
 *  LVGL DASHBOARD WIDGETS
 * ====================================================================== */
/* Speed arc (outer, 270° sweep) */
static lv_obj_t *arcSpeed;
static lv_obj_t *arcSpeedTrack; // background ring (using a second arc for the dim track)

/* Battery arc (inner, 244° sweep) */
static lv_obj_t *arcBatt;

/* Labels */
static lv_obj_t *lblSpeed;
static lv_obj_t *lblSpeedUnit;
static lv_obj_t *lblTime;
static lv_obj_t *lblRange;
static lv_obj_t *lblRangeUnit;
static lv_obj_t *lblRangeLbl;
static lv_obj_t *lblTemp;
static lv_obj_t *lblTempUnit;
static lv_obj_t *lblTempLbl;
static lv_obj_t *lblSoc;
static lv_obj_t *lblSocLbl;
static lv_obj_t *lblRegenLbl;
static lv_obj_t *lblRegenKw;

/* Temperature labels (near time) */
static lv_obj_t *lblCabinTemp;
static lv_obj_t *lblOutdoorTemp;

/* Regen bar */
static lv_obj_t *barRegen;

/* Gear indicator (single) */
static lv_obj_t *gearBox;
static lv_obj_t *gearLabel;

/* Connection dots */
static lv_obj_t *dotBridge;
static lv_obj_t *dotCamera;

/* Speed tick lines drawn on a dedicated object */
static lv_obj_t *tickCanvas;

/* ── Speed arc angles ── */
/* LVGL arcs: 0° = right (3 o'clock), CW.
 * Mockup speed: starts at 135° (bottom-left) → 405° (bottom-right) = 270° sweep.
 * LVGL arc convention: bg_start_angle / bg_end_angle in degrees CW from right.
 * For 270° sweep from 135° to 405°: start=135, end=405 → but LVGL wraps at 360.
 * LVGL handles angles > 360 correctly for arcs. */
#define SPEED_ARC_START 135
#define SPEED_ARC_END   45     // 405 - 360 = 45 (LVGL wraps)
#define SPEED_ARC_RANGE 270
#define SPEED_R_OUTER   158    // outer radius of speed arc
#define SPEED_ARC_WIDTH 6

/* Battery: 148° → 392° = 244° sweep */
#define BATT_ARC_START  148
#define BATT_ARC_END    32     // 392 - 360 = 32
#define BATT_ARC_RANGE  244
#define BATT_R_OUTER    121
#define BATT_ARC_WIDTH  4

/* ======================================================================
 *  TICK MARK DRAWING (event callback on a transparent obj)
 * ====================================================================== */
static void drawTicksCb(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);

    int cx = (obj_coords.x1 + obj_coords.x2) / 2;
    int cy = (obj_coords.y1 + obj_coords.y2) / 2;

    int numTicks = 36;
    float startA = 135.0f * M_PI / 180.0f;
    float rangeA = 270.0f * M_PI / 180.0f;
    int rO = 172;

    for (int i = 0; i <= numTicks; i++) {
        float a = startA + ((float)i / numTicks) * rangeA;
        bool major = (i % 4 == 0);
        int len = major ? 18 : 8;

        lv_point_t pts[2];
        pts[0].x = cx + (int)(rO * cosf(a));
        pts[0].y = cy + (int)(rO * sinf(a));
        pts[1].x = cx + (int)((rO - len) * cosf(a));
        pts[1].y = cy + (int)((rO - len) * sinf(a));

        lv_draw_line_dsc_t ldsc;
        lv_draw_line_dsc_init(&ldsc);
        ldsc.color = major ? COL_TEAL_DIM : COL_TEAL_VDIM;
        ldsc.width = major ? 2 : 1;
        ldsc.opa = LV_OPA_COVER;
        lv_draw_line(draw_ctx, &ldsc, &pts[0], &pts[1]);

        if (major) {
            int spd = (int)(((float)i / numTicks) * 180 + 0.5f);
            char buf[4];
            snprintf(buf, sizeof(buf), "%d", spd);
            int rL = rO - 34;
            int tx = cx + (int)(rL * cosf(a));
            int ty = cy + (int)(rL * sinf(a));

            lv_draw_label_dsc_t lbdsc;
            lv_draw_label_dsc_init(&lbdsc);
            lbdsc.color = COL_TEAL_DIM;
            lbdsc.font = &lv_font_montserrat_12;
            lbdsc.opa = LV_OPA_COVER;
            lbdsc.align = LV_TEXT_ALIGN_CENTER;

            lv_area_t txt_area;
            lv_point_t txt_size;
            lv_txt_get_size(&txt_size, buf, lbdsc.font, 0, 0, 200, LV_TEXT_FLAG_NONE);
            txt_area.x1 = tx - txt_size.x / 2;
            txt_area.y1 = ty - txt_size.y / 2;
            txt_area.x2 = txt_area.x1 + txt_size.x - 1;
            txt_area.y2 = txt_area.y1 + txt_size.y - 1;
            lv_draw_label(draw_ctx, &lbdsc, &txt_area, buf, NULL);
        }
    }
}

/* ======================================================================
 *  CREATE DASHBOARD UI
 * ====================================================================== */
static void createDashboard(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ── Tick marks (custom draw on transparent overlay) ── */
    tickCanvas = lv_obj_create(scr);
    lv_obj_remove_style_all(tickCanvas);
    lv_obj_set_size(tickCanvas, SCREEN_W, SCREEN_H);
    lv_obj_center(tickCanvas);
    lv_obj_add_event_cb(tickCanvas, drawTicksCb, LV_EVENT_DRAW_MAIN_END, NULL);
    lv_obj_clear_flag(tickCanvas, LV_OBJ_FLAG_CLICKABLE);

    /* ── Speed arc ── */
    arcSpeed = lv_arc_create(scr);
    lv_obj_set_size(arcSpeed, SPEED_R_OUTER * 2, SPEED_R_OUTER * 2);
    lv_obj_center(arcSpeed);
    lv_arc_set_rotation(arcSpeed, 0);
    lv_arc_set_range(arcSpeed, 0, 180);
    lv_arc_set_value(arcSpeed, 0);
    lv_arc_set_bg_angles(arcSpeed, SPEED_ARC_START, SPEED_ARC_START + SPEED_ARC_RANGE);
    lv_arc_set_mode(arcSpeed, LV_ARC_MODE_NORMAL);
    lv_obj_clear_flag(arcSpeed, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_style(arcSpeed, NULL, LV_PART_KNOB);

    /* Background (dim track) */
    lv_obj_set_style_arc_color(arcSpeed, COL_TEAL_VDIM, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arcSpeed, SPEED_ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arcSpeed, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arcSpeed, true, LV_PART_MAIN);

    /* Indicator (colored active part) */
    lv_obj_set_style_arc_color(arcSpeed, COL_TEAL, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arcSpeed, SPEED_ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arcSpeed, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arcSpeed, true, LV_PART_INDICATOR);

    /* Remove background of the arc object itself */
    lv_obj_set_style_bg_opa(arcSpeed, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arcSpeed, 0, 0);
    lv_obj_set_style_pad_all(arcSpeed, 0, 0);

    /* ── Battery arc ── */
    arcBatt = lv_arc_create(scr);
    lv_obj_set_size(arcBatt, BATT_R_OUTER * 2, BATT_R_OUTER * 2);
    lv_obj_center(arcBatt);
    lv_arc_set_rotation(arcBatt, 0);
    lv_arc_set_range(arcBatt, 0, 100);
    lv_arc_set_value(arcBatt, 0);
    lv_arc_set_bg_angles(arcBatt, BATT_ARC_START, BATT_ARC_START + BATT_ARC_RANGE);
    lv_arc_set_mode(arcBatt, LV_ARC_MODE_NORMAL);
    lv_obj_clear_flag(arcBatt, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_style(arcBatt, NULL, LV_PART_KNOB);

    lv_obj_set_style_arc_color(arcBatt, COL_TEAL_VDIM, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arcBatt, BATT_ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arcBatt, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arcBatt, true, LV_PART_MAIN);

    lv_obj_set_style_arc_color(arcBatt, COL_TEAL, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arcBatt, BATT_ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arcBatt, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arcBatt, true, LV_PART_INDICATOR);

    lv_obj_set_style_bg_opa(arcBatt, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(arcBatt, 0, 0);
    lv_obj_set_style_pad_all(arcBatt, 0, 0);

    /* ── Gear indicator (single letter, right of SoC) ── */
    {
        gearBox = lv_obj_create(scr);
        lv_obj_remove_style_all(gearBox);
        lv_obj_set_size(gearBox, 28, 26);
        lv_obj_align(gearBox, LV_ALIGN_BOTTOM_MID, 80, -42);
        lv_obj_set_style_bg_opa(gearBox, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(gearBox, 1, 0);
        lv_obj_set_style_border_color(gearBox, COL_TEAL, 0);
        lv_obj_set_style_border_opa(gearBox, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(gearBox, 4, 0);
        lv_obj_clear_flag(gearBox, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        gearLabel = lv_label_create(gearBox);
        lv_label_set_text(gearLabel, "P");
        lv_obj_set_style_text_font(gearLabel, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(gearLabel, COL_BLUE, 0);
        lv_obj_center(gearLabel);
    }

    /* ── Time (above speed) ── */
    lblTime = lv_label_create(scr);
    lv_label_set_text(lblTime, "--:--");
    lv_obj_set_style_text_font(lblTime, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblTime, COL_WHITE, 0);
    lv_obj_set_style_text_align(lblTime, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblTime, LV_ALIGN_CENTER, 0, -80);

    /* ── Cabin temp (right of time) ── */
    lblCabinTemp = lv_label_create(scr);
    lv_label_set_text(lblCabinTemp, "--\xC2\xB0");
    lv_obj_set_style_text_font(lblCabinTemp, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lblCabinTemp, COL_GREY, 0);
    lv_obj_align(lblCabinTemp, LV_ALIGN_CENTER, 55, -80);

    /* ── Outdoor temp (left of time) ── */
    lblOutdoorTemp = lv_label_create(scr);
    lv_label_set_text(lblOutdoorTemp, "--\xC2\xB0");
    lv_obj_set_style_text_font(lblOutdoorTemp, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lblOutdoorTemp, COL_GREY, 0);
    lv_obj_set_style_text_align(lblOutdoorTemp, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lblOutdoorTemp, LV_ALIGN_CENTER, -55, -80);

    /* ── Speed value (large, centered) ── */
    lblSpeed = lv_label_create(scr);
    lv_label_set_text(lblSpeed, "0");
    lv_obj_set_style_text_font(lblSpeed, &font_montserrat_72, 0);
    lv_obj_set_style_text_color(lblSpeed, COL_WHITE, 0);
    lv_obj_set_style_text_align(lblSpeed, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblSpeed, LV_ALIGN_CENTER, 0, -26);

    lblSpeedUnit = lv_label_create(scr);
    lv_label_set_text(lblSpeedUnit, "km/h");
    lv_obj_set_style_text_font(lblSpeedUnit, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lblSpeedUnit, COL_TEAL, 0);
    lv_obj_align(lblSpeedUnit, LV_ALIGN_CENTER, 0, 14);

    /* ── Range (left widget, inside battery arc) ── */
    lblRange = lv_label_create(scr);
    lv_label_set_text(lblRange, "0");
    lv_obj_set_style_text_font(lblRange, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblRange, COL_BLUE, 0);
    lv_obj_set_style_text_align(lblRange, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblRange, LV_ALIGN_CENTER, -72, -8);

    lblRangeUnit = lv_label_create(scr);
    lv_label_set_text(lblRangeUnit, "km");
    lv_obj_set_style_text_font(lblRangeUnit, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblRangeUnit, COL_BLUE, 0);
    lv_obj_align(lblRangeUnit, LV_ALIGN_CENTER, -72, 10);

    lblRangeLbl = lv_label_create(scr);
    lv_label_set_text(lblRangeLbl, "AUTO-\nNOMIE");
    lv_obj_set_style_text_font(lblRangeLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblRangeLbl, COL_TEXTDIM, 0);
    lv_obj_set_style_text_align(lblRangeLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblRangeLbl, LV_ALIGN_CENTER, -72, 30);

    /* ── Temp (right widget, inside battery arc) ── */
    lblTemp = lv_label_create(scr);
    lv_label_set_text(lblTemp, "25");
    lv_obj_set_style_text_font(lblTemp, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblTemp, COL_AMBER, 0);
    lv_obj_set_style_text_align(lblTemp, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblTemp, LV_ALIGN_CENTER, 72, -8);

    lblTempUnit = lv_label_create(scr);
    lv_label_set_text(lblTempUnit, "°C");
    lv_obj_set_style_text_font(lblTempUnit, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblTempUnit, COL_AMBER, 0);
    lv_obj_align(lblTempUnit, LV_ALIGN_CENTER, 72, 10);

    lblTempLbl = lv_label_create(scr);
    lv_label_set_text(lblTempLbl, "TEMP\nBATT.");
    lv_obj_set_style_text_font(lblTempLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblTempLbl, COL_TEXTDIM, 0);
    lv_obj_set_style_text_align(lblTempLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblTempLbl, LV_ALIGN_CENTER, 72, 30);

    /* ── SoC (bottom center) ── */
    lblSoc = lv_label_create(scr);
    lv_label_set_text(lblSoc, "0%");
    lv_obj_set_style_text_font(lblSoc, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblSoc, COL_TEAL, 0);
    lv_obj_set_style_text_align(lblSoc, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblSoc, LV_ALIGN_BOTTOM_MID, 0, -74);

    lblSocLbl = lv_label_create(scr);
    lv_label_set_text(lblSocLbl, "BATTERIE");
    lv_obj_set_style_text_font(lblSocLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblSocLbl, COL_TEXTDIM, 0);
    lv_obj_align(lblSocLbl, LV_ALIGN_BOTTOM_MID, 0, -52);

    /* ── Regen bar ── */
    int barW = 100, barH = 4;
    barRegen = lv_bar_create(scr);
    lv_obj_set_size(barRegen, barW, barH);
    lv_obj_align(barRegen, LV_ALIGN_BOTTOM_MID, 0, -24);
    lv_bar_set_range(barRegen, 0, 100);
    lv_bar_set_value(barRegen, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(barRegen, COL_TEAL_VDIM, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(barRegen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(barRegen, COL_TEAL, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(barRegen, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(barRegen, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(barRegen, 2, LV_PART_INDICATOR);

    lblRegenLbl = lv_label_create(scr);
    lv_label_set_text(lblRegenLbl, "REGEN");
    lv_obj_set_style_text_font(lblRegenLbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblRegenLbl, COL_TEXTDIM, 0);
    lv_obj_align(lblRegenLbl, LV_ALIGN_BOTTOM_MID, -30, -30);

    lblRegenKw = lv_label_create(scr);
    lv_label_set_text(lblRegenKw, "0 kW");
    lv_obj_set_style_text_font(lblRegenKw, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lblRegenKw, COL_TEXTDIM, 0);
    lv_obj_align(lblRegenKw, LV_ALIGN_BOTTOM_MID, 34, -30);

    /* ── Connection status dots (bottom, small, centered) ── */
    dotBridge = lv_obj_create(scr);
    lv_obj_remove_style_all(dotBridge);
    lv_obj_set_size(dotBridge, 5, 5);
    lv_obj_align(dotBridge, LV_ALIGN_BOTTOM_MID, -6, -6);
    lv_obj_set_style_radius(dotBridge, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dotBridge, COL_AMBER, 0);
    lv_obj_set_style_bg_opa(dotBridge, LV_OPA_COVER, 0);
    lv_obj_clear_flag(dotBridge, LV_OBJ_FLAG_CLICKABLE);

    dotCamera = lv_obj_create(scr);
    lv_obj_remove_style_all(dotCamera);
    lv_obj_set_size(dotCamera, 5, 5);
    lv_obj_align(dotCamera, LV_ALIGN_BOTTOM_MID, 6, -6);
    lv_obj_set_style_radius(dotCamera, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dotCamera, COL_RED, 0);
    lv_obj_set_style_bg_opa(dotCamera, LV_OPA_COVER, 0);
    lv_obj_clear_flag(dotCamera, LV_OBJ_FLAG_CLICKABLE);

    Serial.println("[LVGL] Dashboard UI created");
}

/* ======================================================================
 *  UPDATE DASHBOARD FROM CAN DATA
 * ====================================================================== */
static void updateDashboard(void) {
    unsigned long now = millis();
    bool bridgeConnected = bridgeEverSeen && (now - lastBridgeMsg < BRIDGE_TIMEOUT_MS);
    bool noConnection = !bridgeEverSeen || !bridgeConnected;

    int speed = canData.uiSpeed;
    if (speed > 180) speed = 180;
    uint8_t soc = canData.soc;
    uint8_t gear = canData.gear;

    /* Speed arc */
    lv_arc_set_value(arcSpeed, noConnection ? 0 : speed);
    lv_color_t sCol = lvSpeedColor(speed);
    lv_obj_set_style_arc_color(arcSpeed, sCol, LV_PART_INDICATOR);

    /* Battery arc */
    lv_arc_set_value(arcBatt, noConnection ? 0 : soc);
    lv_color_t bCol = lvSocColor(soc);
    lv_obj_set_style_arc_color(arcBatt, bCol, LV_PART_INDICATOR);

    /* Gear indicator */
    if (noConnection) {
        lv_label_set_text(gearLabel, "");
        lv_obj_set_style_text_color(gearLabel, COL_GREY, 0);
        lv_obj_set_style_border_color(gearBox, COL_GREY, 0);
    } else {
        const char *gLetters[] = { "?", "P", "R", "N", "D" };
        const lv_color_t gColors[] = { COL_TEAL_VDIM, COL_BLUE, COL_RED, COL_AMBER, COL_TEAL };
        int idx = (gear >= 1 && gear <= 4) ? gear : 0;
        lv_label_set_text(gearLabel, gLetters[idx]);
        lv_obj_set_style_text_color(gearLabel, gColors[idx], 0);
        lv_obj_set_style_border_color(gearBox, gColors[idx], 0);
    }

    /* Speed number */
    if (noConnection) {
        lv_label_set_text(lblSpeed, "-");
    } else {
        char buf[4];
        snprintf(buf, sizeof(buf), "%d", (int)canData.uiSpeed);
        lv_label_set_text(lblSpeed, buf);
    }

    /* Range */
    if (noConnection) {
        lv_label_set_text(lblRange, "-");
    } else {
        char buf[6];
        snprintf(buf, sizeof(buf), "%d", (int)canData.rangeKm);
        lv_label_set_text(lblRange, buf);
    }
    lv_obj_align(lblRange, LV_ALIGN_CENTER, -72, -8);

    /* Battery temp (coolant inlet temperature — closest to pack average) */
    if (noConnection || !canData.battTempReceived) {
        lv_label_set_text(lblTemp, "-");
        lv_obj_set_style_text_color(lblTemp, COL_GREY, 0);
        lv_obj_set_style_text_color(lblTempUnit, COL_GREY, 0);
    } else {
        float battTemp = canData.coolantTemp;
        char buf[6];
        snprintf(buf, sizeof(buf), "%d", (int)(battTemp + 0.5f));
        lv_label_set_text(lblTemp, buf);
        lv_color_t tc = lvTempColor(battTemp);
        lv_obj_set_style_text_color(lblTemp, tc, 0);
        lv_obj_set_style_text_color(lblTempUnit, tc, 0);
    }
    lv_obj_align(lblTemp, LV_ALIGN_CENTER, 72, -8);

    /* SoC */
    if (noConnection) {
        lv_label_set_text(lblSoc, "-%");
        lv_obj_set_style_text_color(lblSoc, COL_GREY, 0);
    } else {
        char buf[5];
        snprintf(buf, sizeof(buf), "%d%%", soc);
        lv_label_set_text(lblSoc, buf);
        lv_color_t sc = lvSocColor(soc);
        lv_obj_set_style_text_color(lblSoc, sc, 0);
    }
    lv_obj_align(lblSoc, LV_ALIGN_BOTTOM_MID, 0, -74);

    /* Regen */
    {
        float totalPower = canData.rearPowerKw + canData.frontPowerKw;
        float regen = (totalPower < 0) ? -totalPower : 0;
        regenKwSmooth = regenKwSmooth * 0.72f + regen * 0.28f;
        if (regenKwSmooth < 0.2f) regenKwSmooth = 0;

        int pct = (int)(regenKwSmooth / MAX_REGEN_KW * 100);
        if (pct > 100) pct = 100;
        lv_bar_set_value(barRegen, pct, LV_ANIM_OFF);

        char buf[10];
        snprintf(buf, sizeof(buf), "%.0f kW", regenKwSmooth);
        lv_label_set_text(lblRegenKw, buf);
        lv_color_t kwCol = (regenKwSmooth < 0.2f) ? COL_TEXTDIM : COL_TEAL;
        lv_obj_set_style_text_color(lblRegenKw, kwCol, 0);
        lv_obj_align(lblRegenKw, LV_ALIGN_BOTTOM_MID, 34, -30);
    }

    /* Time (UTC+1 for France CET) */
    if (canData.timeReceived && !noConnection) {
        int localHour = (canData.utcHour + 1) % 24;
        char buf[6];
        snprintf(buf, sizeof(buf), "%02d:%02d", localHour, canData.utcMinute);
        lv_label_set_text(lblTime, buf);
        lv_obj_set_style_text_color(lblTime, COL_WHITE, 0);
    } else {
        lv_label_set_text(lblTime, "--:--");
        lv_obj_set_style_text_color(lblTime, COL_GREY, 0);
    }

    /* Cabin temp (right of time) */
    if (canData.cabinTempReceived && !noConnection) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", canData.cabinTemp);
        lv_label_set_text(lblCabinTemp, buf);
        lv_obj_set_style_text_color(lblCabinTemp, COL_AMBER, 0);
    } else {
        lv_label_set_text(lblCabinTemp, "--\xC2\xB0");
        lv_obj_set_style_text_color(lblCabinTemp, COL_GREY, 0);
    }

    /* Outdoor temp (left of time) */
    if (canData.outdoorTempReceived && !noConnection) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", canData.outdoorTemp);
        lv_label_set_text(lblOutdoorTemp, buf);
        lv_obj_set_style_text_color(lblOutdoorTemp, COL_BLUE, 0);
    } else {
        lv_label_set_text(lblOutdoorTemp, "--\xC2\xB0");
        lv_obj_set_style_text_color(lblOutdoorTemp, COL_GREY, 0);
    }

    /* Connection dots */
    {
        bool blinkOn = (now / 500) % 2 == 0;
        lv_color_t bDotCol;
        if (bridgeConnected)
            bDotCol = COL_GREEN;
        else if (!bridgeEverSeen)
            bDotCol = blinkOn ? COL_AMBER : COL_BG;
        else
            bDotCol = COL_RED;
        lv_obj_set_style_bg_color(dotBridge, bDotCol, 0);

        /* Camera dot: green when WiFi connected, red otherwise */
        lv_color_t cDotCol = wifiConnected ? COL_GREEN : COL_RED;
        lv_obj_set_style_bg_color(dotCamera, cDotCol, 0);
    }
}

/* ======================================================================
 *  ESP-NOW RECEIVE CALLBACK
 * ====================================================================== */
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (espNowPaused) return;
    if ((size_t)len != sizeof(ESP_CAN_Message_t)) return;
    const ESP_CAN_Message_t *m = (const ESP_CAN_Message_t *)data;

    lastBridgeMsg = millis();
    bridgeEverSeen = true;

    switch (m->can_id) {

    case CAN_ID_SPEED:
        if (m->dlc >= 4) {
            uint16_t raw = m->data[3] | ((uint16_t)(m->data[4] & 0x01) << 8);
            canData.uiSpeed = raw;
        }
        break;

    case CAN_ID_GEAR:
        if (m->dlc >= 3) {
            uint8_t raw = (m->data[2] >> 5) & 0x07;
            if (raw != GEAR_SNA) canData.gear = raw;
        }
        break;

    case CAN_ID_RANGE_SOC:
        if (m->dlc >= 8) {
            uint16_t rangeMi = m->data[0] | ((uint16_t)(m->data[1] & 0x03) << 8);
            canData.rangeKm = (uint16_t)(rangeMi * 1.609f + 0.5f);
            uint8_t uiSoc = (m->data[6]) & 0x7F;
            uint8_t uiSoe = (m->data[7]) & 0x7F;
            Serial.printf("[CAN] 0x33A bytes: %02X %02X %02X %02X %02X %02X %02X %02X  UI_SOC=%u UI_uSOE=%u\n",
                m->data[0], m->data[1], m->data[2], m->data[3],
                m->data[4], m->data[5], m->data[6], m->data[7], uiSoc, uiSoe);
            /* Use UI_SOC if valid (1-100), otherwise fall through to SOCUI292 */
            if (uiSoc >= 1 && uiSoc <= 100) canData.soc = uiSoc;
        }
        break;

    case CAN_ID_BMS_SOC:
        if (m->dlc >= 4) {
            /* SOCUI292: bit 10, 10 bits, factor 0.1 — raw BMS SoC includes ~5% bottom buffer */
            uint16_t rawSoc = ((m->data[1] >> 2) & 0x3F) | ((uint16_t)(m->data[2] & 0x0F) << 6);
            float socFloat = rawSoc * 0.1f;
            /* Apply buffer correction: bottom ~5%, no degradation (health 100%) */
            float displayedSoc = (socFloat - 5.0f) / 95.0f * 100.0f;
            if (displayedSoc < 0.0f) displayedSoc = 0.0f;
            if (displayedSoc > 100.0f) displayedSoc = 100.0f;
            /* Only use SOCUI292 as fallback if UI_SOC hasn't been set */
            if (canData.soc == 0) canData.soc = (uint8_t)(displayedSoc + 0.5f);
        }
        break;

    case CAN_ID_BATT_TEMP: {
        if (m->dlc >= 8) {
            /* BMSminPackTemperature: bit 44, 9 bits, ×0.25 −25°C */
            uint16_t rawMin = ((m->data[5] >> 4) & 0x0F) | ((uint16_t)(m->data[6] & 0x1F) << 4);
            rawMin &= 0x1FF;
            canData.battTempMin = rawMin * 0.25f - 25.0f;

            /* BMSmaxPackTemperature: bit 53, 9 bits, ×0.25 −25°C */
            uint16_t rawMax = ((m->data[6] >> 5) & 0x07) | ((uint16_t)m->data[7] << 3);
            rawMax &= 0x1FF;
            canData.battTempMax = rawMax * 0.25f - 25.0f;

            Serial.printf("[CAN] 0x312 raw: %02X %02X %02X %02X %02X %02X %02X %02X → min=%.1f max=%.1f\n",
                          m->data[0], m->data[1], m->data[2], m->data[3],
                          m->data[4], m->data[5], m->data[6], m->data[7],
                          canData.battTempMin, canData.battTempMax);
        }
        break;
    }

    case CAN_ID_COOLANT_TEMP:
        if (m->dlc >= 2) {
            /* VCFRONT_tempCoolantBatInlet: bit 0, 10 bits, ×0.125 −40°C */
            uint16_t raw = m->data[0] | ((uint16_t)(m->data[1] & 0x03) << 8);
            float temp = raw * 0.125f - 40.0f;
            if (raw != 0x3FF) {  /* 1023 = SNA */
                canData.coolantTemp = temp;
                canData.battTempReceived = true;
            }
        }
        if (m->dlc >= 6) {
            /* VCFRONT_tempAmbientFiltered: bit 40, 8 bits, ×0.5 −40°C */
            uint8_t raw = m->data[5];
            if (raw != 0) {  /* 0 = SNA */
                canData.outdoorTemp = raw * 0.5f - 40.0f;
                canData.outdoorTempReceived = true;
            }
        }
        break;

    case CAN_ID_HVAC_STATUS:
        if (m->dlc >= 6) {
            /* Multiplexed: index at bits 0-1 */
            uint8_t muxIdx = m->data[0] & 0x03;
            if (muxIdx == 0) {
                /* VCRIGHT_hvacCabinTempEst: bit 30, 11 bits, ×0.1 −40°C */
                uint16_t raw = ((m->data[3] >> 6) & 0x03) |
                               ((uint16_t)m->data[4] << 2) |
                               ((uint16_t)(m->data[5] & 0x01) << 10);
                if (raw != 0x7FF) {  /* 2047 = likely SNA */
                    float temp = raw * 0.1f - 40.0f;
                    if (temp > -40.0f && temp < 60.0f) {
                        canData.cabinTemp = temp;
                        canData.cabinTempReceived = true;
                    }
                }
            }
        }
        break;

    case CAN_ID_REAR_PWR:
        if (m->dlc >= 2) {
            uint16_t raw = m->data[0] | ((uint16_t)(m->data[1] & 0x07) << 8);
            int16_t val = (raw & 0x400) ? (raw | 0xF800) : raw;
            canData.rearPowerKw = val * 0.5f;
        }
        break;

    case CAN_ID_FRONT_PWR:
        if (m->dlc >= 2) {
            uint16_t raw = m->data[0] | ((uint16_t)(m->data[1] & 0x07) << 8);
            int16_t val = (raw & 0x400) ? (raw | 0xF800) : raw;
            canData.frontPowerKw = val * 0.5f;
        }
        break;

    case CAN_ID_UTC_TIME:
        if (m->dlc >= 6) {
            canData.utcHour   = m->data[3];  // UTChour318: byte 3
            canData.utcMinute = m->data[5];  // UTCminutes318: byte 5
            canData.timeReceived = true;
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
void udpRecvTask(void *pvParameters);

void startWiFi() {
    WiFi.mode(WIFI_STA);

    /* Initialize ESP-NOW early on configured channel */
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    initESPNOW();

    WiFi.config(STATIC_IP, GATEWAY, SUBNET);
    WiFi.begin(AP_SSID);
    WiFi.setSleep(false);
    esp_wifi_set_max_tx_power(78);  // Max TX power (19.5 dBm) for EMI resilience
    Serial.printf("[WIFI] Connecting to %s (non-blocking)\n", AP_SSID);
}

void setupNetwork() {
    if (networkReady) return;

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

/* Promote current write buffer as a ready frame (complete or partial) */
static void promoteCurrentFrame() {
    xSemaphoreTake(tbMutex, portMAX_DELAY);
    tbReady = tbWrite;
    tbReadyLen = recvFrameLen;
    for (int i = 0; i < 3; i++)
        if (i != tbReady && i != tbDisplay) { tbWrite = i; break; }
    xSemaphoreGive(tbMutex);
    lastFrameTime = millis();
    receivedChunks = 0;
    recvFrameLen   = 0;
    expectedChunks = 0;
}

void processPacket(uint8_t *data, size_t len) {
    if (len < HEADER_SIZE) return;
    uint16_t frameId, chunkId, totalChunks, chunkSize;
    memcpy(&frameId,     data + 0, 2);
    memcpy(&chunkId,     data + 2, 2);
    memcpy(&totalChunks, data + 4, 2);
    memcpy(&chunkSize,   data + 6, 2);

    /* END-of-frame marker: chunkId == 0xFFFF */
    if (chunkId == 0xFFFF) {
        if (frameId == currentFrameId && expectedChunks > 0 && receivedChunks > 0) {
            promoteCurrentFrame();
        }
        return;
    }

    if (chunkSize > CHUNK_PAYLOAD || chunkSize + HEADER_SIZE > len) return;
    if (frameId != currentFrameId) {
        /* New frame arrived — promote previous partial frame if >50% received */
        if (expectedChunks > 0 && receivedChunks > expectedChunks / 2) {
            promoteCurrentFrame();
        }
        currentFrameId = frameId;
        expectedChunks = totalChunks;
        receivedChunks = 0;
        recvFrameLen   = 0;
        frameStartMs   = millis();
    }
    uint32_t offset = (uint32_t)chunkId * CHUNK_PAYLOAD;
    if (offset + chunkSize > MAX_FRAME_SIZE) return;
    memcpy(jpegBuf[tbWrite] + offset, data + HEADER_SIZE, chunkSize);
    size_t endPos = offset + chunkSize;
    if (endPos > recvFrameLen) recvFrameLen = endPos;
    receivedChunks++;
    if (expectedChunks > 0 && receivedChunks >= expectedChunks) {
        promoteCurrentFrame();
    }
}

void udpRecvTask(void *pvParameters) {
    /* Periodic timeout so we can check frame assembly stalls */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; // 100ms
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buf[HEADER_SIZE + CHUNK_PAYLOAD];
    while (true) {
        int len = recvfrom(udp_sock, buf, sizeof(buf), 0, NULL, NULL);
        if (len > 0) processPacket(buf, len);

        /* Frame assembly timeout: accept partial or discard stale frame */
        if (expectedChunks > 0 && receivedChunks > 0 &&
            (millis() - frameStartMs > FRAME_ASSEMBLY_TIMEOUT_MS)) {
            if (receivedChunks > expectedChunks / 2) {
                promoteCurrentFrame();
            } else {
                receivedChunks = 0;
                recvFrameLen   = 0;
                expectedChunks = 0;
            }
        }
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
    firstFrameReceived = true;
}

/* Draw status dots overlay on camera view (direct GFX, bypasses LVGL) */
static void drawStatusDotsOverlay() {
    unsigned long now = millis();
    bool blinkOn = (now / 500) % 2 == 0;
    uint16_t bCol16;
    if (bridgeEverSeen && (now - lastBridgeMsg < BRIDGE_TIMEOUT_MS))
        bCol16 = 0x07E0; // GREEN
    else if (!bridgeEverSeen)
        bCol16 = blinkOn ? 0xFD40 : 0x0000; // AMBER or BLACK
    else
        bCol16 = 0xF9A6; // RED
    gfx->fillCircle(CX - 6, SCREEN_H - 6, 2, bCol16);
    uint16_t cCol16 = (WiFi.status() == WL_CONNECTED) ? 0x07E0 : 0xF9A6;
    gfx->fillCircle(CX + 6, SCREEN_H - 6, 2, cCol16);
}

/* ======================================================================
 *  SETUP
 * ====================================================================== */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TeslaCam - Display (LVGL Dashboard + Camera) ===");

    /* ── Allocate PSRAM buffers for camera ── */
    for (int i = 0; i < 3; i++) {
        jpegBuf[i] = (uint8_t *)heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM);
        if (!jpegBuf[i]) {
            Serial.printf("[FATAL] PSRAM alloc jpegBuf[%d]\n", i);
            while (true) delay(1000);
        }
    }
    screenBuf = (uint16_t *)heap_caps_malloc(SCREEN_W * SCREEN_H * 2, MALLOC_CAP_SPIRAM);
    if (!screenBuf) {
        Serial.println("[FATAL] Screen buffer alloc failed");
        while (true) delay(1000);
    }
    memset(screenBuf, 0, SCREEN_W * SCREEN_H * 2);
    Serial.printf("[MEM] Free PSRAM: %u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    tbMutex = xSemaphoreCreateMutex();

    /* ── Display init ── */
    if (!gfx->begin()) {
        Serial.println("[FATAL] GFX begin failed");
        while (true) delay(1000);
    }
    gfx->fillScreen(0x0000);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    Serial.println("[GFX] Display OK");

    /* ── LVGL init ── */
    lv_init();

    /* Allocate draw buffers in PSRAM */
    uint32_t bufSize = SCREEN_W * LV_BUF_LINES;
    lvBuf1 = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lvBuf2 = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!lvBuf1 || !lvBuf2) {
        Serial.println("[FATAL] LVGL buffer alloc failed");
        while (true) delay(1000);
    }
    lv_disp_draw_buf_init(&lvDrawBuf, lvBuf1, lvBuf2, bufSize);

    /* Display driver */
    lv_disp_drv_init(&lvDispDrv);
    lvDispDrv.hor_res = SCREEN_W;
    lvDispDrv.ver_res = SCREEN_H;
    lvDispDrv.flush_cb = lvFlushCb;
    lvDispDrv.draw_buf = &lvDrawBuf;
    lv_disp_drv_register(&lvDispDrv);

    Serial.println("[LVGL] Initialized");

    /* ── Create dashboard ── */
    createDashboard();

    /* ── Touch ── */
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    delay(50);
    Serial.println("[TOUCH] CST816S init");

    /* ── WiFi (non-blocking) ── */
    startWiFi();
    lastFrameTime = millis();

    /* First LVGL render */
    lv_timer_handler();
    Serial.println("[DASH] LVGL Dashboard ready - touch to activate camera");
}

/* ======================================================================
 *  MAIN LOOP (core 1)
 * ====================================================================== */
static unsigned long dispFrames = 0;
static unsigned long dispT0 = 0;

void loop() {
    unsigned long now = millis();

    /* ── WiFi connection management ── */
    if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("[WIFI] Connected - IP: %s\n", WiFi.localIP().toString().c_str());
        setupNetwork();
    } else if (wifiConnected && WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        Serial.println("[WIFI] Disconnected");
    }

    /* ── Touch toggle ── */
    bool touchedNow = isTouched();
    if (touchedPrev && !touchedNow && (now - lastToggleMs > TOUCH_DEBOUNCE_MS)) {
        streamActive = !streamActive;
        lastToggleMs = now;
        if (streamActive) {
            espNowPaused = true;
            sendCtrlCommand("STRT");
            lastCtrlSendMs = now;
            lastFrameTime = now;
            firstFrameReceived = false;
            Serial.println("[TOUCH] Camera ON — ESP-NOW paused");
        } else {
            espNowPaused = false;
            sendCtrlCommand("STOP");
            lastCtrlSendMs = now;
            firstFrameReceived = false;
            /* Force LVGL full redraw when returning to dashboard */
            lv_obj_invalidate(lv_scr_act());
            Serial.println("[TOUCH] Camera OFF — ESP-NOW resumed");
        }
    }
    touchedPrev = touchedNow;

    /* ── Heartbeat STRT ── */
    if (streamActive && (now - lastCtrlSendMs >= CTRL_RESEND_MS)) {
        sendCtrlCommand("STRT");
        lastCtrlSendMs = now;
    }

    /* ── Camera mode (bypass LVGL, direct JPEG) ── */
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
            /* No frame ready — show no-signal if stream timed out */
            if (firstFrameReceived && (now - lastFrameTime > FRAME_TIMEOUT_MS)) {
                bool blinkOn = (now / 500) % 2 == 0;
                gfx->fillCircle(CX, CY - 20, 6, blinkOn ? 0xF800 : 0x0000);
                drawStatusDotsOverlay();
            }
            delay(5);
        }
    }
    /* ── Dashboard mode (LVGL) ── */
    else {
        /* Drain any leftover frames */
        size_t dummyLen;
        int idx = grabReadyFrame(&dummyLen);
        if (idx >= 0) releaseDisplayBuffer();

        /* Update widget values from CAN data */
        updateDashboard();

        /* Let LVGL render (handles its own dirty tracking) */
        lv_timer_handler();
    }
}
