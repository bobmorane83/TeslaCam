/*
 * LVGL 9.5 – Animated green arc gauge on JC3636W518C (ST77916, 360×360 round)
 *
 * Green arc (15 px wide) on the extreme edge, spans 270°,
 * fills in 10 s then resets to 0 and repeats.
 */

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "lvgl.h"

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

/* ── LVGL flush callback ────────────────────────────────────────────── */
static void my_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_display_flush_ready(disp);
}

/* ── Animation callback ─────────────────────────────────────────────── */
static lv_obj_t *arc;

static void arc_anim_cb(void *obj, int32_t value)
{
    lv_arc_set_value((lv_obj_t *)obj, value);
}

static void arc_anim_completed_cb(lv_anim_t *a)
{
    /* Restart from 0 */
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, arc);
    lv_anim_set_exec_cb(&anim, arc_anim_cb);
    lv_anim_set_values(&anim, 0, 270);
    lv_anim_set_duration(&anim, 10000);
    lv_anim_set_completed_cb(&anim, arc_anim_completed_cb);
    lv_anim_start(&anim);
}

/* ── Setup ───────────────────────────────────────────────────────────── */
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("LVGL 9.5 arc gauge – init");

    /* Display hardware */
    if (!gfx->begin()) {
        Serial.println("GFX begin FAILED");
        while (true) delay(1000);
    }
    gfx->fillScreen(0x0000);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    Serial.println("Display OK");

    /* LVGL init */
    lv_init();
    Serial.println("lv_init OK");

    /* Create display */
    lv_display_t *disp = lv_display_create(SCREEN_W, SCREEN_H);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, my_flush_cb);

    /* Allocate double buffers in PSRAM */
    static const uint32_t buf_px = SCREEN_W * 40;           /* 40 lines */
    static const uint32_t buf_bytes = buf_px * sizeof(uint16_t);
    uint8_t *buf1 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    uint8_t *buf2 = (uint8_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    if (!buf1 || !buf2) {
        Serial.println("PSRAM alloc FAILED");
        while (true) delay(1000);
    }
    lv_display_set_buffers(disp, buf1, buf2, buf_bytes,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    Serial.println("Display buffers OK");

    /* ── Build UI ────────────────────────────────────────────────────── */
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* Arc widget */
    arc = lv_arc_create(scr);
    lv_obj_set_size(arc, SCREEN_W, SCREEN_H);
    lv_obj_center(arc);

    /* Remove the knob */
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);
    lv_obj_set_style_size(arc, 0, 0, LV_PART_KNOB);

    /* Background arc (track) – dark gray, 15 px */
    lv_obj_set_style_arc_width(arc, 15, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_make(40, 40, 40), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);

    /* Indicator arc – green, 15 px */
    lv_obj_set_style_arc_width(arc, 15, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_make(0, 220, 60), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);

    /* Angles: 270° sweep  (default is 135→45 which is 270° already) */
    lv_arc_set_bg_angles(arc, 135, 45);
    lv_arc_set_range(arc, 0, 270);
    lv_arc_set_value(arc, 0);
    lv_arc_set_mode(arc, LV_ARC_MODE_NORMAL);

    /* Disable touch interaction */
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    /* ── Start animation ─────────────────────────────────────────────── */
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, arc);
    lv_anim_set_exec_cb(&anim, arc_anim_cb);
    lv_anim_set_values(&anim, 0, 270);
    lv_anim_set_duration(&anim, 10000);
    lv_anim_set_completed_cb(&anim, arc_anim_completed_cb);
    lv_anim_start(&anim);

    Serial.println("Arc gauge ready – animation started");
}

/* ── Loop ────────────────────────────────────────────────────────────── */
void loop()
{
    lv_timer_handler();
    delay(5);
}
