#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ── WiFi ────────────────────────────────────────────────────────────────────────
static const char *WIFI_SSID = "nans6";
static const char *WIFI_PASS = "Devisubox";

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

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=frame"
#define STREAM_BOUNDARY "\r\n--frame\r\n"
#define STREAM_PART "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

static httpd_handle_t camera_httpd = nullptr;

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
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;

    if (psramFound()) {
        cfg.frame_size   = FRAMESIZE_VGA;
        cfg.jpeg_quality = 12;
        cfg.fb_count     = 2;
        cfg.fb_location  = CAMERA_FB_IN_PSRAM;
        Serial.println("[CAM] PSRAM – VGA, 2 buffers");
    } else {
        cfg.frame_size   = FRAMESIZE_QVGA;
        cfg.jpeg_quality = 15;
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
    if (s) Serial.printf("[CAM] Sensor PID: 0x%04X\n", s->id.PID);
    Serial.println("[CAM] Camera ready");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────────────────────────────────────────
void connectWiFi() {
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
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

// ─────────────────────────────────────────────────────────────────────────────────
//  MJPEG stream handler (ESP-IDF httpd)
// ─────────────────────────────────────────────────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "25");

    unsigned long frames = 0;
    unsigned long t0 = millis();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Capture fail");
            res = ESP_FAIL;
            break;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)fb->len);

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;

        frames++;
        if (frames % 30 == 0) {
            float fps = frames * 1000.0f / (millis() - t0);
            Serial.printf("[STREAM] %lu frames, %.1f fps\n", frames, fps);
        }
    }
    Serial.printf("[STREAM] Done: %lu frames\n", frames);
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Throughput test handler (sends 1MB of DRAM data)
// ─────────────────────────────────────────────────────────────────────────────────
static esp_err_t test_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/octet-stream");
    static uint8_t buf[4096];
    memset(buf, 0xAA, sizeof(buf));

    unsigned long t0 = millis();
    size_t total = 0;
    for (int i = 0; i < 256; i++) {  // 256 * 4KB = 1MB
        esp_err_t res = httpd_resp_send_chunk(req, (const char *)buf, sizeof(buf));
        if (res != ESP_OK) break;
        total += sizeof(buf);
    }
    httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
    unsigned long dt = millis() - t0;
    Serial.printf("[TEST] Sent %u bytes in %lu ms = %.0f KB/s\n",
                  (unsigned)total, dt, total / 1.024f / dt);
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Single capture handler
// ─────────────────────────────────────────────────────────────────────────────────
static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    Serial.printf("[CAPTURE] Sent %u bytes\n", (unsigned)fb->len);
    return res;
}

void startHTTPD() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 5000;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 4;

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_uri_t stream_uri = { .uri = "/", .method = HTTP_GET, .handler = stream_handler };
        httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler };
        httpd_uri_t test_uri = { .uri = "/test", .method = HTTP_GET, .handler = test_handler };
        httpd_register_uri_handler(camera_httpd, &stream_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &test_uri);
        Serial.println("[HTTP] Server started");
    }
}

// ─────────────────────────────────────────────────────────────────────────────────
//  Setup & Loop
// ─────────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32-S3 MJPEG Streamer ===");

    pinMode(48, OUTPUT);
    digitalWrite(48, LOW);
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed – halting");
        while (true) delay(1000);
    }

    connectWiFi();
    startHTTPD();
    Serial.printf("[STREAM] http://%s:5000/         MJPEG stream\n", WiFi.localIP().toString().c_str());
    Serial.printf("[STREAM] http://%s:5000/capture  Single JPEG\n", WiFi.localIP().toString().c_str());
    Serial.printf("[STREAM] http://%s:5000/test     1MB throughput test\n", WiFi.localIP().toString().c_str());
}

void loop() {
    delay(1000);
}
