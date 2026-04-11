#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <driver/twai.h>
#include "valid_can_ids.h"  // Whitelist: VehicleBus + ChassisBus IDs

// =============================================================================
// ESP32-S3 CAN-to-ESP_NOW Bridge (Tesla Model 3)
// Purpose: Bridge CAN bus to ESP32 devices via ESP_NOW broadcast
// Target: LilyGo T-2CAN (ESP32-S3, MCP2515)
// =============================================================================

// ───────────────────────────────────────────────────────────────────────────
// PIN CONFIGURATION (LilyGo T-2CAN)
// ───────────────────────────────────────────────────────────────────────────
#define MCP_CS_PIN        10     // MCP2515 SPI Chip Select
#define MCP_SCK_PIN       12     // SPI Clock
#define MCP_MOSI_PIN      11     // SPI MOSI
#define MCP_MISO_PIN      13     // SPI MISO
#define MCP_RST_PIN       9      // MCP2515 Hardware Reset

// ───────────────────────────────────────────────────────────────────────────
// TWAI (ESP32-S3 built-in CAN) — ChassisBus
// ───────────────────────────────────────────────────────────────────────────
#define TWAI_TX_PIN       7      // CAN TX (T-2CAN onboard transceiver)
#define TWAI_RX_PIN       6      // CAN RX (T-2CAN onboard transceiver)

// ───────────────────────────────────────────────────────────────────────────
// CAN CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define CAN_IDMOD         0              // 0 = standard 11-bit IDs
#define CAN_SPEED         CAN_500KBPS    // 13 = 500 kbps (Tesla Model 3)
#define CAN_CRYSTAL       MCP_16MHZ      // 16 MHz crystal (LilyGo T-2CAN)

// ───────────────────────────────────────────────────────────────────────────
// WiFi CONFIGURATION (STA mode for ESP-NOW only)
// ───────────────────────────────────────────────────────────────────────────
#define WIFI_CHANNEL      1      // WiFi channel (must match Camera + Display)
#define WIFI_TX_POWER     78     // 19.5 dBm (max). Value = dBm * 4

// ───────────────────────────────────────────────────────────────────────────
// ESP_NOW CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL    0      // 0 = use current WiFi channel
#define MAX_PEERS         3

// ───────────────────────────────────────────────────────────────────────────
// LED FEEDBACK CONFIGURATION
// T-2CAN has no user LED — feedback via serial only
// ───────────────────────────────────────────────────────────────────────────

// ───────────────────────────────────────────────────────────────────────────
// TIMING & BUFFERS
// ───────────────────────────────────────────────────────────────────────────
#define CAN_RX_INTERVAL   1     // Check CAN bus every 1ms (fast polling)
#define STATS_INTERVAL    10000 // Print stats every 10s
#define DEBUG_LEVEL       2     // 1=ERROR, 2=INFO, 3=DEBUG
#define HEARTBEAT_INTERVAL_MS 1000 // Send heartbeat every 1s via ESP-NOW
#define HEARTBEAT_CAN_ID  0xFFF    // Special ID for bridge heartbeat

// ───────────────────────────────────────────────────────────────────────────
// ESP-NOW RATE LIMITING — prevent frequent CAN IDs from saturating channel
// ───────────────────────────────────────────────────────────────────────────
#define ESPNOW_MIN_INTERVAL_MS 200  // Max 5 msg/s per CAN ID via ESP-NOW

// ───────────────────────────────────────────────────────────────────────────
// DATA STRUCTURES
// ───────────────────────────────────────────────────────────────────────────

// CAN message payload for ESP_NOW broadcast
typedef struct __attribute__((packed)) {
    uint32_t can_id;        // CAN ID (11-bit or 29-bit)
    uint8_t  dlc;           // Data Length Code (0-8)
    uint8_t  data[8];       // Data bytes
    uint32_t timestamp;     // Timestamp of reception (ms)
    uint8_t  is_extended;   // 0=standard, 1=extended ID
} ESP_CAN_Message_t;

// ───────────────────────────────────────────────────────────────────────────
// GLOBAL VARIABLES
// ───────────────────────────────────────────────────────────────────────────

MCP_CAN CAN(MCP_CS_PIN);

volatile bool can_message_ready = false;
unsigned long last_can_check = 0;
unsigned long last_stats_print = 0;

// ESP-NOW per-ID rate limiting (2048 × 4 bytes = 8 KB)
static unsigned long espnow_last_sent[2048] = {0};

// ───────────────────────────────────────────────────────────────────────────
// TURN SIGNAL PRIORITY IDs
// ───────────────────────────────────────────────────────────────────────────
#define TURN_CAN_ID_UI     0x311 // UI_warning: leftBlinkerBlinking / rightBlinkerBlinking
#define TURN_CAN_ID_3F5    0x3F5 // VCFRONT_lighting (fallback)
unsigned long stats_mcp_rx = 0;       // MCP2515 (VehicleBus) frames received
unsigned long stats_twai_rx = 0;      // TWAI (ChassisBus) frames received
unsigned long stats_espnow_tx = 0;
unsigned long stats_errors = 0;
bool twai_running = false;            // TWAI driver state

bool espnow_active = false;      // true = ESP_NOW broadcasting
unsigned long last_heartbeat = 0;     // Last heartbeat send timestamp

// ───────────────────────────────────────────────────────────────────────────
// LOGGING MACROS
// ───────────────────────────────────────────────────────────────────────────
#define LOG_ERROR(msg) if(DEBUG_LEVEL >= 1) { Serial.print("[ERR] "); Serial.println(msg); }
#define LOG_INFO(msg)  if(DEBUG_LEVEL >= 2) { Serial.print("[INF] "); Serial.println(msg); }
#define LOG_DEBUG(msg) if(DEBUG_LEVEL >= 3) { Serial.print("[DBG] "); Serial.println(msg); }

// ───────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ───────────────────────────────────────────────────────────────────────────
void initializeCAN(void);
void initializeTWAI(void);
void initializeWiFiForESPNOW(void);
void initializeESPNOW(void);
void checkCANBus(void);
void checkTWAIBus(void);
void handleCANMessage(uint32_t id, uint8_t dlc, uint8_t* data);
void transmitViaESPNOW(ESP_CAN_Message_t* msg);
void sendHeartbeat(void);
void printStatistics(void);
void onESPNOWSend(const uint8_t* mac_addr, esp_now_send_status_t status);
void onESPNOWReceive(const esp_now_recv_info_t *recv_info, const uint8_t* data, int data_len);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n========================================");
    Serial.println("  ESP32-S3 CAN-to-ESP_NOW Bridge v4.3");
    Serial.println("  Tesla Model 3 Bridge");
    Serial.println("  LilyGo T-2CAN (MCP2515 + TWAI)");
    Serial.println("========================================");
    Serial.print("Board: ");
    Serial.println(ARDUINO_BOARD);
    Serial.println("========================================\n");

    // Init subsystems in order
    initializeCAN();
    initializeTWAI();
    initializeWiFiForESPNOW();
    initializeESPNOW();
    espnow_active = true;

    Serial.println("\n========================================");
    Serial.println("  READY");
    Serial.println("  Mode: CAN RX -> ESP-NOW broadcast");
    Serial.println("  WiFi: STA (ESP-NOW only)");
    Serial.println("========================================\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    unsigned long current_time = millis();

    // Check CAN bus for messages (every 1ms — fast polling)
    if (current_time - last_can_check >= CAN_RX_INTERVAL) {
        last_can_check = current_time;
        checkCANBus();
        checkTWAIBus();
    }

    // Send ESP-NOW heartbeat periodically
    if (espnow_active && (current_time - last_heartbeat >= HEARTBEAT_INTERVAL_MS)) {
        last_heartbeat = current_time;
        sendHeartbeat();
    }

    // Print statistics periodically
    if (current_time - last_stats_print >= STATS_INTERVAL) {
        last_stats_print = current_time;
        printStatistics();
    }

    // Yield to avoid watchdog and let WiFi task run
    yield();
}

// =============================================================================
// CAN BUS FUNCTIONS
// =============================================================================

void initializeCAN(void) {
    Serial.println("[CAN] Initializing MCP2515 (T-2CAN)...");

    // Hardware reset MCP2515
    pinMode(MCP_RST_PIN, OUTPUT);
    digitalWrite(MCP_RST_PIN, HIGH);
    delay(100);
    digitalWrite(MCP_RST_PIN, LOW);
    delay(100);
    digitalWrite(MCP_RST_PIN, HIGH);
    delay(100);

    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);

    byte init_result = CAN.begin(CAN_IDMOD, CAN_SPEED, CAN_CRYSTAL);
    if (init_result == CAN_OK) {
        LOG_INFO("MCP2515 initialized @ 500 kbps (16 MHz crystal)");
    } else {
        LOG_ERROR("MCP2515 init FAILED - check wiring!");
        while (1) {
            Serial.println("[ERR] MCP2515 init failed, retrying in 2s...");
            delay(2000);
            init_result = CAN.begin(CAN_IDMOD, CAN_SPEED, CAN_CRYSTAL);
            if (init_result == CAN_OK) break;
        }
    }

    // ── Hardware filters (must be set in CONFIG mode, before setMode) ──────────
    // mask=0x700 checks the top 3 bits of the 11-bit ID → accept 256-ID ranges
    // BLF analysis (2026-02-24, 127017 msgs): 338 unique IDs, 228 invalid
    // Coverage: 154/159 DBC IDs (97%) — skips 0x000-0x0FF and 0x6xx (rarely seen)
    //
    // RXB0 (mask0, filt0+1):  0x200-0x2FF (62 DBC IDs) + 0x300-0x3FF (46 DBC IDs)
    // RXB1 (mask1, filt2-5):  0x100-0x1FF (29) + 0x400-0x4FF (4) +
    //                         0x500-0x5FF (7) + 0x700-0x7FF (6) = 46 DBC IDs
    CAN.init_Mask(0, 0, 0x700);
    CAN.init_Mask(1, 0, 0x700);
    CAN.init_Filt(0, 0, 0x200);   // RXB0: 0x200-0x2FF — top traffic range
    CAN.init_Filt(1, 0, 0x300);   // RXB0: 0x300-0x3FF — 2nd traffic range
    CAN.init_Filt(2, 0, 0x100);   // RXB1: 0x100-0x1FF
    CAN.init_Filt(3, 0, 0x400);   // RXB1: 0x400-0x4FF (0x401 BrickVoltages ~1248/s)
    CAN.init_Filt(4, 0, 0x500);   // RXB1: 0x500-0x5FF
    CAN.init_Filt(5, 0, 0x700);   // RXB1: 0x700-0x7FF (UI odometer, range…)

    CAN.setMode(MCP_LISTENONLY);  // CRITICAL: passive only, no TX, no ACK on bus

    // T-2CAN: no INT pin — use polling only (checkReceive in loop)
    Serial.println("[CAN] Ready (500 kbps, 16 MHz, HW filter, polling mode)\n");
}

void checkCANBus(void) {
    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    unsigned long can_id = 0;
    byte dlc = 0;
    // IMPORTANT: buffer de 16 octets (pas 8) car la lib mcp_can copie
    // m_nDlc octets et le DLC du MCP2515 peut aller jusqu'à 15 (4 bits).
    // Avec un buffer de 8 et du bruit sur le bus, c'est un stack overflow.
    byte data[16] = {0};

    byte result = CAN.readMsgBuf(&can_id, &dlc, data);
    if (result == CAN_OK) {
        // Clamp DLC to CAN 2.0 max (8 bytes)
        if (dlc > 8) dlc = 8;

        stats_mcp_rx++;

        // Compact log (only every 100th message at DEBUG_LEVEL 2)
        if (DEBUG_LEVEL >= 3 || (stats_mcp_rx % 100 == 0)) {
            Serial.printf("[MCP] #%lu ID:0x%03lX DLC:%d\n",
                stats_mcp_rx, can_id, dlc);
        }

        handleCANMessage(can_id, dlc, data);
    } else {
        stats_errors++;
    }
}

// =============================================================================
// TWAI (ESP32-S3 built-in CAN) — ChassisBus listen-only
// =============================================================================

void initializeTWAI(void) {
    Serial.println("[TWAI] Initializing (ChassisBus, listen-only)...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = 32;  // Deeper RX queue for burst traffic

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[TWAI] Driver install FAILED: 0x%X\n", err);
        return;
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[TWAI] Start FAILED: 0x%X\n", err);
        twai_driver_uninstall();
        return;
    }

    twai_running = true;
    Serial.printf("[TWAI] Ready (500 kbps, listen-only, TX=%d RX=%d)\n",
                  TWAI_TX_PIN, TWAI_RX_PIN);
}

void checkTWAIBus(void) {
    if (!twai_running) return;

    twai_message_t rx_msg;
    // Drain all available messages (non-blocking)
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        // Skip RTR and extended frames (Tesla uses standard 11-bit)
        if (rx_msg.rtr || rx_msg.extd) continue;

        uint8_t dlc = rx_msg.data_length_code;
        if (dlc > 8) dlc = 8;

        stats_twai_rx++;
        if (DEBUG_LEVEL >= 3 || (stats_twai_rx % 100 == 0)) {
            Serial.printf("[TWAI] #%lu ID:0x%03lX DLC:%d\n",
                stats_twai_rx, (unsigned long)rx_msg.identifier, dlc);
        }

        handleCANMessage(rx_msg.identifier, dlc, rx_msg.data);
    }
}

// Binary search helper on a sorted uint16_t array
static bool bsearch16(const uint16_t* arr, uint16_t count, uint16_t id) {
    int left = 0, right = count - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (arr[mid] == id) return true;
        if (arr[mid] < id) left = mid + 1;
        else right = mid - 1;
    }
    return false;
}

// Check if CAN ID is in either VehicleBus or ChassisBus whitelist
bool isValidCanId(uint32_t id) {
    uint16_t id16 = (uint16_t)(id & 0x7FF);
    return bsearch16(VEHICLE_BUS_IDS, VEHICLE_BUS_IDS_COUNT, id16)
        || bsearch16(CHASSIS_BUS_IDS, CHASSIS_BUS_IDS_COUNT, id16);
}

void handleCANMessage(uint32_t id, uint8_t dlc, uint8_t* data) {
    // FILTER: Only process valid Tesla CAN IDs
    if (!isValidCanId(id)) {
        stats_errors++;
        return;
    }

    ESP_CAN_Message_t msg;
    msg.can_id = id;
    msg.dlc = dlc;
    msg.timestamp = millis();
    msg.is_extended = 0;
    memcpy(msg.data, data, dlc);
    if (dlc < 8) memset(msg.data + dlc, 0, 8 - dlc);

    // ESP-NOW rate limiting: max 1 per ESPNOW_MIN_INTERVAL_MS per CAN ID
    // Priority IDs bypass rate limiting entirely
    uint16_t idx = id & 0x7FF;
    bool priority = (id == 0x257 || id == 0x266 || id == 0x2E5 || id == 0x33A ||
                     id == 0x292 ||
                     id == TURN_CAN_ID_UI || id == TURN_CAN_ID_3F5 || id == 0x249);
    if (priority || msg.timestamp - espnow_last_sent[idx] >= ESPNOW_MIN_INTERVAL_MS) {
        espnow_last_sent[idx] = msg.timestamp;
        transmitViaESPNOW(&msg);
    }
}

// =============================================================================
// WiFi STA (for ESP-NOW only, no Access Point)
// =============================================================================

void initializeWiFiForESPNOW(void) {
    Serial.println("[WiFi] Initializing STA mode for ESP-NOW...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    WiFi.setSleep(false);
    Serial.println("[WiFi] Power save: OFF");

    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(WIFI_TX_POWER);

    Serial.printf("[WiFi] STA mode ready (channel %d, TX %.1f dBm)\n",
                  WIFI_CHANNEL, WIFI_TX_POWER / 4.0);
    Serial.print("[WiFi] STA MAC: ");
    Serial.println(WiFi.macAddress());
}

// =============================================================================
// ESP_NOW
// =============================================================================

void initializeESPNOW(void) {
    Serial.println("[ESP_NOW] Initializing...");

    if (esp_now_init() != ESP_OK) {
        LOG_ERROR("ESP_NOW init failed!");
        return;
    }

    esp_now_register_send_cb(onESPNOWSend);
    esp_now_register_recv_cb(onESPNOWReceive);

    // Broadcast peer — use WIFI_CHANNEL explicitly for consistency
    esp_now_peer_info_t peer_info = {};
    memset(&peer_info, 0, sizeof(peer_info));
    peer_info.channel = WIFI_CHANNEL;
    peer_info.encrypt = false;
    memset(peer_info.peer_addr, 0xFF, 6);
    peer_info.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        LOG_ERROR("Failed to add ESP_NOW broadcast peer");
        return;
    }

    Serial.print("[ESP_NOW] STA MAC: ");
    uint8_t mac[6];
    WiFi.macAddress(mac);
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X%s", mac[i], i < 5 ? ":" : "");
    }
    Serial.println();
}

void sendHeartbeat(void) {
    ESP_CAN_Message_t hb;
    hb.can_id = HEARTBEAT_CAN_ID;
    hb.dlc = 0;
    memset(hb.data, 0, 8);
    hb.timestamp = millis();
    hb.is_extended = 0;
    transmitViaESPNOW(&hb);
}

void transmitViaESPNOW(ESP_CAN_Message_t* msg) {
    if (!espnow_active) return;  // Skip if ESP_NOW not active

    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcast_addr, (uint8_t*)msg, sizeof(ESP_CAN_Message_t));

    // Stats
    if (result == ESP_OK) {
        stats_espnow_tx++;
    } else {
        stats_errors++;
    }
}

void onESPNOWSend(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        LOG_DEBUG("ESP_NOW send failed");
    }
}

void onESPNOWReceive(const esp_now_recv_info_t *recv_info, const uint8_t* data, int data_len) {
    LOG_DEBUG("ESP_NOW RX from peer");
}

// =============================================================================
// STATISTICS
// =============================================================================

void printStatistics(void) {
    Serial.println("\n--- Stats ---");
    Serial.printf("  Mode:         ESP_NOW only\n");
    Serial.printf("  MCP RX:       %lu (VehicleBus)\n", stats_mcp_rx);
    Serial.printf("  TWAI RX:      %lu (ChassisBus)\n", stats_twai_rx);
    Serial.printf("  ESP_NOW TX:   %lu\n", stats_espnow_tx);
    Serial.printf("  Errors:       %lu\n", stats_errors);
    Serial.printf("  Free heap:    %lu\n", ESP.getFreeHeap());
    Serial.printf("  Uptime:       %lu s\n", millis() / 1000);
    Serial.println("-------------\n");
}

// =============================================================================
// END OF FILE
// =============================================================================
