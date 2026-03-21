#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "valid_can_ids.h"  // Whitelist of 159 valid Tesla CAN IDs

// =============================================================================
// ESP32 CAN-to-WiFi Hub (Tesla Model 3)
// Purpose: Bridge CAN bus to ESP32 devices via ESP_NOW + WiFi AP UDP stream
// Target: NodeMCU-ESP32 DEVKITV1
// =============================================================================

// ───────────────────────────────────────────────────────────────────────────
// PIN CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define MCP_CS_PIN        25     // MCP2515 SPI Chip Select (GPIO 25)
#define MCP_INT_PIN       26     // MCP2515 Interrupt (GPIO 26)
#define LED_PIN           2      // Built-in LED (visual feedback)

// ───────────────────────────────────────────────────────────────────────────
// CAN CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define CAN_IDMOD         0              // 0 = standard 11-bit IDs
#define CAN_SPEED         CAN_500KBPS    // 13 = 500 kbps (Tesla Model 3)
#define CAN_CRYSTAL       MCP_8MHZ       // 2 = 8 MHz crystal (actual oscillator)

// ───────────────────────────────────────────────────────────────────────────
// WiFi ACCESS POINT CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define AP_SSID           "bridge"
#define AP_PASSWORD       "teslamodel3"
#define AP_CHANNEL        1      // WiFi channel (must match Camera + Display)
#define AP_MAX_CLIENTS    4
#define AP_BEACON_MS      100    // Beacon interval (default 100ms)
#define WIFI_TX_POWER     78     // 19.5 dBm (max). Value = dBm * 4

// ───────────────────────────────────────────────────────────────────────────
// UDP BROADCAST CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define UDP_PORT          5556
#define UDP_BROADCAST_IP  IPAddress(192, 168, 4, 255)  // Broadcast on AP subnet

// ───────────────────────────────────────────────────────────────────────────
// ESP_NOW CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL    0      // 0 = use current WiFi channel
#define MAX_PEERS         3
#define ESPNOW_ENABLED_INIT false  // ESP_NOW off at boot (WiFi AP mode)

// ───────────────────────────────────────────────────────────────────────────
// AP AUTO-SHUTDOWN CONFIGURATION
// If no WiFi client connects within this delay after boot,
// the AP is permanently shut down and ESP_NOW takes over.
// ───────────────────────────────────────────────────────────────────────────
#define AP_TIMEOUT_MS     180000  // 3 minutes (180 000 ms)

// ───────────────────────────────────────────────────────────────────────────
// LED FEEDBACK CONFIGURATION
// ───────────────────────────────────────────────────────────────────────────
#define LED_PULSE_DURATION  10   // 10ms pulse when CAN message received
#define LED_OFF_TIME        50   // Minimum time between pulses

// ───────────────────────────────────────────────────────────────────────────
// TIMING & BUFFERS
// ───────────────────────────────────────────────────────────────────────────
#define CAN_RX_INTERVAL   1     // Check CAN bus every 1ms (fast polling)
#define STATS_INTERVAL    10000 // Print stats every 10s
#define DEBUG_LEVEL       2     // 1=ERROR, 2=INFO, 3=DEBUG
#define HEARTBEAT_INTERVAL_MS 2000 // Send heartbeat every 2s via ESP-NOW
#define HEARTBEAT_CAN_ID  0xFFF    // Special ID for bridge heartbeat

// ───────────────────────────────────────────────────────────────────────────
// UDP BATCHING — critical for WiFi stability
// Instead of 1 UDP packet per CAN frame (1000+/s = WiFi death),
// we buffer frames and flush every UDP_FLUSH_INTERVAL_MS.
// Max batch: ~1400 bytes / 18 bytes per frame = 77 frames per UDP packet.
// ───────────────────────────────────────────────────────────────────────────
#define UDP_FLUSH_INTERVAL_MS   20    // Flush every 20ms (50 UDP packets/s max)
#define UDP_BUFFER_MAX_FRAMES   60    // Max frames per batch (60 * 18 = 1080 bytes)
#define UDP_BUFFER_SIZE         (UDP_BUFFER_MAX_FRAMES * 18)  // 1080 bytes

// ───────────────────────────────────────────────────────────────────────────
// UDP PACKET FORMAT (18 bytes per CAN frame)
// ───────────────────────────────────────────────────────────────────────────
// Byte 0:     Magic byte 0xCA
// Byte 1:     Magic byte 0x4E ("CAN" marker)
// Byte 2-5:   CAN ID (uint32_t, little-endian)
// Byte 6:     DLC (0-8)
// Byte 7:     Flags (bit0: is_extended)
// Byte 8-15:  Data bytes (8 bytes, zero-padded)
// Byte 16-17: Sequence number (uint16_t, little-endian)
// ───────────────────────────────────────────────────────────────────────────
#define UDP_PACKET_SIZE   18
#define UDP_MAGIC_0       0xCA
#define UDP_MAGIC_1       0x4E

// ───────────────────────────────────────────────────────────────────────────
// DATA STRUCTURES
// ───────────────────────────────────────────────────────────────────────────

// CAN message payload (shared between ESP_NOW and UDP)
typedef struct __attribute__((packed)) {
    uint32_t can_id;        // CAN ID (11-bit or 29-bit)
    uint8_t  dlc;           // Data Length Code (0-8)
    uint8_t  data[8];       // Data bytes
    uint32_t timestamp;     // Timestamp of reception (ms)
    uint8_t  is_extended;   // 0=standard, 1=extended ID
} ESP_CAN_Message_t;

// LED state machine
typedef struct {
    bool is_on;
    unsigned long pulse_start;
    bool should_pulse;
} LED_State_t;

// ───────────────────────────────────────────────────────────────────────────
// GLOBAL VARIABLES
// ───────────────────────────────────────────────────────────────────────────

MCP_CAN CAN(MCP_CS_PIN);
WiFiUDP udp;
LED_State_t led_state = {false, 0, false};

volatile bool can_message_ready = false;
unsigned long last_can_check = 0;
unsigned long last_stats_print = 0;
unsigned long stats_messages_rx = 0;
unsigned long stats_espnow_tx = 0;
unsigned long stats_udp_tx = 0;
unsigned long stats_udp_packets = 0;  // Actual UDP packets sent (batches)
unsigned long stats_udp_skip = 0;
unsigned long stats_errors = 0;
uint16_t udp_sequence = 0;
uint8_t  ap_clients_count = 0;
unsigned long last_client_check = 0;
#define CLIENT_CHECK_INTERVAL 2000

// AP auto-shutdown state
bool ap_active = true;           // true = WiFi AP running, false = AP shut down
bool espnow_active = false;      // true = ESP_NOW broadcasting
unsigned long ap_no_client_since = 0;  // millis() when last client disconnected (0 = boot)

// UDP batch buffer
uint8_t  udp_buffer[UDP_BUFFER_SIZE];
uint16_t udp_buffer_offset = 0;       // Current write position in buffer
uint8_t  udp_buffer_count = 0;        // Number of frames in buffer
unsigned long last_udp_flush = 0;     // Last flush timestamp
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
void initializeWiFiAP(void);
void initializeESPNOW(void);
void initializeUDP(void);
void checkCANBus(void);
void handleCANMessage(uint32_t id, uint8_t dlc, uint8_t* data);
void transmitViaESPNOW(ESP_CAN_Message_t* msg);
void sendHeartbeat(void);
void bufferForUDP(ESP_CAN_Message_t* msg);
void flushUDPBuffer(void);
void updateLED(void);
void printStatistics(void);
void onESPNOWSend(const uint8_t* mac_addr, esp_now_send_status_t status);
void onESPNOWReceive(const uint8_t* mac_addr, const uint8_t* data, int data_len);
void shutdownAPAndStartESPNOW(void);
void onWiFiEvent(WiFiEvent_t event);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n========================================");
    Serial.println("  ESP32 CAN-to-WiFi Hub v3.0");
    Serial.println("  Tesla Model 3 Bridge");
    Serial.println("  WiFi AP + Batched UDP");
    Serial.println("========================================");
    Serial.print("Board: ");
    Serial.println(ARDUINO_BOARD);
    Serial.println("========================================\n");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Init subsystems in order
    initializeCAN();
    initializeWiFiAP();
    initializeESPNOW();
    espnow_active = true;
    Serial.printf("[AP] Auto-shutdown dans %d s si aucun client\n", AP_TIMEOUT_MS / 1000);
    initializeUDP();

    Serial.println("\n========================================");
    Serial.println("  READY");
    Serial.println("  CAN RX -> Batched UDP TX");
    Serial.printf("  Flush interval: %d ms\n", UDP_FLUSH_INTERVAL_MS);
    Serial.print("  WiFi AP: ");
    Serial.print(AP_SSID);
    Serial.print(" @ ");
    Serial.println(WiFi.softAPIP());
    Serial.print("  UDP Port: ");
    Serial.println(UDP_PORT);
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
    }

    // Send ESP-NOW heartbeat periodically
    if (espnow_active && (current_time - last_heartbeat >= HEARTBEAT_INTERVAL_MS)) {
        last_heartbeat = current_time;
        sendHeartbeat();
    }

    // ── AP auto-shutdown logic ──────────────────────────────────────────
    // If AP is active and no client connected for AP_TIMEOUT_MS,
    // shut down AP and switch to ESP_NOW-only mode.
    // This triggers both at boot (no client ever) and after last client leaves.
    if (ap_active && ap_clients_count == 0 &&
        (current_time - ap_no_client_since >= AP_TIMEOUT_MS)) {
        shutdownAPAndStartESPNOW();
    }

    // Flush UDP buffer periodically (only when AP is active)
    if (ap_active && (current_time - last_udp_flush >= UDP_FLUSH_INTERVAL_MS)) {
        last_udp_flush = current_time;
        flushUDPBuffer();
    }

    // Update LED pulse state machine (only when AP is active)
    if (ap_active) {
        updateLED();
    }

    // Periodically refresh client count (only when AP is active)
    if (ap_active && (current_time - last_client_check >= CLIENT_CHECK_INTERVAL)) {
        last_client_check = current_time;
        uint8_t prev_count = ap_clients_count;
        ap_clients_count = WiFi.softAPgetStationNum();
        // While clients are connected, keep resetting the no-client timer
        if (ap_clients_count > 0) {
            ap_no_client_since = current_time;
        }
    }

    // Print statistics periodically (only when AP is active)
    if (ap_active && (current_time - last_stats_print >= STATS_INTERVAL)) {
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
    Serial.println("[CAN] Initializing MCP2515...");

    SPI.begin(18, 19, 23, MCP_CS_PIN);

    byte init_result = CAN.begin(CAN_IDMOD, CAN_SPEED, CAN_CRYSTAL);
    if (init_result == CAN_OK) {
        LOG_INFO("MCP2515 initialized @ 500 kbps");
    } else {
        LOG_ERROR("MCP2515 init FAILED - check wiring!");
        while (1) {
            digitalWrite(LED_PIN, HIGH); delay(200);
            digitalWrite(LED_PIN, LOW);  delay(200);
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

    // Interrupt on CAN message arrival
    pinMode(MCP_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), [] {
        can_message_ready = true;
    }, FALLING);

    Serial.println("[CAN] Ready (500 kbps, HW filter: 0x100-0x5FF + 0x700-0x7FF, 97% DBC coverage)\n");
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

        // Stats & logging only when AP is active (save CPU in ESP_NOW-only mode)
        if (ap_active) {
            stats_messages_rx++;

            // Compact log (only every 100th message at DEBUG_LEVEL 2)
            if (DEBUG_LEVEL >= 3 || (stats_messages_rx % 100 == 0)) {
                Serial.printf("[CAN] #%lu ID:0x%03lX DLC:%d\n",
                    stats_messages_rx, can_id, dlc);
            }
        }

        handleCANMessage(can_id, dlc, data);
        if (ap_active) led_state.should_pulse = true;
    } else {
        if (ap_active) stats_errors++;
    }
}

// Check if CAN ID is in the whitelist of valid Tesla messages
bool isValidCanId(uint32_t id) {
    // Binary search for faster lookup (159 IDs)
    int left = 0, right = VALID_CAN_IDS_COUNT - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (VALID_CAN_IDS[mid] == id) return true;
        if (VALID_CAN_IDS[mid] < id) left = mid + 1;
        else right = mid - 1;
    }
    return false;
}

void handleCANMessage(uint32_t id, uint8_t dlc, uint8_t* data) {
    // FILTER: Only process valid Tesla CAN IDs
    if (!isValidCanId(id)) {
        if (ap_active) stats_errors++;  // Count as invalid (only in AP mode)
        return;
    }

    ESP_CAN_Message_t msg;
    msg.can_id = id;
    msg.dlc = dlc;
    msg.timestamp = millis();
    msg.is_extended = 0;
    memcpy(msg.data, data, dlc);
    if (dlc < 8) memset(msg.data + dlc, 0, 8 - dlc);

    // Dual path: ESP_NOW (if enabled) + buffer for batched UDP
    transmitViaESPNOW(&msg);
    bufferForUDP(&msg);
}

// =============================================================================
// WiFi ACCESS POINT (AP+STA mode for ESP_NOW compatibility)
// =============================================================================

void onWiFiEvent(WiFiEvent_t event) {
    // IMPORTANT: Ne PAS appeler WiFi.softAPgetStationNum() ici !
    // Les callbacks WiFi doivent être ultra-légers.
    // Le comptage des clients se fait dans loop() via polling.
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("[WiFi] Client connecté");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("[WiFi] Client déconnecté");
            // Reset the no-client timer — timeout restarts from now
            ap_no_client_since = millis();
            break;
        default:
            break;
    }
}

void initializeWiFiAP(void) {
    Serial.println("[WiFi] Initializing...");

    // 1. Register event handler (léger, pas de WiFi API dedans)
    WiFi.onEvent(onWiFiEvent);

    // 2. Set mode — AP only (pas de STA scan en arrière-plan)
    WiFi.mode(WIFI_AP);
    Serial.println("[WiFi] Mode: AP seul");

    // 3. Config IP explicite AVANT softAP pour DHCP prévisible
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // 4. Démarrer l'AP — ceci appelle esp_wifi_start() en interne
    bool ap_ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    if (!ap_ok) {
        Serial.println("[WiFi] ERREUR: softAP a échoué !");
        return;
    }
    delay(500);  // Attendre stabilisation complète de l'AP

    // 5. APRÈS softAP : désactiver power save
    //    CRITIQUE: esp_wifi_set_ps() DOIT être appelé APRÈS esp_wifi_start()
    //    Sinon l'appel échoue silencieusement et le power save reste actif !
    WiFi.setSleep(false);
    Serial.println("[WiFi] Power save: OFF (post-start)");

    // 6. APRÈS softAP : régler la puissance TX max
    esp_wifi_set_max_tx_power(WIFI_TX_POWER);
    Serial.printf("[WiFi] TX power: %.1f dBm\n", WIFI_TX_POWER / 4.0);

    // NE PAS appeler esp_wifi_set_protocol() ni esp_wifi_set_bandwidth()
    // après softAP — les défauts (11b/g/n, HT20 en AP seul) sont déjà optimaux.
    // Ces appels post-start forcent une reconfiguration radio qui déconnecte les clients.

    Serial.print("[WiFi] AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("[WiFi] AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.printf("[WiFi] AP Channel: %d, Max clients: %d\n",
                  AP_CHANNEL, AP_MAX_CLIENTS);
}

// =============================================================================
// UDP BROADCAST
// =============================================================================

void initializeUDP(void) {
    Serial.println("[UDP] Initializing batched mode...");
    udp.begin(UDP_PORT);
    memset(udp_buffer, 0, sizeof(udp_buffer));
    udp_buffer_offset = 0;
    udp_buffer_count = 0;
    Serial.printf("[UDP] Port %d, batch every %d ms, max %d frames/batch\n",
                  UDP_PORT, UDP_FLUSH_INTERVAL_MS, UDP_BUFFER_MAX_FRAMES);
    Serial.printf("[UDP] Broadcast IP: %s\n", UDP_BROADCAST_IP.toString().c_str());
}

void bufferForUDP(ESP_CAN_Message_t* msg) {
    // Skip UDP buffering entirely when AP is shut down
    if (!ap_active) return;

    // Add a CAN frame to the UDP send buffer (18 bytes per frame)
    // Will be flushed as a single UDP packet from the main loop
    if (udp_buffer_count >= UDP_BUFFER_MAX_FRAMES) {
        // Buffer full — force flush now
        flushUDPBuffer();
    }

    uint8_t* p = &udp_buffer[udp_buffer_offset];

    // Magic bytes
    p[0] = UDP_MAGIC_0;
    p[1] = UDP_MAGIC_1;

    // CAN ID (little-endian)
    p[2] = (msg->can_id) & 0xFF;
    p[3] = (msg->can_id >> 8) & 0xFF;
    p[4] = (msg->can_id >> 16) & 0xFF;
    p[5] = (msg->can_id >> 24) & 0xFF;

    // DLC + flags
    p[6] = msg->dlc;
    p[7] = msg->is_extended;

    // Data bytes
    memcpy(&p[8], msg->data, 8);

    // Sequence number (little-endian)
    p[16] = udp_sequence & 0xFF;
    p[17] = (udp_sequence >> 8) & 0xFF;
    udp_sequence++;

    udp_buffer_offset += UDP_PACKET_SIZE;
    udp_buffer_count++;
    stats_udp_tx++;  // Count individual frames buffered
}

void flushUDPBuffer(void) {
    // Send all buffered frames as a single UDP packet
    if (udp_buffer_count == 0) return;

    // Skip if no clients connected
    if (ap_clients_count == 0) {
        stats_udp_skip += udp_buffer_count;
        udp_buffer_offset = 0;
        udp_buffer_count = 0;
        return;
    }

    // Send one big UDP packet containing all buffered frames
    udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
    udp.write(udp_buffer, udp_buffer_offset);
    if (udp.endPacket()) {
        stats_udp_packets++;
    }

    // Reset buffer
    udp_buffer_offset = 0;
    udp_buffer_count = 0;
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

    // Broadcast peer — use AP_CHANNEL explicitly for consistency
    esp_now_peer_info_t peer_info = {};
    memset(&peer_info, 0, sizeof(peer_info));
    peer_info.channel = AP_CHANNEL;
    peer_info.encrypt = false;
    memset(peer_info.peer_addr, 0xFF, 6);

    // Set correct interface based on current WiFi mode
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    peer_info.ifidx = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA)
                      ? WIFI_IF_AP : WIFI_IF_STA;
    Serial.printf("[ESP_NOW] Interface: %s\n",
                  peer_info.ifidx == WIFI_IF_AP ? "AP" : "STA");

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

    // Stats only in AP mode (save CPU in ESP_NOW-only mode)
    if (ap_active) {
        if (result == ESP_OK) {
            stats_espnow_tx++;
        } else {
            stats_errors++;
        }
    }
}

void onESPNOWSend(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (ap_active && status != ESP_NOW_SEND_SUCCESS) {
        LOG_DEBUG("ESP_NOW send failed");
    }
}

void onESPNOWReceive(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    if (ap_active) LOG_DEBUG("ESP_NOW RX from peer");
}

// =============================================================================
// AP AUTO-SHUTDOWN & ESP_NOW TAKEOVER
// =============================================================================

void shutdownAPAndStartESPNOW(void) {
    Serial.println("\n========================================");
    Serial.println("  AP TIMEOUT — No client after 3 min");
    Serial.println("  Switching to ESP_NOW-only mode");
    Serial.println("========================================\n");

    // 1. Stop UDP
    udp.stop();
    udp_buffer_offset = 0;
    udp_buffer_count = 0;
    Serial.println("[UDP] Stopped");

    // 2. Deinit ESP-NOW before WiFi mode change
    esp_now_deinit();
    espnow_active = false;
    Serial.println("[ESP_NOW] Deinit for mode switch");

    // 3. Shut down WiFi AP
    WiFi.softAPdisconnect(true);
    Serial.println("[WiFi] AP shut down");

    // 4. Switch to STA mode (required for ESP_NOW, no AP overhead)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    esp_wifi_set_channel(AP_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.println("[WiFi] Mode: STA (no connection, ESP_NOW carrier)");

    // 5. Re-initialize ESP_NOW on STA interface
    initializeESPNOW();
    espnow_active = true;
    Serial.println("[ESP_NOW] Re-activated on STA interface");

    // 6. Update state flags
    ap_active = false;
    ap_clients_count = 0;

    // 7. Turn off LED and stop all debug output
    digitalWrite(LED_PIN, LOW);
    led_state.is_on = false;
    led_state.should_pulse = false;

    Serial.println("\n========================================");
    Serial.println("  MODE: ESP_NOW ONLY");
    Serial.println("  All CPU dedicated to CAN → ESP_NOW");
    Serial.println("  LED: OFF  |  Serial: errors only");
    Serial.printf("  Free heap: %lu bytes\n", ESP.getFreeHeap());
    Serial.println("========================================\n");
}

// =============================================================================
// LED CONTROL
// =============================================================================

void updateLED(void) {
    unsigned long t = millis();

    if (led_state.should_pulse) {
        digitalWrite(LED_PIN, HIGH);
        led_state.pulse_start = t;
        led_state.is_on = true;
        led_state.should_pulse = false;
    }

    if (led_state.is_on && (t - led_state.pulse_start >= LED_PULSE_DURATION)) {
        digitalWrite(LED_PIN, LOW);
        led_state.is_on = false;
    }
}

// =============================================================================
// STATISTICS
// =============================================================================

void printStatistics(void) {
    Serial.println("\n--- Stats ---");
    Serial.printf("  Mode:         %s\n", ap_active ? "WiFi AP + UDP" : "ESP_NOW only");
    Serial.printf("  CAN RX:       %lu\n", stats_messages_rx);
    if (espnow_active) {
        Serial.printf("  ESP_NOW TX:   %lu\n", stats_espnow_tx);
    }
    Serial.printf("  UDP frames:   %lu\n", stats_udp_tx);
    Serial.printf("  UDP packets:  %lu (batches)\n", stats_udp_packets);
    Serial.printf("  UDP skip:     %lu (no client)\n", stats_udp_skip);
    Serial.printf("  Errors:       %lu\n", stats_errors);
    if (ap_active) {
        Serial.printf("  WiFi clients: %d\n", ap_clients_count);
    }
    Serial.printf("  Free heap:    %lu\n", ESP.getFreeHeap());
    Serial.printf("  Uptime:       %lu s\n", millis() / 1000);
    Serial.println("-------------\n");
}

// =============================================================================
// END OF FILE
// =============================================================================
