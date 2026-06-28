// ================== INCLUDES ==================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LittleFS.h>
#include <WebSocketsClient.h>
#include <WiFi.h>

// ================== DEBUG ==================
#define DEBUG 1
#if DEBUG
#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGN(...) Serial.println(__VA_ARGS__)
#define LOGI(...) Serial.println(String("[INFO ] ") + String(__VA_ARGS__))
#define LOGW(...) Serial.println(String("[WARN ] ") + String(__VA_ARGS__))
#define LOGE(...) Serial.println(String("[ERROR] ") + String(__VA_ARGS__))
#define LOGD(...) Serial.println(String("[DEBUG] ") + String(__VA_ARGS__))
#define LOGF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define LOG(...)
#define LOGN()
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#define LOGD(...)
#define LOGF(fmt, ...)
#endif

// ================== BLE ==================
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ================== PIN CONFIG ==================
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18
#define PIN_CS_W5500 2
#define PIN_W5500_RST 4
#define BTN_PIN 32
#define RXD2 16
#define TXD2 17
#define LED_RUN 33
#define LED_ERR 25

// ================== STM32 FRAME ==================
#define SOF_BYTE 0x7E
#define MAX_PAYLOAD 240
#define MAX_FRAME 256
#define STM_EPOCH_OFFSET 946684800UL

// ===== UART DATA =====
#define MAX_RULES 10
#define MAX_RULE_LEN 16
#define TYPE_DATA 0x01
#define TYPE_ACK 0x02
#define TYPE_RULES 0x03
#define TYPE_RTC 0x04
#define TAG_RULE 0x10
#define TAG_RTC_TS 0x20

// ===== ERR TLV TAG =====
#define TAG_ERR_MID 0x30
#define TAG_ERR_ST 0x32
#define TAG_ERR_EC 0x31

// ===== MODULE ID =====
#define MOD_RTC 1
#define MOD_DHT 2
#define MOD_ADS 3
#define MOD_SD 4

// ===== STATUS =====
#define ST_OK 0
#define ST_ERR 1

// ===== ERROR CODE =====
#define ERR_TIMEOUT 1
#define ERR_NOT_FOUND 2
#define ERR_CRC 3

// ================== FILE PATH ==================
#define FILE_CONFIG "/config.json"

// ================== HELPERS ==================
#define ACK_TIMEOUT_MS 3000
#define BTN_DEBOUNCE_MS 40
#define BTN_LONG_MS 3000
#define ETH_DHCP_RETRY_MS 1000
#define ETH_LINK_TIMEOUT_MS 10000
#define HEARTBEAT_MS 15000
#define SENDWAIT_INTERVAL_MS 5000
#define NET_CHECK_INTERVAL 10000
#define SWITCH_COOLDOWN 10000

// ================== LED STATUS (Manual Book ref. hal. 13-14) ==================
// LED RUN  : blink lambat = init, ON tetap = normal, OFF = fatal/crash
// LED ERROR: OFF = no error, 1 kedip = sensor, 2 kedip = comm, 3 kedip = network,
//            ON tetap = fatal/system error
#define LED_RUN_BLINK_INIT_MS     500U   // periode blink lambat saat init
#define LED_ERR_BLINK_ON_MS       150U   // durasi tiap kedip ON
#define LED_ERR_BLINK_OFF_MS      150U   // jeda antar kedip dalam 1 pattern
#define LED_ERR_PATTERN_GAP_MS    1200U  // jeda sebelum pattern diulang
#define STM_COMM_TIMEOUT_MS       8000U  // batas waktu tanpa frame valid dari STM32

// ================== GLOBAL ==================
BLECharacteristic* bleTxChar = nullptr;
HardwareSerial SerialSTM(2);
PubSubClient mqttClient;
WebSocketsClient ws;
bool btnLastRead = HIGH;
bool btnPressed = false;
bool btnStableState = HIGH;
bool bleClientConnected = false;
unsigned long btnLastChange = 0;
unsigned long btnPressStart = 0;
unsigned long lastNetCheck = 0;
bool longActionDone = false;
bool wsConnected = false;
volatile bool needRestart = false;
volatile bool wsAckOk = false;
volatile uint32_t wsAckSeq = 0;
bool errRTC = false;
bool errDHT = false;
bool errADS = false;
bool errSD = false;
bool anyErrorActive = false;

// ================== SYSTEM / LED STATUS ==================
// errComm   : communication error -> gabungan UART timeout (STM32 diam)
//             DAN comm-error TLV dari STM32 (jika modul melaporkannya)
// errNet    : network error -> turunan dari netState / kondisi hybrid all-down
// fatalErr  : system/fatal error -> ON tetap pada LED RUN (OFF) & LED ERROR (ON)
bool sysReady       = false;   // true setelah boot/init selesai
bool errCommTimeout = false;   // STM32 tidak kirim frame valid > STM_COMM_TIMEOUT_MS
bool errCommTLV     = false;   // comm-error dilaporkan via TLV oleh STM32 (jika ada)
bool errComm        = false;   // errCommTimeout || errCommTLV
bool errNet         = false;   // diturunkan dari netState (lihat updateLedErrorFlags)
bool fatalErr       = false;   // system/fatal error (LED RUN OFF, LED ERROR ON tetap)
unsigned long lastStmFrameMs = 0;  // timestamp frame valid terakhir dari STM32

// ================== NETWORK ==================
byte mac[6];
enum NetState {
  NET_DOWN,
  NET_UP
};
NetState netState = NET_DOWN;
enum NetIf {
  IF_NONE,
  IF_WIFI,
  IF_ETH
};

NetIf activeIf = IF_NONE;
NetIf priorityIf = IF_WIFI;

// ================== TIMERS ==================
static unsigned long lastSwitch = 0;

// ================== BUFFERS ==================
uint8_t frameBuf[MAX_FRAME];
uint16_t frameIdx = 0;
int16_t expectedLength = -1;

// ================== DEFAULT CONFIG =============
const char* defaultConfig = R"rawliteral(
{
}
)rawliteral";

// ================== CONFIG STRUCT ==================
struct DeviceConfig {
  uint16_t boxID;
};

struct NetworkConfig {
  String mode;
  String priority;
};

struct TransportConfig {
  String type;
  String host;
  uint16_t port;
  String path;
  bool ssl;
};

struct WifiConfig {
  String ssid;
  String password;
};

struct RulesConfig {
  uint8_t count;
  char items[MAX_RULES][MAX_RULE_LEN];
};

struct AppConfig {
  DeviceConfig device;
  NetworkConfig network;
  TransportConfig transport;
  WifiConfig wifi;
  RulesConfig rules;
};

AppConfig config;

// ================== UTILITIES ==================
const char* moduleToStr(uint8_t mid) {
  switch (mid) {
    case MOD_RTC: return "RTC";
    case MOD_DHT: return "DHT";
    case MOD_ADS: return "ADS";
    case MOD_SD: return "SDCARD";
    default: return "UNKNOWN";
  }
}

// ================== LED STATUS LOGIC ==================
// Referensi: IoT Node Manual Book, bab "Indikator LED" (hal. 13-14).
//
// LED RUN
//   - Blink lambat  -> sistem masih inisialisasi (sysReady == false)
//   - ON tetap      -> sistem berjalan normal
//   - OFF           -> firmware crash / fatal error (fatalErr == true)
//
// LED ERROR (semua non-blocking, berbasis millis(), tidak pernah delay())
//   - OFF           -> tidak ada error
//   - 1 kedip       -> sensor / peripheral error (RTC/DHT/ADS/SD)
//   - 2 kedip       -> communication error (UART STM32 timeout atau TLV comm error)
//   - 3 kedip       -> network error (netState == NET_DOWN setelah sempat UP)
//   - ON tetap      -> system / fatal error (fatalErr == true), prioritas tertinggi
//
// Jika beberapa kondisi error aktif bersamaan, hanya pattern dengan prioritas
// TERTINGGI yang ditampilkan (fatal > network > comm > sensor), supaya
// operator selalu melihat masalah paling kritikal dahulu, sesuai contoh
// "Diagnostik Cepat" pada manual book.

// Kedip target untuk error non-fatal yang sedang aktif (0 = tidak ada)
static uint8_t ledErrBlinksTarget = 0;

void updateLedErrorFlags() {
  // errComm: gabungan UART timeout (STM32 diam) DAN comm-error TLV dari STM32
  if (!errCommTimeout && (millis() - lastStmFrameMs > STM_COMM_TIMEOUT_MS) && lastStmFrameMs != 0) {
    errCommTimeout = true;
    LOGW("[LED] STM32 comm timeout terdeteksi");
  }
  errComm = errCommTimeout || errCommTLV;

  // errNet: diturunkan langsung dari netState, hanya relevan setelah sistem
  // pernah online sekali (hindari LED ERROR nyala selama proses init normal)
  errNet = sysReady && (netState == NET_DOWN);

  // Tentukan pattern kedip aktif berdasarkan prioritas (fatal ditangani
  // terpisah di updateStatusLeds karena bentuknya ON tetap, bukan kedip)
  if (errNet) {
    ledErrBlinksTarget = 3;
  } else if (errComm) {
    ledErrBlinksTarget = 2;
  } else if (anyErrorActive) {
    ledErrBlinksTarget = 1;
  } else {
    ledErrBlinksTarget = 0;
  }
}

void updateStatusLeds() {
  unsigned long now = millis();

  // ---------- LED RUN ----------
  if (fatalErr) {
    digitalWrite(LED_RUN, LOW);   // OFF -> firmware crash / fatal error
  } else if (!sysReady) {
    // Blink lambat selama inisialisasi
    static unsigned long lastToggle = 0;
    static bool ledState = false;
    if (now - lastToggle >= LED_RUN_BLINK_INIT_MS) {
      lastToggle = now;
      ledState = !ledState;
      digitalWrite(LED_RUN, ledState ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_RUN, HIGH);  // ON tetap -> sistem berjalan normal
  }

  // ---------- LED ERROR ----------
  if (fatalErr) {
    digitalWrite(LED_ERR, HIGH);  // ON tetap -> system / fatal error
    return;
  }

  if (ledErrBlinksTarget == 0) {
    digitalWrite(LED_ERR, LOW);   // OFF -> tidak ada error
    return;
  }

  // State machine kedip non-blocking: ON -> OFF -> ON -> OFF ... lalu
  // diam selama LED_ERR_PATTERN_GAP_MS sebelum pattern diulang dari awal.
  static unsigned long lastStepMs = 0;
  static uint8_t blinkStep = 0;     // berapa kali sudah ON
  static bool errLedOn = false;
  static uint8_t lastTarget = 0;

  // Jika jumlah kedip target berubah (mis. dari 1 jadi 3), reset pattern
  // supaya tidak nyangkut di tengah hitungan lama.
  if (ledErrBlinksTarget != lastTarget) {
    lastTarget = ledErrBlinksTarget;
    blinkStep = 0;
    errLedOn = false;
    lastStepMs = now;
    digitalWrite(LED_ERR, LOW);
    return;
  }

  if (blinkStep >= ledErrBlinksTarget) {
    // Sudah selesai N kedip, diam di gap sebelum mengulang
    if (now - lastStepMs >= LED_ERR_PATTERN_GAP_MS) {
      blinkStep = 0;
      lastStepMs = now;
    }
    digitalWrite(LED_ERR, LOW);
    return;
  }

  if (!errLedOn) {
    if (now - lastStepMs >= LED_ERR_BLINK_OFF_MS) {
      errLedOn = true;
      lastStepMs = now;
      digitalWrite(LED_ERR, HIGH);
    }
  } else {
    if (now - lastStepMs >= LED_ERR_BLINK_ON_MS) {
      errLedOn = false;
      blinkStep++;
      lastStepMs = now;
      digitalWrite(LED_ERR, LOW);
    }
  }
}

static inline uint16_t rd16(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | p[1];
}

static inline uint32_t rd32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

bool saveConfigFromBLE(const String& json) {
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, json)) {
    LOGE("[CFG] invalid JSON from BLE");
    return false;
  }

  File f = LittleFS.open(FILE_CONFIG, "w");
  if (!f) {
    LOGE("[CFG] failed to open config.json");
    return false;
  }

  f.print(json);
  f.close();

  LOGI("[CFG] config.json updated via BLE");

  needRestart = true;
  return true;
}

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {

    String cmd = pChar->getValue();

    if (cmd.length() == 0) return;

    Serial.print("[BLE RX] ");
    Serial.println(cmd);

    if (cmd == "CMD:GET_CONFIG") {
      if (!LittleFS.exists("/config.json")) {
        Serial.println("[BLE] config.json not found");
        return;
      }

      File f = LittleFS.open("/config.json", "r");
      if (!f) {
        Serial.println("[BLE] failed to open config.json");
        return;
      }

      String json = f.readString();
      f.close();

      String resp = "CFG:" + json;

      if (bleTxChar) {
        bleTxChar->setValue(resp.c_str());
        bleTxChar->notify();

        LOGI("[BLE] CFG sent to client");
      } else {
        LOGE("[BLE] TX characteristic not ready");
      }
    } else if (cmd.startsWith("CMD:CONFIG:")) {
      String json = cmd.substring(strlen("CMD:CONFIG:"));
      if (saveConfigFromBLE(json)) {
        LOGI("[BLE] config saved & applied");
      } else {
        LOGE("[BLE] config save failed");
      }
    } else if (cmd.startsWith("CMD:RTC:")) {
      String datetime = cmd.substring(strlen("CMD:RTC:"));
      uint32_t epoch = parseDateTimeToEpoch(datetime);
      if (epoch > 0) {
        sendRTCToSTM32(epoch);
        LOGI("[BLE] RTC sent to STM");
      }
    } else {
      Serial.println("[BLE] Unknown command");
    }
  }
};

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    LOGI("[BLE] client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    LOGI("[BLE] client disconnected");

    BLEDevice::getAdvertising()->start();
    LOGI("[BLE] advertising restarted");
  }
};

bool isMode(const char* m) {
  return strcasecmp(config.network.mode.c_str(), m) == 0;
}

Client* getNetClient() {
  if (isMode("wifi")) {
    static WiFiClient wifiClient;
    return &wifiClient;
  }

  if (isMode("ethernet")) {
    static EthernetClient ethClient;
    return &ethClient;
  }

  return nullptr;
}

// =====================================================
// =================== FILE HELPERS ====================
// =====================================================
String readFile(const char* path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return "";
  String s = f.readString();
  f.close();
  return s;
}

void writeFile(const char* path, const String& data) {
  File f = LittleFS.open(path, "w");
  if (!f) return;
  f.print(data);
  f.close();
}

bool loadConfig() {
  if (!LittleFS.exists(FILE_CONFIG)) {
    LOGW("[CFG] config.json not found");
    return false;
  }

  File f = LittleFS.open(FILE_CONFIG, "r");
  if (!f) {
    LOGE("[CFG] open config failed");
    return false;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    LOGE("[CFG] JSON parse error");
    return false;
  }

  // ---------------- DEVICE ----------------
  config.device.boxID =
    doc["device"]["boxID"] | 0;

  // ---------------- NETWORK ----------------
  config.network.mode =
    doc["network"]["mode"] | "wifi";
  config.network.priority =
    doc["network"]["priority"] | "wifi";

  // ---------------- TRANSPORT ----------------
  config.transport.type =
    doc["transport"]["type"] | "tcp";
  config.transport.host =
    doc["transport"]["host"] | "";
  config.transport.port =
    doc["transport"]["port"] | 8080;
  config.transport.path =
    doc["transport"]["path"] | "/";
  config.transport.ssl =
    doc["transport"]["ssl"] | false;


  // ---------------- WIFI ----------------
  config.wifi.ssid =
    doc["wifi"]["ssid"] | "";
  config.wifi.password =
    doc["wifi"]["password"] | "";

  // ---------------- RULES ----------------
  config.rules.count = 0;

  JsonArray rulesArr = doc["rules"].as<JsonArray>();
  if (!rulesArr.isNull()) {
    for (JsonVariant v : rulesArr) {
      if (config.rules.count >= MAX_RULES) break;
      const char* r = v.as<const char*>();
      if (!r) continue;

      strncpy(
        config.rules.items[config.rules.count],
        r,
        MAX_RULE_LEN - 1);
      config.rules.items[config.rules.count][MAX_RULE_LEN - 1] = '\0';
      config.rules.count++;
    }
  }

  LOGF("[CFG] rules loaded: %d\n", config.rules.count);
  for (uint8_t i = 0; i < config.rules.count; i++) {
    LOGF("  rule[%d]=%s\n", i, config.rules.items[i]);
  }

  // ===== VALIDASI KONSISTENSI transport vs network mode =====
  // WebSocket cuma bisa jalan di mode wifi (lihat startWebSocket()
  // yang sudah ada: if (!isMode("wifi")) return false). Kalau config
  // tersimpan invalid (misal lewat jalur selain BLE UI), jangan
  // gagal diam-diam - paksa ke wifi dan kasih tahu lewat log.
  if (strcasecmp(config.transport.type.c_str(), "ws") == 0 &&
      !isMode("wifi")) {
    LOGE("[CFG] INVALID: transport=ws butuh network mode=wifi, dipaksa ke wifi");
    config.network.mode = "wifi";
  }

  initPriority();
  return true;
}

// =====================================================
// =================== NET HELPERS =====================
// =====================================================
bool startEthernet(uint32_t timeoutMs = ETH_LINK_TIMEOUT_MS) {
  Ethernet.init(PIN_CS_W5500);

  // ---------- CEK LINK ----------
  LOGI("[NET] checking Ethernet link...");
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    auto link = Ethernet.linkStatus();
    if (link == LinkON) {
      LOGI("[NET] Ethernet link UP");
      break;
    }
    if (link == LinkOFF) {
      LOGI("[NET] Ethernet link DOWN, waiting...");
    }
    updateStatusLeds();   // tetap blink LED RUN selama boot menunggu link
    delay(500);
  }
  if (Ethernet.linkStatus() != LinkON) {
    LOGI("[NET] Ethernet cable not detected");
    return false;
  }

  // ---------- DHCP ----------
  LOGI("[NET] requesting DHCP...");
  getEspBaseMac(mac);
  LOGF("[NET] MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  start = millis();
  while (millis() - start < timeoutMs) {
    if (Ethernet.begin(mac) != 0) {
      LOG("[NET] Ethernet IP: ");
      Serial.println(Ethernet.localIP());
      return true;
    }
    updateStatusLeds();   // tetap blink LED RUN selama boot menunggu DHCP
    delay(ETH_DHCP_RETRY_MS);
  }
  LOGI("[NET] DHCP timeout");
  return false;
}

bool tryEthernet() {
  LOGI("[NET] try Ethernet...");
  if (startEthernet()) {
    netState = NET_UP;
    activeIf = IF_ETH;
    sysReady = true;   // boot selesai -> LED RUN lanjut ON tetap
    LOGI("[NET] Ethernet READY");
    return true;
  }
  LOGI("[NET] Ethernet FAIL");
  return false;
}

bool startWiFiSTA(uint32_t timeoutMs = HEARTBEAT_MS) {
  if (config.wifi.ssid.length() == 0) {
    LOGW("[WIFI] SSID empty");
    return false;
  }
  LOGF("[WIFI] connecting to %s...\n", config.wifi.ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifi.ssid.c_str(),
             config.wifi.password.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    updateStatusLeds();   // tetap blink LED RUN selama boot menunggu WiFi
    delay(500);
    LOG(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    LOG("[WIFI] connected, IP: ");
    LOGI(WiFi.localIP());
    return true;
  }
  LOGI("[WIFI] connect timeout");
  WiFi.disconnect(true);
  return false;
}

bool tryWiFi() {
  LOGI("[NET] try WiFi...");
  if (startWiFiSTA()) {
    netState = NET_UP;
    LOGI("[NET] WiFi READY");
    activeIf = IF_WIFI;
    sysReady = true;   // boot selesai -> LED RUN lanjut ON tetap
    if (strcasecmp(config.transport.type.c_str(), "ws") == 0) {
      startWebSocket();
    }
    return true;
  }
  LOGI("[NET] WiFi FAIL");
  return false;
}

bool startWebSocket() {
  if (!isMode("wifi")) {
    LOGW("[WS] skipped (not wifi mode)");
    return false;
  }
  LOGI("[WS] starting...");
  ws.begin(
    config.transport.host.c_str(),
    config.transport.port,
    config.transport.path.c_str());
  ws.onEvent(wsEvent);
  ws.setReconnectInterval(SENDWAIT_INTERVAL_MS);  // auto reconnect
  ws.enableHeartbeat(HEARTBEAT_MS, ACK_TIMEOUT_MS, 2);
  return true;
}

bool wifiAlive() {
  return WiFi.status() == WL_CONNECTED;
}

bool ethAlive() {
  return Ethernet.linkStatus() == LinkON && Ethernet.localIP() != INADDR_NONE;
}

void initPriority() {
  if (strcasecmp(config.network.priority.c_str(), "wifi") == 0)
    priorityIf = IF_WIFI;
  else
    priorityIf = IF_ETH;
}

void switchTo(NetIf target) {
  if (!canSwitchNow()) {
    LOGW("[NET] switch blocked (cooldown)");
    return;
  }

  LOGI("[NET] switching to " + String(target == IF_WIFI ? "WiFi" : "Ethernet"));

  if (activeIf == IF_WIFI) {
    WiFi.disconnect(true);
  }

  delay(200);

  bool ok = false;
  if (target == IF_WIFI) {
    ok = startWiFiSTA();
  } else if (target == IF_ETH) {
    ok = startEthernet();
  }

  if (ok) {
    activeIf = target;
    netState = NET_UP;
    markSwitched();
    LOGI("[NET] switched OK");
  } else {
    LOGW("[NET] switch failed");
  }
}


bool canSwitchNow() {
  return millis() - lastSwitch >= SWITCH_COOLDOWN;
}

void markSwitched() {
  lastSwitch = millis();
}

// =====================================================
// ================ TRANSPORT HELPERS ==================
// =====================================================
bool httpPostJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (netState == NET_DOWN) {
    LOGW("[HTTP] network down");
    return false;
  }
  Client* client = getNetClient();
  if (!client) {
    LOGW("[HTTP] no net client");
    return false;
  }
  LOG("[HTTP] connect ");
  LOG(config.transport.host);
  LOG(":");
  LOGI(config.transport.port);
  if (!client->connect(
        config.transport.host.c_str(),
        config.transport.port)) {
    LOGW("[HTTP] connect FAIL");
    return false;
  }
  int bodyLen = strlen(jsonLine);
  if (bodyLen <= 0) {
    LOGW("[HTTP] empty JSON");
    client->stop();
    return false;
  }

  // ---------- SEND HTTP ----------
  client->print("POST ");
  client->print(config.transport.path);
  client->print(" HTTP/1.1\r\n");
  client->print("Host: ");
  client->print(config.transport.host);
  client->print("\r\n");
  client->println("Content-Type: application/json");
  client->println("Connection: close");
  client->print("Content-Length: ");
  client->println(bodyLen);
  client->println();
  client->write((const uint8_t*)jsonLine, bodyLen);

  // ---------- READ RESPONSE ----------
  unsigned long start = millis();
  String resp;
  while (millis() - start < SENDWAIT_INTERVAL_MS) {
    while (client->available()) {
      resp += (char)client->read();
    }
    if (!client->connected()) break;
  }
  client->stop();
  int jsonStart = resp.indexOf("\r\n\r\n");
  if (jsonStart < 0) {
    LOGW("[HTTP] no body");
    return false;
  }
  String body = resp.substring(jsonStart + 4);
  body.trim();
  return parseAck(body.c_str(), seq);
}

bool tcpSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (netState == NET_DOWN) {
    LOGW("[TCP] network not ready");
    return false;
  }

  Client* client = getNetClient();
  if (!client) {
    LOGW("[TCP] no client");
    return false;
  }

  LOG("[TCP] connect ");
  LOG(config.transport.host);
  LOG(":");
  LOGI(config.transport.port);

  if (!client->connect(
        config.transport.host.c_str(),
        config.transport.port)) {
    LOGW("[TCP] connect FAIL");
    return false;
  }

  uint16_t len = strlen(jsonLine);

  // ===== SEND FRAME =====
  client->print(len);
  client->print('\n');
  client->write((const uint8_t*)jsonLine, len);

  LOG("[TCP] sent len=");
  LOGI(len);

  unsigned long start = millis();
  String resp;

  while (millis() - start < ACK_TIMEOUT_MS) {
    while (client->available()) {
      char c = client->read();
      if (c == '\n') goto PARSE;
      resp += c;
    }
    if (!client->connected()) break;
  }

PARSE:
  client->stop();
  resp.trim();

  if (resp.length() == 0) {
    LOGW("[TCP] no response");
    return false;
  }

  return parseAck(resp.c_str(), seq);
}

bool wsSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (netState != NET_UP) {
    LOGW("[WS] network down");
    return false;
  }
  if (!wsConnected) {
    LOGW("[WS] not connected");
    return false;
  }
  wsAckOk = false;
  wsAckSeq = 0;
  LOG("[WS] TX: ");
  LOGI(jsonLine);

  ws.sendTXT(jsonLine);

  unsigned long start = millis();
  while (millis() - start < ACK_TIMEOUT_MS) {
    ws.loop();
    if (wsAckOk && wsAckSeq == seq) {
      LOGI("[WS] ACK OK");
      return true;
    }
    delay(1);
  }
  LOGW("[WS] ACK TIMEOUT");
  return false;
}

bool mqttConnect() {
  if (mqttClient.connected()) return true;

  Client* netClient = getNetClient();
  if (!netClient) {
    LOGW("[MQTT] no net client");
    return false;
  }

  mqttClient.setClient(*netClient);
  mqttClient.setServer(
    config.transport.host.c_str(),
    config.transport.port);
  mqttClient.setCallback(mqttCallback);

  String clientId = "iot-node-" + String(config.device.boxID);

  LOGI("[MQTT] connecting...");
  if (!mqttClient.connect(clientId.c_str())) {
    LOGW("[MQTT] connect FAIL");
    return false;
  }

  // subscribe ACK topic
  String ackTopic = "iot/ack/" + String(config.device.boxID);
  mqttClient.subscribe(ackTopic.c_str());

  LOGI("[MQTT] connected");
  return true;
}

bool mqttSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (netState == NET_DOWN) {
    LOGW("[MQTT] network down");
    return false;
  }
  if (!mqttConnect()) {
    return false;
  }
  wsAckOk = false;
  wsAckSeq = 0;
  String pubTopic = config.transport.path;
  LOGI("[MQTT] TX:");
  LOGI(jsonLine);
  if (!mqttClient.publish(pubTopic.c_str(), jsonLine)) {
    LOGW("[MQTT] publish FAIL");
    return false;
  }

  unsigned long start = millis();
  while (millis() - start < ACK_TIMEOUT_MS) {
    mqttClient.loop();

    if (wsAckOk && wsAckSeq == seq) {
      LOGI("[MQTT] ACK OK");
      return true;
    }
    delay(1);  // beri jatah CPU ke task lain (WiFi/BLE stack di FreeRTOS)
  }
  LOGI("[MQTT] ACK TIMEOUT");
  return false;
}

bool parseAck(const char* json, uint32_t expectSeq) {
  uint32_t ack;
  char status[8];
  if (sscanf(json,
             "{\"ack_seq\":%lu,\"status\":\"%7[^\"]\"}",
             &ack, status)
      == 2) {

    return ack == expectSeq && strcmp(status, "ok") == 0;
  }
  return false;
}

bool sendByTransport(uint32_t seq, const char* jsonLine) {
  if (netState == NET_DOWN) return false;
  if (strcasecmp(config.transport.type.c_str(), "tcp") == 0) {
    return tcpSendJSONWithSeq(seq, jsonLine);
  }
  if (strcasecmp(config.transport.type.c_str(), "httppost") == 0) {
    return httpPostJSONWithSeq(seq, jsonLine);
  }
  if (strcasecmp(config.transport.type.c_str(), "mqtt") == 0) {
    return mqttSendJSONWithSeq(seq, jsonLine);
  }
  if (strcasecmp(config.transport.type.c_str(), "ws") == 0) {
    if (!wsConnected) {
      LOGW("[WS] skip send (not connected yet)");
      return false;
    }
    return wsSendJSONWithSeq(seq, jsonLine);
  }
  LOG("[CFG] unknown transport: ");
  LOGI(config.transport.type);
  return false;
}

// =====================================================
// ================== STM32 HELPERS ====================
// =====================================================
uint16_t CRC16_Modbus(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

void processFrame(uint8_t* buf, uint16_t len) {
  if (buf[0] != SOF_BYTE) {
    LOGW("[UART] Bad SOF");
    return;
  }

  // ===== CRC CHECK =====
  uint16_t crcRecv = ((uint16_t)buf[len - 2] << 8) | buf[len - 1];
  uint16_t crcCalc = CRC16_Modbus(buf, len - 2);
  if (crcRecv != crcCalc) {
    LOGE("[UART] CRC mismatch");
    return;
  }

  // Frame valid (SOF + CRC OK) -> link UART ke STM32 dianggap hidup,
  // dipakai untuk deteksi communication error (LED ERROR 2 kedip).
  lastStmFrameMs = millis();
  if (errCommTimeout) {
    LOGI("[LED] comm timeout cleared, STM32 frame received");
  }
  errCommTimeout = false;
  errComm = errCommTimeout || errCommTLV;

  uint8_t type = buf[1];
  uint16_t payloadLen = ((uint16_t)buf[2] << 8) | buf[3];
  uint8_t* payload = &buf[4];
  if (type == 0x01) {  // DATA
    handleDataFromSTM(payload, payloadLen);
  } else if (type == 0x02) {  // ACK
    LOGI("[UART] ACK from STM");
  } else if (type == 0x05) {  // ERR
    handleErrorFromSTM(payload, payloadLen);
  }
}

void handleDataFromSTM(uint8_t* tlv, size_t len) {
  uint32_t seq = 0;
  uint32_t ts = 0;
  int16_t temp = 0, hum = 0;
  uint16_t digMask = 0;
  uint16_t an[4] = { 0 };
  uint8_t qMask = 0;

  size_t i = 0;

  while (i + 2 <= len) {
    uint8_t tag = tlv[i++];
    uint8_t l = tlv[i++];

    if (i + l > len) {
      LOGE("[TLV] overflow");
      return;
    }
    switch (tag) {
      case 0x01:  // SEQ
        if (l == 4) seq = rd32(&tlv[i]);
        break;
      case 0x07:  // TIMESTAMP
        if (l == 4) ts = rd32(&tlv[i]);
        break;
      case 0x02:  // TEMP
        if (l == 2) temp = (int16_t)rd16(&tlv[i]);
        break;
      case 0x03:  // HUM
        if (l == 2) hum = (int16_t)rd16(&tlv[i]);
        break;
      case 0x04:  // DIG_IN
        if (l == 2) digMask = rd16(&tlv[i]);
        break;
      case 0x05:  // AN_IN (4 channel)
        if (l == 8) {
          for (int k = 0; k < 4; k++) {
            an[k] = rd16(&tlv[i + k * 2]);
          }
        }
        break;
      case 0x06:  // Q
        if (l == 1) qMask = tlv[i];
        break;
      default:
        LOGW("[TLV] unknown tag");
        break;
    }
    i += l;
  }

  // ===== BUILD JSON =====
  char tsISO[32];
  char digArr[64];
  char qArr[32];

  uint32_t unixEpoch = ts + STM_EPOCH_OFFSET;
  epochToISO8601(unixEpoch, tsISO, sizeof(tsISO));
  buildDigArray(digMask, digArr, sizeof(digArr));
  buildQArray(qMask, qArr, sizeof(qArr));

  char json[384];
  snprintf(json, sizeof(json),
           "{"
           "\"timestamp\":\"%s\","
           "\"env\":{\"temp\":%.1f,\"hum\":%.1f},"
           "\"dig_in\":%s,"
           "\"an_in\":[%u,%u,%u,%u],"
           "\"q\":%s,"
           "\"seq\":%lu"
           "}",
           tsISO,
           temp / 10.0f,
           hum / 10.0f,
           digArr,
           an[0], an[1], an[2], an[3],
           qArr,
           seq);

  LOGI("[UART] DATA OK → SEND");
  LOGI(json);

  if (sendByTransport(seq, json)) {
    sendAckToSTM(seq);
  }
}

void handleErrorFromSTM(uint8_t* tlv, size_t len) {
  uint8_t moduleId = 0xFF;
  uint8_t status = 0xFF;
  uint8_t errCode = 0xFF;

  size_t i = 0;
  while (i + 2 <= len) {
    uint8_t tag = tlv[i++];
    uint8_t l = tlv[i++];

    if (i + l > len) {
      LOGE("[ERR] TLV overflow");
      return;
    }

    switch (tag) {
      case TAG_ERR_MID:
        if (l == 1) moduleId = tlv[i];
        break;
      case TAG_ERR_ST:
        if (l == 1) status = tlv[i];
        break;
      case TAG_ERR_EC:
        if (l == 1) errCode = tlv[i];
        break;
    }
    i += l;
  }

  const char* modStr = moduleToStr(moduleId);

  if (moduleId == MOD_RTC) errRTC = (status == ST_ERR);
  if (moduleId == MOD_DHT) errDHT = (status == ST_ERR);
  if (moduleId == MOD_ADS) errADS = (status == ST_ERR);
  if (moduleId == MOD_SD) errSD = (status == ST_ERR);

  // Modul selain RTC/DHT/ADS/SD (mis. modul comm RS485/UART STM32 di masa
  // depan) dipetakan sebagai communication error pada LED ERROR (2 kedip),
  // bukan sensor error. Saat ini STM32 belum mengirim moduleId semacam ini,
  // jalur ini hanya menjaga agar siap dipakai tanpa perlu ubah LED logic lagi.
  if (moduleId != MOD_RTC && moduleId != MOD_DHT &&
      moduleId != MOD_ADS && moduleId != MOD_SD) {
    errCommTLV = (status == ST_ERR);
  }

  anyErrorActive = errRTC || errDHT || errADS || errSD;
  errComm = errCommTimeout || errCommTLV;
  
  if (status == ST_ERR) {
    LOGF("[ERR] %s ERROR (code=%u)\n", modStr, errCode);
  } else if (status == ST_OK) {
    if (!anyErrorActive) {
      LOGF("[OK] %s OK → ALL MODULES OK\n", modStr);
    } else {
      LOGF("[OK] %s OK (others still error)\n", modStr);
    }
  }
}


void sendAckToSTM(uint32_t seq) {
  uint8_t frame[16];
  uint16_t idx = 0;

  frame[idx++] = SOF_BYTE;
  frame[idx++] = 0x02;  // ACK
  frame[idx++] = 0x00;
  frame[idx++] = 0x04;

  memcpy(&frame[idx], &seq, 4);
  idx += 4;

  uint16_t crc = CRC16_Modbus(frame, idx);
  frame[idx++] = crc >> 8;
  frame[idx++] = crc & 0xFF;

  SerialSTM.write(frame, idx);
  LOGI("[UART] ACK sent seq=" + String(seq));
}

bool buildRulesTLV(uint8_t* out, size_t maxLen, size_t* outLen) {
  size_t idx = 0;

  for (uint8_t i = 0; i < config.rules.count; i++) {
    const char* r = config.rules.items[i];
    uint8_t l = strlen(r);

    if (idx + 2 + l > maxLen) {
      LOGW("[RULE] TLV buffer full");
      return false;
    }

    out[idx++] = TAG_RULE;
    out[idx++] = l;
    memcpy(&out[idx], r, l);
    idx += l;
  }

  *outLen = idx;
  return true;
}

void sendRulesToSTM32() {
  uint8_t tlv[256];
  size_t tlvLen = 0;

  if (!buildRulesTLV(tlv, sizeof(tlv), &tlvLen)) {
    LOGW("[RULE] build TLV failed");
    return;
  }

  uint8_t frame[300];
  uint16_t idx = 0;

  frame[idx++] = SOF_BYTE;
  frame[idx++] = TYPE_RULES;
  frame[idx++] = (tlvLen >> 8) & 0xFF;
  frame[idx++] = tlvLen & 0xFF;

  memcpy(&frame[idx], tlv, tlvLen);
  idx += tlvLen;

  uint16_t crc = CRC16_Modbus(frame, idx);
  frame[idx++] = crc >> 8;
  frame[idx++] = crc & 0xFF;

  SerialSTM.write(frame, idx);
  LOGI("[RULE] TLV rules sent to STM");
}

uint32_t parseDateTimeToEpoch(const String& dt) {
  // format input: "YYYY-MM-DD HH:MM:SS"
  struct tm t {};
  if (sscanf(dt.c_str(),
             "%d-%d-%d %d:%d:%d",
             &t.tm_year,
             &t.tm_mon,
             &t.tm_mday,
             &t.tm_hour,
             &t.tm_min,
             &t.tm_sec)
      != 6) {
    LOGE("[RTC] invalid datetime format");
    return 0;
  }

  t.tm_year -= 1900;
  t.tm_mon -= 1;

  time_t unixEpoch = mktime(&t);
  if (unixEpoch < 0) {
    LOGE("[RTC] mktime failed");
    return 0;
  }

  if (unixEpoch < (time_t)STM_EPOCH_OFFSET) {
    LOGE("[RTC] time before 2000-01-01 not supported");
    return 0;
  }

  uint32_t epoch2000 = (uint32_t)(unixEpoch - STM_EPOCH_OFFSET);
  LOGF("[RTC] PARSE JSON: {\"input\":\"%s\",\"unix\":%lu,\"epoch_2000\":%lu}\n",
       dt.c_str(),
       (unsigned long)unixEpoch,
       (unsigned long)epoch2000);
  return epoch2000;
}

void sendRTCToSTM32(uint32_t epoch2000) {
  uint8_t tlv[8];
  size_t idx = 0;

  tlv[idx++] = TAG_RTC_TS;
  tlv[idx++] = 4;
  tlv[idx++] = (epoch2000 >> 24) & 0xFF;
  tlv[idx++] = (epoch2000 >> 16) & 0xFF;
  tlv[idx++] = (epoch2000 >> 8) & 0xFF;
  tlv[idx++] = epoch2000 & 0xFF;

  uint8_t frame[32];
  uint16_t fidx = 0;

  frame[fidx++] = SOF_BYTE;
  frame[fidx++] = TYPE_RTC;
  frame[fidx++] = 0x00;
  frame[fidx++] = idx;

  memcpy(&frame[fidx], tlv, idx);
  fidx += idx;
  uint16_t crc = CRC16_Modbus(frame, fidx);
  frame[fidx++] = crc >> 8;
  frame[fidx++] = crc & 0xFF;

  SerialSTM.write(frame, fidx);
  char iso[32];
  uint32_t unixEpoch = epoch2000 + STM_EPOCH_OFFSET;
  epochToISO8601(unixEpoch, iso, sizeof(iso));
  LOGF("[RTC] TX JSON: {\"epoch_2000\":%lu,\"unix\":%lu,\"iso\":\"%s\"}\n",
       (unsigned long)epoch2000,
       (unsigned long)unixEpoch,
       iso);
}

void epochToISO8601(uint32_t epoch, char* out, size_t outLen) {
  uint32_t seconds = epoch;
  uint32_t days = seconds / 86400;
  seconds %= 86400;
  uint8_t hour = seconds / 3600;
  seconds %= 3600;
  uint8_t min = seconds / 60;
  uint8_t sec = seconds % 60;
  uint32_t year = 1970;
  const uint16_t daysInYear = 365;
  const uint16_t daysInLeap = 366;

  auto isLeap = [](uint32_t y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
  };
  while (true) {
    uint16_t dy = isLeap(year) ? daysInLeap : daysInYear;
    if (days < dy) break;
    days -= dy;
    year++;
  }
  static const uint8_t mdays[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };
  uint8_t month = 0;
  for (uint8_t i = 0; i < 12; i++) {
    uint8_t d = mdays[i];
    if (i == 1 && isLeap(year)) d = 29;
    if (days < d) break;
    days -= d;
    month++;
  }
  uint8_t day = days + 1;
  snprintf(out, outLen,
           "%04lu-%02u-%02uT%02u:%02u:%02uZ",
           year, month + 1, day,
           hour, min, sec);
}


void buildDigArray(uint16_t mask, char* out, size_t len) {
  size_t pos = 0;
  pos += snprintf(out + pos, len - pos, "[");
  for (int i = 0; i < 12; i++) {
    pos += snprintf(out + pos, len - pos,
                    "%d%s",
                    (mask >> i) & 1,
                    (i < 11) ? "," : "");
  }
  snprintf(out + pos, len - pos, "]");
}

void buildQArray(uint8_t mask, char* out, size_t len) {
  size_t pos = 0;
  pos += snprintf(out + pos, len - pos, "[");
  for (int i = 0; i < 4; i++) {
    pos += snprintf(out + pos, len - pos,
                    "%d%s",
                    (mask >> i) & 1,
                    (i < 3) ? "," : "");
  }
  snprintf(out + pos, len - pos, "]");
}

// =====================================================
// =============== INPUT/OUTPUT HELPERS ================
// =====================================================
void handleButton() {
  unsigned long now = millis();
  bool readNow = digitalRead(BTN_PIN);
  if (readNow != btnLastRead) {
    btnLastChange = now;
    btnLastRead = readNow;
  }
  if ((now - btnLastChange) < BTN_DEBOUNCE_MS) {
    return;
  }
  if (readNow != btnStableState) {
    btnStableState = readNow;
    if (btnStableState == LOW) {
      btnPressed = true;
      btnPressStart = now;
      longActionDone = false;
    } else {
      if (btnPressed) {
        unsigned long pressTime = now - btnPressStart;
        btnPressed = false;
        if (!longActionDone && pressTime < BTN_LONG_MS) {
          LOGI("BTN short press → RESTART");
          delay(100);
          ESP.restart();
        }
      }
    }
  }
  if (btnPressed && !longActionDone) {
    if (now - btnPressStart >= BTN_LONG_MS) {
      longActionDone = true;
      LOGI("BTN long press");
    }
  }
}

// =====================================================
// =================== EVENT HELPERS ===================
// =====================================================
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      LOGI("[WS] connected");
      break;
    case WStype_DISCONNECTED:
      wsConnected = false;
      LOGI("[WS] disconnected");
      break;
    case WStype_TEXT:
      {
        LOG("[WS] RX: ");
        Serial.write(payload, length);
        LOGN();
        uint32_t ackSeq = 0;
        char status[16] = { 0 };
        if (sscanf((char*)payload,
                   "{\"ack_seq\":%lu,\"status\":\"%15[^\"]\"}",
                   &ackSeq, status)
            == 2) {
          if (strcmp(status, "ok") == 0) {
            wsAckSeq = ackSeq;
            wsAckOk = true;
          }
        }
        break;
      }
    default:
      break;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  static char msg[256];
  if (length >= sizeof(msg)) return;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  LOGI("[MQTT] RX:");
  LOGI(msg);

  uint32_t ackSeq = 0;
  char status[8];
  if (sscanf(msg,
             "{\"ack_seq\":%lu,\"status\":\"%7[^\"]\"}",
             &ackSeq, status)
      == 2) {
    if (strcmp(status, "ok") == 0) {
      wsAckSeq = ackSeq;
      wsAckOk = true;
    }
  }
}

// =====================================================
// ======================== BLE ========================
// =====================================================
String buildBleName() {
  if (config.device.boxID > 0) {
    return "IoT-Node-" + String(config.device.boxID);
  }
  return "IoT-Node-UNCFG";
}

void getEspBaseMac(byte* mac) {
  uint64_t chipid = ESP.getEfuseMac();  // 48-bit
  mac[0] = (chipid >> 40) & 0xFF;
  mac[1] = (chipid >> 32) & 0xFF;
  mac[2] = (chipid >> 24) & 0xFF;
  mac[3] = (chipid >> 16) & 0xFF;
  mac[4] = (chipid >> 8) & 0xFF;
  mac[5] = (chipid)&0xFF;
}

// =====================================================
// ======================= SETUP =======================
// =====================================================
void initGPIO() {
  delay(200);
  LOGI("\n=== BOOT ===");

  SerialSTM.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_RUN, OUTPUT);
  pinMode(LED_ERR, OUTPUT);

  // Mulai dari OFF; blink lambat untuk fase inisialisasi diteruskan secara
  // non-blocking oleh updateStatusLeds() selama sysReady masih false.
  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_ERR, LOW);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(PIN_CS_W5500, OUTPUT);
  digitalWrite(PIN_CS_W5500, HIGH);
  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(200);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(500);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  delay(100);
}

void initFS() {
  if (!LittleFS.begin(true)) {
    LOGE("[FS] LittleFS mount FAILED");
  } else {
    LOGI("[FS] LittleFS mounted");
    LOGI("[CFG] loadConfig()");
    if (!loadConfig()) {
      LOGW("[CFG] load failed, use default");
    } else {
      delay(500);
      sendRulesToSTM32();
    }
  }
}

void initNetwork() {
  WiFi.mode(WIFI_OFF);
  delay(200);

  netState = NET_DOWN;

  // ================== ETHERNET ONLY ==================
  if (isMode("ethernet")) {
    LOGI("[CFG] network mode: ethernet");
    if (!tryEthernet()) {
      LOGI("[NET] Ethernet unavailable");
    }
  }

  // ================== WIFI ONLY ==================
  else if (isMode("wifi")) {
    LOGI("[CFG] network mode: wifi");
    if (!tryWiFi()) {
      LOGI("[NET] WiFi unavailable");
    }
  }

  // ================== HYBRID ==================
  else if (isMode("hybrid")) {
    LOGI("[CFG] network mode: hybrid");
    LOGI("[CFG] priority: " + config.network.priority);

    bool connected = false;

    if (strcasecmp(config.network.priority.c_str(), "ethernet") == 0) {
      connected = tryEthernet();
      if (!connected) connected = tryWiFi();
    } else if (strcasecmp(config.network.priority.c_str(), "wifi") == 0) {
      connected = tryWiFi();
      if (!connected) connected = tryEthernet();
    } else {
      LOGI("[CFG] invalid priority, default ethernet→wifi");
      connected = tryEthernet() || tryWiFi();
    }
    if (!connected) {
      LOGI("[NET] hybrid failed");
    }
  } else {
    LOGI("[CFG] unknown network mode");
  }
  LOGI("=== SETUP DONE ===");

  // Inisialisasi selesai (terlepas dari hasil koneksi jaringan). Jika
  // jaringan ternyata tidak berhasil, kondisi tersebut akan tertangkap
  // sebagai network error (LED ERROR 3 kedip) lewat errNet, bukan membuat
  // LED RUN blink inisialisasi selamanya.
  sysReady = true;
}

void initBLE() {
  String bleName = buildBleName();
  LOGI("[BLE] init name = " + bleName);

  BLEDevice::init(bleName.c_str());
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  BLEService* service = server->createService(BLE_SERVICE_UUID);

  BLECharacteristic* rxChar = service->createCharacteristic(
    BLE_CHAR_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);
  rxChar->setCallbacks(new BleRxCallbacks());

  bleTxChar = service->createCharacteristic(
    BLE_CHAR_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  bleTxChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();
  LOGI("[BLE] advertising started");
}

void setup() {
  Serial.begin(115200);
  initGPIO();
  initFS();
  initBLE();
  initNetwork();
}

// =====================================================
// ======================= LOOP ========================
// =====================================================
void handleHybridNetwork() {
  if (!isMode("hybrid")) return;

  unsigned long now = millis();
  if (now - lastNetCheck < NET_CHECK_INTERVAL) return;
  lastNetCheck = now;

  bool wifiOk = wifiAlive();
  bool ethOk = ethAlive();

  // ================= ACTIVE LINK PUTUS =================
  if (activeIf == IF_WIFI && !wifiOk && ethOk) {
    LOGW("[NET] WiFi down → switch to Ethernet");
    switchTo(IF_ETH);
    return;
  }

  if (activeIf == IF_ETH && !ethOk && wifiOk) {
    LOGW("[NET] Ethernet down → switch to WiFi");
    switchTo(IF_WIFI);
    return;
  }

  // ================= PRIORITY RECOVERY =================
  if (priorityIf == IF_WIFI && wifiOk && activeIf != IF_WIFI) {
    LOGI("[NET] WiFi recovered → switch back to priority");
    switchTo(IF_WIFI);
    return;
  }

  if (priorityIf == IF_ETH && ethOk && activeIf != IF_ETH) {
    LOGI("[NET] Ethernet recovered → switch back to priority");
    switchTo(IF_ETH);
    return;
  }

  // ================= ALL DOWN =================
  if (!wifiOk && !ethOk) {
    LOGE("[NET] all interfaces down → restart");
    fatalErr = true;
    updateStatusLeds();   // pastikan LED RUN OFF / LED ERROR ON tetap terlihat
    delay(5000);
    ESP.restart();
  }
}

void handleWebSocket() {
  if (netState == NET_UP && isMode("wifi") && strcasecmp(config.transport.type.c_str(), "ws") == 0) {
    ws.loop();
  }
}

void handleUART() {
  while (SerialSTM.available()) {
    uint8_t b = SerialSTM.read();
    if (frameIdx == 0) {
      if (b != SOF_BYTE) continue;
      frameBuf[frameIdx++] = b;
    } else {
      frameBuf[frameIdx++] = b;
      if (frameIdx == 4) {
        uint16_t payloadLen =
          ((uint16_t)frameBuf[2] << 8) | frameBuf[3];
        expectedLength = 1 + 1 + 2 + payloadLen + 2;
        if (expectedLength > MAX_FRAME) {
          frameIdx = 0;
          expectedLength = -1;
        }
      }
      if (expectedLength > 0 && frameIdx == expectedLength) {
        processFrame(frameBuf, expectedLength);
        frameIdx = 0;
        expectedLength = -1;
      }
    }
  }
}

void handleBLE() {
  if (needRestart) {
    LOGI("[SYS] restarting to apply new config");
    delay(1200);
    ESP.restart();
  }
}

void loop() {
  handleButton();
  handleHybridNetwork();
  handleWebSocket();
  handleUART();
  handleBLE();
  updateLedErrorFlags();
  updateStatusLeds();
  yield();
}