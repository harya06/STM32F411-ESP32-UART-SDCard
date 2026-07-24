// ================== INCLUDES ==================
#include <Arduino.h>
#include <esp_system.h>
#include <esp_timer.h>
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
#include <Wire.h>

enum NetIf {
  IF_NONE,
  IF_WIFI,
  IF_ETH
};

enum GsmState {
  GSM_NO_MODEM,          // belum ada modem terdeteksi di UART
  GSM_MODEM_FOUND,       // UART merespons, modem baru ditemukan
  GSM_POWER_ON,          // sequence power-on modem
  GSM_AT_READY,          // modem merespons AT dasar
  GSM_SIM_READY,         // SIM terdeteksi & siap (AT+CPIN)
  GSM_REGISTER_NETWORK,  // registrasi jaringan seluler (AT+CREG/CGREG)
  GSM_PDP_ACTIVE,        // PDP context aktif (data siap)
  GSM_NETWORK_READY,     // jaringan data siap dipakai
  GSM_READY,             // siap dipakai oleh sendByTransport()
  GSM_NO_RESPONSE        // modem berhenti merespons
};

enum BleTransferState {
  BLE_IDLE,
  BLE_SEND_START,
  BLE_SEND_HEADER,
  BLE_SEND_DATA,
  BLE_SEND_END
};

enum BleRxState {
  BLE_RX_IDLE,
  BLE_RX_HEADER,
  BLE_RX_DATA
};

enum BLEState {
  BLE_OFF,
  BLE_STARTING,
  BLE_RUNNING,
  BLE_STOPPING
};

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
#define BLE_CHUNK_SIZE 180

// ---- BLE Transfer Protocol (S/H/D/E) ----
constexpr size_t BLE_TX_PREFIX_RESERVE = 8;
constexpr size_t BLE_TX_DATA_LEN = (BLE_CHUNK_SIZE > BLE_TX_PREFIX_RESERVE)
                                     ? (BLE_CHUNK_SIZE - BLE_TX_PREFIX_RESERVE)
                                     : 1;
constexpr unsigned long BLE_TX_INTERVAL_MS = 15;
constexpr unsigned long BLE_RX_TIMEOUT_MS = 5000;  // reset kalau transfer RX

// ================== PIN CONFIG ==================
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18
#define PIN_CS_W5500 15
#define PIN_W5500_RST 4
#define BTN_PIN 32
#define RXD2 16
#define TXD2 17
#define LED_RUN 33
#define LED_ERR 25
#define LED_ACTIVE_LOW 1

inline void ledWrite(uint8_t pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

// ---- GSM (SIMCom A7670) UART pins (custom, add-on module) ----
#define PIN_GSM_RX 26
#define PIN_GSM_TX 27

// ---- Addon Ekspansi Input (PCF8574) I2C pins ----
#define PIN_ADDON_SDA 26
#define PIN_ADDON_SCL 27
#define PCF8574_ADDR 0x20

// ================== ADDON: EKSPANSI INPUT (PCF8574) ==================
#define ADDON_POLL_INTERVAL_MS 200
#define ADDON_DEBOUNCE_MS 500

// ================== STM32 FRAME ==================
#define SOF_BYTE 0x7E
#define MAX_PAYLOAD 512
#define MAX_FRAME 540
#define STM_EPOCH_OFFSET 946684800UL

// ===== UART DATA =====
#define MAX_RULES 10
#define MAX_RULE_LEN 100
#define TYPE_DATA 0x01
#define TYPE_ACK 0x02
#define TYPE_RULES 0x03
#define TYPE_RTC 0x04
#define TYPE_ADDON 0x06
#define TYPE_READY 0x07
#define TYPE_RULES_ACK 0x08
#define TAG_RULE_COUNT 0x50
#define TAG_RULE 0x10
#define TAG_RTC_TS 0x20
#define TAG_ADDON_MASK 0x40
#define NUM_INPUTS_TOTAL 19

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

// ---- GSM: state machine modem (probe/negosiasi/health-check) ----
#define GSM_UART_BAUD 9600
#define GSM_PROBE_INTERVAL_MS 3000  // interval cek modem saat NO_MODEM
#define GSM_STEP_INTERVAL_MS 2000
#define GSM_MAX_STEP_FAILS 5
#define GSM_HEALTH_CHECK_INTERVAL_MS 15000  // interval health-check saat READY
#define GSM_MAX_HEALTH_FAILS 3              // gagal health-check

// ---- GSM: AT command engine (gsmSendAT/gsmWaitFor) ----
#define GSM_AT_RESP_MAX 256
#define GSM_AT_DEFAULT_TIMEOUT_MS 300
#define GSM_AT_MATCH_GRACE_MS 150

// ---- GSM: transport TCP (AT+CIPOPEN/CIPSEND/CIPCLOSE) ----
#define GSM_CIPOPEN_TIMEOUT_MS 10000
#define GSM_CIPSEND_PROMPT_TIMEOUT_MS 3000
#define GSM_CIPSEND_CONFIRM_TIMEOUT_MS 5000

// ---- GSM: transport HTTP (AT+HTTPINIT/PARA/DATA/ACTION/READ) ----
#define GSM_HTTP_INIT_TIMEOUT_MS 5000
#define GSM_HTTP_PARA_TIMEOUT_MS 2000
#define GSM_HTTP_DATA_PROMPT_TIMEOUT_MS 3000
#define GSM_HTTP_DATA_CONFIRM_TIMEOUT_MS 5000
#define GSM_HTTP_ACTION_TIMEOUT_MS 15000
#define GSM_HTTP_READ_TIMEOUT_MS 5000

// ---- GSM: transport MQTT (AT+CMQTTSTART/ACCQ/CONNECT/PUB/SUB/DISC/STOP) ----
#define GSM_MQTT_CLIENT_IDX 0
#define GSM_MQTT_START_TIMEOUT_MS 5000
#define GSM_MQTT_ACCQ_TIMEOUT_MS 2000
#define GSM_MQTT_CONNECT_TIMEOUT_MS 10000
#define GSM_MQTT_SUB_TIMEOUT_MS 5000
#define GSM_MQTT_PUB_TIMEOUT_MS 5000
#define GSM_MQTT_TOPIC_PROMPT_TIMEOUT_MS 2000
#define GSM_MQTT_PAYLOAD_PROMPT_TIMEOUT_MS 2000

// ================== LED STATUS ==================
// RUN LED state machine timings (lihat RunLedState_t / updateRunLedState()):
//  - BOOT           : blink cepat, menandakan proses inisialisasi
//  - NETWORK_WAIT   : solid ON, boot selesai tapi network/cloud belum siap
//  - CLOUD_READY    : blink lambat, network & cloud sudah siap
#define LED_RUN_BLINK_BOOT_MS 100U     // periode blink cepat saat boot/init
#define LED_RUN_BLINK_CLOUD_MS 1000U   // periode blink lambat saat cloud ready
#define LED_ERR_BLINK_ON_MS 150U      // durasi tiap kedip ON
#define LED_ERR_BLINK_OFF_MS 150U     // jeda antar kedip dalam 1 pattern
#define LED_ERR_PATTERN_GAP_MS 1200U  // jeda sebelum pattern diulang

// ================== STM32 COMM WATCHDOG ==================
#define STM_DATA_PERIOD_MS 60000UL
#define STM_COMM_MISSED_CYCLES_ALLOWED 2UL
#define STM_COMM_JITTER_MARGIN_MS 15000UL
#define STM_COMM_TIMEOUT_MS ((STM_DATA_PERIOD_MS * STM_COMM_MISSED_CYCLES_ALLOWED) + STM_COMM_JITTER_MARGIN_MS)  // = 135000 ms
#define STM_COMM_CONFIRM_MS 3000UL

// ================== RULES SYNC STATE MACHINE ==================
#define RULES_STM_READY_FALLBACK_MS 3000U
#define RULES_ACK_TIMEOUT_MS 2000U   // tunggu ACK sebelum retry
#define RULES_SYNC_MAX_LOG_RETRY 5U  // hanya utk kurangi spam log

// ================== GLOBAL ==================
BLECharacteristic* bleTxChar = nullptr;
BLEServer* bleServer = nullptr;
BLEService* bleGattService = nullptr;
BLECharacteristic* bleRxChar = nullptr;
BLEState bleState = BLE_OFF;
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

// ================== RULES SYNC STATE MACHINE ==================
enum RulesSyncState_t { RS_WAIT_STM_READY,
                        RS_SENDING,
                        RS_WAIT_ACK,
                        RS_CONFIRMED };
volatile RulesSyncState_t rulesSyncState = RS_WAIT_STM_READY;
uint32_t rulesSyncStateEnteredAt = 0;
uint16_t rulesSyncRetryCount = 0;
uint8_t rulesSyncLastSentCount = 0;
volatile bool stmReadyReceived = false;
volatile bool rulesAckReceived = false;
volatile uint8_t rulesAckCount = 0;
volatile bool wsAckOk = false;
volatile uint32_t wsAckSeq = 0;
bool errRTC = false;
bool errDHT = false;
bool errADS = false;
bool errSD = false;
bool anyErrorActive = false;

struct BleTransfer {
  BleTransferState state = BLE_IDLE;
  String buffer;
  uint16_t crc = 0;
  size_t size = 0;
  uint16_t chunk = 0;
  uint16_t totalChunk = 0;
  unsigned long timer = 0;
};
BleTransfer bleTx;

struct BleRxTransfer {
  BleRxState state = BLE_RX_IDLE;
  String buffer;
  size_t expectedSize = 0;
  uint16_t expectedCrc = 0;
  uint16_t expectedTotalChunk = 0;
  uint16_t receivedChunk = 0;
  unsigned long lastActivity = 0;
};
BleRxTransfer bleRx;

// ================== SYSTEM / LED STATUS ==================
bool sysReady = false;
bool errCommTimeout = false;
bool errCommTLV = false;
bool errComm = false;
bool errNet = false;
bool fatalErr = false;
unsigned long lastStmFrameMs = 0;
static uint8_t ledErrBlinksTarget = 0;

// ---- RUN LED State Machine ----
// Satu-satunya sumber kebenaran status operasional utk RUN LED.
// Modul lain (WiFi/Ethernet/MQTT/WS/GSM/dll) TIDAK menulis GPIO LED_RUN
// secara langsung - mereka hanya mengubah sysReady/netState/wsConnected/
// mqttClient/gsmState, lalu updateRunLedState() yang membaca semua itu
// dan menentukan state; renderRunLed() satu-satunya fungsi yg menulis GPIO.
// Pengecualian: saat BLE aktif (BLE_STARTING/BLE_RUNNING), updateStatusLeds()
// meng-override LED_RUN & LED_ERR jadi solid ON, melewati state machine ini.
enum RunLedState_t {
  RUN_LED_BOOT,          // fast blink: init/boot & sedang mencoba connect ke network
  RUN_LED_NETWORK_WAIT,  // solid ON: setup network selesai tapi TIDAK dapat IP (wifi/eth gagal)
  RUN_LED_CLOUD_READY,   // slow blink: sudah dapat IP (wifi/ethernet)
  RUN_LED_OFF            // off: fatal error (STM ERROR LED yang menyala)
};
static RunLedState_t runLedState = RUN_LED_BOOT;

// ---- STM32 comm watchdog----
enum StmCommState {
  STM_COMM_OK = 0,
  STM_COMM_SUSPECT,
  STM_COMM_LOST
};
static StmCommState stmCommState = STM_COMM_OK;
static unsigned long stmCommSuspectSinceMs = 0;

// ================== NETWORK ==================
byte mac[6];
enum NetState {
  NET_DOWN,
  NET_UP
};
NetState netState = NET_DOWN;
NetIf activeIf = IF_NONE;
NetIf priorityIf = IF_WIFI;

// ================== GSM (SIMCom A7670) ==================
HardwareSerial SerialGSM(1);
GsmState gsmState = GSM_NO_MODEM;
unsigned long gsmLastProbe = 0;
unsigned long gsmLastStateChange = 0;
uint8_t gsmFailCount = 0;
uint8_t gsmHealthFailCount = 0;
bool gsmMqttStarted = false;
bool gsmMqttConnected = false;

// ================== ADDON: EKSPANSI INPUT (PCF8574) ==================
uint8_t addonInputMask = 0;
uint8_t addonRawMaskLast = 0;
unsigned long addonLastPoll = 0;
unsigned long addonLastChangeMs = 0;
bool addonMaskSentOnce = false;

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

struct EthernetConfig {
  bool dhcp;
  String ip;
  String subnet;
  String gateway;
  String dns1;
  String dns2;
};

struct GsmConfig {
  String apn;
};

struct RulesConfig {
  uint8_t count;
  char items[MAX_RULES][MAX_RULE_LEN];
};

struct AddonConfig {
  String type;
};

struct AppConfig {
  DeviceConfig device;
  NetworkConfig network;
  TransportConfig transport;
  WifiConfig wifi;
  EthernetConfig ethernet;
  GsmConfig gsm;
  RulesConfig rules;
  AddonConfig addon;
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

// ================== STM32 COMM WATCHDOG==================

void StmComm_NotifyValidFrame() {
  lastStmFrameMs = millis();

  if (stmCommState != STM_COMM_OK) {
    LOGI("[COMM] STM32 comm pulih, frame valid diterima kembali");
  }
  stmCommState = STM_COMM_OK;
  stmCommSuspectSinceMs = 0;
  errCommTimeout = false;
}

void StmComm_Update() {

  if (lastStmFrameMs == 0) return;

  unsigned long silenceMs = millis() - lastStmFrameMs;
  switch (stmCommState) {
    case STM_COMM_OK:
      if (silenceMs > STM_COMM_TIMEOUT_MS) {
        stmCommState = STM_COMM_SUSPECT;
        stmCommSuspectSinceMs = millis();
        LOGW("[COMM] STM32 comm suspect timeout, menunggu konfirmasi...");
      }
      break;

    case STM_COMM_SUSPECT:
      if (silenceMs <= STM_COMM_TIMEOUT_MS) {
        stmCommState = STM_COMM_OK;
        stmCommSuspectSinceMs = 0;
        break;
      }
      if ((millis() - stmCommSuspectSinceMs) >= STM_COMM_CONFIRM_MS) {
        stmCommState = STM_COMM_LOST;
        errCommTimeout = true;
        LOGF("[ERROR] [COMM] STM32 comm timeout TERKONFIRMASI, sudah %lu ms tanpa frame valid\n",
             silenceMs);
      }
      break;

    case STM_COMM_LOST:
      break;
  }
}

// ================== LED STATUS LOGIC ==================
void updateLedErrorFlags() {
  StmComm_Update();
  errComm = errCommTimeout || errCommTLV;
  errNet = sysReady && (netState == NET_DOWN);

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

// Menentukan apakah "Cloud" sudah siap dipakai, sesuai transport aktif.
// - mqtt / ws  : punya koneksi persistent -> cek status koneksi transport itu.
// - gsm        : gunakan state machine modem (GSM_READY = siap kirim).
// - tcp/httppost: transport ini connect per-pengiriman (tidak persistent,
//   lihat sendByTransport()/CATATAN ARSITEKTUR di atasnya), sehingga tidak
//   ada status koneksi persistent yang bisa dipantau tanpa mengubah arsitektur
//   transport tersebut. Untuk transport ini, "cloud ready" disederhanakan
//   menjadi "network layer sudah UP" (netState == NET_UP).
bool isCloudReady() {
  if (netState != NET_UP) return false;

  if (isMode("gsm")) {
    return gsmState == GSM_READY;
  }

  if (strcasecmp(config.transport.type.c_str(), "mqtt") == 0) {
    return mqttClient.connected();
  }
  if (strcasecmp(config.transport.type.c_str(), "ws") == 0) {
    return wsConnected;
  }

  // tcp / httppost (dan default lainnya)
  return true;
}

// Satu-satunya fungsi yang MENENTUKAN state RUN LED.
// Dipanggil tiap loop() dari updateStatusLeds(); membaca sysReady, netState,
// dan status transport (via isCloudReady()) - tidak ada modul lain yang boleh
// mengubah runLedState secara langsung.
void updateRunLedState() {
  if (fatalErr) {
    runLedState = RUN_LED_OFF;
  } else if (!sysReady) {
    runLedState = RUN_LED_BOOT;
  } else if (netState == NET_UP) {
    // Sudah dapat IP (wifi/ethernet) -> blink lambat, terlepas dari status
    // cloud/transport (mqtt/ws/gsm). isCloudReady() sengaja tidak dipakai
    // di sini sesuai spesifikasi: LED_RUN hanya merefleksikan status IP.
    runLedState = RUN_LED_CLOUD_READY;
  } else {
    // Belum dapat IP sama sekali (wifi maupun ethernet gagal) -> solid ON
    runLedState = RUN_LED_NETWORK_WAIT;
  }
}

// Satu-satunya fungsi yang MENULIS GPIO LED_RUN, berdasarkan runLedState.
// Non-blocking (pakai millis()), tidak mengganggu runtime/komunikasi MCU.
void renderRunLed() {
  unsigned long now = millis();
  static unsigned long lastToggle = 0;
  static bool ledOn = false;
  static RunLedState_t lastRenderedState = RUN_LED_OFF;
  static bool firstRun = true;

  if (firstRun || runLedState != lastRenderedState) {
    firstRun = false;
    lastRenderedState = runLedState;
    lastToggle = now;
    ledOn = true;  // blink states selalu mulai dari ON saat masuk state baru
  }

  switch (runLedState) {
    case RUN_LED_OFF:
      ledWrite(LED_RUN, false);
      break;

    case RUN_LED_NETWORK_WAIT:
      ledWrite(LED_RUN, true);
      break;

    case RUN_LED_BOOT:
      if (now - lastToggle >= LED_RUN_BLINK_BOOT_MS) {
        lastToggle = now;
        ledOn = !ledOn;
      }
      ledWrite(LED_RUN, ledOn);
      break;

    case RUN_LED_CLOUD_READY:
      if (now - lastToggle >= LED_RUN_BLINK_CLOUD_MS) {
        lastToggle = now;
        ledOn = !ledOn;
      }
      ledWrite(LED_RUN, ledOn);
      break;
  }
}

// ---- BOOT BLINK (hardware timer) ----
// setup()/initNetwork() sepenuhnya blocking (banyak delay() & operasi WiFi/
// Ethernet yang bisa makan waktu detik-an) sehingga renderRunLed() yang
// biasanya dipanggil dari loop() TIDAK sempat jalan cukup sering utk blink
// cepat yang terlihat mulus. Solusinya: pakai esp_timer periodik independen
// yang toggle GPIO LED_RUN sendiri, dari === BOOT === sampai === SETUP DONE ===,
// tidak peduli apapun yang sedang blocking di loop utama.
static esp_timer_handle_t bootBlinkTimer = nullptr;

static void bootBlinkTimerCb(void* arg) {
  static bool ledOn = false;
  ledOn = !ledOn;
  ledWrite(LED_RUN, ledOn);
}

void startBootBlink() {
  if (bootBlinkTimer != nullptr) return;
  const esp_timer_create_args_t timerArgs = {
    .callback = &bootBlinkTimerCb,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "boot_blink"
  };
  esp_timer_create(&timerArgs, &bootBlinkTimer);
  esp_timer_start_periodic(bootBlinkTimer, (uint64_t)LED_RUN_BLINK_BOOT_MS * 1000ULL);
}

void stopBootBlink() {
  if (bootBlinkTimer == nullptr) return;
  esp_timer_stop(bootBlinkTimer);
  esp_timer_delete(bootBlinkTimer);
  bootBlinkTimer = nullptr;
}

void updateStatusLeds() {
  unsigned long now = millis();

  // ---------- BLE ACTIVE OVERRIDE ----------
  // Selama BLE menyala (long press tombol -> startBLE()), LED_RUN & LED_ERR
  // dipaksa solid ON sebagai indikator mode BLE, mengabaikan status
  // boot/network/error normal. Ini harus dipanggil tiap loop() (bukan hanya
  // sekali di startBLE()) karena updateStatusLeds() jalan tiap iterasi loop
  // dan akan menimpa nilai GPIO jika tidak di-override di sini.
  if (bleState == BLE_STARTING || bleState == BLE_RUNNING) {
    ledWrite(LED_RUN, true);
    ledWrite(LED_ERR, true);
    return;
  }

  // ---------- LED RUN ----------
  updateRunLedState();
  renderRunLed();

  // ---------- LED ERROR ----------
  if (fatalErr) {
    ledWrite(LED_ERR, true);
    return;
  }

  if (ledErrBlinksTarget == 0) {
    ledWrite(LED_ERR, false);
    return;
  }

  static unsigned long lastStepMs = 0;
  static uint8_t blinkStep = 0;
  static bool errLedOn = false;
  static uint8_t lastTarget = 0;

  if (ledErrBlinksTarget != lastTarget) {
    lastTarget = ledErrBlinksTarget;
    blinkStep = 0;
    errLedOn = false;
    lastStepMs = now;
    ledWrite(LED_ERR, false);
    return;
  }

  if (blinkStep >= ledErrBlinksTarget) {
    if (now - lastStepMs >= LED_ERR_PATTERN_GAP_MS) {
      blinkStep = 0;
      lastStepMs = now;
    }
    ledWrite(LED_ERR, false);
    return;
  }

  if (!errLedOn) {
    if (now - lastStepMs >= LED_ERR_BLINK_OFF_MS) {
      errLedOn = true;
      lastStepMs = now;
      ledWrite(LED_ERR, true);
    }
  } else {
    if (now - lastStepMs >= LED_ERR_BLINK_ON_MS) {
      errLedOn = false;
      blinkStep++;
      lastStepMs = now;
      ledWrite(LED_ERR, false);
    }
  }
}

static inline uint16_t rd16(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | p[1];
}

static inline uint32_t rd32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;

  while (len--) {
    crc ^= *data++;

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;
}

// ================== BLE TRANSFER MODULE (S/H/D/E) ==================
bool bleTransferBusy() {
  return bleTx.state != BLE_IDLE;
}

void bleTransferReset() {
  bleTx.state = BLE_IDLE;
  bleTx.buffer = String();
  bleTx.crc = 0;
  bleTx.size = 0;
  bleTx.chunk = 0;
  bleTx.totalChunk = 0;
}

void bleTransferBegin(const String& json) {
  bleTx.buffer = json;
  bleTx.size = json.length();
  bleTx.crc = crc16((const uint8_t*)bleTx.buffer.c_str(), bleTx.size);
  bleTx.chunk = 0;
  bleTx.totalChunk = (bleTx.size + BLE_TX_DATA_LEN - 1) / BLE_TX_DATA_LEN;
  if (bleTx.totalChunk == 0) bleTx.totalChunk = 1;
  bleTx.timer = millis();
  bleTx.state = BLE_SEND_START;
}

void bleTransferTask() {
  if (bleTx.state == BLE_IDLE) return;

  if (!bleClientConnected || bleTxChar == nullptr) {
    LOGW("[BLE-TX] Client disconnected / characteristic invalid, transfer dibatalkan");
    bleTransferReset();
    return;
  }

  unsigned long now = millis();
  if (now - bleTx.timer < BLE_TX_INTERVAL_MS) return;
  bleTx.timer = now;

  char pkt[BLE_CHUNK_SIZE + 16];

  switch (bleTx.state) {

    case BLE_SEND_START:
      {
        const char* s = "S";
        bleTxChar->setValue((uint8_t*)s, 1);
        bleTxChar->notify();
        LOGI("[BLE-TX] Transfer Started");
        bleTx.state = BLE_SEND_HEADER;
        delay(20);
        break;
      }

    case BLE_SEND_HEADER:
      {
        int n = snprintf(pkt, sizeof(pkt), "H|%u|%04X|%u",
                         (unsigned)bleTx.size, bleTx.crc, (unsigned)bleTx.totalChunk);
        bleTxChar->setValue((uint8_t*)pkt, n);
        bleTxChar->notify();
        LOGI(String("[BLE-TX] Header Sent: ") + pkt);
        bleTx.state = BLE_SEND_DATA;
        delay(20);
        break;
      }

    case BLE_SEND_DATA:
      {
        size_t offset = (size_t)bleTx.chunk * BLE_TX_DATA_LEN;
        size_t remain = (offset < bleTx.size) ? (bleTx.size - offset) : 0;
        size_t len = (remain < BLE_TX_DATA_LEN) ? remain : BLE_TX_DATA_LEN;

        int n = snprintf(pkt, sizeof(pkt), "D|%u|%.*s",
                         (unsigned)(bleTx.chunk + 1), (int)len, bleTx.buffer.c_str() + offset);
        bleTxChar->setValue((uint8_t*)pkt, n);
        bleTxChar->notify();

        bleTx.chunk++;
        LOGI("[BLE-TX] Chunk " + String(bleTx.chunk) + "/" + String(bleTx.totalChunk));
        delay(100);

        if (bleTx.chunk >= bleTx.totalChunk) {
          bleTx.state = BLE_SEND_END;
        }
        break;
      }

    case BLE_SEND_END:
      {
        const char* e = "E";
        bleTxChar->setValue((uint8_t*)e, 1);
        bleTxChar->notify();
        LOGI("[BLE-TX] Transfer Finished");
        bleTransferReset();
        delay(20);
        break;
      }

    default:
      bleTransferReset();
      break;
  }
}

bool saveConfigFromBLE(const String& json) {
  StaticJsonDocument<2048> doc;
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

  return true;
}

// ================== BLE RX MODULE (S/H/D/E) ==================
void bleRxReset() {
  bleRx.state = BLE_RX_IDLE;
  bleRx.buffer = String();
  bleRx.expectedSize = 0;
  bleRx.expectedCrc = 0;
  bleRx.expectedTotalChunk = 0;
  bleRx.receivedChunk = 0;
}

void bleRxTimeoutCheck() {
  if (bleRx.state == BLE_RX_IDLE) return;
  if (millis() - bleRx.lastActivity > BLE_RX_TIMEOUT_MS) {
    LOGW("[BLE-RX] Transfer timeout, buffer direset");
    bleRxReset();
  }
}

void handleBleCommand(const String& cmd) {
  if (cmd == "CMD:GET_CONFIG") {
    // ---- Guard: BLE Client Connected ----
    if (!bleClientConnected) {
      LOGW("[BLE] GET_CONFIG ditolak: tidak ada client terkoneksi");
      return;
    }
    // ---- Guard: Characteristic Valid ----
    if (bleTxChar == nullptr) {
      LOGE("[BLE] GET_CONFIG ditolak: TX characteristic belum siap");
      return;
    }
    // ---- Guard: Transfer Busy ----
    if (bleTransferBusy()) {
      LOGW("[BLE] GET_CONFIG ditolak: transfer sebelumnya masih berjalan (Transfer Busy)");
      return;
    }

    if (!LittleFS.exists("/config.json")) {
      Serial.println("[BLE] config.json not found");
      return;
    }

    File f = LittleFS.open("/config.json", "r");
    if (!f) {
      Serial.println("[BLE] failed to open config.json");
      return;
    }

    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();

    if (err) {
      LOGE("[BLE] config.json parse error, kirim mentah tanpa status GSM");
      File f2 = LittleFS.open("/config.json", "r");
      String raw = f2.readString();
      f2.close();

      bleTransferBegin(raw);
      LOGI("[BLE] Config transfer started (raw fallback)");
      return;
    }
    doc["gsmAttached"] = (gsmState != GSM_NO_MODEM);

    String json;
    serializeJson(doc, json);

    bleTransferBegin(json);

    LOGI("[BLE] Config transfer started");
  } else if (cmd.startsWith("CMD:CONFIG:")) {
    String json = cmd.substring(strlen("CMD:CONFIG:"));

    bool ok = saveConfigFromBLE(json);

    if (bleTxChar) {
      String resp = ok ? "SAVED:OK" : "SAVED:FAIL:invalid_json";
      bleTxChar->setValue(resp.c_str());
      bleTxChar->notify();
    }

    if (ok) {
      LOGI("[BLE] Config Saved");

      needRestart = true;
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

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {

    String raw = pChar->getValue();
    if (raw.length() == 0) return;

    Serial.print("[BLE RX] ");
    Serial.println(raw);

    // ---- Framed protocol (S/H/D/E) untuk payload panjang ----
    if (raw == "S") {
      bleRxReset();
      bleRx.state = BLE_RX_HEADER;
      bleRx.lastActivity = millis();
      LOGI("[BLE-RX] Transfer Started");
      return;
    }

    if (raw.startsWith("H|")) {
      if (bleRx.state != BLE_RX_HEADER) {
        LOGW("[BLE-RX] Header diterima tanpa Start, diabaikan");
        return;
      }
      // format: H|<size>|<crcHex>|<totalChunk>
      int p1 = raw.indexOf('|', 2);
      int p2 = (p1 > 0) ? raw.indexOf('|', p1 + 1) : -1;
      if (p1 < 0 || p2 < 0) {
        LOGE("[BLE-RX] Header format invalid");
        bleRxReset();
        return;
      }
      bleRx.expectedSize = (size_t)raw.substring(2, p1).toInt();
      bleRx.expectedCrc = (uint16_t)strtoul(raw.substring(p1 + 1, p2).c_str(), nullptr, 16);
      bleRx.expectedTotalChunk = (uint16_t)raw.substring(p2 + 1).toInt();
      bleRx.buffer = String();
      bleRx.buffer.reserve(bleRx.expectedSize + 1);
      bleRx.receivedChunk = 0;
      bleRx.state = BLE_RX_DATA;
      bleRx.lastActivity = millis();
      LOGI("[BLE-RX] Header: size=" + String((unsigned)bleRx.expectedSize) + " crc=" + String(bleRx.expectedCrc, HEX) + " totalChunk=" + String(bleRx.expectedTotalChunk));
      return;
    }

    if (raw.startsWith("D|")) {
      if (bleRx.state != BLE_RX_DATA) {
        LOGW("[BLE-RX] Chunk diterima tanpa Header, diabaikan");
        return;
      }
      int p1 = raw.indexOf('|', 2);
      if (p1 < 0) {
        LOGE("[BLE-RX] Chunk format invalid");
        return;
      }
      uint16_t chunkNum = (uint16_t)raw.substring(2, p1).toInt();
      String data = raw.substring(p1 + 1);
      bleRx.buffer += data;
      bleRx.receivedChunk = chunkNum;
      bleRx.lastActivity = millis();
      LOGI("[BLE-RX] Chunk " + String(chunkNum) + "/" + String(bleRx.expectedTotalChunk));
      return;
    }

    if (raw == "E") {
      if (bleRx.state != BLE_RX_DATA) {
        LOGW("[BLE-RX] End diterima tanpa transfer aktif, diabaikan");
        bleRxReset();
        return;
      }

      bool sizeOk = (bleRx.buffer.length() == bleRx.expectedSize);
      uint16_t actualCrc = crc16((const uint8_t*)bleRx.buffer.c_str(), bleRx.buffer.length());
      bool crcOk = (actualCrc == bleRx.expectedCrc);

      if (!sizeOk || !crcOk) {
        LOGE("[BLE-RX] Transfer gagal (size " + String((unsigned)bleRx.buffer.length()) + "/" + String((unsigned)bleRx.expectedSize) + ", crc " + String(actualCrc, HEX) + "/" + String(bleRx.expectedCrc, HEX) + ")");
        if (bleTxChar) {
          bleTxChar->setValue("RXFAIL:crc_or_size");
          bleTxChar->notify();
        }
        bleRxReset();
        return;
      }

      LOGI("[BLE-RX] Transfer selesai, " + String((unsigned)bleRx.buffer.length()) + " byte diterima utuh");
      String completeCmd = bleRx.buffer;
      bleRxReset();
      handleBleCommand(completeCmd);
      return;
    }

    handleBleCommand(raw);
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

    if (bleTransferBusy()) {
      LOGW("[BLE-TX] client disconnect di tengah transfer, transfer dibatalkan");
      bleTransferReset();
    }

    if (bleState == BLE_RUNNING) {
      BLEDevice::getAdvertising()->start();
      LOGI("[BLE] advertising restarted");
    }
  }
};

static BleServerCallbacks bleServerCallbacksInstance;
static BleRxCallbacks bleRxCallbacksInstance;

bool isMode(const char* m) {
  return strcasecmp(config.network.mode.c_str(), m) == 0;
}

bool isAddon(const char* t) {
  return strcasecmp(config.addon.type.c_str(), t) == 0;
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

  if (isMode("hybrid")) {
    static WiFiClient hybridWifiClient;
    static EthernetClient hybridEthClient;
    if (activeIf == IF_WIFI) return &hybridWifiClient;
    if (activeIf == IF_ETH) return &hybridEthClient;
    return nullptr;
  }

  return nullptr;
}

// =================== FILE HELPERS ====================
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

  StaticJsonDocument<2048> doc;
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

  config.wifi.ssid.trim();
  config.wifi.password.trim();

  // ---------------- ETHERNET ----------------
  config.ethernet.dhcp =
    doc["ethernet"]["dhcp"] | true;
  config.ethernet.ip =
    doc["ethernet"]["ip"] | "";
  config.ethernet.subnet =
    doc["ethernet"]["subnet"] | "";
  config.ethernet.gateway =
    doc["ethernet"]["gateway"] | "";
  config.ethernet.dns1 =
    doc["ethernet"]["dns1"] | "";
  config.ethernet.dns2 =
    doc["ethernet"]["dns2"] | "";

  // ---------------- GSM (opsional, add-on) ----------------
  config.gsm.apn =
    doc["gsm"]["apn"] | "";

  // ---------------- ADDON  ----------------
  config.addon.type =
    doc["addon"]["type"] | "none";

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
  if (isMode("gsm") && strcasecmp(config.transport.type.c_str(), "ws") == 0) {
    LOGE("[CFG] INVALID: GSM tidak mendukung WebSocket, transport dipaksa ke tcp");
    config.transport.type = "tcp";
  }

  if (strcasecmp(config.transport.type.c_str(), "ws") == 0 && !isMode("wifi")) {
    LOGE("[CFG] INVALID: transport=ws butuh network mode=wifi, dipaksa ke wifi");
    config.network.mode = "wifi";
  }

  if (isMode("gsm") && !isAddon("gsm")) {
    LOGE("[CFG] INVALID: network mode=gsm tapi addon.type bukan gsm, network mode dipaksa ke wifi");
    config.network.mode = "wifi";
  }

  initPriority();
  return true;
}

// =================== NET HELPERS =====================
bool startEthernet(uint32_t timeoutMs = ETH_LINK_TIMEOUT_MS) {
  Ethernet.init(PIN_CS_W5500);

  auto hw = Ethernet.hardwareStatus();
  LOGF("[NET] hardwareStatus = %d\n", (int)hw);

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
    delay(500);
  }

  // if (Ethernet.linkStatus() != LinkON) {
  //   LOGI("[NET] Ethernet cable not detected");
  //   return false;
  // }

  if (config.ethernet.dhcp) {
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
      delay(ETH_DHCP_RETRY_MS);
    }
    LOGI("[NET] DHCP timeout");
    return false;
  } else {
    // ---------- STATIC IP ----------
    LOGI("[NET] applying Static IP...");
    getEspBaseMac(mac);
    LOGF("[NET] MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    IPAddress ip, dns1, gateway, subnet;
    ip.fromString(config.ethernet.ip);
    subnet.fromString(config.ethernet.subnet);
    gateway.fromString(config.ethernet.gateway);
    dns1.fromString(config.ethernet.dns1);

    Ethernet.begin(mac, ip, dns1, gateway, subnet);
    if (Ethernet.localIP() == IPAddress(255, 255, 255, 255) || Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
      LOGI("[NET] Static IP assignment FAILED (hardware not responding)");
      return false;
    }
    LOG("[NET] Ethernet IP (Static): ");
    Serial.println(Ethernet.localIP());
    return true;
  }
}

bool tryEthernet() {
  LOGI("[NET] try Ethernet...");
  if (startEthernet()) {
    netState = NET_UP;
    activeIf = IF_ETH;
    sysReady = true;
    LOGI("[NET] Ethernet READY");
    return true;
  }
  LOGI("[NET] Ethernet FAIL");
  return false;
}

void printSSIDHex(const String& ssid) {
  LOGN("========== SSID DEBUG ==========");
  LOGF("SSID      : <%s>\n", ssid.c_str());
  LOGF("Length    : %d\n", ssid.length());

  LOG("HEX       : ");

  for (size_t i = 0; i < ssid.length(); i++) {
    LOGF("%02X ", (uint8_t)ssid[i]);
  }

  LOGN("");
  LOGN("===============================");
}

void scanWifiList() {
  LOGI("Scanning WiFi...");

  WiFi.mode(WIFI_STA);
  delay(500);

  WiFi.disconnect(true, true);
  delay(1000);

  int n = WiFi.scanNetworks(false, false);

  if (n == WIFI_SCAN_FAILED) {
    LOGE("Scan failed");
    return;
  }

  LOGF("[INFO ] %d AP ditemukan\n", n);

  for (int i = 0; i < n; i++) {
    LOGF("[%02d] %-32s RSSI:%4d CH:%2d ENC:%d\n",
         i,
         WiFi.SSID(i).c_str(),
         WiFi.RSSI(i),
         WiFi.channel(i),
         WiFi.encryptionType(i));

    if (WiFi.SSID(i) == config.wifi.ssid) {
      LOGI(">>> TARGET SSID DITEMUKAN <<<");
    }
  }

  WiFi.scanDelete();
}

bool startWiFiSTA(uint32_t timeoutMs = HEARTBEAT_MS) {

  config.wifi.ssid.trim();
  config.wifi.password.trim();

  if (config.wifi.ssid.isEmpty()) {
    LOGE("SSID kosong");
    return false;
  }

  printSSIDHex(config.wifi.ssid);

  scanWifiList();

  LOGI("Reset WiFi Driver");

  WiFi.disconnect(true, true);
  delay(300);

  WiFi.mode(WIFI_STA);
  delay(300);

  LOGF("[INFO ] Connecting to <%s>\n",
       config.wifi.ssid.c_str());

  WiFi.begin(config.wifi.ssid.c_str(),
             config.wifi.password.c_str());

  uint32_t start = millis();
  wl_status_t lastStatus = WL_IDLE_STATUS;

  while (millis() - start < timeoutMs) {
    wl_status_t status = WiFi.status();

    if (status != lastStatus) {
      lastStatus = status;

      LOGF("[DEBUG] WiFi Status = %d\n", status);
    }

    if (status == WL_CONNECTED) {
      break;
    }

    delay(500);

    LOG(".");
  }

  LOGN("");

  if (WiFi.status() == WL_CONNECTED) {
    LOGI("WiFi Connected");

    LOGF("SSID      : %s\n", WiFi.SSID().c_str());
    LOGF("IP        : %s\n", WiFi.localIP().toString().c_str());
    LOGF("Gateway   : %s\n", WiFi.gatewayIP().toString().c_str());
    LOGF("Subnet    : %s\n", WiFi.subnetMask().toString().c_str());
    LOGF("DNS       : %s\n", WiFi.dnsIP().toString().c_str());
    LOGF("RSSI      : %d dBm\n", WiFi.RSSI());
    LOGF("Channel   : %d\n", WiFi.channel());
    LOGF("BSSID     : %s\n", WiFi.BSSIDstr().c_str());

    return true;
  }

  LOGE("WiFi Connection Failed");

  WiFi.disconnect(true, true);

  return false;
}

bool tryWiFi() {
  LOGI("[NET] try WiFi...");
  if (startWiFiSTA()) {
    netState = NET_UP;
    LOGI("[NET] WiFi READY");
    activeIf = IF_WIFI;
    sysReady = true;
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
  ws.setReconnectInterval(SENDWAIT_INTERVAL_MS);
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

// ================ GSM (SIMCom A7670) =================
void gsmFlushRx() {
  while (SerialGSM.available()) {
    SerialGSM.read();
  }
}

bool gsmSendAT(const char* cmd, const char* expect,
               uint32_t timeoutMs = GSM_AT_DEFAULT_TIMEOUT_MS,
               char* respBuf = nullptr, size_t respBufLen = 0) {
  gsmFlushRx();

  SerialGSM.print(cmd);
  SerialGSM.print("\r\n");

  String resp;
  resp.reserve(GSM_AT_RESP_MAX);

  unsigned long start = millis();
  bool found = false;
  bool gotError = false;
  unsigned long foundAt = 0;

  while (millis() - start < timeoutMs) {
    while (SerialGSM.available()) {
      char c = (char)SerialGSM.read();
      resp += c;
      if (resp.length() > GSM_AT_RESP_MAX) {
        resp.remove(0, resp.length() - GSM_AT_RESP_MAX);
      }
      if (!found && expect != nullptr && expect[0] != '\0' && resp.indexOf(expect) != -1) {
        found = true;
        foundAt = millis();
      }
      if (resp.indexOf("ERROR") != -1) {
        gotError = true;
      }
    }
    if (gotError) break;

    if (found && (millis() - foundAt >= GSM_AT_MATCH_GRACE_MS)) break;
    yield();
  }

  if (respBuf != nullptr && respBufLen > 0) {
    size_t n = resp.length();
    if (n >= respBufLen) n = respBufLen - 1;
    memcpy(respBuf, resp.c_str(), n);
    respBuf[n] = '\0';
  }

  LOGF("[GSM][AT] TX: %s\n", cmd);
  LOGF("[GSM][AT] RX: %s\n", resp.c_str());

  return found;
}

bool gsmWaitFor(const char* expect, uint32_t timeoutMs,
                char* respBuf = nullptr, size_t respBufLen = 0) {
  String resp;
  resp.reserve(GSM_AT_RESP_MAX);

  unsigned long start = millis();
  bool found = false;

  while (millis() - start < timeoutMs) {
    while (SerialGSM.available()) {
      char c = (char)SerialGSM.read();
      resp += c;
      if (resp.length() > GSM_AT_RESP_MAX) {
        resp.remove(0, resp.length() - GSM_AT_RESP_MAX);
      }
      if (expect != nullptr && expect[0] != '\0' && resp.indexOf(expect) != -1) {
        found = true;
      }
    }
    if (found) break;
    yield();
  }

  if (respBuf != nullptr && respBufLen > 0) {
    size_t n = resp.length();
    if (n >= respBufLen) n = respBufLen - 1;
    memcpy(respBuf, resp.c_str(), n);
    respBuf[n] = '\0';
  }
  return found;
}

const char* gsmStateToStr(GsmState s) {
  switch (s) {
    case GSM_NO_MODEM: return "NO_MODEM";
    case GSM_MODEM_FOUND: return "MODEM_FOUND";
    case GSM_POWER_ON: return "POWER_ON";
    case GSM_AT_READY: return "AT_READY";
    case GSM_SIM_READY: return "SIM_READY";
    case GSM_REGISTER_NETWORK: return "REGISTER_NETWORK";
    case GSM_PDP_ACTIVE: return "PDP_ACTIVE";
    case GSM_NETWORK_READY: return "NETWORK_READY";
    case GSM_READY: return "READY";
    case GSM_NO_RESPONSE: return "NO_RESPONSE";
    default: return "UNKNOWN";
  }
}

void gsmSetState(GsmState newState) {
  if (newState == gsmState) return;
  LOGI("[GSM] " + String(gsmStateToStr(gsmState)) + " -> " + String(gsmStateToStr(newState)));
  GsmState oldState = gsmState;
  gsmState = newState;
  gsmLastStateChange = millis();
  gsmLastProbe = 0;
  gsmFailCount = 0;
  gsmHealthFailCount = 0;

  if (newState == GSM_READY) {
    netState = NET_UP;
  } else if (oldState == GSM_READY) {
    netState = NET_DOWN;
  }

  if (newState == GSM_NO_MODEM) {
    gsmMqttStarted = false;
    gsmMqttConnected = false;
  }
}

bool gsmIsReady() {
  return gsmState == GSM_READY;
}

void gsmInit() {
  LOGI("[GSM] Initializing...");
  SerialGSM.begin(GSM_UART_BAUD, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
  gsmSetState(GSM_NO_MODEM);
  gsmLastProbe = 0;
}

// ============ GSM: DETEKSI MODEM ) ============
void gsmDetectLoop() {
  unsigned long now = millis();

  if (gsmState == GSM_NO_MODEM) {
    if (now - gsmLastProbe >= GSM_PROBE_INTERVAL_MS) {
      gsmLastProbe = now;
      LOGI("[GSM] Waiting Modem...");
      if (gsmSendAT("AT", "OK")) {
        gsmSetState(GSM_MODEM_FOUND);
      }
    }
  }
}

// ============ GSM: NEGOSIASI JARINGAN) ============
void gsmNetworkLoop() {
  unsigned long now = millis();

  if (gsmState != GSM_NO_MODEM && gsmState != GSM_READY && gsmState != GSM_NO_RESPONSE && gsmFailCount >= GSM_MAX_STEP_FAILS) {
    gsmSetState(GSM_NO_RESPONSE);
  }

  switch (gsmState) {
    case GSM_NO_MODEM:
      break;

    case GSM_MODEM_FOUND:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        if (gsmSendAT("ATE0", "OK")) {
          gsmSetState(GSM_POWER_ON);
        } else {
          gsmFailCount++;
        }
      }
      break;

    case GSM_POWER_ON:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        if (gsmSendAT("AT", "OK")) {
          LOGI("[GSM] AT Ready");
          gsmSetState(GSM_AT_READY);
        } else {
          gsmFailCount++;
        }
      }
      break;

    case GSM_AT_READY:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        char resp[GSM_AT_RESP_MAX];
        if (gsmSendAT("AT+CPIN?", "+CPIN:", 1000, resp, sizeof(resp)) && strstr(resp, "READY") != nullptr) {
          LOGI("[GSM] SIM Ready");
          gsmSetState(GSM_SIM_READY);
        } else {
          gsmFailCount++;
          LOGW("[GSM] SIM belum siap...");
        }
      }
      break;

    case GSM_SIM_READY:
      gsmSetState(GSM_REGISTER_NETWORK);
      break;

    case GSM_REGISTER_NETWORK:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        char resp[GSM_AT_RESP_MAX];

        if (gsmSendAT("AT+CREG?", "+CREG:", 1000, resp, sizeof(resp)) && (strstr(resp, ",1") != nullptr || strstr(resp, ",5") != nullptr)) {
          LOGI("[GSM] Registered");
          gsmSetState(GSM_PDP_ACTIVE);
        } else {
          gsmFailCount++;
          LOGW("[GSM] Belum register ke jaringan seluler...");
        }
      }
      break;

    case GSM_PDP_ACTIVE:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        LOGI("[GSM] Opening PDP");
        bool apnOk = true;
        if (config.gsm.apn.length() > 0) {
          char cmd[96];
          snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", config.gsm.apn.c_str());
          apnOk = gsmSendAT(cmd, "OK", 1000);
        }
        if (apnOk && gsmSendAT("AT+CGACT=1,1", "OK", 5000)) {
          gsmSetState(GSM_NETWORK_READY);
        } else {
          gsmFailCount++;
          LOGW("[GSM] PDP context gagal diaktifkan...");
        }
      }
      break;

    case GSM_NETWORK_READY:
      if (now - gsmLastProbe >= GSM_STEP_INTERVAL_MS) {
        gsmLastProbe = now;
        char resp[GSM_AT_RESP_MAX];
        if (gsmSendAT("AT+CGPADDR=1", "+CGPADDR:", 1000, resp, sizeof(resp))) {
          LOGI("[GSM] Network Ready");
          gsmSetState(GSM_READY);
        } else {
          gsmFailCount++;
        }
      }
      break;

    case GSM_READY:
      if (now - gsmLastProbe >= GSM_HEALTH_CHECK_INTERVAL_MS) {
        gsmLastProbe = now;
        if (gsmSendAT("AT", "OK", 500)) {
          gsmHealthFailCount = 0;
        } else {
          gsmHealthFailCount++;
          LOGW("[GSM] health-check gagal (" + String(gsmHealthFailCount) + "/" + String(GSM_MAX_HEALTH_FAILS) + ")");
          if (gsmHealthFailCount >= GSM_MAX_HEALTH_FAILS) {
            gsmSetState(GSM_NO_RESPONSE);
          }
        }
      }
      break;

    case GSM_NO_RESPONSE:
      LOGW("[GSM] Modem Removed");
      gsmSetState(GSM_NO_MODEM);
      break;
  }
}

// ============ GSM: TCP TRANSPORT (AT+CIPOPEN/CIPSEND/CIPCLOSE) ============
bool gsmTcpSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  char cmd[160];

  gsmSendAT("AT+CIPCLOSE=0", "OK", 2000);

  LOG("[GSM][TCP] connect ");
  LOG(config.transport.host);
  LOG(":");
  LOGI(config.transport.port);

  snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%u",
           config.transport.host.c_str(), config.transport.port);
  if (!gsmSendAT(cmd, "+CIPOPEN: 0,0", GSM_CIPOPEN_TIMEOUT_MS)) {
    LOGW("[GSM][TCP] connect FAIL");
    return false;
  }
  LOGI("[GSM] TCP Connected");

  String frame = String((unsigned)strlen(jsonLine)) + "\n" + String(jsonLine);

  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%u", (unsigned)frame.length());
  gsmFlushRx();
  SerialGSM.print(cmd);
  SerialGSM.print("\r\n");

  if (!gsmWaitFor(">", GSM_CIPSEND_PROMPT_TIMEOUT_MS)) {
    LOGW("[GSM][TCP] send prompt timeout");
    gsmSendAT("AT+CIPCLOSE=0", "OK", 2000);
    return false;
  }

  SerialGSM.write((const uint8_t*)frame.c_str(), frame.length());

  if (!gsmWaitFor("OK", GSM_CIPSEND_CONFIRM_TIMEOUT_MS)) {
    LOGW("[GSM][TCP] send FAIL");
    gsmSendAT("AT+CIPCLOSE=0", "OK", 2000);
    return false;
  }

  LOG("[GSM][TCP] sent len=");
  LOGI(frame.length());

  char ackResp[GSM_AT_RESP_MAX];
  bool haveData = gsmWaitFor("}", ACK_TIMEOUT_MS, ackResp, sizeof(ackResp));

  gsmSendAT("AT+CIPCLOSE=0", "OK", 2000);
  LOGI("[GSM] TCP Closed");

  if (!haveData) {
    LOGW("[GSM][TCP] no response");
    return false;
  }

  char* jsonStart = strchr(ackResp, '{');
  if (!jsonStart) {
    LOGW("[GSM][TCP] no JSON in response");
    return false;
  }
  return parseAck(jsonStart, seq);
}

// ============ GSM: HTTP TRANSPORT (AT+HTTPINIT/PARA/DATA/ACTION/READ) ============
bool gsmHttpSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  char cmd[256];

  gsmSendAT("AT+HTTPTERM", "OK", 2000);

  if (!gsmSendAT("AT+HTTPINIT", "OK", GSM_HTTP_INIT_TIMEOUT_MS)) {
    LOGW("[GSM][HTTP] init FAIL");
    return false;
  }

  gsmSendAT("AT+HTTPPARA=\"CID\",1", "OK", GSM_HTTP_PARA_TIMEOUT_MS);

  String url = "http://" + config.transport.host + ":" + String(config.transport.port) + config.transport.path;
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url.c_str());
  if (!gsmSendAT(cmd, "OK", GSM_HTTP_PARA_TIMEOUT_MS)) {
    LOGW("[GSM][HTTP] set URL FAIL");
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  if (!gsmSendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", GSM_HTTP_PARA_TIMEOUT_MS)) {
    LOGW("[GSM][HTTP] set content-type FAIL");
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  uint16_t bodyLen = strlen(jsonLine);
  snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%u,10000", bodyLen);
  gsmFlushRx();
  SerialGSM.print(cmd);
  SerialGSM.print("\r\n");
  if (!gsmWaitFor("DOWNLOAD", GSM_HTTP_DATA_PROMPT_TIMEOUT_MS)) {
    LOGW("[GSM][HTTP] data prompt timeout");
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  SerialGSM.write((const uint8_t*)jsonLine, bodyLen);
  if (!gsmWaitFor("OK", GSM_HTTP_DATA_CONFIRM_TIMEOUT_MS)) {
    LOGW("[GSM][HTTP] data upload FAIL");
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  LOG("[GSM][HTTP] POST ");
  LOGI(url);

  char actionResp[GSM_AT_RESP_MAX];
  if (!gsmSendAT("AT+HTTPACTION=1", "+HTTPACTION: 1,",
                 GSM_HTTP_ACTION_TIMEOUT_MS, actionResp, sizeof(actionResp))) {
    LOGW("[GSM][HTTP] action timeout");
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }

  int httpStatus = 0;
  char* statusPos = strstr(actionResp, "+HTTPACTION: 1,");
  if (statusPos != nullptr) {
    httpStatus = atoi(statusPos + strlen("+HTTPACTION: 1,"));
  }
  if (httpStatus != 200) {
    LOGF("[GSM][HTTP] POST FAIL, status=%d\n", httpStatus);
    gsmSendAT("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  LOGI("[GSM] HTTP POST OK");

  char readResp[GSM_AT_RESP_MAX];
  bool haveBody = gsmSendAT("AT+HTTPREAD", "+HTTPREAD:",
                            GSM_HTTP_READ_TIMEOUT_MS, readResp, sizeof(readResp));

  gsmSendAT("AT+HTTPTERM", "OK", 2000);

  if (!haveBody) {
    LOGW("[GSM][HTTP] no body");
    return false;
  }

  char* jsonStart = strchr(readResp, '{');
  if (!jsonStart) {
    LOGW("[GSM][HTTP] no JSON in response body");
    return false;
  }
  return parseAck(jsonStart, seq);
}

// ============ GSM: MQTT TRANSPORT (AT+CMQTTSTART/ACCQ/CONNECT/PUB/SUB/DISC/STOP) ============
bool gsmMqttSendPrompted(const char* atCmd, const char* data, uint32_t promptTimeoutMs) {
  gsmFlushRx();
  SerialGSM.print(atCmd);
  SerialGSM.print("\r\n");
  if (!gsmWaitFor(">", promptTimeoutMs)) {
    return false;
  }
  SerialGSM.print(data);
  return gsmWaitFor("OK", 2000);
}

bool gsmMqttEnsureConnected() {
  if (gsmMqttConnected) return true;

  char cmd[160];

  if (!gsmMqttStarted) {
    LOGI("[GSM][MQTT] starting service...");
    if (!gsmSendAT("AT+CMQTTSTART", "+CMQTTSTART:", GSM_MQTT_START_TIMEOUT_MS)) {
      LOGW("[GSM][MQTT] start FAIL");
      return false;
    }
    gsmMqttStarted = true;
  }

  String clientId = "iot-node-" + String(config.device.boxID);
  snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=%d,\"%s\"", GSM_MQTT_CLIENT_IDX, clientId.c_str());
  if (!gsmSendAT(cmd, "OK", GSM_MQTT_ACCQ_TIMEOUT_MS)) {
    LOGW("[GSM][MQTT] ACCQ FAIL");
    return false;
  }

  LOG("[GSM][MQTT] connecting ");
  LOG(config.transport.host);
  LOG(":");
  LOGI(config.transport.port);

  snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=%d,\"tcp://%s:%u\",60,1",
           GSM_MQTT_CLIENT_IDX, config.transport.host.c_str(), config.transport.port);
  char connResp[GSM_AT_RESP_MAX];
  if (!gsmSendAT(cmd, "+CMQTTCONNECT:", GSM_MQTT_CONNECT_TIMEOUT_MS, connResp, sizeof(connResp))) {
    LOGW("[GSM][MQTT] connect timeout");
    return false;
  }
  // format: "+CMQTTCONNECT: <idx>,<result>" - result 0 = sukses
  int result = -1;
  char* p = strstr(connResp, "+CMQTTCONNECT:");
  if (p != nullptr) {
    char* comma = strchr(p, ',');
    if (comma != nullptr) result = atoi(comma + 1);
  }
  if (result != 0) {
    LOGF("[GSM][MQTT] connect FAIL, result=%d\n", result);
    return false;
  }
  LOGI("[GSM] MQTT Connected");

  String ackTopic = "iot/ack/" + String(config.device.boxID);
  snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=%d,%u,1",
           GSM_MQTT_CLIENT_IDX, (unsigned)ackTopic.length());
  if (!gsmMqttSendPrompted(cmd, ackTopic.c_str(), GSM_MQTT_TOPIC_PROMPT_TIMEOUT_MS)) {
    LOGW("[GSM][MQTT] set sub-topic FAIL");
    return false;
  }
  snprintf(cmd, sizeof(cmd), "AT+CMQTTSUB=%d", GSM_MQTT_CLIENT_IDX);
  if (!gsmSendAT(cmd, "+CMQTTSUB:", GSM_MQTT_SUB_TIMEOUT_MS)) {
    LOGW("[GSM][MQTT] subscribe FAIL");
    return false;
  }

  gsmMqttConnected = true;
  return true;
}

bool gsmMqttPublish(const char* topic, const char* payload) {
  char cmd[64];

  snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=%d,%u", GSM_MQTT_CLIENT_IDX, (unsigned)strlen(topic));
  if (!gsmMqttSendPrompted(cmd, topic, GSM_MQTT_TOPIC_PROMPT_TIMEOUT_MS)) {
    LOGW("[GSM][MQTT] set topic FAIL");
    return false;
  }

  snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=%d,%u", GSM_MQTT_CLIENT_IDX, (unsigned)strlen(payload));
  if (!gsmMqttSendPrompted(cmd, payload, GSM_MQTT_PAYLOAD_PROMPT_TIMEOUT_MS)) {
    LOGW("[GSM][MQTT] set payload FAIL");
    return false;
  }

  snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=%d,1,60", GSM_MQTT_CLIENT_IDX);  // QoS 1, timeout 60s
  char pubResp[GSM_AT_RESP_MAX];
  if (!gsmSendAT(cmd, "+CMQTTPUB:", GSM_MQTT_PUB_TIMEOUT_MS, pubResp, sizeof(pubResp))) {
    LOGW("[GSM][MQTT] publish timeout");
    return false;
  }
  int result = -1;
  char* p = strstr(pubResp, "+CMQTTPUB:");
  if (p != nullptr) {
    char* comma = strchr(p, ',');
    if (comma != nullptr) result = atoi(comma + 1);
  }
  return result == 0;
}

bool gsmMqttWaitAck(uint32_t seq, uint32_t timeoutMs) {
  char buf[GSM_AT_RESP_MAX];
  if (!gsmWaitFor("+CMQTTRXEND:", timeoutMs, buf, sizeof(buf))) {
    return false;
  }
  char* jsonStart = strchr(buf, '{');
  if (jsonStart == nullptr) {
    return false;
  }
  return parseAck(jsonStart, seq);
}

void gsmMqttDisconnect() {
  if (!gsmMqttStarted && !gsmMqttConnected) return;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CMQTTDISC=%d,60", GSM_MQTT_CLIENT_IDX);
  gsmSendAT(cmd, "OK", 5000);
  snprintf(cmd, sizeof(cmd), "AT+CMQTTREL=%d", GSM_MQTT_CLIENT_IDX);
  gsmSendAT(cmd, "OK", 2000);
  gsmSendAT("AT+CMQTTSTOP", "OK", 5000);
  gsmMqttConnected = false;
  gsmMqttStarted = false;
}

bool gsmMqttSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (!gsmMqttEnsureConnected()) {
    gsmMqttConnected = false;
    return false;
  }

  LOGI("[GSM][MQTT] TX:");
  LOGI(jsonLine);

  if (!gsmMqttPublish(config.transport.path.c_str(), jsonLine)) {
    LOGW("[GSM][MQTT] publish FAIL");
    gsmMqttConnected = false;
    return false;
  }

  bool ackOk = gsmMqttWaitAck(seq, ACK_TIMEOUT_MS);
  if (ackOk) {
    LOGI("[GSM][MQTT] ACK OK");
  } else {
    LOGW("[GSM][MQTT] ACK TIMEOUT");
  }
  return ackOk;
}

bool gsmSendJSONWithSeq(uint32_t seq, const char* jsonLine) {
  if (!gsmIsReady()) {
    LOGW("[GSM] belum READY (state=" + String(gsmStateToStr(gsmState)) + "), kirim dibatalkan");
    return false;
  }
  if (strcasecmp(config.transport.type.c_str(), "tcp") == 0) {
    return gsmTcpSendJSONWithSeq(seq, jsonLine);
  }
  if (strcasecmp(config.transport.type.c_str(), "httppost") == 0) {
    return gsmHttpSendJSONWithSeq(seq, jsonLine);
  }
  if (strcasecmp(config.transport.type.c_str(), "mqtt") == 0) {
    return gsmMqttSendJSONWithSeq(seq, jsonLine);
  }
  LOG("[GSM] transport tidak didukung: ");
  LOGI(config.transport.type);
  return false;
}

// ================ TRANSPORT HELPERS ==================
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
    delay(1);
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

/* ============================================================================
   CATATAN ARSITEKTUR (hasil review reliability end-to-end STM32<->ESP32<->Cloud)
   ============================================================================
   Fungsi sendByTransport() ini dipanggil SECARA BLOCKING dari
   handleDataFromSTM() (dipanggil dari handleUART() setiap loop()). Selama
   fungsi ini berjalan (bisa sampai ~35-40 detik best-case utk mode GSM+HTTP:
   HTTPINIT + PARA + prompt + confirm + HTTPACTION 15s + HTTPREAD), SELURUH
   loop() ESP32 berhenti: UART dari STM32 tidak diproses, BLE tidak jalan,
   watchdog comm tidak di-service.

   Konsekuensi nyata: STM32 menunggu ACK balik hanya ACK_TIMEOUT_MS (lihat
   main.c - sudah dinaikkan jadi 45000ms utk mengakomodasi transport GSM/HTTP
   yg lambat, TAPI kalau instalasi Anda pakai transport itu dan network makin
   lambat dari perkiraan, STM32 tetap bisa timeout & retry SEBELUM ESP32
   selesai proses request sebelumnya -> ESP32 memproses TLV yang sama sebagai
   pengiriman BARU ke cloud -> DUPLIKAT.

   Redesign yang direkomendasikan (belum diimplementasikan di sini karena
   perlu pengujian hardware nyata utk tiap transport - GSM AT engine, HTTP,
   TCP, MQTT, WS - agar tidak merusak yang sudah battle-tested):

   1. Non-blocking dispatcher: ganti sendByTransport() blocking ini dengan
      state machine (mirip TX_Process() di STM32) yang jalan per-iterasi
      loop(), supaya handleUART() tetap responsif menerima frame STM32
      berikutnya (termasuk retry) sambil request cloud sebelumnya masih
      berjalan di background.
   2. Outbox persisten di ESP32 (LittleFS): simpan {seq, json, status:
      "sending"/"acked"} SEBELUM memanggil transport, supaya kalau ESP32
      restart di tengah proses (brownout - lihat logResetReason(), ESP32
      disebut rawan brownout reset saat GSM aktif), status "sedang
      mengirim seq X" tidak hilang - saat boot ESP32 bisa cek status
      terakhir sebelum memutuskan mengirim ulang atau tidak.
   3. WAJIB, terlepas dari 2 poin di atas: Cloud HARUS idempotent
      berdasarkan (device_id, seq) - tolak/abaikan seq yang sudah pernah
      diterima. Ini satu-satunya cara menutup celah duplikat SECARA
      TUNTAS, karena "exactly-once delivery" di jaringan yang tidak
      reliable pada dasarnya mustahil dijamin hanya dari sisi pengirim;
      yang bisa dicapai realistis adalah at-least-once + idempotent
      receiver = effectively-once.
   ============================================================================ */
bool sendByTransport(uint32_t seq, const char* jsonLine) {

  if (isMode("gsm")) {
    return gsmSendJSONWithSeq(seq, jsonLine);
  }
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

// ================== STM32 HELPERS ====================
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

  // Frame valid (SOF + CRC OK) -> beri tahu watchdog komunikasi STM32.
  StmComm_NotifyValidFrame();
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
  } else if (type == TYPE_READY) {
    LOGI("[SYNC] READY diterima dari STM32");
    stmReadyReceived = true;
  } else if (type == TYPE_RULES_ACK) {
    uint8_t count = 0;
    size_t i = 0;
    while (i + 2 <= payloadLen) {
      uint8_t tag = payload[i];
      uint8_t l = payload[i + 1];
      if (i + 2 + l > payloadLen) break;
      if (tag == TAG_RULE_COUNT && l == 1) {
        count = payload[i + 2];
      }
      i += 2 + l;
    }
    LOGF("[SYNC] RULES_ACK diterima dari STM32, count=%d\n", count);
    rulesAckCount = count;
    rulesAckReceived = true;
  }
}

void handleDataFromSTM(uint8_t* tlv, size_t len) {
  uint32_t seq = 0;
  uint32_t ts = 0;
  int16_t temp = 0, hum = 0;
  uint32_t digMask = 0;
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
        if (l == 4) digMask = rd32(&tlv[i]);
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
  if (moduleId != MOD_RTC && moduleId != MOD_DHT && moduleId != MOD_ADS && moduleId != MOD_SD) {
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

  const uint8_t payloadLen = 6;  // 1 (tag) + 1 (len) + 4 (value)

  frame[idx++] = SOF_BYTE;
  frame[idx++] = TYPE_ACK;
  frame[idx++] = (payloadLen >> 8) & 0xFF;
  frame[idx++] = payloadLen & 0xFF;

  frame[idx++] = 0x01;  // TAG_SEQ
  frame[idx++] = 0x04;  // LEN = 4 byte

  frame[idx++] = (seq >> 24) & 0xFF;
  frame[idx++] = (seq >> 16) & 0xFF;
  frame[idx++] = (seq >> 8) & 0xFF;
  frame[idx++] = seq & 0xFF;

  uint16_t crc = CRC16_Modbus(frame, idx);
  frame[idx++] = crc >> 8;
  frame[idx++] = crc & 0xFF;

  SerialSTM.flush();
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

/* rulesSyncTask() - dipanggil tiap loop(), non-blocking.
 * Menggantikan pemanggilan langsung sendRulesToSTM32() dari initFS().
 * Lihat komentar di deklarasi RulesSyncState_t utk penjelasan alur. */
void rulesSyncTask() {
  uint32_t now = millis();

  switch (rulesSyncState) {
    case RS_WAIT_STM_READY:
      if (stmReadyReceived) {
        LOGI("[SYNC] STM32 READY diterima, lanjut kirim Rules");
        rulesSyncState = RS_SENDING;
      } else if (now - rulesSyncStateEnteredAt > RULES_STM_READY_FALLBACK_MS) {
        // READY tak kunjung datang - kemungkinan besar STM32 sudah lama
        // hidup sebelum ESP32 ini boot/restart (mis. lewat tombol Save
        // Config, STM32 tidak mengulang kirim READY). Tetap coba kirim;
        // kalaupun STM32 ternyata belum siap, mekanisme retry-by-ACK di
        // bawah akan menutupinya.
        LOGW("[SYNC] Timeout menunggu READY dari STM32, coba kirim Rules tanpa handshake");
        rulesSyncState = RS_SENDING;
      }
      break;

    case RS_SENDING:
      rulesSyncLastSentCount = config.rules.count;
      sendRulesToSTM32();
      rulesAckReceived = false;
      rulesSyncStateEnteredAt = now;
      rulesSyncState = RS_WAIT_ACK;
      break;

    case RS_WAIT_ACK:
      if (rulesAckReceived) {
        if (rulesAckCount == rulesSyncLastSentCount) {
          LOGF("[SYNC] Rules terkonfirmasi diterapkan STM32, count=%d\n", rulesAckCount);
          rulesSyncState = RS_CONFIRMED;
        } else {
          LOGF("[SYNC] Jumlah rule tidak cocok (kirim=%d, ack=%d), retry\n",
               rulesSyncLastSentCount, rulesAckCount);
          rulesSyncRetryCount++;
          rulesSyncState = RS_SENDING;
        }
      } else if (now - rulesSyncStateEnteredAt > RULES_ACK_TIMEOUT_MS) {
        rulesSyncRetryCount++;
        if (rulesSyncRetryCount <= RULES_SYNC_MAX_LOG_RETRY) {
          LOGF("[SYNC] ACK timeout, retry ke-%d\n", rulesSyncRetryCount);
        }
        // Retry TANPA BATAS - target: selalu tersinkron otomatis tanpa
        // perlu buka BLE/tekan Save Config manual. Interval retry sudah
        // dibatasi RULES_ACK_TIMEOUT_MS (2s), cukup jarang utk tidak
        // membanjiri UART tapi cukup sering utk cepat pulih.
        rulesSyncState = RS_SENDING;
      }
      break;

    case RS_CONFIRMED:
    default:
      break;
  }
}

uint32_t parseDateTimeToEpoch(const String& dt) {

  struct tm t {};
  int n = sscanf(dt.c_str(),
                 "%d-%d-%d %d:%d:%d",
                 &t.tm_year,
                 &t.tm_mon,
                 &t.tm_mday,
                 &t.tm_hour,
                 &t.tm_min,
                 &t.tm_sec);

  if (n == 5) {

    t.tm_sec = 0;
  } else if (n != 6) {
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


void sendAddonMaskToSTM32(uint8_t mask) {
  uint8_t tlv[3];
  size_t idx = 0;

  tlv[idx++] = TAG_ADDON_MASK;
  tlv[idx++] = 1;  // len = 1 byte
  tlv[idx++] = mask;

  uint8_t frame[16];
  uint16_t fidx = 0;

  frame[fidx++] = SOF_BYTE;
  frame[fidx++] = TYPE_ADDON;
  frame[fidx++] = 0x00;
  frame[fidx++] = idx;

  memcpy(&frame[fidx], tlv, idx);
  fidx += idx;

  uint16_t crc = CRC16_Modbus(frame, fidx);
  frame[fidx++] = crc >> 8;
  frame[fidx++] = crc & 0xFF;

  SerialSTM.write(frame, fidx);
  LOGF("[ADDON] mask sent to STM32: 0x%02X\n", mask);
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

void buildDigArray(uint32_t digMask, char* out, size_t len) {
  size_t pos = 0;
  pos += snprintf(out + pos, len - pos, "[");

  int totalInputs = isAddon("ekspansi_input") ? 19 : 11;

  for (int i = 0; i < totalInputs; i++) {
    int bit = (digMask >> i) & 1;
    pos += snprintf(out + pos, len - pos,
                    "%d%s",
                    bit,
                    (i < totalInputs - 1) ? "," : "");
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

// =============== INPUT/OUTPUT HELPERS ================
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
      LOGI("[BLE] Long Press Detected");
      handleLongPress();
    }
  }
}

void handleLongPress() {
  if (bleState == BLE_OFF) {
    startBLE();
  } else {
    LOGI("[BLE] Long press diabaikan, BLE state saat ini = " + String((int)bleState));
  }
}

// =================== EVENT HELPERS ===================
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

// ======================== BLE ========================
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

// ======================= SETUP =======================
// ================== ADDON: EKSPANSI INPUT (PCF8574) ==================
uint8_t addonReadInputMask() {
  uint8_t data = 0;
  uint8_t mask = 0;

  Wire.requestFrom(PCF8574_ADDR, 1);
  if (Wire.available()) {
    data = Wire.read();
    for (uint8_t i = 0; i < 8; i++) {
      if ((data & (1 << i)) == 0) {
        // LOW = AKTIF
        mask |= (1 << i);
      }
    }
  }
  return mask;
}

void addonExpansionInit() {
  LOGI("[ADDON] Initializing Ekspansi Input (PCF8574)...");
  Wire.begin(PIN_ADDON_SDA, PIN_ADDON_SCL);

  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();

  addonLastPoll = 0;
  addonRawMaskLast = addonReadInputMask();
  addonLastChangeMs = millis();
  LOGI("[ADDON] PCF8574 Input ready");
}

void addonExpansionLoop() {
  unsigned long now = millis();
  if (now - addonLastPoll < ADDON_POLL_INTERVAL_MS) return;
  addonLastPoll = now;

  uint8_t rawMask = addonReadInputMask();

  if (rawMask != addonRawMaskLast) {
    addonRawMaskLast = rawMask;
    addonLastChangeMs = now;
  }


  if ((rawMask == addonInputMask) && addonMaskSentOnce) return;
  if ((now - addonLastChangeMs) < ADDON_DEBOUNCE_MS) return;

  String bin = "";
  for (int i = 7; i >= 0; i--) {
    bin += ((rawMask >> i) & 0x01) ? "1" : "0";
  }
  LOGF("[ADDON] Mask stabil: 0x%02X  BIN: %s\n", rawMask, bin.c_str());

  if (rawMask != addonInputMask || !addonMaskSentOnce) {
    addonInputMask = rawMask;
    sendAddonMaskToSTM32(addonInputMask);
    addonMaskSentOnce = true;
  }
}

void initAddon() {
  if (isAddon("gsm")) {
    LOGI("[ADDON] type=gsm");
  } else if (isAddon("ekspansi_input")) {
    LOGI("[ADDON] type=ekspansi_input");
    addonExpansionInit();
  } else {
    LOGI("[ADDON] type=none");
  }
}

void initGPIO() {
  delay(200);
  LOGI("\n=== BOOT ===");

  SerialSTM.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_RUN, OUTPUT);
  pinMode(LED_ERR, OUTPUT);

  ledWrite(LED_RUN, false);
  ledWrite(LED_ERR, false);

  startBootBlink();

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
      // Rules TIDAK dikirim langsung di sini lagi (sebelumnya: delay(500)
      // lalu sendRulesToSTM32() sekali, fire-and-forget, tanpa ACK/retry -
      // inilah akar penyebab race saat STM32+ESP32 boot bersamaan).
      // Sekarang cukup aktifkan state machine; pengiriman sesungguhnya
      // menunggu handshake TYPE_READY dari STM32 (atau fallback timeout),
      // lalu retry otomatis sampai ACK jumlah rule-nya cocok.
      // Lihat rulesSyncTask(), dipanggil tiap loop().
      rulesSyncState = RS_WAIT_STM_READY;
      rulesSyncStateEnteredAt = millis();
      stmReadyReceived = false;
      LOGI("[SYNC] Rules sync state machine diaktifkan, menunggu STM32 READY");
    }
  }
}

void startNetworkByConfig() {
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

  // ================== GSM (add-on module) ==================
  else if (isMode("gsm")) {
    LOGI("[CFG] network mode: gsm");

    if (!isAddon("gsm")) {
      LOGE("[ADDON] network mode=gsm tapi addon.type bukan gsm, GSM TIDAK diinisialisasi");
    } else {
      gsmInit();
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
}

void initNetwork() {
  WiFi.mode(WIFI_OFF);
  delay(200);

  netState = NET_DOWN;

  startNetworkByConfig();

  LOGI("=== SETUP DONE ===");

  stopBootBlink();
  sysReady = true;
}

void startBLE() {
  if (bleState != BLE_OFF) {
    LOGW("[BLE] startBLE() dipanggil saat bleState != BLE_OFF, diabaikan");
    return;
  }

  bleState = BLE_STARTING;
  LOGI("[BLE] Initializing...");

  String bleName = buildBleName();
  LOGI("[BLE] init name = " + bleName);

  BLEDevice::init(bleName.c_str());
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(&bleServerCallbacksInstance);

  bleGattService = bleServer->createService(BLE_SERVICE_UUID);

  bleRxChar = bleGattService->createCharacteristic(
    BLE_CHAR_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);
  bleRxChar->setCallbacks(&bleRxCallbacksInstance);

  bleTxChar = bleGattService->createCharacteristic(
    BLE_CHAR_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  bleTxChar->addDescriptor(new BLE2902());

  bleGattService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();

  bleClientConnected = false;
  bleTransferReset();
  bleRxReset();

  bleState = BLE_RUNNING;
  LOGI("[BLE] Advertising Started");
}

void stopBLE() {
  if (bleState == BLE_OFF) {
    LOGW("[BLE] stopBLE() dipanggil saat sudah BLE_OFF, diabaikan");
    return;
  }

  bleState = BLE_STOPPING;

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  if (adv) adv->stop();
  LOGI("[BLE] Stop Advertising");

  bleTransferReset();
  bleRxReset();
  bleClientConnected = false;

  bleServer = nullptr;
  bleGattService = nullptr;
  bleRxChar = nullptr;
  bleTxChar = nullptr;

  BLEDevice::deinit(true);

  bleState = BLE_OFF;
  LOGI("[BLE] Deinitialized");

  delay(200);
}

void logResetReason() {

  esp_reset_reason_t reason = esp_reset_reason();
  const char* reasonStr;
  switch (reason) {
    case ESP_RST_POWERON: reasonStr = "POWERON (nyala normal dari mati total)"; break;
    case ESP_RST_EXT: reasonStr = "EXTERNAL (reset pin dari luar)"; break;
    case ESP_RST_SW: reasonStr = "SOFTWARE (ESP.restart() dipanggil kode)"; break;
    case ESP_RST_PANIC: reasonStr = "PANIC (crash/exception firmware)"; break;
    case ESP_RST_INT_WDT: reasonStr = "INTERRUPT WATCHDOG (ISR terlalu lama)"; break;
    case ESP_RST_TASK_WDT: reasonStr = "TASK WATCHDOG (loop() terlalu lama tanpa yield)"; break;
    case ESP_RST_WDT: reasonStr = "OTHER WATCHDOG"; break;
    case ESP_RST_BROWNOUT: reasonStr = "BROWNOUT - suplai daya drop (KEMUNGKINAN BESAR penyebab restart saat GSM disambungkan - lihat komentar di logResetReason())"; break;
    case ESP_RST_SDIO: reasonStr = "SDIO"; break;
    default: reasonStr = "UNKNOWN/lainnya"; break;
  }
  LOGI("[BOOT] Reset reason: " + String(reasonStr));
}

void setup() {
  Serial.begin(115200);
  logResetReason();
  initGPIO();
  initFS();
  initAddon();
  initNetwork();
}

// ======================= LOOP ========================
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
    updateStatusLeds();
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
      if (frameIdx >= MAX_FRAME) {
        frameIdx = 0;
        expectedLength = -1;
        continue;
      }
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
  bleTransferTask();
  bleRxTimeoutCheck();

  if (needRestart) {
    if (bleTransferBusy()) return;
    LOGI("[SYS] restarting to apply new config");
    delay(1200);
    ESP.restart();
  }
}

void handleGSM() {

  if (!isMode("gsm") || !isAddon("gsm")) return;

  gsmDetectLoop();
  gsmNetworkLoop();
}

void handleAddonExpansion() {
  if (!isAddon("ekspansi_input")) return;
  addonExpansionLoop();
}

void loop() {
  handleButton();
  handleHybridNetwork();
  handleWebSocket();
  handleGSM();
  handleAddonExpansion();
  handleUART();
  rulesSyncTask();
  handleBLE();
  updateLedErrorFlags();
  updateStatusLeds();
  yield();
}