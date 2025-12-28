// ================== INCLUDES ==================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LittleFS.h>
#include <WebServer.h>
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

// ================== PIN CONFIG ==================
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18
#define PIN_CS_SD 5
#define PIN_CS_W5500 2
#define PIN_W5500_RST 4

// ================== BUTTON ==================
#define BTN_PIN 32  // active LOW
#define BTN_LONG_MS 3000

// ================== UART STM32 ==================
#define RXD2 16
#define TXD2 17

// ================== STM32 FRAME ==================
#define SOF_BYTE 0x7E
#define MAX_PAYLOAD 240
#define MAX_FRAME 256

// ===== UART DATA =====
#define MAX_RULES 10
#define MAX_RULE_LEN 16
#define TYPE_DATA 0x01
#define TYPE_ACK 0x02
#define TYPE_RULES 0x03
#define TYPE_RTC 0x04
#define TAG_RULE 0x10
#define TAG_RTC_TS 0x20

// ================== FILE PATH ==================
#define FILE_CONFIG "/config.json"

// ================== HELPERS ==================
#define SENDWAIT_INTERVAL_MS 5000
#define ACK_TIMEOUT_MS 3000
#define AP_TIMEOUT_MS 150000
#define HEARTBEAT_MS 15000
#define ETH_LINK_TIMEOUT_MS 10000
#define ETH_DHCP_RETRY_MS 1000
#define BTN_DEBOUNCE_MS 40

// ================== AP CONFIG ==================
char apSSID[32];
const char* AP_PASS = "12345678";

// ================== GLOBAL OBJECTS ==================
HardwareSerial SerialSTM(2);
WebServer server(80);
WebSocketsClient ws;
PubSubClient mqttClient;
bool mqttConnected = false;
bool btnStableState = HIGH;
bool btnLastRead = HIGH;
unsigned long btnLastChange = 0;

// ================== NETWORK ==================
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
enum NetState {
  NET_DOWN,
  NET_UP,
  NET_AP
};
NetState netState = NET_DOWN;

// ================== WEBSOCKET STATE ==================
bool wsConnected = false;
volatile bool wsAckOk = false;
volatile uint32_t wsAckSeq = 0;

// ================== TIMERS ==================
unsigned long lastSendMs = 0;
unsigned long apStartMs = 0;

// ================== BUTTON STATE ==================
bool btnPressed = false;
bool longActionDone = false;
unsigned long btnPressStart = 0;

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
// ==================== SD HELPERS =====================
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
    doc["network"]["priority"] | "ethernet";

  // ---------------- TRANSPORT ----------------
  config.transport.type =
    doc["transport"]["type"] | "mqtt";
  config.transport.host =
    doc["transport"]["host"] | "";
  config.transport.port =
    doc["transport"]["port"] | 1883;
  config.transport.path =
    doc["transport"]["path"] | "/";

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

  return true;
}

// =====================================================
// =================== NET HELPERS =====================
// =====================================================
void buildAPSSID() {
  snprintf(
    apSSID,
    sizeof(apSSID),
    "IoT-Node-%02d",
    config.device.boxID);

  LOG("[CFG] AP SSID set to: ");
  LOGI(apSSID);
}

bool startEthernet(uint32_t timeoutMs = ETH_LINK_TIMEOUT_MS) {

  Ethernet.init(PIN_CS_W5500);

  // ---------- 1. CEK LINK ----------
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

  if (Ethernet.linkStatus() != LinkON) {
    LOGI("[NET] Ethernet cable not detected");
    return false;
  }

  // ---------- 2. DHCP ----------
  LOGI("[NET] requesting DHCP...");
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
}

bool tryEthernet() {
  digitalWrite(PIN_CS_SD, HIGH);
  LOGI("[NET] try Ethernet...");

  if (startEthernet()) {
    netState = NET_UP;
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

    if (strcasecmp(config.transport.type.c_str(), "ws") == 0) {
      startWebSocket();
    }
    return true;
  }

  LOGI("[NET] WiFi FAIL");
  return false;
}

void startAPMode() {
  buildAPSSID();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, AP_PASS);

  LOG("[AP] IP: ");
  LOGI(WiFi.softAPIP());

  // server.on("/", handleRoot);
  // server.on("/config", handleGetConfig);
  // server.on("/save", handleSaveConfig);
  // server.on("/download", handleDownloadConfig);
  server.on("/setrtc", handleSetRTC);
  server.begin();

  apStartMs = millis();
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
  // contoh: "iot/data/box01"

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

  // ---------- TCP ----------
  if (strcasecmp(config.transport.type.c_str(), "tcp") == 0) {
    return tcpSendJSONWithSeq(seq, jsonLine);
  }

  // ---------- HTTP ----------
  if (strcasecmp(config.transport.type.c_str(), "httppost") == 0) {
    return httpPostJSONWithSeq(seq, jsonLine);
  }

  // ---------- MQTT ----------
  if (strcasecmp(config.transport.type.c_str(), "mqtt") == 0) {
    return mqttSendJSONWithSeq(seq, jsonLine);
  }

  // ---------- WS ----------
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

  uint8_t type = buf[1];
  uint16_t payloadLen = ((uint16_t)buf[2] << 8) | buf[3];
  uint8_t* payload = &buf[4];

  if (type == 0x01) {  // DATA
    handleDataFromSTM(payload, payloadLen);
  } else if (type == 0x02) {  // ACK
    LOGI("[UART] ACK from STM");
  }
}

void handleDataFromSTM(uint8_t* tlv, size_t len) {

  uint32_t seq = 0;
  uint32_t ts = 0;
  int16_t temp = 0, hum = 0;
  uint16_t dig = 0, an = 0;
  uint8_t q = 0;

  size_t i = 0;

  while (i + 2 <= len) {
    uint8_t tag = tlv[i++];
    uint8_t l = tlv[i++];

    if (i + l > len) {
      LOGE("[TLV] overflow");
      return;
    }

    switch (tag) {
      case 0x01: memcpy(&seq, &tlv[i], 4); break;
      case 0x02: memcpy(&temp, &tlv[i], 2); break;
      case 0x03: memcpy(&hum, &tlv[i], 2); break;
      case 0x04: memcpy(&dig, &tlv[i], 2); break;
      case 0x05: memcpy(&an, &tlv[i], 2); break;
      case 0x06: memcpy(&q, &tlv[i], 1); break;
      case 0x07: memcpy(&ts, &tlv[i], 4); break;
      default:
        LOGW("[TLV] unknown tag");
        break;
    }

    i += l;
  }

  // ===== BUILD JSON FOR NETWORK ONLY =====
  char json[256];
  snprintf(json, sizeof(json),
           "{\"seq\":%lu,\"timestamp\":%lu,"
           "\"temp\":%.1f,\"hum\":%.1f,"
           "\"dig_in\":%u,\"an_in\":%u,\"q\":%u}",
           seq, ts,
           temp / 10.0f, hum / 10.0f,
           dig, an, q);

  LOGI("[UART] DATA OK → SEND");
  LOGI(json);

  if (sendByTransport(seq, json)) {
    sendAckToSTM(seq);
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

void handleSetRTC() {
  if (!server.hasArg("datetime")) {
    server.send(400, "text/plain", "Missing datetime");
    return;
  }

  String dt = server.arg("datetime");  // "YYYY-MM-DD HH:MM:SS"
  LOGI("[RTC] web request: " + dt);

  struct tm tm {};
  strptime(dt.c_str(), "%Y-%m-%d %H:%M:%S", &tm);
  time_t epoch = mktime(&tm);

  sendRTCToSTM32((uint32_t)epoch);

  server.send(200, "text/plain", "RTC update sent to STM");
}

void sendRTCToSTM32(uint32_t epoch) {

  uint8_t tlv[8];
  size_t idx = 0;

  tlv[idx++] = TAG_RTC_TS;
  tlv[idx++] = 4;
  memcpy(&tlv[idx], &epoch, 4);
  idx += 4;

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

  LOGI("[RTC] epoch sent to STM = " + String(epoch));
}

// =====================================================
// =============== INPUT/OUTPUT HELPERS ================
// =====================================================
void handleButton() {
  unsigned long now = millis();
  bool readNow = digitalRead(BTN_PIN);
  // ---------- DEBOUNCE ----------
  if (readNow != btnLastRead) {
    btnLastChange = now;
    btnLastRead = readNow;
  }

  if ((now - btnLastChange) < BTN_DEBOUNCE_MS) {
    return;  // tunggu sampai stabil
  }

  // ---------- STATE STABIL ----------
  if (readNow != btnStableState) {
    btnStableState = readNow;

    // ===== BUTTON DITEKAN =====
    if (btnStableState == LOW) {
      btnPressed = true;
      btnPressStart = now;
      longActionDone = false;
    }

    // ===== BUTTON DILEPAS =====
    else {
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

  // ---------- LONG PRESS ----------
  if (btnPressed && !longActionDone) {
    if (now - btnPressStart >= BTN_LONG_MS) {
      longActionDone = true;

      LOGI("BTN long press → AP MODE");

      if (netState != NET_AP) {
        netState = NET_AP;
        startAPMode();
      }
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
      wsAckSeq = ackSeq;  // reuse variable
      wsAckOk = true;
    }
  }
}

// =====================================================
// ======================= SETUP =======================
// =====================================================
void initGPIO() {
  delay(200);
  LOGI("\n=== BOOT ===");

  SerialSTM.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(BTN_PIN, INPUT_PULLUP);

  // ---------------- GPIO CS ----------------
  pinMode(PIN_CS_SD, OUTPUT);
  pinMode(PIN_CS_W5500, OUTPUT);
  digitalWrite(PIN_CS_W5500, HIGH);

  // ---------------- RESET W5500 ----------------
  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(200);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(500);

  // ---------------- SPI INIT ----------------
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
      LOGI("[NET] Ethernet unavailable → AP MODE");
      netState = NET_AP;
      startAPMode();
    }
  }

  // ================== WIFI ONLY ==================
  else if (isMode("wifi")) {
    LOGI("[CFG] network mode: wifi");

    if (!tryWiFi()) {
      LOGI("[NET] WiFi unavailable → AP MODE");
      netState = NET_AP;
      startAPMode();
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
      LOGI("[NET] hybrid failed → AP MODE");
      netState = NET_AP;
      startAPMode();
    }
  }

  // ================== UNKNOWN ==================
  else {
    LOGI("[CFG] unknown network mode → AP MODE");
    netState = NET_AP;
    startAPMode();
  }

  LOGI("=== SETUP DONE ===");
}

void setup() {
  Serial.begin(115200);
  initGPIO();
  initFS();
  initNetwork();
}

// =====================================================
// ======================= LOOP ========================
// =====================================================
void handleWebSocket() {
  if (netState == NET_UP && isMode("wifi") && strcasecmp(config.transport.type.c_str(), "ws") == 0) {
    ws.loop();
  }
}

void handleAP() {
  if (netState == NET_AP) {
    server.handleClient();

    if (apStartMs != 0 && millis() - apStartMs >= AP_TIMEOUT_MS) {
      LOGI("[AP] timeout, reboot...");
      delay(100);
      ESP.restart();
    }
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

void loop() {
  handleButton();
  handleWebSocket();
  handleAP();
  handleUART();
  yield();
}