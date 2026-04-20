#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// =====================================================
// PRO VERSION - Water Quality Site Node
// Board: ES30485 / ESP32 Dev
// 1 site = 1 ESP32
// slaveid1 = DO + Water Temp
// slaveid2 = pH + EC + Water Temp
// MQTT JSON per site
// =====================================================

// =========================
// Fixed board pins
// =========================
#define RS485_RX_PIN 3
#define RS485_TX_PIN 1
#define RS485_DE_RE_PIN 17
#define MQTT_LED_PIN 2     // LED (Active HIGH)
#define MQTT_RELAY_PIN 18  // Relay IN1 (Active LOW)
#define RESET_BUTTON_PIN 0

// =========================
// Change this default per device if needed
// =========================
#define DEFAULT_SITE_ID "site01"

// =========================
// WiFiManager AP
// =========================
#define AP_NAME "ESP32-WaterQuality"
#define AP_PASS "12345678"

// =========================
// Timing
// =========================
const unsigned long SENSOR_READ_INTERVAL_MS = 5000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
const unsigned long STATUS_INTERVAL_MS = 30000;
const unsigned long SENSOR_GAP_MS = 200;
const unsigned long BUTTON_HOLD_MS = 5000;

// =========================
// NTP (Thailand)
// =========================
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";
const long GMT_OFFSET_SEC = 7 * 3600;
const int DAYLIGHT_OFFSET_SEC = 0;

// =========================
// Preferences
// =========================
Preferences prefs;

// =========================
// Network / MQTT
// =========================
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wm;

// =========================
// Modbus
// =========================
HardwareSerial& modbusSerial = Serial;  // ES30485 fixed UART0
ModbusMaster node1;                     // DO + temp
ModbusMaster node2;                     // pH + EC + temp

// =========================
// Config
// =========================
char site_id[20] = DEFAULT_SITE_ID;
char mqtt_server[40] = "";
char mqtt_port[6] = "1883";
char mqtt_user[32] = "";
char mqtt_pass[32] = "";

// =========================
// WiFiManager Parameters
// =========================
WiFiManagerParameter param_site_id("site_id", "Site ID", DEFAULT_SITE_ID, 20);
WiFiManagerParameter param_mqtt_server("mqtt_server", "MQTT Server IP", "", 40);
WiFiManagerParameter param_mqtt_port("mqtt_port", "MQTT Port", "1883", 6);
WiFiManagerParameter param_mqtt_user("mqtt_user", "MQTT Username", "", 32);
WiFiManagerParameter param_mqtt_pass("mqtt_pass", "MQTT Password", "", 32);

// =========================
// Topics
// =========================
char topicData[80];
char topicStatus[80];
char topicLWT[80];

// =========================
// Runtime
// =========================
unsigned long lastSensorReadMs = 0;
unsigned long lastMqttRetryMs = 0;
unsigned long lastStatusMs = 0;

// =========================
// Sensor Data
// =========================
struct SensorData {
  float doValue = NAN;
  float doTemp = NAN;

  float phValue = NAN;
  float ecValue = NAN;
  float phTemp = NAN;

  bool slave1_ok = false;
  bool slave2_ok = false;

  uint32_t read_count = 0;
  uint32_t fail_slave1 = 0;
  uint32_t fail_slave2 = 0;
  unsigned long last_success_ms = 0;
} sensorData;

// =====================================================
// Utility
// =====================================================
String getDeviceId() {
  uint64_t chipid = ESP.getEfuseMac();
  char id[24];
  snprintf(id, sizeof(id), "ESP32-%04X%08X",
           (uint16_t)(chipid >> 32),
           (uint32_t)chipid);
  return String(id);
}

String getISOTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1000)) {
    return "";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
  return String(buf);
}

void buildTopics() {
  snprintf(topicData, sizeof(topicData), "waterq/v1/%s/data", site_id);
  snprintf(topicStatus, sizeof(topicStatus), "waterq/v1/%s/status", site_id);
  snprintf(topicLWT, sizeof(topicLWT), "waterq/v1/%s/lwt", site_id);
}

// =====================================================
// OUTPUT CONTROL
// =====================================================
void setMqttOutputs(bool connected) {
  digitalWrite(MQTT_LED_PIN, connected ? HIGH : LOW);     // LED Active HIGH
  digitalWrite(MQTT_RELAY_PIN, connected ? LOW : HIGH);   // Relay Active LOW
}

// =====================================================
// Preferences
// =====================================================
void loadConfig() {
  prefs.begin("waterq", true);

  String s_site = prefs.getString("site_id", DEFAULT_SITE_ID);
  String s_srv = prefs.getString("mqtt_server", "");
  String s_port = prefs.getString("mqtt_port", "1883");
  String s_user = prefs.getString("mqtt_user", "");
  String s_pass = prefs.getString("mqtt_pass", "");

  prefs.end();

  s_site.toCharArray(site_id, sizeof(site_id));
  s_srv.toCharArray(mqtt_server, sizeof(mqtt_server));
  s_port.toCharArray(mqtt_port, sizeof(mqtt_port));
  s_user.toCharArray(mqtt_user, sizeof(mqtt_user));
  s_pass.toCharArray(mqtt_pass, sizeof(mqtt_pass));

  buildTopics();
}

void saveConfig() {
  prefs.begin("waterq", false);
  prefs.putString("site_id", site_id);
  prefs.putString("mqtt_server", mqtt_server);
  prefs.putString("mqtt_port", mqtt_port);
  prefs.putString("mqtt_user", mqtt_user);
  prefs.putString("mqtt_pass", mqtt_pass);
  prefs.end();
}

// =====================================================
// Reset button
// Hold GPIO0 low on boot for 5 sec to clear config
// =====================================================
void checkResetButtonOnBoot() {
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    unsigned long startMs = millis();

    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
      if (millis() - startMs >= BUTTON_HOLD_MS) {
        wm.resetSettings();

        prefs.begin("waterq", false);
        prefs.clear();
        prefs.end();

        for (int i = 0; i < 10; i++) {
          digitalWrite(MQTT_LED_PIN, !digitalRead(MQTT_LED_PIN));
          delay(150);
        }
        setMqttOutputs(false);
        ESP.restart();
      }
      delay(50);
    }
  }
}

// =====================================================
// RS485 direction control
// =====================================================
void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(200);
}

void postTransmission() {
  delayMicroseconds(200);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// =====================================================
// NTP
// =====================================================
void initTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
}

// =====================================================
// MQTT
// =====================================================
void applyMqttConfig() {
  uint16_t port = atoi(mqtt_port);
  if (port == 0) port = 1883;

  mqttClient.setServer(mqtt_server, port);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
}

bool connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    setMqttOutputs(false);
    return false;
  }

  if (strlen(mqtt_server) == 0) {
    setMqttOutputs(false);
    return false;
  }

  String clientId = String(site_id) + "-" + getDeviceId();
  bool ok = false;

  if (strlen(mqtt_user) > 0) {
    ok = mqttClient.connect(
      clientId.c_str(),
      mqtt_user,
      mqtt_pass,
      topicLWT,
      1,
      true,
      "offline");
  } else {
    ok = mqttClient.connect(
      clientId.c_str(),
      topicLWT,
      1,
      true,
      "offline");
  }

  if (ok) {
    setMqttOutputs(true);
    mqttClient.publish(topicLWT, "online", true);
  } else {
    setMqttOutputs(false);
  }

  return ok;
}

// =====================================================
// WiFiManager + Config
// =====================================================
void syncConfigToParams() {
  param_site_id.setValue(site_id, sizeof(site_id));
  param_mqtt_server.setValue(mqtt_server, sizeof(mqtt_server));
  param_mqtt_port.setValue(mqtt_port, sizeof(mqtt_port));
  param_mqtt_user.setValue(mqtt_user, sizeof(mqtt_user));
  param_mqtt_pass.setValue(mqtt_pass, sizeof(mqtt_pass));
}

void syncParamsToConfig() {
  strncpy(site_id, param_site_id.getValue(), sizeof(site_id) - 1);
  site_id[sizeof(site_id) - 1] = '\0';

  strncpy(mqtt_server, param_mqtt_server.getValue(), sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0';

  strncpy(mqtt_port, param_mqtt_port.getValue(), sizeof(mqtt_port) - 1);
  mqtt_port[sizeof(mqtt_port) - 1] = '\0';

  strncpy(mqtt_user, param_mqtt_user.getValue(), sizeof(mqtt_user) - 1);
  mqtt_user[sizeof(mqtt_user) - 1] = '\0';

  strncpy(mqtt_pass, param_mqtt_pass.getValue(), sizeof(mqtt_pass) - 1);
  mqtt_pass[sizeof(mqtt_pass) - 1] = '\0';

  saveConfig();
  buildTopics();
  applyMqttConfig();
}

void setupWiFiAndConfig() {
  wm.setConfigPortalTimeout(180);
  wm.setBreakAfterConfig(true);

  syncConfigToParams();

  wm.addParameter(&param_site_id);
  wm.addParameter(&param_mqtt_server);
  wm.addParameter(&param_mqtt_port);
  wm.addParameter(&param_mqtt_user);
  wm.addParameter(&param_mqtt_pass);

  bool ok = wm.autoConnect(AP_NAME, AP_PASS);
  if (!ok) {
    ESP.restart();
  }

  syncParamsToConfig();
}

void reconnectWiFiIfNeeded() {
  if (WiFi.status() == WL_CONNECTED) return;

  setMqttOutputs(false);
  mqttClient.disconnect();

  syncConfigToParams();

  bool ok = wm.autoConnect(AP_NAME, AP_PASS);
  if (!ok) return;

  syncParamsToConfig();
  initTime();
}

// =====================================================
// Publish
// =====================================================
void publishStatus(const char* stateText) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<512> doc;
  doc["site_id"] = site_id;
  doc["device_id"] = getDeviceId();
  doc["state"] = stateText;
  doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["mqtt_connected"] = mqttClient.connected();
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_ms"] = millis();
  doc["time"] = getISOTime();

  JsonObject s1 = doc.createNestedObject("slaveid1");
  s1["ok"] = sensorData.slave1_ok;

  JsonObject s2 = doc.createNestedObject("slaveid2");
  s2["ok"] = sensorData.slave2_ok;

  JsonObject stats = doc.createNestedObject("stats");
  stats["read_count"] = sensorData.read_count;
  stats["fail_slave1"] = sensorData.fail_slave1;
  stats["fail_slave2"] = sensorData.fail_slave2;
  stats["last_success_ms"] = sensorData.last_success_ms;

  char payload[512];
  size_t len = serializeJson(doc, payload);
  mqttClient.publish(topicStatus, (const uint8_t*)payload, len, true);
}

void publishData() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<768> doc;
  doc["site_id"] = site_id;
  doc["device_id"] = getDeviceId();
  doc["time"] = getISOTime();
  doc["uptime_ms"] = millis();
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();

  JsonObject s1 = doc.createNestedObject("slaveid1");
  s1["type"] = "do_temp";
  s1["ok"] = sensorData.slave1_ok;
  if (sensorData.slave1_ok) {
    s1["do"] = sensorData.doValue;
    s1["wtemp"] = sensorData.doTemp;
  }

  JsonObject s2 = doc.createNestedObject("slaveid2");
  s2["type"] = "ph_ec_temp";
  s2["ok"] = sensorData.slave2_ok;
  if (sensorData.slave2_ok) {
    s2["ph"] = sensorData.phValue;
    s2["ec"] = sensorData.ecValue;
    s2["wtemp"] = sensorData.phTemp;
  }

  JsonObject stats = doc.createNestedObject("stats");
  stats["read_count"] = sensorData.read_count;
  stats["fail_slave1"] = sensorData.fail_slave1;
  stats["fail_slave2"] = sensorData.fail_slave2;
  stats["last_success_ms"] = sensorData.last_success_ms;

  char payload[768];
  size_t len = serializeJson(doc, payload);
  mqttClient.publish(topicData, (const uint8_t*)payload, len, true);
}

// =====================================================
// Modbus helper
// =====================================================
bool readHoldingWithRetry(ModbusMaster& node, uint16_t startAddr, uint16_t qty, uint8_t retries = 3) {
  for (uint8_t i = 0; i < retries; i++) {
    uint8_t result = node.readHoldingRegisters(startAddr, qty);
    if (result == node.ku8MBSuccess) {
      return true;
    }
    delay(150);
  }
  return false;
}

// =====================================================
// Sensor reads
// slaveid1 = DO + WTemp
// reg0 = DO, reg1 = WTemp
// =====================================================
bool readSlave1_DO(float& doVal, float& tempVal) {
  bool ok = readHoldingWithRetry(node1, 0x0000, 2, 3);
  if (!ok) return false;

  uint16_t reg0 = node1.getResponseBuffer(0);
  uint16_t reg1 = node1.getResponseBuffer(1);

  doVal = reg0 / 100.0f;
  tempVal = reg1 / 10.0f;
  return true;
}

// =====================================================
// slaveid2 = pH + EC + Temp
// 0x0000 pH   /100
// 0x0001 EC   /1
// 0x0002 Temp /10
// =====================================================
bool readSlave2_PHEC(float& phVal, float& ecVal, float& tempVal) {
  bool ok = readHoldingWithRetry(node2, 0x0000, 3, 3);
  if (!ok) return false;

  uint16_t regPH = node2.getResponseBuffer(0);
  uint16_t regEC = node2.getResponseBuffer(1);
  uint16_t regTemp = node2.getResponseBuffer(2);

  phVal = regPH / 100.0f;
  ecVal = regEC * 1.0f;
  tempVal = regTemp / 10.0f;
  return true;
}

// =====================================================
// Setup
// =====================================================
void setup() {

  pinMode(MQTT_LED_PIN, OUTPUT);
  pinMode(MQTT_RELAY_PIN, OUTPUT);
  pinMode(RS485_DE_RE_PIN, OUTPUT);

  setMqttOutputs(false);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  delay(500);

  loadConfig();
  checkResetButtonOnBoot();

  modbusSerial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  node1.begin(1, modbusSerial);
  node1.preTransmission(preTransmission);
  node1.postTransmission(postTransmission);

  node2.begin(2, modbusSerial);
  node2.preTransmission(preTransmission);
  node2.postTransmission(postTransmission);

  setupWiFiAndConfig();
  initTime();

  if (connectMQTT()) {
    publishStatus("boot_completed");
  } else {
    setMqttOutputs(false);
  }
}

// =====================================================
// Loop
// =====================================================
void loop() {
  unsigned long now = millis();

  reconnectWiFiIfNeeded();

  if (!mqttClient.connected()) {
    setMqttOutputs(false);

    if (now - lastMqttRetryMs >= MQTT_RETRY_INTERVAL_MS) {
      lastMqttRetryMs = now;
      if (connectMQTT()) {
        publishStatus("mqtt_reconnected");
      }
    }
  } else {
    setMqttOutputs(true);
    mqttClient.loop();
  }

  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;

    float doVal, doTemp;
    float phVal, ecVal, phTemp;

    bool ok1 = readSlave1_DO(doVal, doTemp);
    delay(SENSOR_GAP_MS);
    bool ok2 = readSlave2_PHEC(phVal, ecVal, phTemp);

    sensorData.read_count++;
    sensorData.slave1_ok = ok1;
    sensorData.slave2_ok = ok2;

    if (ok1) {
      sensorData.doValue = doVal;
      sensorData.doTemp = doTemp;
    } else {
      sensorData.fail_slave1++;
    }

    if (ok2) {
      sensorData.phValue = phVal;
      sensorData.ecValue = ecVal;
      sensorData.phTemp = phTemp;
    } else {
      sensorData.fail_slave2++;
    }

    if (ok1 || ok2) {
      sensorData.last_success_ms = now;
    }

    if (mqttClient.connected()) {
      publishData();

      if (ok1 && ok2) {
        publishStatus("read_ok");
      } else if (ok1 || ok2) {
        publishStatus("read_partial");
      } else {
        publishStatus("read_failed");
      }
    }
  }

  if (mqttClient.connected() && (now - lastStatusMs >= STATUS_INTERVAL_MS)) {
    lastStatusMs = now;
    publishStatus("heartbeat");
  }
}
