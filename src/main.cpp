#include <Arduino.h>
#include <ESPAsync_WiFiManager.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#include "main.h"
#include "configuration.h"

AsyncWebServer webServer(80);
DNSServer dnsServer;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
    Serial.println("[SERIAL] Serial started with baud rate: " + String(BAUD_RATE));

    pin_setup();
    wifi_setup();
    mqtt_setup();

    mqtt_connect();
}

void loop() {
}

/**
 * @brief Pin setup
 */
void pin_setup() {
    Serial.println("[PIN] Start pin setup process");

    // Builtin LED
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("[PIN] Setup pin: LED_BUILTIN as OUTPUT");
    // Wifi Reset Pin
    pinMode(GPIO_NUM_16, INPUT);
    Serial.println("[PIN] Setup pin: GPIO_NUM_16 as INPUT");

    Serial.println("[PIN] Pin setup process finished");
}

/**
 * @brief WiFi setup
 */
void wifi_setup() {
    Serial.println("[WIFI] Connecting to WiFi");
    Serial.print("[WIFI] WiFiManager: ");
    Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);
    ESPAsync_WiFiManager wifiManager = ESPAsync_WiFiManager(&webServer, &dnsServer, "NodeMCU-ESP32-S");
    if (digitalRead(GPIO_NUM_16) == HIGH) {
        Serial.println("[WIFI] Wifi Reset Pin is HIGH, reset WiFi config");
        wifiManager.resetSettings();
    }
    wifiManager.autoConnect("NodeMCU-ESP32-S");
    if (WiFiClass::status() == WL_CONNECTED) {
        Serial.println("[WIFI] Connected to WiFi");
        Serial.println("    - SSID: " + WiFi.SSID());
        Serial.println("    - IP Address: " + WiFi.localIP().toString());
    }
    else {
        Serial.print("[WIFI] Failed to connect to WiFi - ");
        Serial.println(wifiManager.getStatus(WiFiClass::status()));
    }
}


/**
 * @brief MQTT setup
 */
void mqtt_setup() {
    Serial.println("[MQTT] Setup MQTT");
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    Serial.println("[MQTT] MQTT setup finished");
}

void mqtt_connect() {
    Serial.println("[MQTT] Connecting to MQTT");
    mqttClient.connect();
}
void onMqttConnect(bool sessionPresent) {
    Serial.println("[MQTT] Connected to MQTT successfully");
    Serial.print("[MQTT] Session present: ");
    Serial.println(sessionPresent);
    uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC, MQTT_TOPIC_QOS);
    Serial.print("[MQTT] Subscribing at QoS " + String(MQTT_TOPIC_QOS) + ", packetId: ");
    Serial.println(packetIdSub);
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("[MQTT] Disconnected from MQTT.");

    if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
        Serial.println("[MQTT] Bad server fingerprint.");
    }

    if (WiFi.isConnected()) {
        Serial.println("[MQTT] Try to reconnect to MQTT in 5 seconds...");
        mqttReconnectTimer.once(MQTT_RECONNECT_TIMEOUT, mqtt_connect);
    }
}
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
    Serial.println("[MQTT] Subscribe acknowledged.");
    Serial.print("    - packetId: ");
    Serial.println(packetId);
    Serial.print("    - qos: ");
    Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
    Serial.println("[MQTT] Unsubscribe acknowledged.");
    Serial.print("    - packetId: ");
    Serial.println(packetId);
}
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    Serial.println("[MQTT] Publish received. Message info:");
    Serial.print("    - topic: ");
    Serial.println(topic);
    Serial.print("    - qos: ");
    Serial.println(properties.qos);
    Serial.print("    - dup: ");
    Serial.println(properties.dup);
    Serial.print("    - retain: ");
    Serial.println(properties.retain);
    Serial.print("    - len: ");
    Serial.println(len);
    Serial.print("    - index: ");
    Serial.println(index);
    Serial.print("    - total: ");
    Serial.println(total);
    Serial.println("    - payload: ");
    Serial.println(payload);
    Serial.println("[MQTT] Message info printing finished");
}
void onMqttPublish(uint16_t packetId) {
    Serial.println("[MQTT] Publish acknowledged.");
    Serial.print("    - packetId: ");
    Serial.println(packetId);
}
