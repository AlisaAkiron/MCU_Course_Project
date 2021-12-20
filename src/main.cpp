#include <Arduino.h>
#include <ESPAsync_WiFiManager.h>

#include "main.h"

#define BAUD_RATE 9600

AsyncWebServer webServer(80);
DNSServer dnsServer;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
    Serial.println("[SERIAL] Serial started with baud rate: " + String(BAUD_RATE));

    pin_setup();
    wifi_setup();
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
