#include <Arduino.h>
#include <ESPAsync_WiFiManager.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <NeoPixelBus.h>
#include <TimeLib.h>

#include "main.h"
#include "configuration.h"

AsyncWebServer webServer(80);
DNSServer dnsServer;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(NEO_PIXEL_COUNT, NEO_PIXEL_PIN);
NeoTopology<ColumnMajorAlternatingLayout> matrix(NEO_PIXEL_WIDTH, NEO_PIXEL_HEIGHT);

RgbColor colors[NEO_PIXEL_WIDTH][NEO_PIXEL_HEIGHT];
RgbColor digitColor(255, 255, 255);
const RgbColor blackColor(0, 0, 0);

WiFiUDP Udp;
unsigned int localPort = 8888;
static const char ntpServerName[] = "ntp.ntsc.ac.cn";
int timezone = 0;
time_t prevDisplay = 0;

Mission LoopMission = no_mission;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
    Serial.println("[SERIAL] Serial started with baud rate: " + String(BAUD_RATE));

    pin_setup();
    wifi_setup();
    mqtt_setup();
    led_setup();
    mqtt_connect();
    delay(2000);
    time_sync_setup();
}

void loop() {
    if (timeStatus() != timeNotSet) {
        if (now() != prevDisplay) {
            prevDisplay = now();
        }
    }
    switch (LoopMission) {
        case clock_display_update:
            time_display_update_mission();
            delay(100);
            break;
        case no_mission:
        default:
            break;
    }
}

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
void led_setup() {
    Serial.println("[LED] Setup LED, display every LED in black");
    strip.Begin();
    led_matrix_init();
    Serial.println("[LED] LED setup finished");
}
void time_sync_setup() {
    log("[NTP] Setup time sync");
    Udp.begin(localPort);
    setSyncProvider(get_ntp_time);
    setSyncInterval(3600);
    log("[TIME] Time sync setup finished");
}

void mqtt_connect() {
    Serial.println("[MQTT] Connecting to MQTT");
    mqttClient.connect();
}
void onMqttConnect(bool sessionPresent) {
    log("========================= NEW MQTT LOG SESSION ===========================");
    log("[MQTT] Connected to MQTT successfully");
    log("[MQTT] Session present: " + String(sessionPresent));
    uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC, MQTT_TOPIC_QOS);
    log("[MQTT] Subscribing at QoS " + String(MQTT_TOPIC_QOS) + ", packetId: " + String(packetIdSub));
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
    log("[MQTT] Subscribe acknowledged.");
    log("    - packetId: " + String(packetId));
    log("    - qos: " + String(qos));
}
void onMqttUnsubscribe(uint16_t packetId) {
    log("[MQTT] Unsubscribe acknowledged.");
    log("    - packetId: " + String(packetId));
}
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
#ifdef DEBUG
    log("[MQTT] Message received.");
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
#endif
    mqttMessageHandler(payload);
}
void onMqttPublish(uint16_t packetId) {
    Serial.println("[MQTT] Publish acknowledged.");
    Serial.println("    - packetId: " + String(packetId));
}

void log(const String& message) {
    const size_t strLength = message.length();
    char* str = new char[strLength + 1];
    message.toCharArray(str, strLength + 1);
    Serial.println(message);
    mqttClient.publish("esp32_log", 2, false, str, strLength, false, 0);
}

void mqttMessageHandler(char* data) {
    ArduinoJson6185_91::StaticJsonDocument<4096> doc;
    ArduinoJson6185_91::DeserializationError error = ArduinoJson6185_91::deserializeJson(doc, data);
    if (error) {
        log("[HANDLER] Json message deserialization failed: " + String(error.f_str()));
        return;
    }

    led_mode_helper(doc);
}

// LED matrix helpers
void led_mode_helper(const ArduinoJson6185_91::StaticJsonDocument<4096>& doc) {
    log("[HANDLER] Detecting led matrix mode");
    const char* mode = doc["mode"];
    if (mode == nullptr) {
        log("[HANDLER] Mode not found, the \"mode\" field is missing");
        return;
    }
    if (strcmp(mode, "single") == 0) {
        log("[HANDLER] Single led mode detected");
        int16_t single_x = doc["data"]["x"];
        int16_t single_y = doc["data"]["y"];
        int8_t single_r = doc["data"]["r"];
        int8_t single_g = doc["data"]["g"];
        int8_t single_b = doc["data"]["b"];
        display_single_pixel(single_x, single_y, RgbColor(single_r, single_g, single_b));
    }
    else if (strcmp(mode, "clock") == 0) {
        int clock_timezone_offset = doc["data"]["tz"];
        display_clock(clock_timezone_offset);
    }
    else {
        log("[HANDLER] Unknown mode detected");
    }
}
void display_single_pixel(int16_t x, int16_t y, RgbColor color) {
    colors[x][y] = color;
    strip.SetPixelColor(matrix.Map(x, y), colors[x][y]);
    strip.Show();
}
void display_clock(int tz) {
    // Set timezone and fetch time from NTP
    timezone = tz;
    time_sync_setup();

    // Clean LED matrix
    led_matrix_init();

    // Display clock dot
    display_single_pixel(11, 2, digitColor);
    display_single_pixel(11, 4, digitColor);
    display_single_pixel(21, 2, digitColor);
    display_single_pixel(21, 4, digitColor);

    // Setup loop mission
    LoopMission = Mission::clock_display_update;
}

/**********************/
/*       Mission      */
/**********************/

// Loop mission
void time_display_update_mission() {
    String h_str = String(hour());
    String m_str = String(minute());
    String s_str = String(second());
    if (h_str.length() == 1) {
        h_str = "0" + h_str;
    }
    if (m_str.length() == 1) {
        m_str = "0" + m_str;
    }
    if (s_str.length() == 1) {
        s_str = "0" + s_str;
    }
    char h[2] = {h_str[0], h_str[1]};
    char m[2] = {m_str[0], m_str[1]};
    char s[2] = {s_str[0], s_str[1]};

    set_digit_color(3, 1, h[0]);
    set_digit_color(7, 1, h[1]);
    set_digit_color(13, 1, m[0]);
    set_digit_color(17, 1, m[1]);
    set_digit_color(23, 1, s[0]);
    set_digit_color(27, 1, s[1]);

    led_matrix_refresh();
}

// Basic mission
void led_matrix_refresh() {
    for (int16_t x = 0; x < NEO_PIXEL_WIDTH; x++) {
        for (int16_t y = 0; y < NEO_PIXEL_HEIGHT; y++) {
            strip.SetPixelColor(matrix.Map(x, y), colors[x][y]);
        }
    }
    strip.Show();
}
void led_matrix_init() {
    for (auto & x : colors) {
        for (auto & y : x) {
            y = blackColor;
        }
    }
    strip.Show();
}

/**********************/
/*      Utilities     */
/**********************/

// NTP
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

time_t get_ntp_time() {
    IPAddress ntpServerIP;

    while (Udp.parsePacket() > 0) ;
    log("[NTP] Transmit NTP Request");
    WiFiClass::hostByName(ntpServerName, ntpServerIP);
    log("[NTP] Use NTP server " + String(ntpServerName) + " : " + String(ntpServerIP));
    sendNTPPacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE) {
            log("[NTP] Receive NTP Response");
            Udp.read(packetBuffer, NTP_PACKET_SIZE);
            unsigned long secsSince1900;
            secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            return secsSince1900 - 2208988800UL + timezone * 3600; // NOLINT(cppcoreguidelines-narrowing-conversions)
        }
    }
    log("[NTP] NTP server not response :-(");
    return 0;
}
void sendNTPPacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

// Digit display set
void set_digit_color(int16_t start_x, int16_t start_y, char ch) {
    char group[16];
    switch (ch) {
        case '0':
        {
            String("XXXXOXXOXXOXXXX").toCharArray(group, 16);
            Serial.println(group);
            break;
        }
        case '1':
        {
            String("OXOOXOOXOOXOOXO").toCharArray(group, 16);
            break;
        }
        case '2':
        {
            String("XXXOOXXXXXOOXXX").toCharArray(group, 16);
            break;
        }
        case '3':
        {
            String("XXXOOXXXXOOXXXX").toCharArray(group, 16);
            break;
        }
        case '4':
        {
            String("XOXXOXXXXOOXOOX").toCharArray(group, 16);
            break;
        }
        case '5':
        {
            String("XXXXOOXXXOOXXXX").toCharArray(group, 16);
            break;
        }
        case '6':
        {
            String("XXXXOOXXXXOXXXX").toCharArray(group, 16);
            break;
        }
        case '7':
        {
            String("XXXOOXOOXOOXOOX").toCharArray(group, 16);
            break;
        }
        case '8':
        {
            String("XXXXOXXXXXOXXXX").toCharArray(group, 16);
            break;
        }
        case '9':
        {
            String("XXXXOXXXXOOXXXX").toCharArray(group, 16);
            break;
        }
        default: {
            String("XOXXOXOXOXOXXOX").toCharArray(group, 16);
            break;
        }
    }
    set_digit_color(start_x, start_y, group);
}
void set_digit_color(int16_t start_x, int16_t start_y, const char* group) {
    for (int16_t y = start_y; y < start_y + 5; y++) {
        for (int16_t x = start_x; x < start_x + 3; x++) {
            if (group[3 * (y - start_y) + (x - start_x)] == 'X') {
                colors[x][y] = digitColor;
            }
            else {
                colors[x][y] = blackColor;
            }
        }
    }
}
