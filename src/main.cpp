#include <Arduino.h>
#include <ESPAsync_WiFiManager.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <NeoPixelBus.h>
#include <TimeLib.h>

#include "main.h"

#include "MqttConfiguration.h"
#include "Configuration.h"

/**********************/
/*    Declaration     */
/**********************/

// Wifi manager required web server and dns server
AsyncWebServer webServer(WEB_SERVER_PORT);
DNSServer dnsServer;

// MQTT client declaration
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

// Neo pixel declaration
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(NEO_PIXEL_COUNT, NEO_PIXEL_PIN);
NeoTopology<ColumnMajorAlternatingLayout> matrix(NEO_PIXEL_WIDTH, NEO_PIXEL_HEIGHT);

// LED matrix colors and brightness definition
HslColor colors[NEO_PIXEL_WIDTH][NEO_PIXEL_HEIGHT];
bool pixel_status[NEO_PIXEL_WIDTH][NEO_PIXEL_HEIGHT];

// Clock display mode
HslColor digitColor;

// NTP time services
WiFiUDP Udp;
int time_zone = 0;
time_t prev_update_time = 0;

// Loop mission
Mission LoopMission = no_mission;

// Buffer
String mqttBuffer;
uint32_t mqttLength = 0;
HslColor frameBuffer[NEO_PIXEL_WIDTH][NEO_PIXEL_HEIGHT];

/**********************/
/* Arduino Life Cycle */
/**********************/

// Arduino setup
void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
    Serial.println();
    Serial.println("[SERIAL] Serial started with baud rate: " + String(BAUD_RATE));

    pin_setup();
    wifi_setup();
    mqtt_setup();
    led_setup();
    mqtt_connect();
    delay(2000);
    time_sync_setup();
}

// Arduino loop
void loop() {
    const unsigned long before = millis();
    switch (LoopMission) {
        case clock_display_update:
            if (timeStatus() != timeNotSet) {
                if (now() != prev_update_time) {
                    prev_update_time = now();
                    time_display_update_mission();
                }
            }
            break;
        case no_mission:
        default:
            break;
    }
    led_matrix_refresh();
    delayMicroseconds(max(0, (int32_t)(16666 - (micros() - before))));
}

/**********************/
/*      Set up        */
/**********************/

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
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    Serial.println("[MQTT] MQTT setup finished");
}
void led_setup() {
    Serial.println("[LED] Setup LED, display every LED in black");
    digitColor = HslColor(digitColorHue, digitColorSaturation, digitColorLightness);
    strip.Begin();
    led_matrix_init();
    Serial.println("[LED] LED setup finished");
}
void time_sync_setup() {
    log("[NTP] Setup time sync", 1);
    Udp.begin(udpLocalPort);
    setSyncProvider(get_ntp_time);
    setSyncInterval(3600);
    log("[TIME] Time sync setup finished", 1);
}

/**********************/
/*     Async MQTT     */
/**********************/

void mqtt_connect() {
    Serial.println("[MQTT] Connecting to MQTT");
    mqttClient.connect();
}
void onMqttConnect(bool sessionPresent) {
    log("========================= NEW MQTT LOG SESSION ===========================", 1);
    log("[MQTT] Connected to MQTT successfully", 1);
    log("[MQTT] Session present: " + String(sessionPresent), 1);
    uint16_t packetIdSub = mqttClient.subscribe(MQTT_MAIN_TOPIC, MQTT_TOPIC_QOS);
    log("[MQTT] Subscribing at QoS " + String(MQTT_TOPIC_QOS) + ", packetId: " + String(packetIdSub), 1);
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
    log("[MQTT] Subscribe acknowledged.", 1);
    log("    - packetId: " + String(packetId), 1);
    log("    - qos: " + String(qos), 1);
}
void onMqttUnsubscribe(uint16_t packetId) {
    log("[MQTT] Unsubscribe acknowledged.", 1);
    log("    - packetId: " + String(packetId), 1);
}
void onMqttMessage(__attribute__((unused)) char* topic, char* payload,
                   __attribute__((unused)) AsyncMqttClientMessageProperties properties, size_t len,
                   __attribute__((unused)) size_t index, size_t total) {
    log("[MQTT] Message received.");
    mqttBuffer.concat(payload);
    mqttLength += len;
    if (mqttLength == total) {
        log("[MQTT] Message received completely");
        mqttMessageHandler(mqttBuffer);
        mqttBuffer.clear();
        mqttLength = 0;
    }
}

/**********************/
/*       Logger       */
/**********************/

void log(const String& message, int log_level) {
    if (log_level < LOG_LEVEL) {
        return;
    }
    const size_t strLength = message.length();
    char* str = new char[strLength + 1];
    message.toCharArray(str, strLength + 1);
    Serial.println(message);
    mqttClient.publish(MQTT_LOG_TOPIC, 2, false, str, strLength, false, 0);
}

/**********************/
/*    MQTT Handler    */
/**********************/

void mqttMessageHandler(String data) {
    ArduinoJson::DynamicJsonDocument doc(36864);
    ArduinoJson::DeserializationError error = ArduinoJson::deserializeJson(doc, data);
    if (error) {
        log("[HANDLER] Json message deserialization failed: " + String(error.f_str()), 2);
        return;
    }
    mqtt_message_distributor(doc);
}

/**********************/
/*   Message Resolve  */
/**********************/

// Distributor
void mqtt_message_distributor(const ArduinoJson::DynamicJsonDocument & doc) {
    log("[HANDLER] Detecting led matrix mode");
    const char* mode = doc["type"];
    if (mode == nullptr) {
        log("[HANDLER] Type not found, the \"type\" field is missing", 2);
        return;
    }

    if (strcmp(mode, "mode") == 0) {
        log("[HANDLER] Mode type detected");
        mode_resolver(doc);
    }
    else if (strcmp(mode, "command") == 0) {
        log("[HANDLER] Command type detected");
        command_resolver(doc);
    }
    else {
        log("[HANDLER] Unknown type detected, field value is " + String(mode), 2);
    }
}

// LED matrix helpers
void display_single_pixel(int16_t x, int16_t y, HslColor color) {
    colors[x][y] = HslColor(color.H, color.S, color.L * generalLightness);
    pixel_status[x][y] = PIXEL_ON;
}
void display_clock(int tz) {
    // Set timezone and fetch time from NTP
    time_zone = tz;
    time_sync_setup();

    // Clean LED matrix
    led_matrix_init();

    // Setup loop mission
    LoopMission = Mission::clock_display_update;
}

// Main Resolver
void mode_resolver(const ArduinoJson::DynamicJsonDocument& doc) {
    auto data = doc["data"];
    String mode = data["mode"];
    if (mode == "single") {
        log("[HANDLER] Single mode detected");
        int16_t x = data["value"]["x"];
        int16_t y = data["value"]["y"];
        int16_t r = data["value"]["r"];
        int16_t g = data["value"]["g"];
        int16_t b = data["value"]["b"];
        HslColor color = HslColor(RgbColor(r, g, b));
        display_single_pixel(x, y, color);
        log("[HANDLER] Single mode resolved, display at (" + String(x) + ", " + String(y) + ") with color" +
            "RGB(" + String(r) + ", " + String(g) + ", " + String(b) + ") aka HSL(" +
            String(color.H) + ", " + String(color.S) + ", " + String(color.L) + ")", 1);
    }
    else if (mode == "clock") {
        log("[HANDLER] Clock mode detected");
        int tz = data["value"]["tz"];
        display_clock(tz);
        log("[HANDLER] Clock mode activated", 1);
    }
    else if (mode == "picture") {
        log("[HANDLER] Picture mode detected");
        int16_t part = data["value"]["part"];
        auto frame = data["value"]["frame"];
        for (int i = 0; i < NEO_PIXEL_WIDTH * NEO_PIXEL_HEIGHT; i++) {
            int16_t x = frame[i]["x"];
            int16_t y = frame[i]["y"];
            int16_t r = frame[i]["r"];
            int16_t g = frame[i]["g"];
            int16_t b = frame[i]["b"];
            HslColor color = HslColor(RgbColor(r, g, b));
            frameBuffer[x][y] = color;
        }
        log("[HANDLER] Add frame command resolved, part " + String(part) + " added to buffer");
        if (part == 8) {
            log("[HANDLER] All frame parts added, display picture");
            led_matrix_init();
            LoopMission = Mission::no_mission;
            for (int x = 0; x < NEO_PIXEL_WIDTH; x++) {
                for (int y = 0; y < NEO_PIXEL_HEIGHT; y++) {
                    colors[x][y] = frameBuffer[x][y];
                }
            }
            led_matrix_toggle(0, 0, 31, 31, PIXEL_ON);
            led_matrix_refresh();
        }
    }
    else if (mode == "stop") {
        log("[HANDLER] Stop mode detected");
        LoopMission = no_mission;
        led_matrix_init();
        log("[HANDLER] Stop mode mission completed", 1);
    }
    else {
        log("[HANDLER] Unknown mode detected, field value is " + String(mode), 2);
    }
}
void command_resolver(const ArduinoJson::DynamicJsonDocument& doc) {
    auto data = doc["data"];
    String command = data["command"];
    if (command == "set_general_lightness") {
        log("[HANDLER] Set general lightness command detected");
        float lightness = data["param"]["lightness"];
        generalLightness = lightness;
        log("[HANDLER] General lightness set to " + String(lightness), 1);
    }
    else if (command == "set_digit_color") {
        log("[HANDLER] Set digit color command detected");
        int16_t r = data["param"]["r"];
        int16_t g = data["param"]["g"];
        int16_t b = data["param"]["b"];
        digitColor = HslColor(RgbColor(r, g, b));
        log("[HANDLER] Set digit color to RGB(" + String(r) + ", " + String(g) + ", " + String(b) + ") " +
            "aka HSL(" + String(digitColor.H) + ", " + String(digitColor.S) + ", " + String(digitColor.L) + ")", 1);
    }
    else {
        log("[HANDLER] Unknown command detected, field value is " + String(command), 2);
    }
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

    display_single_pixel(11, 2, digitColor);
    display_single_pixel(11, 4, digitColor);
    display_single_pixel(21, 2, digitColor);
    display_single_pixel(21, 4, digitColor);
}

// Basic mission
void led_matrix_refresh() {
    for (int16_t x = 0; x < NEO_PIXEL_WIDTH; x++) {
        for (int16_t y = 0; y < NEO_PIXEL_HEIGHT; y++) {
            if (pixel_status[x][y]) {
                strip.SetPixelColor(matrix.Map(x, y), colors[x][y]);
            }
            else {
                strip.SetPixelColor(matrix.Map(x, y), blackColor);
            }
        }
    }
    strip.Show();
}
void led_matrix_init() {
    led_matrix_fill(0, 0, NEO_PIXEL_WIDTH - 1, NEO_PIXEL_HEIGHT - 1, blackColor);
    led_matrix_toggle(0, 0, NEO_PIXEL_WIDTH - 1, NEO_PIXEL_HEIGHT - 1, PIXEL_OFF);
    strip.Show();
}
void led_matrix_fill(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, HslColor color) {
    for (int16_t x = start_x; x <= end_x; x++) {
        for (int16_t y = start_y; y <= end_y; y++) {
            colors[x][y] = color;
        }
    }
}
void led_matrix_toggle(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, bool status) {
    for (int16_t x = start_x; x <= end_x; x++) {
        for (int16_t y = start_y; y <= end_y; y++) {
            pixel_status[x][y] = status;
        }
    }
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
            return secsSince1900 - 2208988800UL + time_zone * 3600; // NOLINT(cppcoreguidelines-narrowing-conversions)
        }
    }
    log("[NTP] NTP server not response :-(", 2);
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
    led_matrix_fill(start_x, start_y, (int16_t)(start_x + 2), (int16_t)(start_y + 4),
                    HslColor(digitColor.H, digitColor.S, digitColor.L * generalLightness));
    for (int16_t y = start_y; y < start_y + 5; y++) {
        for (int16_t x = start_x; x < start_x + 3; x++) {
            if (group[3 * (y - start_y) + (x - start_x)] == 'X') {
                pixel_status[x][y] = true;
            }
            else {
                pixel_status[x][y] = false;
            }
        }
    }
}
