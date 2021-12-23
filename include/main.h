//
// Created by Liam Sho on 2021/12/21.
//

#ifndef MCU_COURSE_PROJECT_MAIN_H
#define MCU_COURSE_PROJECT_MAIN_H

enum Mission {
    no_mission,
    clock_display_update
};

void pin_setup();
void wifi_setup();
void led_setup();
void time_sync_setup();
void mqtt_setup();
void mqtt_connect();

void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

void log(const String& message);

void mqttMessageHandler(char* data);

void led_mode_helper(const ArduinoJson6185_91::StaticJsonDocument<4096>& doc);

void display_single_pixel(int16_t x, int16_t y, RgbColor color);
void display_clock(int tz);

void time_display_update_mission();
void led_matrix_refresh();
void led_matrix_init();

time_t get_ntp_time();
void sendNTPPacket(IPAddress &address);

void set_digit_color(int16_t start_x, int16_t start_y, char ch);
void set_digit_color(int16_t start_x, int16_t start_y, const char* group);


#endif //MCU_COURSE_PROJECT_MAIN_H
