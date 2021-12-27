#include <utility>

//
// Created by Liam Sho on 2021/12/21.
//

#ifndef MCU_COURSE_PROJECT_MAIN_H
#define MCU_COURSE_PROJECT_MAIN_H

enum Mission {
    no_mission,
    clock_display_update,
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
void onMqttMessage(__attribute__((unused)) char* topic, char* payload,
                   __attribute__((unused)) AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

void log(const String& message, bool disable = false);

void mqttMessageHandler(String data);

void mqtt_message_distributor(const ArduinoJson::DynamicJsonDocument& doc);

void display_single_pixel(int16_t x, int16_t y, HslColor color);
void display_clock(int tz);

void mode_resolver(const ArduinoJson::DynamicJsonDocument& doc);
void command_resolver(const ArduinoJson::DynamicJsonDocument& doc);

void time_display_update_mission();

void led_matrix_refresh();
void led_matrix_init();
void led_matrix_fill(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, HslColor color);
void led_matrix_toggle(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, bool status);

time_t get_ntp_time();
void sendNTPPacket(IPAddress &address);

void set_digit_color(int16_t start_x, int16_t start_y, char ch);
void set_digit_color(int16_t start_x, int16_t start_y, const char* group);


#endif //MCU_COURSE_PROJECT_MAIN_H
