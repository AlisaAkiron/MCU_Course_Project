//
// Created by Liam Sho on 2021/12/21.
//

#ifndef MCU_COURSE_PROJECT_MAIN_H
#define MCU_COURSE_PROJECT_MAIN_H


void pin_setup();
void wifi_setup();

void mqtt_setup();
void mqtt_connect();

void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

void mqttMessageHandler(char* data);

#endif //MCU_COURSE_PROJECT_MAIN_H
