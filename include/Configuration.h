//
// Created by Liam Sho on 2021/12/23.
//

#ifndef MCU_COURSE_PROJECT_CONFIGURATION_H
#define MCU_COURSE_PROJECT_CONFIGURATION_H

#include <NeoPixelBus.h>

#define BAUD_RATE 9600

#define NEO_PIXEL_WIDTH 32
#define NEO_PIXEL_HEIGHT 8
#define NEO_PIXEL_COUNT NEO_PIXEL_WIDTH * NEO_PIXEL_HEIGHT
#define NEO_PIXEL_PIN 23

static unsigned int udpLocalPort = 8888;
const char ntpServerName[] = "ntp.ntsc.ac.cn";

const RgbColor blackColor(0, 0, 0);
float brightness = 1.0;
int16_t digitColorRed = 255;
int16_t digitColorGreen = 255;
int16_t digitColorBlue = 255;


#endif //MCU_COURSE_PROJECT_CONFIGURATION_H
