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

#define PIXEL_ON true
#define PIXEL_OFF false

static unsigned int udpLocalPort = 8888;
const char ntpServerName[] = "ntp.ntsc.ac.cn";

const HslColor blackColor(0, 0, 0);
float generalLightness = 1.0;
float digitColorHue = 255;
float digitColorSaturation = 255;
float digitColorLightness = 1;


#endif //MCU_COURSE_PROJECT_CONFIGURATION_H
