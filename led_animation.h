#ifndef LED_ANIMATION_H
#define LED_ANIMATION_H

#include <Adafruit_NeoPixel.h>

// External LED strip object
extern Adafruit_NeoPixel strip;

// Function prototypes
void initLEDs();
void animationTask();
void setLEDPattern(uint8_t pattern);

#endif // LED_ANIMATION_H
