#ifndef POV_DISPLAY_H
#define POV_DISPLAY_H

#include <Arduino.h>
#include "commands.h"

// Initialize POV display system
void initPOV();

// Called from encoder ISR on each transition
void IRAM_ATTR povEncoderUpdate(bool isSync);

// Display current column (called from ISR)
void IRAM_ATTR displayPOVColumn();

// POV control functions
void setPOVEnable(bool enable);
bool getPOVEnable();
void selectAnimation(uint8_t animationId);
void setFrameTiming(uint16_t revolutionsPerFrame);
uint32_t getRevolutionCount();
void resetRevolutionCount();
uint16_t getCurrentFrame();

// Animation management
bool startAnimationUpload(uint8_t animationId, uint16_t totalFrames);
bool uploadFrameData(uint16_t frameNum, uint8_t column, const uint8_t* rgbData);
bool endAnimationUpload();

// Pre-compiled animations
void loadClockAnimation();
void loadTestAnimation();

// External access to POV state
extern POVState povState;

#endif // POV_DISPLAY_H
