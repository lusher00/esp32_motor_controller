#include "pov_display.h"
#include "config.h"
#include "debug.h"
#include <Adafruit_NeoPixel.h>

// Global POV state
POVState povState;

// NeoPixel strip (defined in led_animation.cpp but we need access)
extern Adafruit_NeoPixel strip;

// Upload state
static uint8_t uploadAnimationId = 0;
static uint16_t uploadTotalFrames = 0;
static bool uploadInProgress = false;

void initPOV() {
  // Initialize POV state
  povState.enabled = false;
  povState.currentAnimation = 0;
  povState.currentFrame = 0;
  povState.revolutionCount = 0;
  povState.currentColumn = 0;
  
  // Initialize all animations as inactive
  for (int i = 0; i < MAX_ANIMATIONS; i++) {
    povState.animations[i].active = false;
    povState.animations[i].frames = nullptr;
  }
  
  DEBUG_INFO("✓ POV Display initialized");
}

void IRAM_ATTR povEncoderUpdate(bool isSync) {
  if (!povState.enabled) return;
  
  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  if (!anim->active) return;
  
  if (isSync) {
    // Sync pulse detected - reset to column 0
    povState.currentColumn = 0;
    povState.revolutionCount++;
    
    // Check if we need to advance frame
    if (povState.revolutionCount >= anim->revolutionsPerFrame) {
      povState.revolutionCount = 0;
      povState.currentFrame = (povState.currentFrame + 1) % anim->frameCount;
    }
  } else {
    // Normal transition - advance column
    povState.currentColumn = (povState.currentColumn + 1) % POV_COLUMNS;
  }
  
  // Display is handled separately to avoid long ISR execution
}

void IRAM_ATTR displayPOVColumn() {
  if (!povState.enabled) return;
  
  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  if (!anim->active || !anim->frames) return;
  
  // Get current frame and column
  POVFrame* frame = &anim->frames[povState.currentFrame];
  POVColumn* column = &frame->columns[povState.currentColumn];
  
  // Update NeoPixels
  for (int i = 0; i < POV_LEDS; i++) {
    strip.setPixelColor(i, 
                       column->rgb[i][0],  // R
                       column->rgb[i][1],  // G
                       column->rgb[i][2]); // B
  }
  strip.show();
}

void setPOVEnable(bool enable) {
  povState.enabled = enable;
  
  if (enable) {
    DEBUG_POV("POV Display ENABLED");
  } else {
    DEBUG_POV("POV Display DISABLED");
    // Clear strip
    strip.clear();
    strip.show();
  }
}

bool getPOVEnable() {
  return povState.enabled;
}

void selectAnimation(uint8_t animationId) {
  if (animationId >= MAX_ANIMATIONS) {
    DEBUG_WARN("Invalid animation ID");
    return;
  }
  
  if (!povState.animations[animationId].active) {
    DEBUG_WARN("Animation not loaded");
    return;
  }
  
  povState.currentAnimation = animationId;
  povState.currentFrame = 0;
  povState.revolutionCount = 0;
  povState.currentColumn = 0;
  
  DEBUG_POVF("Selected animation: %d", animationId);
  
}

void setFrameTiming(uint16_t revolutionsPerFrame) {
  POVAnimation* anim = &povState.animations[povState.currentAnimation];
  anim->revolutionsPerFrame = revolutionsPerFrame;
  
  DEBUG_POVF("Frame timing set to %d revolutions per frame", revolutionsPerFrame);
  
  
}

uint32_t getRevolutionCount() {
  return povState.revolutionCount;
}

void resetRevolutionCount() {
  povState.revolutionCount = 0;
  DEBUG_POV("Revolution count reset");
}

uint16_t getCurrentFrame() {
  return povState.currentFrame;
}

bool startAnimationUpload(uint8_t animationId, uint16_t totalFrames) {
  if (animationId >= MAX_ANIMATIONS) {
    DEBUG_WARN("Invalid animation ID");
    return false;
  }
  
  if (totalFrames > MAX_POV_FRAMES) {
    Serial.println("Too many frames");
    return false;
  }
  
  // Allocate memory for frames
  size_t frameSize = sizeof(POVFrame) * totalFrames;
  POVFrame* frames = (POVFrame*)malloc(frameSize);
  
  if (!frames) {
    DEBUG_ERROR("Failed to allocate memory for animation");
    return false;
  }
  
  // Clear existing animation if any
  if (povState.animations[animationId].frames) {
    free(povState.animations[animationId].frames);
  }
  
  // Initialize new animation
  povState.animations[animationId].id = animationId;
  povState.animations[animationId].frameCount = totalFrames;
  povState.animations[animationId].revolutionsPerFrame = 1;
  povState.animations[animationId].frames = frames;
  povState.animations[animationId].active = true;
  
  // Zero out frame data
  memset(frames, 0, frameSize);
  
  uploadAnimationId = animationId;
  uploadTotalFrames = totalFrames;
  uploadInProgress = true;
  
  DEBUG_POVF("Started upload for animation %d with %d frames", animationId, totalFrames);
  
  
  
  
  
  return true;
}

bool uploadFrameData(uint16_t frameNum, uint8_t column, const uint8_t* rgbData) {
  if (!uploadInProgress) {
    DEBUG_WARN("No upload in progress");
    return false;
  }
  
  if (frameNum >= uploadTotalFrames) {
    DEBUG_WARN("Frame number out of range");
    return false;
  }
  
  if (column >= POV_COLUMNS) {
    DEBUG_WARN("Column number out of range");
    return false;
  }
  
  POVAnimation* anim = &povState.animations[uploadAnimationId];
  POVFrame* frame = &anim->frames[frameNum];
  POVColumn* col = &frame->columns[column];
  
  // Copy RGB data
  memcpy(col->rgb, rgbData, POV_LEDS * 3);
  
  return true;
}

bool endAnimationUpload() {
  if (!uploadInProgress) {
    DEBUG_WARN("No upload in progress");
    return false;
  }
  
  uploadInProgress = false;
  
  DEBUG_POVF("Upload complete for animation %d", uploadAnimationId);
  
  
  return true;
}

// ============================================================================
// PRE-COMPILED ANIMATIONS
// ============================================================================

void loadTestAnimation() {
  // Simple test pattern: rotating rainbow
  
  if (!startAnimationUpload(0, 1)) {
    return;
  }
  
  uint8_t rgbData[POV_LEDS * 3];
  
  for (uint8_t col = 0; col < POV_COLUMNS; col++) {
    // Calculate color based on angle
    float angle = (float)col / POV_COLUMNS * 360.0;
    uint8_t hue = (uint8_t)(angle * 255.0 / 360.0);
    
    for (uint8_t led = 0; led < POV_LEDS; led++) {
      // Simple HSV to RGB conversion (hue only, S=V=1)
      uint8_t sector = hue / 43;
      uint8_t offset = (hue % 43) * 6;
      
      uint8_t r, g, b;
      
      switch(sector) {
        case 0: r = 255; g = offset; b = 0; break;
        case 1: r = 255 - offset; g = 255; b = 0; break;
        case 2: r = 0; g = 255; b = offset; break;
        case 3: r = 0; g = 255 - offset; b = 255; break;
        case 4: r = offset; g = 0; b = 255; break;
        case 5: r = 255; g = 0; b = 255 - offset; break;
        default: r = 0; g = 0; b = 0; break;
      }
      
      // Fade based on radius
      float brightness = (float)led / POV_LEDS;
      rgbData[led * 3 + 0] = r * brightness;
      rgbData[led * 3 + 1] = g * brightness;
      rgbData[led * 3 + 2] = b * brightness;
    }
    
    uploadFrameData(0, col, rgbData);
  }
  
  endAnimationUpload();
  
  DEBUG_INFO("✓ Test animation loaded");
}

void loadClockAnimation() {
  // TODO: Implement clock face with hour/minute hands
  // This would require multiple frames (60 frames for seconds, or 3600 for smooth minute hand)
  
  DEBUG_WARN("Clock animation not yet implemented");
}
