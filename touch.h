/*
 * touch.h - Touch Input Management
 * FT3168 Capacitive Touch Controller
 */

#ifndef TOUCH_H
#define TOUCH_H

#include "config.h"
#include <Wire.h>

// Touch point structure
struct TouchPoint {
  int x, y;
  bool pressed;
  unsigned long timestamp;
};

// =============================================================================
// TOUCH FUNCTIONS
// =============================================================================

bool initializeTouch();
bool initTouch();  // Alias for initializeTouch()
bool touchRead(uint16_t &x, uint16_t &y);
bool touchReadEx(uint16_t &x, uint16_t &y, uint8_t &pressure);
TouchGesture handleTouchInput();
bool isTouchPressed();
void getTouchPosition(int& x, int& y);

// Touch interrupt handler (ISR)
void IRAM_ATTR touchISR();

// Gesture recognition
TouchEvent recognizeGesture(int start_x, int start_y, int end_x, int end_y, unsigned long duration);
bool isSwipeGesture(int start_x, int start_y, int end_x, int end_y);
bool isTapGesture(unsigned long duration, int movement);

// Calibration
void calibrateTouch();
void saveTouchCalibration();
void loadTouchCalibration();

// Digital crown simulation
int getDigitalCrownValue();
void resetDigitalCrown();

// I2C recovery
bool recoverTouchI2C();
bool checkAndRecoverTouchI2C();

#endif // TOUCH_H
