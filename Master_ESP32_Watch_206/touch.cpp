/*
 * touch.cpp - Touch Input Management (FIXED v3)
 * FT3168 returns coordinates in SCREEN PIXELS directly (0-410, 0-502)
 * NOT in 0-4095 range!
 * 
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 */

#include "touch.h"
#include "config.h"
#include <Wire.h>

// Touch state tracking
static bool touchPressed = false;
static int touchStartX = 0, touchStartY = 0;
static int touchLastX = 0, touchLastY = 0;
static unsigned long touchStartTime = 0;
static int digitalCrownValue = 0;

// FT3168 I2C address
#define FT3168_ADDR 0x38

// Swipe threshold - LOWERED for better detection
#define TOUCH_SWIPE_THRESHOLD 25  // 25 pixels minimum for swipe

// =============================================================================
// LOW-LEVEL TOUCH READ - FT3168 returns screen coordinates directly!
// =============================================================================
bool touchRead(uint16_t &x, uint16_t &y) {
  Wire.beginTransmission(FT3168_ADDR);
  Wire.write(0x02);  // Touch data register
  if (Wire.endTransmission(false) != 0) return false;
  
  Wire.requestFrom((uint8_t)FT3168_ADDR, (uint8_t)5);
  if (Wire.available() < 5) return false;
  
  uint8_t touches = Wire.read();
  if (touches == 0 || touches > 2) return false;
  
  uint8_t xh = Wire.read();
  uint8_t xl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t yl = Wire.read();
  
  // FT3168 returns coordinates in screen pixels directly!
  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
  
  return true;
}

bool touchReadEx(uint16_t &x, uint16_t &y, uint8_t &pressure) {
  pressure = 100;
  return touchRead(x, y);
}

// =============================================================================
// INITIALIZE TOUCH - With reset pin support for 2.06" board
// =============================================================================
bool initializeTouch() {
  // Reset touch controller if TP_RST is defined
  #if defined(TP_RST) && TP_RST >= 0
    pinMode(TP_RST, OUTPUT);
    digitalWrite(TP_RST, LOW);
    delay(10);
    digitalWrite(TP_RST, HIGH);
    delay(50);
    Serial.println("[TOUCH] Touch controller reset via TP_RST");
  #endif
  
  Wire.beginTransmission(FT3168_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[TOUCH] FT3168 not found!");
    return false;
  }
  
  Serial.println("[TOUCH] FT3168 initialized (2.06\" board)");
  Serial.printf("[TOUCH] Swipe threshold: %d pixels\n", TOUCH_SWIPE_THRESHOLD);
  return true;
}

// Alias function for initializeTouch
bool initTouch() {
  return initializeTouch();
}

// Touch interrupt flag (declared extern in main .ino)
extern volatile bool touch_interrupt;

// Touch ISR handler
void IRAM_ATTR touchISR() {
  touch_interrupt = true;
}

// =============================================================================
// TOUCH GESTURE HANDLING - NO MAPPING NEEDED, coordinates are screen pixels
// =============================================================================
TouchGesture handleTouchInput() {
  TouchGesture gesture = {TOUCH_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, millis(), false};
  
  uint16_t raw_x, raw_y;
  bool hasTouchNow = touchRead(raw_x, raw_y);
  
  // NO MAPPING - FT3168 returns screen coordinates directly
  int touch_x = raw_x;
  int touch_y = raw_y;
  
  // Constrain to screen bounds
  if (hasTouchNow) {
    touch_x = constrain(touch_x, 0, LCD_WIDTH - 1);
    touch_y = constrain(touch_y, 0, LCD_HEIGHT - 1);
  }
  
  // === TOUCH RELEASE ===
  if (!hasTouchNow && touchPressed) {
    touchPressed = false;
    
    // Calculate total movement from START to END
    int dx = touchLastX - touchStartX;
    int dy = touchLastY - touchStartY;
    unsigned long duration = millis() - touchStartTime;
    
    gesture.event = TOUCH_RELEASE;
    gesture.x = touchLastX;
    gesture.y = touchLastY;
    gesture.start_x = touchStartX;
    gesture.start_y = touchStartY;
    gesture.end_x = touchLastX;
    gesture.end_y = touchLastY;
    gesture.dx = dx;
    gesture.dy = dy;
    gesture.duration = duration;
    gesture.is_valid = true;
    
    Serial.printf("[TOUCH] Release: start(%d,%d) end(%d,%d) dx=%d dy=%d dur=%lu\n",
                  touchStartX, touchStartY, touchLastX, touchLastY, dx, dy, duration);
    
    // Determine gesture type
    if (duration < 800) {  // Quick gesture (increased from 600)
      if (abs(dx) > TOUCH_SWIPE_THRESHOLD || abs(dy) > TOUCH_SWIPE_THRESHOLD) {
        // SWIPE DETECTED
        if (abs(dx) > abs(dy)) {
          gesture.event = (dx > 0) ? TOUCH_SWIPE_RIGHT : TOUCH_SWIPE_LEFT;
          Serial.printf("[TOUCH] >>> SWIPE %s (dx=%d) <<<\n", dx > 0 ? "RIGHT" : "LEFT", dx);
        } else {
          gesture.event = (dy > 0) ? TOUCH_SWIPE_DOWN : TOUCH_SWIPE_UP;
          Serial.printf("[TOUCH] >>> SWIPE %s (dy=%d) <<<\n", dy > 0 ? "DOWN" : "UP", dy);
        }
      } else if (duration < 500 && abs(dx) < 20 && abs(dy) < 20) {
        // TAP - relaxed thresholds for small watch screen
        gesture.event = TOUCH_TAP;
        gesture.x = touchStartX;
        gesture.y = touchStartY;
        Serial.printf("[TOUCH] TAP at (%d,%d)\n", touchStartX, touchStartY);
      }
    } else if (duration > 800 && abs(dx) < 20 && abs(dy) < 20) {
      // LONG PRESS
      gesture.event = TOUCH_LONG_PRESS;
      Serial.printf("[TOUCH] LONG_PRESS at (%d,%d) dur=%lu\n", touchStartX, touchStartY, duration);
    }
    
    return gesture;
  }
  
  // === NEW TOUCH ===
  if (hasTouchNow && !touchPressed) {
    touchPressed = true;
    touchStartX = touch_x;
    touchStartY = touch_y;
    touchLastX = touch_x;
    touchLastY = touch_y;
    touchStartTime = millis();
    
    gesture.event = TOUCH_PRESS;
    gesture.x = touch_x;
    gesture.y = touch_y;
    gesture.start_x = touch_x;
    gesture.start_y = touch_y;
    gesture.is_valid = true;
    
    Serial.printf("[TOUCH] Press at (%d,%d)\n", touch_x, touch_y);
    
    return gesture;
  }
  
  // === TOUCH MOVE ===
  if (hasTouchNow && touchPressed) {
    // Only update end position, keep start position
    touchLastX = touch_x;
    touchLastY = touch_y;
    
    int dx = touch_x - touchStartX;
    int dy = touch_y - touchStartY;
    
    if (abs(dx) > 3 || abs(dy) > 3) {
      gesture.event = TOUCH_MOVE;  // FIXED: TOUCH_MOVE is now defined in config.h
      gesture.x = touch_x;
      gesture.y = touch_y;
      gesture.start_x = touchStartX;
      gesture.start_y = touchStartY;
      gesture.end_x = touch_x;
      gesture.end_y = touch_y;
      gesture.dx = dx;
      gesture.dy = dy;
      gesture.is_valid = true;
      
      // Digital crown simulation for scrolling
      digitalCrownValue += dy;
    }
  }
  
  return gesture;
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================
bool isTouchPressed() {
  return touchPressed;
}

void getTouchPosition(int& x, int& y) {
  x = touchLastX;
  y = touchLastY;
}

TouchEvent recognizeGesture(int start_x, int start_y, int end_x, int end_y, unsigned long duration) {
  int dx = end_x - start_x;
  int dy = end_y - start_y;
  
  if (duration > 800) return TOUCH_LONG_PRESS;
  
  if (abs(dx) > TOUCH_SWIPE_THRESHOLD || abs(dy) > TOUCH_SWIPE_THRESHOLD) {
    if (abs(dx) > abs(dy)) {
      return (dx > 0) ? TOUCH_SWIPE_RIGHT : TOUCH_SWIPE_LEFT;
    } else {
      return (dy > 0) ? TOUCH_SWIPE_DOWN : TOUCH_SWIPE_UP;
    }
  }
  
  if (duration < 500) return TOUCH_TAP;
  
  return TOUCH_NONE;
}

bool isSwipeGesture(int start_x, int start_y, int end_x, int end_y) {
  int dx = abs(end_x - start_x);
  int dy = abs(end_y - start_y);
  return (dx > TOUCH_SWIPE_THRESHOLD || dy > TOUCH_SWIPE_THRESHOLD);
}

bool isTapGesture(unsigned long duration, int movement) {
  return (duration < 500 && movement < 20);
}

// Calibration - not needed for FT3168, coordinates are screen pixels
void calibrateTouch() { }
void saveTouchCalibration() { }
void loadTouchCalibration() { }

// Digital crown
int getDigitalCrownValue() { return digitalCrownValue; }
void resetDigitalCrown() { digitalCrownValue = 0; }

// I2C recovery
bool recoverTouchI2C() {
  Wire.end();
  delay(10);
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  return true;
}

bool checkAndRecoverTouchI2C() {
  Wire.beginTransmission(FT3168_ADDR);
  if (Wire.endTransmission() != 0) {
    return recoverTouchI2C();
  }
  return true;
}
