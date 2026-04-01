/*
 * compass_app.cpp - Digital Compass Implementation
 * Quick Win Feature: Magnetometer-based compass
 */

#include "compass_app.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"
#include <Wire.h>
#include <math.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Compass state
CompassData compass = {0, 0, 0, 0, false, 0};

// QMI8658 Registers
#define QMI8658_MAG_X_L    0x33
#define QMI8658_MAG_X_H    0x34
#define QMI8658_MAG_Y_L    0x35
#define QMI8658_MAG_Y_H    0x36
#define QMI8658_MAG_Z_L    0x37
#define QMI8658_MAG_Z_H    0x38

// =============================================================================
// INITIALIZATION
// =============================================================================

void initCompassApp() {
  compass.calibrated = false;
  compass.heading = 0;
  compass.last_update = 0;
  
  // Note: QMI8658 initialization is done in hardware.cpp
  Serial.println("[Compass] Compass app initialized");
}

// =============================================================================
// COMPASS SCREEN
// =============================================================================

void drawCompassApp() {
  // ========================================
  // RETRO ANIME COMPASS - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 42 + 1, 14 + 1);
  gfx->print("COMPASS");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 42, 14);
  gfx->print("COMPASS");
  
  int centerX = LCD_WIDTH / 2;
  int centerY = 240;
  int outerRadius = 120;
  int innerRadius = 100;
  
  updateCompassReading();
  
  // Outer compass ring - pixel style tick marks
  for (int i = 0; i < 360; i += 3) {
    float a = i * PI / 180.0 - PI/2;
    int x = centerX + cos(a) * outerRadius;
    int y = centerY + sin(a) * outerRadius;
    
    if (i % 30 == 0) {
      // Major marks - pixel squares
      gfx->fillRect(x - 2, y - 2, 5, 5, theme->primary);
    } else if (i % 15 == 0) {
      gfx->fillRect(x - 1, y - 1, 3, 3, RGB565(50, 55, 70));
    }
  }
  
  // Cardinal directions - retro styled
  const char* cardinals[] = {"N", "E", "S", "W"};
  uint16_t cardinalColors[] = {COLOR_RED, RGB565(180, 185, 200), RGB565(180, 185, 200), RGB565(180, 185, 200)};
  int cardinalAngles[] = {0, 90, 180, 270};
  
  for (int i = 0; i < 4; i++) {
    float a = cardinalAngles[i] * PI / 180.0 - PI/2;
    int x = centerX + cos(a) * (outerRadius + 20);
    int y = centerY + sin(a) * (outerRadius + 20);
    
    gfx->setTextSize(2);
    gfx->setTextColor(cardinalColors[i]);
    gfx->setCursor(x - 6, y - 8);
    gfx->print(cardinals[i]);
  }
  
  // Compass needle - pixel rectangles instead of circles
  float needleAngle = compass.heading * PI / 180.0 - PI/2;
  
  // North pointer (red pixel trail)
  for (int i = 0; i < 8; i++) {
    int x = centerX + cos(needleAngle) * (i * 12);
    int y = centerY + sin(needleAngle) * (i * 12);
    int sz = 8 - i;
    gfx->fillRect(x - sz/2, y - sz/2, sz, sz, COLOR_RED);
  }
  
  // South pointer (dim pixel trail)
  float southAngle = needleAngle + PI;
  for (int i = 0; i < 5; i++) {
    int x = centerX + cos(southAngle) * (i * 10);
    int y = centerY + sin(southAngle) * (i * 10);
    gfx->fillRect(x - 2, y - 2, 5, 5, RGB565(80, 85, 100));
  }
  
  // Center - pixel cross
  gfx->fillRect(centerX - 6, centerY - 6, 12, 12, RGB565(30, 32, 42));
  gfx->fillRect(centerX - 3, centerY - 3, 6, 6, theme->accent);
  
  // Heading display - retro framed
  gfx->fillRect(centerX - 60, 80, 120, 50, RGB565(12, 14, 20));
  gfx->drawRect(centerX - 60, 80, 120, 50, RGB565(40, 45, 60));
  gfx->fillRect(centerX - 60, 80, 5, 5, theme->primary);
  gfx->fillRect(centerX + 55, 80, 5, 5, theme->primary);
  
  gfx->setTextSize(4);
  gfx->setTextColor(theme->primary);
  char headingStr[8];
  sprintf(headingStr, "%3d", (int)compass.heading);
  gfx->setCursor(centerX - 42, 90);
  gfx->print(headingStr);
  gfx->setTextSize(2);
  gfx->print("o");
  
  // Cardinal direction text
  const char* direction = getCardinalDirection(compass.heading);
  gfx->setTextSize(2);
  gfx->setTextColor(theme->accent);
  int dirLen = strlen(direction) * 12;
  gfx->setCursor(centerX - dirLen/2, 140);
  gfx->print(direction);
  
  // Calibration status - retro
  if (!compass.calibrated) {
    gfx->fillRect(50, 395, LCD_WIDTH - 100, 22, RGB565(10, 12, 18));
    gfx->drawRect(50, 395, LCD_WIDTH - 100, 22, RGB565(255, 200, 0));
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(255, 200, 0));
    gfx->setCursor(65, 402);
    gfx->print("Rotate watch to calibrate");
  }
  
  drawSwipeIndicator();
}

// =============================================================================
// COMPASS LOGIC
// =============================================================================

void updateCompassReading() {
  // Read magnetometer data from QMI8658
  // Note: This is a simplified version. Real implementation needs proper I2C reads
  
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(QMI8658_MAG_X_L);
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 6);
  
  if (Wire.available() >= 6) {
    int16_t mx = Wire.read() | (Wire.read() << 8);
    int16_t my = Wire.read() | (Wire.read() << 8);
    int16_t mz = Wire.read() | (Wire.read() << 8);
    
    compass.mag_x = mx;
    compass.mag_y = my;
    compass.mag_z = mz;
    
    compass.heading = calculateHeading(mx, my, mz);
    compass.last_update = millis();
    
    // Simple calibration check
    if (abs(mx) > 50 || abs(my) > 50) {
      compass.calibrated = true;
    }
  }
}

float calculateHeading(float mx, float my, float mz) {
  // Calculate heading from magnetometer data
  // Simplified 2D heading (assumes watch is held flat)
  
  float heading = atan2(my, mx) * 180.0 / PI;
  
  // Normalize to 0-359
  if (heading < 0) {
    heading += 360;
  }
  
  // Adjust for magnetic declination if needed
  // (Varies by location, typically -20 to +20 degrees)
  
  return heading;
}

const char* getCardinalDirection(float heading) {
  // 16-point compass rose
  const char* directions[] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };
  
  int index = (int)((heading + 11.25) / 22.5) % 16;
  return directions[index];
}

// =============================================================================
// TOUCH HANDLING
// =============================================================================

void handleCompassTouch(TouchGesture& gesture) {
  // Swipe UP to exit
  if (gesture.event == TOUCH_SWIPE_UP) {
    returnToAppGrid();
    return;
  }
  
  // Tap to recalibrate
  if (gesture.event == TOUCH_TAP) {
    compass.calibrated = false;
    drawCompassApp();
  }
}
