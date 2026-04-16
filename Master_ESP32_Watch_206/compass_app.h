/*
 * compass_app.h - Digital Compass Application
 * Uses QMI8658 magnetometer for heading
 */

#ifndef COMPASS_APP_H
#define COMPASS_APP_H

#include <Arduino.h>
#include "config.h"

// Compass data
struct CompassData {
  float heading;        // 0-359 degrees
  float mag_x;
  float mag_y;
  float mag_z;
  bool calibrated;
  unsigned long last_update;
};

// Function declarations
void initCompassApp();
void drawCompassApp();
void handleCompassTouch(TouchGesture& gesture);
void updateCompassReading();
float calculateHeading(float mx, float my, float mz);
const char* getCardinalDirection(float heading);

#endif
