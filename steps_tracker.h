/*
 * steps_tracker.h - Step Counter & Activity Tracker
 * Uses QMI8658 accelerometer for step counting
 */

#ifndef STEPS_TRACKER_H
#define STEPS_TRACKER_H

#include <Arduino.h>
#include "config.h"

// Step data structure
struct StepsData {
  uint32_t steps_today;
  uint32_t steps_goal;
  float distance_km;
  uint16_t calories_burned;
  uint32_t active_minutes;
  uint8_t current_hour;
  uint16_t hourly_steps[24];
  uint32_t weekly_steps[7];
  unsigned long last_update;
};

// Activity thresholds
struct ActivityThresholds {
  float step_threshold;
  uint16_t step_cooldown_ms;
  float stride_length_m;
};

// Function declarations
void initStepsTracker();
void drawStepsCard();
void forceStepsCardRedraw();  // FIX: Reset internal anti-flicker state for navigation
void drawRetroStatCard(int x, int y, int w, int h, const char* label, float value, const char* unit, uint16_t color, bool showDecimal);
void handleStepsCardTouch(TouchGesture& gesture);
void updateStepCount();
void resetStepsToday();  // Renamed to avoid conflict with hardware.cpp
uint32_t getTodaySteps();
float getDistanceKm();
uint16_t getCaloriesBurned();
void saveStepsData();
void loadStepsData();
void processAccelerometerData(float ax, float ay, float az);
bool detectStep(float magnitude);

#endif
