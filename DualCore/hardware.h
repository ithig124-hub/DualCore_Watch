/*
 * hardware.h - Hardware Management
 * Consolidates: power.h/cpp, sensors.h/cpp, rtc.h/cpp
 * 
 * AXP2101 PMU, QMI8658 IMU, PCF85063 RTC
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include "types.h"  // Must include types.h for BatteryInfo, PowerState, IMUData
#include "config.h"
#include <Wire.h>
#include <time.h>

// =============================================================================
// POWER MANAGEMENT
// =============================================================================

// Initialize all hardware
void initializeHardware();

bool initializePower();
bool initializeAXP2101();
BatteryInfo updateBatteryStatus();
int getBatteryPercentage();
int getBatteryVoltage();
bool isCharging();
bool isPluggedIn();

void setPowerState(PowerState state);
PowerState getCurrentPowerState();
void enterSleepMode();
void enterDeepSleepMode();
void wakeFromSleep();

void setDisplayPower(bool on);
void enableLowPowerMode();
void disableLowPowerMode();
void showLowBatteryWarning();
void showChargingNotification();

// =============================================================================
// SENSORS (QMI8658 IMU)
// =============================================================================

bool initializeSensors();
bool initializeIMU();
IMUData readIMU();
void calibrateIMU();

void updateStepCounter();
bool detectStep(IMUData& imu);
void resetDailySteps();
int getDailySteps();

bool isMoving();
bool isRunning();
String getCurrentActivity();
bool detectWristRaise();
bool detectWristFlick();

float getHeartRate();
void processSensorData();
void updateActivityMetrics();
void updateSensors();

// =============================================================================
// RTC (PCF85063)
// =============================================================================

bool initializeRTC();
WatchTime getCurrentTime();
void setCurrentTime(WatchTime& time);
void syncTimeWithWiFi();
bool isValidTime(WatchTime& time);

String formatTime(WatchTime& time, bool twelve_hour = false);
String formatDate(WatchTime& time);
String formatDateTime(WatchTime& time);

void setTimezone(int offset_hours);
int getTimezone();

void setAlarm(int id, int hour, int minute, bool enabled);
void checkAlarms();
void triggerAlarm(int id);
void snoozeAlarm(int id);
void dismissAlarm(int id);

void startTimer(int minutes, String label = "");
void stopTimer();
void pauseTimer();
void resumeTimer();
int getTimerRemaining();

void startStopwatch();
void stopStopwatch();
void pauseStopwatch();
void resumeStopwatch();
void resetStopwatch();
unsigned long getStopwatchTime();

void checkTimeBasedEvents();

#endif // HARDWARE_H
