/*
 * hardware.cpp - Hardware Implementation
 * Power, Sensors, and RTC functionality
 */

#include "hardware.h"
#include "config.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

XPowersAXP2101 PMU;
extern SystemState system_state;

// Power state
static PowerState current_power_state = POWER_ACTIVE;

// Sensor data
static StepData step_data = {0};
static IMUData last_imu_data = {0};
static bool imu_initialized = false;

// RTC data
static WatchTime current_time = {12, 0, 0, 1, 1, 2025, 0};
static int timezone_offset = 0;

// Timer/Stopwatch
static unsigned long stopwatch_start = 0;
static unsigned long stopwatch_elapsed = 0;
static bool stopwatch_running = false;
static int timer_remaining = 0;
static bool timer_running = false;

// =============================================================================
// POWER MANAGEMENT
// =============================================================================

void initializeHardware() {
  Serial.println("[HW] Initializing hardware...");
  
  // Initialize I2C
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);  // 400kHz fast mode
  
  // Initialize I/O Expander (XCA9554) - MUST be done FIRST to power peripherals
  // The Waveshare board uses an I/O expander to control power rails
  Serial.println("[HW] Initializing I/O Expander (XCA9554)...");
  Wire.beginTransmission(EXPANDER_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[HW] XCA9554 found at 0x20");
    
    // Set pins 0, 1, 2 as outputs (config register 0x03, 0 = output)
    Wire.beginTransmission(EXPANDER_ADDR);
    Wire.write(0x03);  // Configuration register
    Wire.write(0xF8);  // Pins 0,1,2 as output, rest input
    Wire.endTransmission();
    
    // Pull all LOW first (reset peripherals)
    Wire.beginTransmission(EXPANDER_ADDR);
    Wire.write(0x01);  // Output register
    Wire.write(0x00);  // All low
    Wire.endTransmission();
    delay(20);
    
    // Pull HIGH to power on peripherals (display, touch, PMU)
    Wire.beginTransmission(EXPANDER_ADDR);
    Wire.write(0x01);  // Output register
    Wire.write(0x07);  // Pins 0,1,2 high
    Wire.endTransmission();
    delay(50);  // Give peripherals time to power up
    
    Serial.println("[HW] I/O Expander configured - peripherals powered on");
  } else {
    Serial.println("[HW] XCA9554 not found - peripherals may not power on");
  }
  
  // Initialize power management
  if (initializePower()) {
    system_state.power_available = true;
  }
  
  // Initialize sensors
  if (initializeSensors()) {
    Serial.println("[HW] Sensors initialized");
  }
  
  // Initialize RTC
  if (initializeRTC()) {
    Serial.println("[HW] RTC initialized");
  }
  
  Serial.println("[HW] Hardware initialization complete");
}

bool initializePower() {
  return initializeAXP2101();
}

bool initializeAXP2101() {
  if (PMU.begin(Wire, AXP2101_ADDR, IIC_SDA, IIC_SCL)) {
    Serial.println("[PMU] AXP2101 initialized");
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    PMU.enableBattDetection();
    PMU.enableBattVoltageMeasure();
    
    // Enable display power rails
    PMU.setALDO1Voltage(1800); PMU.enableALDO1();
    PMU.setALDO2Voltage(2800); PMU.enableALDO2();
    PMU.setALDO3Voltage(3300); PMU.enableALDO3();
    PMU.setALDO4Voltage(3300); PMU.enableALDO4();
    PMU.setBLDO1Voltage(1800); PMU.enableBLDO1();
    PMU.setBLDO2Voltage(3300); PMU.enableBLDO2();
    
    return true;
  }
  return false;
}

BatteryInfo updateBatteryStatus() {
  BatteryInfo info = {0};
  if (system_state.power_available && PMU.isBatteryConnect()) {
    info.percentage = PMU.getBatteryPercent();
    info.voltage_mv = PMU.getBattVoltage();
    info.is_charging = PMU.isCharging();
    info.is_plugged = PMU.isVbusInsertOnSource();
  }
  return info;
}

int getBatteryPercentage() {
  if (system_state.power_available && PMU.isBatteryConnect())
    return PMU.getBatteryPercent();
  return 100;
}

int getBatteryVoltage() {
  if (system_state.power_available && PMU.isBatteryConnect())
    return PMU.getBattVoltage();
  return 4200;
}

bool isCharging() {
  if (system_state.power_available) return PMU.isCharging();
  return false;
}

bool isPluggedIn() {
  if (system_state.power_available) return PMU.isVbusInsertOnSource();
  return false;
}

void setPowerState(PowerState state) {
  current_power_state = state;
}

PowerState getCurrentPowerState() {
  return current_power_state;
}

void enterSleepMode() {
  current_power_state = POWER_SCREEN_OFF;  // FUSION OS: Screen off state
}

void enterDeepSleepMode() {
  current_power_state = POWER_SCREEN_OFF;  // FUSION OS: Screen off before deep sleep
  esp_deep_sleep_start();
}

void wakeFromSleep() {
  current_power_state = POWER_ACTIVE;
}

void setDisplayPower(bool on) {
  if (on) wakeFromSleep();
  else enterSleepMode();
}

void enableLowPowerMode() {
  system_state.low_power_mode = true;
}

void disableLowPowerMode() {
  system_state.low_power_mode = false;
}

void showLowBatteryWarning() {
  Serial.println("[PMU] Low battery warning!");
}

void showChargingNotification() {
  Serial.println("[PMU] Charging started");
}

// =============================================================================
// SENSORS
// =============================================================================

bool initializeSensors() {
  return initializeIMU();
}

bool initializeIMU() {
  Serial.println("[IMU] Initializing QMI8658...");
  
  Wire.beginTransmission(QMI8658_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[IMU] QMI8658 detected");
    imu_initialized = true;
    return true;
  }
  Serial.println("[IMU] QMI8658 not found");
  return false;
}

IMUData readIMU() {
  IMUData data = {0};
  data.timestamp = millis();
  
  if (!imu_initialized) return data;
  
  // Read accelerometer data from QMI8658
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(0x35);  // Accelerometer data register
  if (Wire.endTransmission(false) != 0) return data;
  
  Wire.requestFrom((uint8_t)QMI8658_ADDR, (uint8_t)12);
  if (Wire.available() >= 12) {
    int16_t ax = Wire.read() | (Wire.read() << 8);
    int16_t ay = Wire.read() | (Wire.read() << 8);
    int16_t az = Wire.read() | (Wire.read() << 8);
    int16_t gx = Wire.read() | (Wire.read() << 8);
    int16_t gy = Wire.read() | (Wire.read() << 8);
    int16_t gz = Wire.read() | (Wire.read() << 8);
    
    data.accel_x = ax / 16384.0;
    data.accel_y = ay / 16384.0;
    data.accel_z = az / 16384.0;
    data.gyro_x = gx / 131.0;
    data.gyro_y = gy / 131.0;
    data.gyro_z = gz / 131.0;
  }
  
  last_imu_data = data;
  return data;
}

void calibrateIMU() {
  Serial.println("[IMU] Calibrating...");
  // Collect samples for calibration
}

void updateStepCounter() {
  IMUData imu = readIMU();
  if (detectStep(imu)) {
    step_data.daily_steps++;
    system_state.steps_today = step_data.daily_steps;
  }
}

bool detectStep(IMUData& imu) {
  static float last_magnitude = 1.0;
  static unsigned long last_step_time = 0;
  
  float magnitude = sqrt(imu.accel_x*imu.accel_x + imu.accel_y*imu.accel_y + imu.accel_z*imu.accel_z);
  float delta = abs(magnitude - last_magnitude);
  last_magnitude = magnitude;
  
  if (delta > MIN_STEP_THRESHOLD && millis() - last_step_time > STEP_TIME_WINDOW) {
    last_step_time = millis();
    step_data.last_step_time = last_step_time;
    return true;
  }
  return false;
}

void resetDailySteps() {
  step_data.daily_steps = 0;
  system_state.steps_today = 0;
}

int getDailySteps() {
  return step_data.daily_steps;
}

bool isMoving() {
  float magnitude = sqrt(last_imu_data.accel_x*last_imu_data.accel_x + 
                         last_imu_data.accel_y*last_imu_data.accel_y + 
                         last_imu_data.accel_z*last_imu_data.accel_z);
  return abs(magnitude - 1.0) > 0.1;
}

bool isRunning() {
  return isMoving() && millis() - step_data.last_step_time < 300;
}

String getCurrentActivity() {
  if (isRunning()) return "Running";
  if (isMoving()) return "Walking";
  return "Idle";
}

bool detectWristRaise() {
  return last_imu_data.accel_z < -0.5 && last_imu_data.accel_y > 0.5;
}

bool detectWristFlick() {
  return abs(last_imu_data.gyro_x) > 200;
}

float getHeartRate() {
  return 72 + random(-5, 5);  // Simulated
}

void processSensorData() {
  updateStepCounter();
}

void updateActivityMetrics() {
  step_data.distance_km = step_data.daily_steps * 0.0008;  // ~0.8m per step
  step_data.calories_burned = step_data.daily_steps * 0.04;  // ~0.04 cal per step
}

void updateSensors() {
  processSensorData();
  updateActivityMetrics();
}

// =============================================================================
// RTC
// =============================================================================

// Forward declarations for BCD conversion
uint8_t bcdToDec(uint8_t bcd);
uint8_t decToBcd(uint8_t dec);

bool initializeRTC() {
  Serial.println("[RTC] Initializing PCF85063...");
  
  Wire.beginTransmission(RTC_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[RTC] PCF85063 detected");
    return true;
  }
  Serial.println("[RTC] PCF85063 not found");
  return false;
}

WatchTime getCurrentTime() {
  // Read from PCF85063
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // Seconds register
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7);
    if (Wire.available() >= 7) {
      current_time.second = bcdToDec(Wire.read() & 0x7F);
      current_time.minute = bcdToDec(Wire.read() & 0x7F);
      current_time.hour = bcdToDec(Wire.read() & 0x3F);
      current_time.day = bcdToDec(Wire.read() & 0x3F);
      current_time.weekday = Wire.read() & 0x07;
      current_time.month = bcdToDec(Wire.read() & 0x1F);
      current_time.year = 2000 + bcdToDec(Wire.read());
    }
  }
  return current_time;
}

uint8_t bcdToDec(uint8_t bcd) {
  return (bcd >> 4) * 10 + (bcd & 0x0F);
}

uint8_t decToBcd(uint8_t dec) {
  return ((dec / 10) << 4) | (dec % 10);
}

void setCurrentTime(WatchTime& time) {
  // Try up to 3 times to write to RTC
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(0x04);  // Start at seconds register
    Wire.write(decToBcd(time.second));
    Wire.write(decToBcd(time.minute));
    Wire.write(decToBcd(time.hour));
    Wire.write(decToBcd(time.day));
    Wire.write(time.weekday);
    Wire.write(decToBcd(time.month));
    Wire.write(decToBcd(time.year - 2000));
    
    uint8_t result = Wire.endTransmission();
    
    if (result == 0) {
      // Write successful, verify by reading back
      delay(10);  // Give RTC time to process
      WatchTime verify = getCurrentTime();
      
      if (verify.hour == time.hour && verify.minute == time.minute) {
        current_time = time;
        Serial.printf("[RTC] Time set successfully: %02d:%02d:%02d\n", 
                      time.hour, time.minute, time.second);
        return;
      } else {
        Serial.printf("[RTC] Verification failed (attempt %d): expected %02d:%02d, got %02d:%02d\n",
                      attempt + 1, time.hour, time.minute, verify.hour, verify.minute);
      }
    } else {
      Serial.printf("[RTC] I2C write failed (attempt %d), error code: %d\n", attempt + 1, result);
    }
    
    delay(50);  // Wait before retry
  }
  
  Serial.println("[RTC] ERROR: Failed to set time after 3 attempts!");
  // Still update local time even if RTC write failed
  current_time = time;
}

void syncTimeWithWiFi() {
  configTime(timezone_offset * 3600, 0, "pool.ntp.org");
}

bool isValidTime(WatchTime& time) {
  return time.hour >= 0 && time.hour < 24 &&
         time.minute >= 0 && time.minute < 60 &&
         time.second >= 0 && time.second < 60;
}

String formatTime(WatchTime& time, bool twelve_hour) {
  char buf[16];
  if (twelve_hour) {
    int hour = time.hour % 12;
    if (hour == 0) hour = 12;
    sprintf(buf, "%d:%02d %s", hour, time.minute, time.hour < 12 ? "AM" : "PM");
  } else {
    sprintf(buf, "%02d:%02d", time.hour, time.minute);
  }
  return String(buf);
}

String formatDate(WatchTime& time) {
  char buf[16];
  sprintf(buf, "%02d/%02d/%04d", time.month, time.day, time.year);
  return String(buf);
}

String formatDateTime(WatchTime& time) {
  return formatDate(time) + " " + formatTime(time, false);
}

void setTimezone(int offset_hours) {
  timezone_offset = offset_hours;
}

int getTimezone() {
  return timezone_offset;
}

void setAlarm(int id, int hour, int minute, bool enabled) {
  // Store alarm configuration
}

void checkAlarms() {
  // Check if any alarm should trigger
}

void triggerAlarm(int id) {
  Serial.printf("[RTC] Alarm %d triggered!\n", id);
}

void snoozeAlarm(int id) {
  // Snooze for 5 minutes
}

void dismissAlarm(int id) {
  // Dismiss alarm
}

void startTimer(int minutes, String label) {
  timer_remaining = minutes * 60;
  timer_running = true;
}

void stopTimer() {
  timer_running = false;
  timer_remaining = 0;
}

void pauseTimer() {
  timer_running = false;
}

void resumeTimer() {
  timer_running = true;
}

int getTimerRemaining() {
  return timer_remaining;
}

void startStopwatch() {
  stopwatch_start = millis();
  stopwatch_running = true;
}

void stopStopwatch() {
  if (stopwatch_running) {
    stopwatch_elapsed += millis() - stopwatch_start;
    stopwatch_running = false;
  }
}

void pauseStopwatch() {
  stopStopwatch();
}

void resumeStopwatch() {
  startStopwatch();
}

void resetStopwatch() {
  stopwatch_elapsed = 0;
  stopwatch_running = false;
}

unsigned long getStopwatchTime() {
  if (stopwatch_running)
    return stopwatch_elapsed + (millis() - stopwatch_start);
  return stopwatch_elapsed;
}

void checkTimeBasedEvents() {
  checkAlarms();
  if (timer_running && timer_remaining > 0) {
    static unsigned long last_tick = 0;
    if (millis() - last_tick >= 1000) {
      timer_remaining--;
      last_tick = millis();
      if (timer_remaining == 0) {
        timer_running = false;
        Serial.println("[RTC] Timer finished!");
      }
    }
  }
}
