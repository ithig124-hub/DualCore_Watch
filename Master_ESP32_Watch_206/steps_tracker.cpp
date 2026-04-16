/*
 * steps_tracker.cpp - Step Counter Implementation
 * FIXED: Type mismatch in max() function at line 196
 * FUSION OS: Added XP rewards for steps
 */

#include "steps_tracker.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"
#include "hardware.h"
#include "xp_system.h"  // FUSION OS: XP rewards
#include <Wire.h>
#include "sd_preferences.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

StepsData steps_data = {0, 10000, 0.0, 0, 0, 0, {}, {}, 0};
// ULTRA SENSITIVE: Lowest possible threshold (0.15), shortest cooldown (100ms)
// Any wrist movement will register as a step
ActivityThresholds activity_config = {0.15, 100, 0.65};

static float last_magnitude = 1.0;
static unsigned long last_step_time = 0;

// FUSION OS: XP tracking
static int last_xp_steps = 0;       // Last step count when XP was awarded
static bool goal_reached_today = false;

SDPreferences prefs;

// =============================================================================
// FIX: External flag to force full redraw of steps card
// This is needed because drawStepsCard() uses static vars for anti-flicker,
// but when navigating BACK to the steps screen, the screen gets cleared
// while the static vars still say "no redraw needed" → black screen!
// =============================================================================
static bool g_force_steps_card_redraw = true;

void forceStepsCardRedraw() {
  g_force_steps_card_redraw = true;
  Serial.println("[Steps] Force full redraw requested");
}

void initStepsTracker() {
  loadStepsData();
  WatchTime current_time = getCurrentTime();
  if (current_time.hour == 0 && steps_data.current_hour != 0) {
    for (int i = 6; i > 0; i--) {
      steps_data.weekly_steps[i] = steps_data.weekly_steps[i-1];
    }
    steps_data.weekly_steps[0] = steps_data.steps_today;
    resetStepsToday();
  }
  steps_data.current_hour = current_time.hour;
  Serial.printf("[Steps] Today: %d steps\n", steps_data.steps_today);
  
  // CRITICAL: Sync to system_state on initialization
  system_state.steps_today = steps_data.steps_today;
  system_state.step_goal = steps_data.steps_goal;
}

void drawStepsCard() {
  // ========================================
  // RETRO ANIME STEP TRACKER - CRT STYLE (Optimized for 410x502)
  // ANTI-FLICKER: Only redraw when steps or goal changes
  // ========================================
  
  // ANTI-FLICKER: Track changes
  static bool needs_redraw = true;
  static int last_steps = -1;
  static int last_goal = -1;
  
  // FIX: Check external force redraw flag (set when navigating to this screen)
  if (g_force_steps_card_redraw) {
    needs_redraw = true;
    g_force_steps_card_redraw = false;
  }
  
  // Check if data changed
  bool data_changed = (steps_data.steps_today != last_steps) ||
                      (steps_data.steps_goal != last_goal);
  
  if (!needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  needs_redraw = false;
  last_steps = steps_data.steps_today;
  last_goal = steps_data.steps_goal;
  
  // CRITICAL: Full screen clear to prevent overlap from watchface
  gfx->fillScreen(RGB565(2, 2, 5));  // Deep AMOLED black with slight blue tint
  
  ThemeColors* theme = getCurrentTheme();
  
  // === RETRO CRT SCAN LINES (subtle) ===
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(5, 5, 8));
  }
  
  // === RETRO HEADER WITH PIXEL BORDER - Taller ===
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  
  // Pixel-style border (retro game aesthetic)
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  
  // Header text with retro glow - larger
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(40, 45, 60));  // Shadow
  gfx->setCursor(LCD_WIDTH/2 - 72 + 2, 14);
  gfx->print("ACTIVITY");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH/2 - 72, 12);
  gfx->print("ACTIVITY");
  
  int centerX = LCD_WIDTH / 2;  // 205
  int centerY = 185;  // Adjusted for taller display
  int radius = 100;   // Larger radius
  float progress = (float)steps_data.steps_today / steps_data.steps_goal;
  if (progress > 1.0) progress = 1.0;
  
  // === RETRO CIRCULAR GAUGE (Pixel Art Style) - Larger ===
  // Outer pixel ring frame
  for (int i = 0; i < 360; i += 10) {
    float a = i * PI / 180.0 - PI/2;
    int x = centerX + cos(a) * (radius + 10);
    int y = centerY + sin(a) * (radius + 10);
    gfx->fillRect(x - 3, y - 3, 6, 6, RGB565(25, 30, 40));  // Pixel dots
  }
  
  // Background ring - retro segment style
  for (int i = 0; i < 360; i += 6) {
    float a = i * PI / 180.0 - PI/2;
    int x = centerX + cos(a) * radius;
    int y = centerY + sin(a) * radius;
    gfx->fillRect(x - 4, y - 4, 8, 8, RGB565(20, 22, 30));  // Square segments
  }
  
  // Progress arc - glowing retro style
  int progressDeg = progress * 360;
  for (int i = 0; i < progressDeg; i += 6) {
    float a = i * PI / 180.0 - PI/2;
    int x = centerX + cos(a) * radius;
    int y = centerY + sin(a) * radius;
    
    // Color based on progress with retro palette
    uint16_t color;
    if (progress < 0.3) {
      color = RGB565(255, 60, 80);      // Retro red
    } else if (progress < 0.6) {
      color = RGB565(255, 180, 40);     // Retro amber
    } else if (progress < 0.9) {
      color = RGB565(80, 255, 120);     // Retro green
    } else {
      color = RGB565(120, 200, 255);    // Retro cyan (goal reached!)
    }
    
    // Glow effect (outer)
    gfx->fillRect(x - 5, y - 5, 10, 10, RGB565(color >> 12, (color >> 6) & 0x1F, color & 0x1F));
    // Core segment
    gfx->fillRect(x - 4, y - 4, 8, 8, color);
  }
  
  // === CENTER STEP COUNT - RETRO DIGITAL STYLE - Larger ===
  // Dark panel behind numbers
  gfx->fillRect(centerX - 90, centerY - 40, 180, 85, RGB565(8, 10, 15));
  gfx->drawRect(centerX - 90, centerY - 40, 180, 85, RGB565(40, 50, 70));
  
  // Big step number with pixel shadow
  gfx->setTextSize(5);
  char stepsStr[10];
  sprintf(stepsStr, "%d", steps_data.steps_today);
  int strLen = strlen(stepsStr) * 30;
  
  // Shadow
  gfx->setTextColor(RGB565(20, 25, 35));
  gfx->setCursor(centerX - strLen/2 + 2, centerY - 28 + 2);
  gfx->print(stepsStr);
  
  // Main text - bright retro color
  gfx->setTextColor(theme->primary);
  gfx->setCursor(centerX - strLen/2, centerY - 28);
  gfx->print(stepsStr);
  
  // "STEPS" label - larger
  gfx->setTextSize(2);
  gfx->setTextColor(theme->accent);
  gfx->setCursor(centerX - 30, centerY + 15);
  gfx->print("STEPS");
  
  // Goal with retro bracket style
  gfx->setTextColor(RGB565(100, 110, 130));
  gfx->setCursor(centerX - 52, centerY + 35);
  gfx->printf("[ GOAL: %d ]", steps_data.steps_goal);
  
  // === RETRO STAT CARDS (Pixel Frame Style) ===
  int cardY = 265;
  int cardW = 108;
  int cardH = 65;
  int cardGap = 8;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Distance Card
  drawRetroStatCard(cardStartX, cardY, cardW, cardH, 
                    "DISTANCE", steps_data.distance_km, "km", 
                    RGB565(80, 200, 255), true);
  
  // Calories Card  
  drawRetroStatCard(cardStartX + cardW + cardGap, cardY, cardW, cardH,
                    "CALORIES", (float)steps_data.calories_burned, "kcal",
                    RGB565(255, 150, 80), false);
  
  // Active Minutes Card
  drawRetroStatCard(cardStartX + 2*(cardW + cardGap), cardY, cardW, cardH,
                    "ACTIVE", (float)steps_data.active_minutes, "min",
                    RGB565(100, 255, 150), false);
  
  // === RETRO WEEKLY CHART (Bar Graph) ===
  int chartY = 350;
  int barW = 42;
  int barMaxH = 55;
  int chartStartX = (LCD_WIDTH - (7 * barW + 6 * 6)) / 2;
  
  // Chart frame
  gfx->fillRect(chartStartX - 10, chartY - 5, 7 * barW + 6 * 6 + 20, barMaxH + 35, RGB565(10, 12, 18));
  gfx->drawRect(chartStartX - 10, chartY - 5, 7 * barW + 6 * 6 + 20, barMaxH + 35, RGB565(40, 45, 60));
  
  // "7-DAY" label
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(120, 130, 150));
  gfx->setCursor(chartStartX, chartY);
  gfx->print("7-DAY HISTORY");
  
  const char* days[] = {"M", "T", "W", "T", "F", "S", "S"};
  
  for (int i = 0; i < 7; i++) {
    int x = chartStartX + i * (barW + 6);
    int y = chartY + 15;
    uint32_t daySteps = steps_data.weekly_steps[6 - i];
    // FIX: Cast to same type to avoid max() template deduction error
    int barH = (daySteps * barMaxH) / max((uint32_t)1, steps_data.steps_goal);
    if (barH > barMaxH) barH = barMaxH;
    if (barH < 3) barH = 3;  // Minimum visible height
    
    // Bar background (empty part)
    gfx->fillRect(x, y, barW - 2, barMaxH, RGB565(18, 20, 28));
    
    // Bar fill - retro gradient effect
    uint16_t barColor;
    if (i == 6) {
      // Today - highlighted with theme color
      barColor = theme->primary;
      // Glow effect for today
      gfx->fillRect(x - 1, y + (barMaxH - barH) - 1, barW, barH + 2, RGB565(theme->primary >> 12, 20, 30));
    } else {
      barColor = RGB565(60, 70, 100);
    }
    gfx->fillRect(x, y + (barMaxH - barH), barW - 2, barH, barColor);
    
    // Day label
    gfx->setTextSize(1);
    gfx->setTextColor(i == 6 ? theme->accent : RGB565(80, 90, 110));
    gfx->setCursor(x + barW/2 - 3, y + barMaxH + 4);
    gfx->print(days[i]);
  }
  
  drawSwipeIndicator();
}

// Helper function for retro stat cards
void drawRetroStatCard(int x, int y, int w, int h, const char* label, float value, const char* unit, uint16_t color, bool showDecimal) {
  // Card background with pixel border
  gfx->fillRect(x, y, w, h, RGB565(12, 14, 20));
  
  // Pixel corner decorations (retro game style)
  gfx->fillRect(x, y, 4, 4, color);
  gfx->fillRect(x + w - 4, y, 4, 4, color);
  gfx->fillRect(x, y + h - 4, 4, 4, color);
  gfx->fillRect(x + w - 4, y + h - 4, 4, 4, color);
  
  // Border
  gfx->drawRect(x, y, w, h, RGB565(40, 45, 60));
  
  // Label
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 110, 130));
  gfx->setCursor(x + 8, y + 8);
  gfx->print(label);
  
  // Value with glow effect
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(color >> 12, (color >> 6) & 0x3F, color & 0x1F));  // Dimmed shadow
  if (showDecimal) {
    gfx->setCursor(x + 9, y + 26);
    gfx->printf("%.1f", value);
  } else {
    gfx->setCursor(x + 9, y + 26);
    gfx->printf("%d", (int)value);
  }
  
  gfx->setTextColor(color);
  if (showDecimal) {
    gfx->setCursor(x + 8, y + 25);
    gfx->printf("%.1f", value);
  } else {
    gfx->setCursor(x + 8, y + 25);
    gfx->printf("%d", (int)value);
  }
  
  // Unit
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 90, 110));
  gfx->setCursor(x + 8, y + h - 15);
  gfx->print(unit);
}

void updateStepCount() {
  // =====================================================
  // USE QMI8658 HARDWARE PEDOMETER - SUPER SENSITIVE MODE
  // Lowered all thresholds for maximum step detection
  // =====================================================
  
  static bool pedometer_initialized = false;
  static uint32_t last_hw_steps = 0;
  
  if (!pedometer_initialized) {
    // Configure QMI8658 pedometer - SUPER SENSITIVE SETTINGS
    
    // Step 1: Configure pedometer parameters via CAL registers
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0B);  // CAL1_L - ped_sample_cnt low
    Wire.write(0x20);  // 32 samples batch (was 80) - faster response
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0C);  // CAL1_H - ped_sample_cnt high  
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0D);  // CAL2_L - peak2peak threshold low (20mg = 0x14) ULTRA SENSITIVE
    Wire.write(0x14);  // Was 0x32 (50mg) - now 20mg for any tiny movement!
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0E);  // CAL2_H - peak2peak threshold high
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0F);  // CAL3_L - peak threshold low (15mg = 0x0F) ULTRA SENSITIVE
    Wire.write(0x0F);  // Was 0x1E (30mg) - now 15mg for any tiny movement!
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x10);  // CAL3_H - peak threshold high
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Configure timing via CTRL9 command
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x11);  // CAL4_H
    Wire.write(0x01);  // First phase config
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0A);  // CTRL9 - send CONFIGURE_PEDOMETER command
    Wire.write(0x0D);
    Wire.endTransmission();
    delay(10);
    
    // Phase 2: Timing parameters - FASTER DETECTION
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0B);  // CAL1_L - time_up low (0.5s @ 50Hz = 25)
    Wire.write(0x19);  // Was 0x28 (0.8s) - now 0.5s for even faster step window
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0C);  // CAL1_H - time_up high
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0D);  // CAL2_L - time_low (0.1s @ 50Hz = 5)
    Wire.write(0x05);  // Was 0x07 (0.15s) - even faster minimum step time
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0E);  // CAL2_H - time_cnt_entry (1 step to confirm) INSTANT
    Wire.write(0x01);  // Was 0x02 (2 steps) - now just 1 step to start counting!
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0F);  // CAL3_L - precision (0)
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x10);  // CAL3_H - sig_count (report every 1 step) INSTANT
    Wire.write(0x01);  // Was 0x04 (every 4 steps) - now report EVERY step!
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x11);  // CAL4_H
    Wire.write(0x02);  // Second phase config
    Wire.endTransmission();
    
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x0A);  // CTRL9 - send CONFIGURE_PEDOMETER command
    Wire.write(0x0D);
    Wire.endTransmission();
    delay(10);
    
    // Enable pedometer in CTRL8
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x08);  // CTRL8
    uint8_t ctrl8 = 0x80 | 0x10;  // STATUS_INT handshake + Pedometer enable
    Wire.write(ctrl8);
    Wire.endTransmission();
    
    pedometer_initialized = true;
    Serial.println("[Steps] QMI8658 SUPER SENSITIVE pedometer initialized");
  }
  
  // Read step count from QMI8658 pedometer registers (0x58, 0x59, 0x5A)
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(0x58);  // STEP_CNT_LOW
  Wire.endTransmission(false);
  Wire.requestFrom(QMI8658_ADDR, 3);
  
  if (Wire.available() >= 3) {
    uint8_t low = Wire.read();
    uint8_t mid = Wire.read();
    uint8_t high = Wire.read();
    
    uint32_t hw_steps = (uint32_t)low | ((uint32_t)mid << 8) | ((uint32_t)high << 16);
    
    // Add new steps to today's count
    if (hw_steps > last_hw_steps) {
      uint32_t new_steps = hw_steps - last_hw_steps;
      steps_data.steps_today += new_steps;
      steps_data.hourly_steps[steps_data.current_hour] += new_steps;
      
      if (new_steps > 0) {
        steps_data.active_minutes++;
        Serial.printf("[Steps] +%d steps (total: %d)\n", new_steps, steps_data.steps_today);
      }
    }
    last_hw_steps = hw_steps;
  }
  
  // Calculate derived metrics
  steps_data.distance_km = steps_data.steps_today * activity_config.stride_length_m / 1000.0;
  steps_data.calories_burned = steps_data.steps_today * 0.04;
  
  // CRITICAL: Sync to system_state so UI can display steps
  system_state.steps_today = steps_data.steps_today;
  
  // FUSION OS: XP Rewards for steps
  // Award 1 XP per 100 steps
  if (steps_data.steps_today - last_xp_steps >= 100) {
    int xp_gained = (steps_data.steps_today - last_xp_steps) / 100;
    gainExperience(xp_gained, "Walking");
    last_xp_steps = steps_data.steps_today;
  }
  
  // FUSION OS: Big XP bonus when daily goal is reached
  if (steps_data.steps_today >= steps_data.steps_goal && !goal_reached_today) {
    gainExperience(XP_DAILY_GOAL_BONUS, "Daily Step Goal!");
    goal_reached_today = true;
    Serial.println("[Steps] 🎉 Daily goal reached! +50 XP bonus");
  }
  
  // Save periodically
  if (steps_data.steps_today % 100 == 0 && steps_data.steps_today > 0) {
    saveStepsData();
  }
}

void processAccelerometerData(float ax, float ay, float az) {
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  
  if (detectStep(magnitude)) {
    steps_data.steps_today++;
    steps_data.hourly_steps[steps_data.current_hour]++;
    
    if (millis() - last_step_time < 60000) {
      steps_data.active_minutes++;
    }
    last_step_time = millis();
  }
  last_magnitude = magnitude;
}

bool detectStep(float magnitude) {
  unsigned long now = millis();
  if (now - last_step_time < activity_config.step_cooldown_ms) return false;
  
  float delta = magnitude - last_magnitude;
  return (delta > activity_config.step_threshold);
}

void resetStepsToday() {
  steps_data.steps_today = 0;
  steps_data.distance_km = 0.0;
  steps_data.calories_burned = 0;
  steps_data.active_minutes = 0;
  memset(steps_data.hourly_steps, 0, sizeof(steps_data.hourly_steps));
  
  // FUSION OS: Reset XP tracking for new day
  last_xp_steps = 0;
  goal_reached_today = false;
  
  saveStepsData();
}

uint32_t getTodaySteps() { return steps_data.steps_today; }
float getDistanceKm() { return steps_data.distance_km; }
uint16_t getCaloriesBurned() { return steps_data.calories_burned; }

void saveStepsData() {
  prefs.begin("steps", false);
  prefs.putUInt("steps_today", steps_data.steps_today);
  prefs.putUInt("steps_goal", steps_data.steps_goal);
  prefs.putFloat("distance", steps_data.distance_km);
  for (int i = 0; i < 7; i++) {
    char key[12];
    sprintf(key, "week_%d", i);
    prefs.putUInt(key, steps_data.weekly_steps[i]);
  }
  prefs.end();
}

void loadStepsData() {
  prefs.begin("steps", true);
  steps_data.steps_today = prefs.getUInt("steps_today", 0);
  steps_data.steps_goal = prefs.getUInt("steps_goal", 10000);
  steps_data.distance_km = prefs.getFloat("distance", 0.0);
  for (int i = 0; i < 7; i++) {
    char key[12];
    sprintf(key, "week_%d", i);
    steps_data.weekly_steps[i] = prefs.getUInt(key, 0);
  }
  prefs.end();
}

void handleStepsCardTouch(TouchGesture& gesture) {
  // Swipes are now handled by main navigation loop
  // Steps is part of: Watchface → Steps → App Grid → Stats → loop
  
  // Only handle tap to change goal
  if (gesture.event == TOUCH_TAP && gesture.y >= 180 && gesture.y <= 210) {
    uint32_t goals[] = {5000, 10000, 15000, 20000};
    for (int i = 0; i < 4; i++) {
      if (steps_data.steps_goal == goals[i]) {
        steps_data.steps_goal = goals[(i + 1) % 4];
        break;
      }
    }
    saveStepsData();
    drawStepsCard();
  }
}
