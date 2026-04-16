/*
 * training.cpp - Training Mini-Games Implementation (FIXED)
 * Complete training dojo with XP rewards
 * 
 * FIXES:
 * - Touch handling now properly starts games
 * - Back button returns to app grid
 * - Improved UI
 */

#include "training.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "xp_system.h"  // FUSION OS: For gainExperience() and XP rewards
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Training state
TrainingScore current_training_score;
TrainingType current_training_type;
bool training_active = false;
int training_round = 0;
unsigned long training_start_time = 0;

// Reflex test state
static int reflex_current_button = -1;
static unsigned long reflex_flash_time = 0;
static int reflex_times[10];
static int reflex_round = 0;
static bool reflex_waiting = false;

// Target shoot state
static int target_x[10], target_y[10];
static bool target_active[10];
static int targets_hit = 0;
static int targets_missed = 0;
static unsigned long target_timer = 0;

// Speed tap state
static int tap_count = 0;
static unsigned long speed_tap_end_time = 0;
static int last_combo = 0;

// Memory state
static int memory_pattern[20];
static int memory_length = 3;
static int memory_input_index = 0;
static bool memory_showing = false;
static unsigned long memory_show_time = 0;

// =============================================================================
// TRAINING SYSTEM
// =============================================================================

void initTrainingSystem() {
  Serial.println("[Training] Initializing training system...");
  loadTrainingProgress();
}

void saveTrainingProgress() {
  Serial.println("[Training] Progress saved");
}

bool loadTrainingProgress() {
  return false;
}

void startTrainingGame(TrainingType type) {
  Serial.printf("[Training] Starting game type: %d\n", type);
  
  current_training_type = type;
  training_active = true;
  training_round = 0;
  training_start_time = millis();
  
  current_training_score.type = type;
  current_training_score.score = 0;
  current_training_score.xp_earned = 0;
  current_training_score.best_time_ms = 9999;
  current_training_score.combo_count = 0;
  
  switch(type) {
    case TRAINING_REFLEX:    initReflexTest(); break;
    case TRAINING_TARGET:    initTargetShoot(); break;
    case TRAINING_SPEED_TAP: initSpeedTap(); break;
    case TRAINING_MEMORY:    initTrainingMemory(); break;
  }
  
  system_state.current_screen = SCREEN_TRAINING;
}

void updateTrainingGame() {
  if (!training_active) return;
  
  switch(current_training_type) {
    case TRAINING_REFLEX:    updateReflexTest(); break;
    case TRAINING_TARGET:    updateTargetShoot(); break;
    case TRAINING_SPEED_TAP: updateSpeedTap(); break;
    case TRAINING_MEMORY:    updateTrainingMemory(); break;
  }
}

void endTrainingGame() {
  training_active = false;
  
  bool perfect = (current_training_score.score > 80);
  int xp = calculateTrainingXP(current_training_type, current_training_score.score, perfect);
  applyStreakBonus(xp);
  
  current_training_score.xp_earned = xp;
  gainExperience(xp, "Training");
  
  updateTrainingStreak();
  saveTrainingProgress();
  
  drawTrainingResults(current_training_score);
}

bool isTrainingActive() {
  return training_active;
}

int calculateTrainingXP(TrainingType type, int score, bool perfect) {
  int base_xp = TRAINING_XP_PER_GAME_MIN + (score * (TRAINING_XP_PER_GAME_MAX - TRAINING_XP_PER_GAME_MIN) / 100);
  if (perfect) base_xp += TRAINING_PERFECT_BONUS;
  return base_xp;
}

void applyStreakBonus(int& xp) {
  int streak = system_state.training_streak;
  if (streak >= 30) xp *= 2.0;
  else if (streak >= 14) xp *= 1.75;
  else if (streak >= 7) xp *= 1.5;
  else if (streak >= 3) xp *= 1.25;
}

int getTrainingStreak() {
  return system_state.training_streak;
}

void updateTrainingStreak() {
  system_state.training_streak++;
}

// =============================================================================
// REFLEX TEST
// =============================================================================

void initReflexTest() {
  Serial.println("[Training] Starting Reflex Test");
  reflex_round = 0;
  reflex_current_button = -1;
  reflex_waiting = false;
  for (int i = 0; i < 10; i++) reflex_times[i] = 0;
  drawReflexTest();
}

void updateReflexTest() {
  if (reflex_round >= 5) {
    // Calculate average
    int total = 0;
    int best = 9999;
    for (int i = 0; i < 5; i++) {
      total += reflex_times[i];
      if (reflex_times[i] < best) best = reflex_times[i];
    }
    current_training_score.score = max(0, 100 - (total / 5) / 10);
    current_training_score.best_time_ms = best;
    endTrainingGame();
    return;
  }
  
  if (!reflex_waiting && millis() - reflex_flash_time > 2000 + random(1000, 3000)) {
    reflex_current_button = random(0, 4);
    reflex_waiting = true;
    reflex_flash_time = millis();
    drawReflexTest();
  }
}

void drawReflexTest() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(80, 20);
  gfx->print("REFLEX TEST");
  
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(1);
  gfx->setCursor(100, 50);
  gfx->printf("Round: %d/5", reflex_round + 1);
  
  // 4 buttons in 2x2 grid
  int btnSize = 100;
  int gap = 20;
  int startX = (LCD_WIDTH - 2 * btnSize - gap) / 2;
  int startY = 120;
  
  for (int i = 0; i < 4; i++) {
    int col = i % 2;
    int row = i / 2;
    int x = startX + col * (btnSize + gap);
    int y = startY + row * (btnSize + gap);
    
    bool lit = (i == reflex_current_button && reflex_waiting);
    drawTrainingButton(x, y, btnSize, i, lit, false);
  }
  
  if (!reflex_waiting) {
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setTextSize(1);
    gfx->setCursor(100, 380);
    gfx->print("Wait for the light...");
  } else {
    gfx->setTextColor(getCurrentTheme()->accent);
    gfx->setTextSize(2);
    gfx->setCursor(130, 380);
    gfx->print("TAP NOW!");
  }
}

void handleReflexTouch(int x, int y) {
  if (!reflex_waiting) return;
  
  int btnSize = 100;
  int gap = 20;
  int startX = (LCD_WIDTH - 2 * btnSize - gap) / 2;
  int startY = 120;
  
  for (int i = 0; i < 4; i++) {
    int col = i % 2;
    int row = i / 2;
    int bx = startX + col * (btnSize + gap);
    int by = startY + row * (btnSize + gap);
    
    if (x >= bx && x < bx + btnSize && y >= by && y < by + btnSize) {
      if (i == reflex_current_button) {
        int reaction_time = millis() - reflex_flash_time;
        reflex_times[reflex_round] = reaction_time;
        reflex_round++;
        reflex_waiting = false;
        reflex_current_button = -1;
        Serial.printf("[Training] Reaction time: %dms\n", reaction_time);
      }
      break;
    }
  }
  
  drawReflexTest();
}

// =============================================================================
// TARGET SHOOT
// =============================================================================

void initTargetShoot() {
  Serial.println("[Training] Starting Target Shoot");
  targets_hit = 0;
  targets_missed = 0;
  target_timer = millis();
  
  for (int i = 0; i < 10; i++) {
    target_active[i] = false;
  }
  
  // Spawn initial targets
  for (int i = 0; i < 3; i++) {
    target_x[i] = random(50, LCD_WIDTH - 100);
    target_y[i] = random(100, 350);
    target_active[i] = true;
  }
  
  drawTargetShoot();
}

void spawnNewTargets(int count) {
  for (int i = 0; i < count && i < 10; i++) {
    if (!target_active[i]) {
      target_x[i] = random(50, LCD_WIDTH - 100);
      target_y[i] = random(100, 350);
      target_active[i] = true;
    }
  }
}

void updateTargetShoot() {
  if (millis() - target_timer > 15000) {
    current_training_score.score = (targets_hit * 100) / max(1, targets_hit + targets_missed);
    endTrainingGame();
    return;
  }
  
  int active = 0;
  for (int i = 0; i < 10; i++) if (target_active[i]) active++;
  if (active == 0) {
    // Spawn new targets
    for (int i = 0; i < 3; i++) {
      target_x[i] = random(50, LCD_WIDTH - 100);
      target_y[i] = random(100, 350);
      target_active[i] = true;
    }
    drawTargetShoot();  // Only redraw on new spawn (fixes flicker)
  }
  // Timer update only - no full redraw every frame
  static unsigned long lastTimerDraw = 0;
  if (millis() - lastTimerDraw > 1000) {
    lastTimerDraw = millis();
    int remaining = max(0, 15 - (int)((millis() - target_timer) / 1000));
    drawTrainingTimer(remaining);
  }
}

void drawTargetShoot() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(70, 20);
  gfx->print("TARGET SHOOT");
  
  int remaining = max(0, 15 - (int)((millis() - target_timer) / 1000));
  drawTrainingTimer(remaining);
  drawTrainingScore(targets_hit);
  
  // Draw targets
  for (int i = 0; i < 10; i++) {
    if (target_active[i]) {
      gfx->fillCircle(target_x[i], target_y[i], 30, COLOR_RED);
      gfx->drawCircle(target_x[i], target_y[i], 30, COLOR_WHITE);
      gfx->fillCircle(target_x[i], target_y[i], 15, COLOR_WHITE);
      gfx->fillCircle(target_x[i], target_y[i], 8, COLOR_RED);
    }
  }
}

void handleTargetTouch(int x, int y) {
  for (int i = 0; i < 10; i++) {
    if (target_active[i]) {
      int dx = x - target_x[i];
      int dy = y - target_y[i];
      if (dx*dx + dy*dy < 30*30) {
        target_active[i] = false;
        targets_hit++;
        Serial.printf("[Training] Target hit! Total: %d\n", targets_hit);
        break;
      }
    }
  }
  drawTargetShoot();
}

// =============================================================================
// SPEED TAP
// =============================================================================

void initSpeedTap() {
  Serial.println("[Training] Starting Speed Tap");
  tap_count = 0;
  speed_tap_end_time = millis() + 10000;
  last_combo = 0;
  drawSpeedTap();
}

void updateSpeedTap() {
  if (millis() > speed_tap_end_time) {
    current_training_score.score = min(100, tap_count * 2);
    current_training_score.combo_count = last_combo;
    endTrainingGame();
    return;
  }
  // Only update timer display, not full redraw (fixes flicker)
  static unsigned long lastTimerDraw = 0;
  if (millis() - lastTimerDraw > 500) {
    lastTimerDraw = millis();
    int remaining = max(0, (int)(speed_tap_end_time - millis()) / 1000);
    drawTrainingTimer(remaining);
  }
}

void drawSpeedTap() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(90, 20);
  gfx->print("SPEED TAP");
  
  int remaining = max(0, (int)(speed_tap_end_time - millis()) / 1000);
  drawTrainingTimer(remaining);
  
  // Big tap zone
  int btnY = 120;
  int btnH = 200;
  gfx->fillRect(40, btnY, LCD_WIDTH - 80, btnH, getCurrentTheme()->primary);
  gfx->drawRect(40, btnY, LCD_WIDTH - 80, btnH, COLOR_WHITE);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(LCD_WIDTH/2 - 50, btnY + 70);
  gfx->printf("%d", tap_count);
  
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 30, btnY + 150);
  gfx->print("TAP HERE!");
}

void handleSpeedTapTouch(int x, int y) {
  if (y >= 120 && y < 320) {
    tap_count++;
    last_combo++;
    Serial.printf("[Training] Tap count: %d\n", tap_count);
  }
  drawSpeedTap();
}

// =============================================================================
// MEMORY MATCH
// =============================================================================

void initTrainingMemory() {
  Serial.println("[Training] Starting Memory Match");
  memory_length = 3;
  memory_input_index = 0;
  memory_showing = true;
  memory_show_time = millis();
  
  for (int i = 0; i < memory_length; i++) {
    memory_pattern[i] = random(0, 4);
  }
  
  drawTrainingMemory();
}

void updateTrainingMemory() {
  if (memory_showing) {
    int idx = (millis() - memory_show_time) / 800;
    if (idx >= memory_length) {
      memory_showing = false;
      memory_input_index = 0;
    }
    drawTrainingMemory();
  }
}

void drawTrainingMemory() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(70, 20);
  gfx->print("MEMORY MATCH");
  
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(1);
  gfx->setCursor(100, 50);
  gfx->printf("Pattern length: %d", memory_length);
  
  // 4 buttons
  int btnSize = 100;
  int gap = 20;
  int startX = (LCD_WIDTH - 2 * btnSize - gap) / 2;
  int startY = 120;
  
  int showing_idx = -1;
  if (memory_showing) {
    showing_idx = (millis() - memory_show_time) / 800;
    if (showing_idx < memory_length) showing_idx = memory_pattern[showing_idx];
    else showing_idx = -1;
  }
  
  for (int i = 0; i < 4; i++) {
    int col = i % 2;
    int row = i / 2;
    int x = startX + col * (btnSize + gap);
    int y = startY + row * (btnSize + gap);
    
    drawTrainingButton(x, y, btnSize, i, i == showing_idx, false);
  }
  
  if (memory_showing) {
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setTextSize(1);
    gfx->setCursor(90, 380);
    gfx->print("Watch the pattern...");
  } else {
    gfx->setTextColor(getCurrentTheme()->accent);
    gfx->setTextSize(1);
    gfx->setCursor(80, 380);
    gfx->printf("Repeat! (%d/%d)", memory_input_index, memory_length);
  }
}

void handleMemoryTouch(int x, int y) {
  if (memory_showing) return;
  
  int btnSize = 100;
  int gap = 20;
  int startX = (LCD_WIDTH - 2 * btnSize - gap) / 2;
  int startY = 120;
  
  for (int i = 0; i < 4; i++) {
    int col = i % 2;
    int row = i / 2;
    int bx = startX + col * (btnSize + gap);
    int by = startY + row * (btnSize + gap);
    
    if (x >= bx && x < bx + btnSize && y >= by && y < by + btnSize) {
      if (i == memory_pattern[memory_input_index]) {
        memory_input_index++;
        if (memory_input_index >= memory_length) {
          memory_length++;
          current_training_score.score = memory_length * 15;
          
          if (memory_length > 8) {
            endTrainingGame();
          } else {
            for (int j = 0; j < memory_length; j++) {
              memory_pattern[j] = random(0, 4);
            }
            memory_showing = true;
            memory_show_time = millis();
            memory_input_index = 0;
          }
        }
      } else {
        current_training_score.score = memory_length * 10;
        endTrainingGame();
      }
      break;
    }
  }
  
  drawTrainingMemory();
}

// =============================================================================
// TRAINING UI - IMPROVED
// =============================================================================

void drawTrainingMenu() {
  // ANTI-FLICKER: Only redraw when streak changes
  static bool needs_redraw = true;
  static int last_streak = -1;
  
  // Check if data changed
  bool data_changed = (system_state.training_streak != last_streak);
  
  if (!needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  needs_redraw = false;
  last_streak = system_state.training_streak;
  
  gfx->fillScreen(RGB565(2, 2, 5));
  
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(60, 14);
  gfx->print("TRAINING DOJO");
  
  // Streak bar with multiplier
  int streakY = 55;
  gfx->fillRect(20, streakY, LCD_WIDTH - 40, 28, RGB565(20, 18, 10));
  gfx->drawRect(20, streakY, LCD_WIDTH - 40, 28, RGB565(255, 180, 50));
  gfx->fillRect(20, streakY, 5, 5, COLOR_GOLD);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(30, streakY + 5);
  gfx->printf("STREAK: %d", system_state.training_streak);
  // Multiplier
  const char* mult = "1x";
  if (system_state.training_streak >= 30) mult = "2x XP!";
  else if (system_state.training_streak >= 14) mult = "1.75x";
  else if (system_state.training_streak >= 7) mult = "1.5x";
  else if (system_state.training_streak >= 3) mult = "1.25x";
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(255, 220, 100));
  gfx->setCursor(LCD_WIDTH - 80, streakY + 10);
  gfx->print(mult);
  
  // Training game buttons - improved cards
  const char* games[] = {"REFLEX TEST", "TARGET SHOOT", "SPEED TAP", "MEMORY MATCH"};
  const char* descs[] = {"Test your reaction speed!", "Hit all the targets!", "Tap as fast as you can!", "Remember the pattern!"};
  const char* icons[] = {"ZAP", "AIM", "TAP", "MEM"};
  uint16_t colors[] = {RGB565(255, 80, 80), RGB565(80, 255, 120), RGB565(80, 130, 255), RGB565(255, 200, 80)};
  
  for (int i = 0; i < 4; i++) {
    int y = 92 + i * 88;
    
    gfx->fillRect(20, y, LCD_WIDTH - 40, 78, RGB565(12, 14, 20));
    gfx->drawRect(20, y, LCD_WIDTH - 40, 78, colors[i]);
    gfx->fillRect(20, y, 5, 5, colors[i]);
    gfx->fillRect(LCD_WIDTH - 25, y, 5, 5, colors[i]);
    gfx->fillRect(20, y + 2, 5, 74, colors[i]);
    
    // Icon box
    gfx->fillRect(32, y + 15, 48, 48, RGB565(20, 24, 35));
    gfx->drawRect(32, y + 15, 48, 48, colors[i]);
    gfx->setTextColor(colors[i]);
    gfx->setTextSize(2);
    gfx->setCursor(38, y + 32);
    gfx->print(icons[i]);
    
    // Game name
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(90, y + 15);
    gfx->print(games[i]);
    
    // Description
    gfx->setTextColor(RGB565(120, 125, 140));
    gfx->setTextSize(1);
    gfx->setCursor(90, y + 40);
    gfx->print(descs[i]);
    
    // Play arrow + XP hint
    gfx->setTextColor(colors[i]);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH - 50, y + 28);
    gfx->print(">");
    gfx->setTextColor(RGB565(80, 200, 100));
    gfx->setTextSize(1);
    gfx->setCursor(90, y + 56);
    gfx->printf("+%d-%d XP", TRAINING_XP_PER_GAME_MIN, TRAINING_XP_PER_GAME_MAX);
  }
  
  drawSwipeIndicator();
}

void drawTrainingResults(TrainingScore& score) {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->accent);
  }
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(3);
  gfx->setCursor(centerX - 65, 10);
  gfx->print("COMPLETE!");
  
  // Score rating
  const char* rating;
  uint16_t ratingColor;
  if (score.score >= 90) { rating = "S RANK"; ratingColor = RGB565(255, 200, 50); }
  else if (score.score >= 70) { rating = "A RANK"; ratingColor = RGB565(100, 255, 100); }
  else if (score.score >= 50) { rating = "B RANK"; ratingColor = RGB565(100, 180, 255); }
  else if (score.score >= 30) { rating = "C RANK"; ratingColor = RGB565(255, 200, 100); }
  else { rating = "D RANK"; ratingColor = RGB565(200, 100, 100); }
  
  gfx->setTextColor(ratingColor);
  gfx->setTextSize(4);
  int rLen = strlen(rating) * 24;
  gfx->setCursor(centerX - rLen / 2, 70);
  gfx->print(rating);
  
  // Score panel
  gfx->fillRect(30, 130, LCD_WIDTH - 60, 170, RGB565(12, 14, 20));
  gfx->drawRect(30, 130, LCD_WIDTH - 60, 170, theme->accent);
  gfx->fillRect(30, 130, 5, 5, theme->accent);
  gfx->fillRect(LCD_WIDTH - 35, 130, 5, 5, theme->accent);
  
  // Score number
  gfx->setTextColor(RGB565(120, 125, 140));
  gfx->setTextSize(1);
  gfx->setCursor(50, 145);
  gfx->print("SCORE");
  gfx->setTextSize(6);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(50, 160);
  gfx->printf("%d", score.score);
  
  // XP earned
  gfx->setTextColor(RGB565(80, 255, 120));
  gfx->setTextSize(3);
  gfx->setCursor(50, 230);
  gfx->printf("+%d XP", score.xp_earned);
  
  // Extra stats
  if (score.type == TRAINING_REFLEX && score.best_time_ms < 9999) {
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(1);
    gfx->setCursor(220, 145);
    gfx->printf("Best: %dms", score.best_time_ms);
  }
  if (score.combo_count > 0) {
    gfx->setTextColor(COLOR_GOLD);
    gfx->setTextSize(1);
    gfx->setCursor(220, 165);
    gfx->printf("Max Combo: %d", score.combo_count);
  }
  // Streak bonus info
  gfx->setTextColor(RGB565(255, 200, 50));
  gfx->setTextSize(1);
  gfx->setCursor(50, 268);
  gfx->printf("Streak: %d days", system_state.training_streak);

  // Continue button
  int btnY = 330;
  gfx->fillRect(centerX - 80, btnY, 160, 50, theme->primary);
  gfx->drawRect(centerX - 80, btnY, 160, 50, COLOR_WHITE);
  gfx->fillRect(centerX - 80, btnY, 5, 5, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 40, btnY + 15);
  gfx->print("CONTINUE");

  drawSwipeIndicator();
}

void handleTrainingMenuTouch(TouchGesture& gesture) {
  // Swipe up to exit
  if (gesture.event == TOUCH_SWIPE_UP) {
    returnToAppGrid();
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Check game buttons (4 games, each 88px spacing, starting at y=92)
  for (int i = 0; i < 4; i++) {
    int by = 92 + i * 88;
    if (y >= by && y < by + 78 && x >= 20 && x < LCD_WIDTH - 20) {
      Serial.printf("[Training] Selected game: %d\n", i);
      startTrainingGame((TrainingType)i);
      return;
    }
  }
}

void handleTrainingGameTouch(TouchGesture& gesture) {
  if (!training_active) {
    // Results screen - continue button
    if (gesture.event == TOUCH_TAP && gesture.y >= 340 && gesture.y < 390) {
      returnToAppGrid();
    }
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  switch(current_training_type) {
    case TRAINING_REFLEX:    handleReflexTouch(x, y); break;
    case TRAINING_TARGET:    handleTargetTouch(x, y); break;
    case TRAINING_SPEED_TAP: handleSpeedTapTouch(x, y); break;
    case TRAINING_MEMORY:    handleMemoryTouch(x, y); break;
  }
}

void drawTrainingButton(int x, int y, int size, int button_id, bool lit, bool pressed) {
  uint16_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_GOLD};
  uint16_t bg = lit ? colors[button_id] : RGB565(30, 32, 40);
  if (pressed) bg = getCurrentTheme()->accent;
  
  gfx->fillRect(x, y, size, size, bg);
  gfx->drawRect(x, y, size, size, COLOR_WHITE);
  
  if (lit) {
    gfx->drawRect(x - 3, y - 3, size + 6, size + 6, colors[button_id]);
  }
}

void drawTrainingTimer(int seconds_remaining) {
  gfx->fillRect(270, 10, 60, 30, RGB565(2, 2, 5));
  gfx->setTextColor(seconds_remaining < 5 ? COLOR_RED : COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(280, 15);
  gfx->printf("%ds", seconds_remaining);
}

void drawTrainingScore(int score) {
  gfx->fillRect(10, 10, 80, 30, RGB565(2, 2, 5));
  gfx->setTextColor(COLOR_GREEN);
  gfx->setTextSize(2);
  gfx->setCursor(20, 15);
  gfx->printf("%d", score);
}
