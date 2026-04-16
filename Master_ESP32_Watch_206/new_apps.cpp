/*
 * new_apps.cpp - New Apps: Pomodoro Timer, Habit Tracker, Daily Dungeon, 
 *                Login Streak Viewer, Card Crafting
 * FUSION OS RPG Features
 */

#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"
#include "xp_system.h"
#include "sd_preferences.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;
extern XPSystemState xp_system;

// =============================================================================
// POMODORO TIMER
// =============================================================================

struct PomodoroState {
  bool active;
  bool is_break;
  int work_minutes;
  int break_minutes;
  int seconds_remaining;
  int sessions_completed;
  unsigned long last_tick;
};

static PomodoroState pomo = {false, false, 25, 5, 25 * 60, 0, 0};

void initPomodoroApp() {
  if (!pomo.active) {
    pomo.seconds_remaining = pomo.work_minutes * 60;
    pomo.is_break = false;
  }
}

void drawPomodoroApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  uint16_t headerAccent = pomo.is_break ? RGB565(80, 200, 150) : RGB565(255, 80, 80);
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, headerAccent);
  }
  gfx->setTextColor(headerAccent);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 60, 14);
  gfx->print(pomo.is_break ? "BREAK" : "POMODORO");
  
  // Circular progress ring
  int ringCX = centerX;
  int ringCY = 180;
  int ringR = 90;
  
  int total_seconds = pomo.is_break ? (pomo.break_minutes * 60) : (pomo.work_minutes * 60);
  float progress = 1.0f - ((float)pomo.seconds_remaining / (float)total_seconds);
  
  // Background ring
  for (int angle = 0; angle < 360; angle += 2) {
    float rad = angle * 3.14159 / 180.0;
    int px = ringCX + (int)(ringR * cos(rad));
    int py = ringCY + (int)(ringR * sin(rad));
    gfx->fillRect(px-1, py-1, 3, 3, RGB565(20, 22, 30));
  }
  
  // Progress ring
  int progress_degrees = (int)(progress * 360);
  for (int angle = -90; angle < -90 + progress_degrees; angle += 2) {
    float rad = angle * 3.14159 / 180.0;
    int px = ringCX + (int)(ringR * cos(rad));
    int py = ringCY + (int)(ringR * sin(rad));
    gfx->fillRect(px-2, py-2, 5, 5, headerAccent);
  }
  
  // Time display
  int mins = pomo.seconds_remaining / 60;
  int secs = pomo.seconds_remaining % 60;
  char timeStr[8];
  sprintf(timeStr, "%02d:%02d", mins, secs);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(5);
  int timeW = strlen(timeStr) * 30;
  gfx->setCursor(ringCX - timeW / 2, ringCY - 15);
  gfx->print(timeStr);
  
  // Session counter
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setCursor(ringCX - 40, ringCY + 30);
  gfx->printf("Session: %d", pomo.sessions_completed + 1);
  
  // Start/Pause button
  int btnW = 160;
  int btnH = 50;
  int btnX = centerX - btnW/2;
  int btnY = 310;
  
  uint16_t btnColor = pomo.active ? RGB565(200, 80, 50) : RGB565(50, 200, 100);
  gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
  gfx->drawRect(btnX, btnY, btnW, btnH, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 30, btnY + 15);
  gfx->print(pomo.active ? "PAUSE" : "START");
  
  // Reset button
  int rstBtnY = btnY + btnH + 15;
  gfx->fillRect(btnX, rstBtnY, btnW, 35, RGB565(40, 40, 50));
  gfx->drawRect(btnX, rstBtnY, btnW, 35, RGB565(80, 80, 90));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 15, rstBtnY + 12);
  gfx->print("RESET");
  
  // Info
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 80, 440);
  gfx->printf("%dmin work / %dmin break", pomo.work_minutes, pomo.break_minutes);
  gfx->setCursor(centerX - 60, 455);
  gfx->print("Earn 25 gems per session!");
  
  drawSwipeIndicator();
}

void updatePomodoro() {
  if (!pomo.active) return;
  
  unsigned long now = millis();
  if (now - pomo.last_tick >= 1000) {
    pomo.last_tick = now;
    pomo.seconds_remaining--;
    
    if (pomo.seconds_remaining <= 0) {
      if (!pomo.is_break) {
        // Work session complete - reward!
        pomo.sessions_completed++;
        system_state.player_gems += 25;
        gainExperience(15, "Pomodoro Complete");
        
        // Switch to break
        pomo.is_break = true;
        pomo.seconds_remaining = pomo.break_minutes * 60;
        Serial.printf("[POMO] Work done! Session %d. Break time.\n", pomo.sessions_completed);
      } else {
        // Break complete - back to work
        pomo.is_break = false;
        pomo.seconds_remaining = pomo.work_minutes * 60;
        Serial.println("[POMO] Break over! Back to work.");
      }
    }
    
    // Redraw if on pomodoro screen
    if (system_state.current_screen == SCREEN_POMODORO) {
      drawPomodoroApp();
    }
  }
}

void handlePomodoroTouch(int x, int y) {
  int centerX = LCD_WIDTH / 2;
  int btnW = 160;
  int btnH = 50;
  int btnX = centerX - btnW/2;
  int btnY = 310;
  
  // Start/Pause button
  if (x >= btnX && x < btnX + btnW && y >= btnY && y < btnY + btnH) {
    pomo.active = !pomo.active;
    if (pomo.active) pomo.last_tick = millis();
    drawPomodoroApp();
    return;
  }
  
  // Reset button
  int rstBtnY = btnY + btnH + 15;
  if (x >= btnX && x < btnX + btnW && y >= rstBtnY && y < rstBtnY + 35) {
    pomo.active = false;
    pomo.is_break = false;
    pomo.seconds_remaining = pomo.work_minutes * 60;
    drawPomodoroApp();
    return;
  }
}

// =============================================================================
// HABIT TRACKER
// =============================================================================

#define MAX_HABITS 6

struct Habit {
  const char* name;
  const char* icon;
  bool completed_today;
  int current_streak;
  int best_streak;
};

static Habit habits[MAX_HABITS] = {
  {"Train", "SWORD", false, 0, 0},
  {"1K Steps", "WALK", false, 0, 0},
  {"Meditate", "ZEN", false, 0, 0},
  {"Read", "BOOK", false, 0, 0},
  {"Hydrate", "DROP", false, 0, 0},
  {"Study", "STAR", false, 0, 0}
};

static int habits_last_day = -1;

void initHabitsApp() {
  // Load from NVS
  SDPreferences hPrefs;
  hPrefs.begin("habits", true);
  int saved_day = hPrefs.getInt("last_day", -1);
  
  WatchTime current = getCurrentTime();
  bool is_new_day = (saved_day != current.day);
  
  for (int i = 0; i < MAX_HABITS; i++) {
    char key[12];
    snprintf(key, sizeof(key), "h%d_str", i);
    habits[i].current_streak = hPrefs.getInt(key, 0);
    snprintf(key, sizeof(key), "h%d_best", i);
    habits[i].best_streak = hPrefs.getInt(key, 0);
    snprintf(key, sizeof(key), "h%d_done", i);
    habits[i].completed_today = is_new_day ? false : hPrefs.getBool(key, false);
  }
  habits_last_day = saved_day;
  hPrefs.end();
  
  // If new day, check if habits were NOT completed yesterday (streak break)
  if (is_new_day && habits_last_day >= 0) {
    SDPreferences hSave;
    hSave.begin("habits", false);
    for (int i = 0; i < MAX_HABITS; i++) {
      char key[12];
      snprintf(key, sizeof(key), "h%d_done", i);
      bool was_done = hSave.getBool(key, false);
      if (!was_done) {
        habits[i].current_streak = 0; // Streak broken
      }
      // Reset today's status
      hSave.putBool(key, false);
    }
    hSave.putInt("last_day", current.day);
    hSave.end();
  }
}

void saveHabitsData() {
  SDPreferences hPrefs;
  hPrefs.begin("habits", false);
  WatchTime current = getCurrentTime();
  hPrefs.putInt("last_day", current.day);
  for (int i = 0; i < MAX_HABITS; i++) {
    char key[12];
    snprintf(key, sizeof(key), "h%d_str", i);
    hPrefs.putInt(key, habits[i].current_streak);
    snprintf(key, sizeof(key), "h%d_best", i);
    hPrefs.putInt(key, habits[i].best_streak);
    snprintf(key, sizeof(key), "h%d_done", i);
    hPrefs.putBool(key, habits[i].completed_today);
  }
  hPrefs.end();
}

void drawHabitsApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(80, 220, 160));
  }
  gfx->setTextColor(RGB565(80, 220, 160));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 50, 14);
  gfx->print("HABITS");
  
  // Count completed
  int done_count = 0;
  for (int i = 0; i < MAX_HABITS; i++) {
    if (habits[i].completed_today) done_count++;
  }
  
  // Progress bar
  gfx->fillRect(30, 60, LCD_WIDTH - 60, 20, RGB565(20, 22, 30));
  gfx->drawRect(30, 60, LCD_WIDTH - 60, 20, RGB565(40, 45, 55));
  int progressW = ((LCD_WIDTH - 64) * done_count) / MAX_HABITS;
  if (progressW > 0) {
    gfx->fillRect(32, 62, progressW, 16, RGB565(80, 220, 160));
  }
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 20, 64);
  gfx->printf("%d / %d", done_count, MAX_HABITS);
  
  // Habit list
  int itemH = 55;
  int startY = 90;
  int itemW = LCD_WIDTH - 40;
  
  for (int i = 0; i < MAX_HABITS; i++) {
    int iy = startY + i * (itemH + 8);
    
    // Background
    uint16_t bg = habits[i].completed_today ? RGB565(15, 25, 18) : RGB565(12, 14, 20);
    uint16_t border = habits[i].completed_today ? RGB565(80, 220, 160) : RGB565(35, 40, 55);
    gfx->fillRect(20, iy, itemW, itemH, bg);
    gfx->drawRect(20, iy, itemW, itemH, border);
    
    // Check/X indicator
    if (habits[i].completed_today) {
      gfx->fillRect(28, iy + 15, 25, 25, RGB565(80, 220, 160));
      gfx->setTextColor(RGB565(2, 2, 5));
      gfx->setTextSize(2);
      gfx->setCursor(32, iy + 18);
      gfx->print("V");
    } else {
      gfx->drawRect(28, iy + 15, 25, 25, RGB565(60, 65, 80));
    }
    
    // Habit name
    gfx->setTextColor(habits[i].completed_today ? RGB565(80, 220, 160) : COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(65, iy + 10);
    gfx->print(habits[i].name);
    
    // Streak info
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(255, 200, 50));
    gfx->setCursor(65, iy + 35);
    gfx->printf("Streak: %d | Best: %d", habits[i].current_streak, habits[i].best_streak);
  }
  
  // All done reward info
  if (done_count == MAX_HABITS) {
    gfx->setTextColor(RGB565(255, 200, 50));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 60, LCD_HEIGHT - 30);
    gfx->print("ALL DONE! +100 gems +50 XP");
  }
  
  drawSwipeIndicator();
}

void handleHabitsTouch(int x, int y) {
  int itemH = 55;
  int startY = 90;
  int itemW = LCD_WIDTH - 40;
  
  for (int i = 0; i < MAX_HABITS; i++) {
    int iy = startY + i * (itemH + 8);
    if (x >= 20 && x < 20 + itemW && y >= iy && y < iy + itemH) {
      if (!habits[i].completed_today) {
        habits[i].completed_today = true;
        habits[i].current_streak++;
        if (habits[i].current_streak > habits[i].best_streak) {
          habits[i].best_streak = habits[i].current_streak;
        }
        gainExperience(10, "Habit Complete");
        system_state.player_gems += 10;
        
        // Check if all habits done
        int done = 0;
        for (int j = 0; j < MAX_HABITS; j++) {
          if (habits[j].completed_today) done++;
        }
        if (done == MAX_HABITS) {
          system_state.player_gems += 100;
          gainExperience(50, "All Habits Complete");
          Serial.println("[HABITS] All habits complete! Bonus awarded!");
        }
        
        saveHabitsData();
      }
      drawHabitsApp();
      return;
    }
  }
}

// =============================================================================
// DAILY DUNGEON
// =============================================================================

#define DUNGEON_ROOMS 5

struct DungeonRoom {
  const char* name;
  const char* enemy;
  int enemy_hp;
  int enemy_atk;
  int gem_reward;
  int xp_reward;
  bool cleared;
};

struct DungeonState {
  bool active;
  bool completed_today;
  int current_room;
  int player_hp;
  int player_max_hp;
  int player_atk;
  DungeonRoom rooms[DUNGEON_ROOMS];
  int last_dungeon_day;
  int total_clears;
};

static DungeonState dungeon = {false, false, 0, 100, 100, 15, {}, -1, 0};

// Themed enemy names per character
const char* getThemedEnemy(int room) {
  switch (system_state.current_theme) {
    case THEME_LUFFY_GEAR5:
      { const char* e[] = {"Marine Captain", "Pacifista", "Warlord", "Admiral", "Yonko"}; return e[room]; }
    case THEME_SUNG_JINWOO:
      { const char* e[] = {"Goblin", "Ice Bear", "Naga", "Red Gate Boss", "Monarch"}; return e[room]; }
    case THEME_NARUTO_SAGE:
      { const char* e[] = {"Chunin", "Anbu", "Akatsuki", "Jinchuriki", "Otsutsuki"}; return e[room]; }
    case THEME_GOJO_INFINITY:
      { const char* e[] = {"Cursed Spirit", "Grade 2 Curse", "Special Grade", "Disaster Curse", "Sukuna Fragment"}; return e[room]; }
    case THEME_TANJIRO_SUN:
      { const char* e[] = {"Lower Demon", "Upper Moon 6", "Upper Moon 4", "Upper Moon 2", "Muzan Clone"}; return e[room]; }
    case THEME_GOKU_UI:
      { const char* e[] = {"Saibaman", "Frieza Soldier", "Cell Jr.", "Buu Spawn", "God of Destruction"}; return e[room]; }
    case THEME_SAITAMA_OPM:
      { const char* e[] = {"Tiger Threat", "Demon Threat", "Dragon Threat", "God Threat", "Cosmic Entity"}; return e[room]; }
    default:
      { const char* e[] = {"Goblin", "Skeleton", "Dark Knight", "Dragon", "Demon Lord"}; return e[room]; }
  }
}

void initDungeonApp() {
  SDPreferences dPrefs;
  dPrefs.begin("dungeon", true);
  dungeon.last_dungeon_day = dPrefs.getInt("last_day", -1);
  dungeon.total_clears = dPrefs.getInt("clears", 0);
  dPrefs.end();
  
  WatchTime current = getCurrentTime();
  dungeon.completed_today = (dungeon.last_dungeon_day == current.day);
  
  if (!dungeon.completed_today && !dungeon.active) {
    // Fresh dungeon
    dungeon.current_room = 0;
    dungeon.player_hp = 100 + system_state.player_level * 5;
    dungeon.player_max_hp = dungeon.player_hp;
    dungeon.player_atk = 10 + system_state.player_level * 2;
    dungeon.active = false;
    
    for (int i = 0; i < DUNGEON_ROOMS; i++) {
      dungeon.rooms[i].name = getThemedEnemy(i);
      dungeon.rooms[i].enemy = dungeon.rooms[i].name;
      dungeon.rooms[i].enemy_hp = 30 + i * 25 + system_state.player_level * 3;
      dungeon.rooms[i].enemy_atk = 5 + i * 8;
      dungeon.rooms[i].gem_reward = 20 + i * 30;
      dungeon.rooms[i].xp_reward = 15 + i * 20;
      dungeon.rooms[i].cleared = false;
    }
  }
}

void saveDungeonData() {
  SDPreferences dPrefs;
  dPrefs.begin("dungeon", false);
  WatchTime current = getCurrentTime();
  dPrefs.putInt("last_day", current.day);
  dPrefs.putInt("clears", dungeon.total_clears);
  dPrefs.end();
}

void drawDungeonApp() {
  gfx->fillScreen(RGB565(5, 2, 8));
  for (int y = 0; y < LCD_HEIGHT; y += 6) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(8, 4, 12));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(12, 8, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(180, 60, 255));
  }
  gfx->setTextColor(RGB565(180, 60, 255));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 50, 14);
  gfx->print("DUNGEON");
  
  if (dungeon.completed_today) {
    // Already completed today
    gfx->setTextColor(RGB565(150, 150, 160));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 75, 120);
    gfx->print("CLEARED TODAY!");
    
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(255, 200, 50));
    gfx->setCursor(centerX - 50, 160);
    gfx->printf("Total Clears: %d", dungeon.total_clears);
    
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(centerX - 65, 200);
    gfx->print("Come back tomorrow!");
    
    drawSwipeIndicator();
    return;
  }
  
  if (!dungeon.active) {
    // Entry screen
    gfx->setTextColor(theme->primary);
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 70, 80);
    gfx->print("DAILY DUNGEON");
    
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setCursor(centerX - 80, 115);
    gfx->printf("Rooms: %d | Your ATK: %d", DUNGEON_ROOMS, dungeon.player_atk);
    gfx->setCursor(centerX - 70, 132);
    gfx->printf("Your HP: %d/%d", dungeon.player_hp, dungeon.player_max_hp);
    
    // Room preview
    for (int i = 0; i < DUNGEON_ROOMS; i++) {
      int ry = 160 + i * 40;
      gfx->fillRect(30, ry, LCD_WIDTH - 60, 32, RGB565(15, 10, 22));
      gfx->drawRect(30, ry, LCD_WIDTH - 60, 32, RGB565(60, 40, 80));
      gfx->setTextColor(RGB565(200, 180, 230));
      gfx->setTextSize(1);
      gfx->setCursor(40, ry + 5);
      gfx->printf("Room %d: %s", i + 1, dungeon.rooms[i].enemy);
      gfx->setTextColor(RGB565(255, 200, 50));
      gfx->setCursor(40, ry + 18);
      gfx->printf("HP:%d ATK:%d | +%dg +%dXP", 
        dungeon.rooms[i].enemy_hp, dungeon.rooms[i].enemy_atk,
        dungeon.rooms[i].gem_reward, dungeon.rooms[i].xp_reward);
    }
    
    // Enter button
    int btnY = 375;
    gfx->fillRect(centerX - 80, btnY, 160, 50, RGB565(180, 60, 255));
    gfx->drawRect(centerX - 80, btnY, 160, 50, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 35, btnY + 15);
    gfx->print("ENTER");
    
  } else {
    // In dungeon - combat screen
    DungeonRoom* room = &dungeon.rooms[dungeon.current_room];
    
    gfx->setTextColor(RGB565(180, 60, 255));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 30, 60);
    gfx->printf("Room %d/%d", dungeon.current_room + 1, DUNGEON_ROOMS);
    
    // Enemy display
    gfx->fillRect(40, 85, LCD_WIDTH - 80, 80, RGB565(20, 12, 28));
    gfx->drawRect(40, 85, LCD_WIDTH - 80, 80, RGB565(255, 60, 60));
    gfx->setTextColor(RGB565(255, 100, 100));
    gfx->setTextSize(2);
    gfx->setCursor(55, 95);
    gfx->print(room->enemy);
    
    // Enemy HP bar
    gfx->fillRect(55, 125, LCD_WIDTH - 110, 15, RGB565(30, 15, 15));
    int enemyHPW = (room->enemy_hp > 0) ? (room->enemy_hp * (LCD_WIDTH - 114)) / (30 + dungeon.current_room * 25 + system_state.player_level * 3) : 0;
    if (enemyHPW > LCD_WIDTH - 114) enemyHPW = LCD_WIDTH - 114;
    if (enemyHPW > 0) gfx->fillRect(57, 127, enemyHPW, 11, RGB565(255, 60, 60));
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(55, 148);
    gfx->printf("Enemy HP: %d | ATK: %d", room->enemy_hp, room->enemy_atk);
    
    // Player HP bar
    gfx->fillRect(40, 185, LCD_WIDTH - 80, 50, RGB565(12, 18, 14));
    gfx->drawRect(40, 185, LCD_WIDTH - 80, 50, RGB565(80, 220, 160));
    gfx->setTextColor(RGB565(80, 220, 160));
    gfx->setTextSize(2);
    gfx->setCursor(55, 192);
    gfx->print("YOUR HP");
    gfx->fillRect(55, 215, LCD_WIDTH - 110, 12, RGB565(30, 30, 15));
    int playerHPW = (dungeon.player_hp * (LCD_WIDTH - 114)) / dungeon.player_max_hp;
    if (playerHPW > 0) gfx->fillRect(57, 217, playerHPW, 8, RGB565(80, 220, 160));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 30, 230);
    gfx->printf("%d / %d", dungeon.player_hp, dungeon.player_max_hp);
    
    // ATTACK button
    int atkBtnY = 265;
    gfx->fillRect(40, atkBtnY, LCD_WIDTH - 80, 55, RGB565(200, 60, 60));
    gfx->drawRect(40, atkBtnY, LCD_WIDTH - 80, 55, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(centerX - 50, atkBtnY + 12);
    gfx->print("ATTACK");
    
    // HEAL button (costs 30 gems)
    int healBtnY = atkBtnY + 65;
    bool canHeal = system_state.player_gems >= 30 && dungeon.player_hp < dungeon.player_max_hp;
    gfx->fillRect(40, healBtnY, LCD_WIDTH - 80, 45, canHeal ? RGB565(40, 120, 80) : RGB565(25, 30, 35));
    gfx->drawRect(40, healBtnY, LCD_WIDTH - 80, 45, canHeal ? RGB565(80, 220, 160) : RGB565(50, 55, 65));
    gfx->setTextColor(canHeal ? COLOR_WHITE : RGB565(80, 85, 100));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 65, healBtnY + 12);
    gfx->print("HEAL (30g)");
    
    // Reward preview
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(255, 200, 50));
    gfx->setCursor(centerX - 50, healBtnY + 55);
    gfx->printf("Reward: +%dg +%dXP", room->gem_reward, room->xp_reward);
  }
  
  drawSwipeIndicator();
}

void handleDungeonTouch(int x, int y) {
  int centerX = LCD_WIDTH / 2;
  
  if (dungeon.completed_today) return;
  
  if (!dungeon.active) {
    // Enter button
    int btnY = 375;
    if (x >= centerX - 80 && x < centerX + 80 && y >= btnY && y < btnY + 50) {
      dungeon.active = true;
      drawDungeonApp();
    }
    return;
  }
  
  // ATTACK button
  int atkBtnY = 265;
  if (x >= 40 && x < LCD_WIDTH - 40 && y >= atkBtnY && y < atkBtnY + 55) {
    DungeonRoom* room = &dungeon.rooms[dungeon.current_room];
    
    // Player attacks
    int damage = dungeon.player_atk + (rand() % 5);
    room->enemy_hp -= damage;
    
    // Enemy attacks back
    int enemy_dmg = room->enemy_atk + (rand() % 3);
    dungeon.player_hp -= enemy_dmg;
    
    Serial.printf("[DUNGEON] You deal %d, enemy deals %d\n", damage, enemy_dmg);
    
    if (room->enemy_hp <= 0) {
      // Enemy defeated!
      room->cleared = true;
      system_state.player_gems += room->gem_reward;
      gainExperience(room->xp_reward, "Dungeon Clear");
      
      if (dungeon.current_room >= DUNGEON_ROOMS - 1) {
        // Dungeon complete!
        dungeon.completed_today = true;
        dungeon.active = false;
        dungeon.total_clears++;
        
        // Bonus for full clear
        system_state.player_gems += 200;
        gainExperience(100, "Dungeon Full Clear");
        saveDungeonData();
        
        extern void saveAllGameData();
        saveAllGameData();
        
        Serial.println("[DUNGEON] FULL CLEAR! +200 bonus gems");
      } else {
        // Next room
        dungeon.current_room++;
        extern void saveAllGameData();
        saveAllGameData();
      }
    }
    
    if (dungeon.player_hp <= 0) {
      // Player died - dungeon failed but can retry
      dungeon.player_hp = dungeon.player_max_hp / 2;
      dungeon.active = false;
      Serial.println("[DUNGEON] Defeated! Respawned at half HP.");
    }
    
    drawDungeonApp();
    return;
  }
  
  // HEAL button
  int healBtnY = atkBtnY + 65;
  if (x >= 40 && x < LCD_WIDTH - 40 && y >= healBtnY && y < healBtnY + 45) {
    if (system_state.player_gems >= 30 && dungeon.player_hp < dungeon.player_max_hp) {
      system_state.player_gems -= 30;
      dungeon.player_hp = min(dungeon.player_hp + 40, dungeon.player_max_hp);
      drawDungeonApp();
    }
    return;
  }
}

// =============================================================================
// LOGIN STREAK VIEWER APP
// =============================================================================

void drawStreakApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(255, 200, 50));
  }
  gfx->setTextColor(RGB565(255, 200, 50));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 50, 14);
  gfx->print("STREAK");
  
  // Current streak - big display
  gfx->fillRect(40, 70, LCD_WIDTH - 80, 90, RGB565(25, 20, 10));
  gfx->drawRect(40, 70, LCD_WIDTH - 80, 90, COLOR_GOLD);
  gfx->fillRect(40, 70, 8, 8, COLOR_GOLD);
  gfx->fillRect(LCD_WIDTH - 48, 70, 8, 8, COLOR_GOLD);
  
  gfx->setTextColor(RGB565(180, 160, 100));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 40, 78);
  gfx->print("CURRENT STREAK");
  
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(5);
  char streakStr[8];
  sprintf(streakStr, "%d", xp_system.login_streak);
  int strW = strlen(streakStr) * 30;
  gfx->setCursor(centerX - strW/2, 95);
  gfx->print(streakStr);
  
  gfx->setTextSize(1);
  gfx->setCursor(centerX + strW/2 + 5, 115);
  gfx->print("days");
  
  // Best streak
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 50, 175);
  gfx->printf("Best Streak: %d days", xp_system.longest_streak);
  
  // Milestone progress
  int milestoneY = 200;
  int milestones[] = {7, 14, 30};
  const char* milestone_names[] = {"Weekly", "Biweekly", "Monthly"};
  int milestone_gems[] = {500, 1000, 2000};
  const char* milestone_bonus[] = {"+Free Pull", "+Epic Card", "+Legendary"};
  
  for (int i = 0; i < 3; i++) {
    int my = milestoneY + i * 70;
    int target = milestones[i];
    int streak_in_cycle = xp_system.login_streak % target;
    if (streak_in_cycle == 0 && xp_system.login_streak > 0) streak_in_cycle = target;
    float prog = (float)streak_in_cycle / (float)target;
    bool achieved = (xp_system.login_streak >= target) && (streak_in_cycle == target);
    
    gfx->fillRect(30, my, LCD_WIDTH - 60, 58, achieved ? RGB565(20, 25, 15) : RGB565(12, 14, 20));
    gfx->drawRect(30, my, LCD_WIDTH - 60, 58, achieved ? RGB565(255, 200, 50) : RGB565(35, 40, 55));
    
    gfx->setTextColor(achieved ? RGB565(255, 200, 50) : RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(40, my + 5);
    gfx->printf("Day %d", target);
    
    gfx->setTextSize(1);
    gfx->setTextColor(achieved ? RGB565(255, 200, 50) : RGB565(100, 105, 120));
    gfx->setCursor(40, my + 28);
    gfx->printf("%s: +%dg %s", milestone_names[i], milestone_gems[i], milestone_bonus[i]);
    
    // Progress bar
    gfx->fillRect(40, my + 43, LCD_WIDTH - 80, 8, RGB565(20, 22, 30));
    int barW = (int)(prog * (LCD_WIDTH - 84));
    if (barW > 0) gfx->fillRect(42, my + 45, barW, 4, achieved ? RGB565(255, 200, 50) : theme->primary);
  }
  
  // Total logins
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 50, LCD_HEIGHT - 25);
  gfx->printf("Total Logins: %d", system_state.daily_login_count);
  
  drawSwipeIndicator();
}

// =============================================================================
// CARD CRAFTING (Combine duplicates)
// =============================================================================

extern bool cards_owned[];
extern int cards_duplicates[];
struct GachaCard;
extern GachaCard gacha_cards[];

struct CraftRecipe {
  const char* name;
  int dups_needed;
  int gem_cost;
  int gem_reward;
  int xp_reward;
};

static CraftRecipe recipes[] = {
  {"Gem Forge: 3 dupes -> 150 gems", 3, 0, 150, 25},
  {"XP Crucible: 5 dupes -> 200 XP", 5, 50, 0, 200},
  {"Power Craft: 2 dupes -> 75g+50XP", 2, 25, 75, 50},
};

static int craft_scroll = 0;

// Count total duplicates
int getTotalDuplicates() {
  int total = 0;
  for (int i = 0; i < 60; i++) { // GACHA_TOTAL_CARDS approximated
    if (cards_duplicates[i] > 0) total += cards_duplicates[i];
  }
  return total;
}

void initCraftApp() {
  craft_scroll = 0;
}

void drawCraftApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(200, 150, 100));
  }
  gfx->setTextColor(RGB565(200, 150, 100));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 35, 14);
  gfx->print("CRAFT");
  
  // Duplicate count
  int totalDups = getTotalDuplicates();
  gfx->fillRect(40, 60, LCD_WIDTH - 80, 40, RGB565(20, 18, 12));
  gfx->drawRect(40, 60, LCD_WIDTH - 80, 40, RGB565(200, 150, 100));
  gfx->setTextColor(RGB565(200, 150, 100));
  gfx->setTextSize(1);
  gfx->setCursor(55, 68);
  gfx->printf("Duplicate Cards: %d | Gems: %d", totalDups, system_state.player_gems);
  
  // Recipe list
  int recipeY = 115;
  for (int i = 0; i < 3; i++) {
    CraftRecipe* r = &recipes[i];
    int ry = recipeY + i * 90;
    bool canCraft = (totalDups >= r->dups_needed) && (system_state.player_gems >= r->gem_cost);
    
    gfx->fillRect(25, ry, LCD_WIDTH - 50, 75, canCraft ? RGB565(18, 16, 10) : RGB565(12, 14, 20));
    gfx->drawRect(25, ry, LCD_WIDTH - 50, 75, canCraft ? RGB565(200, 150, 100) : RGB565(35, 40, 55));
    gfx->fillRect(25, ry, 6, 6, canCraft ? RGB565(200, 150, 100) : RGB565(35, 40, 55));
    
    gfx->setTextColor(canCraft ? RGB565(200, 150, 100) : RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(40, ry + 10);
    gfx->print(r->name);
    
    gfx->setTextColor(canCraft ? RGB565(255, 200, 50) : RGB565(60, 65, 80));
    gfx->setCursor(40, ry + 28);
    if (r->gem_cost > 0) {
      gfx->printf("Cost: %d dupes + %d gems", r->dups_needed, r->gem_cost);
    } else {
      gfx->printf("Cost: %d duplicate cards", r->dups_needed);
    }
    
    // Craft button
    int cbX = LCD_WIDTH - 120;
    int cbY = ry + 45;
    gfx->fillRect(cbX, cbY, 80, 22, canCraft ? RGB565(200, 150, 100) : RGB565(30, 32, 40));
    gfx->setTextColor(canCraft ? RGB565(2, 2, 5) : RGB565(60, 65, 80));
    gfx->setTextSize(1);
    gfx->setCursor(cbX + 18, cbY + 6);
    gfx->print("CRAFT");
  }
  
  // Info
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 80, LCD_HEIGHT - 25);
  gfx->print("Get dupes from Gacha pulls!");
  
  drawSwipeIndicator();
}

void handleCraftTouch(int x, int y) {
  int recipeY = 115;
  int totalDups = getTotalDuplicates();
  
  for (int i = 0; i < 3; i++) {
    CraftRecipe* r = &recipes[i];
    int ry = recipeY + i * 90;
    int cbX = LCD_WIDTH - 120;
    int cbY = ry + 45;
    
    if (x >= cbX && x < cbX + 80 && y >= cbY && y < cbY + 22) {
      bool canCraft = (totalDups >= r->dups_needed) && (system_state.player_gems >= r->gem_cost);
      if (canCraft) {
        // Consume duplicates (remove from first cards that have dupes)
        int dupsToRemove = r->dups_needed;
        for (int j = 0; j < 60 && dupsToRemove > 0; j++) {
          if (cards_duplicates[j] > 0) {
            int remove = min(cards_duplicates[j], dupsToRemove);
            cards_duplicates[j] -= remove;
            dupsToRemove -= remove;
          }
        }
        
        // Apply cost and reward
        system_state.player_gems -= r->gem_cost;
        system_state.player_gems += r->gem_reward;
        if (r->xp_reward > 0) gainExperience(r->xp_reward, "Card Crafting");
        
        extern void saveAllGameData();
        saveAllGameData();
        
        Serial.printf("[CRAFT] Crafted: %s\n", r->name);
      }
      drawCraftApp();
      return;
    }
  }
}
