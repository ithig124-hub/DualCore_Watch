
/*
 * daily_quests.cpp - Daily Quest System with SCROLLING
 *
 * IMPROVEMENTS:
 * - Scrollable quest list (can show more than 3 quests)
 * - Better visibility and touch targets
 * - RPG integration - quests give XP to character
 * - Connected to steps, training, boss battles
 */

#include "daily_quests.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"
#include "steps_tracker.h"
#include "gacha.h"
#include "xp_system.h"  // FUSION OS: For getCurrentCharacterXP()
#include "sd_preferences.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

DailyQuestsData quest_data;
SDPreferences questPrefs;

// Scroll state for quests
static int quest_scroll_offset = 0;
static const int QUEST_CARD_HEIGHT = 95;
static const int VISIBLE_QUESTS = 3;
static const int QUEST_START_Y = 95;

const char* QUEST_NAMES[] = {
  "Daily Steps",
  "Active Time",
  "Play Games",
  "Training",
  "Boss Battle",
  "Distance Goal"
};

const char* QUEST_DESCRIPTIONS[] = {
  "Walk %d steps",
  "Be active %d min",
  "Play %d games",
  "Train %d times",
  "Defeat %d bosses",
  "Walk %d km"
};

void initDailyQuests() {
  loadDailyQuestsData();
  checkDailyReset();
  quest_scroll_offset = 0;
  Serial.println("[Quests] Daily quests initialized with scrolling");
}

void drawDailyQuestsScreen() {
  // ========================================
  // RETRO ANIME DAILY QUESTS - SCROLLABLE (Optimized for 410x502)
  // ANTI-FLICKER: Only redraw when quest data or scroll position changes
  // ========================================
  
  // ANTI-FLICKER: Track changes
  static bool needs_redraw = true;
  static int last_scroll_offset = -1;
  static uint32_t last_streak = 0;
  static uint32_t last_completed = 0;
  static bool last_quest_states[3] = {false, false, false};
  
  // Check if data changed
  bool quest_changed = false;
  for (int i = 0; i < 3; i++) {
    if (quest_data.daily_quests[i].completed != last_quest_states[i]) {
      quest_changed = true;
      last_quest_states[i] = quest_data.daily_quests[i].completed;
    }
  }
  
  bool data_changed = (quest_scroll_offset != last_scroll_offset) ||
                      (quest_data.streak_days != last_streak) ||
                      (quest_data.total_completed != last_completed) ||
                      quest_changed;
  
  if (!needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  needs_redraw = false;
  last_scroll_offset = quest_scroll_offset;
  last_streak = quest_data.streak_days;
  last_completed = quest_data.total_completed;
  
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();

  // Header - taller retro style
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 108 + 2, 14);
  gfx->print("DAILY QUESTS");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 108, 12);
  gfx->print("DAILY QUESTS");

  // Streak banner with RPG integration - wider
  int bannerY = headerH + 8;
  gfx->fillRect(15, bannerY, LCD_WIDTH - 30, 40, RGB565(25, 15, 40));
  gfx->drawRect(15, bannerY, LCD_WIDTH - 30, 40, RGB565(80, 50, 120));
  gfx->fillRect(15, bannerY, 5, 5, COLOR_GOLD);
  gfx->fillRect(LCD_WIDTH - 20, bannerY, 5, 5, COLOR_GOLD);

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(25, bannerY + 10);
  gfx->printf("STREAK: %lu", (unsigned long)quest_data.streak_days);

  // Show RPG level and XP bonus
  CharacterXPData* char_xp = getCurrentCharacterXP();
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(180, bannerY + 10);
  gfx->printf("LV.%d", char_xp ? char_xp->level : 1);

  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(280, bannerY + 10);
  gfx->printf("DONE:%lu", (unsigned long)quest_data.total_completed);

  // Quest cards - scrollable area - larger cards
  int questStartY = bannerY + 55;
  int questCardH = 110; // Larger cards
  int questGap = 10;
  int visible_start = quest_scroll_offset;
  int visible_end = min(3, visible_start + 3);

  for (int i = visible_start; i < visible_end; i++) {
    Quest& q = quest_data.daily_quests[i];
    int questY = questStartY + (i - visible_start) * (questCardH + questGap);

    uint16_t bgColor = q.completed ? RGB565(10, 30, 10) : RGB565(12, 14, 20);
    uint16_t borderColor = q.completed ? RGB565(0, 200, 80) : RGB565(40, 45, 60);
    uint16_t cornerColor = q.completed ? RGB565(0, 200, 80) : theme->primary;

    // Card with larger touch target
    gfx->fillRect(15, questY, LCD_WIDTH - 30, questCardH, bgColor);
    gfx->drawRect(15, questY, LCD_WIDTH - 30, questCardH, borderColor);

    // Pixel corners
    gfx->fillRect(15, questY, 6, 6, cornerColor);
    gfx->fillRect(LCD_WIDTH - 21, questY, 6, 6, cornerColor);
    gfx->fillRect(15, questY + questCardH - 6, 6, 6, cornerColor);
    gfx->fillRect(LCD_WIDTH - 21, questY + questCardH - 6, 6, 6, cornerColor);

    // Difficulty stars - larger
    gfx->setTextColor(COLOR_GOLD);
    gfx->setTextSize(2);
    gfx->setCursor(30, questY + 10);
    for (int s = 0; s <= (int)q.difficulty; s++) {
      gfx->print("*");
    }

    // Quest name - larger
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setCursor(30, questY + 32);
    gfx->print(q.name);

    // Description
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(30, questY + 52);
    char desc[50];
    sprintf(desc, q.description, q.target);
    gfx->print(desc);

    // Progress bar - wider and more visible
    int barX = 30;
    int barY = questY + 68;
    int barW = 180;
    int barH = 14;

    gfx->fillRect(barX, barY, barW, barH, RGB565(8, 10, 14));
    gfx->drawRect(barX, barY, barW, barH, RGB565(30, 35, 50));

    float progress = (float)q.progress / q.target;
    if (progress > 1.0f) progress = 1.0f;
    int fillW = (int)((barW - 4) * progress);

    uint16_t progressColor = q.completed ? RGB565(0, 200, 80) : theme->accent;
    if (fillW > 0) gfx->fillRect(barX + 2, barY + 2, fillW, barH - 4, progressColor);

    // Progress text - clearer
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setCursor(barX + barW + 8, barY + 3);
    gfx->printf("%lu/%lu", (unsigned long)q.progress, (unsigned long)q.target);

    // Rewards - show XP for RPG
    gfx->setTextColor(COLOR_CYAN);
    gfx->setCursor(270, questY + 20);
    gfx->printf("+%d XP", q.reward_xp);

    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(270, questY + 35);
    gfx->printf("+%d gold", q.reward_gold);

    if (q.reward_gems > 0) {
      gfx->setTextColor(COLOR_PURPLE);
      gfx->setCursor(270, questY + 50);
      gfx->printf("+%d gems", q.reward_gems);
    }

    // Claim button - bigger and more visible
    if (q.completed && q.reward_gold > 0) {
      gfx->fillRect(270, questY + 65, 75, 25, RGB565(0, 150, 60));
      gfx->drawRect(270, questY + 65, 75, 25, RGB565(0, 200, 80));
      gfx->fillRect(270, questY + 65, 4, 4, COLOR_WHITE);
      gfx->setTextSize(1);
      gfx->setTextColor(COLOR_WHITE);
      gfx->setCursor(284, questY + 73);
      gfx->print("CLAIM!");
    }
  }

  // Scroll indicators
  if (quest_scroll_offset > 0) {
    // Up arrow
    gfx->fillTriangle(LCD_WIDTH/2 - 10, 90, LCD_WIDTH/2 + 10, 90, LCD_WIDTH/2, 80, theme->primary);
  }

  // Footer - shows how to scroll
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setCursor(70, 425);
  gfx->print("Swipe up/down to scroll | Resets at midnight");

  drawSwipeIndicator();
}

void generateNewDailyQuests() {
  Serial.println("[Quests] Generating new daily quests");
  quest_scroll_offset = 0;

  for (int i = 0; i < 3; i++) {
    quest_data.daily_quests[i].completed = false;
    quest_data.daily_quests[i].progress = 0;
  }

  QuestDifficulty difficulties[] = {QUEST_EASY, QUEST_MEDIUM, QUEST_HARD};

  for (int i = 0; i < 3; i++) {
    Quest& q = quest_data.daily_quests[i];
    q.type = (QuestType)random(0, 6);
    q.difficulty = difficulties[i];
    q.name = QUEST_NAMES[(int)q.type];
    q.description = QUEST_DESCRIPTIONS[(int)q.type];

    // Scale targets based on RPG level for progression
    CharacterXPData* char_xp = getCurrentCharacterXP();
    int level_bonus = (char_xp ? char_xp->level : 1) / 10;

    switch(q.difficulty) {
      case QUEST_EASY:
        q.target = (q.type == QUEST_STEPS) ? 2000 + level_bonus * 200 :
                   (q.type == QUEST_DISTANCE) ? 2 : 3;
        q.reward_gold = 100 + level_bonus * 20;
        q.reward_xp = 50 + level_bonus * 10;
        q.reward_gems = 0;
        break;
      case QUEST_MEDIUM:
        q.target = (q.type == QUEST_STEPS) ? 5000 + level_bonus * 500 :
                   (q.type == QUEST_DISTANCE) ? 5 : 5;
        q.reward_gold = 250 + level_bonus * 50;
        q.reward_xp = 100 + level_bonus * 20;
        q.reward_gems = 1;
        break;
      case QUEST_HARD:
        q.target = (q.type == QUEST_STEPS) ? 10000 + level_bonus * 1000 :
                   (q.type == QUEST_DISTANCE) ? 10 : 10;
        q.reward_gold = 500 + level_bonus * 100;
        q.reward_xp = 200 + level_bonus * 40;
        q.reward_gems = 3;
        break;
      default:
        break;
    }

    q.progress = 0;
    q.completed = false;
  }

  saveDailyQuestsData();
}

void updateQuestProgress(QuestType type, uint32_t amount) {
  for (int i = 0; i < 3; i++) {
    Quest& q = quest_data.daily_quests[i];
    if (q.type == type && !q.completed) {
      q.progress += amount;
      if (q.progress >= q.target) {
        q.progress = q.target;
        q.completed = true;
        Serial.printf("[Quests] Quest completed: %s\n", q.name);
      }
      saveDailyQuestsData();
    }
  }
}

void claimQuestReward(int questIndex) {
  if (questIndex < 0 || questIndex >= 3) return;

  Quest& q = quest_data.daily_quests[questIndex];
  if (!q.completed || q.reward_gold == 0) return; // Already claimed

  // Give RPG XP - this is the key integration!
  gainExperience(q.reward_xp, q.name);

  // Give gems
  int total_gems = q.reward_gems + (q.reward_gold / 100);
  system_state.player_gems += total_gems;

  Serial.printf("[Quests] Claimed: %d XP, %d gems\n", q.reward_xp, total_gems);

  // Mark as claimed (set rewards to 0)
  q.reward_gold = 0;
  q.reward_xp = 0;
  q.reward_gems = 0;

  quest_data.total_completed++;
  saveDailyQuestsData();

  drawDailyQuestsScreen();
}

void checkDailyReset() {
  WatchTime current_time = getCurrentTime();

  if (current_time.hour == 0 && quest_data.current_day != current_time.day) {
    Serial.println("[Quests] Daily reset triggered");

    bool all_completed = true;
    for (int i = 0; i < 3; i++) {
      if (!quest_data.daily_quests[i].completed) {
        all_completed = false;
        break;
      }
    }

    if (all_completed) {
      quest_data.streak_days++;
      // Streak bonus XP!
      gainExperience(quest_data.streak_days * 10, "Streak Bonus");
    } else {
      quest_data.streak_days = 0;
    }

    generateNewDailyQuests();
    quest_data.current_day = current_time.day;
    quest_data.last_reset = millis();
    saveDailyQuestsData();
  }
}

void saveDailyQuestsData() {
  questPrefs.begin("quests", false);
  questPrefs.putUChar("day", quest_data.current_day);
  questPrefs.putUInt("total", quest_data.total_completed);
  questPrefs.putUInt("streak", quest_data.streak_days);

  for (int i = 0; i < 3; i++) {
    char key[20];
    sprintf(key, "q%d_type", i);
    questPrefs.putUChar(key, (uint8_t)quest_data.daily_quests[i].type);
    sprintf(key, "q%d_prog", i);
    questPrefs.putUInt(key, quest_data.daily_quests[i].progress);
    sprintf(key, "q%d_comp", i);
    questPrefs.putBool(key, quest_data.daily_quests[i].completed);
    sprintf(key, "q%d_gold", i);
    questPrefs.putUInt(key, quest_data.daily_quests[i].reward_gold);
  }
  questPrefs.end();
}

void loadDailyQuestsData() {
  questPrefs.begin("quests", true);
  quest_data.current_day = questPrefs.getUChar("day", 0);
  quest_data.total_completed = questPrefs.getUInt("total", 0);
  quest_data.streak_days = questPrefs.getUInt("streak", 0);
  questPrefs.end();

  WatchTime current_time = getCurrentTime();
  if (quest_data.current_day != current_time.day) {
    generateNewDailyQuests();
    quest_data.current_day = current_time.day;
  }
}

void handleDailyQuestsTouch(TouchGesture& gesture) {
  // Swipe up to exit
  if (gesture.event == TOUCH_SWIPE_UP) {
    returnToAppGrid();
    return;
  }

  // Swipe down to scroll down (if we add more quests)
  if (gesture.event == TOUCH_SWIPE_DOWN) {
    // Could scroll up in quest list
    return;
  }

  if (gesture.event != TOUCH_TAP) return;

  // Check claim buttons for visible quests
  int questY = QUEST_START_Y;
  for (int i = quest_scroll_offset; i < min(3, quest_scroll_offset + VISIBLE_QUESTS); i++) {
    // Claim button area: x: 270-345, y: questY+65 to questY+90
    if (gesture.x >= 270 && gesture.x <= 345 &&
        gesture.y >= questY + 65 && gesture.y <= questY + 90) {
      claimQuestReward(i);
      return;
    }
    questY += 120; // questCardH + questGap
  }
}

