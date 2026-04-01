/*
 * rpg.cpp - RPG System Implementation
 * Character progression, quests, and leveling
 */

#include "rpg.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "hardware.h"
#include <SD_MMC.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// RPG character data
RPGCharacterData rpg_character;

// Quest data
QuestData active_quests[10];
int active_quest_count = 0;

// Experience table (1-100)
const long long EXPERIENCE_TABLE[] = {
  0, 100, 250, 450, 700, 1000, 1350, 1750, 2200, 2700,          // 1-10
  3250, 3850, 4500, 5200, 5950, 6750, 7600, 8500, 9450, 10450,  // 11-20
  11500, 12600, 13750, 14950, 16200, 17500, 18850, 20250, 21700, 23200,  // 21-30
  24750, 26350, 28000, 29700, 31450, 33250, 35100, 37000, 38950, 40950,  // 31-40
  43000, 45100, 47250, 49450, 51700, 54000, 56350, 58750, 61200, 63700,  // 41-50
  66250, 68850, 71500, 74200, 76950, 79750, 82600, 85500, 88450, 91450,  // 51-60
  94500, 97600, 100750, 103950, 107200, 110500, 113850, 117250, 120700, 124200,  // 61-70
  127750, 131350, 135000, 138700, 142450, 146250, 150100, 154000, 157950, 161950,  // 71-80
  166000, 170100, 174250, 178450, 182700, 187000, 191350, 195750, 200200, 204700,  // 81-90
  209250, 213850, 218500, 223200, 227950, 232750, 237600, 242500, 247450, 252450   // 91-100
};

// Title data for each character
const char* LUFFY_TITLES[] = {
  "Rookie Pirate", "Brave Sailor", "Rubber Man", "Straw Hat Captain", "Gear Second User",
  "Gear Third Master", "Haki Awakener", "Conqueror's Haki", "Gear Fourth: Boundman",
  "Snakeman", "Tankman", "Advanced Haki", "Wano Warrior", "Roof Piece Fighter",
  "Joy Boy Candidate", "Nika Awakening", "Gear Fifth User", "Sun God Transformation",
  "Cartoon Physics", "Reality Bender", "Joy Boy", "Sun God Nika"
};

const char* JINWOO_TITLES[] = {
  "E-Rank Hunter", "Weakest Hunter", "System User", "Double Dungeon Survivor", "D-Rank Hunter",
  "C-Rank Hunter", "Shadow Extract User", "B-Rank Hunter", "Demon Castle Conqueror",
  "A-Rank Hunter", "Knight Slayer", "S-Rank Hunter", "Nation Level Hunter", "Shadow Army Commander",
  "Ruler Fragment", "Shadow Sovereign", "King's Domain", "True Shadow Monarch",
  "Time Reversal", "World Reset", "Cup of Reincarnation", "Shadow Monarch"
};

const char* YUGO_TITLES[] = {
  "Eliatrope Child", "Portal Novice", "Wakfu Learner", "Brotherhood Member", "Zaap User",
  "Portal Opener", "Wakfu Channeler", "Dimensional Traveler", "Eliacube Seeker",
  "Dragon Bond", "Adamai's Brother", "King's Court", "Eliatrope King", "Krosmoz Defender",
  "Dimension Walker", "Time-Space Master", "Wakfu Guardian", "Dragon King Path / Portal Master Path",
  "Primordial Power", "Krosmoz Guardian", "Eliatrope God-King", "Master of Dimensions"
};

// =============================================================================
// RPG SYSTEM
// =============================================================================

void initRPGSystem() {
  Serial.println("[RPG] Initializing RPG system...");
  
  if (!loadRPGProgress()) {
    // Initialize default character based on current theme
    switch(system_state.current_theme) {
      case THEME_LUFFY_GEAR5:  selectRPGCharacter(RPG_LUFFY); break;
      case THEME_SUNG_JINWOO:  selectRPGCharacter(RPG_JINWOO); break;
      case THEME_YUGO_WAKFU:   selectRPGCharacter(RPG_YUGO); break;
      default: selectRPGCharacter(RPG_LUFFY); break;
    }
  }
}

void updateRPGSystem() {
  // Check for level ups
  while (canLevelUp()) {
    levelUp();
  }
  
  // Update stats
  calculateStats();
}

void saveRPGProgress() {
  // Save to SD card if available
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    Serial.println("[RPG] SD Card not available, progress not saved");
    return;
  }
  
  File dataFile = SD_MMC.open("/WATCH/data/rpg_progress.dat", FILE_WRITE);
  if (!dataFile) {
    Serial.println("[RPG] Cannot save progress");
    return;
  }
  
  dataFile.printf("VERSION=1\n");
  dataFile.printf("CHARACTER_TYPE=%d\n", (int)rpg_character.character_type);
  dataFile.printf("LEVEL=%d\n", rpg_character.level);
  dataFile.printf("EXPERIENCE=%lld\n", rpg_character.experience);
  dataFile.printf("YUGO_PATH=%d\n", (int)rpg_character.yugo_path);
  dataFile.printf("SHADOW_ARMY=%d\n", rpg_character.shadow_army_size);
  dataFile.printf("AWAKENED=%d\n", rpg_character.awakened_form ? 1 : 0);
  dataFile.printf("QUESTS_COMPLETED=%d\n", rpg_character.quests_completed);
  dataFile.printf("BATTLES_WON=%d\n", rpg_character.battles_won);
  dataFile.printf("DAYS_ACTIVE=%d\n", rpg_character.days_active);
  dataFile.printf("TOTAL_STEPS=%ld\n", rpg_character.total_steps);
  
  // Save base stats
  dataFile.printf("BASE_STR=%d\n", rpg_character.base_stats.strength);
  dataFile.printf("BASE_SPD=%d\n", rpg_character.base_stats.speed);
  dataFile.printf("BASE_INT=%d\n", rpg_character.base_stats.intelligence);
  dataFile.printf("BASE_END=%d\n", rpg_character.base_stats.endurance);
  dataFile.printf("BASE_MAG=%d\n", rpg_character.base_stats.magic);
  dataFile.printf("BASE_SPECIAL=%d\n", rpg_character.base_stats.special_power);
  
  dataFile.close();
  Serial.println("[RPG] Progress saved to SD card");
}

bool loadRPGProgress() {
  // Load from SD card if available
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    return false;
  }
  
  File dataFile = SD_MMC.open("/WATCH/data/rpg_progress.dat", FILE_READ);
  if (!dataFile) {
    Serial.println("[RPG] No saved progress found");
    return false;
  }
  
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();
    
    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;
    
    String key = line.substring(0, eqPos);
    String value = line.substring(eqPos + 1);
    
    if (key == "CHARACTER_TYPE") rpg_character.character_type = (RPGCharacterType)value.toInt();
    else if (key == "LEVEL") rpg_character.level = value.toInt();
    else if (key == "EXPERIENCE") rpg_character.experience = value.toInt();
    else if (key == "YUGO_PATH") rpg_character.yugo_path = (YugoEndgamePath)value.toInt();
    else if (key == "SHADOW_ARMY") rpg_character.shadow_army_size = value.toInt();
    else if (key == "AWAKENED") rpg_character.awakened_form = value.toInt() == 1;
    else if (key == "QUESTS_COMPLETED") rpg_character.quests_completed = value.toInt();
    else if (key == "BATTLES_WON") rpg_character.battles_won = value.toInt();
    else if (key == "DAYS_ACTIVE") rpg_character.days_active = value.toInt();
    else if (key == "TOTAL_STEPS") rpg_character.total_steps = value.toInt();
    else if (key == "BASE_STR") rpg_character.base_stats.strength = value.toInt();
    else if (key == "BASE_SPD") rpg_character.base_stats.speed = value.toInt();
    else if (key == "BASE_INT") rpg_character.base_stats.intelligence = value.toInt();
    else if (key == "BASE_END") rpg_character.base_stats.endurance = value.toInt();
    else if (key == "BASE_MAG") rpg_character.base_stats.magic = value.toInt();
    else if (key == "BASE_SPECIAL") rpg_character.base_stats.special_power = value.toInt();
  }
  
  dataFile.close();
  
  // Calculate current stats and experience needed
  rpg_character.experience_to_next = getExperienceRequiredForLevel(rpg_character.level + 1);
  calculateStats();
  
  Serial.printf("[RPG] Progress loaded: Level %d, %lld XP\n", 
                rpg_character.level, rpg_character.experience);
  return true;
}

void selectRPGCharacter(RPGCharacterType character) {
  rpg_character.character_type = character;
  rpg_character.level = 1;
  rpg_character.experience = 0;
  rpg_character.experience_to_next = EXPERIENCE_TABLE[1];
  
  // Initialize base stats based on character
  switch(character) {
    case RPG_LUFFY:
      rpg_character.base_stats = {10, 8, 5, 12, 3, 5, 43};  // STR, SPD, INT, END, MAG, SPECIAL
      break;
    case RPG_JINWOO:
      rpg_character.base_stats = {8, 10, 8, 7, 6, 6, 45};
      break;
    case RPG_YUGO:
      rpg_character.base_stats = {5, 9, 12, 6, 10, 8, 50};
      break;
  }
  
  rpg_character.yugo_path = YUGO_UNDECIDED;
  rpg_character.shadow_army_size = 0;
  rpg_character.awakened_form = false;
  rpg_character.quests_completed = 0;
  rpg_character.battles_won = 0;
  rpg_character.days_active = 0;
  rpg_character.total_steps = 0;
  
  calculateStats();
}

void calculateStats() {
  // Current stats = base stats + level bonuses + equipment
  int level_bonus = rpg_character.level - 1;
  
  rpg_character.current_stats.strength = rpg_character.base_stats.strength + level_bonus * 2;
  rpg_character.current_stats.speed = rpg_character.base_stats.speed + level_bonus * 2;
  rpg_character.current_stats.intelligence = rpg_character.base_stats.intelligence + level_bonus;
  rpg_character.current_stats.endurance = rpg_character.base_stats.endurance + level_bonus * 2;
  rpg_character.current_stats.magic = rpg_character.base_stats.magic + level_bonus;
  rpg_character.current_stats.special_power = rpg_character.base_stats.special_power + level_bonus;
  
  // Total power calculation
  rpg_character.current_stats.total_power = 
    rpg_character.current_stats.strength +
    rpg_character.current_stats.speed +
    rpg_character.current_stats.intelligence +
    rpg_character.current_stats.endurance +
    rpg_character.current_stats.magic +
    rpg_character.current_stats.special_power;
}

RPGCharacterData* getCurrentCharacter() {
  return &rpg_character;
}

// =============================================================================
// EXPERIENCE & LEVELING
// =============================================================================

void gainExperience(int amount, const char* source) {
  rpg_character.experience += amount;
  Serial.printf("[RPG] Gained %d XP from %s\n", amount, source);
  
  if (canLevelUp()) {
    levelUp();
  }
}

void levelUp() {
  if (rpg_character.level >= 100) return;
  
  rpg_character.level++;
  rpg_character.experience_to_next = getExperienceRequiredForLevel(rpg_character.level + 1);
  
  Serial.printf("[RPG] Level Up! Now level %d\n", rpg_character.level);
  
  calculateStats();
  checkTitleUnlocks();
  
  // Special events at certain levels
  if (rpg_character.level == 50) {
    rpg_character.awakened_form = true;
    Serial.println("[RPG] Awakened form unlocked!");
  }
  
  if (rpg_character.character_type == RPG_YUGO && rpg_character.level == 90) {
    // Yugo path selection available
    Serial.println("[RPG] Yugo endgame path selection available!");
  }
}

bool canLevelUp() {
  if (rpg_character.level >= 100) return false;
  return rpg_character.experience >= getExperienceRequiredForLevel(rpg_character.level + 1);
}

long long getExperienceRequiredForLevel(int level) {
  if (level <= 0) return 0;
  if (level > 100) return EXPERIENCE_TABLE[100];
  return EXPERIENCE_TABLE[level];
}

// =============================================================================
// TITLES
// =============================================================================

void checkTitleUnlocks() {
  // Titles unlock at specific levels
  int title_index = (rpg_character.level - 1) / 5;  // New title every 5 levels
  if (title_index > 21) title_index = 21;
  
  Serial.printf("[RPG] Title: %s\n", getTitleForLevel(rpg_character.character_type, rpg_character.level));
}

const char* getTitleForLevel(RPGCharacterType character, int level) {
  int idx = (level - 1) / 5;
  if (idx > 21) idx = 21;
  
  switch(character) {
    case RPG_LUFFY: return LUFFY_TITLES[idx];
    case RPG_JINWOO: return JINWOO_TITLES[idx];
    case RPG_YUGO: return YUGO_TITLES[idx];
    default: return "Unknown";
  }
}

const char* getCharacterName(RPGCharacterType character) {
  switch(character) {
    case RPG_LUFFY: return "Monkey D. Luffy";
    case RPG_JINWOO: return "Sung Jin-Woo";
    case RPG_YUGO: return "Yugo";
    default: return "Unknown";
  }
}

const char* getSpecialStatName(RPGCharacterType character) {
  switch(character) {
    case RPG_LUFFY: return "Haki";
    case RPG_JINWOO: return "Shadow Power";
    case RPG_YUGO: return "Wakfu";
    default: return "Special";
  }
}

uint16_t getTitleTierColor(int tier) {
  uint16_t colors[] = {COLOR_GRAY, COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_ORANGE, COLOR_GOLD};
  if (tier >= 0 && tier < 6) return colors[tier];
  return COLOR_WHITE;
}

// =============================================================================
// CHARACTER PATHS
// =============================================================================

void initLuffyProgression() {
  selectRPGCharacter(RPG_LUFFY);
}

void initYugoProgression() {
  selectRPGCharacter(RPG_YUGO);
}

void initJinwooProgression() {
  selectRPGCharacter(RPG_JINWOO);
}

void selectYugoEndgamePath(YugoEndgamePath path) {
  if (rpg_character.character_type != RPG_YUGO) return;
  if (rpg_character.level < 90) return;
  
  rpg_character.yugo_path = path;
  Serial.printf("[RPG] Yugo selected path: %s\n", 
    path == YUGO_DRAGON_KING ? "Dragon King" : "Portal Master");
}

// =============================================================================
// RPG UI
// =============================================================================

void drawRPGOverview() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  // Title
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(100, 20);
  gfx->print("RPG STATUS");
  
  // Character name
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(2);
  gfx->setCursor(50, 70);
  gfx->print(getCharacterName(rpg_character.character_type));
  
  // Level
  gfx->setTextSize(3);
  gfx->setCursor(100, 110);
  gfx->printf("Lv.%d", rpg_character.level);
  
  // Title
  gfx->setTextSize(1);
  gfx->setTextColor(getTitleTierColor(rpg_character.level / 20));
  gfx->setCursor(50, 160);
  gfx->print(getTitleForLevel(rpg_character.character_type, rpg_character.level));
  
  // XP bar
  long long current_xp = rpg_character.experience;
  long long needed_xp = rpg_character.experience_to_next;
  float xp_progress = (float)current_xp / max(1LL, needed_xp);
  
  drawThemeProgressBar(40, 200, 280, 25, xp_progress, "Experience");
  
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(1);
  gfx->setCursor(40, 235);
  gfx->printf("%lld / %lld XP", current_xp, needed_xp);
  
  // Stats
  drawCharacterStats();
}

void drawCharacterStats() {
  int startY = 280;
  int col1 = 40, col2 = 200;
  
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(1);
  
  gfx->setCursor(col1, startY);
  gfx->printf("STR: %d", rpg_character.current_stats.strength);
  gfx->setCursor(col2, startY);
  gfx->printf("SPD: %d", rpg_character.current_stats.speed);
  
  gfx->setCursor(col1, startY + 25);
  gfx->printf("INT: %d", rpg_character.current_stats.intelligence);
  gfx->setCursor(col2, startY + 25);
  gfx->printf("END: %d", rpg_character.current_stats.endurance);
  
  gfx->setCursor(col1, startY + 50);
  gfx->printf("MAG: %d", rpg_character.current_stats.magic);
  gfx->setCursor(col2, startY + 50);
  gfx->printf("%s: %d", getSpecialStatName(rpg_character.character_type), 
              rpg_character.current_stats.special_power);
  
  gfx->setCursor(col1, startY + 80);
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->printf("Total Power: %d", rpg_character.current_stats.total_power);
}

void drawTitleScreen() {
  // Draw title collection screen
}

void drawAbilityTree() {
  // Draw ability unlock tree
}

void drawInventory() {
  // Draw inventory screen
}

void drawLevelUpAnimation() {
  // Level up animation
  for (int i = 0; i < 3; i++) {
    gfx->fillScreen(getCurrentTheme()->primary);
    delay(100);
    gfx->fillScreen(RGB565(2, 2, 5));
    delay(100);
  }
}

// =============================================================================
// QUEST INTEGRATION
// =============================================================================

void completeRPGQuest(const char* quest_name, int xp_reward) {
  gainExperience(xp_reward, quest_name);
  rpg_character.quests_completed++;
}

void generateDailyRPGQuests() {
  // Generate quests based on character type
}

int calculateRPGXP(int quest_type, int steps_taken, int completion_time) {
  int base_xp = 10;
  base_xp += steps_taken / 100;
  if (completion_time < 3600) base_xp *= 1.5;  // Bonus for fast completion
  return base_xp;
}

// =============================================================================
// QUEST SYSTEM
// =============================================================================

void initializeQuests() {
  Serial.println("[Quest] Initializing quest system...");
  generateDailyQuests();
}

void generateDailyQuests() {
  active_quest_count = 0;
  
  // Generate 3 daily quests
  active_quests[0] = {"Morning Walk", "Take 1000 steps", QUEST_STEPS, QUEST_LUFFY, QUEST_EASY, 1000, 0, 50, 0, millis(), false, false, true};
  active_quests[1] = {"Active Hour", "Stay active for 30 minutes", QUEST_ACTIVE_TIME, QUEST_JINWOO, QUEST_MEDIUM, 30, 0, 100, 0, millis(), false, false, true};
  active_quests[2] = {"Distance Challenge", "Walk 2km", QUEST_DISTANCE, QUEST_YUGO, QUEST_HARD, 2000, 0, 150, 0, millis(), false, false, true};
  
  active_quest_count = 3;
}

void generateUrgentQuest() {
  if (active_quest_count >= 10) return;
  
  active_quests[active_quest_count] = {"Urgent Mission!", "Complete 500 steps in 10 minutes", QUEST_STEPS, 
    (QuestCharacter)system_state.current_theme, QUEST_LEGENDARY, 500, 0, 300, 600000, millis(), false, true, false};
  active_quest_count++;
}

void updateQuestProgress() {
  for (int i = 0; i < active_quest_count; i++) {
    if (active_quests[i].completed) continue;
    
    switch(active_quests[i].type) {
      case QUEST_STEPS:
        active_quests[i].current_progress = system_state.steps_today;
        break;
      case QUEST_DISTANCE:
        active_quests[i].current_progress = (int)(system_state.steps_today * 0.8);  // ~0.8m per step
        break;
      case QUEST_ACTIVE_TIME:
        // Track active minutes
        break;
      default:
        break;
    }
    
    if (active_quests[i].current_progress >= active_quests[i].target_value) {
      completeQuest(i);
    }
  }
}

bool completeQuest(int quest_id) {
  if (quest_id < 0 || quest_id >= active_quest_count) return false;
  if (active_quests[quest_id].completed) return false;
  
  active_quests[quest_id].completed = true;
  giveQuestReward(active_quests[quest_id]);
  showQuestCompleted(active_quests[quest_id]);
  
  return true;
}

void drawQuestScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(120, 20);
  gfx->print("QUESTS");
  
  for (int i = 0; i < min(active_quest_count, 4); i++) {
    drawQuestCard(20, 70 + i * 90, 320, 80, active_quests[i]);
  }
  
  drawThemeButton(140, 420, 80, 30, "Back", false);
}

// Touch handler for quest screen
void handleQuestTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  // Check for quest taps to claim rewards
  for (int i = 0; i < min(active_quest_count, 4); i++) {
    int y = 70 + i * 90;
    if (gesture.y >= y && gesture.y < y + 80) {
      if (active_quests[i].completed && !active_quests[i].claimed) {
        giveQuestReward(active_quests[i]);
        drawQuestScreen();
      }
      return;
    }
  }
  
  // Back button
  if (gesture.y >= 410) {
    system_state.current_screen = SCREEN_CHARACTER_STATS;
  }
}

void drawQuestCard(int x, int y, int w, int h, QuestData& quest) {
  uint16_t bg = quest.completed ? RGB565(0, 60, 0) : RGB565(40, 40, 40);
  if (quest.urgent) bg = RGB565(80, 0, 0);
  
  gfx->fillRect(x, y, w, h, bg);
  gfx->drawRect(x, y, w, h, getCurrentTheme()->primary);
  
  // Title
  gfx->setTextColor(quest.urgent ? COLOR_ORANGE : COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 10, y + 10);
  gfx->print(quest.title);
  
  // Description
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setCursor(x + 10, y + 28);
  gfx->print(quest.description);
  
  // Progress bar
  float progress = (float)quest.current_progress / max(1, quest.target_value);
  int barW = (int)((w - 20) * min(1.0f, progress));
  gfx->fillRect(x + 10, y + 50, w - 20, 15, RGB565(60, 60, 60));
  if (barW > 0) {
    uint16_t barColor = quest.completed ? COLOR_GREEN : getCurrentTheme()->primary;
    gfx->fillRect(x + 10, y + 50, barW, 15, barColor);
  }
  
  // Progress text
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setCursor(x + 10, y + 68);
  gfx->printf("%d/%d  +%dXP", quest.current_progress, quest.target_value, quest.reward_points);
}

void showQuestNotification(QuestData& quest) {
  drawThemeNotification("New Quest!", quest.title);
}

void showQuestCompleted(QuestData& quest) {
  drawThemeNotification("Quest Complete!", quest.title);
}

QuestData generateLuffyQuest() {
  return {"Pirate Training", "Train like a captain!", QUEST_STEPS, QUEST_LUFFY, QUEST_MEDIUM, 2000, 0, 100, 0, millis(), false, false, true};
}

QuestData generateJinwooQuest() {
  return {"Shadow Training", "Level up your shadow power!", QUEST_STEPS, QUEST_JINWOO, QUEST_HARD, 3000, 0, 150, 0, millis(), false, false, true};
}

QuestData generateYugoQuest() {
  return {"Wakfu Mastery", "Channel your wakfu energy!", QUEST_ACTIVE_TIME, QUEST_YUGO, QUEST_MEDIUM, 20, 0, 120, 0, millis(), false, false, true};
}

void giveQuestReward(QuestData& quest) {
  gainExperience(quest.reward_points, quest.title);
}

void updatePlayerLevel() {
  updateRPGSystem();
}

int getPlayerLevel() {
  return rpg_character.level;
}

int getPlayerXP() {
  return (int)rpg_character.experience;
}
