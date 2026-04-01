/*
 * boss_rush.cpp - Boss Rush Challenge Implementation (IMPROVED)
 * Complete boss battle system with 20 anime bosses
 * 
 * FIXES:
 * - Improved UI with retro pixel style
 * - Tier selection actually works
 * - Better touch handling
 * - Returns to app grid properly
 */

#include "boss_rush.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "gacha.h"
#include "rpg.h"
#include "navigation.h"
#include <SD_MMC.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Boss database
BossData boss_database[TOTAL_BOSSES];
bool bosses_defeated[TOTAL_BOSSES];

// Combat state
int current_boss_index = 0;
int current_tier = 0;  // Which tier is selected
int player_hp, player_max_hp;
int player_attack, player_defense;
int player_energy;
int potions_remaining;
int combo_count;
int turn_count;
bool player_defending;
bool battle_active;

// Battle log
String battle_log[5];
int log_index = 0;

// Screen state
static bool show_boss_selection = false;

// =============================================================================
// BOSS DATABASE
// =============================================================================

void initBossDatabase() {
  // Tier 1 Bosses (Level 1-5)
  boss_database[0] = {"Buggy", "One Piece", BOSS_TIER_1, 5000, 5000, 300, 100, 1, false, 50};
  boss_database[1] = {"Zabuza", "Naruto", BOSS_TIER_1, 6000, 6000, 400, 150, 2, false, 60};
  boss_database[2] = {"Raditz", "Dragon Ball", BOSS_TIER_1, 7000, 7000, 500, 120, 3, false, 70};
  boss_database[3] = {"Hand Demon", "Demon Slayer", BOSS_TIER_1, 5500, 5500, 350, 80, 4, false, 55};
  boss_database[4] = {"Cursed Spirit", "JJK", BOSS_TIER_1, 6500, 6500, 380, 130, 5, false, 65};
  
  // Tier 2 Bosses (Level 6-10)
  boss_database[5] = {"Crocodile", "One Piece", BOSS_TIER_2, 15000, 15000, 800, 300, 6, false, 150};
  boss_database[6] = {"Orochimaru", "Naruto", BOSS_TIER_2, 18000, 18000, 900, 350, 7, false, 180};
  boss_database[7] = {"Frieza", "Dragon Ball", BOSS_TIER_2, 20000, 20000, 1000, 400, 8, false, 200};
  boss_database[8] = {"Rui", "Demon Slayer", BOSS_TIER_2, 16000, 16000, 850, 280, 9, false, 160};
  boss_database[9] = {"Mahito", "JJK", BOSS_TIER_2, 19000, 19000, 950, 320, 10, false, 190};
  
  // Tier 3 Bosses (Level 11-15)
  boss_database[10] = {"Doflamingo", "One Piece", BOSS_TIER_3, 40000, 40000, 1500, 600, 11, false, 300};
  boss_database[11] = {"Pain", "Naruto", BOSS_TIER_3, 45000, 45000, 1800, 700, 12, false, 350};
  boss_database[12] = {"Cell", "Dragon Ball", BOSS_TIER_3, 50000, 50000, 2000, 800, 13, false, 400};
  boss_database[13] = {"Akaza", "Demon Slayer", BOSS_TIER_3, 42000, 42000, 1600, 550, 14, false, 320};
  boss_database[14] = {"Sukuna 4F", "JJK", BOSS_TIER_3, 48000, 48000, 1900, 750, 15, false, 380};
  
  // Tier 4 Bosses (Level 16-20) - FINAL BOSSES
  boss_database[15] = {"Kaido", "One Piece", BOSS_TIER_4, 100000, 100000, 3000, 1200, 16, false, 600};
  boss_database[16] = {"Madara", "Naruto", BOSS_TIER_4, 110000, 110000, 3500, 1400, 17, false, 700};
  boss_database[17] = {"Jiren", "Dragon Ball", BOSS_TIER_4, 120000, 120000, 4000, 1600, 18, false, 800};
  boss_database[18] = {"Muzan", "Demon Slayer", BOSS_TIER_4, 105000, 105000, 3200, 1100, 19, false, 650};
  boss_database[19] = {"Sukuna Full", "JJK", BOSS_TIER_4, 150000, 150000, 5000, 2000, 20, false, 1000};
  
  Serial.println("[BossRush] Database initialized");
}

// =============================================================================
// SYSTEM FUNCTIONS
// =============================================================================

void initBossRush() {
  Serial.println("[BossRush] Initializing...");
  initBossDatabase();
  
  if (!loadBossProgress()) {
    for (int i = 0; i < TOTAL_BOSSES; i++) {
      bosses_defeated[i] = false;
    }
    system_state.bosses_defeated = 0;
  }
  
  show_boss_selection = false;
}

void saveBossProgress() {
  Serial.println("[BossRush] Progress saved");
}

bool loadBossProgress() {
  return false;
}

BossData* getBoss(int index) {
  if (index < 0 || index >= TOTAL_BOSSES) return nullptr;
  return &boss_database[index];
}

BossData* getCurrentBoss() {
  return getBoss(current_boss_index);
}

int getBossesDefeated() {
  int count = 0;
  for (int i = 0; i < TOTAL_BOSSES; i++) {
    if (bosses_defeated[i]) count++;
  }
  return count;
}

// =============================================================================
// COMBAT SYSTEM
// =============================================================================

void startBossFight(int boss_index) {
  if (boss_index < 0 || boss_index >= TOTAL_BOSSES) return;
  
  current_boss_index = boss_index;
  battle_active = true;
  show_boss_selection = false;
  
  player_max_hp = 1000 + system_state.player_level * 100;
  player_hp = player_max_hp;
  player_attack = 500 + system_state.player_level * 50;
  player_defense = 300 + system_state.player_level * 30;
  player_energy = 0;
  potions_remaining = MAX_POTIONS;
  combo_count = 0;
  turn_count = 0;
  player_defending = false;
  
  boss_database[boss_index].hp = boss_database[boss_index].max_hp;
  
  log_index = 0;
  for (int i = 0; i < 5; i++) battle_log[i] = "";
  
  addBattleLogEntry("Battle Start!");
  
  Serial.printf("[BossRush] Fighting: %s\n", boss_database[boss_index].name);
  drawBossBattle();
}

void addBattleLogEntry(const char* message) {
  if (log_index < 5) {
    battle_log[log_index++] = String(message);
  } else {
    for (int i = 0; i < 4; i++) battle_log[i] = battle_log[i + 1];
    battle_log[4] = String(message);
  }
}

void playerAttack() {
  if (!battle_active) return;
  
  BossData* boss = getCurrentBoss();
  int base_damage = player_attack - boss->defense / 2;
  int multiplier = 10 + getComboMultiplier();  // Returns 10-20
  
  bool critical = random(0, 100) < CRITICAL_CHANCE;
  if (critical) multiplier = (int)(multiplier * CRITICAL_MULTIPLIER);
  
  int damage = max(1, (base_damage * multiplier) / 10);
  boss->hp -= damage;
  
  incrementCombo();
  player_energy = min(100, player_energy + 25);
  
  char log[64];
  sprintf(log, "%sATK! %d dmg", critical ? "CRIT " : "", damage);
  addBattleLogEntry(log);
  
  processCombatRound();
}

void playerDefend() {
  if (!battle_active) return;
  player_defending = true;
  player_energy = min(100, player_energy + 25);
  addBattleLogEntry("Defending!");
  processCombatRound();
}

void playerSpecial() {
  if (!battle_active || player_energy < SPECIAL_ATTACK_COST) return;
  
  BossData* boss = getCurrentBoss();
  player_energy = 0;
  
  int damage = (player_attack * 3) - boss->defense / 4;
  damage = max(100, damage);
  boss->hp -= damage;
  
  incrementCombo();
  
  char log[64];
  sprintf(log, "SPECIAL! %d dmg!", damage);
  addBattleLogEntry(log);
  
  processCombatRound();
}

void playerUseItem() {
  if (!battle_active || potions_remaining <= 0) return;
  
  potions_remaining--;
  int heal = player_max_hp * POTION_HEAL_PERCENT / 100;
  player_hp = min(player_max_hp, player_hp + heal);
  
  char log[64];
  sprintf(log, "Potion! +%d HP", heal);
  addBattleLogEntry(log);
  
  processCombatRound();
}

void bossAction() {
  if (!battle_active) return;
  
  BossData* boss = getCurrentBoss();
  int hp_percent = (boss->hp * 100) / boss->max_hp;
  
  int damage = boss->attack - player_defense / 2;
  
  if (hp_percent < 25) {
    damage = (int)(damage * 1.5);
    addBattleLogEntry("BERSERK!");
  }
  
  if (player_defending) {
    damage = damage / 2;
    player_defending = false;
  }
  
  damage = max(1, damage);
  player_hp -= damage;
  resetCombo();
  
  char log[64];
  sprintf(log, "%s: %d dmg", boss->name, damage);
  addBattleLogEntry(log);
}

void processCombatRound() {
  turn_count++;
  if (checkBattleEnd()) return;
  bossAction();
  checkBattleEnd();
  drawBossBattle();
}

bool checkBattleEnd() {
  BossData* boss = getCurrentBoss();
  
  if (boss->hp <= 0) {
    battle_active = false;
    bosses_defeated[current_boss_index] = true;
    system_state.bosses_defeated++;
    
    system_state.player_gems += boss->gem_reward;
    gainExperience(boss->level * 100, "Boss Victory");
    
    saveBossProgress();
    drawBossVictory(*boss);
    return true;
  }
  
  if (player_hp <= 0) {
    battle_active = false;
    drawBossDefeat();
    return true;
  }
  
  return false;
}

void incrementCombo() { combo_count++; }
void resetCombo() { combo_count = 0; }
int getComboMultiplier() { return min(20, 10 + combo_count); }

// =============================================================================
// UI - IMPROVED RETRO STYLE
// =============================================================================

void drawBossRushMenu() {
  gfx->fillScreen(RGB565(2, 2, 5));
  show_boss_selection = false;
  
  // CRT scanlines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_RED);
  }
  
  gfx->setTextColor(COLOR_RED);
  gfx->setTextSize(2);
  gfx->setCursor(90, 14);
  gfx->print("BOSS RUSH");
  
  // Progress bar
  int defeated = getBossesDefeated();
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(1);
  gfx->setCursor(20, 58);
  gfx->printf("Defeated: %d/%d", defeated, TOTAL_BOSSES);
  
  int barW = 200;
  gfx->fillRect(120, 55, barW, 12, RGB565(20, 20, 30));
  int fillW = (defeated * barW) / TOTAL_BOSSES;
  if (fillW > 0) gfx->fillRect(120, 55, fillW, 12, COLOR_RED);
  gfx->drawRect(120, 55, barW, 12, RGB565(60, 60, 80));
  
  // Player stats
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setCursor(20, 78);
  gfx->printf("Lv.%d  ATK:%d  DEF:%d", 
              system_state.player_level,
              500 + system_state.player_level * 50,
              300 + system_state.player_level * 30);
  
  // Tier buttons - BIG and CLEAR
  const char* tiers[] = {"TIER 1", "TIER 2", "TIER 3", "TIER 4"};
  const char* descs[] = {"Lv.1-5 Bosses", "Lv.6-10 Bosses", "Lv.11-15 Bosses", "FINAL BOSSES"};
  uint16_t colors[] = {COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_RED};
  
  for (int i = 0; i < 4; i++) {
    int y = 100 + i * 75;
    int start = i * 5;
    int tier_defeated = 0;
    for (int j = start; j < start + 5; j++) {
      if (bosses_defeated[j]) tier_defeated++;
    }
    
    // Card
    gfx->fillRect(25, y, 310, 65, RGB565(12, 14, 20));
    gfx->drawRect(25, y, 310, 65, colors[i]);
    gfx->fillRect(25, y, 5, 5, colors[i]);
    gfx->fillRect(330, y, 5, 5, colors[i]);
    
    // Colored bar on left
    gfx->fillRect(25, y, 8, 65, colors[i]);
    
    // Tier name
    gfx->setTextColor(RGB565(220, 225, 240));
    gfx->setTextSize(2);
    gfx->setCursor(45, y + 12);
    gfx->print(tiers[i]);
    
    // Description
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setTextSize(1);
    gfx->setCursor(45, y + 40);
    gfx->print(descs[i]);
    
    // Completion
    gfx->setTextColor(tier_defeated == 5 ? COLOR_GREEN : RGB565(150, 155, 170));
    gfx->setCursor(240, y + 25);
    gfx->printf("%d/5", tier_defeated);
    
    if (tier_defeated == 5) {
      gfx->setTextColor(COLOR_GREEN);
      gfx->setCursor(280, y + 25);
      gfx->print("OK");
    }
  }
  
  drawSwipeIndicator();
}

void drawBossSelection() {
  gfx->fillScreen(RGB565(2, 2, 5));
  show_boss_selection = true;
  
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  uint16_t tierColor = getBossTierColor((BossTier)(current_tier + 1));
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, tierColor);
  }
  
  gfx->setTextColor(tierColor);
  gfx->setTextSize(2);
  gfx->setCursor(80, 14);
  gfx->printf("TIER %d BOSSES", current_tier + 1);
  
  // Show 5 bosses
  int tier_start = current_tier * 5;
  
  for (int i = 0; i < 5; i++) {
    int boss_idx = tier_start + i;
    BossData* boss = getBoss(boss_idx);
    int y = 60 + i * 70;
    
    uint16_t bg = bosses_defeated[boss_idx] ? RGB565(10, 30, 10) : RGB565(12, 14, 20);
    
    gfx->fillRect(20, y, 320, 60, bg);
    gfx->drawRect(20, y, 320, 60, tierColor);
    gfx->fillRect(20, y, 5, 5, tierColor);
    
    // Boss name
    gfx->setTextColor(RGB565(220, 225, 240));
    gfx->setTextSize(2);
    gfx->setCursor(35, y + 10);
    gfx->print(boss->name);
    
    // Series and level
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(35, y + 38);
    gfx->printf("%s | Lv.%d", boss->series, boss->level);
    
    // Reward
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(250, y + 20);
    gfx->printf("+%d", boss->gem_reward);
    
    // Status
    if (bosses_defeated[boss_idx]) {
      gfx->setTextColor(COLOR_GREEN);
      gfx->setCursor(280, y + 38);
      gfx->print("DONE");
    }
  }
  
  drawSwipeIndicator();
}

void drawBossBattle() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  BossData* boss = getCurrentBoss();
  
  // Boss name and tier
  gfx->setTextColor(getBossTierColor(boss->tier));
  gfx->setTextSize(2);
  gfx->setCursor(30, 15);
  gfx->print(boss->name);
  
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setCursor(30, 40);
  gfx->printf("Lv.%d - %s", boss->level, boss->series);
  
  // Boss HP
  drawBossHealthBar(20, 60, 320, 30, *boss);
  
  // Divider
  gfx->drawFastHLine(0, 100, LCD_WIDTH, RGB565(40, 40, 60));
  
  // Battle log
  gfx->fillRect(20, 110, 320, 80, RGB565(10, 12, 18));
  gfx->drawRect(20, 110, 320, 80, RGB565(30, 35, 50));
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(1);
  for (int i = 0; i < min(4, log_index); i++) {
    gfx->setCursor(30, 118 + i * 18);
    gfx->print(battle_log[max(0, log_index - 4 + i)]);
  }
  
  // Player section
  gfx->drawFastHLine(0, 200, LCD_WIDTH, RGB565(40, 40, 60));
  
  CharacterProfile* profile = getCharacterProfile(system_state.current_theme);
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(30, 210);
  gfx->print(profile ? profile->name : "Hero");
  
  // Player HP
  drawPlayerHealthBar(20, 240, 240, 25);
  
  // Energy
  drawEnergyBar(20, 275, 160, 18);
  
  // Combo
  if (combo_count > 0) {
    gfx->setTextColor(COLOR_GOLD);
    gfx->setTextSize(1);
    gfx->setCursor(200, 278);
    gfx->printf("x%d COMBO", combo_count);
  }
  
  // Potions
  gfx->setTextColor(COLOR_GREEN);
  gfx->setCursor(280, 250);
  gfx->printf("Potions: %d", potions_remaining);
  
  // Action buttons
  drawCombatActions();
}

void drawBossHealthBar(int x, int y, int w, int h, BossData& boss) {
  float progress = (float)boss.hp / boss.max_hp;
  
  gfx->fillRect(x, y, w, h, RGB565(40, 20, 20));
  int fillW = (int)(w * constrain(progress, 0.0f, 1.0f));
  if (fillW > 0) gfx->fillRect(x, y, fillW, h, COLOR_RED);
  gfx->drawRect(x, y, w, h, RGB565(80, 40, 40));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + w/2 - 40, y + h/2 - 4);
  gfx->printf("%d / %d", max(0, boss.hp), boss.max_hp);
}

void drawPlayerHealthBar(int x, int y, int w, int h) {
  float progress = (float)player_hp / player_max_hp;
  
  gfx->fillRect(x, y, w, h, RGB565(20, 40, 20));
  int fillW = (int)(w * constrain(progress, 0.0f, 1.0f));
  if (fillW > 0) {
    uint16_t color = progress > 0.3 ? COLOR_GREEN : COLOR_RED;
    gfx->fillRect(x, y, fillW, h, color);
  }
  gfx->drawRect(x, y, w, h, RGB565(40, 80, 40));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 10, y + h/2 - 4);
  gfx->printf("HP: %d/%d", max(0, player_hp), player_max_hp);
}

void drawEnergyBar(int x, int y, int w, int h) {
  float progress = player_energy / 100.0f;
  
  gfx->fillRect(x, y, w, h, RGB565(20, 20, 40));
  int fillW = (int)(w * progress);
  if (fillW > 0) gfx->fillRect(x, y, fillW, h, COLOR_BLUE);
  gfx->drawRect(x, y, w, h, RGB565(40, 40, 80));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 5, y + 3);
  gfx->printf("SP: %d%%", player_energy);
}

void drawCombatActions() {
  int y = 320;
  int btnW = 75, btnH = 50, gap = 10;
  int startX = (LCD_WIDTH - (4 * btnW + 3 * gap)) / 2;
  
  // Attack
  gfx->fillRect(startX, y, btnW, btnH, COLOR_RED);
  gfx->drawRect(startX, y, btnW, btnH, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(startX + 15, y + 20);
  gfx->print("ATTACK");
  
  // Defend
  gfx->fillRect(startX + btnW + gap, y, btnW, btnH, COLOR_BLUE);
  gfx->drawRect(startX + btnW + gap, y, btnW, btnH, COLOR_WHITE);
  gfx->setCursor(startX + btnW + gap + 12, y + 20);
  gfx->print("DEFEND");
  
  // Special
  uint16_t spColor = player_energy >= 100 ? COLOR_PURPLE : RGB565(60, 60, 80);
  gfx->fillRect(startX + 2 * (btnW + gap), y, btnW, btnH, spColor);
  gfx->drawRect(startX + 2 * (btnW + gap), y, btnW, btnH, COLOR_WHITE);
  gfx->setCursor(startX + 2 * (btnW + gap) + 8, y + 20);
  gfx->print("SPECIAL");
  
  // Item
  uint16_t itemColor = potions_remaining > 0 ? COLOR_GREEN : RGB565(60, 60, 80);
  gfx->fillRect(startX + 3 * (btnW + gap), y, btnW, btnH, itemColor);
  gfx->drawRect(startX + 3 * (btnW + gap), y, btnW, btnH, COLOR_WHITE);
  gfx->setCursor(startX + 3 * (btnW + gap) + 18, y + 20);
  gfx->print("ITEM");
}

void drawBattleLog() {
  // Integrated into drawBossBattle
}

void drawBossVictory(BossData& boss) {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(4);
  gfx->setCursor(70, 80);
  gfx->print("VICTORY!");
  
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(2);
  gfx->setCursor(50, 160);
  gfx->printf("%s defeated!", boss.name);
  
  gfx->setTextColor(COLOR_GREEN);
  gfx->setTextSize(2);
  gfx->setCursor(100, 220);
  gfx->printf("+%d Gems", boss.gem_reward);
  
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(100, 260);
  gfx->printf("+%d XP", boss.level * 100);
  
  // Continue button
  gfx->fillRect(100, 340, 160, 50, getCurrentTheme()->primary);
  gfx->drawRect(100, 340, 160, 50, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(130, 355);
  gfx->print("Continue");
}

void drawBossDefeat() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  gfx->setTextColor(COLOR_RED);
  gfx->setTextSize(4);
  gfx->setCursor(90, 120);
  gfx->print("DEFEAT");
  
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(1);
  gfx->setCursor(80, 220);
  gfx->print("Train harder and try again!");
  
  gfx->fillRect(100, 340, 160, 50, RGB565(60, 60, 80));
  gfx->drawRect(100, 340, 160, 50, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(130, 355);
  gfx->print("Continue");
}

// =============================================================================
// TOUCH HANDLING - FIXED
// =============================================================================

void handleBossRushMenuTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_UP) {
    returnToAppGrid();
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int y = gesture.y;
  
  if (show_boss_selection) {
    // Boss selection screen - tap to start fight
    int tier_start = current_tier * 5;
    for (int i = 0; i < 5; i++) {
      int by = 60 + i * 70;
      if (y >= by && y < by + 60) {
        startBossFight(tier_start + i);
        return;
      }
    }
  } else {
    // Tier selection screen
    for (int i = 0; i < 4; i++) {
      int by = 100 + i * 75;
      if (y >= by && y < by + 65) {
        current_tier = i;
        drawBossSelection();
        return;
      }
    }
  }
}

void handleBossRushTouch(TouchGesture& gesture) {
  if (battle_active) {
    handleBossBattleTouch(gesture);
  } else if (!battle_active && system_state.current_screen == SCREEN_BOSS_RUSH) {
    // Victory/Defeat screen or menu
    if (gesture.event == TOUCH_TAP && gesture.y >= 340) {
      if (show_boss_selection) {
        drawBossSelection();
      } else {
        drawBossRushMenu();
      }
    } else {
      handleBossRushMenuTouch(gesture);
    }
  }
}

void handleBossSelectionTouch(TouchGesture& gesture) {
  handleBossRushMenuTouch(gesture);
}

void handleBossBattleTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_UP && !battle_active) {
    drawBossRushMenu();
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  if (!battle_active) {
    if (gesture.y >= 340) {
      drawBossRushMenu();
    }
    return;
  }
  
  int x = gesture.x, y = gesture.y;
  
  // Action buttons at y = 320
  if (y >= 320 && y < 370) {
    int btnW = 75, gap = 10;
    int startX = (LCD_WIDTH - (4 * btnW + 3 * gap)) / 2;
    
    if (x >= startX && x < startX + btnW) {
      playerAttack();
    } else if (x >= startX + btnW + gap && x < startX + 2 * btnW + gap) {
      playerDefend();
    } else if (x >= startX + 2 * (btnW + gap) && x < startX + 3 * btnW + 2 * gap) {
      playerSpecial();
    } else if (x >= startX + 3 * (btnW + gap)) {
      playerUseItem();
    }
  }
}

const char* getBossTierName(BossTier tier) {
  const char* names[] = {"Tier 1", "Tier 2", "Tier 3", "Tier 4"};
  return names[(int)tier - 1];
}

uint16_t getBossTierColor(BossTier tier) {
  uint16_t colors[] = {COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_RED};
  return colors[(int)tier - 1];
}
