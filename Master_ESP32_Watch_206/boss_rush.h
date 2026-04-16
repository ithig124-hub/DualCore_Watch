/*
 * boss_rush.h - Boss Rush Challenge Mode
 * 20 anime bosses across 4 tiers
 * Turn-based combat, combo system, rewards
 */

#ifndef BOSS_RUSH_H
#define BOSS_RUSH_H

#include "types.h"  // Must include types.h for BossData, BossTier
#include "config.h"

// =============================================================================
// BOSS RUSH SYSTEM
// =============================================================================

void initBossRush();
void saveBossProgress();
bool loadBossProgress();

// Boss management
BossData* getCurrentBoss();
BossData* getBoss(int index);
int getBossesDefeated();
bool isBossDefeated(int boss_index);
void markBossDefeated(int boss_index);

// Combat
void startBossFight(int boss_index);
void addBattleLogEntry(const char* message);
void playerAttack();
void playerDefend();
void playerSpecial();
void playerUseItem();
void bossAction();
void processCombatRound();
bool checkBattleEnd();

// Combo system
void incrementCombo();
void resetCombo();
int getComboMultiplier();

// Rewards
int calculateBossReward(BossData& boss, bool no_damage, bool fast_clear);
void giveBossReward(int boss_index);

// =============================================================================
// BOSS RUSH UI
// =============================================================================

void drawBossRushMenu();
void drawBossSelection();
void drawBossBattle();
void drawBossHealthBar(int x, int y, int w, int h, BossData& boss);
void drawPlayerHealthBar(int x, int y, int w, int h);
void drawEnergyBar(int x, int y, int w, int h);
void drawCombatActions();
void drawBattleLog();
void drawBossVictory(BossData& boss);
void drawBossDefeat();
void drawBossAttackAnimation(int boss_id, int move_id);  // FUSION OS: Attack animations

void handleBossRushMenuTouch(TouchGesture& gesture);
void handleBossRushTouch(TouchGesture& gesture);  // Alias for menu touch
void handleBossSelectionTouch(TouchGesture& gesture);
void handleBossBattleTouch(TouchGesture& gesture);

// =============================================================================
// BOSS DATABASE
// =============================================================================

extern BossData boss_database[TOTAL_BOSSES];
extern bool bosses_defeated[TOTAL_BOSSES];

void initBossDatabase();
const char* getBossTierName(BossTier tier);
uint16_t getBossTierColor(BossTier tier);

// Combat state
extern int current_boss_index;
extern int player_hp, player_max_hp;
extern int player_attack, player_defense;
extern int player_energy;
extern int potions_remaining;
extern int combo_count;
extern int turn_count;
extern bool player_defending;
extern bool battle_active;

#endif // BOSS_RUSH_H
