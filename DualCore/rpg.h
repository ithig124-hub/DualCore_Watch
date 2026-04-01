/*
 * rpg.h - RPG System
 * Consolidates: rpg_system.h/cpp, rpg_data.h/cpp, rpg_ui.cpp, quests.h/cpp, hybrid_features.h/cpp
 * 
 * Solo Leveling-inspired character progression with Luffy, Yugo, and Jin-Woo paths
 */

#ifndef RPG_H
#define RPG_H

#include "types.h"  // Must include types.h for RPGCharacterType, QuestData, etc.
#include "config.h"

// =============================================================================
// RPG SYSTEM
// =============================================================================

void initRPGSystem();
void updateRPGSystem();
void saveRPGProgress();
bool loadRPGProgress();

void selectRPGCharacter(RPGCharacterType character);
void calculateStats();
RPGCharacterData* getCurrentCharacter();

// Experience & Leveling
void gainExperience(int amount, const char* source);
void levelUp();
bool canLevelUp();
long long getExperienceRequiredForLevel(int level);

// Titles
void checkTitleUnlocks();
const char* getTitleForLevel(RPGCharacterType character, int level);
const char* getCharacterName(RPGCharacterType character);
const char* getSpecialStatName(RPGCharacterType character);
uint16_t getTitleTierColor(int tier);

// Character paths
void initLuffyProgression();
void initYugoProgression();
void initJinwooProgression();
void selectYugoEndgamePath(YugoEndgamePath path);

// RPG UI
void drawRPGOverview();
void drawCharacterStats();
void drawTitleScreen();
void drawAbilityTree();
void drawInventory();
void drawLevelUpAnimation();

// Quest integration
void completeRPGQuest(const char* quest_name, int xp_reward);
void generateDailyRPGQuests();
int calculateRPGXP(int quest_type, int steps_taken, int completion_time);

// Global state
extern RPGCharacterData rpg_character;

// =============================================================================
// QUEST SYSTEM
// =============================================================================

void initializeQuests();
void generateDailyQuests();
void generateUrgentQuest();
void updateQuestProgress();
bool completeQuest(int quest_id);

void drawQuestScreen();
void handleQuestTouch(TouchGesture& gesture);  // Quest screen touch handler
void drawQuestCard(int x, int y, int w, int h, QuestData& quest);
void showQuestNotification(QuestData& quest);
void showQuestCompleted(QuestData& quest);

// Character-specific quests
QuestData generateLuffyQuest();
QuestData generateJinwooQuest();
QuestData generateYugoQuest();

// Quest rewards
void giveQuestReward(QuestData& quest);
void updatePlayerLevel();
int getPlayerLevel();
int getPlayerXP();

// Quest arrays
extern QuestData active_quests[];
extern int active_quest_count;

#endif // RPG_H
