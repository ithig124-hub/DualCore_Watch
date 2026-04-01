/*
 * daily_quests.h - Daily Quest System
 */

#ifndef DAILY_QUESTS_H
#define DAILY_QUESTS_H

#include <Arduino.h>
#include "config.h"

// QuestType and QuestDifficulty are already defined in types.h
// So we don't redefine them here

struct Quest {
  QuestType type;
  QuestDifficulty difficulty;
  const char* name;
  const char* description;
  uint32_t target;
  uint32_t progress;
  bool completed;
  uint32_t reward_gold;
  uint16_t reward_xp;
  uint8_t reward_gems;
};

struct DailyQuestsData {
  Quest daily_quests[3];
  uint8_t current_day;
  uint32_t total_completed;
  uint32_t streak_days;
  unsigned long last_reset;
};

void initDailyQuests();
void drawDailyQuestsScreen();
void handleDailyQuestsTouch(TouchGesture& gesture);
void generateNewDailyQuests();  // Renamed to avoid conflict with rpg.cpp
void updateQuestProgress(QuestType type, uint32_t amount);
void claimQuestReward(int questIndex);
void checkDailyReset();
void saveDailyQuestsData();
void loadDailyQuestsData();

#endif
