/*
 * storyline.h - Story Mode System with NVS Persistence
 * FUSION OS - RPG Story Mode (Memory-Optimized)
 *
 * Uses const char* pointers for dialogue (strings in flash, not RAM)
 */

#ifndef STORYLINE_H
#define STORYLINE_H

#include <Arduino.h>
#include "sd_preferences.h"
#include "config.h"
#include "types.h"

// =============================================================================
// STORY CONSTANTS - EXPANDED TO 15 CHAPTERS
// =============================================================================
#define MAX_CHAPTERS_PER_CHARACTER 15
#define MAX_DIALOGUE_SCREENS 8
#define MAX_BOSS_TYPES 5
#define NOTIFICATION_TIMEOUT_MS 5000  // Auto-dismiss notification after 5 seconds

// Chapter unlock levels (15 chapters with progressive level requirements)
#define CHAPTER_1_LEVEL 1
#define CHAPTER_2_LEVEL 3
#define CHAPTER_3_LEVEL 5
#define CHAPTER_4_LEVEL 8
#define CHAPTER_5_LEVEL 12
#define CHAPTER_6_LEVEL 16
#define CHAPTER_7_LEVEL 20
#define CHAPTER_8_LEVEL 25
#define CHAPTER_9_LEVEL 30
#define CHAPTER_10_LEVEL 35
#define CHAPTER_11_LEVEL 42
#define CHAPTER_12_LEVEL 50
#define CHAPTER_13_LEVEL 60
#define CHAPTER_14_LEVEL 75
#define CHAPTER_15_LEVEL 90

// Rewards
#define XP_CHAPTER_COMPLETE 100
#define XP_BOSS_DEFEAT 200
#define GEMS_CHAPTER_COMPLETE 50
#define GEMS_BOSS_DEFEAT 100

#define STORY_NVS_NAMESPACE "story_data"

// =============================================================================
// CHAPTER UNLOCK NOTIFICATION SYSTEM
// =============================================================================
struct ChapterUnlockNotification {
    bool isActive;                  // Notification currently showing
    bool isPending;                 // Waiting to show
    int chapterUnlocked;            // Which chapter was just unlocked
    unsigned long startTime;        // When notification started (for auto-dismiss)
    bool userDismissed;             // User chose "Later"
    bool userAccepted;              // User chose "Ready"
};

// =============================================================================
// YUGO'S BRANCHING PATH
// =============================================================================
enum YugoStoryPath {
    YUGO_PATH_UNDECIDED = 0,
    YUGO_PATH_DRAGONS,
    YUGO_PATH_PORTALS
};

// Boss visual type for sprite drawing
enum BossVisualType {
    BOSS_HUMANOID = 0,
    BOSS_BEAST,
    BOSS_GIANT,
    BOSS_ELEMENTAL,
    BOSS_DEMON
};

// =============================================================================
// STRUCTURES (Memory-optimized: const char* instead of char[])
// =============================================================================

struct DialogueScreen {
    const char* speaker;
    const char* text;
    uint16_t speaker_color;
    bool has_choices;
    const char* choice1;
    const char* choice2;
    int choice1_next_index;
    int choice2_next_index;
};

struct StoryBoss {
    const char* name;
    const char* series;
    int hp;
    int attack;
    int defense;
    int xp_reward;
    int gem_reward;
    int exclusive_card_id;
    BossVisualType visual_type;
    uint16_t boss_color;
    uint16_t boss_accent;
};

struct StoryChapter {
    int chapter_number;
    const char* title;
    int level_required;
    int dialogue_count;
    DialogueScreen dialogues[MAX_DIALOGUE_SCREENS];
    StoryBoss boss;
    bool completed;
    bool boss_defeated;
    bool rewards_claimed;
};

struct CharacterStory {
    ThemeType character;
    const char* story_name;
    int current_chapter;
    int chapters_completed;
    StoryChapter chapters[MAX_CHAPTERS_PER_CHARACTER];
    bool story_completed;
    YugoStoryPath yugo_path;
};

struct StoryEvent {
    const char* title;
    const char* description;
    int xp_reward;
    int start_hour;
    int end_hour;
    bool completed_today;
    ThemeType character;
};

struct StorySystemState {
    CharacterStory* current_story;
    int current_dialogue_index;
    bool in_story_mode;
    bool in_boss_battle;
    bool showing_rewards;
    int player_hp;
    int player_max_hp;
    int boss_hp;
    int boss_max_hp;
    StoryEvent daily_events[4];
    int last_event_check_day;
    SDPreferences prefs;
    bool nvs_initialized;
    ChapterUnlockNotification notification;  // NEW: Chapter unlock notification
    int lastCheckedLevel;                     // NEW: Track level for unlock checks
};

// =============================================================================
// GLOBALS
// =============================================================================
extern StorySystemState story_system;
extern CharacterStory stories[THEME_COUNT];

// =============================================================================
// FUNCTIONS
// =============================================================================
void initStorySystem();
void initStoryDialogues();
void saveStoryProgress();
void loadStoryProgress();
void saveStoryProgressForCharacter(ThemeType character);
void loadStoryProgressForCharacter(ThemeType character);
void clearAllStoryProgress();

CharacterStory* getCharacterStory(ThemeType theme);
void setCurrentStory(ThemeType theme);
bool isChapterUnlocked(int chapter_number);
int getUnlockedChapterCount();

void startChapter(int chapter_number);
void advanceDialogue();
void handleDialogueChoice(int choice);
void completeCurrentChapter();

void startStoryBoss();
void handleStoryBossAttack();
void handleStoryBossSpecial();
bool isStoryBossDefeated();
void claimBossRewards();

void setYugoPath(YugoStoryPath path);
YugoStoryPath getYugoPath();
bool isYugoDecisionPoint();
void loadYugoPathStory(YugoStoryPath path);

void checkDailyStoryEvents();
StoryEvent* getCurrentActiveEvent();
void claimEventReward(int event_index);
bool hasActiveEvent();

void drawStoryMenu();
void drawChapterSelect();
void drawDialogueScreen();
void drawChoiceScreen(const char* choice1, const char* choice2);
void drawStoryBossScreen();
void drawBossSprite(int x, int y, StoryBoss* boss, int size, int anim_frame);
void drawChapterRewards();
void drawStoryEventPopup();
void drawYugoPathChoice();
void drawChapterUnlockNotification();  // NEW: Draw "New Chapter Unlocked!" popup

void handleStoryMenuTouch(TouchGesture& gesture);
void handleChapterSelectTouch(TouchGesture& gesture);
void handleDialogueTouch(TouchGesture& gesture);
void handleStoryBossTouch(TouchGesture& gesture);
void handleUnlockNotificationTouch(TouchGesture& gesture);  // NEW: Handle notification buttons

// NEW: Chapter unlock notification functions
void checkChapterUnlocks();             // Check if new chapters unlocked based on level
void triggerChapterUnlock(int chapterNum);  // Show unlock notification
void dismissNotification();             // User chose "Later"
void acceptNotification();              // User chose "Ready" - go to chapter
void updateNotificationTimeout();       // Auto-dismiss after 5 seconds

uint16_t getStoryThemeColor(ThemeType theme);
const char* getChapterStatusText(int chapter);
float getChapterProgress();

#endif
