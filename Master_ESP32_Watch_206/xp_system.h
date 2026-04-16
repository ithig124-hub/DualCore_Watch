/*
 * xp_system.h - XP & Leveling System Header
 * FUSION OS RPG Integration
 *
 * UPDATED: Compact NVS Save, Rebirth System, SD Card Backup
 * Fixes NVS fragmentation by using single packed struct (~85 bytes)
 */

#ifndef XP_SYSTEM_H
#define XP_SYSTEM_H

#include "config.h"    // ThemeType is defined in config.h

// =============================================================================
// XP CONSTANTS
// =============================================================================

// Base XP values
#define BASE_XP_PER_LEVEL 100
#define XP_SCALING_FACTOR 1.15f

// Daily/Hourly rewards
#define XP_DAILY_LOGIN 50
#define XP_HOURLY_SHOP 10
#define XP_DAILY_GOAL_BONUS 150

// Gacha XP rewards
#define XP_GACHA_PULL 5
#define XP_GACHA_LEGENDARY 20

// =============================================================================
// XP REWARDS - Card Selling
// =============================================================================
#define XP_SELL_MYTHIC      10000
#define XP_SELL_LEGENDARY   5000
#define XP_SELL_EPIC        1000
#define XP_SELL_OTHER       500

// =============================================================================
// XP REWARDS - Battles
// =============================================================================
#define XP_BATTLE_WIN       100
#define XP_BATTLE_LOSE      25
#define XP_BOSS_DEFEAT      500
#define XP_BOSS_RUSH_CLEAR  1000

// Boss tier XP rewards
#define XP_BOSS_TIER_1      100
#define XP_BOSS_TIER_2      250
#define XP_BOSS_TIER_3      500
#define XP_BOSS_TIER_4      750
#define XP_BOSS_TIER_5      1500
#define XP_BOSS_TIER_6      3000
#define XP_BOSS_TIER_7      5000

// =============================================================================
// XP REWARDS - Daily Quests
// =============================================================================
#define XP_QUEST_EASY       50
#define XP_QUEST_MEDIUM     100
#define XP_QUEST_HARD       200
#define XP_QUEST_ALL_DAILY  500

// =============================================================================
// XP REWARDS - Card Evolution
// =============================================================================
#define XP_EVOLVE_TO_LV1    100
#define XP_EVOLVE_TO_LV2    250
#define XP_EVOLVE_TO_LV3    500

// =============================================================================
// XP REWARDS - Miscellaneous
// =============================================================================
#define XP_STEP_GOAL        100
#define XP_NEW_CARD         50
#define XP_COLLECTION_10    200
#define XP_DECK_COMPLETE    300

// =============================================================================
// GEM REWARDS - Card Selling
// =============================================================================
#define GEMS_SELL_MYTHIC    500
#define GEMS_SELL_LEGENDARY 200
#define GEMS_SELL_EPIC      100
#define GEMS_SELL_OTHER     50

// =============================================================================
// LEVELING SYSTEM
// =============================================================================

#define MAX_TITLES_PER_CHARACTER 22
#define MAX_TITLE_LEVEL 100

// =============================================================================
// BACKUP CONSTANTS
// =============================================================================
#define BACKUP_FOLDER "/backups"
#define BACKUP_PREFIX "backup_"
#define BACKUP_EXT ".dat"
#define MAX_BACKUPS 10
#define BACKUP_MAGIC 0x57415443  // "WATC"

// =============================================================================
// REBIRTH STATE MACHINE
// =============================================================================
enum RebirthState {
    REBIRTH_IDLE,        // Normal settings view
    REBIRTH_CONFIRM_1,   // "Are you sure?" screen
    REBIRTH_CONFIRM_2,   // "FINAL WARNING" screen
    REBIRTH_COMPLETE     // Executing reset
};

// =============================================================================
// COMPACT NVS SAVE STRUCTURE (85 bytes total)
// =============================================================================
#pragma pack(push, 1)
struct CompactXPSave {
    uint8_t  version;           // 1 byte  - Format version
    uint16_t levels[11];        // 22 bytes - All character levels
    uint32_t xp[11];            // 44 bytes - All character XP
    int8_t   titles[11];        // 11 bytes - Equipped title index per char
    uint16_t gems;              // 2 bytes  - Total gems
    uint8_t  streak;            // 1 byte   - Current login streak
    uint8_t  last_day;          // 1 byte   - Last login day
    uint8_t  streak_day;        // 1 byte   - Streak start day
    uint8_t  streak_month;      // 1 byte   - Streak start month
    uint8_t  longest_streak;    // 1 byte   - Best streak ever
};  // Total: 85 bytes
#pragma pack(pop)

// =============================================================================
// SD CARD BACKUP STRUCTURE (~114 bytes)
// =============================================================================
#pragma pack(push, 1)
struct SDBackupData {
    uint32_t magic;              // "WATC" - file validation
    uint8_t  version;            // Format version (1)
    uint32_t timestamp;          // When backup was created
    CompactXPSave xp_data;       // All XP data (85 bytes)
    uint32_t total_steps;        // Lifetime steps
    uint32_t bosses_defeated;    // Boss kill count
    uint32_t cards_collected;    // Gacha cards
    uint16_t rebirth_count;      // Rebirths at backup time
    uint16_t total_levels;       // Sum of all levels (for preview)
    uint32_t checksum;           // Data integrity validation
};
#pragma pack(pop)

// =============================================================================
// BACKUP INFO (for preview display)
// =============================================================================
struct BackupInfo {
    bool exists;
    int slot;
    uint16_t total_levels;
    uint16_t gems;
    uint16_t rebirth_count;
    uint32_t timestamp;
};

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct CharacterTitle {
    const char* name;
    int level_required;
    bool unlocked;
};

struct CharacterXPData {
    ThemeType theme;
    int level;
    long xp;
    long xp_to_next_level;
    int equipped_title_index;
    CharacterTitle titles[MAX_TITLES_PER_CHARACTER];
};

struct XPSystemState {
    CharacterXPData* current_character;
    int total_gems;
    int last_login_day;
    int last_hourly_claim_hour;
    bool daily_step_goal_claimed;
    int login_streak;
    int last_streak_day;
    int last_streak_month;
    int longest_streak;
};

extern XPSystemState xp_system;

// =============================================================================
// REBIRTH & BACKUP STATE (extern)
// =============================================================================
extern RebirthState rebirth_state;
extern unsigned long rebirth_timeout;
extern int selected_backup_slot;
extern bool in_backup_list_view;

// =============================================================================
// FUNCTION DECLARATIONS - Core XP System
// =============================================================================
void initXPSystem();
void saveXPData();
void loadXPData();

// Character management
CharacterXPData* getCurrentCharacterXP();
void switchCharacter(ThemeType theme);

// XP operations
void gainExperience(int amount, const char* source);
void handleLevelUp(int new_level);
long calculateXPForLevel(int level);
float getXPProgress();

// Title system
const char* getEquippedTitle();
void equipTitle(int title_index);

// Daily/hourly rewards
void checkDailyLoginBonus();
void checkHourlyShopClaim();

// =============================================================================
// FUNCTION DECLARATIONS - Rebirth System
// =============================================================================
void showRebirthButton();
void showRebirthConfirm1();
void showRebirthConfirm2();
void executeRebirth();
void handleRebirthTouch(int x, int y);
void cancelRebirth();

// =============================================================================
// FUNCTION DECLARATIONS - Rebirth Counter
// =============================================================================
int getRebirthCount();
void incrementRebirthCount();

// =============================================================================
// FUNCTION DECLARATIONS - SD Card Backup
// =============================================================================
bool saveStatsToSD(int slot);
bool loadStatsFromSD(int slot);
int getNextBackupSlot();
void getBackupInfo(int slot, BackupInfo* info);
void showSDBackupMenu();
void showBackupListMenu();
void handleSDBackupTouch(int x, int y);
void handleBackupListTouch(int x, int y);

#endif // XP_SYSTEM_H
