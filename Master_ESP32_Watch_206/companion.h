/*
 * companion.h - Virtual Pet System Header
 * FUSION OS - 11 unique companions with evolution sprites
 * 
 * FIXED: Watchdog-safe, companion matches current theme
 */

#ifndef COMPANION_H
#define COMPANION_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "types.h"

// ============================================================================
// CONSTANTS
// ============================================================================

#define COMPANION_COUNT THEME_COUNT

#define STAT_MAX 100
#define STAT_MIN 0
#define STAT_CRITICAL 20
#define STAT_LOW 40

#define FEED_COST_GEMS 5
#define FEED_HUNGER_GAIN 30
#define FEED_HAPPINESS_GAIN 10
#define PLAY_ENERGY_COST 20
#define PLAY_HAPPINESS_GAIN 25
#define PLAY_BOND_GAIN 5
#define TRAIN_ENERGY_COST 30
#define TRAIN_XP_GAIN 15
#define TRAIN_BOND_GAIN 3
#define REST_ENERGY_GAIN 50
#define REST_DURATION_MS 30000

#define BOND_MAX 100
#define BOND_RANK_1 20
#define BOND_RANK_2 40
#define BOND_RANK_3 60
#define BOND_RANK_4 80
#define BOND_RANK_5 100

#define HUNGER_DECAY_RATE 5
#define HAPPINESS_DECAY_RATE 3
#define ENERGY_DECAY_RATE 2

#define COMPANION_NVS_NAMESPACE "companion_data"

// ============================================================================
// ENUMS
// ============================================================================

enum CompanionMood {
    MOOD_ECSTATIC = 0,
    MOOD_HAPPY,
    MOOD_CONTENT,
    MOOD_SAD,
    MOOD_MISERABLE
};

enum CompanionEvolution {
    EVO_BABY = 0,
    EVO_CHILD,
    EVO_ADULT,
    EVO_AWAKENED
};

enum CompanionType {
    COMP_SUNNY = 0,      // Luffy theme
    COMP_IGRIS,          // Jinwoo theme
    COMP_AZ,             // Yugo theme
    COMP_KURAMA,         // Naruto theme
    COMP_PUAR,           // Goku theme
    COMP_NEZUKO,         // Tanjiro theme
    COMP_SPIRIT,         // Gojo theme
    COMP_BLADES,         // Levi theme
    COMP_GENOS,          // Saitama theme
    COMP_ALLMIGHT,       // Deku theme
    COMP_OCHOBOT         // BoBoiBoy theme
};

// ============================================================================
// STRUCTS
// ============================================================================

struct CompanionStats {
    int hunger;
    int happiness;
    int energy;
    int bond_level;
    int bond_rank;
    CompanionMood mood;
    CompanionEvolution evolution;
    int total_interactions;
    int days_together;
};

struct CompanionCare {
    unsigned long last_feed_time;
    unsigned long last_play_time;
    unsigned long last_train_time;
    unsigned long last_stat_update;
    int daily_feed_count;
    int daily_play_count;
    int daily_train_count;
    bool is_sleeping;
    unsigned long sleep_start_time;
};

struct CompanionProfile {
    const char* name;
    const char* series;
    uint16_t primary_color;
    uint16_t secondary_color;
    const char* favorite_food;
    const char* favorite_game;
    const char* catchphrase;
};

struct CompanionData {
    CompanionType type;
    CompanionProfile* profile;
    CompanionStats stats;
    CompanionCare care;
};

struct CurrentGame {
    const char* name;
    int score;
    int high_score;
    bool active;
    unsigned long start_time;
    int target_x;
    int target_y;
};

struct CompanionSystemState {
    CompanionData companions[COMPANION_COUNT];
    CompanionData* current_companion;
    int current_companion_index;
    bool in_care_mode;
    bool in_mini_game;
    int care_menu_selection;
    CurrentGame current_game;
    Preferences prefs;
    bool nvs_initialized;
    int animation_frame;
    unsigned long last_animation_time;
};

// ============================================================================
// EXTERN DECLARATIONS
// ============================================================================

extern CompanionSystemState companion_system;
extern CompanionProfile COMPANION_PROFILES[COMPANION_COUNT];

// ============================================================================
// INITIALIZATION & NVS (Watchdog-safe, loads only active companion)
// ============================================================================

void initCompanionSystem();                    // Initializes system, loads companion for current theme
void saveCompanionData();                      // Saves current companion to NVS
void loadCompanionData();                      // Loads current companion from NVS
void saveCompanionDataForIndex(int index);     // Saves specific companion to NVS
void loadCompanionDataForIndex(int index);     // Loads specific companion from NVS
void clearAllCompanionData();                  // Clears all NVS data

// ============================================================================
// COMPANION SELECTION (Companion always matches current theme)
// ============================================================================

void syncCompanionWithTheme();                 // Sync companion to current theme (call after theme change)

// ============================================================================
// STATS MANAGEMENT
// ============================================================================

void updateCompanionStats();
void updateCompanionMood();
void updateCompanionEvolution();
void checkCompanionDailyReset();

// ============================================================================
// CARE ACTIONS
// ============================================================================

bool feedCompanion();
bool playWithCompanion();
bool trainCompanion();
void toggleCompanionSleep();
void wakeCompanion();

// ============================================================================
// MINI GAME
// ============================================================================

void startCompanionGame();
void updateCompanionGame();
void endCompanionGame();
bool handleGameTouch(int tx, int ty);

// ============================================================================
// ANIMATION
// ============================================================================

void updateCompanionAnimation();

// ============================================================================
// SPRITE DRAWING
// ============================================================================

void drawCompanionSprite(int x, int y, CompanionType type, CompanionEvolution evo);
void drawSunnySprite(int x, int y, int size, CompanionEvolution evo);
void drawShadowSprite(int x, int y, int size, CompanionEvolution evo);
void drawPortalSprite(int x, int y, int size, CompanionEvolution evo);
void drawFoxSprite(int x, int y, int size, CompanionEvolution evo);
void drawCatSprite(int x, int y, int size, CompanionEvolution evo);
void drawDemonSprite(int x, int y, int size, CompanionEvolution evo);
void drawInfinitySprite(int x, int y, int size, CompanionEvolution evo);
void drawBladesSprite(int x, int y, int size, CompanionEvolution evo);
void drawHeroSprite(int x, int y, int size, CompanionEvolution evo);
void drawMightSprite(int x, int y, int size, CompanionEvolution evo);
void drawElementSprite(int x, int y, int size, CompanionEvolution evo);

// ============================================================================
// CARE MODE UI
// ============================================================================

void enterCompanionCareMode();
void exitCompanionCareMode();
void drawCompanionCareScreen();
void handleCareModeTouch(int tx, int ty);

// ============================================================================
// GETTERS
// ============================================================================

CompanionData* getCurrentCompanion();
bool isCompanionCareMode();
bool isCompanionGameActive();

#endif // COMPANION_H
