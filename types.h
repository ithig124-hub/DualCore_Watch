/*
 * types.h - Central Type Definitions (FIXED)
 * All shared structs, enums, and constants for the ESP32 Watch
 * 
 * MUST BE INCLUDED BEFORE ALL OTHER PROJECT HEADERS
 */

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// =============================================================================
// CONSTANTS
// =============================================================================

// Boss Rush (7 tiers x 5 bosses = 35 total)
#define TOTAL_BOSSES          35
#define MAX_POTIONS           3
#define CRITICAL_CHANCE       15
#define CRITICAL_MULTIPLIER   2.0
#define SPECIAL_ATTACK_COST   30
#define POTION_HEAL_PERCENT   30

// Training
#define MAX_TRAINING_TIME     60000
#define TRAINING_XP_PER_GAME_MIN  10
#define TRAINING_XP_PER_GAME_MAX  50
#define TRAINING_PERFECT_BONUS    25

// Reflex timing thresholds (in milliseconds)
#define REFLEX_PERFECT_MS     150
#define REFLEX_GREAT_MS       250
#define REFLEX_GOOD_MS        400

// RPG
#define MAX_ACTIVE_QUESTS     5
#define MAX_QUEST_NAME_LEN    32
#define MAX_QUEST_DESC_LEN    64
#define MAX_SNAKE_LENGTH      100
#define MAX_BATTLE_LOG        5

// =============================================================================
// GAME ENUMERATIONS
// =============================================================================

// Game types
enum GameType {
  GAME_NONE = 0,
  GAME_BATTLE_ARENA,
  GAME_SHADOW_DUNGEON,
  GAME_PIRATE_ADVENTURE,
  GAME_WAKFU_QUEST,
  GAME_SNAKE,
  GAME_MINI_SNAKE,
  GAME_MEMORY,
  GAME_MEMORY_MATCH,
  GAME_TRAINING,
  GAME_TRAINING_REFLEX,
  GAME_TRAINING_TARGET,
  GAME_TRAINING_SPEED,
  GAME_TRAINING_MEMORY,
  GAME_BOSS_RUSH,
  GAME_GACHA,
  GAME_FUSION,
  GAME_CHARACTER
};

// Game state
enum GameState {
  GAME_STATE_MENU = 0,
  GAME_MENU,
  GAME_STATE_PLAYING,
  GAME_PLAYING,
  GAME_STATE_PAUSED,
  GAME_PAUSED,
  GAME_STATE_WIN,
  GAME_WIN,
  GAME_STATE_LOSE,
  GAME_OVER,
  GAME_STATE_RESULTS,
  GAME_BATTLE_SELECT,
  GAME_BATTLE_FIGHT
};

// Boss tiers
enum BossTier {
  BOSS_TIER_1 = 1,
  BOSS_TIER_2 = 2,
  BOSS_TIER_3 = 3,
  BOSS_TIER_4 = 4,
  BOSS_TIER_5 = 5,
  BOSS_TIER_6 = 6,
  BOSS_TIER_7 = 7
};

// Gacha rarity
enum GachaRarity {
  RARITY_COMMON = 0,
  RARITY_UNCOMMON,
  RARITY_RARE,
  RARITY_EPIC,
  RARITY_LEGENDARY,
  RARITY_MYTHIC
};

// Training types
enum TrainingType {
  TRAINING_REFLEX = 0,
  TRAINING_TARGET,
  TRAINING_SPEED_TAP,
  TRAINING_SPEED = TRAINING_SPEED_TAP,  // Alias for backward compatibility
  TRAINING_MEMORY
};

// RPG character types
enum RPGCharacterType {
  RPG_LUFFY = 0,
  RPG_JINWOO,
  RPG_YUGO,
  RPG_CUSTOM
};

// Yugo endgame paths
enum YugoEndgamePath {
  YUGO_PATH_NONE = 0,
  YUGO_UNDECIDED,
  YUGO_PATH_ELIATROPE_KING,
  YUGO_PATH_WORLD_GUARDIAN,
  YUGO_PATH_PORTAL_MASTER,
  YUGO_DRAGON_KING
};

// Power state - MOVED TO power_manager.h (has 5 states: ACTIVE, IDLE, DIMMED, MINIMAL, SCREEN_OFF)

// WiFi state
enum WiFiState {
  WIFI_STATE_DISCONNECTED = 0,
  WIFI_STATE_SCANNING,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_ERROR,
  WIFI_DISCONNECTED = 0,
  WIFI_SCANNING = 1,
  WIFI_CONNECTING = 2,
  WIFI_CONNECTED = 3,
  WIFI_FAILED = 4
};

// File types
enum FileType {
  FILE_UNKNOWN = 0,
  FILE_MP3,
  FILE_WAV,
  FILE_PDF,
  FILE_TXT,
  FILE_JPG,
  FILE_PNG,
  FILE_BMP,
  FILE_CONFIG
};

// Quest types
enum QuestType {
  QUEST_STEPS = 0,
  QUEST_ACTIVE_TIME,
  QUEST_GAMES,
  QUEST_TRAINING,
  QUEST_BOSS,
  QUEST_DISTANCE
};

// Quest character association
enum QuestCharacter {
  QUEST_ANY = 0,
  QUEST_LUFFY,
  QUEST_JINWOO,
  QUEST_YUGO
};

// Quest difficulty
enum QuestDifficulty {
  QUEST_EASY = 0,
  QUEST_MEDIUM,
  QUEST_HARD,
  QUEST_LEGENDARY
};

// =============================================================================
// GAME STRUCTURES
// =============================================================================

// Boss data
struct BossData {
  const char* name;
  const char* series;
  BossTier tier;
  int hp;
  int max_hp;
  int attack;
  int defense;
  int level;
  bool defeated;
  int gem_reward;    // FIXED: removed duplicate reward_gems field
};

// Gacha card
struct GachaCard {
  int id;
  String character_name;
  String series;
  String catchphrase;
  GachaRarity rarity;
  int power;
  int power_rating;
  uint16_t card_color;
  bool owned;
  int duplicates;
  int duplicate_count;
  int evolution_level;     // 0=base, 1=evolved, 2=awakened, 3=transcended
  int hp;
  int attack;
  int defense;
};

// Training score - FIXED: Added missing members
struct TrainingScore {
  TrainingType type;       // FIXED: Added training type
  int score;
  int hits;
  int misses;
  int streak;
  int best_streak;
  int best_time_ms;        // FIXED: Added best time tracking
  int combo_count;         // FIXED: Added combo counter
  unsigned long time_ms;
  bool perfect;
  int xp_earned;
};

// Character stats structure for RPG
struct CharacterStats {
  int strength;
  int speed;
  int intelligence;
  int endurance;
  int magic;
  int special_power;
  int total_power;
};

// RPG character data - comprehensive
struct RPGCharacterData {
  RPGCharacterType type;
  RPGCharacterType character_type;
  const char* name;
  const char* title;
  int level;
  long long xp;
  long long experience;
  long long xp_to_next;
  long long experience_to_next;
  int hp;
  int max_hp;
  int attack;
  int defense;
  int special;
  int stat_points;
  bool awakened;
  bool awakened_form;
  CharacterStats base_stats;
  CharacterStats current_stats;
  YugoEndgamePath yugo_path;
  int shadow_army_size;
  int quests_completed;
  int battles_won;
  int days_active;
  long total_steps;
};

// Quest data - use const char* for easy initialization
struct QuestData {
  const char* title;
  const char* description;
  QuestType type;
  QuestCharacter character;
  QuestDifficulty difficulty;
  int target_value;
  int current_progress;
  int reward_points;
  int reward_gems;
  unsigned long created_time;
  bool completed;
  bool claimed;
  bool urgent;
};

// =============================================================================
// HARDWARE STRUCTURES
// =============================================================================

// Battery info
struct BatteryInfo {
  int percentage;
  int voltage_mv;
  bool charging;
  bool plugged_in;
  bool low_battery;
  bool is_charging;
  bool is_plugged;
};

// IMU data
struct IMUData {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temperature;
  unsigned long timestamp;
};

// Step tracking data
struct StepData {
  int steps_today;
  int daily_steps;  // Alias for steps_today
  int step_goal;
  int steps_this_hour;
  unsigned long last_step_time;
  float calories_burned;
  float distance_km;
};

// =============================================================================
// INTERNET APP STRUCTURES
// =============================================================================

// Weather data
struct WeatherData {
  String location;
  String description;
  String icon;
  String last_update;
  float temperature;
  float humidity;
  float wind_speed;
  int pressure;
  bool valid;
};

// News article - FIXED: Added missing summary member
struct NewsArticle {
  String title;
  String summary;          // FIXED: Added summary field
  String source;
  String url;
  String time_ago;         // FIXED: Added time_ago field for display
  unsigned long timestamp;
};

// =============================================================================
// FILE SYSTEM STRUCTURES
// =============================================================================

// File info
struct FileInfo {
  char name[64];
  char path[128];
  String filename;
  String filepath;
  size_t size;
  bool is_directory;
  FileType type;
};

// Music file
struct MusicFile {
  String filename;
  String filepath;
  String title;
  String artist;
  int duration_seconds;
  size_t file_size;
};

// Wallpaper file
struct WallpaperFile {
  String filename;
  String filepath;
  int width;
  int height;
  int theme_index;
  int theme;
  size_t file_size;
  bool is_valid;
};

// =============================================================================
// BATTLE SYSTEM STRUCTURES
// =============================================================================

// Battle creature - comprehensive for games.cpp
struct BattleCreature {
  const char* name;
  const char* type;
  int level;
  int hp;
  int max_hp;
  int attack;
  int defense;
  int speed;
  const char* moves[4];
  int move_power[4];
  bool is_player;
  uint16_t color;
};

// Game session - comprehensive for games.cpp
struct GameSession {
  GameType game_type;
  GameType current_game;
  GameState state;
  int score;
  int high_score;
  int level;
  unsigned long start_time;
  unsigned long elapsed_time;
  bool active;
  
  // Battle fields
  BattleCreature player_creature;
  BattleCreature enemy_creature;
  int selected_move;
  String battle_log[MAX_BATTLE_LOG];
  int log_count;
  
  // Snake game fields
  int snake_x[MAX_SNAKE_LENGTH];
  int snake_y[MAX_SNAKE_LENGTH];
  int snake_length;
  int food_x;
  int food_y;
  int direction;
  int snake_speed_level;
  int snake_extra_lives;
  int lives;
  bool sound_enabled;
  
  // Dungeon/Adventure fields
  int player_x;
  int player_y;
  int map_level;
  int items_collected;
};

#endif // TYPES_H
