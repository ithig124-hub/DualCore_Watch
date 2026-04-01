/*
 * config.h - IMPROVED CONFIGURATION (FIXED)
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 * Display: CO5300 QSPI AMOLED (410x502)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "types.h"  // Central type definitions - MUST be included first

// =============================================================================
// DISPLAY CONFIGURATION - ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
// =============================================================================
#define LCD_WIDTH           410
#define LCD_HEIGHT          502

// Display pins (QSPI) - FROM WAVESHARE OFFICIAL pin_config.h
#define LCD_SDIO0           4
#define LCD_SDIO1           5
#define LCD_SDIO2           6
#define LCD_SDIO3           7
#define LCD_SCLK            11
#define LCD_CS              12
#define LCD_RESET           8       // Display Reset (for 2.06" board)

// Display column offset (required for CO5300)
#define LCD_COL_OFFSET      22

// =============================================================================
// I2C CONFIGURATION - FROM WAVESHARE OFFICIAL pin_config.h
// =============================================================================
#define IIC_SDA             15
#define IIC_SCL             14

// I2C Addresses
#define FT3168_ADDR         0x38
#define AXP2101_ADDR        0x34
#define PCF85063_ADDR       0x51
#define RTC_ADDR            0x51
#define QMI8658_ADDR        0x6B
#define ES8311_ADDR         0x18
#define EXPANDER_ADDR       0x20

// =============================================================================
// TOUCH CONFIGURATION - IMPROVED SENSITIVITY (2.06" board)
// =============================================================================
#define TP_INT              38      // Touch Interrupt (changed from 21 for 2.06" board)
#define TP_RST              9       // Touch Reset (for 2.06" board)

// IMPROVED Touch thresholds - better responsiveness
#define SWIPE_THRESHOLD_MIN       30    // Reduced for easier swipes
#define SWIPE_THRESHOLD_MAX       250
#define SWIPE_MAX_DURATION_MS     600   // Increased for easier swipes
#define TAP_MAX_DURATION_MS       250   // Tap timeout
#define TAP_MAX_MOVEMENT          20    // Max movement for tap

// Navigation cooldown - prevents accidental double navigation
#define NAVIGATION_COOLDOWN_MS    200   // Reduced for snappier feel

// =============================================================================
// AUDIO CONFIGURATION
// =============================================================================
#define PA_PIN              46

// =============================================================================
// BATTERY CONFIGURATION
// =============================================================================
#define BATTERY_LOW_THRESHOLD     20
#define BATTERY_CRITICAL          10

// =============================================================================
// STEP DETECTION CONFIGURATION
// =============================================================================
#define MIN_STEP_THRESHOLD        1.2    // Minimum acceleration delta for step
#define STEP_TIME_WINDOW          250    // Minimum ms between steps

// =============================================================================
// NAVIGATION CONFIGURATION - FIXED
// =============================================================================
#define APP_GRID_PAGES            3     // Three pages of apps (0, 1, 2)
// NOTE: MAIN_SCREEN_COUNT is defined later with the MainScreen enum (= 4)

// =============================================================================
// COLOR DEFINITIONS - RGB565 FORMAT
// =============================================================================
#define RGB565(r, g, b)  ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

// Basic Colors
#define COLOR_BLACK       0x0000
#define COLOR_WHITE       0xFFFF
#define COLOR_RED         RGB565(255, 0, 0)
#define COLOR_GREEN       RGB565(0, 255, 0)
#define COLOR_BLUE        RGB565(0, 0, 255)
#define COLOR_YELLOW      RGB565(255, 255, 0)
#define COLOR_ORANGE      RGB565(255, 165, 0)
#define COLOR_PURPLE      RGB565(128, 0, 128)
#define COLOR_CYAN        RGB565(0, 255, 255)
#define COLOR_PINK        RGB565(255, 105, 180)
#define COLOR_GRAY        RGB565(128, 128, 128)
#define COLOR_GOLD        RGB565(255, 215, 0)

// =============================================================================
// LUFFY GEAR 5 - SUN GOD NIKA (Modern Anime Style)
// =============================================================================
#define LUFFY_SUN_GOLD          RGB565(255, 215, 50)   // Bright golden sun
#define LUFFY_NIKA_WHITE        RGB565(255, 255, 250)  // Divine white
#define LUFFY_ENERGY_ORANGE     RGB565(255, 140, 40)   // Energy burst
#define LUFFY_DEEP_BLACK        RGB565(5, 5, 10)       // AMOLED black
#define LUFFY_CLOUD_WHITE       RGB565(240, 245, 255)  // Cloud wisps
#define LUFFY_SUN_GLOW          RGB565(255, 200, 100)  // Sun aura
#define LUFFY_JOY_YELLOW        RGB565(255, 235, 80)   // Joy/freedom
#define LUFFY_FREEDOM_SKY       RGB565(135, 206, 250)  // Sky blue
#define LUFFY_RUBBER_PINK       RGB565(255, 180, 180)  // Rubber stretch
#define LUFFY_STRAW_RED         RGB565(200, 30, 30)    // Straw hat

// =============================================================================
// SUNG JIN-WOO - SHADOW MONARCH (Dark Purple Power)
// =============================================================================
#define JINWOO_MONARCH_PURPLE   RGB565(120, 50, 200)   // Monarch power
#define JINWOO_VOID_BLACK       RGB565(10, 5, 20)      // Absolute darkness
#define JINWOO_POWER_VIOLET     RGB565(150, 80, 220)   // Power surge
#define JINWOO_ABSOLUTE_DARK    RGB565(5, 0, 15)       // Deep void
#define JINWOO_DEATH_BLACK      RGB565(15, 10, 25)     // Death essence
#define JINWOO_ARISE_GLOW       RGB565(180, 120, 255)  // ARISE! glow
#define JINWOO_DAGGER_BLUE      RGB565(100, 150, 255)  // Kamish daggers
#define JINWOO_MIST_PURPLE      RGB565(80, 40, 120)    // Shadow mist
#define JINWOO_SOUL_PURPLE      RGB565(140, 90, 200)   // Soul energy
#define JINWOO_PURPLE           RGB565(130, 60, 210)   // Main purple

// =============================================================================
// YUGO - PORTAL MASTER (Cyan Portal Energy)
// =============================================================================
#define YUGO_PORTAL_CYAN        RGB565(0, 220, 255)    // Portal color
#define YUGO_ELIATROPE_TEAL     RGB565(0, 180, 200)    // Eliatrope energy
#define YUGO_HAT_GOLD           RGB565(255, 200, 80)   // Hat accent
#define YUGO_SKY_BLUE_GREY      RGB565(60, 80, 100)    // Background
#define YUGO_MYSTERY_BLUE       RGB565(40, 60, 120)    // Mystery
#define YUGO_PORTAL_GLOW        RGB565(100, 255, 255)  // Portal glow
#define YUGO_WAKFU_ENERGY       RGB565(80, 200, 220)   // Wakfu power
#define YUGO_PORTAL_RING        RGB565(50, 180, 200)   // Ring effect
#define YUGO_ENERGY_BURST       RGB565(150, 255, 255)  // Energy burst
#define YUGO_TEAL               RGB565(0, 200, 180)    // Main teal

// =============================================================================
// NARUTO - SAGE MODE (Orange Chakra)
// =============================================================================
#define NARUTO_CHAKRA_ORANGE    RGB565(255, 140, 0)    // Chakra color
#define NARUTO_OUTFIT_BLACK     RGB565(30, 30, 35)     // Outfit
#define NARUTO_SAGE_GOLD        RGB565(255, 200, 50)   // Sage mode
#define NARUTO_SLATE_GREY       RGB565(50, 55, 65)     // Background
#define NARUTO_SHADOW_ORANGE    RGB565(200, 100, 20)   // Shadow
#define NARUTO_KURAMA_FLAME     RGB565(255, 100, 30)   // Kurama fire
#define NARUTO_RASENGAN_BLUE    RGB565(100, 180, 255)  // Rasengan
#define NARUTO_SAGE_EYES        RGB565(255, 180, 100)  // Sage eyes
#define NARUTO_WILL_FIRE        RGB565(255, 80, 20)    // Will of fire
#define NARUTO_KURAMA_RED       RGB565(255, 60, 30)    // Nine-tails

// =============================================================================
// GOKU - ULTRA INSTINCT (Divine Silver)
// =============================================================================
#define GOKU_UI_SILVER          RGB565(200, 210, 225)  // UI silver
#define GOKU_GI_BLUE            RGB565(40, 60, 120)    // Gi color
#define GOKU_AURA_WHITE         RGB565(230, 235, 255)  // UI aura
#define GOKU_VOID_BLACK         RGB565(5, 5, 15)       // Space
#define GOKU_SILVER_GLOW        RGB565(180, 190, 210)  // Silver glow
#define GOKU_DIVINE_SILVER      RGB565(210, 220, 240)  // Divine
#define GOKU_KI_BLAST_BLUE      RGB565(80, 150, 255)   // Ki blast
#define GOKU_SPEED_LINES        RGB565(150, 160, 180)  // Speed
#define GOKU_LIGHT_SILVER       RGB565(220, 225, 240)  // Light

// =============================================================================
// TANJIRO - SUN BREATHING (Fire Orange/Red)
// =============================================================================
#define TANJIRO_FIRE_ORANGE     RGB565(255, 120, 30)   // Fire color
#define TANJIRO_CHECK_GREEN     RGB565(60, 120, 80)    // Haori pattern
#define TANJIRO_HANAFUDA_RED    RGB565(200, 50, 50)    // Hanafuda
#define TANJIRO_DARK_CHARCOAL   RGB565(35, 30, 30)     // Background
#define TANJIRO_SCAR_BURGUNDY   RGB565(150, 40, 60)    // Scar color
#define TANJIRO_FLAME_GLOW      RGB565(255, 150, 50)   // Flame glow
#define TANJIRO_WATER_BLUE      RGB565(80, 160, 220)   // Water breathing
#define TANJIRO_SUN_FIRE        RGB565(255, 100, 0)    // Sun breathing
#define TANJIRO_SPIRIT_WHITE    RGB565(240, 240, 255)  // Spirit
#define TANJIRO_STEEL_GREY      RGB565(160, 165, 175)  // Nichirin

// =============================================================================
// GOJO - INFINITY (Intense Blue)
// =============================================================================
#define GOJO_INFINITY_BLUE      RGB565(60, 120, 255)   // Infinity
#define GOJO_SNOW_WHITE         RGB565(250, 250, 255)  // Hair
#define GOJO_HOLLOW_PURPLE      RGB565(180, 80, 200)   // Hollow purple
#define GOJO_VOID_BLACK         RGB565(5, 5, 20)       // Domain void
#define GOJO_DOMAIN_PURPLE      RGB565(100, 40, 150)   // Domain
#define GOJO_LIGHT_BLUE_GLOW    RGB565(150, 200, 255)  // Light glow
#define GOJO_SIX_EYES_BLUE      RGB565(100, 180, 255)  // Six Eyes
#define GOJO_DEEP_INFINITY      RGB565(40, 80, 180)    // Deep infinity
#define GOJO_CURSED_PURPLE      RGB565(150, 60, 180)   // Cursed energy

// =============================================================================
// LEVI - HUMANITY'S STRONGEST (Military Green/Grey)
// =============================================================================
#define LEVI_SURVEY_GREEN       RGB565(80, 140, 80)    // Survey Corps
#define LEVI_DARK_UNIFORM       RGB565(40, 45, 50)     // Uniform
#define LEVI_SILVER_BLADE       RGB565(200, 205, 215)  // Blades
#define LEVI_CHARCOAL_DARK      RGB565(30, 32, 38)     // Background
#define LEVI_MILITARY_GREY      RGB565(100, 105, 115)  // Military
#define LEVI_STEEL_METAL        RGB565(180, 185, 195)  // Steel
#define LEVI_CAPE_GREEN         RGB565(60, 100, 60)    // Cape
#define LEVI_SPEED_BLUR         RGB565(150, 155, 165)  // Speed blur
#define LEVI_CLEAN_WHITE        RGB565(245, 248, 255)  // Clean aesthetic

// =============================================================================
// SAITAMA - ONE PUNCH (Bold Yellow/Red)
// =============================================================================
#define SAITAMA_HERO_YELLOW     RGB565(255, 230, 50)   // Hero suit
#define SAITAMA_CAPE_RED        RGB565(220, 40, 40)    // Cape
#define SAITAMA_GOLDEN_PUNCH    RGB565(255, 200, 0)    // Punch glow
#define SAITAMA_SIMPLE_BLACK    RGB565(10, 10, 12)     // Minimalist bg
#define SAITAMA_DEEP_RED_CAPE   RGB565(180, 30, 30)    // Cape shadow
#define SAITAMA_IMPACT_ORANGE   RGB565(255, 150, 50)   // Impact
#define SAITAMA_GLOVE_RED       RGB565(200, 50, 50)    // Gloves
#define SAITAMA_BOOT_YELLOW     RGB565(220, 200, 50)   // Boots
#define SAITAMA_BALD_WHITE      RGB565(255, 250, 245)  // Bald shine
#define SAITAMA_DOT_EYES        RGB565(0, 0, 0)        // Simple eyes

// =============================================================================
// DEKU - PLUS ULTRA (Green Lightning)
// =============================================================================
#define DEKU_HERO_GREEN         RGB565(50, 180, 80)    // Hero suit
#define DEKU_COSTUME_BLACK      RGB565(25, 30, 35)     // Costume
#define DEKU_OFA_LIGHTNING      RGB565(100, 255, 150)  // OFA lightning
#define DEKU_DARK_HERO          RGB565(20, 25, 30)     // Background
#define DEKU_BLACKWHIP_DARK     RGB565(30, 30, 40)     // Blackwhip
#define DEKU_FULL_COWL          RGB565(80, 220, 120)   // Full Cowl
#define DEKU_AIR_FORCE          RGB565(150, 200, 255)  // Air Force
#define DEKU_POWER_VEINS        RGB565(255, 100, 100)  // Strain
#define DEKU_ALLMIGHT_GOLD      RGB565(255, 200, 50)   // All Might

// =============================================================================
// BOBOIBOY - ELEMENTAL HERO (Malaysian Anime)
// 7 Elements with Tier Progression & Fusion
// =============================================================================

// Base Element Colors (Galaxy Series)
#define BBB_LIGHTNING_YELLOW     RGB565(255, 220, 0)     // Halilintar - Electric
#define BBB_LIGHTNING_BLACK      RGB565(30, 30, 35)      // Dark accents
#define BBB_WIND_BLUE            RGB565(0, 150, 255)     // Taufan - Sky Blue
#define BBB_WIND_CYAN            RGB565(100, 220, 255)   // Light cyan
#define BBB_EARTH_BROWN          RGB565(180, 120, 60)    // Gempa - Earth
#define BBB_EARTH_ORANGE         RGB565(255, 140, 50)    // Orange glow
#define BBB_FIRE_RED             RGB565(220, 30, 50)     // Blaze - Crimson
#define BBB_FIRE_ORANGE          RGB565(255, 100, 30)    // Fire glow
#define BBB_WATER_CYAN           RGB565(0, 255, 255)     // Ice - Pure Cyan
#define BBB_WATER_WHITE          RGB565(230, 250, 255)   // Ice white
#define BBB_LEAF_GREEN           RGB565(34, 180, 50)     // Thorn - Forest
#define BBB_LEAF_DARK            RGB565(20, 100, 30)     // Dark green
#define BBB_LIGHT_GOLD           RGB565(255, 200, 50)    // Solar - Golden
#define BBB_LIGHT_WHITE          RGB565(255, 255, 240)   // Light white

// Second Tier / Evolution Colors
#define BBB_THUNDERSTORM_BLACK   RGB565(25, 25, 30)      // Dark + red eyes
#define BBB_THUNDERSTORM_RED     RGB565(255, 50, 50)     // Red lightning
#define BBB_CYCLONE_DARK_BLUE    RGB565(0, 50, 150)      // Dark blue
#define BBB_QUAKE_DARK           RGB565(100, 60, 30)     // Dark brown
#define BBB_BLAZE_CRIMSON        RGB565(180, 20, 20)     // Deep red
#define BBB_ICE_LIGHT            RGB565(200, 240, 255)   // Light ice
#define BBB_THORN_DARK           RGB565(0, 80, 20)       // Dark forest
#define BBB_SOLAR_ORANGE         RGB565(255, 180, 50)    // Bright orange

// Fusion Colors
#define BBB_FROSTFIRE_PINK       RGB565(255, 100, 180)   // Blaze + Ice
#define BBB_GLACIER_BLUE         RGB565(100, 180, 220)   // Quake + Ice
#define BBB_SUPRA_GOLD           RGB565(255, 200, 100)   // Thunder + Solar

// Power Band & UI
#define BBB_BAND_ORANGE          RGB565(255, 140, 30)    // Power Band base
#define BBB_BAND_GLOW            RGB565(255, 200, 100)   // Band glow
#define BBB_OCHOBOT_WHITE        RGB565(240, 245, 255)   // Ochobot helper

// =============================================================================
// GAME ECONOMY SETTINGS
// =============================================================================
#define GEMS_DAILY_LOGIN        50
#define GEMS_QUEST_COMPLETE     25
#define GEMS_GAME_WIN_MIN       10
#define GEMS_GAME_WIN_MAX       50
#define GEMS_BOSS_DEFEAT        100

#define GACHA_SINGLE_PULL_COST  100
#define GACHA_TEN_PULL_COST     900
#define GACHA_TOTAL_CARDS       100

// Pity system thresholds
#define PITY_EPIC_GUARANTEE     30    // Guaranteed Epic at 30 pulls
#define PITY_LEGENDARY_GUARANTEE 90   // Guaranteed Legendary at 90 pulls

// Card evolution costs (duplicates needed)
#define EVOLVE_COST_LV1         3     // Base -> Evolved
#define EVOLVE_COST_LV2         5     // Evolved -> Awakened
#define EVOLVE_COST_LV3         10    // Awakened -> Transcended
#define EVOLVE_POWER_MULT_LV1   1.3f  // 30% power boost
#define EVOLVE_POWER_MULT_LV2   1.6f  // 60% power boost
#define EVOLVE_POWER_MULT_LV3   2.0f  // 100% power boost

// Battle deck
#define MAX_DECK_SIZE           5
#define DECK_ATK_BONUS_PER_CARD 5
#define DECK_HP_BONUS_PER_CARD  10
#define DECK_DEF_BONUS_PER_CARD 3

// =============================================================================
// ENUMERATIONS
// =============================================================================

// Theme Types - All 11 anime characters (including BoBoiBoy)
enum ThemeType {
  THEME_LUFFY_GEAR5 = 0,
  THEME_SUNG_JINWOO,
  THEME_YUGO_WAKFU,
  THEME_NARUTO_SAGE,
  THEME_GOKU_UI,
  THEME_TANJIRO_SUN,
  THEME_GOJO_INFINITY,
  THEME_LEVI_STRONGEST,
  THEME_SAITAMA_OPM,
  THEME_DEKU_PLUSULTRA,
  THEME_BOBOIBOY,         // NEW: BoBoiBoy Elemental
  THEME_CUSTOM,
  THEME_COUNT = 11
};

// BoBoiBoy Element Types (for element switching)
enum BoBoiBoyElement {
  BBB_LIGHTNING = 0,   // Halilintar - Yellow/Black
  BBB_WIND = 1,        // Taufan/Cyclone - Blue
  BBB_EARTH = 2,       // Gempa/Quake - Brown/Orange
  BBB_FIRE = 3,        // Blaze - Red/Crimson
  BBB_WATER = 4,       // Ice - Cyan
  BBB_LEAF = 5,        // Thorn - Green
  BBB_LIGHT = 6,       // Solar - Gold/Orange
  BBB_ELEMENT_COUNT = 7
};

// Screen Types - FIXED: Added missing screen types
enum ScreenType {
  SCREEN_SPLASH = 0,
  SCREEN_WATCHFACE,
  SCREEN_APP_GRID,
  SCREEN_CHARACTER_STATS,
  SCREEN_STEPS_TRACKER,        // Steps card
  SCREEN_SETTINGS,
  SCREEN_THEME_SELECTOR,
  SCREEN_MUSIC,
  SCREEN_NOTES,
  SCREEN_FILE_BROWSER,
  SCREEN_PDF_READER,
  SCREEN_CALCULATOR,
  SCREEN_FLASHLIGHT,
  SCREEN_WALLPAPER_SELECTOR,
  SCREEN_WEATHER_APP,
  SCREEN_WIFI_MANAGER,
  SCREEN_WIFI_SETUP,
  SCREEN_NETWORK_STATUS,
  SCREEN_NEWS_APP,
  SCREEN_GAMES,
  SCREEN_GACHA,
  SCREEN_COLLECTION,
  SCREEN_TRAINING,
  SCREEN_BOSS_RUSH,
  SCREEN_QUESTS,
  SCREEN_DAILY_QUESTS,         // Daily quests
  SCREEN_ELEMENT_TREE,
  SCREEN_FUSION_GAME,
  SCREEN_CHARACTER_GAME,
  SCREEN_CARD_EVOLUTION,       // Card evolution screen
  SCREEN_DECK_BUILDER,         // Battle deck builder
  SCREEN_TIMER,                // NEW: Timer/Stopwatch
  SCREEN_CONVERTER,            // NEW: Unit Converter
  SCREEN_ACHIEVEMENTS,         // NEW: Achievements
  SCREEN_SHOP,                 // NEW: In-game Shop
  SCREEN_GALLERY,              // NEW: Photo Gallery
  SCREEN_PROGRESSION,          // NEW: Character Progression Tree
  SCREEN_DUAL_CORE,            // DUAL CORE: Switch between Fusion/Widget OS
  SCREEN_SLEEP
};

// App Types
enum AppType {
  APP_WATCHFACE = 0,
  APP_MUSIC,
  APP_NOTES,
  APP_FILES,
  APP_SETTINGS,
  APP_WEATHER,
  APP_GAMES,
  APP_CALCULATOR,
  APP_FLASHLIGHT,
  APP_WALLPAPER_SELECTOR
};

// Main Navigation Screens - UPDATED for infinite loop
// Order: Watchface → Steps → App Grid → Stats → Watchface (loop)
enum MainScreen {
  MAIN_WATCHFACE = 0,
  MAIN_STEPS_TRACKER,      // NEW: Steps card in the loop
  MAIN_APP_GRID_1,
  MAIN_CHARACTER_STATS
};

#define MAIN_SCREEN_COUNT 4   // Updated: 4 screens in loop

// Touch Events - FIXED: Added TOUCH_MOVE
enum TouchEvent {
  TOUCH_NONE = 0,
  TOUCH_PRESS,
  TOUCH_RELEASE,
  TOUCH_TAP,
  TOUCH_LONG_PRESS,
  TOUCH_SWIPE_LEFT,
  TOUCH_SWIPE_RIGHT,
  TOUCH_SWIPE_UP,
  TOUCH_SWIPE_DOWN,
  TOUCH_DRAG,
  TOUCH_MOVE              // FIXED: Added missing touch event
};

// =============================================================================
// STRUCTURES
// =============================================================================

// Touch gesture data - IMPROVED
struct TouchGesture {
  TouchEvent event;
  int16_t x;
  int16_t y;
  int16_t start_x;
  int16_t start_y;
  int16_t end_x;          // End X position
  int16_t end_y;          // End Y position
  int16_t dx;             // Total X movement
  int16_t dy;             // Total Y movement
  uint32_t duration;      // Touch duration in ms
  uint32_t timestamp;
  bool is_valid;
};

// Theme colors structure
struct ThemeColors {
  uint16_t primary;
  uint16_t secondary;
  uint16_t accent;
  uint16_t background;
  uint16_t text;
  uint16_t shadow;
  uint16_t effect1;
  uint16_t effect2;
  uint16_t effect3;
  uint16_t effect4;
  uint8_t corner_radius;
  uint8_t glow_intensity;
  uint8_t animation_speed;
  uint8_t border_width;
};

// Character stats for profile display (renamed to avoid conflict with types.h)
struct ProfileStats {
  const char* stat1_name;
  const char* stat1_desc;
  int stat1_value;
  const char* stat2_name;
  const char* stat2_desc;
  int stat2_value;
  const char* stat3_name;
  const char* stat3_desc;
  int stat3_value;
  const char* stat4_name;
  const char* stat4_desc;
  int stat4_value;
};

// Character profile
struct CharacterProfile {
  const char* name;
  const char* title;
  const char* series;
  const char* signature_move;
  const char* catchphrase;
  ThemeType theme;
  ProfileStats stats;
};

// Watch time structure
struct WatchTime {
  int hour;
  int minute;
  int second;
  int day;
  int month;
  int year;
  int weekday;
};

// Navigation state
struct NavigationState {
  MainScreen currentMain;
  int appGridPage;
  bool isTransitioning;
  uint32_t lastNavigationMs;
  bool navigationLocked;
};

// System state - IMPROVED
struct SystemState {
  // Display
  ScreenType current_screen;
  ThemeType current_theme;
  AppType current_app;
  uint8_t brightness;
  bool display_available;
  
  // Touch
  bool touch_available;
  bool touch_active;
  
  // Power
  bool power_available;
  int battery_percentage;
  bool is_charging;
  bool low_battery_warning;
  
  // LVGL
  bool lvgl_available;
  
  // Time
  uint32_t sleep_timer;
  
  // Connectivity
  bool wifi_connected;
  String wifi_ssid;
  int wifi_signal_strength;
  
  // Activity
  int steps_today;
  int step_goal;
  
  // Player stats - Game economy
  int player_gems;
  int player_level;
  int player_xp;
  int gacha_cards_collected;
  int bosses_defeated;
  int training_streak;
  int daily_login_count;
  
  // Pity system
  int pity_counter;           // Pulls since last Epic+
  int pity_legendary_counter; // Pulls since last Legendary+
  
  // Battle deck (card IDs, -1 = empty slot)
  int battle_deck[5];
  int deck_size;
  
  // Music
  bool music_playing;
  int total_mp3_files;
  int total_pdf_files;
  
  // Filesystem
  bool filesystem_available;
  String current_wallpaper_path;
  bool wallpaper_enabled;
  
  // Power mode
  bool low_power_mode;
};

// =============================================================================
// EXTERNAL DECLARATIONS
// =============================================================================
extern SystemState system_state;
extern NavigationState navState;
extern bool sdCardInitialized;
extern bool wifiConnected;

// Time functions
WatchTime getCurrentTime();

#endif // CONFIG_H
