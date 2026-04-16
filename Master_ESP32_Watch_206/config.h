/*
 * config.h - IMPROVED CONFIGURATION (FIXED)
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 *
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 * Display: CO5300 QSPI AMOLED (410x502)
 *
 * UPDATED: Story Mode + Companion screen types added
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "types.h" // Central type definitions - MUST be included first

// =============================================================================
// DISPLAY CONFIGURATION - ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
// =============================================================================
#define LCD_WIDTH  410
#define LCD_HEIGHT 502

// Display pins (QSPI) - FROM WAVESHARE OFFICIAL pin_config.h
#define LCD_SDIO0  4
#define LCD_SDIO1  5
#define LCD_SDIO2  6
#define LCD_SDIO3  7
#define LCD_SCLK   11
#define LCD_CS     12
#define LCD_RESET  8   // Display Reset (for 2.06" board)

// Display column offset (required for CO5300)
#define LCD_COL_OFFSET 22

// =============================================================================
// I2C CONFIGURATION - FROM WAVESHARE OFFICIAL pin_config.h
// =============================================================================
#define IIC_SDA  15
#define IIC_SCL  14

// I2C Addresses
#define FT3168_ADDR    0x38
#define AXP2101_ADDR   0x34
#define PCF85063_ADDR  0x51
#define RTC_ADDR       0x51
#define QMI8658_ADDR   0x6B
#define ES8311_ADDR    0x18
#define EXPANDER_ADDR  0x20

// =============================================================================
// TOUCH CONFIGURATION - IMPROVED SENSITIVITY (2.06" board)
// =============================================================================
#define TP_INT  38  // Touch Interrupt (changed from 21 for 2.06" board)
#define TP_RST  9   // Touch Reset (for 2.06" board)

// IMPROVED Touch thresholds - better responsiveness
#define SWIPE_THRESHOLD_MIN   30    // Reduced for easier swipes
#define SWIPE_THRESHOLD_MAX   250
#define SWIPE_MAX_DURATION_MS 600   // Increased for easier swipes
#define TAP_MAX_DURATION_MS   250   // Tap timeout
#define TAP_MAX_MOVEMENT      20    // Max movement for tap

// Navigation cooldown - prevents accidental double navigation
#define NAVIGATION_COOLDOWN_MS 200  // Reduced for snappier feel

// =============================================================================
// SD CARD CONFIGURATION (SDMMC) - FOR ESP32-S3-Touch-AMOLED-2.06
// Pins from Waveshare official pin_config.h
// =============================================================================
#define SD_MMC_CLK  2   // SD Clock
#define SD_MMC_CMD  1   // SD Command (FIXED: was 3, official is 1)
#define SD_MMC_D0   3   // SD Data0 (FIXED: was 4 which conflicts with LCD_SDIO0!)
#define SD_MMC_CS   17  // SD Chip Select (from Waveshare official)

// =============================================================================
// AUDIO CONFIGURATION
// =============================================================================
#define PA_PIN  46

// =============================================================================
// BATTERY CONFIGURATION
// =============================================================================
#define BATTERY_LOW_THRESHOLD  20
#define BATTERY_CRITICAL       10

// =============================================================================
// STEP DETECTION CONFIGURATION
// =============================================================================
#define MIN_STEP_THRESHOLD  1.2   // Minimum acceleration delta for step
#define STEP_TIME_WINDOW    250   // Minimum ms between steps

// =============================================================================
// NAVIGATION CONFIGURATION - FIXED
// =============================================================================
#define APP_GRID_PAGES 4   // Four pages of apps (0, 1, 2, 3)
// NOTE: MAIN_SCREEN_COUNT is defined later with the MainScreen enum (= 4)

// =============================================================================
// COLOR DEFINITIONS - RGB565 FORMAT
// =============================================================================
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

// Basic Colors
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     RGB565(255, 0, 0)
#define COLOR_GREEN   RGB565(0, 255, 0)
#define COLOR_BLUE    RGB565(0, 0, 255)
#define COLOR_YELLOW  RGB565(255, 255, 0)
#define COLOR_ORANGE  RGB565(255, 165, 0)
#define COLOR_PURPLE  RGB565(128, 0, 128)
#define COLOR_CYAN    RGB565(0, 255, 255)
#define COLOR_PINK    RGB565(255, 105, 180)
#define COLOR_GRAY    RGB565(128, 128, 128)
#define COLOR_GOLD    RGB565(255, 215, 0)

// =============================================================================
// LUFFY GEAR 5 - SUN GOD NIKA (Modern Anime Style)
// =============================================================================
#define LUFFY_SUN_GOLD      RGB565(255, 215, 50)
#define LUFFY_NIKA_WHITE    RGB565(255, 255, 250)
#define LUFFY_ENERGY_ORANGE RGB565(255, 140, 40)
#define LUFFY_DEEP_BLACK    RGB565(5, 5, 10)
#define LUFFY_CLOUD_WHITE   RGB565(240, 245, 255)
#define LUFFY_SUN_GLOW      RGB565(255, 200, 100)
#define LUFFY_JOY_YELLOW    RGB565(255, 235, 80)
#define LUFFY_FREEDOM_SKY   RGB565(135, 206, 250)
#define LUFFY_RUBBER_PINK   RGB565(255, 180, 180)
#define LUFFY_STRAW_RED     RGB565(200, 30, 30)

// =============================================================================
// SUNG JIN-WOO - SHADOW MONARCH (Dark Purple Power)
// =============================================================================
#define JINWOO_MONARCH_PURPLE RGB565(120, 50, 200)
#define JINWOO_VOID_BLACK     RGB565(10, 5, 20)
#define JINWOO_POWER_VIOLET   RGB565(150, 80, 220)
#define JINWOO_ABSOLUTE_DARK  RGB565(5, 0, 15)
#define JINWOO_DEATH_BLACK    RGB565(15, 10, 25)
#define JINWOO_ARISE_GLOW     RGB565(180, 120, 255)
#define JINWOO_DAGGER_BLUE    RGB565(100, 150, 255)
#define JINWOO_MIST_PURPLE    RGB565(80, 40, 120)
#define JINWOO_SOUL_PURPLE    RGB565(140, 90, 200)
#define JINWOO_PURPLE         RGB565(130, 60, 210)

// =============================================================================
// YUGO - PORTAL MASTER (Cyan Portal Energy)
// =============================================================================
#define YUGO_PORTAL_CYAN    RGB565(0, 220, 255)
#define YUGO_ELIATROPE_TEAL RGB565(0, 180, 200)
#define YUGO_HAT_GOLD       RGB565(255, 200, 80)
#define YUGO_SKY_BLUE_GREY  RGB565(60, 80, 100)
#define YUGO_MYSTERY_BLUE   RGB565(40, 60, 120)
#define YUGO_PORTAL_GLOW    RGB565(100, 255, 255)
#define YUGO_WAKFU_ENERGY   RGB565(80, 200, 220)
#define YUGO_PORTAL_RING    RGB565(50, 180, 200)
#define YUGO_ENERGY_BURST   RGB565(150, 255, 255)
#define YUGO_TEAL           RGB565(0, 200, 180)

// =============================================================================
// NARUTO - SAGE MODE (Orange Chakra)
// =============================================================================
#define NARUTO_CHAKRA_ORANGE  RGB565(255, 140, 0)
#define NARUTO_OUTFIT_BLACK   RGB565(30, 30, 35)
#define NARUTO_SAGE_GOLD      RGB565(255, 200, 50)
#define NARUTO_SLATE_GREY     RGB565(50, 55, 65)
#define NARUTO_SHADOW_ORANGE  RGB565(200, 100, 20)
#define NARUTO_KURAMA_FLAME   RGB565(255, 100, 30)
#define NARUTO_RASENGAN_BLUE  RGB565(100, 180, 255)
#define NARUTO_SAGE_EYES      RGB565(255, 180, 100)
#define NARUTO_WILL_FIRE      RGB565(255, 80, 20)
#define NARUTO_KURAMA_RED     RGB565(255, 60, 30)

// =============================================================================
// GOKU - ULTRA INSTINCT (Divine Silver)
// =============================================================================
#define GOKU_UI_SILVER      RGB565(200, 210, 225)
#define GOKU_GI_BLUE        RGB565(40, 60, 120)
#define GOKU_AURA_WHITE     RGB565(230, 235, 255)
#define GOKU_VOID_BLACK     RGB565(5, 5, 15)
#define GOKU_SILVER_GLOW    RGB565(180, 190, 210)
#define GOKU_DIVINE_SILVER  RGB565(210, 220, 240)
#define GOKU_KI_BLAST_BLUE  RGB565(80, 150, 255)
#define GOKU_SPEED_LINES    RGB565(150, 160, 180)
#define GOKU_LIGHT_SILVER   RGB565(220, 225, 240)

// =============================================================================
// TANJIRO - SUN BREATHING (Fire Orange/Red)
// =============================================================================
#define TANJIRO_FIRE_ORANGE   RGB565(255, 120, 30)
#define TANJIRO_CHECK_GREEN   RGB565(60, 120, 80)
#define TANJIRO_HANAFUDA_RED  RGB565(200, 50, 50)
#define TANJIRO_DARK_CHARCOAL RGB565(35, 30, 30)
#define TANJIRO_SCAR_BURGUNDY RGB565(150, 40, 60)
#define TANJIRO_FLAME_GLOW    RGB565(255, 150, 50)
#define TANJIRO_WATER_BLUE    RGB565(80, 160, 220)
#define TANJIRO_SUN_FIRE      RGB565(255, 100, 0)
#define TANJIRO_SPIRIT_WHITE  RGB565(240, 240, 255)
#define TANJIRO_STEEL_GREY    RGB565(160, 165, 175)

// =============================================================================
// GOJO - INFINITY (Intense Blue)
// =============================================================================
#define GOJO_INFINITY_BLUE    RGB565(60, 120, 255)
#define GOJO_SNOW_WHITE       RGB565(250, 250, 255)
#define GOJO_HOLLOW_PURPLE    RGB565(180, 80, 200)
#define GOJO_VOID_BLACK       RGB565(5, 5, 20)
#define GOJO_DOMAIN_PURPLE    RGB565(100, 40, 150)
#define GOJO_LIGHT_BLUE_GLOW  RGB565(150, 200, 255)
#define GOJO_SIX_EYES_BLUE    RGB565(100, 180, 255)
#define GOJO_DEEP_INFINITY    RGB565(40, 80, 180)
#define GOJO_CURSED_PURPLE    RGB565(150, 60, 180)

// =============================================================================
// LEVI - HUMANITY'S STRONGEST (Military Green/Grey)
// =============================================================================
#define LEVI_SURVEY_GREEN   RGB565(80, 140, 80)
#define LEVI_DARK_UNIFORM   RGB565(40, 45, 50)
#define LEVI_SILVER_BLADE   RGB565(200, 205, 215)
#define LEVI_CHARCOAL_DARK  RGB565(30, 32, 38)
#define LEVI_MILITARY_GREY  RGB565(100, 105, 115)
#define LEVI_STEEL_METAL    RGB565(180, 185, 195)
#define LEVI_CAPE_GREEN     RGB565(60, 100, 60)
#define LEVI_SPEED_BLUR     RGB565(150, 155, 165)
#define LEVI_CLEAN_WHITE    RGB565(245, 248, 255)

// =============================================================================
// SAITAMA - ONE PUNCH (Bold Yellow/Red)
// =============================================================================
#define SAITAMA_HERO_YELLOW   RGB565(255, 230, 50)
#define SAITAMA_CAPE_RED      RGB565(220, 40, 40)
#define SAITAMA_GOLDEN_PUNCH  RGB565(255, 200, 0)
#define SAITAMA_SIMPLE_BLACK  RGB565(10, 10, 12)
#define SAITAMA_DEEP_RED_CAPE RGB565(180, 30, 30)
#define SAITAMA_IMPACT_ORANGE RGB565(255, 150, 50)
#define SAITAMA_GLOVE_RED     RGB565(200, 50, 50)
#define SAITAMA_BOOT_YELLOW   RGB565(220, 200, 50)
#define SAITAMA_BALD_WHITE    RGB565(255, 250, 245)
#define SAITAMA_DOT_EYES      RGB565(0, 0, 0)

// =============================================================================
// DEKU - PLUS ULTRA (Green Lightning)
// =============================================================================
#define DEKU_HERO_GREEN     RGB565(50, 180, 80)
#define DEKU_COSTUME_BLACK  RGB565(25, 30, 35)
#define DEKU_OFA_LIGHTNING  RGB565(100, 255, 150)
#define DEKU_DARK_HERO      RGB565(20, 25, 30)
#define DEKU_BLACKWHIP_DARK RGB565(30, 30, 40)
#define DEKU_FULL_COWL      RGB565(80, 220, 120)
#define DEKU_AIR_FORCE      RGB565(150, 200, 255)
#define DEKU_POWER_VEINS    RGB565(255, 100, 100)
#define DEKU_ALLMIGHT_GOLD   RGB565(255, 200, 50)

// =============================================================================
// BOBOIBOY - ELEMENTAL HERO (Malaysian Anime)
// 7 Elements with Tier Progression & Fusion
// =============================================================================

// Base Element Colors (Galaxy Series)
#define BBB_LIGHTNING_YELLOW  RGB565(255, 220, 0)
#define BBB_LIGHTNING_BLACK   RGB565(30, 30, 35)
#define BBB_WIND_BLUE         RGB565(0, 150, 255)
#define BBB_WIND_CYAN         RGB565(100, 220, 255)
#define BBB_EARTH_BROWN       RGB565(180, 120, 60)
#define BBB_EARTH_ORANGE      RGB565(255, 140, 50)
#define BBB_FIRE_RED          RGB565(220, 30, 50)
#define BBB_FIRE_ORANGE       RGB565(255, 100, 30)
#define BBB_WATER_CYAN        RGB565(0, 255, 255)
#define BBB_WATER_WHITE       RGB565(230, 250, 255)
#define BBB_LEAF_GREEN        RGB565(34, 180, 50)
#define BBB_LEAF_DARK         RGB565(20, 100, 30)
#define BBB_LIGHT_GOLD        RGB565(255, 200, 50)
#define BBB_LIGHT_WHITE       RGB565(255, 255, 240)

// Second Tier / Evolution Colors
#define BBB_THUNDERSTORM_BLACK RGB565(25, 25, 30)
#define BBB_THUNDERSTORM_RED   RGB565(255, 50, 50)
#define BBB_CYCLONE_DARK_BLUE  RGB565(0, 50, 150)
#define BBB_QUAKE_DARK         RGB565(100, 60, 30)
#define BBB_BLAZE_CRIMSON      RGB565(180, 20, 20)
#define BBB_ICE_LIGHT          RGB565(200, 240, 255)
#define BBB_THORN_DARK         RGB565(0, 80, 20)
#define BBB_SOLAR_ORANGE       RGB565(255, 180, 50)

// Fusion Colors
#define BBB_FROSTFIRE_PINK  RGB565(255, 100, 180)
#define BBB_GLACIER_BLUE    RGB565(100, 180, 220)
#define BBB_SUPRA_GOLD      RGB565(255, 200, 100)

// Power Band & UI
#define BBB_BAND_ORANGE    RGB565(255, 140, 30)
#define BBB_BAND_GLOW      RGB565(255, 200, 100)
#define BBB_OCHOBOT_WHITE  RGB565(240, 245, 255)

// =============================================================================
// STORY & COMPANION APP COLORS
// =============================================================================
#define STORY_COLOR  RGB565(200, 100, 50)   // Bronze/adventure
#define CARE_COLOR   RGB565(255, 150, 200)  // Pink/caring

// =============================================================================
// GAME ECONOMY SETTINGS
// =============================================================================
#define GEMS_DAILY_LOGIN     50
#define GEMS_QUEST_COMPLETE  25
#define GEMS_GAME_WIN_MIN    10
#define GEMS_GAME_WIN_MAX    50
#define GEMS_BOSS_DEFEAT     100

#define GACHA_SINGLE_PULL_COST  100
#define GACHA_TEN_PULL_COST     900
#define GACHA_TOTAL_CARDS       100

// Pity system thresholds
#define PITY_EPIC_GUARANTEE      30
#define PITY_LEGENDARY_GUARANTEE 90

// Card evolution costs (duplicates needed)
#define EVOLVE_COST_LV1        3
#define EVOLVE_COST_LV2        5
#define EVOLVE_COST_LV3        10
#define EVOLVE_POWER_MULT_LV1  1.3f
#define EVOLVE_POWER_MULT_LV2  1.6f
#define EVOLVE_POWER_MULT_LV3  2.0f

// Battle deck
#define MAX_DECK_SIZE          5
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
    THEME_BOBOIBOY,
    THEME_CUSTOM,
    THEME_COUNT = 11
};

// BoBoiBoy Element Types (for element switching)
enum BoBoiBoyElement {
    BBB_LIGHTNING = 0,
    BBB_WIND = 1,
    BBB_EARTH = 2,
    BBB_FIRE = 3,
    BBB_WATER = 4,
    BBB_LEAF = 5,
    BBB_LIGHT = 6,
    BBB_ELEMENT_COUNT = 7
};

// Screen Types - UPDATED: Added Story Mode + Companion screens
enum ScreenType {
    SCREEN_SPLASH = 0,
    SCREEN_WATCHFACE,
    SCREEN_APP_GRID,
    SCREEN_CHARACTER_STATS,
    SCREEN_STEPS_TRACKER,
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
    SCREEN_DAILY_QUESTS,
    SCREEN_ELEMENT_TREE,
    SCREEN_FUSION_GAME,
    SCREEN_CHARACTER_GAME,
    SCREEN_CARD_EVOLUTION,
    SCREEN_DECK_BUILDER,
    SCREEN_TIMER,
    SCREEN_CONVERTER,
    SCREEN_ACHIEVEMENTS,
    SCREEN_SHOP,
    SCREEN_GALLERY,
    SCREEN_PROGRESSION,
    // NEW: Story Mode & Companion screens
    SCREEN_STORY_MENU,
    SCREEN_CHAPTER_SELECT,
    SCREEN_STORY_DIALOGUE,
    SCREEN_STORY_BOSS,
    SCREEN_COMPANION,
    SCREEN_COMPANION_GAME,
    SCREEN_SLEEP,
    SCREEN_POMODORO,
    SCREEN_HABITS,
    SCREEN_DUNGEON,
    SCREEN_SD_BACKUP
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
// Order: Watchface -> Steps -> App Grid -> Stats -> Watchface (loop)
enum MainScreen {
    MAIN_WATCHFACE = 0,
    MAIN_STEPS_TRACKER,
    MAIN_APP_GRID_1,
    MAIN_CHARACTER_STATS
};

#define MAIN_SCREEN_COUNT 4

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
    TOUCH_MOVE
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
    int16_t end_x;
    int16_t end_y;
    int16_t dx;
    int16_t dy;
    uint32_t duration;
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
    int pity_counter;
    int pity_legendary_counter;

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
    bool power_saver_enabled;  // User-toggled power saver mode
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
