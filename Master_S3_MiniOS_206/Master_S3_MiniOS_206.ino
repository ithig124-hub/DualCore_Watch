/**
 *  Widget OS v8.0 - SPORT TRACKER + QUICK ACTIONS EDITION
 *  ESP32-S3-Touch-AMOLED-2.06" Smartwatch Firmware
 *  Based on S3 MiniOS v7.4
 *
 *  ═══════════════════════════════════════════════════════════════════════════
 *  NEW IN v8.0:
 *  ═══════════════════════════════════════════════════════════════════════════
 *
 *  1. SPORT + MOVEMENT TRACKER (Activity → Subcard 3, under Steps)
 *     - Session timer with tap-to-start/stop
 *     - Step counter during session
 *     - Sprint detection via IMU acceleration spikes (>15 m/s²)
 *     - Activity streak system (gamified daily goals - 30 min to count)
 *     - RTC persistence for streaks across deep sleep
 *
 *  2. QUICK ACTIONS DOCK (Control Center)
 *     - Swipe DOWN from clock to open
 *     - Swipe UP to close and return to clock
 *     - 5 Apple-style quick toggle icons:
 *       • BLE toggle (on/off)
 *       • WiFi scan
 *       • Most Used Card (Steps shortcut)
 *       • Recent Notification (badge with count)
 *       • Torch toggle
 *
 *  3. ADAPTIVE BATTERY SAVING (Normal Mode Only)
 *     - Dynamic loop delay: 10ms active → 30ms idle → 50ms deep idle
 *     - LVGL tick rate: 10ms (was 2ms) - 5x CPU wakeup reduction
 *     - Automatically adjusts based on user interaction
 *     - No impact on UI responsiveness when active
 *
 *  4. DEEP SLEEP FIX
 *     - Power button (GPIO10) now properly turns screen ON during deep sleep
 *     - Full boot sequence on power button wake (WiFi + NTP sync)
 *
 *  ═══════════════════════════════════════════════════════════════════════════
 *  EXTREME SLEEP MODE - Ultra Low Power Deep Sleep (Preserved from v7.4)
 *  ═══════════════════════════════════════════════════════════════════════════
 *
 *  EXTREME SLEEP BEHAVIOR (when in EXTREME battery saver mode):
 *  1. Power button press → Wake from deep sleep → FULL BOOT (WiFi + NTP sync)
 *  2. Touch screen → Wake → Glance mode (quick time, can return to sleep)
 *  3. No touch for 2 seconds → Turn off display → Enter deep sleep
 *
 *  WAKE SOURCES (EXT1):
 *  - GPIO10: Power button (PWR) - Active LOW → Always FULL BOOT
 *  - GPIO38: Touch controller (FT3168 INT) - Active LOW → Glance mode
 *  - Timer: 30 minute backup → Full boot with NTP sync
 *
 *  HARDWARE THAT STAYS POWERED DURING SLEEP:
 *  - PCF85063 RTC chip: Always on (battery backed) - maintains time
 *  - FT3168 Touch chip: MONITOR mode (~50µA) - generates wake interrupt
 *  - ESP32 RTC memory: For RTC_DATA_ATTR variables
 *
 *  EXPECTED BATTERY LIFE (500mAh battery):
 *  - Always active: ~3-4 hours
 *  - With adaptive saving (normal mode): ~5-8 hours
 *  - With extreme sleep (90% sleep): ~3-7 days
 *  - With extreme sleep (98% sleep): ~10-20 days
 */

#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <time.h>
#include <Arduino.h>
#include "pin_config.h"
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include "ESP_I2S.h"  // For ES8311 audio codec
#include <esp_wifi.h>

// ═══════════════════════════════════════════════════════════════════════════
// GADGETBRIDGE BLE SUPPORT - Time Sync via Bluetooth
// ═══════════════════════════════════════════════════════════════════════════
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#ifdef PCF85063_SLAVE_ADDRESS
#undef PCF85063_SLAVE_ADDRESS
#endif

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "SensorQMI8658.hpp"
#include "SensorPCF85063.hpp"
#include "XPowersLib.h"
#include <FS.h>
#include <SD_MMC.h>
#include "HWCDC.h"
#include <math.h>
#include <Preferences.h>

// ═══════════════════════════════════════════════════════════════
// DUAL BOOT SUPPORT - Double-tap BOOT button to switch OS
// ═══════════════════════════════════════════════════════════════
#include "dual_boot_manager.h"

// Wallpaper images (from 206q)
//#include "AdobeStock_17557.c"
//#include "AdobeStock_2026.c"
//#include "AdobeStock_184869446.c"
//#include "AdobeStock_174564.c"

// NIKE CUSTOM FONT - Futura Condensed Extra Bold 60px
#include "NIKE_FONT.c"
LV_FONT_DECLARE(NIKE_FONT);

#if ARDUINO_USB_CDC_ON_BOOT
#define USBSerial Serial
#else
#if !defined(USBSerial)
HWCDC USBSerial;
#endif
#endif

#define WIDGET_OS_NAME      "Widget OS"

// ═══════════════════════════════════════════════════════════════════════════
// GADGETBRIDGE BLE CONFIGURATION
// Uses Nordic UART Service (NUS) for Bangle.js protocol compatibility
// ═══════════════════════════════════════════════════════════════════════════
#define BLE_DEVICE_NAME "Bangle.js 7C3E"  // Gadgetbridge expects "Bangle.js" prefix

// Nordic UART Service UUIDs (Bangle.js compatible)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // Write
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // Notify

// BLE Objects
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
BLECharacteristic *pRxCharacteristic = NULL;

// BLE State
bool bleDeviceConnected = false;
bool blePreviouslyConnected = false;
bool bleTimeSynced = false;
unsigned long bleLastActivity = 0;
String bleIncomingBuffer = "";

// BLE Auto-Off Timer (1 minute timeout to save battery)
bool bleEnabled = false;                    // BLE on/off state
bool bleAutoOffEnabled = true;              // Auto-off after 3 min of no activity
unsigned long bleStartTime = 0;             // When BLE was enabled
unsigned long bleLastActivityTime = 0;      // Last notification/sync activity
#define BLE_AUTO_OFF_MS 180000              // 3 minutes auto-off
String bleConnectedDeviceName = "";         // Name of connected device

// ═══════════════════════════════════════════════════════════════════════════
// NOTIFICATION SYSTEM
// ═══════════════════════════════════════════════════════════════════════════
#define MAX_NOTIFICATIONS 10
struct Notification {
    String title;
    String body;
    String app;
    unsigned long timestamp;
    bool read;
};
Notification notifications[MAX_NOTIFICATIONS];
int notificationCount = 0;
int notificationFilter = 0;  // 0=All, then cycles through unique apps
String notifAppFilters[MAX_NOTIFICATIONS + 1] = {"All"};  // "All" + unique app names
int notifAppFilterCount = 1;  // Start with "All"
int selectedNotifIndex = -1;  // -1 = list view, >=0 = viewing full notification

// Step sync from phone
int syncedSteps = 0;
int syncedCalories = 0;
int syncedDistance = 0;  // in meters
bool stepsSynced = false;

// Activity tracking variables (for Gadgetbridge)
int lastHeartRate = 0;          // Last measured heart rate (BPM)
int activityIntensity = 0;      // Movement intensity (0-255)

// ═══════════════════════════════════════════════════════════════════════════
// MOON PHASE & SUNRISE/SUNSET DATA
// ═══════════════════════════════════════════════════════════════════════════
float moonPhase = 0.0;          // 0.0 = new moon, 0.5 = full moon, 1.0 = new moon
String moonPhaseName = "---";
String sunriseTime = "--:--";
String sunsetTime = "--:--";
bool astroDataSynced = false;

// ═══════════════════════════════════════════════════════════════════════════
// TALLY COUNTER
// ═══════════════════════════════════════════════════════════════════════════
// State moved to calculator section for grouping

// ═══════════════════════════════════════════════════════════════════════════
// VOICE MEMO - ES8311 Audio Recording
// ═══════════════════════════════════════════════════════════════════════════
#define VOICE_MEMO_FOLDER "/WATCH/MEMOS"
#define VOICE_SAMPLE_RATE 16000
#define VOICE_BUF_SIZE 8000
int voiceMemoCount = 0;
bool voiceMemoRecording = false;
unsigned long voiceRecordStart = 0;
int voiceRecordDuration = 0;  // seconds
I2SClass voiceI2S;
bool voiceI2SInitialized = false;
File voiceMemoFile;  // Current recording file

// ES8311 I2S Pins (from Waveshare example)
#define I2S_BCK_PIN  41
#define I2S_WS_PIN   45
#define I2S_DOUT_PIN 40
#define I2S_DIN_PIN  42
#define I2S_MCLK_PIN 16
#define ES8311_PA_PIN 46  // Power amplifier enable

// Include ES8311 codec driver
extern "C" {
    #include "es8311.h"
}

#define ES8311_MIC_GAIN_SETTING (es8311_mic_gain_t)(3)  // 0-7, 3 = 18dB

// ═══════════════════════════════════════════════════════════════════════════
// RUNNING PACE (Stopwatch + Activity)
// ═══════════════════════════════════════════════════════════════════════════
bool runningModeActive = false;
unsigned long runningStartTime = 0;
int runningStartSteps = 0;
float runningPace = 0.0;  // steps per minute
float runningDistance = 0.0;  // km

// ═══════════════════════════════════════════════════════════════════════════
// DICE ROLLER & MAGIC 8 BALL
// ═══════════════════════════════════════════════════════════════════════════
int diceValue1 = 1;
int diceValue2 = 1;
bool diceRolling = false;
unsigned long diceRollStart = 0;

String magic8BallAnswer = "Shake to ask";
bool magic8Shaking = false;
const char* magic8Answers[] = {
    "Yes", "No", "Maybe", "Definitely", "Ask again",
    "Absolutely", "No way", "Probably", "Unlikely", "100%",
    "Not now", "Soon", "Never", "Of course", "Doubtful"
};
#define NUM_8BALL_ANSWERS 15

// Forward declarations for BLE
void initGadgetbridgeBLE();
void handleGadgetbridgeMessage(String message);
void sendBLEResponse(String response);
void stopBLE();
void createBluetoothCard();
void startBLE();  // Forward declaration for BLE start function
void createNotificationsCard();
void addNotification(String app, String title, String body);
void showNotificationPopup(int index);
void updateNotifAppFilters();
void createDiceRollerCard();
void createMagic8BallCard();
void createTallyCounterCard();
void createVoiceMemoCard();
void createRunningCard();
void createSportTrackerCard();
void createQuickActionsDock();
void handleQuickActionsTap(int x, int y);

// Feature gating functions
bool shouldUpdateForCategory(int category, int subCard);
bool isGameActive(int gameSubCard);

// ═══════════════════════════════════════════════════════════════════════════
// MISSING DECLARATIONS - COMPILATION FIXES
// ═══════════════════════════════════════════════════════════════════════════
#define FUSION_PROTOCOL_VERSION "1.0"
#define WEB_SERIAL_BUFFER_SIZE 256
char webSerialBuffer[WEB_SERIAL_BUFFER_SIZE];
int webSerialBufferIndex = 0;

#define WIDGET_OS_VERSION   "8.0-SportTracker"

// Weather icon color helper function
uint32_t getWeatherIconColor(const char* description) {
    String desc = String(description);
    desc.toLowerCase();
    if (desc.indexOf("clear") >= 0 || desc.indexOf("sunny") >= 0) return 0xFFD60A;
    if (desc.indexOf("cloud") >= 0 || desc.indexOf("overcast") >= 0) return 0xAEAEB2;
    if (desc.indexOf("rain") >= 0 || desc.indexOf("drizzle") >= 0) return 0x0A84FF;
    if (desc.indexOf("snow") >= 0 || desc.indexOf("sleet") >= 0) return 0xE0F7FA;
    if (desc.indexOf("thunder") >= 0 || desc.indexOf("storm") >= 0) return 0xFFD60A;
    if (desc.indexOf("mist") >= 0 || desc.indexOf("fog") >= 0 || desc.indexOf("haze") >= 0) return 0x9E9E9E;
    return 0x8E8E93;
}

#define WIDGET_OS_BUILD     "merged-lowpower-nike"
#define DEVICE_ID           "WOS-208A"

// Forward declaration for navigation
void navigateTo(int category, int subCard);

#define DEVICE_SCREEN       "2.06"
#define DEVICE_HW_REV       "A"

// ═══════════════════════════════════════════════════════════════════════════
// LOW POWER MODE - Light sleep enabled in EXTREME mode (no battery threshold)
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// LOW POWER STATE MACHINE - FROM v7.3
// ═══════════════════════════════════════════════════════════════════════════
enum PowerState {
    POWER_STATE_ACTIVE = 0,      // Full power, screen on, all systems running
    POWER_STATE_DIMMED,          // Reduced brightness, slower refresh
    POWER_STATE_SCREEN_OFF,      // Screen off but CPU active (transition state)
    POWER_STATE_LIGHT_SLEEP,     // CPU paused, RAM retained, wake sources active
    POWER_STATE_WAKING           // Transitioning from sleep to active
};

volatile PowerState currentPowerState = POWER_STATE_ACTIVE;
volatile PowerState previousPowerState = POWER_STATE_ACTIVE;

// Interrupt flags for wake sources
volatile bool touchWakeFlag = false;
volatile bool imuWakeFlag = false;
volatile bool buttonWakeFlag = false;
volatile bool rtcAlarmFlag = false;
volatile bool lightSleepEnabled = false;  // Dynamic based on battery + mode

// Step counting via IMU interrupt
volatile uint32_t stepCounterISR = 0;
volatile bool stepDetectedFlag = false;

// LVGL task control for sleep
volatile bool lvglTaskRunning = true;

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE DEFINITIONS - 7 PREMIUM STYLES
// ═══════════════════════════════════════════════════════════════════════════
#define NUM_WATCH_FACES 7

typedef struct {
  const char* name;
  const char* description;
} WatchFaceInfo;

WatchFaceInfo watchFaces[NUM_WATCH_FACES] = {
  {"Digital", "Big time with stats"},
  {"Word Clock", "Time in words"},
  {"Analog Rings", "Classic with activity"},
  {"Nike Sport", "Tap dial to change color"},
  {"Minimal", "Clean dark"},
  {"Fitness", "Activity rings focus"},
  {"Photo", "SD card photos"}
};

// Nike Watch Face Color Variants - ENHANCED with all Nike colorways
#define NUM_NIKE_COLORS 10
uint8_t currentNikeColor = 0;  // Start with Nike OG

struct NikeColorScheme {
    uint32_t primary;      // Main text color
    uint32_t secondary;    // Shadow/outline color  
    uint32_t background;   // Background color (0=black for AMOLED)
    uint32_t swoosh;       // Nike swoosh/tick color
    const char* name;
    bool fullColor;        // true = solid bg, false = black AMOLED
    bool hasDial;          // true = circular dial with tick marks
};

// ENHANCED: 10 Nike color variants - ALL CLASSIC COLORWAYS
NikeColorScheme nikeColors[NUM_NIKE_COLORS] = {
    // 0. Nike OG - Classic Black/Red (THE ORIGINAL!)
    {0xE53935, 0x1A0000, 0x000000, 0xE53935, "Nike OG", false, false},
    // 1. Nike Volt - Neon Yellow/Black
    {0x000000, 0x1A1A00, 0xCCFF00, 0xFF4500, "Volt", true, false},
    // 2. Nike Orange - Orange/Black
    {0x000000, 0x331100, 0xFF6D00, 0xFFFFFF, "Orange", true, false},
    // 3. Nike Blue - Blue/White
    {0xFFFFFF, 0x0D47A1, 0x0D47A1, 0x90CAF9, "Blue", true, false},
    // 4. Nike Infrared - Hot Pink/Black (CLASSIC!)
    {0xFF1744, 0x1A0008, 0x000000, 0xFF1744, "Infrared", false, false},
    // 5. Nike Aqua - Teal/Black
    {0x00E5FF, 0x001A1F, 0x000000, 0x00E5FF, "Aqua", false, false},
    // 6. Blue Dial - White text on black, has circular dial
    {0x007AFF, 0x003366, 0x000000, 0xFFFFFF, "Blue Dial", false, true},
    // 7. Purple - White text on deep purple
    {0xFFFFFF, 0x4A0066, 0x9932CC, 0xFFFFFF, "Purple", true, false},
    // 8. Pink - White text on hot pink
    {0xFFFFFF, 0x330022, 0xFF2D92, 0xFFFFFF, "Pink", true, false},
    // 9. White - Black text on clean white (classic Nike)
    {0x000000, 0xCCCCCC, 0xFFFFFF, 0x000000, "White", true, false}
};

// ═══════════════════════════════════════════════════════════════════════════
// PHOTO CLOCKFACE - SD Card Image Display
// ═══════════════════════════════════════════════════════════════════════════
#define MAX_SD_PHOTOS 50
uint8_t currentPhotoIndex = 0;
int numSDPhotos = 0;
String sdPhotoFiles[MAX_SD_PHOTOS];
uint8_t *photoBuf = NULL;           // RGB565 image buffer (PSRAM)
lv_img_dsc_t photoImgDsc;           // LVGL image descriptor
bool photoLoaded = false;

// ═══════════════════════════════════════════════════════════════════════════
// 5-DAY FORECAST DATA STRUCTURE
// ═══════════════════════════════════════════════════════════════════════════
struct ForecastDay {
  char dayName[4];
  float tempHigh;
  float tempLow;
  char icon[16];
  char condition[16];
};

ForecastDay forecast5Day[5] = {
  {"Mon", 28, 18, "sun", "Sunny"},
  {"Tue", 26, 17, "cloud", "Cloudy"},
  {"Wed", 24, 16, "rain", "Rain"},
  {"Thu", 27, 19, "sun", "Sunny"},
  {"Fri", 29, 20, "sun", "Clear"}
};
bool forecastLoaded = false;

// ═══════════════════════════════════════════════════════════════════════════
// GRADIENT THEME STRUCT
// ═══════════════════════════════════════════════════════════════════════════
typedef struct {
  const char* name;
  lv_color_t color1;
  lv_color_t color2;
  lv_color_t text;
  lv_color_t accent;
  lv_color_t secondary;
} GradientTheme;

#define NUM_THEMES 8

// ═══════════════════════════════════════════════════════════════════════════
// WALLPAPER THEME STRUCT - Enhanced gradient wallpapers
// ═══════════════════════════════════════════════════════════════════════════
typedef struct {
  const char* name;
  lv_color_t top;
  lv_color_t mid1;
  lv_color_t mid2;
  lv_color_t bottom;
} WallpaperTheme;

// Wallpaper themes - PREMIUM custom scenes BETTER THAN APPLE!
WallpaperTheme gradientWallpapers[] = {
  // None - solid theme color
  {"None", lv_color_hex(0x1C1C1E), lv_color_hex(0x1C1C1E), lv_color_hex(0x1C1C1E), lv_color_hex(0x1C1C1E)},

  // 1. Mountain Sunset - Epic layered mountains with sun
  {"Mountain Sunset", lv_color_hex(0x4A90D9), lv_color_hex(0xFF7F50), lv_color_hex(0xE07020), lv_color_hex(0x5D4037)},

  // 2. Golden Peaks - Sunrise with golden rays
  {"Golden Peaks", lv_color_hex(0xFF6B35), lv_color_hex(0xFFD700), lv_color_hex(0xDC6B00), lv_color_hex(0x1A0A00)},

  // 3. Canyon Dawn - Pink desert canyon walls
  {"Canyon Dawn", lv_color_hex(0x87CEEB), lv_color_hex(0xFFB6C1), lv_color_hex(0xCD5C5C), lv_color_hex(0x8B4513)},

  // 4. Island Paradise - Tropical sunset with palm trees
  {"Island Paradise", lv_color_hex(0xE6B3CC), lv_color_hex(0x9370DB), lv_color_hex(0x4169E1), lv_color_hex(0x006994)},

  // 5. Alpine Meadow - Green hills with flowers
  {"Alpine Meadow", lv_color_hex(0x87CEEB), lv_color_hex(0xF0E68C), lv_color_hex(0x32CD32), lv_color_hex(0x228B22)},

  // 6. Twilight Ocean - Night sea with moon and stars
  {"Twilight Ocean", lv_color_hex(0x0D0D1A), lv_color_hex(0x191970), lv_color_hex(0x4169E1), lv_color_hex(0x000033)}
};
#define NUM_GRADIENT_WALLPAPERS 7

// ═══════════════════════════════════════════════════════════════════════════
// UI EVENT SYSTEM (FROM FIXED - STABLE)
// ═══════════════════════════════════════════════════════════════════════════
enum UIEventType {
    UI_EVENT_NONE = 0,
    UI_EVENT_NAV_LEFT,
    UI_EVENT_NAV_RIGHT,
    UI_EVENT_NAV_UP,
    UI_EVENT_NAV_DOWN,
    UI_EVENT_TAP,
    UI_EVENT_SCREEN_ON,
    UI_EVENT_SCREEN_OFF,
    UI_EVENT_REFRESH,
    UI_EVENT_LOW_BATTERY,
    UI_EVENT_SHUTDOWN,
    UI_EVENT_ENTER_SLEEP,
    UI_EVENT_WAKE_UP
};

volatile UIEventType ui_event = UI_EVENT_NONE;
volatile int ui_event_param1 = 0;
volatile int ui_event_param2 = 0;
TaskHandle_t ui_task_handle = NULL;

volatile uint32_t last_ui_activity = 0;
volatile uint32_t last_lvgl_response = 0;
const uint32_t LVGL_STALL_TIMEOUT_MS = 5000;

void panic_recover();
void ui_activity_ping() { last_ui_activity = millis(); }

// ═══════════════════════════════════════════════════════════════════════════
// SD CARD PATHS (FROM 206q - FULL STRUCTURE)
// ═══════════════════════════════════════════════════════════════════════════
#define SD_ROOT_PATH            "/WATCH"
#define SD_SYSTEM_PATH          "/WATCH/SYSTEM"
#define SD_SYSTEM_LOGS_PATH     "/WATCH/SYSTEM/logs"
#define SD_CONFIG_PATH          "/WATCH/CONFIG"
#define SD_FACES_PATH           "/WATCH/FACES"
#define SD_FACES_CUSTOM_PATH    "/WATCH/FACES/custom"
#define SD_FACES_IMPORTED_PATH  "/WATCH/FACES/imported"
#define SD_IMAGES_PATH          "/WATCH/IMAGES"
#define SD_MUSIC_PATH           "/WATCH/MUSIC"
#define SD_CACHE_PATH           "/WATCH/CACHE"
#define SD_CACHE_TEMP_PATH      "/WATCH/CACHE/temp"
#define SD_UPDATE_PATH          "/WATCH/UPDATE"
#define SD_WIFI_PATH            "/WATCH/wifi"
#define SD_BACKUP_PATH          "/WATCH/BACKUPS"
#define SD_FIRMWARE_PATH        "/WATCH/FIRMWARE"
#define SD_LOGS_PATH            "/WATCH/LOGS"
#define SD_WALLPAPERS_PATH      "/WATCH/WALLPAPERS"
#define SD_TXT_PATH             "/WATCH/TXT"
#define SD_POWER_LOGS_PATH      "/WATCH/POWER_LOGS"
#define SD_TIME_BACKUP_PATH     "/WATCH/TIME"
#define SD_TIME_BACKUP_FILE     "/WATCH/TIME/backup.txt"

// System files
#define SD_DEVICE_JSON          "/WATCH/SYSTEM/device.json"
#define SD_OS_JSON              "/WATCH/SYSTEM/os.json"
#define SD_BUILD_TXT            "/WATCH/SYSTEM/build.txt"
#define SD_BOOT_LOG             "/WATCH/SYSTEM/logs/boot.log"

// Config files
#define SD_USER_JSON            "/WATCH/CONFIG/user.json"
#define SD_DISPLAY_JSON         "/WATCH/CONFIG/display.json"
#define SD_POWER_JSON           "/WATCH/CONFIG/power.json"
#define SD_USER_DATA_PATH       "/WATCH/CONFIG/user_data.json"
#define SD_CONFIG_TXT           "/WATCH/CONFIG/config.txt"

// Power log files
#define SD_POWER_LOG_CURRENT    "/WATCH/POWER_LOGS/current.txt"
#define SD_POWER_SUMMARY        "/WATCH/POWER_LOGS/summary.txt"

// Other files
#define SD_UPDATE_README        "/WATCH/UPDATE/README.txt"
#define SD_WIFI_CONFIG          "/WATCH/wifi/config.txt"
#define SD_PHOTOS_PATH          "/WATCH/PHOTOS"

enum SDCardStatus { 
    SD_STATUS_NOT_PRESENT = 0, 
    SD_STATUS_MOUNTED_OK, 
    SD_STATUS_MOUNT_FAILED,
    SD_STATUS_CORRUPT,
    SD_STATUS_READ_ONLY,
    SD_STATUS_INIT_IN_PROGRESS
};
SDCardStatus sdCardStatus = SD_STATUS_NOT_PRESENT;
bool sdCardInitialized = false;
bool sdStructureCreated = false;
String sdErrorMessage = "";
uint64_t sdCardSizeMB = 0;
String sdCardType = "Unknown";

#define AUTO_BACKUP_INTERVAL_MS (24UL * 60UL * 60UL * 1000UL)
bool autoBackupEnabled = true;
unsigned long lastAutoBackup = 0;
unsigned long lastBackupTime = 0;
int totalBackups = 0;

struct SDCardHealth {
    uint64_t totalBytes, usedBytes, freeBytes;
    float usedPercent;
    bool mounted, healthy;
    String lastError;
    int writeErrors, readErrors;
};
SDCardHealth sdHealth = {0};

// ═══════════════════════════════════════════════════════════════════════════
// WIFI CONFIG (ENHANCED FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
#define MAX_WIFI_NETWORKS 5

struct WiFiNetwork {
    char ssid[64];
    char password[64];
    bool valid;
    bool isOpen;
    int32_t rssi;
};
WiFiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
int numWifiNetworks = 0;
int connectedNetworkIndex = -1;

char weatherCity[64] = "Perth";
char weatherCountry[8] = "AU";
long gmtOffsetSec = 8 * 3600;
const char* NTP_SERVER = "pool.ntp.org";
const int DAYLIGHT_OFFSET_SEC = 0;

const char* OPENWEATHER_API = "3795c13a0d3f7e17799d638edda60e3c";
bool wifiConnected = false;
bool wifiConfigFromSD = false;

// ═══════════════════════════════════════════════════════════════════════════
// TIMING & BATTERY
// ═══════════════════════════════════════════════════════════════════════════
#define SAVE_INTERVAL_MS 7200000UL
#define SCREEN_OFF_TIMEOUT_MS 3000
#define SCREEN_OFF_TIMEOUT_SAVER_MS 3000
#define USAGE_HISTORY_SIZE 24
#define CARD_USAGE_SLOTS 12

// LVGL Tick Period - Increased to 10ms for better battery efficiency (was 2ms)
// This cuts CPU wakeups by 5x while maintaining smooth UI
#define LVGL_TICK_PERIOD_MS 10
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;

// ═══════════════════════════════════════════════════════════════════════════
// POWER SAVING CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════
#define CPU_FREQ_NORMAL 240
#define CPU_FREQ_SAVER 80
#define CPU_FREQ_SLEEP 40
#define SENSOR_POLL_NORMAL_MS 200   // 5 times per second is enough
#define SENSOR_POLL_SAVER_MS 500    // 2 times per second in saver mode
#define LVGL_REFRESH_NORMAL_MS 33
#define LVGL_REFRESH_SAVER_MS 67
#define WEATHER_SYNC_INTERVAL_MS (60UL * 60UL * 1000UL)

bool powerSaverActive = false;
unsigned long lastSensorPoll = 0;
unsigned long lastWeatherSync = 0;
unsigned long sensorPollInterval = SENSOR_POLL_NORMAL_MS;
bool touchEnabled = true;

// ═══════════════════════════════════════════════════════════════════════════
// POWER CONSUMPTION LOGGING
// ═══════════════════════════════════════════════════════════════════════════
#define POWER_LOG_INTERVAL_MS (5UL * 60UL * 1000UL)
#define POWER_SUMMARY_INTERVAL_MS (60UL * 60UL * 1000UL)

struct PowerLogEntry {
    uint32_t timestamp;
    uint8_t batteryPercent;
    uint16_t batteryVoltage;
    uint8_t saverLevel;
    uint16_t cpuFreqMHz;
    uint32_t screenOnSecs;
    uint32_t screenOffSecs;
    float drainRatePerHour;
};

struct PowerLogSession {
    uint32_t sessionStartMs;
    uint8_t startBatteryPercent;
    uint8_t currentBatteryPercent;
    uint32_t totalScreenOnMs;
    uint32_t totalScreenOffMs;
    uint32_t modeChanges;
    float avgDrainRate;
    uint8_t dominantMode;
};

PowerLogSession currentPowerSession = {0};
unsigned long lastPowerLogMs = 0;
unsigned long lastPowerSummaryMs = 0;
unsigned long lastLogRotationCheck = 0;
bool powerLoggingEnabled = true;

#define MAX_LOG_FILE_SIZE (50UL * 1024UL)
#define LOG_ROTATION_CHECK_MS (60UL * 60UL * 1000UL)
uint8_t lastLogDay = 0;

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════════════════
SensorQMI8658 qmi;
SensorPCF85063 rtc;
XPowersPMU power;
IMUdata acc, gyr;
Preferences prefs;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COL_OFFSET1, LCD_ROW_OFFSET1, LCD_COL_OFFSET2, LCD_ROW_OFFSET2);
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

#define NUM_IDENTITIES 15

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION - 11 CATEGORIES
// ═══════════════════════════════════════════════════════════════════════════
#define NUM_CATEGORIES 12
enum Category {
  CAT_CLOCK = 0,
  CAT_COMPASS,
  CAT_ACTIVITY,
  CAT_GAMES,
  CAT_WEATHER,
  CAT_TIMER,
  CAT_TORCH,
  CAT_TOOLS,
  CAT_BLUETOOTH,
  CAT_SETTINGS,
  CAT_SYSTEM,
  CAT_ABOUT
};

int currentCategory = CAT_CLOCK;
int currentSubCard = 0;
const int maxSubCards[] = {1, 2, 4, 11, 2, 1, 2, 5, 2, 2, 6, 1};  // Activity now has 4 subcards (added Sport Tracker)

// ═══════════════════════════════════════════════════════════════════════════
// QUICK ACTIONS DOCK (Control Center) - Swipe DOWN from clock
// ═══════════════════════════════════════════════════════════════════════════
bool quickActionsDockVisible = false;
unsigned long quickDockOpenTime = 0;

// ═══════════════════════════════════════════════════════════════════════════
// FEATURE GATING - Only update active screen, pause background apps
// ═══════════════════════════════════════════════════════════════════════════
// Forward declarations for feature gating (actual vars declared later)
extern bool screenOn;
extern bool isTransitioning;

bool shouldUpdateForCategory(int category, int subCard) {
    // Always allow time updates (CAT_CLOCK) and step counting
    // Pause all other background processing when not on that screen
    
    // If screen is off, only allow essential background tasks
    if (!screenOn) {
        return false;  // No UI updates when screen is off
    }
    
    // Check if the requested category/subcard is the currently active one
    if (category == currentCategory && (subCard == -1 || subCard == currentSubCard)) {
        return true;
    }
    
    return false;  // Pause updates for non-active screens
}

// Quick check for whether to run game loops
bool isGameActive(int gameSubCard) {
    return screenOn && 
           currentCategory == CAT_GAMES && 
           currentSubCard == gameSubCard && 
           !isTransitioning;
}

// ═══════════════════════════════════════════════════════════════════════════
// BRIGHTNESS SLIDER - Interactive drag control for Control Center
// ═══════════════════════════════════════════════════════════════════════════
bool brightnessDragActive = false;
int dockBrightness = 150;  // Current brightness value (0-255)
int dockBrightnessBarX = 48;  // X start of brightness bar (from left edge)
int dockBrightnessBarWidth = 0;  // Will be calculated based on LCD_WIDTH
int dockBrightnessBarY = 0;  // Y position of brightness bar

// ═══════════════════════════════════════════════════════════════════════════
// SPORT + MOVEMENT TRACKER (Activity subcard 3)
// ═══════════════════════════════════════════════════════════════════════════
bool sportTrackerActive = false;
unsigned long sportSessionStart = 0;
int sportSessionSteps = 0;
int sportSessionStartSteps = 0;
float sportSessionDistance = 0.0;  // km
int sportSprintCount = 0;          // Number of sprints detected
bool sprintDetected = false;
float lastAccelMagnitude = 0.0;
#define SPRINT_THRESHOLD 15.0      // m/s² acceleration spike for sprint detection

// Activity streak system
RTC_DATA_ATTR int activityStreakDays = 0;
RTC_DATA_ATTR int lastActivityDay = -1;
RTC_DATA_ATTR int dailyActiveMinutes = 0;
#define STREAK_GOAL_MINUTES 30     // 30 min active to count as streak day

// ═══════════════════════════════════════════════════════════════════════════
// ADAPTIVE BATTERY SAVING (Normal Mode Only)
// ═══════════════════════════════════════════════════════════════════════════
unsigned long lastUserInteraction = 0;
int adaptiveLoopDelay = 10;        // Start at 10ms, increases when idle
#define ADAPTIVE_IDLE_THRESHOLD_1 5000   // 5 sec idle → 20ms delay
#define ADAPTIVE_IDLE_THRESHOLD_2 15000  // 15 sec idle → 30ms delay  
#define ADAPTIVE_IDLE_THRESHOLD_3 30000  // 30 sec deep idle → 50ms delay

// Track if we're in a game launched from selector (for swipe-up navigation)
bool inGameFromSelector = false;

bool isTransitioning = false;
int transitionDir = 0;
float transitionProgress = 0.0;
unsigned long transitionStartMs = 0;
const unsigned long TRANSITION_DURATION = 200;

volatile bool navigationLocked = false;
unsigned long lastNavigationMs = 0;
const unsigned long NAVIGATION_COOLDOWN_MS = 150;

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY INTELLIGENCE CONFIGURATION - 350mAh battery
// ═══════════════════════════════════════════════════════════════════════════
#define BATTERY_CAPACITY_MAH 350
#define SCREEN_ON_CURRENT_MA 80
#define SCREEN_OFF_CURRENT_MA 15
#define SAVER_MODE_CURRENT_MA 40
#define LIGHT_SLEEP_CURRENT_MA 3  // NEW: Light sleep current

#define LOW_BATTERY_WARNING 20
#define CRITICAL_BATTERY_WARNING 10

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY INTELLIGENCE DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════
struct BatteryStats {
    uint32_t screenOnTimeMs;
    uint32_t screenOffTimeMs;
    uint32_t sessionStartMs;
    uint16_t hourlyScreenOnMins[USAGE_HISTORY_SIZE];
    uint16_t hourlyScreenOffMins[USAGE_HISTORY_SIZE];
    uint16_t hourlySteps[USAGE_HISTORY_SIZE];
    uint8_t currentHourIndex;
    uint32_t cardUsageTime[CARD_USAGE_SLOTS];
    uint8_t batteryAtHourStart;
    float avgDrainPerHour;
    float weightedDrainRate;
    float dailyAvgScreenOnHours[7];
    float dailyAvgDrainRate[7];
    uint8_t currentDayIndex;
    uint32_t simpleEstimateMins;
    uint32_t weightedEstimateMins;
    uint32_t learnedEstimateMins;
    uint32_t combinedEstimateMins;
    uint32_t lightSleepTimeMs;  // NEW: Track light sleep time
};

BatteryStats batteryStats = {0};

bool batterySaverMode = false;
bool batterySaverAutoEnabled = false;

// ═══════════════════════════════════════════════════════════════════════════
// DEEP SLEEP & POWER MANAGEMENT - EXTREME SLEEP MODE
// ═══════════════════════════════════════════════════════════════════════════
#define DEEP_SLEEP_TIMEOUT_MS 240000         // 4 minutes of no touch = deep sleep (normal)
#define EXTREME_SLEEP_TIMEOUT_MS 2000        // 2 seconds of no touch = deep sleep (EXTREME mode)
#define GLANCE_MODE_TIMEOUT_MS 2000          // 2 seconds to interact or go back to sleep
#define GLANCE_MODE_DISPLAY_MS 500           // Show time for 500ms minimum
unsigned long lastTouchTime = 0;
bool inDeepSleep = false;
bool extremeSleepEnabled = true;              // Enable 2-second extreme sleep
bool glanceModeEnabled = true;                // Enable quick-wake glance mode

// RTC time structure (stored in RTC memory to persist across deep sleep)
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    bool valid;  // Flag to check if time is valid
} ExtremeSleep_RTC_Time;

// Store in RTC memory (survives deep sleep)
RTC_DATA_ATTR ExtremeSleep_RTC_Time savedRtcTime = {12, 0, 0, 1, 1, 2026, false};
RTC_DATA_ATTR uint32_t extremeSleepCount = 0;
RTC_DATA_ATTR bool wokeFromExtremeSleep = false;
RTC_DATA_ATTR int savedBatterySaverLevel = 0;  // Remember mode across sleep (0=OFF, 1=MEDIUM, 2=EXTREME)
RTC_DATA_ATTR int savedWatchFaceIndex = 0;  // Remember watch face for glance mode
RTC_DATA_ATTR uint32_t savedStepCount = 0;  // Remember step count across sleep/watchdog

// ═══════════════════════════════════════════════════════════════════════════
// AMOLED BURN-IN PREVENTION
// ═══════════════════════════════════════════════════════════════════════════
#define BURN_IN_SHIFT_INTERVAL_MS 60000  // Shift every 60 seconds
#define BURN_IN_SHIFT_PIXELS 2           // Shift by 2 pixels max
int8_t burnInOffsetX = 0;
int8_t burnInOffsetY = 0;
unsigned long lastBurnInShift = 0;

// ═══════════════════════════════════════════════════════════════════════════
// PARTIAL DISPLAY REFRESH
// ═══════════════════════════════════════════════════════════════════════════
bool partialRefreshEnabled = true;
uint32_t lastDisplayHash = 0;  // Track if display needs full refresh

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY SAVER LEVELS - ENHANCED WITH LIGHT SLEEP FOR EXTREME
// ═══════════════════════════════════════════════════════════════════════════
// BATTERY SAVER LEVELS - 3 OPTIONS ONLY: OFF, MEDIUM, EXTREME
// ═══════════════════════════════════════════════════════════════════════════
enum BatterySaverLevel {
    BATTERY_SAVER_OFF = 0,
    BATTERY_SAVER_MEDIUM = 1,      // Was index 2, now index 1
    BATTERY_SAVER_EXTREME = 2      // Was index 3, now index 2 - Light sleep enabled
};

// POWER RESERVE MODE - Critical battery mode (< 20%)
// Only shows time briefly, no navigation allowed
bool powerReserveMode = false;
unsigned long powerReserveWakeTime = 0;
const int POWER_RESERVE_DISPLAY_MS = 500;  // Show time for 0.5 seconds only

// Forward declarations for power reserve functions (must be before screenOnFunc)
void createPowerReserveFace();
void enterPowerReserveMode();
void exitPowerReserveMode();
void checkPowerReserveTimeout();
void disableGyroIfNotNeeded();

BatterySaverLevel batterySaverLevel = BATTERY_SAVER_OFF;  // Default to OFF

struct BatterySaverSettings {
    const char* name;
    int brightness;
    int screenTimeoutMs;
    bool disableWifiSync;
    bool reduceRefresh;
    float estimatedLifeHours;
    bool enableLightSleep;  // Light sleep flag
    int cpuFreqMhz;         // CPU frequency
    bool bleAutoOff;        // Auto-disable BLE
    int clockRefreshSec;    // How often to update clock (seconds)
};

// 3 MODES ONLY: Off, Medium, Extreme
// Off: Balanced - reasonable settings for daily use
// Medium: Power saver - lower brightness, faster timeout
// Extreme: Max saving - CPU 80MHz, light sleep, minimal screen
BatterySaverSettings saverModes[3] = {
    // name,     bright, timeout, noWifi, slowRefresh, hours, sleep, CPU,  BLEoff, clockSec
    {"Off",      100,    5000,    false,  false,       18.0,  false, 160,  false,  1},
    {"Medium",    60,    3000,    false,  true,        28.0,  false, 80,   false,  5},
    {"Extreme",   30,    1500,    true,   true,        55.0,  true,  80,   true,   60}
};

// Forward declarations for functions using custom types
void applyBatterySaverMode(BatterySaverLevel level);
void handle_ui_event(UIEventType event, int param1, int param2);
GradientTheme* getSafeTheme();
int getSafeThemeIndex();


// ═══════════════════════════════════════════════════════════════════════════
// USER DATA
// ═══════════════════════════════════════════════════════════════════════════
struct UserData {
  uint32_t steps;
  uint32_t dailyGoal;
  int stepStreak;
  float totalDistance;
  float totalCalories;
  uint32_t stepHistory[7];
  int blackjackStreak;
  int gamesWon;
  int gamesPlayed;
  uint32_t clickerScore;
  int brightness;
  int screenTimeout;
  int themeIndex;
  int compassMode;
  int wallpaperIndex;
  int watchFaceIndex;
  bool identitiesUnlocked[NUM_IDENTITIES];
  uint32_t identityProgress[NUM_IDENTITIES];
  int selectedIdentity;
} userData = {0, 10000, 7, 0.0, 0.0, {0}, 0, 0, 0, 0, 200, 1, 0, 0, 0, 0};

bool screenOn = true;
unsigned long lastActivityMs = 0;
unsigned long screenOnStartMs = 0;
unsigned long screenOffStartMs = 0;

#define BOOT_BUTTON     0
#define PWR_BUTTON      10

bool powerButtonPressed = false;
unsigned long powerButtonPressStartMs = 0;

uint8_t clockHour = 10, clockMinute = 30, clockSecond = 0;
uint8_t currentDay = 3;

float weatherTemp = 24.0;
String weatherDesc = "Sunny";
float weatherHigh = 28.0;
float weatherLow = 18.0;
bool weatherDataLoaded = false;

uint16_t batteryVoltage = 4100;
uint8_t batteryPercent = 85;
bool isCharging = false;
uint32_t freeRAM = 234567;

bool hasIMU = false, hasRTC = false, hasPMU = false, hasSD = false;
bool gyroEnabled = false;  // Track gyro power state - disabled by default to save power

const float ALPHA = 0.98;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float compassHeading = 0.0, compassHeadingSmooth = 0.0;
float compassNorthOffset = 0.0;
float tiltX = 0.0, tiltY = 0.0;

bool torchOn = false;
bool torchEnabled = false;  // Torch state for Quick Actions dock
int torchBrightness = 255;
int torchColorIndex = 0;
uint32_t torchColors[] = {0xFFFFFF, 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00};
const char* torchColorNames[] = {"White", "Red", "Green", "Blue", "Yellow"};
#define NUM_TORCH_COLORS 5

bool stopwatchRunning = false;
unsigned long stopwatchStartMs = 0;
unsigned long stopwatchElapsedMs = 0;

#define MAX_LAPS 10
unsigned long lapTimes[MAX_LAPS];
int lapCount = 0;

double calcValue = 0;
double calcOperand = 0;
double calcFirstNum = 0;
char calcOperator = ' ';
bool calcNewNumber = true;
char calcDisplay[16] = "0";
double calcMemory = 0;  // Memory storage
String calcHistory[5] = {"", "", "", "", ""};  // Last 5 calculations
int calcHistoryCount = 0;

// Tally Counter enhanced state
int tallyCount = 0;
int tallyCounters[4] = {0, 0, 0, 0};  // 4 separate counters
int currentTallyCounter = 0;
const char* tallyNames[4] = {"COUNT 1", "COUNT 2", "COUNT 3", "COUNT 4"};
int tallyGoals[4] = {0, 0, 0, 0};  // Optional goals

// Voice Memo enhanced state
String voiceMemoList[20];  // List of memo filenames
int voiceMemoListCount = 0;
int selectedMemoIndex = -1;
bool voiceMemoPlaying = false;

// TXT Files enhanced state
int txtFontSize = 14;  // Adjustable font size (12, 14, 16, 18)
int txtScrollPosition = 0;
String txtBookmarks[5];  // Bookmarked files
int txtBookmarkCount = 0;

// Blackjack game state
int playerCards[10], dealerCards[10];
int playerCount = 0, dealerCount = 0;
int blackjackBet = 100;
bool blackjackGameActive = false;
bool playerStand = false;

uint32_t clickerCount = 0;

// Tilt Maze game state
float mazeBallX = 205.0f, mazeBallY = 251.0f;  // Ball position (center of screen)
float mazeBallVelX = 0.0f, mazeBallVelY = 0.0f;  // Ball velocity
const int MAZE_BALL_RADIUS = 8;
const float MAZE_FRICTION = 0.95f;
const float MAZE_TILT_SENSITIVITY = 0.8f;
bool mazeGameActive = false;
bool mazeGameWon = false;
unsigned long mazeStartTime = 0;
unsigned long mazeBestTime = 0;

// Maze walls: {x, y, width, height}
const int MAZE_WALL_COUNT = 12;
int mazeWalls[MAZE_WALL_COUNT][4] = {
    {0, 100, 280, 10},      // Top horizontal
    {130, 100, 10, 100},    // Top-left vertical
    {280, 100, 10, 150},    // Top-right vertical
    {0, 200, 180, 10},      // Mid-left horizontal
    {230, 200, 180, 10},    // Mid-right horizontal
    {70, 200, 10, 100},     // Mid-left vertical
    {330, 200, 10, 100},    // Mid-right vertical
    {70, 300, 120, 10},     // Lower-left horizontal
    {220, 300, 120, 10},    // Lower-right horizontal
    {180, 250, 10, 60},     // Center vertical
    {0, 400, 150, 10},      // Bottom-left horizontal
    {260, 400, 150, 10}     // Bottom-right horizontal
};

// Maze start and goal positions
const int MAZE_START_X = 50, MAZE_START_Y = 150;
const int MAZE_GOAL_X = 360, MAZE_GOAL_Y = 450;
const int MAZE_GOAL_SIZE = 30;

// Pong game state - IMPROVED
float pongBallX = 205.0f, pongBallY = 251.0f;
float pongBallVelX = 5.0f, pongBallVelY = 3.0f;  // Faster initial speed
float pongPaddleY = 200.0f;       // Player paddle Y position
float pongCpuPaddleY = 200.0f;    // CPU paddle Y position
int pongPlayerScore = 0;
int pongCpuScore = 0;
bool pongGameActive = false;
const int PONG_PADDLE_WIDTH = 10;
const int PONG_PADDLE_HEIGHT = 60;
const int PONG_BALL_SIZE = 12;
const float PONG_CPU_SPEED = 7.0f;           // FASTER AI paddle (improved)
const float PONG_BALL_SPEED_INCREASE = 0.5f;  // FASTER ball speed increase
unsigned long lastPongUpdate = 0;

// Tic-Tac-Toe game state
int tttBoard[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // 0=empty, 1=X(player), 2=O(AI)
bool tttPlayerTurn = true;
bool tttGameOver = false;
int tttWinner = 0;  // 0=none/draw, 1=player, 2=AI
int tttPlayerWins = 0;
int tttAiWins = 0;

// 2048 game state
int grid2048[4][4] = {{0}};
int score2048 = 0;
int bestScore2048 = 0;
bool game2048Over = false;
bool game2048Won = false;

// Wordle game state
char wordleTarget[6] = "WATCH";  // Current target word
char wordleGuesses[6][6];        // Up to 6 guesses of 5 letters
int wordleCurrentRow = 0;
int wordleCurrentCol = 0;
bool wordleGameOver = false;
bool wordleWon = false;
int wordleGamesWon = 0;
int wordleGamesPlayed = 0;
String wordleWordList[1000];     // Word list from SD
int wordleWordCount = 0;
bool wordleWordsLoaded = false;

// Breakout game state - ENHANCED with 5 levels
float breakoutPaddleX = 180.0f;
float breakoutBallX = 205.0f, breakoutBallY = 400.0f;
float breakoutBallVelX = 5.0f, breakoutBallVelY = -7.0f;  // DOUBLED speed
bool breakoutBricks[6][10];       // 6 rows x 10 cols of bricks (more bricks)
int breakoutScore = 0;
int breakoutLives = 3;
bool breakoutGameActive = false;
bool breakoutGameOver = false;
bool breakoutLevelComplete = false;  // NEW: Track level completion
bool breakoutShowNextLevel = false;  // NEW: Show "Next Level?" prompt
int breakoutCurrentLevel = 1;        // NEW: Current level (1-5)
RTC_DATA_ATTR int breakoutSavedLevel = 1;  // NEW: Persist level across restarts
RTC_DATA_ATTR int breakoutSavedScore = 0;  // NEW: Persist score across restarts
const int BREAKOUT_PADDLE_WIDTH = 60;
const int BREAKOUT_PADDLE_HEIGHT = 10;
const int BREAKOUT_BALL_SIZE = 10;
const int BREAKOUT_BRICK_WIDTH = 36;   // Smaller bricks for more columns
const int BREAKOUT_BRICK_HEIGHT = 12;
const int BREAKOUT_MAX_LEVELS = 5;     // NEW: 5 levels total
const float BREAKOUT_BASE_SPEED = 5.0f;  // NEW: Base ball speed (was 2.5f)

// Tetris game state
int tetrisBoard[20][10] = {{0}};  // 20 rows x 10 cols
int tetrisCurrentPiece[4][4];
int tetrisCurrentX = 3, tetrisCurrentY = 0;
int tetrisCurrentType = 0;
int tetrisScore = 0;
int tetrisLevel = 1;
int tetrisLines = 0;
bool tetrisGameActive = false;
bool tetrisGameOver = false;
unsigned long tetrisLastDrop = 0;
int tetrisDropSpeed = 500;  // ms between drops

int32_t touchStartX = 0, touchStartY = 0;
int32_t touchCurrentX = 0, touchCurrentY = 0;
bool touchActive = false;
unsigned long touchStartMs = 0;
volatile bool touchInterruptFlag = false;

const int SWIPE_THRESHOLD_MIN = 25;
const int TAP_THRESHOLD = 20;
const unsigned long SWIPE_MAX_DURATION = 800;

unsigned long lastClockUpdate = 0;
unsigned long lastStepUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastSaveTime = 0;
unsigned long lastNTPSync = 0;
unsigned long lastWeatherUpdate = 0;
bool ntpSyncedOnce = false;

// Time format
bool use24HourFormat = true;

// Sunrise/Sunset
float sunriseHour = 5.5;
float sunsetHour = 19.0;

// Low battery warning
bool lowBatteryWarningShown = false;
bool criticalBatteryWarningShown = false;
unsigned long lowBatteryPopupTime = 0;
bool showingLowBatteryPopup = false;

// Charging animation
uint8_t chargingAnimFrame = 0;
unsigned long lastChargingAnimMs = 0;

// TXT file reader
#define SD_SAMPLE_TXT "/WATCH/TXT/sample.txt"
#define MAX_TXT_FILES 20
#define TXT_FILE_MAX_SIZE (64 * 1024)

struct TxtFileInfo {
    char filename[64];
    uint32_t fileSize;
};

TxtFileInfo txtFiles[MAX_TXT_FILES];
int numTxtFiles = 0;
int selectedTxtFile = -1;
char* txtFileContent = NULL;
int txtScrollOffset = 0;
int txtFileListOffset = 0;
bool txtFileLoaded = false;

// WiFi Manager
#define MAX_SCANNED_NETWORKS 10

struct ScannedNetwork {
    char ssid[64];
    int32_t rssi;
    bool isOpen;
    bool isConnected;
};

ScannedNetwork scannedNetworks[MAX_SCANNED_NETWORKS];
int numScannedNetworks = 0;
bool wifiScanComplete = false;
int wifiCardScrollOffset = 0;

const char* hardcodedSSID = "Optus_9D2E3D";
const char* hardcodedPass = "snucktemptGLeQU";

// ═══════════════════════════════════════════════════════════════════════════
// THEME ARRAY
// ═══════════════════════════════════════════════════════════════════════════
GradientTheme gradientThemes[NUM_THEMES] = {
  {"Midnight", lv_color_hex(0x1C1C1E), lv_color_hex(0x2C2C2E), lv_color_hex(0xFFFFFF), lv_color_hex(0x0A84FF), lv_color_hex(0x5E5CE6)},
  {"Ocean", lv_color_hex(0x0D3B66), lv_color_hex(0x1A759F), lv_color_hex(0xFFFFFF), lv_color_hex(0x52B2CF), lv_color_hex(0x99E1D9)},
  {"Sunset", lv_color_hex(0xFF6B35), lv_color_hex(0xF7931E), lv_color_hex(0xFFFFFF), lv_color_hex(0xFFD166), lv_color_hex(0xFFA62F)},
  {"Aurora", lv_color_hex(0x7B2CBF), lv_color_hex(0x9D4EDD), lv_color_hex(0xFFFFFF), lv_color_hex(0xC77DFF), lv_color_hex(0xE0AAFF)},
  {"Forest", lv_color_hex(0x1B4332), lv_color_hex(0x2D6A4F), lv_color_hex(0xFFFFFF), lv_color_hex(0x52B788), lv_color_hex(0x95D5B2)},
  {"Ruby", lv_color_hex(0x9B2335), lv_color_hex(0xC41E3A), lv_color_hex(0xFFFFFF), lv_color_hex(0xFF6B6B), lv_color_hex(0xFFA07A)},
  {"Graphite", lv_color_hex(0x1C1C1E), lv_color_hex(0x3A3A3C), lv_color_hex(0xFFFFFF), lv_color_hex(0x8E8E93), lv_color_hex(0xAEAEB2)},
  {"Mint", lv_color_hex(0x00A896), lv_color_hex(0x02C39A), lv_color_hex(0x1C1C1E), lv_color_hex(0x00F5D4), lv_color_hex(0xB5FFE1)}
};

// ═══════════════════════════════════════════════════════════════════════════
// SAFE THEME GETTER
// ═══════════════════════════════════════════════════════════════════════════
int getSafeThemeIndex() {
    int idx = userData.themeIndex;
    if (idx < 0 || idx >= NUM_THEMES) {
        idx = 0;
        userData.themeIndex = 0;
    }
    return idx;
}

GradientTheme* getSafeTheme() {
    return &gradientThemes[getSafeThemeIndex()];
}

// ═══════════════════════════════════════════════════════════════════════════
// INTERRUPT SERVICE ROUTINES - FROM v7.3
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Touch controller interrupt handler
 * Sets flag for main loop - no I2C operations in ISR!
 */
void IRAM_ATTR Arduino_IIC_Touch_Interrupt(void) {
    touchInterruptFlag = true;
    touchWakeFlag = true;
    lastActivityMs = millis();
}

/**
 * IMU motion/step interrupt handler
 */
void IRAM_ATTR imuInterruptHandler(void) {
    stepDetectedFlag = true;
    imuWakeFlag = true;
    stepCounterISR++;
}

/**
 * Button interrupt handler
 */
void IRAM_ATTR buttonInterruptHandler(void) {
    buttonWakeFlag = true;
    lastActivityMs = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// I2C BUS RECOVERY - CRITICAL FOR WAKE FROM LIGHT SLEEP
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Recovers I2C bus after light sleep
 * This is the key fix for the I2C reinitialization issue!
 */
bool recoverI2CBus() {
    USBSerial.println("[I2C] Recovering bus after sleep...");
    
    // Step 1: End current Wire instance
    Wire.end();
    delay(10);
    
    // Step 2: Manually toggle SCL to clear any stuck slaves
    pinMode(IIC_SCL, OUTPUT);
    pinMode(IIC_SDA, INPUT_PULLUP);
    
    for (int i = 0; i < 9; i++) {
        digitalWrite(IIC_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(IIC_SCL, HIGH);
        delayMicroseconds(5);
    }
    
    // Step 3: Generate STOP condition
    pinMode(IIC_SDA, OUTPUT);
    digitalWrite(IIC_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(IIC_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(IIC_SDA, HIGH);
    delayMicroseconds(5);
    
    // Step 4: Reinitialize Wire
    Wire.begin(IIC_SDA, IIC_SCL);
    Wire.setClock(400000);  // 400kHz I2C
    delay(10);
    
    // Step 5: Verify devices respond
    bool rtcOk = false, pmuOk = false, touchOk = false;
    
    Wire.beginTransmission(0x51);  // PCF85063 RTC address
    rtcOk = (Wire.endTransmission() == 0);
    
    Wire.beginTransmission(0x34);  // AXP2101 PMU address
    pmuOk = (Wire.endTransmission() == 0);
    
    Wire.beginTransmission(FT3168_DEVICE_ADDRESS);
    touchOk = (Wire.endTransmission() == 0);
    
    USBSerial.printf("[I2C] Recovery result - RTC:%d PMU:%d Touch:%d\n", rtcOk, pmuOk, touchOk);
    
    // Reinitialize touch controller
    if (touchOk) {
        // Reinitialize touch controller via I2C reset
        Wire.beginTransmission(FT3168_DEVICE_ADDRESS);
        Wire.endTransmission();
        delay(50);
        delay(20);
    }
    
    return rtcOk && pmuOk;
}

// ═══════════════════════════════════════════════════════════════════════════
// LIGHT SLEEP MANAGEMENT - FROM v7.3
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Check if light sleep should be enabled
 * Light sleep is enabled in EXTREME mode regardless of battery level
 */
bool shouldEnableLightSleep() {
    return (batterySaverLevel == BATTERY_SAVER_EXTREME) && !isCharging;
}

/**
 * Configures wake sources for light sleep
 * - Touch GPIO wake (for touch screen wake)
 * - RTC Timer wake (for periodic step sync)
 * - RTC always runs during light sleep (per ESP-IDF docs)
 */
void configureSleepWakeSources() {
    // Keep RTC peripherals powered for touch wake and RTC timer
    // This ensures RTC always works during sleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    
    // Touch interrupt as wake source using GPIO wakeup (light-sleep compatible)
    // GPIO wakeup works for any IO in light sleep, more reliable than ext0
    gpio_wakeup_enable((gpio_num_t)TP_INT, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    
    // RTC Timer wake for periodic step sync every 5 MINUTES
    // RTC timer always runs during light sleep (per ESP-IDF: "RTC controller has built-in timer")
    esp_sleep_enable_timer_wakeup(300 * 1000000ULL);  // 5 minutes = 300 seconds
    
    USBSerial.println("[SLEEP] Wake sources: GPIO touch, RTC timer (5min)");
    USBSerial.println("[SLEEP] RTC peripherals kept ON during sleep");
}

/**
 * Prepares system for light sleep
 * - Watchdog handled properly (removed from current task before sleep)
 * - RTC continues running during sleep
 * - Touch wake enabled via GPIO
 */
void prepareForSleep() {
    USBSerial.println("[SLEEP] Preparing for light sleep...");
    
    // CRITICAL: Save time to RTC memory before sleep (for watchdog recovery)
    saveRtcTimeToMemory();
    
    // IMPORTANT: Prepare watchdog before sleep
    prepareWatchdogForSleep();
    
    // 1. Stop LVGL task processing
    lvglTaskRunning = false;
    
    // 2. Turn off display completely
    gfx->setBrightness(0);
    gfx->displayOff();
    
    // 3. Disable WiFi if active
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
    
    // 4. Configure IMU for low power with pedometer still active
    if (hasIMU) {
        // Keep pedometer running during light sleep
        qmi.configPedometer(
            0x007D, 0x00CC, 0x0066, 0x0050,
            0x14, 0x0A, 0x00, 0x04
        );
    }
    
    // 5. Lower CPU frequency for sleep entry
    setCpuFrequencyMhz(CPU_FREQ_SLEEP);
    
    // 6. Configure wake sources (touch GPIO + RTC timer)
    configureSleepWakeSources();
    
    // 7. Clear any pending interrupts
    touchWakeFlag = false;
    imuWakeFlag = false;
    buttonWakeFlag = false;
    
    USBSerial.println("[SLEEP] Ready for light sleep (RTC always running)");
    USBSerial.flush();
    delay(10);
}

/**
 * Enters light sleep mode
 * CPU pauses, RAM retained, RTC continues
 */
void enterLightSleep() {
    if (!shouldEnableLightSleep()) {
        USBSerial.println("[SLEEP] Light sleep not enabled - conditions not met");
        return;
    }
    
    previousPowerState = currentPowerState;
    currentPowerState = POWER_STATE_LIGHT_SLEEP;
    
    // Track sleep start time for stats
    unsigned long sleepStartMs = millis();
    
    prepareForSleep();
    
    // Enter light sleep - CPU will pause here
    esp_light_sleep_start();
    
    // ═══ WAKE UP POINT ═══
    // Execution continues here after wake event
    
    // Track sleep duration
    batteryStats.lightSleepTimeMs += (millis() - sleepStartMs);
    
    wakeFromSleep();
}

/**
 * Handles wake up from light sleep
 * - Re-adds task to watchdog immediately on wake
 * - RTC time is accurate (RTC ran during sleep)
 */
void wakeFromSleep() {
    currentPowerState = POWER_STATE_WAKING;
    uint32_t wakeStart = millis();
    
    // CRITICAL: Re-add task to watchdog immediately on wake
    restoreWatchdogAfterWake();
    
    // 1. Restore CPU frequency immediately
    setCpuFrequencyMhz(CPU_FREQ_SAVER);  // Start at saver freq, can increase
    
    // Feed watchdog after CPU restore
    esp_task_wdt_reset();
    
    // 2. Determine wake cause
    esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
    const char* wakeCauseStr = "Unknown";
    switch(wakeupCause) {
        case ESP_SLEEP_WAKEUP_TIMER: wakeCauseStr = "RTC Timer"; break;
        case ESP_SLEEP_WAKEUP_GPIO: wakeCauseStr = "GPIO/Touch"; break;
        case ESP_SLEEP_WAKEUP_EXT0: wakeCauseStr = "EXT0"; break;
        case ESP_SLEEP_WAKEUP_EXT1: wakeCauseStr = "EXT1"; break;
        default: break;
    }
    USBSerial.printf("[WAKE] Cause: %s (%d)\n", wakeCauseStr, wakeupCause);
    
    // Disable GPIO wakeup after wake (will be re-enabled next sleep)
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    
    // 3. CRITICAL: Recover I2C bus
    if (!recoverI2CBus()) {
        USBSerial.println("[WAKE] I2C recovery failed! Attempting full reset...");
        Wire.end();
        delay(50);
        Wire.begin(IIC_SDA, IIC_SCL);
        Wire.setClock(400000);
        delay(50);
    }
    
    // FIX: Feed watchdog after I2C recovery
    esp_task_wdt_reset();
    
    // 4. Read RTC time (RTC ran continuously during sleep - time is accurate)
    if (hasRTC) {
        RTC_DateTime dt = rtc.getDateTime();
        clockHour = dt.getHour();
        clockMinute = dt.getMinute();
        clockSecond = dt.getSecond();
        USBSerial.printf("[WAKE] RTC time (accurate): %02d:%02d:%02d\n", clockHour, clockMinute, clockSecond);
    }
    
    // 5. Read step count from IMU (accumulated during sleep)
    if (hasIMU) {
        uint32_t imuSteps = qmi.getPedometerCounter();
        if (imuSteps > userData.steps) {
            userData.steps = imuSteps;
        }
    }
    
    // 6. Read battery status
    if (hasPMU) {
        batteryPercent = power.getBatteryPercent();
        batteryVoltage = power.getBattVoltage();
        isCharging = power.isCharging();
    }
    
    // FIX: Feed watchdog before display init
    esp_task_wdt_reset();
    
    // 7. Reconfigure touch GPIO for normal operation (was configured for wake)
    gpio_wakeup_disable((gpio_num_t)TP_INT);
    
    // 8. Turn on display
    gfx->displayOn();
    gfx->setBrightness(saverModes[batterySaverLevel].brightness);
    
    // 9. Resume LVGL
    lvglTaskRunning = true;
    
    // 10. Update activity timestamp
    lastActivityMs = millis();
    
    // 11. Transition to active state
    currentPowerState = POWER_STATE_ACTIVE;
    screenOn = true;
    ui_event = UI_EVENT_WAKE_UP;
    
    // Final watchdog reset after full wake
    esp_task_wdt_reset();
    
    uint32_t wakeTime = millis() - wakeStart;
    USBSerial.printf("[WAKE] Complete in %lu ms\n", wakeTime);
}

// ═══════════════════════════════════════════════════════════════════════════
// STEP COUNTING - IMU PEDOMETER FROM v7.3
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Initialize IMU for low-power step counting
 */
void initStepCounter() {
    if (!hasIMU) return;
    
    // Enable accelerometer at low ODR for power saving
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_LOWPOWER_21Hz
    );
    
    // Enable built-in hardware pedometer
    qmi.enablePedometer();
    
    /**
     * Configure QMI8658 hardware pedometer
     * Based on Waveshare SensorQMI8658 library
     * 
     * Parameters (from library):
     * @param ped_sample_cnt: Sample batch/window for calculation (default 0x007D = 125)
     * @param ped_fix_peak2peak: Peak-to-peak threshold (0x00CC = 200mg recommended)
     * @param ped_fix_peak: Peak detection threshold vs average (0x0066 = 100mg)
     * @param ped_time_up: Max duration for a step - timeout (80 = 1.6s @ 50Hz)
     * @param ped_time_low: Min duration for a step - quiet time (12 = 0.24s @ 50Hz)
     * @param ped_time_cnt_entry: Min continuous steps before counting starts (10)
     * @param ped_fix_precision: Precision (0 recommended)
     * @param ped_sig_count: Steps between register updates (4 = update every 4 steps)
     */
    qmi.configPedometer(
        0x007D,     // ped_sample_cnt: 125 samples batch
        0x00CC,     // ped_fix_peak2peak: 200mg threshold for valid peak-to-peak
        0x0066,     // ped_fix_peak: 100mg threshold comparing to average
        0x0050,     // ped_time_up: 80 = 1.6s timeout @ 50Hz ODR
        0x14,       // ped_time_low: 20 = 0.4s quiet time @ 50Hz ODR
        0x0A,       // ped_time_cnt_entry: 10 steps before counting starts
        0x00,       // ped_fix_precision: 0 recommended
        0x01        // ped_sig_count: Update register every 1 step
    );
    
    USBSerial.println("[STEP] QMI8658 hardware pedometer configured");
    USBSerial.println("[STEP] - Peak threshold: 200mg, Min steps: 10");
}

/**
 * Read step count from IMU hardware counter
 */
void updateStepCountFromIMU() {
    if (!hasIMU) return;
    
    uint32_t hwSteps = qmi.getPedometerCounter();
    
    if (hwSteps > userData.steps) {
        uint32_t newSteps = hwSteps - userData.steps;
        userData.steps = hwSteps;
        
        // Update derived metrics
        userData.totalDistance = userData.steps * 0.0007;
        userData.totalCalories = userData.steps * 0.04;
        
        USBSerial.printf("[STEP] Count: %lu (+%lu)\n", userData.steps, newSteps);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY ESTIMATION - ENHANCED WITH LIGHT SLEEP
// ═══════════════════════════════════════════════════════════════════════════

void calculateBatteryEstimates() {
    float remainingMAh = (batteryPercent / 100.0) * BATTERY_CAPACITY_MAH;
    
    // Calculate current draw based on screen state and power state
    float avgCurrentMA;
    if (currentPowerState == POWER_STATE_LIGHT_SLEEP) {
        avgCurrentMA = LIGHT_SLEEP_CURRENT_MA;
    } else if (screenOn) {
        avgCurrentMA = batterySaverMode ? SAVER_MODE_CURRENT_MA : SCREEN_ON_CURRENT_MA;
    } else {
        avgCurrentMA = SCREEN_OFF_CURRENT_MA;
    }

    // Simple estimate
    batteryStats.simpleEstimateMins = (uint32_t)((remainingMAh / avgCurrentMA) * 60.0);

    // Weighted estimate using usage patterns
    uint32_t totalScreenOnMs = batteryStats.screenOnTimeMs;
    uint32_t totalScreenOffMs = batteryStats.screenOffTimeMs;
    uint32_t totalLightSleepMs = batteryStats.lightSleepTimeMs;
    uint32_t totalSessionMs = totalScreenOnMs + totalScreenOffMs + totalLightSleepMs;

    if (totalSessionMs > 0) {
        float screenOnRatio = (float)totalScreenOnMs / (float)totalSessionMs;
        float screenOffRatio = (float)totalScreenOffMs / (float)totalSessionMs;
        float sleepRatio = (float)totalLightSleepMs / (float)totalSessionMs;

        float weightedCurrentMA = (screenOnRatio * SCREEN_ON_CURRENT_MA) +
                                  (screenOffRatio * SCREEN_OFF_CURRENT_MA) +
                                  (sleepRatio * LIGHT_SLEEP_CURRENT_MA);

        if (batterySaverMode) {
            weightedCurrentMA *= 0.7;
        }

        batteryStats.weightedEstimateMins = (uint32_t)((remainingMAh / weightedCurrentMA) * 60.0);
        batteryStats.weightedDrainRate = weightedCurrentMA;
    } else {
        batteryStats.weightedEstimateMins = batteryStats.simpleEstimateMins;
    }

    // Learned estimate
    float avgDailyDrain = 0;
    int validDays = 0;

    for (int i = 0; i < 7; i++) {
        if (batteryStats.dailyAvgDrainRate[i] > 0) {
            avgDailyDrain += batteryStats.dailyAvgDrainRate[i];
            validDays++;
        }
    }

    if (validDays > 0) {
        avgDailyDrain /= validDays;
        batteryStats.avgDrainPerHour = avgDailyDrain;

        if (avgDailyDrain > 0) {
            float hoursRemaining = (remainingMAh / avgDailyDrain);
            batteryStats.learnedEstimateMins = (uint32_t)(hoursRemaining * 60.0);
        } else {
            batteryStats.learnedEstimateMins = batteryStats.weightedEstimateMins;
        }
    } else {
        batteryStats.learnedEstimateMins = batteryStats.weightedEstimateMins;
    }

    // Combined estimate
    batteryStats.combinedEstimateMins = (uint32_t)(
        (batteryStats.simpleEstimateMins * 0.2) +
        (batteryStats.weightedEstimateMins * 0.4) +
        (batteryStats.learnedEstimateMins * 0.4)
    );

    if (batteryStats.combinedEstimateMins > 2880) {
        batteryStats.combinedEstimateMins = 2880;
    }
}

void toggleBatterySaver() {
    batterySaverMode = !batterySaverMode;

    if (batterySaverMode) {
        gfx->setBrightness(50);
        USBSerial.println("[BATTERY] Saver mode ON - Brightness reduced to 50");
    } else {
        gfx->setBrightness(userData.brightness);
        USBSerial.println("[BATTERY] Saver mode OFF - Brightness restored");
    }
    
    prefs.begin("minios", false);
    prefs.putBool("batterySaver", batterySaverMode);
    prefs.end();
}

// Forward declaration
void logModeChange(BatterySaverLevel oldMode, BatterySaverLevel newMode);

// ═══════════════════════════════════════════════════════════════════════════
// APPLY BATTERY SAVER MODE - 3 MODES: OFF, MEDIUM, EXTREME
// ═══════════════════════════════════════════════════════════════════════════
void applyBatterySaverMode(BatterySaverLevel level) {
    BatterySaverLevel oldLevel = batterySaverLevel;
    
    batterySaverLevel = level;
    batterySaverMode = (level != BATTERY_SAVER_OFF);
    
    BatterySaverSettings &mode = saverModes[level];
    
    // Apply brightness
    gfx->setBrightness(mode.brightness);
    
    // Apply CPU frequency
    setCpuFrequencyMhz(mode.cpuFreqMhz);
    
    // Apply sensor poll interval based on mode
    switch (level) {
        case BATTERY_SAVER_OFF:
            sensorPollInterval = SENSOR_POLL_NORMAL_MS;
            lightSleepEnabled = false;
            break;
        case BATTERY_SAVER_MEDIUM:
            sensorPollInterval = SENSOR_POLL_SAVER_MS;
            lightSleepEnabled = false;
            break;
        case BATTERY_SAVER_EXTREME:
            sensorPollInterval = 500;
            lightSleepEnabled = shouldEnableLightSleep() && mode.enableLightSleep;
            if (lightSleepEnabled) {
                USBSerial.println("[POWER] Light sleep ENABLED (Extreme mode)");
            }
            break;
    }
    
    // Handle BLE auto-off
    if (mode.bleAutoOff && bleEnabled && !bleDeviceConnected) {
        // Will auto-off via timer, no immediate action needed
        USBSerial.println("[POWER] BLE auto-off enabled for this mode");
    }
    
    // Disable WiFi in extreme mode
    if (mode.disableWifiSync) {
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.disconnect();
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;
            USBSerial.println("[POWER] WiFi disabled to save battery");
        }
    }
    
    // Log mode change
    if (oldLevel != level) {
        logModeChange(oldLevel, level);
    }
    
    // Save preference
    prefs.begin("minios", false);
    prefs.putUChar("saverLevel", (uint8_t)level);
    prefs.end();
    
    USBSerial.printf("[POWER] Mode: %s | CPU: %dMHz | Bright: %d | Timeout: %ds | Sleep: %s\n", 
        mode.name, getCpuFrequencyMhz(), mode.brightness, 
        mode.screenTimeoutMs / 1000, lightSleepEnabled ? "ON" : "off");
}

void logModeChange(BatterySaverLevel oldMode, BatterySaverLevel newMode) {
    currentPowerSession.modeChanges++;
    USBSerial.printf("[POWER] Mode change: %s -> %s\n", 
        saverModes[oldMode].name, saverModes[newMode].name);
}

// Forward declarations for deep sleep
void saveUserData();
void stopBLE();

// ═══════════════════════════════════════════════════════════════════════════
// IMPROVED SLEEP SYSTEM - Watchdog Safe, RTC Persistent, Multi-wake Sources
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Save current RTC time to RTC memory before any sleep
 * This ensures time is preserved across watchdog resets and sleep cycles
 */
void saveRtcTimeToMemory() {
    if (hasRTC) {
        RTC_DateTime dt = rtc.getDateTime();
        savedRtcTime.hours = dt.getHour();
        savedRtcTime.minutes = dt.getMinute();
        savedRtcTime.seconds = dt.getSecond();
        savedRtcTime.day = dt.getDay();
        savedRtcTime.month = dt.getMonth();
        savedRtcTime.year = dt.getYear();
        savedRtcTime.valid = true;
        USBSerial.printf("[TIME] Saved to RTC memory: %02d:%02d:%02d %02d/%02d/%04d\n", 
                        savedRtcTime.hours, savedRtcTime.minutes, savedRtcTime.seconds,
                        savedRtcTime.day, savedRtcTime.month, savedRtcTime.year);
    }
}

/**
 * Restore time from RTC memory after wake/reset
 * Call this after hardware RTC initialization
 */
void restoreRtcTimeFromMemory() {
    if (savedRtcTime.valid && hasRTC) {
        // Check if saved time is reasonable
        if (savedRtcTime.year >= 2024 && savedRtcTime.year <= 2100) {
            USBSerial.printf("[TIME] Restoring from RTC memory: %02d:%02d:%02d\n",
                            savedRtcTime.hours, savedRtcTime.minutes, savedRtcTime.seconds);
            // The hardware RTC should maintain time, but we use this as backup validation
        }
    }
}

/**
 * Prepare watchdog for sleep - must be called before ANY sleep
 * ESP-IDF: Watchdog keeps running during light sleep but task is removed
 * For deep sleep: Device resets, so watchdog is reinitialized on boot
 */
void prepareWatchdogForSleep() {
    // Feed watchdog first
    esp_task_wdt_reset();
    
    // Remove current task from watchdog monitoring
    // This prevents watchdog timeout during sleep
    esp_err_t err = esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
    if (err == ESP_OK) {
        USBSerial.println("[WDT] Task removed from watchdog for sleep");
    } else {
        USBSerial.printf("[WDT] Warning: Could not remove task (%d)\n", err);
    }
}

/**
 * Restore watchdog after wake from light sleep
 */
void restoreWatchdogAfterWake() {
    // Re-add current task to watchdog
    esp_err_t err = esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    if (err == ESP_OK || err == ESP_ERR_INVALID_ARG) {  // INVALID_ARG means already added
        esp_task_wdt_reset();
        USBSerial.println("[WDT] Task restored to watchdog after wake");
    } else {
        USBSerial.printf("[WDT] Warning: Could not restore task (%d)\n", err);
    }
}

/**
 * Configure optimal power domains for ULTRA LOW POWER deep sleep
 * Target: ~10 µA (RTC timer + RTC memory only)
 * 
 * IMPORTANT: External PCF85063 RTC chip (I2C address 0x51) runs continuously
 * on its own power/battery - it is NOT affected by ESP32 sleep modes!
 * The RTC maintains accurate time through:
 * - All ESP32 sleep modes (light sleep, deep sleep)
 * - Watchdog resets
 * - Power cycles (if RTC has battery backup)
 * 
 * Per ESP32 power table:
 * - RTC timer + RTC memory = 10 µA
 * - ULP active = 150 µA  
 * - ULP sensor = 100 µA
 */
void configureDeepSleepPowerDomains() {
    // ══════════════════════════════════════════════════════════════════════
    // ULTRA LOW POWER CONFIG - Target 10 µA
    // ══════════════════════════════════════════════════════════════════════
    // 
    // NOTE: External RTC (PCF85063) is powered separately and ALWAYS runs!
    // It connects via I2C which we re-initialize on wake.
    // ══════════════════════════════════════════════════════════════════════
    
    // Turn OFF RTC peripherals - use HOLD feature instead for GPIO states
    // This is key to achieving 10 µA vs 150+ µA
    // NOTE: This is ESP32's internal RTC peripheral, NOT the external PCF85063!
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    
    // Keep RTC slow memory ON for RTC_DATA_ATTR variables (minimal power impact)
    // This preserves our savedRtcTime backup and other persistent variables
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST,   ESP_PD_OPTION_OFF);
    
    // Turn OFF RTC fast memory - not needed for our wake stub
   
   
   
    
    // Turn OFF XTAL during deep sleep (uses internal RC oscillator)
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    
    // Turn OFF internal 8MHz oscillator if not needed
    esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);
    
    // Turn OFF VDD_SDIO domain (flash power) - we don't need flash in deep sleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    
    USBSerial.println("[POWER] Ultra-low power domains configured (target: ~10 µA)");
    USBSerial.println("[POWER] External RTC (PCF85063) runs independently - time preserved!");
}

/**
 * Isolate unused GPIOs to prevent leakage current during deep sleep
 * Critical for achieving minimum power consumption
 * 
 * On ESP32-S3, unconnected or floating GPIOs can draw extra current
 * Isolating them disconnects internal pullups/pulldowns
 */
void isolateUnusedGPIOs() {
    USBSerial.println("[GPIO] Isolating unused pins for minimum leakage...");
    
    // List of GPIOs we're actively using (DO NOT isolate these):
    // PWR_BUTTON (GPIO10), TP_INT (GPIO38), IIC_SDA, IIC_SCL
    // LCD pins, SD card pins, IMU pins, etc.
    
    // For RTC-capable GPIOs that might have external pullups causing current:
    // Isolate them to disconnect internal resistors
    
    // Common GPIOs to isolate on ESP32-S3 if not used:
    // Note: Only isolate pins you're 100% sure are unused
    // Be careful not to isolate pins connected to external components
    
    // Example: If GPIO12 has an external pullup and internal pulldown
    // rtc_gpio_isolate(GPIO_NUM_12);
    
    // For this watch, we'll be conservative and only isolate known unused RTC GPIOs
    // The main power savings come from the power domain configuration
    
    // Disable ROM logging to save a bit more power
    esp_deep_sleep_disable_rom_logging();
    
    USBSerial.println("[GPIO] ROM logging disabled for deep sleep");
}

/**
 * Configure wake sources for ULTRA LOW POWER deep sleep
 * Uses HOLD feature instead of keeping RTC_PERIPH powered
 * GPIO10 (PWR button) + GPIO38 (Touch) via EXT1
 */
void configureDeepSleepWakeSources() {
    // Create bitmask for wake GPIOs
    uint64_t wakeup_pin_mask = (1ULL << PWR_BUTTON) | (1ULL << TP_INT);
    
    // ══════════════════════════════════════════════════════════════════════
    // Configure GPIOs with HOLD feature (works with RTC_PERIPH OFF)
    // The HOLD feature maintains pullup/pulldown state during sleep
    // This allows 10 µA vs 150+ µA with RTC_PERIPH ON
    // ══════════════════════════════════════════════════════════════════════
    
    // Configure PWR_BUTTON (GPIO10) with pullup, then hold
    gpio_reset_pin((gpio_num_t)PWR_BUTTON);
    gpio_set_direction((gpio_num_t)PWR_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)PWR_BUTTON);
    gpio_pulldown_dis((gpio_num_t)PWR_BUTTON);
    gpio_hold_en((gpio_num_t)PWR_BUTTON);  // HOLD maintains state with RTC_PERIPH off
    
    // Configure TP_INT (GPIO38) with pullup, then hold
    gpio_reset_pin((gpio_num_t)TP_INT);
    gpio_set_direction((gpio_num_t)TP_INT, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)TP_INT);
    gpio_pulldown_dis((gpio_num_t)TP_INT);
    gpio_hold_en((gpio_num_t)TP_INT);  // HOLD maintains state with RTC_PERIPH off
    
    // Enable EXT1 wake on ANY_LOW (wake when either button pressed or touch detected)
    // EXT1 works even with RTC_PERIPH OFF - it uses RTC controller logic
    esp_err_t err = esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW);
    
    if (err != ESP_OK) {
        USBSerial.printf("[WAKE] WARNING: EXT1 config failed (%d), using EXT0 fallback\n", err);
        // Fallback: Use EXT0 for just the power button (requires RTC_PERIPH ON)
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PWR_BUTTON, 0);
    } else {
        USBSerial.println("[WAKE] EXT1 configured: GPIO10 (PWR) + GPIO38 (Touch) on LOW");
        USBSerial.println("[WAKE] Using HOLD feature - RTC_PERIPH can stay OFF");
    }
    
    // Timer wake as backup (wake every 30 minutes to sync time if needed)
    // RTC timer uses negligible power
    esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);  // 30 minutes
    USBSerial.println("[WAKE] Timer backup: 30 minute periodic wake");
}

// ═══════════════════════════════════════════════════════════════════════════
// DEEP SLEEP MODE - EXTREME SLEEP with EXT1 wake (GPIO10 PWR + GPIO38 Touch)
// ═══════════════════════════════════════════════════════════════════════════
void enterDeepSleep() {
    USBSerial.println("════════════════════════════════════════════════════════════");
    USBSerial.println("  ENTERING EXTREME DEEP SLEEP");
    USBSerial.printf("  Sleep count: %lu\n", (unsigned long)extremeSleepCount + 1);
    USBSerial.println("  Wake sources: GPIO10 (PWR) + GPIO38 (Touch) + Timer(30m)");
    USBSerial.println("════════════════════════════════════════════════════════════");
    
    inDeepSleep = true;
    extremeSleepCount++;
    wokeFromExtremeSleep = true;
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 1: Save ALL critical state to RTC memory BEFORE anything else
    // ══════════════════════════════════════════════════════════════════════
    
    // Save battery saver level
    savedBatterySaverLevel = (int)batterySaverLevel;
    USBSerial.printf("  Saved battery saver level: %d\n", savedBatterySaverLevel);
    
    // Save watch face index
    savedWatchFaceIndex = userData.watchFaceIndex;
    USBSerial.printf("  Saved watch face: %s (%d)\n", watchFaces[savedWatchFaceIndex].name, savedWatchFaceIndex);
    
    // CRITICAL: Save RTC time to RTC memory for watchdog recovery
    saveRtcTimeToMemory();
    
    // Save step count and other fitness data
    savedStepCount = userData.steps;

    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 2: Prepare watchdog BEFORE any sleep operations
    // ══════════════════════════════════════════════════════════════════════
    prepareWatchdogForSleep();
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 3: Save to persistent storage (flash/SD)
    // ══════════════════════════════════════════════════════════════════════
    saveUserData();
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 4: Configure peripherals for sleep
    // ══════════════════════════════════════════════════════════════════════
    
    // Turn off display completely
    gfx->setBrightness(0);
    gfx->displayOff();
    
    // Disable WiFi completely
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    
    // Disable BLE if active
    if (bleEnabled) {
        stopBLE();
    }
    
    // Put IMU in lowest power mode but keep pedometer counting
    if (hasIMU) {
        // Configure pedometer for sleep - updates every 4 steps to save power
        qmi.configPedometer(
            0x007D, 0x00CC, 0x0066, 0x0050,  // Sample, peak2peak, peak, timeout
            0x14, 0x0A, 0x00, 0x04           // timelow, entry, precision, sigcount=4
        );
        USBSerial.println("  IMU pedometer configured for sleep");
    }
    
    // ══════════════════════════════════════════════════════════════════════
    // CRITICAL: Configure touch chip (FT3168) for MONITOR mode
    // The touch chip MUST stay powered in monitor mode to generate
    // the wake interrupt on GPIO38 (TP_INT)
    // 
    // Monitor mode: Low power (~50µA) but still detects touches
    // This is required for touch-to-wake functionality!
    // ══════════════════════════════════════════════════════════════════════
    USBSerial.println("  Configuring touch chip for sleep monitor mode...");
    
    // Reset touch controller to ensure clean state
    pinMode(TP_RESET, OUTPUT);
    digitalWrite(TP_RESET, LOW);
    delay(10);
    digitalWrite(TP_RESET, HIGH);
    delay(50);
    
    // Initialize I2C for touch config
    Wire.begin(IIC_SDA, IIC_SCL);
    Wire.setClock(400000);
    
    // Try to set touch chip to monitor mode
    Wire.beginTransmission(FT3168_DEVICE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        // Touch chip is responding - set to monitor mode
        FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                       FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
        USBSerial.println("  Touch chip set to MONITOR mode (touch-to-wake enabled)");
    } else {
        USBSerial.println("  WARNING: Touch chip not responding - touch wake may not work");
    }
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 5: Isolate unused GPIOs to prevent leakage current
    // This is critical for achieving ~10 µA deep sleep
    // ══════════════════════════════════════════════════════════════════════
    isolateUnusedGPIOs();
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 6: Configure power domains for minimal consumption
    // ══════════════════════════════════════════════════════════════════════
    configureDeepSleepPowerDomains();
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 7: Configure wake sources
    // ══════════════════════════════════════════════════════════════════════
    configureDeepSleepWakeSources();
    
    // ══════════════════════════════════════════════════════════════════════
    // STEP 8: Enter deep sleep
    // ══════════════════════════════════════════════════════════════════════
    USBSerial.println("  All data saved, entering deep sleep NOW...");
    USBSerial.println("  Target current: ~10 µA");
    USBSerial.flush();
    delay(10);  // Allow serial to flush
    
    // Enter deep sleep - CPU will restart on wake
    esp_deep_sleep_start();
    
    // Never reaches here - device completely resets on wake from deep sleep
}

void checkDeepSleepTimeout() {
    // Don't enter deep sleep if recording voice memo
    if (voiceMemoRecording) return;
    
    // Don't enter deep sleep if BLE connected
    if (bleDeviceConnected) return;
    
    // Calculate timeout based on mode
    unsigned long sleepTimeout = DEEP_SLEEP_TIMEOUT_MS;  // Default 4 minutes
    
    // EXTREME SLEEP: Use 2-second timeout when in EXTREME mode
    if (extremeSleepEnabled && batterySaverLevel == BATTERY_SAVER_EXTREME) {
        sleepTimeout = EXTREME_SLEEP_TIMEOUT_MS;  // 2 seconds
    }
    
    // Check if timeout passed since last touch
    if (millis() - lastTouchTime > sleepTimeout) {
        USBSerial.printf("[SLEEP] Timeout reached (%lums) - entering deep sleep\n", sleepTimeout);
        enterDeepSleep();
    }
}


// ═══════════════════════════════════════════════════════════════════════════
// GLANCE MODE - Shows user's selected watch face on wake (MEDIUM + EXTREME modes)
// Shows watch face for 500ms min, returns to sleep if no touch within 2 seconds
// ═══════════════════════════════════════════════════════════════════════════

// LVGL display buffer for glance mode
static lv_disp_draw_buf_t glance_draw_buf;
static lv_color_t *glance_buf = NULL;
static lv_disp_drv_t glance_disp_drv;
static bool glance_lvgl_initialized = false;

/**
 * LVGL display flush callback for glance mode
 */
void glance_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    
    lv_disp_flush_ready(disp);
}

/**
 * Initialize minimal LVGL for glance mode
 */
bool initGlanceLVGL() {
    if (glance_lvgl_initialized) return true;
    
    USBSerial.println("[GLANCE] Initializing minimal LVGL...");
    
    lv_init();
    
    // Allocate display buffer
    uint32_t buf_size = LCD_WIDTH * LVGL_BUFFER_LINES;
    glance_buf = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!glance_buf) {
        USBSerial.println("[GLANCE] ERROR: Failed to allocate LVGL buffer");
        return false;
    }
    
    lv_disp_draw_buf_init(&glance_draw_buf, glance_buf, NULL, buf_size);
    
    lv_disp_drv_init(&glance_disp_drv);
    glance_disp_drv.hor_res = LCD_WIDTH;
    glance_disp_drv.ver_res = LCD_HEIGHT;
    glance_disp_drv.flush_cb = glance_disp_flush;
    glance_disp_drv.draw_buf = &glance_draw_buf;
    lv_disp_drv_register(&glance_disp_drv);
    
    glance_lvgl_initialized = true;
    USBSerial.println("[GLANCE] LVGL initialized successfully");
    return true;
}

/**
 * Glance Digital Face - Simple large time
 */
void createGlanceDigitalFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    char timeBuf[6];
    sprintf(timeBuf, "%02d:%02d", hours, minutes);
    
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, -20);
}

/**
 * Glance Word Clock Face
 */
void createGlanceWordFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes) {
    int displayHour = hours % 12;
    if (displayHour == 0) displayHour = 12;
    
    const char *hourWords[] = {"TWELVE", "ONE", "TWO", "THREE", "FOUR", "FIVE", 
                               "SIX", "SEVEN", "EIGHT", "NINE", "TEN", "ELEVEN"};
    
    lv_obj_t *hourLabel = lv_label_create(parent);
    lv_label_set_text(hourLabel, hourWords[displayHour % 12]);
    lv_obj_set_style_text_color(hourLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(hourLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(hourLabel, LV_ALIGN_CENTER, 0, -40);
    
    char minBuf[20];
    if (minutes == 0) {
        sprintf(minBuf, "O'CLOCK");
    } else if (minutes < 10) {
        sprintf(minBuf, "OH %d", minutes);
    } else {
        sprintf(minBuf, "%d", minutes);
    }
    
    lv_obj_t *minLabel = lv_label_create(parent);
    lv_label_set_text(minLabel, minBuf);
    lv_obj_set_style_text_color(minLabel, lv_color_hex(0x00D4AA), 0);
    lv_obj_set_style_text_font(minLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(minLabel, LV_ALIGN_CENTER, 0, 10);
}

/**
 * Glance Analog Face - Simple clock with digital backup
 */
void createGlanceAnalogFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    int radius = 100;
    
    // Clock circle
    lv_obj_t *circle = lv_obj_create(parent);
    lv_obj_set_size(circle, radius * 2, radius * 2);
    lv_obj_align(circle, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(circle, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_border_color(circle, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(circle, 2, 0);
    lv_obj_set_scrollbar_mode(circle, LV_SCROLLBAR_MODE_OFF);
    
    // Digital time in center
    char timeBuf[6];
    sprintf(timeBuf, "%02d:%02d", hours, minutes);
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, -20);
}

/**
 * Glance Nike Face - Bold sport style with Nike font
 */
void createGlanceNikeFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    lv_color_t nikeColor = lv_color_hex(0xFFD700);  // Nike yellow
    
    char timeBuf[6];
    sprintf(timeBuf, "%02d:%02d", hours, minutes);
    
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, nikeColor, 0);
    lv_obj_set_style_text_font(timeLabel, &NIKE_FONT, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, -20);
    
    char secBuf[4];
    sprintf(secBuf, ":%02d", seconds);
    lv_obj_t *secLabel = lv_label_create(parent);
    lv_label_set_text(secLabel, secBuf);
    lv_obj_set_style_text_color(secLabel, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(secLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(secLabel, LV_ALIGN_CENTER, 0, 50);
}

/**
 * Glance Minimal Face - Ultra clean
 */
void createGlanceMinimalFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes) {
    char timeBuf[6];
    sprintf(timeBuf, "%02d:%02d", hours, minutes);
    
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, 0);
}

/**
 * Glance Fitness Face - Time with steps placeholder
 */
void createGlanceFitnessFace(lv_obj_t *parent, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    char timeBuf[6];
    sprintf(timeBuf, "%02d:%02d", hours, minutes);
    
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, -30);
    
    lv_obj_t *stepsLabel = lv_label_create(parent);
    lv_label_set_text(stepsLabel, "--- steps");
    lv_obj_set_style_text_color(stepsLabel, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(stepsLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(stepsLabel, LV_ALIGN_CENTER, 0, 30);
}

/**
 * Create the user's selected watch face for glance mode
 */
void createGlanceWatchFace(int watchFaceIndex, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    USBSerial.printf("[GLANCE] Creating watch face: %s (index %d)\n", 
                    watchFaces[watchFaceIndex].name, watchFaceIndex);
    
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    
    switch (watchFaceIndex) {
        case 0:  // Digital
            createGlanceDigitalFace(scr, hours, minutes, seconds);
            break;
        case 1:  // Word Clock
            createGlanceWordFace(scr, hours, minutes);
            break;
        case 2:  // Analog
            createGlanceAnalogFace(scr, hours, minutes, seconds);
            break;
        case 3:  // Nike Sport
            createGlanceNikeFace(scr, hours, minutes, seconds);
            break;
        case 4:  // Minimal
            createGlanceMinimalFace(scr, hours, minutes);
            break;
        case 5:  // Fitness
            createGlanceFitnessFace(scr, hours, minutes, seconds);
            break;
        case 6:  // Photo - simple dark background for fast glance
            createGlanceNikeFace(scr, hours, minutes, seconds);  // Reuse Nike glance (dark bg + Nike font)
            break;
        default:
            createGlanceDigitalFace(scr, hours, minutes, seconds);
            break;
    }
    
    // Add subtle hint at bottom
    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "tap to wake");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -15);
    
    lv_timer_handler();
}

/**
 * Run glance mode with user's selected watch face
 * Returns true if user interacted (proceed to full boot)
 * Returns false if timeout (go back to sleep)
 */
bool runGlanceMode(uint8_t hours, uint8_t minutes) {
    USBSerial.println("[GLANCE] ════════════════════════════════════════════");
    USBSerial.printf("[GLANCE] Quick wake - showing %02d:%02d\n", hours, minutes);
    USBSerial.printf("[GLANCE] Watch face: %s (index %d)\n", 
                    watchFaces[savedWatchFaceIndex].name, savedWatchFaceIndex);
    USBSerial.println("[GLANCE] Tap screen to fully wake, or wait 2s to sleep");
    USBSerial.println("[GLANCE] ════════════════════════════════════════════");
    
    // Initialize display
    pinMode(LCD_RESET, OUTPUT);
    digitalWrite(LCD_RESET, HIGH);
    delay(50);
    
    gfx->begin();
    gfx->setBrightness(30);
    gfx->displayOn();
    gfx->fillScreen(0x0000);  // BLACK
    
    // Initialize minimal LVGL and show watch face
    if (!initGlanceLVGL()) {
        USBSerial.println("[GLANCE] LVGL init failed, using fallback");
        gfx->setTextSize(4);
        gfx->setTextColor(0xFFFF);  // WHITE
        char buf[6];
        sprintf(buf, "%02d:%02d", hours, minutes);
        gfx->setCursor(100, 220);
        gfx->print(buf);
    } else {
        createGlanceWatchFace(savedWatchFaceIndex, hours, minutes, savedRtcTime.seconds);
    }
    
    pinMode(TP_INT, INPUT);
    pinMode(PWR_BUTTON, INPUT_PULLUP);
    
    unsigned long glanceStartTime = millis();
    unsigned long minDisplayTime = glanceStartTime + GLANCE_MODE_DISPLAY_MS;
    unsigned long maxWaitTime = glanceStartTime + GLANCE_MODE_TIMEOUT_MS;
    
    while (millis() < maxWaitTime) {
        if (glance_lvgl_initialized) {
            lv_timer_handler();
        }
        
        if (digitalRead(TP_INT) == LOW) {
            delay(20);
            if (digitalRead(TP_INT) == LOW && millis() >= minDisplayTime) {
                USBSerial.println("[GLANCE] Touch detected - proceeding to full boot");
                if (glance_buf) {
                    heap_caps_free(glance_buf);
                    glance_buf = NULL;
                }
                glance_lvgl_initialized = false;
                return true;
            }
        }
        
        if (digitalRead(PWR_BUTTON) == LOW) {
            delay(20);
            if (digitalRead(PWR_BUTTON) == LOW && millis() >= minDisplayTime) {
                USBSerial.println("[GLANCE] Button detected - proceeding to full boot");
                if (glance_buf) {
                    heap_caps_free(glance_buf);
                    glance_buf = NULL;
                }
                glance_lvgl_initialized = false;
                return true;
            }
        }
        
        delay(10);
    }
    
    USBSerial.println("[GLANCE] No interaction - returning to sleep");
    
    if (glance_buf) {
        heap_caps_free(glance_buf);
        glance_buf = NULL;
    }
    glance_lvgl_initialized = false;
    
    gfx->displayOff();
    gfx->setBrightness(0);
    
    return false;
}

void updateBurnInOffset() {
    if (millis() - lastBurnInShift < BURN_IN_SHIFT_INTERVAL_MS) return;
    
    lastBurnInShift = millis();
    
    // Shift in a small pattern to prevent burn-in
    static int8_t shiftPhase = 0;
    shiftPhase = (shiftPhase + 1) % 8;
    
    // Create a subtle shifting pattern
    switch (shiftPhase) {
        case 0: burnInOffsetX = 0;  burnInOffsetY = 0;  break;
        case 1: burnInOffsetX = 1;  burnInOffsetY = 0;  break;
        case 2: burnInOffsetX = 2;  burnInOffsetY = 1;  break;
        case 3: burnInOffsetX = 1;  burnInOffsetY = 2;  break;
        case 4: burnInOffsetX = 0;  burnInOffsetY = 2;  break;
        case 5: burnInOffsetX = -1; burnInOffsetY = 1;  break;
        case 6: burnInOffsetX = -2; burnInOffsetY = 0;  break;
        case 7: burnInOffsetX = -1; burnInOffsetY = -1; break;
    }
}

int8_t getBurnInOffsetX() { return burnInOffsetX; }
int8_t getBurnInOffsetY() { return burnInOffsetY; }

// ═══════════════════════════════════════════════════════════════════════════
// POWER SAVING MODE CONTROL
// ═══════════════════════════════════════════════════════════════════════════
void applyPowerSavingMode(bool enable) {
    powerSaverActive = enable;
    
    if (enable) {
        setCpuFrequencyMhz(CPU_FREQ_SAVER);
        sensorPollInterval = SENSOR_POLL_SAVER_MS;
    } else {
        setCpuFrequencyMhz(CPU_FREQ_NORMAL);
        sensorPollInterval = SENSOR_POLL_NORMAL_MS;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN ON/OFF FUNCTIONS - ENHANCED WITH LIGHT SLEEP
// ═══════════════════════════════════════════════════════════════════════════
void setTouchEnabled(bool enabled) {
    touchEnabled = enabled;
}

void screenOff() {
    if (!screenOn) return;
    
    screenOn = false;
    screenOffStartMs = millis();
    batteryStats.screenOnTimeMs += (screenOffStartMs - screenOnStartMs);
    
    gfx->setBrightness(0);
    
    // In Medium or Extreme modes, apply power saving
    if (batterySaverLevel >= BATTERY_SAVER_MEDIUM) {
        applyPowerSavingMode(true);
    }
    
    // If in Extreme mode AND battery low, enter light sleep
    if (shouldEnableLightSleep()) {
        USBSerial.println("[POWER] Entering light sleep (EXTREME + low battery)...");
        enterLightSleep();
    }
    
    ui_event = UI_EVENT_SCREEN_OFF;
}

void screenOnFunc() {
    if (screenOn) return;
    
    // If waking from light sleep, recovery is already done
    if (currentPowerState == POWER_STATE_WAKING) {
        currentPowerState = POWER_STATE_ACTIVE;
    }
    
    screenOn = true;
    screenOnStartMs = millis();
    batteryStats.screenOffTimeMs += (screenOnStartMs - screenOffStartMs);
    lastActivityMs = millis();
    last_ui_activity = millis();  // Reset activity timer to prevent immediate screen-off
    
    // ═══ POWER RESERVE MODE - Show minimal face only ═══
    if (powerReserveMode) {
        gfx->setBrightness(20);  // Very low brightness
        createPowerReserveFace();
        setTouchEnabled(true);
        ui_event = UI_EVENT_SCREEN_ON;
        return;  // Don't do normal navigation
    }
    
    // Restore brightness based on saver level
    gfx->setBrightness(saverModes[batterySaverLevel].brightness);
    
    // Restore CPU if power saving was active
    if (powerSaverActive && batterySaverLevel != BATTERY_SAVER_EXTREME) {
        applyPowerSavingMode(false);
    }
    
    setTouchEnabled(true);
    
    // Refresh the current screen
    navigateTo(currentCategory, currentSubCard);
    
    ui_event = UI_EVENT_SCREEN_ON;
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY WARNINGS - ENHANCED WITH AUTO EXTREME MODE
// ═══════════════════════════════════════════════════════════════════════════
void checkBatteryWarnings() {
    if (batteryPercent <= CRITICAL_BATTERY_WARNING && !criticalBatteryWarningShown && !isCharging) {
        criticalBatteryWarningShown = true;
        showingLowBatteryPopup = true;
        lowBatteryPopupTime = millis();
        
        // Auto-enable Extreme saver at critical battery
        if (batterySaverLevel != BATTERY_SAVER_EXTREME) {
            applyBatterySaverMode(BATTERY_SAVER_EXTREME);
            batterySaverAutoEnabled = true;
            USBSerial.println("[BATTERY] CRITICAL! Auto-enabled EXTREME saver with light sleep");
        }
        
    } else if (batteryPercent <= LOW_BATTERY_WARNING && !lowBatteryWarningShown && !isCharging) {
        lowBatteryWarningShown = true;
        showingLowBatteryPopup = true;
        lowBatteryPopupTime = millis();
        
        // Check if we should enable light sleep now
        if (batterySaverLevel == BATTERY_SAVER_EXTREME && !lightSleepEnabled) {
            lightSleepEnabled = true;
            USBSerial.println("[BATTERY] LOW! Light sleep now enabled in EXTREME mode");
        }
    }
    
    // Reset warnings when charging or battery recovers
    if (isCharging || batteryPercent > LOW_BATTERY_WARNING) {
        lowBatteryWarningShown = false;
        criticalBatteryWarningShown = false;
        
        // Disable auto-enabled battery saver when charging
        if (isCharging && batterySaverAutoEnabled) {
            batterySaverAutoEnabled = false;
        }
        
        // Exit power reserve mode when battery recovers above 20%
        if (batteryPercent > 20 && powerReserveMode) {
            exitPowerReserveMode();
        }
    }
    
    // ═══ POWER RESERVE MODE - Battery below 20% ═══
    // Enter ultra-low-power mode: only shows time briefly, no navigation
    if (batteryPercent < 20 && !isCharging && !powerReserveMode) {
        enterPowerReserveMode();
    }
    
    // Auto-dismiss popup after 3 seconds
    if (showingLowBatteryPopup && (millis() - lowBatteryPopupTime > 3000)) {
        showingLowBatteryPopup = false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void disableAllScrolling(lv_obj_t *obj) {
    if (obj == NULL) return;
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

float getEstimatedCurrentMA() {
    switch (currentPowerState) {
        case POWER_STATE_ACTIVE:
            return 80.0 + (saverModes[batterySaverLevel].brightness / 255.0) * 50.0;
        case POWER_STATE_DIMMED:
            return 40.0;
        case POWER_STATE_SCREEN_OFF:
            return 15.0;
        case POWER_STATE_LIGHT_SLEEP:
            return LIGHT_SLEEP_CURRENT_MA;
        default:
            return 50.0;
    }
}

uint32_t getEstimatedBatteryMinutes() {
    float remainingMAh = (batteryPercent / 100.0) * BATTERY_CAPACITY_MAH;
    float currentMA = getEstimatedCurrentMA();
    
    if (currentMA <= 0) return 0;
    
    return (uint32_t)((remainingMAh / currentMA) * 60.0);
}

void printPowerStats() {
    USBSerial.println("\n═══ POWER STATISTICS ═══");
    USBSerial.printf("Power State: %d\n", currentPowerState);
    USBSerial.printf("Saver Level: %s\n", saverModes[batterySaverLevel].name);
    USBSerial.printf("Battery: %d%% (%dmV)\n", batteryPercent, batteryVoltage);
    USBSerial.printf("Charging: %s\n", isCharging ? "Yes" : "No");
    USBSerial.printf("Light Sleep: %s\n", lightSleepEnabled ? "ENABLED" : "disabled");
    USBSerial.printf("Est Current: %.1f mA\n", getEstimatedCurrentMA());
    USBSerial.printf("Est Remaining: %lu min\n", getEstimatedBatteryMinutes());
    USBSerial.printf("CPU Freq: %d MHz\n", getCpuFrequencyMhz());
    USBSerial.printf("Steps: %lu\n", userData.steps);
    USBSerial.printf("Light Sleep Time: %lu ms\n", batteryStats.lightSleepTimeMs);
    USBSerial.println("═════════════════════════\n");
}

void calculateSunTimes() {
    RTC_DateTime dt = rtc.getDateTime();
    int dayOfYear = dt.getDay();
    
    float seasonOffset = sin((dayOfYear - 80) * 3.14159 / 182.5) * 1.5;
    
    sunriseHour = 6.0 - seasonOffset;
    sunsetHour = 18.5 + seasonOffset;
    
    if (sunriseHour < 4.5) sunriseHour = 4.5;
    if (sunriseHour > 7.5) sunriseHour = 7.5;
    if (sunsetHour < 16.5) sunsetHour = 16.5;
    if (sunsetHour > 21.0) sunsetHour = 21.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS - UI Functions (FROM v7.2)
// ═══════════════════════════════════════════════════════════════════════════
void navigateTo(int category, int subCard);
void handleSwipe(int dx, int dy);
void handleTap(int x, int y);
void saveUserData();
void loadUserData();
void shutdownDevice();
void drawNikeSwoosh(lv_obj_t *parent, int cx, int cy, int size, uint32_t color);
void scanSDPhotos();
void createPhotoFace(lv_obj_t *parent);
void createClockCard();
void createCompassCard();
void createTiltCard();
void createStepsCard();
void createDistanceCard();
void createBlackjackCard();
void createWeatherCard();
void createForecastCard();
void createStopwatchCard();
void createTorchCard();
void createTorchSettingsCard();
void createCalculatorCard();
void createToolsSelectorCard();
void createSettingsCard();
void createBatteryCard();
void createBatteryStatsCard();
void createUsagePatternsCard();
void createSDCardHealthCard();
void createPowerStatsCard();
void createAboutCard();
void createBatterySaverCard();
void createTxtFilesCard();
void createWiFiCard();
void connectToHardcodedWiFi();
void scanWiFiNetworks();
void fetchLocationFromIP();
void updateSDCardHealth();
void displayWallpaperImage(lv_obj_t *parent, int wallpaperIndex);

// Battery estimation functions
void handleSettingsTap(int x, int y);
void handleBluetoothTap(int x, int y);
void handleNotificationsTap(int x, int y);
void drawBLEIndicator(lv_obj_t *parent);

// TXT File functions
void loadTxtFileList();
void loadTxtFileContent(int index);
void createSampleTxtFile();

// TIME BACKUP functions
void saveTimeBackup();
void restoreTimeBackup();
bool hasTimeBackup();

// Power Logging functions
void initPowerLogging();
void logPowerConsumption();
void writePowerSummary();
String formatPowerLogEntry(PowerLogEntry &entry);
void rotatePowerLogIfNeeded();
void archiveDailyPowerLog();
void writeDailyStats();

// Blackjack game functions
void drawPlayingCard(lv_obj_t *parent, int cardValue, int x, int y, bool faceUp);
int calculateHandValue(int *cards, int count);
void dealCard(int *cards, int *count, bool toPlayer);
void startNewBlackjackGame();
void playerHit();
void playerStandAction();

// Gyro power management - enable only when needed
void enableGyroForCard();
void disableGyroIfNotNeeded();
bool cardNeedsGyro(int category, int subCard);

// Power Reserve mode - critical battery (< 20%)
void createPowerReserveFace();
void enterPowerReserveMode();
void exitPowerReserveMode();
void checkPowerReserveTimeout();

// Tilt Maze game functions
void createTiltMazeCard();
void updateTiltMaze();
void resetTiltMaze();
bool checkMazeCollision(float newX, float newY);
bool checkMazeGoal();

// Pong game functions
void createPongCard();
void updatePongGame();
void resetPongGame();

// Game selector functions
void createGameSelectorCard();

// Tic-Tac-Toe functions
void createTicTacToeCard();
void resetTicTacToe();
int checkTTTWinner();
void tttAiMove();
int tttMinimax(bool isMaximizing);

// 2048 functions
void create2048Card();
void reset2048();
void add2048Tile();
bool move2048(int direction);  // 0=up, 1=right, 2=down, 3=left
bool canMove2048();
uint32_t getTileColor(int value);
uint32_t getTileTextColor(int value);

// Wordle functions
void createWordleCard();
void resetWordle();
void loadWordleWords();
void wordleAddLetter(char c);
void wordleDeleteLetter();
void wordleSubmitGuess();
int getWordleLetterState(int row, int col);  // 0=empty, 1=gray, 2=yellow, 3=green
uint32_t getWordleKeyColor(char letter);     // Get keyboard key color

// Breakout functions
void createBreakoutCard();
void resetBreakout();
void resetBreakoutFull();
void initBreakoutLevel(int level);
void advanceBreakoutLevel();
void updateBreakout();

// Tetris functions
void createTetrisCard();
void resetTetris();
void updateTetris();
void tetrisSpawnPiece();
bool tetrisCanMove(int dx, int dy);
bool tetrisCanRotate();
void tetrisRotatePiece();
void tetrisLockPiece();
void tetrisClearLines();

// SD Card functions
bool createDirectoryIfNotExists(const char* path);
void logToBootLog(const char* message);
bool createWidgetOSFolderStructure();
bool initWidgetOSSDCard();
bool loadWiFiConfigFromSD();
void checkAutoBackup();
bool createBackup(bool isAuto);
bool restoreFromBackup(const String& backupName);
void handleFusionLabsProtocol();
void sendDeviceStatus();
void sendSDHealth();

static lv_obj_t *current_screen_obj = NULL;
// ═══════════════════════════════════════════════════════════════════════════
// DISPLAY DRIVER
// ═══════════════════════════════════════════════════════════════════════════
static void lvgl_tick_cb(void *arg) {
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    int32_t x = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
    int32_t y = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
    uint8_t fingers = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

    if (fingers > 0) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
        ui_activity_ping();
        lastActivityMs = millis();  // Keep screen alive on any touch

        // WAKE SCREEN ON TOUCH
        if (!screenOn) {
            ui_event = UI_EVENT_SCREEN_ON;
            return;  // Don't process touch as navigation when waking
        }

        // ═══ BRIGHTNESS SLIDER DRAG HANDLING ═══
        // Check if dragging in the brightness slider area while dock is open
        if (quickActionsDockVisible) {
            int moduleY = 22;
            int moduleHeight = 90;
            int row2Y = moduleY + moduleHeight + 10;
            int sliderHeight = 50;
            
            // Check if touch is in brightness slider region
            if (y >= row2Y && y <= row2Y + sliderHeight && x >= 8 && x <= LCD_WIDTH - 70) {
                // Calculate brightness from X position
                int sliderWidth = LCD_WIDTH - 80;
                int barStartX = 8 + 30;  // Module start + icon space
                int barWidth = sliderWidth - 70;
                
                // Map touch X to brightness (30-255, never fully off)
                int relativeX = x - barStartX;
                relativeX = max(0, min(relativeX, barWidth));
                int newBrightness = 30 + (relativeX * 225) / barWidth;
                newBrightness = max(30, min(255, newBrightness));
                
                if (abs(newBrightness - dockBrightness) > 5) {  // Debounce small changes
                    dockBrightness = newBrightness;
                    gfx->setBrightness(dockBrightness);
                    brightnessDragActive = true;
                    USBSerial.printf("[DOCK] Brightness: %d%%\n", (dockBrightness * 100) / 255);
                    
                    // Refresh dock to update visual
                    createQuickActionsDock();
                }
                return;  // Don't process as swipe/tap while dragging brightness
            }
        }

        if (!touchActive) {
            touchActive = true;
            touchStartX = x;
            touchStartY = y;
            touchStartMs = millis();
            USBSerial.printf("[TOUCH] Start at (%d, %d)\n", x, y);  // Debug
        }
        touchCurrentX = x;
        touchCurrentY = y;
    } else {
        data->state = LV_INDEV_STATE_REL;

        // End brightness drag if it was active
        if (brightnessDragActive) {
            brightnessDragActive = false;
            // Save brightness to preferences
            USBSerial.printf("[DOCK] Brightness saved: %d\n", dockBrightness);
        }

        if (touchActive) {
            touchActive = false;
            unsigned long touchDuration = millis() - touchStartMs;
            int32_t dx = touchCurrentX - touchStartX;
            int32_t dy = touchCurrentY - touchStartY;

            USBSerial.printf("[TOUCH] End - dx=%d dy=%d duration=%lu\n", dx, dy, touchDuration);  // Debug

            if (touchDuration < SWIPE_MAX_DURATION) {
                if (abs(dx) > SWIPE_THRESHOLD_MIN && abs(dx) > abs(dy)) {
                    USBSerial.printf("[TOUCH] Swipe %s\n", (dx > 0) ? "LEFT" : "RIGHT");
                    ui_event = (dx > 0) ? UI_EVENT_NAV_LEFT : UI_EVENT_NAV_RIGHT;
                } else if (abs(dy) > SWIPE_THRESHOLD_MIN && abs(dy) > abs(dx)) {
                    USBSerial.printf("[TOUCH] Swipe %s\n", (dy > 0) ? "UP" : "DOWN");
                    ui_event = (dy > 0) ? UI_EVENT_NAV_UP : UI_EVENT_NAV_DOWN;
                } else if (abs(dx) < TAP_THRESHOLD && abs(dy) < TAP_THRESHOLD) {
                    USBSerial.printf("[TOUCH] Tap at (%d, %d)\n", touchCurrentX, touchCurrentY);
                    ui_event = UI_EVENT_TAP;
                    ui_event_param1 = touchCurrentX;
                    ui_event_param2 = touchCurrentY;
                }
            }
        }
    }
}

void shutdownDevice() {
    saveUserData();
    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Shutting down...");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(label);

    lv_task_handler();
    delay(1000);

    gfx->setBrightness(0);
    if (hasPMU) power.shutdown();
    esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════════════════════
// WALLPAPER IMAGE DISPLAY (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
//void displayWallpaperImage(lv_obj_t *parent, int wallpaperIndex) {
//    if (wallpaperIndex == 0) {
//        return;  // Solid color - use current gradient theme
//    }
//
//    lv_obj_t *img = lv_img_create(parent);
//
//    switch(wallpaperIndex) {
//        case 1:
//            lv_img_set_src(img, &AdobeStock_17557);
//            break;
//        case 2:
//            lv_img_set_src(img, &AdobeStock_2026);
//            break;
//        case 3:
//            lv_img_set_src(img, &AdobeStock_184869446);
//            break;
//        case 4:
//            lv_img_set_src(img, &AdobeStock_174564);
//            break;
//        default:
//            lv_obj_del(img);
//            return;
//    }
//
//    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
//    lv_obj_set_size(img, LCD_WIDTH - 24, LCD_HEIGHT - 60);
//}

// ═══════════════════════════════════════════════════════════════════════════
// GYRO POWER MANAGEMENT - Only enable gyro for cards that need it
// ═══════════════════════════════════════════════════════════════════════════
bool cardNeedsGyro(int category, int subCard) {
    // Compass card needs gyro
    if (category == CAT_COMPASS && subCard == 0) return true;
    // Tilt card needs gyro  
    if (category == CAT_COMPASS && subCard == 1) return true;
    // Tilt Maze game needs gyro (subCard 4 in games)
    if (category == CAT_GAMES && subCard == 4) return true;
    // Note: Breakout uses TOUCH not gyro now
    // Note: Tetris uses SWIPE not gyro
    return false;
}

void enableGyroForCard() {
    if (!hasIMU || gyroEnabled) return;
    
    qmi.enableGyroscope();
    gyroEnabled = true;
    USBSerial.println("[IMU] Gyroscope ENABLED for current card");
}

void disableGyroIfNotNeeded() {
    if (!hasIMU || !gyroEnabled) return;
    
    qmi.disableGyroscope();
    gyroEnabled = false;
    USBSerial.println("[IMU] Gyroscope DISABLED to save power");
}

// ═══════════════════════════════════════════════════════════════════════════
// POWER RESERVE MODE - Critical battery saver (< 20%)
// Shows minimal time-only face, screen off after 0.5s, no navigation allowed
// ═══════════════════════════════════════════════════════════════════════════
void createPowerReserveFace() {
    lv_obj_clean(lv_scr_act());
    disableAllScrolling(lv_scr_act());
    
    // Black background
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
    
    // Red lightning bolt icon (left of time)
    lv_obj_t *boltLabel = lv_label_create(lv_scr_act());
    lv_label_set_text(boltLabel, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_color(boltLabel, lv_color_hex(0xFF3B30), 0);  // Red
    lv_obj_set_style_text_font(boltLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(boltLabel, LV_ALIGN_CENTER, -90, 0);
    
    // Time display - large white text
    char timeBuf[16];
    if (hasRTC) {
        RTC_DateTime now = rtc.getDateTime();
        snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", now.getHour(), now.getMinute());
    } else {
        snprintf(timeBuf, sizeof(timeBuf), "--:--");
    }
    
    lv_obj_t *timeLabel = lv_label_create(lv_scr_act());
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 20, 0);
    
    powerReserveWakeTime = millis();
    USBSerial.println("[POWER RESERVE] Face displayed - will turn off in 0.5s");
}

void enterPowerReserveMode() {
    if (powerReserveMode) return;  // Already in power reserve
    
    powerReserveMode = true;
    
    // Disable all power-hungry features
    disableGyroIfNotNeeded();
    
    // Set minimum brightness
    gfx->setBrightness(20);
    
    // Show the power reserve face
    createPowerReserveFace();
    
    USBSerial.println("[POWER RESERVE] ENTERED - Battery critical < 20%");
    USBSerial.println("[POWER RESERVE] Only time shown, no navigation until charged");
}

void exitPowerReserveMode() {
    if (!powerReserveMode) return;
    
    powerReserveMode = false;
    
    // Restore normal brightness
    gfx->setBrightness(saverModes[batterySaverLevel].brightness);
    
    // Navigate back to clock
    navigateTo(CAT_CLOCK, 0);
    
    USBSerial.println("[POWER RESERVE] EXITED - Battery recovered > 20%");
}

void checkPowerReserveTimeout() {
    if (!powerReserveMode || !screenOn) return;
    
    // Turn off screen after 0.5 seconds
    if (millis() - powerReserveWakeTime > POWER_RESERVE_DISPLAY_MS) {
        screenOn = false;
        gfx->displayOff();
        gfx->setBrightness(0);
        USBSerial.println("[POWER RESERVE] Screen OFF after timeout");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION (FROM FIXED - STABLE)
// ═══════════════════════════════════════════════════════════════════════════
bool canNavigate() {
    // Block ALL navigation in power reserve mode
    if (powerReserveMode) return false;
    if (isTransitioning) return false;
    if (millis() - lastNavigationMs < NAVIGATION_COOLDOWN_MS) return false;
    return true;
}

void navigateTo(int category, int subCard) {
    // Block navigation completely in power reserve mode
    if (powerReserveMode) {
        USBSerial.println("[POWER RESERVE] Navigation BLOCKED - charge battery above 20%");
        return;
    }
    
    if (!canNavigate() && (category != currentCategory || subCard != currentSubCard)) {
        return;
    }

    navigationLocked = true;
    lastNavigationMs = millis();

    currentCategory = category;
    currentSubCard = subCard;
    
    // ═══ GYRO POWER MANAGEMENT - Enable/disable based on card needs ═══
    if (cardNeedsGyro(category, subCard)) {
        enableGyroForCard();
    } else {
        disableGyroIfNotNeeded();
    }

    lv_obj_clean(lv_scr_act());

    switch (category) {
        case CAT_CLOCK:
            createClockCard();  // Only clock card now (no time settings)
            break;
        case CAT_COMPASS:
            if (subCard == 0) createCompassCard();
            else if (subCard == 1) createTiltCard();
            else createCompassCard();
            break;
        case CAT_ACTIVITY:
            if (subCard == 0) createStepsCard();
            else if (subCard == 1) createDistanceCard();
            else if (subCard == 2) createRunningCard();  // Running pace
            else if (subCard == 3) createSportTrackerCard();  // NEW: Sport + Movement Tracker
            else createStepsCard();
            break;
        case CAT_GAMES:
            if (subCard == 0) createBlackjackCard();
            else if (subCard == 1) createGameSelectorCard();  // Game Selector with icons
            else if (subCard == 2) createDiceRollerCard();
            else if (subCard == 3) createMagic8BallCard();
            else if (subCard == 4) createTiltMazeCard();
            else if (subCard == 5) createPongCard();
            else if (subCard == 6) createTicTacToeCard();
            else if (subCard == 7) create2048Card();
            else if (subCard == 8) createWordleCard();
            else if (subCard == 9) createBreakoutCard();
            else if (subCard == 10) createTetrisCard();
            else createBlackjackCard();
            break;
        case CAT_WEATHER:
            if (subCard == 0) createWeatherCard();
            else createForecastCard();
            break;
        case CAT_TIMER:
            createStopwatchCard();
            break;
        case CAT_TORCH:
            if (subCard == 0) createTorchCard();
            else createTorchSettingsCard();
            break;
        case CAT_TOOLS:
            if (subCard == 0) createToolsSelectorCard();      // Tools Selector
            else if (subCard == 1) createCalculatorCard();
            else if (subCard == 2) createTallyCounterCard();
            else if (subCard == 3) createVoiceMemoCard();
            else if (subCard == 4) createTxtFilesCard();
            else createToolsSelectorCard();
            break;
        case CAT_BLUETOOTH:
            if (subCard == 0) createBluetoothCard();
            else if (subCard == 1) createNotificationsCard();
            else createBluetoothCard();
            break;
        case CAT_SETTINGS:
            if (subCard == 0) createSettingsCard();
            else if (subCard == 1) createWiFiCard();  // WiFi moved to Settings
            else createSettingsCard();
            break;
        case CAT_SYSTEM:
            if (subCard == 0) createBatteryCard();
            else if (subCard == 1) createBatterySaverCard();  // NEW: Battery Saver Modes
            else if (subCard == 2) createBatteryStatsCard();
            else if (subCard == 3) createUsagePatternsCard();
            else if (subCard == 4) createSDCardHealthCard();
            else if (subCard == 5) createPowerStatsCard();  // NEW: Power Stats from logs
            else createBatteryCard();
            break;
        case CAT_ABOUT:
            createAboutCard();  // NEW: Software Info Card
            break;
        default:
            createClockCard();
            break;
    }

    lv_refr_now(NULL);
    lastActivityMs = millis();
    ui_activity_ping();
    navigationLocked = false;
}

void handleSwipe(int dx, int dy) {
    // Block all swipes in power reserve mode
    if (powerReserveMode) {
        USBSerial.println("[POWER RESERVE] Swipe BLOCKED - charge battery above 20%");
        return;
    }
    
    if (isTransitioning || millis() - lastNavigationMs < NAVIGATION_COOLDOWN_MS) return;
    
    lastTouchTime = millis();  // Reset deep sleep timer
    lastUserInteraction = millis();  // Reset adaptive battery timer

    if (showingLowBatteryPopup) {
        showingLowBatteryPopup = false;
        navigateTo(currentCategory, currentSubCard);
        lastNavigationMs = millis();
        return;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // QUICK ACTIONS DOCK - Swipe UP to close, Swipe DOWN from clock to open
    // ═══════════════════════════════════════════════════════════════════════
    if (quickActionsDockVisible) {
        // Swipe UP closes the dock and returns to clock
        if (abs(dy) > abs(dx) && abs(dy) > SWIPE_THRESHOLD_MIN && dy < 0) {
            quickActionsDockVisible = false;
            USBSerial.println("[DOCK] Closing Quick Actions - returning to clock");
            navigateTo(CAT_CLOCK, 0);
            lastNavigationMs = millis();
            return;
        }
        // Block horizontal swipes while dock is open
        if (abs(dx) > SWIPE_THRESHOLD_MIN) {
            USBSerial.println("[DOCK] Horizontal swipe blocked - swipe UP to close");
            return;
        }
        return;
    }
    
    // Swipe DOWN from clock opens Quick Actions Dock
    if (currentCategory == CAT_CLOCK && currentSubCard == 0) {
        if (abs(dy) > abs(dx) && abs(dy) > SWIPE_THRESHOLD_MIN && dy > 0) {
            quickActionsDockVisible = true;
            quickDockOpenTime = millis();
            USBSerial.println("[DOCK] Opening Quick Actions Dock");
            createQuickActionsDock();
            lastNavigationMs = millis();
            return;
        }
    }
    
    // Special handling for 2048 game swipes
    if (currentCategory == CAT_GAMES && currentSubCard == 7 && !game2048Over) {
        int direction = -1;
        if (abs(dy) > abs(dx) && abs(dy) > SWIPE_THRESHOLD_MIN) {
            direction = (dy < 0) ? 0 : 2;  // 0=up, 2=down
        } else if (abs(dx) > abs(dy) && abs(dx) > SWIPE_THRESHOLD_MIN) {
            direction = (dx > 0) ? 1 : 3;  // 1=right, 3=left
        }
        if (direction >= 0) {
            if (move2048(direction)) {
                add2048Tile();
                if (!canMove2048()) {
                    game2048Over = true;
                }
            }
            navigateTo(currentCategory, currentSubCard);
            lastNavigationMs = millis();
            return;
        }
    }
    
    // TETRIS - Swipe to move piece one block in swipe direction
    if (currentCategory == CAT_GAMES && currentSubCard == 10 && tetrisGameActive && !tetrisGameOver) {
        if (abs(dx) > SWIPE_THRESHOLD_MIN && abs(dx) > abs(dy)) {
            // Horizontal swipe - move one block left or right
            if (dx > 0 && tetrisCanMove(1, 0)) {
                tetrisCurrentX++;
            } else if (dx < 0 && tetrisCanMove(-1, 0)) {
                tetrisCurrentX--;
            }
            navigateTo(currentCategory, currentSubCard);
            lastNavigationMs = millis();
            return;
        } else if (abs(dy) > SWIPE_THRESHOLD_MIN && dy > 0) {
            // Swipe down - drop piece faster (move down until can't)
            while (tetrisCanMove(0, 1)) {
                tetrisCurrentY++;
            }
            navigateTo(currentCategory, currentSubCard);
            lastNavigationMs = millis();
            return;
        }
    }

    int newCategory = currentCategory;
    int newSubCard = currentSubCard;

    if (abs(dx) > abs(dy) && abs(dx) > SWIPE_THRESHOLD_MIN) {
        if (dx < 0) {
            newCategory = (currentCategory + 1) % NUM_CATEGORIES;
        } else {
            newCategory = currentCategory - 1;
            if (newCategory < 0) newCategory = NUM_CATEGORIES - 1;
        }
        newSubCard = 0;
    } else if (abs(dy) > abs(dx) && abs(dy) > SWIPE_THRESHOLD_MIN) {
        // Special handling for Games category navigation
        if (currentCategory == CAT_GAMES) {
            if (currentSubCard == 0) {
                // Blackjack: swipe DOWN goes to selector (subcard 1)
                if (dy > 0) {
                    newSubCard = 1;  // Go to Game Selector
                    inGameFromSelector = false;
                }
                // swipe UP from Blackjack does nothing (it's the main card)
            } else if (currentSubCard == 1) {
                // Game Selector: swipe UP goes back to Blackjack
                if (dy < 0) {
                    newSubCard = 0;  // Go back to Blackjack
                    inGameFromSelector = false;
                }
                // swipe DOWN from selector does nothing
            } else {
                // In a game (subcards 2-10): swipe UP goes back to selector
                if (dy < 0) {
                    newSubCard = 1;  // Go back to Game Selector
                    inGameFromSelector = false;
                    // Reset game states when leaving
                    if (currentSubCard == 2) diceRolling = false;
                    if (currentSubCard == 3) magic8Shaking = false;
                    if (currentSubCard == 4) mazeGameActive = false;
                    if (currentSubCard == 5) pongGameActive = false;
                    if (currentSubCard == 6) { tttGameOver = false; resetTicTacToe(); }
                    if (currentSubCard == 7) { game2048Over = false; }
                    if (currentSubCard == 8) { wordleGameOver = false; }
                    if (currentSubCard == 9) { breakoutGameActive = false; }
                    if (currentSubCard == 10) { tetrisGameActive = false; }
                }
            }
        } else {
            // Normal vertical swipe handling for other categories
            if (dy > 0 && currentSubCard < maxSubCards[currentCategory] - 1) {
                newSubCard = currentSubCard + 1;
            } else if (dy < 0 && currentSubCard > 0) {
                newSubCard = currentSubCard - 1;
            }
        }
        
        // Special handling for Tools category navigation (like Games)
        if (currentCategory == CAT_TOOLS) {
            if (currentSubCard == 0) {
                // Tools Selector: swipe UP does nothing (main card)
                // swipe DOWN goes to first tool? No, we use tap to select
                newSubCard = 0;  // Stay on selector
            } else {
                // In a tool (subcards 1-4): swipe UP goes back to selector
                if (dy < 0) {
                    newSubCard = 0;  // Go back to Tools Selector
                }
            }
        }
    }

    if (newCategory != currentCategory || newSubCard != currentSubCard) {
        currentCategory = newCategory;
        currentSubCard = newSubCard;
        navigateTo(currentCategory, currentSubCard);
        lastNavigationMs = millis();
    }
}

void handleTap(int x, int y) {
    ui_activity_ping();
    lastActivityMs = millis();
    lastTouchTime = millis();  // Reset deep sleep timer
    lastUserInteraction = millis();  // Reset adaptive battery timer

    // ═══ POWER RESERVE MODE - Tap only shows time briefly, no other action ═══
    if (powerReserveMode) {
        // Just refresh the power reserve face (time display)
        powerReserveWakeTime = millis();  // Reset the 0.5s timeout
        USBSerial.println("[POWER RESERVE] Tap detected - time refreshed");
        return;  // Block all other tap actions
    }

    // ═══ QUICK ACTIONS DOCK - Handle taps when visible ═══
    if (quickActionsDockVisible) {
        handleQuickActionsTap(x, y);
        return;
    }

    // DEBUG: Always print current state
    USBSerial.printf("[TAP] x=%d y=%d cat=%d sub=%d\n", x, y, currentCategory, currentSubCard);

    // ═══ CLOCK SCREEN - TAP TO CYCLE! ═══
    if (currentCategory == CAT_CLOCK && currentSubCard == 0) {
        // Nike face: LEFT = cycle colors, RIGHT = cycle faces
        if (userData.watchFaceIndex == 3) {  // Nike face
            if (x < LCD_WIDTH / 2) {
                // LEFT HALF - cycle through Nike color variants
                currentNikeColor = (currentNikeColor + 1) % NUM_NIKE_COLORS;
                USBSerial.printf("[NIKE] Color: %s (%d/%d)\n", 
                                nikeColors[currentNikeColor].name, 
                                currentNikeColor + 1, NUM_NIKE_COLORS);
                saveUserData();
                navigateTo(currentCategory, currentSubCard);
                return;
            }
            // RIGHT HALF - fall through to cycle watch faces
        }
        // Photo face: LEFT = cycle photos, RIGHT = cycle faces
        if (userData.watchFaceIndex == 6) {  // Photo face
            if (x < LCD_WIDTH / 2 && numSDPhotos > 1) {
                // LEFT HALF - cycle through SD card photos
                currentPhotoIndex = (currentPhotoIndex + 1) % numSDPhotos;
                USBSerial.printf("[PHOTO] Photo: %s (%d/%d)\n",
                                sdPhotoFiles[currentPhotoIndex].c_str(),
                                currentPhotoIndex + 1, numSDPhotos);
                saveUserData();
                navigateTo(currentCategory, currentSubCard);
                return;
            }
            // RIGHT HALF - fall through to cycle watch faces
        }
        // Cycle through watch faces (right side of Nike/Photo, or any tap on other faces)
        userData.watchFaceIndex = (userData.watchFaceIndex + 1) % NUM_WATCH_FACES;
        saveUserData();
        USBSerial.printf("[CLOCK] Face: %s (%d/%d)\n", 
                        watchFaces[userData.watchFaceIndex].name, 
                        userData.watchFaceIndex + 1, NUM_WATCH_FACES);
        navigateTo(currentCategory, currentSubCard);
        return;
    }
    
    if (currentCategory == CAT_TORCH && currentSubCard == 0) {
        torchOn = !torchOn;
        if (torchOn) {
            gfx->setBrightness(torchBrightness);
        } else {
            gfx->setBrightness(userData.brightness);
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TIMER && currentSubCard == 0) {
        // Stopwatch tap handling
        static unsigned long lastStopwatchTap = 0;
        unsigned long now = millis();
        
        // Double tap to reset
        if (now - lastStopwatchTap < 400 && !stopwatchRunning) {
            // Reset stopwatch
            stopwatchElapsedMs = 0;
            lapCount = 0;
        } else {
            // Single tap to start/stop
            if (stopwatchRunning) {
                stopwatchRunning = false;
                stopwatchElapsedMs += (now - stopwatchStartMs);
                // Record lap
                if (lapCount < MAX_LAPS) {
                    lapTimes[lapCount++] = stopwatchElapsedMs;
                }
            } else {
                stopwatchRunning = true;
                stopwatchStartMs = now;
            }
        }
        lastStopwatchTap = now;
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_GAMES) {
        if (currentSubCard == 0) {
            // BLACKJACK TAP HANDLING - Fully functional
            if (!blackjackGameActive) {
                // Start new game
                startNewBlackjackGame();
            } else {
                int playerVal = calculateHandValue(playerCards, playerCount);
                
                if (playerVal > 21) {
                    // Bust - start new game
                    blackjackGameActive = false;
                    startNewBlackjackGame();
                } else if (!playerStand) {
                    // Check which button was tapped
                    if (y > LCD_HEIGHT - 100) {
                        // Bottom area - buttons
                        if (x < LCD_WIDTH / 2) {
                            // HIT button (left side)
                            playerHit();
                        } else {
                            // STAND button (right side)
                            playerStandAction();
                        }
                    }
                } else {
                    // Game over - tap to start new game
                    blackjackGameActive = false;
                    startNewBlackjackGame();
                }
            }
        } else if (currentSubCard == 1) {
            // GAME SELECTOR - tap on icons to launch games (3x3 grid)
            int iconSize = 75;
            int spacing = 12;
            int startX = (LCD_WIDTH - (iconSize * 3 + spacing * 2)) / 2;
            int startY = 70;
            
            // Calculate which icon was tapped
            int col = (x - startX) / (iconSize + spacing);
            int row = (y - startY) / (iconSize + spacing);
            
            if (col >= 0 && col < 3 && row >= 0 && row < 3 && 
                x >= startX && y >= startY && 
                y < startY + 3 * (iconSize + spacing)) {
                inGameFromSelector = true;
                int gameIndex = row * 3 + col;
                // Map: 0=Dice, 1=8Ball, 2=Maze, 3=Pong, 4=TTT, 5=2048, 6=Wordle, 7=Breakout, 8=Tetris
                int subcardMap[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
                if (gameIndex < 9) {
                    navigateTo(CAT_GAMES, subcardMap[gameIndex]);
                }
                return;
            }
        } else if (currentSubCard == 2) {
            // DICE ROLLER - shake or tap to roll
            diceRolling = true;
            diceRollStart = millis();
            diceValue1 = random(1, 7);
            diceValue2 = random(1, 7);
        } else if (currentSubCard == 3) {
            // MAGIC 8 BALL - shake or tap for answer
            magic8Shaking = true;
            magic8BallAnswer = magic8Answers[random(NUM_8BALL_ANSWERS)];
        } else if (currentSubCard == 4) {
            // TILT MAZE - tap to start/reset
            if (!mazeGameActive || mazeGameWon) {
                resetTiltMaze();
                mazeGameActive = true;
                mazeGameWon = false;
                mazeStartTime = millis();
            }
        } else if (currentSubCard == 5) {
            // PONG - tap to start/reset
            if (!pongGameActive) {
                resetPongGame();
                pongGameActive = true;
            }
        } else if (currentSubCard == 6) {
            // TIC-TAC-TOE - tap to place X
            if (tttGameOver) {
                resetTicTacToe();
            } else if (tttPlayerTurn) {
                // Calculate which cell was tapped
                int cellSize = 85;
                int gridStartX = (LCD_WIDTH - cellSize * 3) / 2;
                int gridStartY = 90;
                
                int col = (x - gridStartX) / cellSize;
                int row = (y - gridStartY) / cellSize;
                
                if (col >= 0 && col < 3 && row >= 0 && row < 3) {
                    int idx = row * 3 + col;
                    if (tttBoard[idx] == 0) {
                        tttBoard[idx] = 1;  // Player X
                        tttPlayerTurn = false;
                        int winner = checkTTTWinner();
                        if (winner != 0 || winner == -1) {
                            tttGameOver = true;
                            tttWinner = winner;
                            if (winner == 1) tttPlayerWins++;
                            else if (winner == 2) tttAiWins++;
                        } else {
                            // AI's turn
                            tttAiMove();
                            winner = checkTTTWinner();
                            if (winner != 0) {
                                tttGameOver = true;
                                tttWinner = winner;
                                if (winner == 2) tttAiWins++;
                            }
                            tttPlayerTurn = true;
                        }
                    }
                }
            }
        } else if (currentSubCard == 7) {
            // 2048 - tap to start new game if game over
            if (game2048Over) {
                reset2048();
            }
        } else if (currentSubCard == 8) {
            // WORDLE - tap keyboard letters or actions - UPDATED for larger keys
            if (wordleGameOver) {
                resetWordle();
            } else {
                // Keyboard layout - 3 rows with LARGER keys
                int keyW = 36, keyH = 48, keyGap = 5;  // Matches createWordleCard
                int row1Y = 295, row2Y = 348, row3Y = 401;
                const char* row1 = "QWERTYUIOP";
                const char* row2 = "ASDFGHJKL";
                const char* row3 = "ZXCVBNM";
                
                // Row 1 (10 keys)
                int row1X = (LCD_WIDTH - (10 * keyW + 9 * keyGap)) / 2;
                if (y >= row1Y && y < row1Y + keyH) {
                    int idx = (x - row1X) / (keyW + keyGap);
                    if (idx >= 0 && idx < 10) wordleAddLetter(row1[idx]);
                }
                // Row 2 (9 keys)
                int row2X = (LCD_WIDTH - (9 * keyW + 8 * keyGap)) / 2;
                if (y >= row2Y && y < row2Y + keyH) {
                    int idx = (x - row2X) / (keyW + keyGap);
                    if (idx >= 0 && idx < 9) wordleAddLetter(row2[idx]);
                }
                // Row 3 (ENTER + 7 keys + DEL) - wider action keys
                if (y >= row3Y && y < row3Y + keyH) {
                    if (x < 62) {
                        wordleSubmitGuess();  // ENTER (wider: 56px starting at 6)
                    } else if (x > LCD_WIDTH - 62) {
                        wordleDeleteLetter();  // DEL (wider: 56px)
                    } else {
                        int idx = (x - 66) / (keyW + keyGap);
                        if (idx >= 0 && idx < 7) wordleAddLetter(row3[idx]);
                    }
                }
            }
        } else if (currentSubCard == 9) {
            // BREAKOUT - tap to start/restart or advance level
            if (breakoutShowNextLevel) {
                // Advance to next level when tapped on "Next Level?" screen
                advanceBreakoutLevel();
                breakoutShowNextLevel = false;
            } else if (!breakoutGameActive || breakoutGameOver) {
                if (breakoutGameOver && breakoutLives <= 0) {
                    // Game over - full reset
                    resetBreakoutFull();
                } else {
                    // Start or continue
                    resetBreakout();
                }
                breakoutGameActive = true;
                breakoutGameOver = false;
            }
        } else if (currentSubCard == 10) {
            // TETRIS - tap to rotate piece
            if (!tetrisGameActive || tetrisGameOver) {
                resetTetris();
                tetrisGameActive = true;
                tetrisGameOver = false;
            } else {
                tetrisRotatePiece();
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TOOLS && currentSubCard == 0) {
        // TOOLS SELECTOR - tap on icons to launch tools
        int iconSize = 90;
        int spacing = 15;
        int startX = (LCD_WIDTH - (iconSize * 2 + spacing)) / 2;
        int startY = 80;
        
        int col = (x - startX) / (iconSize + spacing);
        int row = (y - startY) / (iconSize + spacing);
        
        if (col >= 0 && col < 2 && row >= 0 && row < 2 && 
            x >= startX && y >= startY) {
            int toolIndex = row * 2 + col;
            // Map: 0=Calculator, 1=Tally, 2=Voice, 3=TxtFiles
            int subcardMap[] = {1, 2, 3, 4};
            if (toolIndex < 4) {
                navigateTo(CAT_TOOLS, subcardMap[toolIndex]);
            }
            return;
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TOOLS && currentSubCard == 1) {
        // CALCULATOR TAP HANDLING - Enhanced with Memory
        int btnW = 50, btnH = 44;
        int btnSpacing = 4;
        int startY = 88;
        int startX = (LCD_WIDTH - (4 * btnW + 3 * btnSpacing)) / 2;
        
        // Determine which button was tapped
        if (y >= startY && y < startY + 6 * (btnH + btnSpacing)) {
            int row = (y - startY) / (btnH + btnSpacing);
            int col = (x - startX) / (btnW + btnSpacing);
            
            if (row >= 0 && row < 6 && col >= 0 && col < 4) {
                const char* btnTexts[6][4] = {
                    {"MC", "MR", "M+", "M-"},
                    {"AC", "±", "%", "÷"},
                    {"7", "8", "9", "×"},
                    {"4", "5", "6", "-"},
                    {"1", "2", "3", "+"},
                    {"0", ".", "⌫", "="}
                };
                
                const char* btn = btnTexts[row][col];
                
                // Memory functions
                if (strcmp(btn, "MC") == 0) {
                    calcMemory = 0;
                } else if (strcmp(btn, "MR") == 0) {
                    snprintf(calcDisplay, sizeof(calcDisplay), "%g", calcMemory);
                    calcNewNumber = true;
                } else if (strcmp(btn, "M+") == 0) {
                    calcMemory += atof(calcDisplay);
                } else if (strcmp(btn, "M-") == 0) {
                    calcMemory -= atof(calcDisplay);
                }
                // Standard functions
                else if (strcmp(btn, "AC") == 0) {
                    strcpy(calcDisplay, "0");
                    calcOperator = ' ';
                    calcFirstNum = 0;
                    calcNewNumber = true;
                } else if (strcmp(btn, "⌫") == 0) {
                    int len = strlen(calcDisplay);
                    if (len > 1) {
                        calcDisplay[len - 1] = '\0';
                    } else {
                        strcpy(calcDisplay, "0");
                    }
                } else if (strcmp(btn, "±") == 0) {
                    double val = atof(calcDisplay);
                    val = -val;
                    snprintf(calcDisplay, sizeof(calcDisplay), "%g", val);
                } else if (strcmp(btn, "%") == 0) {
                    double val = atof(calcDisplay);
                    val = val / 100.0;
                    snprintf(calcDisplay, sizeof(calcDisplay), "%g", val);
                } else if (strcmp(btn, "÷") == 0 || strcmp(btn, "×") == 0 || 
                           strcmp(btn, "-") == 0 || strcmp(btn, "+") == 0) {
                    calcFirstNum = atof(calcDisplay);
                    if (strcmp(btn, "÷") == 0) calcOperator = '/';
                    else if (strcmp(btn, "×") == 0) calcOperator = '*';
                    else if (strcmp(btn, "-") == 0) calcOperator = '-';
                    else calcOperator = '+';
                    calcNewNumber = true;
                } else if (strcmp(btn, "=") == 0) {
                    double secondNum = atof(calcDisplay);
                    double result = 0;
                    if (calcOperator == '+') result = calcFirstNum + secondNum;
                    else if (calcOperator == '-') result = calcFirstNum - secondNum;
                    else if (calcOperator == '*') result = calcFirstNum * secondNum;
                    else if (calcOperator == '/' && secondNum != 0) result = calcFirstNum / secondNum;
                    else result = secondNum;
                    
                    snprintf(calcDisplay, sizeof(calcDisplay), "%g", result);
                    calcOperator = ' ';
                    calcNewNumber = true;
                } else if (strcmp(btn, ".") == 0) {
                    if (strchr(calcDisplay, '.') == NULL) {
                        strcat(calcDisplay, ".");
                    }
                } else {
                    // Number button
                    if (calcNewNumber || strcmp(calcDisplay, "0") == 0) {
                        strcpy(calcDisplay, btn);
                        calcNewNumber = false;
                    } else if (strlen(calcDisplay) < 12) {
                        strcat(calcDisplay, btn);
                    }
                }
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TOOLS && currentSubCard == 2) {
        // TALLY COUNTER - Enhanced with multiple counters
        // Top area: switch counters
        if (y < 80) {
            // Tap on counter name to switch
            currentTallyCounter = (currentTallyCounter + 1) % 4;
        }
        // Middle: count display - tap to increment
        else if (y < LCD_HEIGHT - 120) {
            tallyCounters[currentTallyCounter]++;
        }
        // Bottom buttons
        else {
            if (x < LCD_WIDTH / 3) {
                // Left: decrement
                tallyCounters[currentTallyCounter] = max(0, tallyCounters[currentTallyCounter] - 1);
            } else if (x < LCD_WIDTH * 2 / 3) {
                // Middle: reset current
                tallyCounters[currentTallyCounter] = 0;
            } else {
                // Right: reset all
                for (int i = 0; i < 4; i++) tallyCounters[i] = 0;
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TOOLS && currentSubCard == 3) {
        // VOICE MEMO - tap to record/stop
        if (y > LCD_HEIGHT - 80) {
            // Button area
            if (voiceMemoRecording) {
                stopVoiceRecording();
            } else {
                startVoiceRecording();
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_ACTIVITY && currentSubCard == 2) {
        // RUNNING PACE - Start/Stop button at bottom
        if (y > LCD_HEIGHT - 80) {
            if (runningModeActive) {
                // Stop running
                runningModeActive = false;
            } else {
                // Start running
                runningModeActive = true;
                runningStartTime = millis();
                runningStartSteps = userData.steps;
                runningPace = 0;
                runningDistance = 0;
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_ACTIVITY && currentSubCard == 3) {
        // SPORT TRACKER - Start/Stop session button at bottom
        if (y > LCD_HEIGHT - 70) {
            if (sportTrackerActive) {
                // Stop session
                sportTrackerActive = false;
                sportSessionSteps = (int)userData.steps - sportSessionStartSteps;
                if (sportSessionSteps < 0) sportSessionSteps = 0;
                sportSessionDistance = sportSessionSteps * 0.0007;
                
                // Update activity streak (check if 30+ min active)
                unsigned long sessionDurationMs = millis() - sportSessionStart;
                int sessionMinutes = sessionDurationMs / 60000;
                dailyActiveMinutes += sessionMinutes;
                
                // Check streak - if 30+ active minutes today
                struct tm timeinfo;
                if (getLocalTime(&timeinfo)) {
                    int today = timeinfo.tm_yday;
                    if (dailyActiveMinutes >= STREAK_GOAL_MINUTES) {
                        if (lastActivityDay != today) {
                            if (lastActivityDay == today - 1 || lastActivityDay == -1) {
                                activityStreakDays++;  // Continue streak
                            } else {
                                activityStreakDays = 1;  // Reset streak
                            }
                            lastActivityDay = today;
                        }
                    }
                }
                
                USBSerial.printf("[SPORT] Session ended: %d steps, %.2f km, %d sprints\n", 
                    sportSessionSteps, sportSessionDistance, sportSprintCount);
            } else {
                // Start session
                sportTrackerActive = true;
                sportSessionStart = millis();
                sportSessionStartSteps = userData.steps;
                sportSprintCount = 0;
                sportSessionSteps = 0;
                sportSessionDistance = 0;
                USBSerial.println("[SPORT] Session started");
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_TOOLS && currentSubCard == 4) {
        // TXT Files card - handle file selection and scrolling
        if (numTxtFiles > 0 && !txtFileLoaded) {
            // Check for scroll zones first (if more than 5 files)
            if (numTxtFiles > 5) {
                if (y < 60) {
                    // Scroll up in file list
                    txtFileListOffset = max(0, txtFileListOffset - 1);
                    navigateTo(currentCategory, currentSubCard);
                    return;
                } else if (y > LCD_HEIGHT - 50) {
                    // Scroll down in file list
                    int maxScroll = max(0, numTxtFiles - 5);
                    txtFileListOffset = min(maxScroll, txtFileListOffset + 1);
                    navigateTo(currentCategory, currentSubCard);
                    return;
                }
            }
            
            // Select a file based on tap position (accounting for scroll offset)
            int fileIndex = ((y - 48) / 48) + txtFileListOffset;
            if (fileIndex >= 0 && fileIndex < numTxtFiles) {
                loadTxtFileContent(fileIndex);
            }
        } else if (txtFileLoaded) {
            // Back button area (header)
            if (y < 50) {
                txtFileLoaded = false;
                txtScrollOffset = 0;
            }
            // FAST SCROLL - 10 lines per tap (was 5)
            // Scroll down on bottom tap
            else if (y > LCD_HEIGHT - 80) {
                txtScrollOffset += 10;  // Scroll down FASTER!
            }
            // Scroll up on middle-top tap
            else if (y < LCD_HEIGHT / 2) {
                txtScrollOffset = max(0, txtScrollOffset - 10);  // Scroll up FASTER!
            }
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_BLUETOOTH) {
        if (currentSubCard == 0) {
            // Bluetooth Card tap handling
            handleBluetoothTap(x, y);
        } else if (currentSubCard == 1) {
            // Notifications Card tap handling
            handleNotificationsTap(x, y);
        }
    }
    else if (currentCategory == CAT_SETTINGS && currentSubCard == 1) {
        // WiFi Card tap handling (moved from Tools)
        // Button layout: status card (52-122), connect button (130-185), saved networks (212-307)
        if (y >= 130 && y <= 185) {
            // CONNECT button area - Connect to hardcoded network
            USBSerial.println("[WIFI] Connect button tapped!");
            connectToHardcodedWiFi();
        } else if (y >= 212 && y <= 307) {
            // Saved networks section - tap to connect
            int netIndex = ((y - 212) / 40) + wifiCardScrollOffset;
            
            // Check if tapping on saved networks section
            if (netIndex >= 0 && netIndex < numWifiNetworks) {
                // Connect to saved network
                WiFi.begin(wifiNetworks[netIndex].ssid, wifiNetworks[netIndex].password);
                USBSerial.printf("[WIFI] Connecting to saved: %s\n", wifiNetworks[netIndex].ssid);
            } else {
                // Check roaming networks
                int roamIdx = netIndex - numWifiNetworks;
                if (roamIdx >= 0 && roamIdx < numScannedNetworks) {
                    if (scannedNetworks[roamIdx].isOpen) {
                        WiFi.begin(scannedNetworks[roamIdx].ssid);
                        USBSerial.printf("[WIFI] Connecting to open: %s\n", scannedNetworks[roamIdx].ssid);
                    }
                }
            }
        } else if (y < 52) {
            // Top area - rescan networks
            wifiScanComplete = false;
            scanWiFiNetworks();
        }
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_COMPASS && currentSubCard == 0) {
        // Calibrate compass - set current heading as north
        compassNorthOffset = -compassHeadingSmooth;
        prefs.begin("minios", false);
        prefs.putFloat("compassOffset", compassNorthOffset);
        prefs.end();
        navigateTo(currentCategory, currentSubCard);
    }
    else if (currentCategory == CAT_SYSTEM && currentSubCard == 0) {
        // Battery card - tap to go to battery saver modes card
        if (y > LCD_HEIGHT / 2) {
            currentSubCard = 1;  // Go to battery saver modes card
            navigateTo(currentCategory, currentSubCard);
        }
    }
    else if (currentCategory == CAT_SYSTEM && currentSubCard == 1) {
        // Battery Saver Modes card - 3 OPTIONS: Off, Medium, Extreme
        int selectedMode = -1;
        // Updated Y ranges for 3 larger rows (75px each starting at y=55)
        if (y > 55 && y < 130) selectedMode = 0;        // Off (Normal)
        else if (y > 130 && y < 205) selectedMode = 1;  // Medium
        else if (y > 205 && y < 280) selectedMode = 2;  // Extreme
        
        if (selectedMode >= 0 && selectedMode <= 2) {
            applyBatterySaverMode((BatterySaverLevel)selectedMode);
            navigateTo(currentCategory, currentSubCard);
        }
    }
    else if (currentCategory == CAT_SETTINGS && currentSubCard == 0) {
        // Handle settings taps - individual buttons
        // Button layout: each button is 50px tall with 8px gap, starting at y=40
        if (y >= 40 && y < 90) {
            // WATCH FACE button
            userData.watchFaceIndex = (userData.watchFaceIndex + 1) % NUM_WATCH_FACES;
        } 
        else if (y >= 98 && y < 148) {
            // WALLPAPER button
            userData.wallpaperIndex = (userData.wallpaperIndex + 1) % NUM_GRADIENT_WALLPAPERS;
        } 
        else if (y >= 156 && y < 206) {
            // THEME button
            userData.themeIndex = (userData.themeIndex + 1) % NUM_THEMES;
        } 
        else if (y >= 214 && y < 264) {
            // BRIGHTNESS button - tap left to decrease, right to increase
            if (x < LCD_WIDTH / 2) {
                userData.brightness = max(50, userData.brightness - 25);
            } else {
                userData.brightness = min(255, userData.brightness + 25);
            }
            gfx->setBrightness(batterySaverMode ? 50 : userData.brightness);
        } 
        else if (y >= 272 && y < 320) {
            // BATTERY SAVER button
            toggleBatterySaver();
        }
        saveUserData();
        navigateTo(currentCategory, currentSubCard);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BLE STATUS INDICATOR - Shows on non-Nike watch faces
// ═══════════════════════════════════════════════════════════════════════════
void drawBLEIndicator(lv_obj_t *parent) {
    // Only show if BLE is enabled
    if (!bleEnabled) return;
    
    // Create small BLE indicator in top-left corner
    lv_obj_t *bleIcon = lv_label_create(parent);
    lv_label_set_text(bleIcon, LV_SYMBOL_BLUETOOTH);
    
    // Color based on connection status
    if (bleDeviceConnected) {
        lv_obj_set_style_text_color(bleIcon, lv_color_hex(0x0A84FF), 0);  // Blue when connected
    } else {
        lv_obj_set_style_text_color(bleIcon, lv_color_hex(0x636366), 0);  // Gray when searching
    }
    
    lv_obj_set_style_text_font(bleIcon, &lv_font_montserrat_12, 0);
    lv_obj_align(bleIcon, LV_ALIGN_TOP_LEFT, 8, 8);
}

// ═══════════════════════════════════════════════════════════════════════════
// HELPER: DISABLE SCROLLING (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════

// Forward declaration for digital face
void createDigitalFace(lv_obj_t *parent);

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE 1: WORD CLOCK - "IT IS ELEVEN O'CLOCK" STYLE (THEME AWARE)
// ═══════════════════════════════════════════════════════════════════════════
void createWordClockFace(lv_obj_t *parent) {
    GradientTheme *theme = getSafeTheme();
    RTC_DateTime dt = rtc.getDateTime();
    int hour = dt.getHour();
    int minute = dt.getMinute();
    
    // BLE indicator (non-Nike face)
    drawBLEIndicator(parent);
    
    int hour12 = hour % 12;
    if (hour12 == 0) hour12 = 12;
    
    const char* hourWords[] = {"TWELVE", "ONE", "TWO", "THREE", "FOUR", "FIVE", 
                               "SIX", "SEVEN", "EIGHT", "NINE", "TEN", "ELEVEN", "TWELVE"};
    
    // Use theme accent color instead of hardcoded
    uint32_t dimColor = 0x3A3A3A;
    
    int minRounded = (minute + 2) / 5 * 5;
    if (minRounded == 60) minRounded = 0;
    
    // Determine display hour for "TO" cases
    int displayHour = hour12;
    if (minRounded > 30) displayHour = (hour12 % 12) + 1;
    if (displayHour > 12) displayHour = 1;
    
    // IT IS always highlighted
    int yPos = 35;
    lv_obj_t *itLabel = lv_label_create(parent);
    lv_label_set_text(itLabel, "IT IS");
    lv_obj_set_style_text_color(itLabel, theme->accent, 0);
    lv_obj_set_style_text_font(itLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(itLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    
    // HALF / TEN / QUARTER / TWENTY / FIVE for minutes
    yPos += 35;
    if (minRounded == 30) {
        lv_obj_t *halfLabel = lv_label_create(parent);
        lv_label_set_text(halfLabel, "HALF");
        lv_obj_set_style_text_color(halfLabel, theme->accent, 0);
        lv_obj_set_style_text_font(halfLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(halfLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    } else if (minRounded == 15 || minRounded == 45) {
        lv_obj_t *quarterLabel = lv_label_create(parent);
        lv_label_set_text(quarterLabel, "QUARTER");
        lv_obj_set_style_text_color(quarterLabel, theme->accent, 0);
        lv_obj_set_style_text_font(quarterLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(quarterLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    } else if (minRounded == 20 || minRounded == 40) {
        lv_obj_t *twentyLabel = lv_label_create(parent);
        lv_label_set_text(twentyLabel, "TWENTY");
        lv_obj_set_style_text_color(twentyLabel, theme->accent, 0);
        lv_obj_set_style_text_font(twentyLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(twentyLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    } else if (minRounded == 5 || minRounded == 55 || minRounded == 25 || minRounded == 35) {
        if (minRounded == 25 || minRounded == 35) {
            lv_obj_t *twentyLabel = lv_label_create(parent);
            lv_label_set_text(twentyLabel, "TWENTY");
            lv_obj_set_style_text_color(twentyLabel, theme->accent, 0);
            lv_obj_set_style_text_font(twentyLabel, &lv_font_montserrat_16, 0);
            lv_obj_align(twentyLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
            yPos += 35;
        }
        lv_obj_t *fiveLabel = lv_label_create(parent);
        lv_label_set_text(fiveLabel, "FIVE");
        lv_obj_set_style_text_color(fiveLabel, theme->accent, 0);
        lv_obj_set_style_text_font(fiveLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(fiveLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    } else if (minRounded == 10 || minRounded == 50) {
        lv_obj_t *tenLabel = lv_label_create(parent);
        lv_label_set_text(tenLabel, "TEN");
        lv_obj_set_style_text_color(tenLabel, theme->accent, 0);
        lv_obj_set_style_text_font(tenLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(tenLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    }
    
    // PAST / TO
    yPos += 35;
    if (minRounded > 0 && minRounded != 0) {
        const char* pastTo = (minRounded <= 30) ? "PAST" : "TO";
        lv_obj_t *pastLabel = lv_label_create(parent);
        lv_label_set_text(pastLabel, pastTo);
        lv_obj_set_style_text_color(pastLabel, theme->accent, 0);
        lv_obj_set_style_text_font(pastLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(pastLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    }
    
    // Hour word - larger and bold
    yPos += 45;
    lv_obj_t *hourLabel = lv_label_create(parent);
    lv_label_set_text(hourLabel, hourWords[displayHour]);
    lv_obj_set_style_text_color(hourLabel, theme->accent, 0);
    lv_obj_set_style_text_font(hourLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(hourLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    
    // O'CLOCK only at exact hour
    if (minRounded == 0) {
        yPos += 40;
        lv_obj_t *oclockLabel = lv_label_create(parent);
        lv_label_set_text(oclockLabel, "O'CLOCK");
        lv_obj_set_style_text_color(oclockLabel, theme->accent, 0);
        lv_obj_set_style_text_font(oclockLabel, &lv_font_montserrat_24, 0);
        lv_obj_align(oclockLabel, LV_ALIGN_TOP_LEFT, 25, yPos);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE 2: ANALOG WITH ACTIVITY RINGS
// ═══════════════════════════════════════════════════════════════════════════
void createAnalogRingsFace(lv_obj_t *parent) {
    GradientTheme *theme = getSafeTheme();
    RTC_DateTime dt = rtc.getDateTime();
    
    // BLE indicator (non-Nike face)
    drawBLEIndicator(parent);
    
    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    
    // Move ring - Red (outermost)
    lv_obj_t *moveRing = lv_arc_create(parent);
    lv_obj_set_size(moveRing, 220, 220);
    lv_obj_center(moveRing);
    lv_arc_set_rotation(moveRing, 270);
    lv_arc_set_bg_angles(moveRing, 0, 360);
    lv_arc_set_range(moveRing, 0, 100);
    int moveProgress = min(100, (int)(userData.steps * 100 / userData.dailyGoal));
    lv_arc_set_value(moveRing, moveProgress);
    lv_obj_set_style_arc_width(moveRing, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(moveRing, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(moveRing, lv_color_hex(0xFF2D55), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(moveRing, lv_color_hex(0x3A1520), LV_PART_MAIN);
    lv_obj_remove_style(moveRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(moveRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Exercise ring - Green
    lv_obj_t *exerciseRing = lv_arc_create(parent);
    lv_obj_set_size(exerciseRing, 195, 195);
    lv_obj_center(exerciseRing);
    lv_arc_set_rotation(exerciseRing, 270);
    lv_arc_set_bg_angles(exerciseRing, 0, 360);
    lv_arc_set_range(exerciseRing, 0, 100);
    lv_arc_set_value(exerciseRing, 65);
    lv_obj_set_style_arc_width(exerciseRing, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(exerciseRing, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(exerciseRing, lv_color_hex(0xA2FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(exerciseRing, lv_color_hex(0x1A3A00), LV_PART_MAIN);
    lv_obj_remove_style(exerciseRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(exerciseRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Stand ring - Cyan
    lv_obj_t *standRing = lv_arc_create(parent);
    lv_obj_set_size(standRing, 170, 170);
    lv_obj_center(standRing);
    lv_arc_set_rotation(standRing, 270);
    lv_arc_set_bg_angles(standRing, 0, 360);
    lv_arc_set_range(standRing, 0, 12);
    lv_arc_set_value(standRing, 8);
    lv_obj_set_style_arc_width(standRing, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(standRing, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(standRing, lv_color_hex(0x00D4FF), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(standRing, lv_color_hex(0x002A3A), LV_PART_MAIN);
    lv_obj_remove_style(standRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(standRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Hour numbers 12, 3, 6, 9
    int dialRadius = 60;
    int positions[][2] = {{0, -dialRadius}, {dialRadius, 0}, {0, dialRadius}, {-dialRadius, 0}};
    const char* nums[] = {"12", "3", "6", "9"};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *numLabel = lv_label_create(parent);
        lv_label_set_text(numLabel, nums[i]);
        lv_obj_set_style_text_color(numLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(numLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(numLabel, LV_ALIGN_CENTER, positions[i][0], positions[i][1]);
    }
    
    // Clock hands
    int hour = dt.getHour() % 12;
    int minute = dt.getMinute();
    int second = dt.getSecond();
    
    // Hour hand
    int hourLen = 35;
    lv_obj_t *hourHand = lv_obj_create(parent);
    lv_obj_set_size(hourHand, 6, hourLen);
    lv_obj_align(hourHand, LV_ALIGN_CENTER, 0, -hourLen/2);
    lv_obj_set_style_bg_color(hourHand, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(hourHand, 3, 0);
    lv_obj_set_style_border_width(hourHand, 0, 0);
    lv_obj_set_style_transform_pivot_x(hourHand, 3, 0);
    lv_obj_set_style_transform_pivot_y(hourHand, hourLen, 0);
    lv_obj_set_style_transform_angle(hourHand, (int)((hour * 30 + minute * 0.5) * 10), 0);
    disableAllScrolling(hourHand);
    
    // Minute hand
    int minLen = 50;
    lv_obj_t *minHand = lv_obj_create(parent);
    lv_obj_set_size(minHand, 4, minLen);
    lv_obj_align(minHand, LV_ALIGN_CENTER, 0, -minLen/2);
    lv_obj_set_style_bg_color(minHand, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(minHand, 2, 0);
    lv_obj_set_style_border_width(minHand, 0, 0);
    lv_obj_set_style_transform_pivot_x(minHand, 2, 0);
    lv_obj_set_style_transform_pivot_y(minHand, minLen, 0);
    lv_obj_set_style_transform_angle(minHand, minute * 60, 0);
    disableAllScrolling(minHand);
    
    // Second hand (red)
    int secLen = 55;
    lv_obj_t *secHand = lv_obj_create(parent);
    lv_obj_set_size(secHand, 2, secLen);
    lv_obj_align(secHand, LV_ALIGN_CENTER, 0, -secLen/2);
    lv_obj_set_style_bg_color(secHand, lv_color_hex(0xFF2D55), 0);
    lv_obj_set_style_radius(secHand, 1, 0);
    lv_obj_set_style_border_width(secHand, 0, 0);
    lv_obj_set_style_transform_pivot_x(secHand, 1, 0);
    lv_obj_set_style_transform_pivot_y(secHand, secLen, 0);
    lv_obj_set_style_transform_angle(secHand, second * 60, 0);
    disableAllScrolling(secHand);
    
    // Center dot
    lv_obj_t *centerDot = lv_obj_create(parent);
    lv_obj_set_size(centerDot, 10, 10);
    lv_obj_center(centerDot);
    lv_obj_set_style_bg_color(centerDot, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(centerDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(centerDot, 0, 0);
    disableAllScrolling(centerDot);
    
    // Day and date at bottom
    char dateBuf[16];
    const char* dayNames[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
    snprintf(dateBuf, sizeof(dateBuf), "%s %d", dayNames[dt.getWeek()], dt.getDay());
    lv_obj_t *dateLabel = lv_label_create(parent);
    lv_label_set_text(dateLabel, dateBuf);
    lv_obj_set_style_text_color(dateLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dateLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(dateLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE 3: NIKE STYLE - AUTHENTIC APPLE WATCH NIKE FACE
// ═══════════════════════════════════════════════════════════════════════════

// Helper: Draw Nike swoosh - ENHANCED authentic curved tick shape
void drawNikeSwoosh(lv_obj_t *parent, int cx, int cy, int size, uint32_t color) {
    // AUTHENTIC Nike swoosh - simple and clean like reference image
    // Main curved body
    lv_obj_t *swoosh = lv_obj_create(parent);
    lv_obj_set_size(swoosh, size, size / 2.5);
    lv_obj_set_pos(swoosh, cx - size/2, cy);
    lv_obj_set_style_bg_color(swoosh, lv_color_hex(color), 0);
    lv_obj_set_style_radius(swoosh, size / 4, 0);
    lv_obj_set_style_border_width(swoosh, 0, 0);
    lv_obj_set_style_transform_angle(swoosh, -150, 0);
    disableAllScrolling(swoosh);
    
    // Tail pointing up-right
    lv_obj_t *tail = lv_obj_create(parent);
    lv_obj_set_size(tail, size * 0.7, size / 4);
    lv_obj_set_pos(tail, cx + size/5, cy - size/2);
    lv_obj_set_style_bg_color(tail, lv_color_hex(color), 0);
    lv_obj_set_style_radius(tail, size / 8, 0);
    lv_obj_set_style_border_width(tail, 0, 0);
    lv_obj_set_style_transform_angle(tail, 300, 0);
    disableAllScrolling(tail);
}

// Helper: Draw large Nike-style digit with bold italic effect
// Nike Globe font: Bold, condensed, slightly italic, with depth
void drawNikeDigit(lv_obj_t *parent, const char* digit, int x, int y, 
                   uint32_t fillColor, uint32_t shadowColor, int digitSize) {
    // Use largest available font and scale effect
    const lv_font_t *font = &lv_font_montserrat_48;
    
    // Shadow layer (offset for 3D depth effect)
    lv_obj_t *shadow = lv_label_create(parent);
    lv_label_set_text(shadow, digit);
    lv_obj_set_style_text_color(shadow, lv_color_hex(shadowColor), 0);
    lv_obj_set_style_text_font(shadow, font, 0);
    lv_obj_set_pos(shadow, x + 4, y + 4);
    
    // Main digit (front layer)
    lv_obj_t *main = lv_label_create(parent);
    lv_label_set_text(main, digit);
    lv_obj_set_style_text_color(main, lv_color_hex(fillColor), 0);
    lv_obj_set_style_text_font(main, font, 0);
    lv_obj_set_pos(main, x, y);
}

void createNikeStyleFace(lv_obj_t *parent) {
    RTC_DateTime dt = rtc.getDateTime();
    
    // Get current Nike color scheme
    NikeColorScheme colors = nikeColors[currentNikeColor];
    uint32_t textColor = colors.primary;
    uint32_t shadowColor = colors.secondary;
    uint32_t bgColor = colors.background;
    uint32_t swooshColor = colors.swoosh;  // NEW: Get swoosh color from scheme
    bool isFullColor = colors.fullColor;
    bool hasDial = colors.hasDial;          // NEW: Check if variant has dial
    
    // ═══ BACKGROUND ═══
    if (isFullColor) {
        // Solid color background (Volt, Purple, Orange, Pink, White variants)
        lv_obj_set_style_bg_color(parent, lv_color_hex(bgColor), 0);
        lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    } else {
        // Pure black AMOLED background (Blue variant)
        lv_obj_set_style_bg_color(parent, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    }
    
    // Blue variant has circular dial with complications
    // Use hasDial from color scheme (Blue Dial variant)
    bool hasCircularDial = hasDial;
    
    if (hasCircularDial) {
        // ════════════════════════════════════════════════════════════════
        // BLUE DIAL VARIANT - THICK TICK MARKS + BIG NUMBERS (Reference!)
        // ════════════════════════════════════════════════════════════════
        
        int centerX = LCD_WIDTH / 2;
        int centerY = LCD_HEIGHT / 2;
        
        // Outer dial ring - THICKER (was 1px, now 2px)
        lv_obj_t *dialRing = lv_arc_create(parent);
        lv_obj_set_size(dialRing, LCD_WIDTH - 10, LCD_HEIGHT - 10);
        lv_arc_set_rotation(dialRing, 270);
        lv_arc_set_bg_angles(dialRing, 0, 360);
        lv_arc_set_angles(dialRing, 0, 0);
        lv_obj_set_style_arc_color(dialRing, lv_color_hex(0x4A4A4A), LV_PART_MAIN);
        lv_obj_set_style_arc_width(dialRing, 2, LV_PART_MAIN);
        lv_obj_remove_style(dialRing, NULL, LV_PART_INDICATOR);
        lv_obj_remove_style(dialRing, NULL, LV_PART_KNOB);
        lv_obj_center(dialRing);
        
        // Draw ONLY major tick marks (12 ticks instead of 60 for performance!)
        int outerRadius = (LCD_WIDTH - 14) / 2;
        
        for (int i = 0; i < 12; i++) {
            float angle = (i * 30 - 90) * 3.14159 / 180.0;
            int tickLen = 14;
            int tickWidth = 4;
            int innerRadius = outerRadius - tickLen;
            
            int x1 = centerX + (int)(cos(angle) * outerRadius);
            int y1 = centerY + (int)(sin(angle) * outerRadius);
            int x2 = centerX + (int)(cos(angle) * innerRadius);
            int y2 = centerY + (int)(sin(angle) * innerRadius);
            
            lv_obj_t *tick = lv_obj_create(parent);
            lv_obj_set_size(tick, tickWidth, tickLen);
            lv_obj_set_pos(tick, (x1 + x2) / 2 - tickWidth/2, (y1 + y2) / 2 - tickLen / 2);
            lv_obj_set_style_bg_color(tick, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_bg_opa(tick, LV_OPA_90, 0);
            lv_obj_set_style_radius(tick, 1, 0);
            lv_obj_set_style_border_width(tick, 0, 0);
            disableAllScrolling(tick);
        }
        
        // Minute numbers around dial (reduced to 4 for performance)
        const char* dialNums[] = {"15", "30", "45", "00"};
        int dialPositions[] = {90, 180, 270, 0};
        for (int i = 0; i < 4; i++) {
            float angle = (dialPositions[i] - 90) * 3.14159 / 180.0;
            int numRadius = outerRadius - 22;
            int nx = centerX + (int)(cos(angle) * numRadius);
            int ny = centerY + (int)(sin(angle) * numRadius);
            
            lv_obj_t *numLabel = lv_label_create(parent);
            lv_label_set_text(numLabel, dialNums[i]);
            lv_obj_set_style_text_color(numLabel, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(numLabel, &lv_font_montserrat_12, 0);
            lv_obj_set_pos(numLabel, nx - 8, ny - 6);
        }
        
        // Date numbers on right side (19, 20, 21 for calendar)
        int dayNum = dt.getDay();
        char dateBuf[4];
        for (int i = -1; i <= 1; i++) {
            snprintf(dateBuf, sizeof(dateBuf), "%d", dayNum + i);
            lv_obj_t *dateLabel = lv_label_create(parent);
            lv_label_set_text(dateLabel, dateBuf);
            uint32_t dateColor = (i == 0) ? 0xFF3B30 : 0x8E8E93;  // Today = red
            lv_obj_set_style_text_color(dateLabel, lv_color_hex(dateColor), 0);
            lv_obj_set_style_text_font(dateLabel, &lv_font_montserrat_12, 0);
            lv_obj_align(dateLabel, LV_ALIGN_RIGHT_MID, -20, -20 + (i * 18));
        }
        
        // Temperature - top left
        char tempBuf[8];
        snprintf(tempBuf, sizeof(tempBuf), "%d°", (int)weatherTemp);
        lv_obj_t *tempLabel = lv_label_create(parent);
        lv_label_set_text(tempLabel, tempBuf);
        lv_obj_set_style_text_color(tempLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(tempLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(tempLabel, LV_ALIGN_TOP_LEFT, 25, 22);
        
        // Day name - top right
        const char* shortDays[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
        lv_obj_t *dayLabel = lv_label_create(parent);
        lv_label_set_text(dayLabel, shortDays[dt.getWeek()]);
        lv_obj_set_style_text_color(dayLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(dayLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(dayLabel, LV_ALIGN_TOP_RIGHT, -25, 22);
        
        // "TAP TO OPEN" - top center
        lv_obj_t *tapText = lv_label_create(parent);
        lv_label_set_text(tapText, "TAP TO OPEN");
        lv_obj_set_style_text_color(tapText, lv_color_hex(0x6E6E73), 0);
        lv_obj_set_style_text_font(tapText, &lv_font_montserrat_12, 0);
        lv_obj_align(tapText, LV_ALIGN_TOP_MID, 0, 55);
        
        // Nike swoosh - LEFT OF CENTER, clearly visible (like reference!)
        drawNikeSwoosh(parent, centerX - 55, centerY - 35, 30, swooshColor);
        
        // Activity arc indicators - THICKER! (8px instead of 4px)
        lv_obj_t *arcLeft = lv_arc_create(parent);
        lv_obj_set_size(arcLeft, LCD_WIDTH - 35, LCD_HEIGHT - 35);
        lv_arc_set_rotation(arcLeft, 195);
        lv_arc_set_bg_angles(arcLeft, 0, 55);
        lv_arc_set_angles(arcLeft, 0, 40);
        lv_obj_set_style_arc_color(arcLeft, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
        lv_obj_set_style_arc_color(arcLeft, lv_color_hex(textColor), LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(arcLeft, 8, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arcLeft, 8, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(arcLeft, true, LV_PART_INDICATOR);
        lv_obj_remove_style(arcLeft, NULL, LV_PART_KNOB);
        lv_obj_center(arcLeft);
        
        lv_obj_t *arcBottom = lv_arc_create(parent);
        lv_obj_set_size(arcBottom, LCD_WIDTH - 35, LCD_HEIGHT - 35);
        lv_arc_set_rotation(arcBottom, 250);
        lv_arc_set_bg_angles(arcBottom, 0, 45);
        lv_arc_set_angles(arcBottom, 0, 35);
        lv_obj_set_style_arc_color(arcBottom, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
        lv_obj_set_style_arc_color(arcBottom, lv_color_hex(textColor), LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(arcBottom, 8, LV_PART_MAIN);
        lv_obj_set_style_arc_width(arcBottom, 8, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(arcBottom, true, LV_PART_INDICATOR);
        lv_obj_remove_style(arcBottom, NULL, LV_PART_KNOB);
        lv_obj_center(arcBottom);
        
        // Bottom left stats (steps, distance, calories) - BIGGER font
        char statsBuf[20];
        snprintf(statsBuf, sizeof(statsBuf), "%d • %02d • %02d", 
                (int)(userData.totalDistance * 10) % 100,
                (int)userData.totalCalories % 100,
                (int)(userData.totalCalories / 10) % 100);
        lv_obj_t *statsLabel = lv_label_create(parent);
        lv_label_set_text(statsLabel, statsBuf);
        lv_obj_set_style_text_color(statsLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(statsLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(statsLabel, LV_ALIGN_BOTTOM_LEFT, 22, -22);
        
        // Battery percentage - bottom right - BIGGER
        char battBuf[8];
        snprintf(battBuf, sizeof(battBuf), "%d%%", batteryPercent);
        lv_obj_t *battLabel = lv_label_create(parent);
        lv_label_set_text(battLabel, battBuf);
        lv_obj_set_style_text_color(battLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(battLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(battLabel, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
        
    } else {
        // ════════════════════════════════════════════════════════════════
        // FULL COLOR VARIANTS (Volt, Purple, Orange, Pink, White)
        // Simple, bold time with Nike swoosh - like reference images
        // ════════════════════════════════════════════════════════════════
        
        // Nike swoosh at bottom - use swoosh color from scheme (BIGGER swoosh!)
        drawNikeSwoosh(parent, LCD_WIDTH/2, LCD_HEIGHT - 50, 45, swooshColor);
        
        // "RUNNING TODAY?" text
        lv_obj_t *runText = lv_label_create(parent);
        lv_label_set_text(runText, "RUNNING TODAY?");
        lv_obj_set_style_text_color(runText, lv_color_hex(textColor), 0);
        lv_obj_set_style_text_opa(runText, LV_OPA_60, 0);
        lv_obj_set_style_text_font(runText, &lv_font_montserrat_12, 0);
        lv_obj_align(runText, LV_ALIGN_BOTTOM_MID, 0, -95);
    }
    
    // ════════════════════════════════════════════════════════════════
    // MAIN TIME - HUGE NIKE STYLE DIGITS (OPTIMIZED - less objects!)
    // ════════════════════════════════════════════════════════════════
    
    // Format time
    int displayHour = use24HourFormat ? dt.getHour() : (dt.getHour() % 12);
    if (!use24HourFormat && displayHour == 0) displayHour = 12;
    
    char hourBuf[4], minBuf[4];
    snprintf(hourBuf, sizeof(hourBuf), "%02d", displayHour);
    snprintf(minBuf, sizeof(minBuf), "%02d", dt.getMinute());
    
    // Position: Stacked in center
    int hourY = hasCircularDial ? -45 : -55;
    int minY = hasCircularDial ? 35 : 45;
    
    // ═══ HOUR - SIMPLE 4-LAYER SHADOW (was 48 layers - too slow!) ═══
    // Shadow layers - just 4 directions for performance
    int shadowOffsets[4][2] = {{-3, -3}, {3, -3}, {-3, 3}, {3, 3}};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *shadow = lv_label_create(parent);
        lv_label_set_text(shadow, hourBuf);
        lv_obj_set_style_text_color(shadow, lv_color_hex(0x000033), 0);
        lv_obj_set_style_text_font(shadow, &NIKE_FONT, 0);
        lv_obj_set_style_text_opa(shadow, LV_OPA_70, 0);
        lv_obj_align(shadow, LV_ALIGN_CENTER, shadowOffsets[i][0], hourY + shadowOffsets[i][1]);
    }
    // Main hour
    lv_obj_t *hourMain = lv_label_create(parent);
    lv_label_set_text(hourMain, hourBuf);
    lv_obj_set_style_text_color(hourMain, lv_color_hex(textColor), 0);
    lv_obj_set_style_text_font(hourMain, &NIKE_FONT, 0);
    lv_obj_align(hourMain, LV_ALIGN_CENTER, 0, hourY);
    
    // ═══ MINUTE - SIMPLE 4-LAYER SHADOW ═══
    for (int i = 0; i < 4; i++) {
        lv_obj_t *shadow = lv_label_create(parent);
        lv_label_set_text(shadow, minBuf);
        lv_obj_set_style_text_color(shadow, lv_color_hex(0x000033), 0);
        lv_obj_set_style_text_font(shadow, &NIKE_FONT, 0);
        lv_obj_set_style_text_opa(shadow, LV_OPA_70, 0);
        lv_obj_align(shadow, LV_ALIGN_CENTER, shadowOffsets[i][0], minY + shadowOffsets[i][1]);
    }
    // Main minute
    lv_obj_t *minMain = lv_label_create(parent);
    lv_label_set_text(minMain, minBuf);
    lv_obj_set_style_text_color(minMain, lv_color_hex(textColor), 0);
    lv_obj_set_style_text_font(minMain, &NIKE_FONT, 0);
    lv_obj_align(minMain, LV_ALIGN_CENTER, 0, minY);
    
    // Color variant indicator (non-dial variants only)
    if (!hasCircularDial) {
        lv_obj_t *colorHint = lv_label_create(parent);
        char hintBuf[24];
        snprintf(hintBuf, sizeof(hintBuf), "< %s >", colors.name);
        lv_label_set_text(colorHint, hintBuf);
        lv_obj_set_style_text_color(colorHint, lv_color_hex(textColor), 0);
        lv_obj_set_style_text_font(colorHint, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_opa(colorHint, LV_OPA_70, 0);
        lv_obj_align(colorHint, LV_ALIGN_BOTTOM_MID, 0, -8);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE 4: MINIMAL DARK
// ═══════════════════════════════════════════════════════════════════════════
void createMinimalDarkFace(lv_obj_t *parent) {
    GradientTheme *theme = getSafeTheme();
    RTC_DateTime dt = rtc.getDateTime();
    
    // BLE indicator (non-Nike face)
    drawBLEIndicator(parent);
    
    // ═══ ULTRA-MINIMAL PREMIUM FACE ═══
    
    // Subtle corner accent line
    lv_obj_t *accentLine = lv_obj_create(parent);
    lv_obj_set_size(accentLine, 3, 60);
    lv_obj_align(accentLine, LV_ALIGN_TOP_LEFT, 20, 40);
    lv_obj_set_style_bg_color(accentLine, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_radius(accentLine, 2, 0);
    lv_obj_set_style_border_width(accentLine, 0, 0);
    disableAllScrolling(accentLine);
    
    // Day name - all caps, subtle
    const char* dayNames[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
    lv_obj_t *dayLabel = lv_label_create(parent);
    lv_label_set_text(dayLabel, dayNames[dt.getWeek()]);
    lv_obj_set_style_text_color(dayLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(dayLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(dayLabel, LV_ALIGN_TOP_LEFT, 32, 45);
    
    // Date number - large accent
    char dateBuf[4];
    snprintf(dateBuf, sizeof(dateBuf), "%d", dt.getDay());
    lv_obj_t *dateLabel = lv_label_create(parent);
    lv_label_set_text(dateLabel, dateBuf);
    lv_obj_set_style_text_color(dateLabel, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(dateLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(dateLabel, LV_ALIGN_TOP_LEFT, 32, 62);
    
    // MASSIVE time - left aligned for impact
    char hourBuf[4];
    snprintf(hourBuf, sizeof(hourBuf), "%02d", dt.getHour());
    
    lv_obj_t *hourLabel = lv_label_create(parent);
    lv_label_set_text(hourLabel, hourBuf);
    lv_obj_set_style_text_color(hourLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(hourLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(hourLabel, LV_ALIGN_LEFT_MID, 25, -15);
    
    char minBuf[4];
    snprintf(minBuf, sizeof(minBuf), "%02d", dt.getMinute());
    
    lv_obj_t *minLabel = lv_label_create(parent);
    lv_label_set_text(minLabel, minBuf);
    lv_obj_set_style_text_color(minLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(minLabel, LV_OPA_60, 0);
    lv_obj_set_style_text_font(minLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(minLabel, LV_ALIGN_LEFT_MID, 25, 40);
    
    // Seconds - small, floating
    char secBuf[4];
    snprintf(secBuf, sizeof(secBuf), "%02d", dt.getSecond());
    lv_obj_t *secLabel = lv_label_create(parent);
    lv_label_set_text(secLabel, secBuf);
    lv_obj_set_style_text_color(secLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(secLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(secLabel, LV_ALIGN_LEFT_MID, 115, 55);
    
    // Right side - minimal stats
    // Battery pill
    lv_obj_t *battPill = lv_obj_create(parent);
    lv_obj_set_size(battPill, 50, 24);
    lv_obj_align(battPill, LV_ALIGN_RIGHT_MID, -20, -40);
    lv_obj_set_style_bg_color(battPill, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(battPill, 12, 0);
    lv_obj_set_style_border_width(battPill, 0, 0);
    disableAllScrolling(battPill);
    
    char battBuf[8];
    snprintf(battBuf, sizeof(battBuf), "%d%%", batteryPercent);
    lv_obj_t *battLabel = lv_label_create(battPill);
    lv_label_set_text(battLabel, battBuf);
    uint32_t battColor = batteryPercent > 20 ? 0x30D158 : 0xFF3B30;
    lv_obj_set_style_text_color(battLabel, lv_color_hex(battColor), 0);
    lv_obj_set_style_text_font(battLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(battLabel);
    
    // Weather temp
    if (wifiConnected) {
        lv_obj_t *tempPill = lv_obj_create(parent);
        lv_obj_set_size(tempPill, 50, 24);
        lv_obj_align(tempPill, LV_ALIGN_RIGHT_MID, -20, 0);
        lv_obj_set_style_bg_color(tempPill, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(tempPill, 12, 0);
        lv_obj_set_style_border_width(tempPill, 0, 0);
        disableAllScrolling(tempPill);
        
        char tempBuf[8];
        snprintf(tempBuf, sizeof(tempBuf), "%.0f°", weatherTemp);
        lv_obj_t *tempLabel = lv_label_create(tempPill);
        lv_label_set_text(tempLabel, tempBuf);
        lv_obj_set_style_text_color(tempLabel, lv_color_hex(0x5AC8FA), 0);
        lv_obj_set_style_text_font(tempLabel, &lv_font_montserrat_12, 0);
        lv_obj_center(tempLabel);
    }
    
    // Moon Phase & Sunrise/Sunset - bottom row
    if (astroDataSynced) {
        // Moon phase (left)
        lv_obj_t *moonLabel = lv_label_create(parent);
        char moonBuf[24];
        snprintf(moonBuf, sizeof(moonBuf), "%s", moonPhaseName.c_str());
        lv_label_set_text(moonLabel, moonBuf);
        lv_obj_set_style_text_color(moonLabel, lv_color_hex(0xFFD60A), 0);
        lv_obj_set_style_text_font(moonLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(moonLabel, LV_ALIGN_BOTTOM_RIGHT, -15, -40);
        
        // Sunrise/Sunset (right)
        lv_obj_t *sunLabel = lv_label_create(parent);
        char sunBuf[24];
        snprintf(sunBuf, sizeof(sunBuf), "%s/%s", sunriseTime.c_str(), sunsetTime.c_str());
        lv_label_set_text(sunLabel, sunBuf);
        lv_obj_set_style_text_color(sunLabel, lv_color_hex(0xFF9F0A), 0);
        lv_obj_set_style_text_font(sunLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(sunLabel, LV_ALIGN_BOTTOM_RIGHT, -15, -25);
    }
    
    // Steps - bottom
    char stepBuf[16];
    snprintf(stepBuf, sizeof(stepBuf), "%lu", (unsigned long)userData.steps);
    lv_obj_t *stepLabel = lv_label_create(parent);
    lv_label_set_text(stepLabel, stepBuf);
    lv_obj_set_style_text_color(stepLabel, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(stepLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(stepLabel, LV_ALIGN_BOTTOM_LEFT, 25, -25);
    
    lv_obj_t *stepsWord = lv_label_create(parent);
    lv_label_set_text(stepsWord, "steps");
    lv_obj_set_style_text_color(stepsWord, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(stepsWord, &lv_font_montserrat_12, 0);
    lv_obj_align(stepsWord, LV_ALIGN_BOTTOM_LEFT, 25, -12);
}

// ═══════════════════════════════════════════════════════════════════════════
// WATCH FACE 5: FITNESS RINGS - ACTIVITY FOCUSED
// ═══════════════════════════════════════════════════════════════════════════
void createFitnessRingsFace(lv_obj_t *parent) {
    GradientTheme *theme = getSafeTheme();
    RTC_DateTime dt = rtc.getDateTime();
    
    // BLE indicator (non-Nike face)
    drawBLEIndicator(parent);
    
    int ringCenterX = 80;
    int ringCenterY = LCD_HEIGHT / 2;
    
    // Move ring - Red
    lv_obj_t *moveRing = lv_arc_create(parent);
    lv_obj_set_size(moveRing, 130, 130);
    lv_obj_set_pos(moveRing, ringCenterX - 65, ringCenterY - 65);
    lv_arc_set_rotation(moveRing, 270);
    lv_arc_set_bg_angles(moveRing, 0, 360);
    lv_arc_set_range(moveRing, 0, 100);
    int moveProgress = min(100, (int)(userData.steps * 100 / userData.dailyGoal));
    lv_arc_set_value(moveRing, moveProgress);
    lv_obj_set_style_arc_width(moveRing, 15, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(moveRing, 15, LV_PART_MAIN);
    lv_obj_set_style_arc_color(moveRing, lv_color_hex(0xFF2D55), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(moveRing, lv_color_hex(0x3A1520), LV_PART_MAIN);
    lv_obj_remove_style(moveRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(moveRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Exercise ring - Green
    lv_obj_t *exerciseRing = lv_arc_create(parent);
    lv_obj_set_size(exerciseRing, 95, 95);
    lv_obj_set_pos(exerciseRing, ringCenterX - 47, ringCenterY - 47);
    lv_arc_set_rotation(exerciseRing, 270);
    lv_arc_set_bg_angles(exerciseRing, 0, 360);
    lv_arc_set_range(exerciseRing, 0, 100);
    lv_arc_set_value(exerciseRing, 72);
    lv_obj_set_style_arc_width(exerciseRing, 15, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(exerciseRing, 15, LV_PART_MAIN);
    lv_obj_set_style_arc_color(exerciseRing, lv_color_hex(0xA2FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(exerciseRing, lv_color_hex(0x1A3A00), LV_PART_MAIN);
    lv_obj_remove_style(exerciseRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(exerciseRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Stand ring - Cyan
    lv_obj_t *standRing = lv_arc_create(parent);
    lv_obj_set_size(standRing, 60, 60);
    lv_obj_set_pos(standRing, ringCenterX - 30, ringCenterY - 30);
    lv_arc_set_rotation(standRing, 270);
    lv_arc_set_bg_angles(standRing, 0, 360);
    lv_arc_set_range(standRing, 0, 12);
    lv_arc_set_value(standRing, 9);
    lv_obj_set_style_arc_width(standRing, 15, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(standRing, 15, LV_PART_MAIN);
    lv_obj_set_style_arc_color(standRing, lv_color_hex(0x00D4FF), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(standRing, lv_color_hex(0x002A3A), LV_PART_MAIN);
    lv_obj_remove_style(standRing, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(standRing, LV_OBJ_FLAG_CLICKABLE);
    
    // Time on right side
    char timeBuf[8];
    int hour12 = dt.getHour() % 12;
    if (hour12 == 0) hour12 = 12;
    snprintf(timeBuf, sizeof(timeBuf), "%d:%02d", hour12, dt.getMinute());
    
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(timeLabel, LV_ALIGN_TOP_RIGHT, -25, 70);
    
    // Calorie count (red)
    char calBuf[16];
    snprintf(calBuf, sizeof(calBuf), "%d", (int)userData.totalCalories);
    lv_obj_t *calLabel = lv_label_create(parent);
    lv_label_set_text(calLabel, calBuf);
    lv_obj_set_style_text_color(calLabel, lv_color_hex(0xFF2D55), 0);
    lv_obj_set_style_text_font(calLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(calLabel, LV_ALIGN_TOP_RIGHT, -25, 120);
    
    // Exercise minutes (green)
    lv_obj_t *exLabel = lv_label_create(parent);
    lv_label_set_text(exLabel, "32");
    lv_obj_set_style_text_color(exLabel, lv_color_hex(0xA2FF00), 0);
    lv_obj_set_style_text_font(exLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(exLabel, LV_ALIGN_TOP_RIGHT, -25, 155);
    
    // Stand hours (cyan)
    lv_obj_t *standLabel = lv_label_create(parent);
    lv_label_set_text(standLabel, "9");
    lv_obj_set_style_text_color(standLabel, lv_color_hex(0x00D4FF), 0);
    lv_obj_set_style_text_font(standLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(standLabel, LV_ALIGN_TOP_RIGHT, -25, 190);
    
    // Icons at top
    lv_obj_t *musicIcon = lv_label_create(parent);
    lv_label_set_text(musicIcon, LV_SYMBOL_AUDIO);
    lv_obj_set_style_text_color(musicIcon, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(musicIcon, LV_ALIGN_TOP_RIGHT, -60, 25);
    
    lv_obj_t *cloudIcon = lv_label_create(parent);
    lv_label_set_text(cloudIcon, LV_SYMBOL_DOWNLOAD);
    lv_obj_set_style_text_color(cloudIcon, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(cloudIcon, LV_ALIGN_TOP_RIGHT, -25, 25);
}

// ═══════════════════════════════════════════════════════════════════════════
// DIGITAL FACE - ORIGINAL STYLE (REFACTORED INTO FUNCTION)
// ═══════════════════════════════════════════════════════════════════════════
void createDigitalFace(lv_obj_t *parent) {
    GradientTheme *theme = getSafeTheme();
    RTC_DateTime dt = rtc.getDateTime();
    
    // BLE indicator (non-Nike face)
    drawBLEIndicator(parent);
    
    // ═══ PREMIUM APPLE WATCH DIGITAL FACE ═══
    
    // Large time display - centered, massive, clean
    char timeBuf[10];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", dt.getHour(), dt.getMinute());

    // Subtle glow effect behind time
    lv_obj_t *timeGlow = lv_label_create(parent);
    lv_label_set_text(timeGlow, timeBuf);
    lv_obj_set_style_text_color(timeGlow, theme->accent, 0);
    lv_obj_set_style_text_opa(timeGlow, LV_OPA_20, 0);
    lv_obj_set_style_text_font(timeGlow, &lv_font_montserrat_48, 0);
    lv_obj_align(timeGlow, LV_ALIGN_CENTER, 2, -22);

    // Main time - crisp white
    lv_obj_t *clockLabel = lv_label_create(parent);
    lv_label_set_text(clockLabel, timeBuf);
    lv_obj_set_style_text_color(clockLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(clockLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(clockLabel, LV_ALIGN_CENTER, 0, -25);

    // Seconds - small accent badge
    char secBuf[4];
    snprintf(secBuf, sizeof(secBuf), "%02d", dt.getSecond());
    
    lv_obj_t *secBadge = lv_obj_create(parent);
    lv_obj_set_size(secBadge, 36, 24);
    lv_obj_align(secBadge, LV_ALIGN_CENTER, 75, -25);
    lv_obj_set_style_bg_color(secBadge, theme->accent, 0);
    lv_obj_set_style_bg_opa(secBadge, LV_OPA_30, 0);
    lv_obj_set_style_radius(secBadge, 12, 0);
    lv_obj_set_style_border_width(secBadge, 0, 0);
    disableAllScrolling(secBadge);
    
    lv_obj_t *secLabel = lv_label_create(secBadge);
    lv_label_set_text(secLabel, secBuf);
    lv_obj_set_style_text_color(secLabel, theme->accent, 0);
    lv_obj_set_style_text_font(secLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(secLabel);

    // Day and date - elegant top positioning
    const char* dayNames[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
    const char* monthNames[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
    
    // Day name - subtle
    lv_obj_t *dayLabel = lv_label_create(parent);
    lv_label_set_text(dayLabel, dayNames[dt.getWeek()]);
    lv_obj_set_style_text_color(dayLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(dayLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(dayLabel, LV_ALIGN_TOP_MID, 0, 35);

    // Date badge - pill style
    char dateBuf[12];
    snprintf(dateBuf, sizeof(dateBuf), "%s %d", monthNames[dt.getMonth()-1], dt.getDay());
    
    lv_obj_t *dateBadge = lv_obj_create(parent);
    lv_obj_set_size(dateBadge, 70, 24);
    lv_obj_align(dateBadge, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_bg_color(dateBadge, lv_color_hex(0xFF3B30), 0);
    lv_obj_set_style_radius(dateBadge, 12, 0);
    lv_obj_set_style_border_width(dateBadge, 0, 0);
    disableAllScrolling(dateBadge);
    
    lv_obj_t *dateLabel = lv_label_create(dateBadge);
    lv_label_set_text(dateLabel, dateBuf);
    lv_obj_set_style_text_color(dateLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dateLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(dateLabel);

    // ═══ COMPLICATIONS BAR - Bottom ═══
    
    // Left complication - Steps with ring
    lv_obj_t *stepsComplication = lv_obj_create(parent);
    lv_obj_set_size(stepsComplication, 70, 70);
    lv_obj_align(stepsComplication, LV_ALIGN_BOTTOM_LEFT, 15, -15);
    lv_obj_set_style_bg_color(stepsComplication, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(stepsComplication, 35, 0);
    lv_obj_set_style_border_width(stepsComplication, 0, 0);
    disableAllScrolling(stepsComplication);
    
    // Steps progress arc
    int stepProgress = min(100, (int)(userData.steps * 100 / userData.dailyGoal));
    lv_obj_t *stepArc = lv_arc_create(stepsComplication);
    lv_obj_set_size(stepArc, 65, 65);
    lv_obj_center(stepArc);
    lv_arc_set_rotation(stepArc, 270);
    lv_arc_set_bg_angles(stepArc, 0, 360);
    lv_arc_set_range(stepArc, 0, 100);
    lv_arc_set_value(stepArc, stepProgress);
    lv_obj_set_style_arc_width(stepArc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(stepArc, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_color(stepArc, lv_color_hex(0x30D158), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(stepArc, lv_color_hex(0x2C2C2E), LV_PART_MAIN);
    lv_obj_remove_style(stepArc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(stepArc, LV_OBJ_FLAG_CLICKABLE);
    
    char stepBuf[8];
    snprintf(stepBuf, sizeof(stepBuf), "%lu", (unsigned long)(userData.steps / 1000));
    lv_obj_t *stepNum = lv_label_create(stepsComplication);
    lv_label_set_text(stepNum, stepBuf);
    lv_obj_set_style_text_color(stepNum, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(stepNum, &lv_font_montserrat_16, 0);
    lv_obj_align(stepNum, LV_ALIGN_CENTER, 0, -5);
    
    lv_obj_t *stepK = lv_label_create(stepsComplication);
    lv_label_set_text(stepK, "K");
    lv_obj_set_style_text_color(stepK, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(stepK, &lv_font_montserrat_12, 0);
    lv_obj_align(stepK, LV_ALIGN_CENTER, 0, 12);

    // Center complication - Battery
    lv_obj_t *battComplication = lv_obj_create(parent);
    lv_obj_set_size(battComplication, 60, 60);
    lv_obj_align(battComplication, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(battComplication, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(battComplication, 30, 0);
    lv_obj_set_style_border_width(battComplication, 0, 0);
    disableAllScrolling(battComplication);
    
    // Battery arc
    lv_obj_t *battArc = lv_arc_create(battComplication);
    lv_obj_set_size(battArc, 55, 55);
    lv_obj_center(battArc);
    lv_arc_set_rotation(battArc, 270);
    lv_arc_set_bg_angles(battArc, 0, 360);
    lv_arc_set_range(battArc, 0, 100);
    lv_arc_set_value(battArc, batteryPercent);
    lv_obj_set_style_arc_width(battArc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(battArc, 4, LV_PART_MAIN);
    uint32_t battColor = batteryPercent > 20 ? 0xFFD60A : 0xFF3B30;
    lv_obj_set_style_arc_color(battArc, lv_color_hex(battColor), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(battArc, lv_color_hex(0x2C2C2E), LV_PART_MAIN);
    lv_obj_remove_style(battArc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(battArc, LV_OBJ_FLAG_CLICKABLE);
    
    char battBuf[5];
    snprintf(battBuf, sizeof(battBuf), "%d", batteryPercent);
    lv_obj_t *battNum = lv_label_create(battComplication);
    lv_label_set_text(battNum, battBuf);
    lv_obj_set_style_text_color(battNum, lv_color_hex(battColor), 0);
    lv_obj_set_style_text_font(battNum, &lv_font_montserrat_16, 0);
    lv_obj_align(battNum, LV_ALIGN_CENTER, 0, -3);
    
    lv_obj_t *battPct = lv_label_create(battComplication);
    lv_label_set_text(battPct, "%");
    lv_obj_set_style_text_color(battPct, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(battPct, &lv_font_montserrat_12, 0);
    lv_obj_align(battPct, LV_ALIGN_CENTER, 0, 12);

    // Right complication - WiFi/Weather
    lv_obj_t *wifiComplication = lv_obj_create(parent);
    lv_obj_set_size(wifiComplication, 70, 70);
    lv_obj_align(wifiComplication, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
    lv_obj_set_style_bg_color(wifiComplication, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(wifiComplication, 35, 0);
    lv_obj_set_style_border_width(wifiComplication, 0, 0);
    disableAllScrolling(wifiComplication);
    
    // WiFi icon or temp
    if (wifiConnected) {
        char tempBuf[8];
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", weatherTemp);
        lv_obj_t *tempLabel = lv_label_create(wifiComplication);
        lv_label_set_text(tempLabel, tempBuf);
        lv_obj_set_style_text_color(tempLabel, lv_color_hex(0x5AC8FA), 0);
        lv_obj_set_style_text_font(tempLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(tempLabel, LV_ALIGN_CENTER, 0, -5);
        
        lv_obj_t *degLabel = lv_label_create(wifiComplication);
        lv_label_set_text(degLabel, "C");
        lv_obj_set_style_text_color(degLabel, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(degLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(degLabel, LV_ALIGN_CENTER, 0, 12);
    } else {
        lv_obj_t *wifiIcon = lv_label_create(wifiComplication);
        lv_label_set_text(wifiIcon, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(wifiIcon, lv_color_hex(0xFF3B30), 0);
        lv_obj_set_style_text_font(wifiIcon, &lv_font_montserrat_24, 0);
        lv_obj_center(wifiIcon);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PHOTO CLOCKFACE FUNCTIONS - SD Card Image Loading & Display
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Scan /WATCH/PHOTOS/ for .bin and .raw image files (RGB565, 360x440)
 */
void scanSDPhotos() {
    numSDPhotos = 0;
    if (!hasSD) return;

    if (!SD_MMC.exists(SD_PHOTOS_PATH)) {
        SD_MMC.mkdir(SD_PHOTOS_PATH);
        USBSerial.println("[PHOTO] Created /WATCH/PHOTOS/ folder");
        return;
    }

    File dir = SD_MMC.open(SD_PHOTOS_PATH);
    if (!dir || !dir.isDirectory()) return;

    File file = dir.openNextFile();
    while (file && numSDPhotos < MAX_SD_PHOTOS) {
        String name = String(file.name());
        // Accept .bin and .raw extensions (case insensitive)
        if (name.endsWith(".bin") || name.endsWith(".raw") ||
            name.endsWith(".BIN") || name.endsWith(".RAW")) {
            sdPhotoFiles[numSDPhotos] = String(SD_PHOTOS_PATH) + "/" + name;
            USBSerial.printf("[PHOTO] Found: %s (%lu bytes)\n",
                            sdPhotoFiles[numSDPhotos].c_str(), (unsigned long)file.size());
            numSDPhotos++;
        }
        file = dir.openNextFile();
    }
    dir.close();

    if (currentPhotoIndex >= numSDPhotos) {
        currentPhotoIndex = 0;
    }
    USBSerial.printf("[PHOTO] Total photos: %d\n", numSDPhotos);
}

/**
 * Load a photo from SD card into PSRAM buffer for LVGL display
 * Returns true if successful
 */
bool loadSDPhoto(int index) {
    if (index < 0 || index >= numSDPhotos || !hasSD) return false;

    uint32_t imgSize = LCD_WIDTH * LCD_HEIGHT * 2;  // RGB565 = 2 bytes per pixel

    // Allocate buffer in PSRAM if not done yet
    if (!photoBuf) {
        photoBuf = (uint8_t *)heap_caps_malloc(imgSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!photoBuf) {
            // Fallback to regular RAM (unlikely to fit but try)
            photoBuf = (uint8_t *)malloc(imgSize);
        }
        if (!photoBuf) {
            USBSerial.println("[PHOTO] Failed to allocate image buffer!");
            return false;
        }
        USBSerial.printf("[PHOTO] Allocated %lu byte buffer\n", (unsigned long)imgSize);
    }

    File f = SD_MMC.open(sdPhotoFiles[index].c_str(), FILE_READ);
    if (!f) {
        USBSerial.printf("[PHOTO] Failed to open: %s\n", sdPhotoFiles[index].c_str());
        return false;
    }

    size_t bytesRead = f.read(photoBuf, imgSize);
    f.close();

    if (bytesRead < imgSize) {
        USBSerial.printf("[PHOTO] Warning: read %lu of %lu bytes from %s\n",
                        (unsigned long)bytesRead, (unsigned long)imgSize,
                        sdPhotoFiles[index].c_str());
        // Zero-fill remainder if file is shorter
        if (bytesRead > 0) {
            memset(photoBuf + bytesRead, 0, imgSize - bytesRead);
        } else {
            return false;
        }
    }

    // Setup LVGL image descriptor
    photoImgDsc.header.always_zero = 0;
    photoImgDsc.header.w = LCD_WIDTH;
    photoImgDsc.header.h = LCD_HEIGHT;
    photoImgDsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    photoImgDsc.data_size = imgSize;
    photoImgDsc.data = photoBuf;

    photoLoaded = true;
    USBSerial.printf("[PHOTO] Loaded: %s (%d/%d)\n",
                    sdPhotoFiles[index].c_str(), index + 1, numSDPhotos);
    return true;
}

/**
 * WATCH FACE 7: PHOTO - Full-screen SD card image with Nike font time overlay
 * Left tap = cycle photos, Right tap = cycle watch faces
 * Fallback: active gradient theme when no photos available
 */
void createPhotoFace(lv_obj_t *parent) {
    // Try to load current photo from SD card
    bool hasPhoto = false;
    if (numSDPhotos > 0) {
        hasPhoto = loadSDPhoto(currentPhotoIndex);
    }

    if (hasPhoto) {
        // Display the photo as full-screen background
        lv_obj_t *img = lv_img_create(parent);
        lv_img_set_src(img, &photoImgDsc);
        lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    }
    // If no photo, the gradient theme background from createClockCard() shows through

    // ═══ TIME OVERLAY - Nike font, light blue, top center ═══
    // Space-separated format: "18 15" (no colon - not in Nike font)
    char timeBuf[16];
    RTC_DateTime dt = rtc.getDateTime();
    int hr, mn;
    if (use24HourFormat) {
        hr = dt.getHour();
    } else {
        hr = dt.getHour() % 12;
        if (hr == 0) hr = 12;
    }
    mn = dt.getMinute();
    sprintf(timeBuf, "%02d %02d", hr, mn);

    // Text shadow for readability on bright photos
    lv_obj_t *timeShadow = lv_label_create(parent);
    lv_label_set_text(timeShadow, timeBuf);
    lv_obj_set_style_text_color(timeShadow, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_opa(timeShadow, LV_OPA_60, 0);
    lv_obj_set_style_text_font(timeShadow, &NIKE_FONT, 0);
    lv_obj_align(timeShadow, LV_ALIGN_TOP_MID, 2, 32);

    // Main time label - light blue (#82CFFF)
    lv_obj_t *timeLabel = lv_label_create(parent);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0x82CFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &NIKE_FONT, 0);
    lv_obj_align(timeLabel, LV_ALIGN_TOP_MID, 0, 30);

    // Photo counter indicator (small, bottom right)
    if (numSDPhotos > 0) {
        char countBuf[12];
        snprintf(countBuf, sizeof(countBuf), "%d/%d", currentPhotoIndex + 1, numSDPhotos);
        lv_obj_t *countLabel = lv_label_create(parent);
        lv_label_set_text(countLabel, countBuf);
        lv_obj_set_style_text_color(countLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(countLabel, LV_OPA_50, 0);
        lv_obj_set_style_text_font(countLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(countLabel, LV_ALIGN_BOTTOM_RIGHT, -10, -25);
    } else {
        // No photos hint
        lv_obj_t *noPhotoHint = lv_label_create(parent);
        lv_label_set_text(noPhotoHint, "Add photos to\n/WATCH/PHOTOS/");
        lv_obj_set_style_text_color(noPhotoHint, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(noPhotoHint, LV_OPA_40, 0);
        lv_obj_set_style_text_font(noPhotoHint, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(noPhotoHint, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(noPhotoHint, LV_ALIGN_CENTER, 0, 40);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CLOCK CARD - DISPATCHES TO SELECTED WATCH FACE
// ═══════════════════════════════════════════════════════════════════════════
void createClockCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    // Background with wallpaper support
    lv_obj_t *bgCard = lv_obj_create(lv_scr_act());
    lv_obj_set_size(bgCard, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_align(bgCard, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(bgCard, 0, 0);
    lv_obj_set_style_border_width(bgCard, 0, 0);
    lv_obj_set_style_pad_all(bgCard, 0, 0);
    disableAllScrolling(bgCard);

    // ═══════════════════════════════════════════════════════════════════════
    // PREMIUM CUSTOM WALLPAPER BACKGROUNDS - BETTER THAN APPLE!
    // ═══════════════════════════════════════════════════════════════════════
    if (userData.wallpaperIndex > 0 && userData.wallpaperIndex < NUM_GRADIENT_WALLPAPERS) {
        WallpaperTheme *wp = &gradientWallpapers[userData.wallpaperIndex];

        // Base gradient
        lv_obj_set_style_bg_color(bgCard, wp->top, 0);
        lv_obj_set_style_bg_grad_color(bgCard, wp->bottom, 0);
        lv_obj_set_style_bg_grad_dir(bgCard, LV_GRAD_DIR_VER, 0);

        // ═══ WALLPAPER 1: MOUNTAIN SUNSET - Epic mountain scene ═══
        if (userData.wallpaperIndex == 1) {
            // Sky gradient layers
            lv_obj_t *skyMid = lv_obj_create(bgCard);
            lv_obj_set_size(skyMid, LCD_WIDTH, LCD_HEIGHT / 2);
            lv_obj_align(skyMid, LV_ALIGN_TOP_MID, 0, 0);
            lv_obj_set_style_bg_color(skyMid, lv_color_hex(0x4A90D9), 0);
            lv_obj_set_style_bg_grad_color(skyMid, lv_color_hex(0xFF7F50), 0);
            lv_obj_set_style_bg_grad_dir(skyMid, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(skyMid, 0, 0);
            lv_obj_set_style_radius(skyMid, 0, 0);
            disableAllScrolling(skyMid);
            
            // Sun with glow
            lv_obj_t *sunGlow = lv_obj_create(bgCard);
            lv_obj_set_size(sunGlow, 80, 80);
            lv_obj_align(sunGlow, LV_ALIGN_CENTER, 40, -30);
            lv_obj_set_style_bg_color(sunGlow, lv_color_hex(0xFFD700), 0);
            lv_obj_set_style_bg_opa(sunGlow, LV_OPA_30, 0);
            lv_obj_set_style_radius(sunGlow, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sunGlow, 0, 0);
            disableAllScrolling(sunGlow);
            
            lv_obj_t *sun = lv_obj_create(bgCard);
            lv_obj_set_size(sun, 40, 40);
            lv_obj_align(sun, LV_ALIGN_CENTER, 40, -30);
            lv_obj_set_style_bg_color(sun, lv_color_hex(0xFFE4B5), 0);
            lv_obj_set_style_radius(sun, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sun, 0, 0);
            disableAllScrolling(sun);
            
            // Far mountains (darkest, back layer)
            lv_obj_t *farMtn = lv_obj_create(bgCard);
            lv_obj_set_size(farMtn, LCD_WIDTH + 40, 120);
            lv_obj_align(farMtn, LV_ALIGN_BOTTOM_MID, 0, -60);
            lv_obj_set_style_bg_color(farMtn, lv_color_hex(0x4A3728), 0);
            lv_obj_set_style_border_width(farMtn, 0, 0);
            lv_obj_set_style_radius(farMtn, 60, 0);
            disableAllScrolling(farMtn);
            
            // Mid mountains
            lv_obj_t *midMtn = lv_obj_create(bgCard);
            lv_obj_set_size(midMtn, 180, 140);
            lv_obj_align(midMtn, LV_ALIGN_BOTTOM_LEFT, -30, -20);
            lv_obj_set_style_bg_color(midMtn, lv_color_hex(0x5D4037), 0);
            lv_obj_set_style_bg_grad_color(midMtn, lv_color_hex(0x8D6E63), 0);
            lv_obj_set_style_bg_grad_dir(midMtn, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(midMtn, 0, 0);
            lv_obj_set_style_radius(midMtn, 80, 0);
            disableAllScrolling(midMtn);
            
            // Close mountain (lighter, front)
            lv_obj_t *closeMtn = lv_obj_create(bgCard);
            lv_obj_set_size(closeMtn, 160, 160);
            lv_obj_align(closeMtn, LV_ALIGN_BOTTOM_RIGHT, 20, 0);
            lv_obj_set_style_bg_color(closeMtn, lv_color_hex(0x6D4C41), 0);
            lv_obj_set_style_bg_grad_color(closeMtn, lv_color_hex(0xA1887F), 0);
            lv_obj_set_style_bg_grad_dir(closeMtn, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(closeMtn, 0, 0);
            lv_obj_set_style_radius(closeMtn, 80, 0);
            disableAllScrolling(closeMtn);
            
            // Snow caps
            lv_obj_t *snow1 = lv_obj_create(bgCard);
            lv_obj_set_size(snow1, 50, 25);
            lv_obj_align(snow1, LV_ALIGN_BOTTOM_LEFT, 35, -130);
            lv_obj_set_style_bg_color(snow1, lv_color_hex(0xFFFAF0), 0);
            lv_obj_set_style_border_width(snow1, 0, 0);
            lv_obj_set_style_radius(snow1, 12, 0);
            disableAllScrolling(snow1);
            
            lv_obj_t *snow2 = lv_obj_create(bgCard);
            lv_obj_set_size(snow2, 40, 20);
            lv_obj_align(snow2, LV_ALIGN_BOTTOM_RIGHT, -45, -140);
            lv_obj_set_style_bg_color(snow2, lv_color_hex(0xFFFAF0), 0);
            lv_obj_set_style_border_width(snow2, 0, 0);
            lv_obj_set_style_radius(snow2, 10, 0);
            disableAllScrolling(snow2);
        }
        
        // ═══ WALLPAPER 2: GOLDEN PEAKS - Sunrise gold mountains ═══
        else if (userData.wallpaperIndex == 2) {
            // Warm sunrise sky
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0xFF6B35), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x1A0A00), 0);
            
            // Golden sun rays
            for (int i = 0; i < 8; i++) {
                lv_obj_t *ray = lv_obj_create(bgCard);
                lv_obj_set_size(ray, 8, 200);
                lv_obj_align(ray, LV_ALIGN_TOP_MID, 30, -50);
                lv_obj_set_style_bg_color(ray, lv_color_hex(0xFFD700), 0);
                lv_obj_set_style_bg_opa(ray, LV_OPA_20, 0);
                lv_obj_set_style_radius(ray, 4, 0);
                lv_obj_set_style_border_width(ray, 0, 0);
                lv_obj_set_style_transform_angle(ray, i * 200 - 700, 0);
                disableAllScrolling(ray);
            }
            
            // Sun disk
            lv_obj_t *sun = lv_obj_create(bgCard);
            lv_obj_set_size(sun, 60, 60);
            lv_obj_align(sun, LV_ALIGN_TOP_MID, 30, 20);
            lv_obj_set_style_bg_color(sun, lv_color_hex(0xFFE066), 0);
            lv_obj_set_style_radius(sun, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sun, 0, 0);
            lv_obj_set_style_shadow_width(sun, 40, 0);
            lv_obj_set_style_shadow_color(sun, lv_color_hex(0xFFD700), 0);
            lv_obj_set_style_shadow_opa(sun, LV_OPA_60, 0);
            disableAllScrolling(sun);
            
            // Layered golden mountains
            lv_obj_t *mtn1 = lv_obj_create(bgCard);
            lv_obj_set_size(mtn1, 200, 180);
            lv_obj_align(mtn1, LV_ALIGN_BOTTOM_MID, -60, 20);
            lv_obj_set_style_bg_color(mtn1, lv_color_hex(0x3D2817), 0);
            lv_obj_set_style_border_width(mtn1, 0, 0);
            lv_obj_set_style_radius(mtn1, 100, 0);
            disableAllScrolling(mtn1);
            
            lv_obj_t *mtn2 = lv_obj_create(bgCard);
            lv_obj_set_size(mtn2, 180, 200);
            lv_obj_align(mtn2, LV_ALIGN_BOTTOM_RIGHT, 30, 30);
            lv_obj_set_style_bg_color(mtn2, lv_color_hex(0x4A3520), 0);
            lv_obj_set_style_border_width(mtn2, 0, 0);
            lv_obj_set_style_radius(mtn2, 90, 0);
            disableAllScrolling(mtn2);
            
            // Golden rim light on peaks
            lv_obj_t *rimLight = lv_obj_create(bgCard);
            lv_obj_set_size(rimLight, 150, 10);
            lv_obj_align(rimLight, LV_ALIGN_BOTTOM_MID, -60, -140);
            lv_obj_set_style_bg_color(rimLight, lv_color_hex(0xFFD700), 0);
            lv_obj_set_style_bg_opa(rimLight, LV_OPA_70, 0);
            lv_obj_set_style_border_width(rimLight, 0, 0);
            lv_obj_set_style_radius(rimLight, 5, 0);
            disableAllScrolling(rimLight);
        }
        
        // ═══ WALLPAPER 3: CANYON DAWN - Pink desert canyon ═══
        else if (userData.wallpaperIndex == 3) {
            // Pink/orange sky
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0x87CEEB), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x8B4513), 0);
            
            // Gradient overlay for sunset effect
            lv_obj_t *sunsetGlow = lv_obj_create(bgCard);
            lv_obj_set_size(sunsetGlow, LCD_WIDTH, LCD_HEIGHT / 2);
            lv_obj_align(sunsetGlow, LV_ALIGN_CENTER, 0, -20);
            lv_obj_set_style_bg_color(sunsetGlow, lv_color_hex(0xFFB6C1), 0);
            lv_obj_set_style_bg_grad_color(sunsetGlow, lv_color_hex(0xFF6347), 0);
            lv_obj_set_style_bg_grad_dir(sunsetGlow, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(sunsetGlow, LV_OPA_60, 0);
            lv_obj_set_style_border_width(sunsetGlow, 0, 0);
            lv_obj_set_style_radius(sunsetGlow, 0, 0);
            disableAllScrolling(sunsetGlow);
            
            // Canyon walls - left
            lv_obj_t *canyonL = lv_obj_create(bgCard);
            lv_obj_set_size(canyonL, 100, LCD_HEIGHT);
            lv_obj_align(canyonL, LV_ALIGN_LEFT_MID, -30, 0);
            lv_obj_set_style_bg_color(canyonL, lv_color_hex(0xCD5C5C), 0);
            lv_obj_set_style_bg_grad_color(canyonL, lv_color_hex(0x8B0000), 0);
            lv_obj_set_style_bg_grad_dir(canyonL, LV_GRAD_DIR_HOR, 0);
            lv_obj_set_style_border_width(canyonL, 0, 0);
            lv_obj_set_style_radius(canyonL, 20, 0);
            disableAllScrolling(canyonL);
            
            // Canyon walls - right
            lv_obj_t *canyonR = lv_obj_create(bgCard);
            lv_obj_set_size(canyonR, 100, LCD_HEIGHT);
            lv_obj_align(canyonR, LV_ALIGN_RIGHT_MID, 30, 0);
            lv_obj_set_style_bg_color(canyonR, lv_color_hex(0x8B0000), 0);
            lv_obj_set_style_bg_grad_color(canyonR, lv_color_hex(0xCD5C5C), 0);
            lv_obj_set_style_bg_grad_dir(canyonR, LV_GRAD_DIR_HOR, 0);
            lv_obj_set_style_border_width(canyonR, 0, 0);
            lv_obj_set_style_radius(canyonR, 20, 0);
            disableAllScrolling(canyonR);
            
            // Distant mesa
            lv_obj_t *mesa = lv_obj_create(bgCard);
            lv_obj_set_size(mesa, 120, 60);
            lv_obj_align(mesa, LV_ALIGN_CENTER, 0, 50);
            lv_obj_set_style_bg_color(mesa, lv_color_hex(0xA0522D), 0);
            lv_obj_set_style_border_width(mesa, 0, 0);
            lv_obj_set_style_radius(mesa, 8, 0);
            disableAllScrolling(mesa);
        }
        
        // ═══ WALLPAPER 4: ISLAND PARADISE - Tropical sunset ═══
        else if (userData.wallpaperIndex == 4) {
            // Purple/pink sky
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0xE6B3CC), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x006994), 0);
            
            // Setting sun
            lv_obj_t *sunGlow = lv_obj_create(bgCard);
            lv_obj_set_size(sunGlow, 120, 120);
            lv_obj_align(sunGlow, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_bg_color(sunGlow, lv_color_hex(0xFF69B4), 0);
            lv_obj_set_style_bg_opa(sunGlow, LV_OPA_30, 0);
            lv_obj_set_style_radius(sunGlow, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sunGlow, 0, 0);
            disableAllScrolling(sunGlow);
            
            lv_obj_t *sun = lv_obj_create(bgCard);
            lv_obj_set_size(sun, 50, 50);
            lv_obj_align(sun, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_bg_color(sun, lv_color_hex(0xFFB6C1), 0);
            lv_obj_set_style_radius(sun, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sun, 0, 0);
            disableAllScrolling(sun);
            
            // Ocean
            lv_obj_t *ocean = lv_obj_create(bgCard);
            lv_obj_set_size(ocean, LCD_WIDTH, LCD_HEIGHT / 3);
            lv_obj_align(ocean, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(ocean, lv_color_hex(0x4169E1), 0);
            lv_obj_set_style_bg_grad_color(ocean, lv_color_hex(0x006994), 0);
            lv_obj_set_style_bg_grad_dir(ocean, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(ocean, 0, 0);
            lv_obj_set_style_radius(ocean, 0, 0);
            disableAllScrolling(ocean);
            
            // Sun reflection on water
            lv_obj_t *reflection = lv_obj_create(bgCard);
            lv_obj_set_size(reflection, 8, 80);
            lv_obj_align(reflection, LV_ALIGN_BOTTOM_MID, 0, -20);
            lv_obj_set_style_bg_color(reflection, lv_color_hex(0xFFB6C1), 0);
            lv_obj_set_style_bg_opa(reflection, LV_OPA_50, 0);
            lv_obj_set_style_border_width(reflection, 0, 0);
            lv_obj_set_style_radius(reflection, 4, 0);
            disableAllScrolling(reflection);
            
            // Palm tree silhouette - left
            lv_obj_t *palmTrunk = lv_obj_create(bgCard);
            lv_obj_set_size(palmTrunk, 8, 100);
            lv_obj_align(palmTrunk, LV_ALIGN_BOTTOM_LEFT, 40, -40);
            lv_obj_set_style_bg_color(palmTrunk, lv_color_hex(0x1A1A2E), 0);
            lv_obj_set_style_border_width(palmTrunk, 0, 0);
            lv_obj_set_style_radius(palmTrunk, 4, 0);
            lv_obj_set_style_transform_angle(palmTrunk, 100, 0);
            disableAllScrolling(palmTrunk);
            
            // Palm leaves
            for (int i = 0; i < 5; i++) {
                lv_obj_t *leaf = lv_obj_create(bgCard);
                lv_obj_set_size(leaf, 40, 8);
                lv_obj_align(leaf, LV_ALIGN_BOTTOM_LEFT, 50, -120);
                lv_obj_set_style_bg_color(leaf, lv_color_hex(0x1A1A2E), 0);
                lv_obj_set_style_border_width(leaf, 0, 0);
                lv_obj_set_style_radius(leaf, 4, 0);
                lv_obj_set_style_transform_angle(leaf, (i - 2) * 250, 0);
                disableAllScrolling(leaf);
            }
        }
        
        // ═══ WALLPAPER 5: ALPINE MEADOW - Green mountains ═══
        else if (userData.wallpaperIndex == 5) {
            // Blue sky with golden glow
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0x87CEEB), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x228B22), 0);
            
            // Sun
            lv_obj_t *sun = lv_obj_create(bgCard);
            lv_obj_set_size(sun, 35, 35);
            lv_obj_align(sun, LV_ALIGN_TOP_RIGHT, -30, 30);
            lv_obj_set_style_bg_color(sun, lv_color_hex(0xFFD700), 0);
            lv_obj_set_style_radius(sun, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(sun, 0, 0);
            lv_obj_set_style_shadow_width(sun, 20, 0);
            lv_obj_set_style_shadow_color(sun, lv_color_hex(0xFFD700), 0);
            disableAllScrolling(sun);
            
            // Distant blue mountain
            lv_obj_t *distMtn = lv_obj_create(bgCard);
            lv_obj_set_size(distMtn, LCD_WIDTH + 40, 100);
            lv_obj_align(distMtn, LV_ALIGN_CENTER, 0, 20);
            lv_obj_set_style_bg_color(distMtn, lv_color_hex(0x6B8E9F), 0);
            lv_obj_set_style_border_width(distMtn, 0, 0);
            lv_obj_set_style_radius(distMtn, 50, 0);
            disableAllScrolling(distMtn);
            
            // Green hills
            lv_obj_t *hill1 = lv_obj_create(bgCard);
            lv_obj_set_size(hill1, 200, 140);
            lv_obj_align(hill1, LV_ALIGN_BOTTOM_LEFT, -40, 20);
            lv_obj_set_style_bg_color(hill1, lv_color_hex(0x32CD32), 0);
            lv_obj_set_style_bg_grad_color(hill1, lv_color_hex(0x228B22), 0);
            lv_obj_set_style_bg_grad_dir(hill1, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(hill1, 0, 0);
            lv_obj_set_style_radius(hill1, 100, 0);
            disableAllScrolling(hill1);
            
            lv_obj_t *hill2 = lv_obj_create(bgCard);
            lv_obj_set_size(hill2, 180, 120);
            lv_obj_align(hill2, LV_ALIGN_BOTTOM_RIGHT, 30, 10);
            lv_obj_set_style_bg_color(hill2, lv_color_hex(0x3CB371), 0);
            lv_obj_set_style_bg_grad_color(hill2, lv_color_hex(0x2E8B57), 0);
            lv_obj_set_style_bg_grad_dir(hill2, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(hill2, 0, 0);
            lv_obj_set_style_radius(hill2, 90, 0);
            disableAllScrolling(hill2);
            
            // Flowers (yellow dots)
            for (int i = 0; i < 8; i++) {
                lv_obj_t *flower = lv_obj_create(bgCard);
                lv_obj_set_size(flower, 6, 6);
                lv_obj_align(flower, LV_ALIGN_BOTTOM_MID, (i - 4) * 20, -40 - (i % 3) * 15);
                lv_obj_set_style_bg_color(flower, lv_color_hex(0xFFD700), 0);
                lv_obj_set_style_radius(flower, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_border_width(flower, 0, 0);
                disableAllScrolling(flower);
            }
        }
        
        // ═══ WALLPAPER 6: TWILIGHT OCEAN - Night sea ═══
        else if (userData.wallpaperIndex == 6) {
            // Deep night sky
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0x0D0D1A), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x1A1A3A), 0);
            
            // Stars
            for (int i = 0; i < 20; i++) {
                lv_obj_t *star = lv_obj_create(bgCard);
                int size = (i % 3) + 2;
                lv_obj_set_size(star, size, size);
                lv_obj_set_pos(star, (i * 37) % LCD_WIDTH, (i * 23) % (LCD_HEIGHT / 2));
                lv_obj_set_style_bg_color(star, lv_color_hex(0xFFFFFF), 0);
                lv_obj_set_style_bg_opa(star, LV_OPA_40 + (i % 4) * 20, 0);
                lv_obj_set_style_radius(star, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_border_width(star, 0, 0);
                disableAllScrolling(star);
            }
            
            // Moon
            lv_obj_t *moonGlow = lv_obj_create(bgCard);
            lv_obj_set_size(moonGlow, 70, 70);
            lv_obj_align(moonGlow, LV_ALIGN_TOP_RIGHT, -25, 35);
            lv_obj_set_style_bg_color(moonGlow, lv_color_hex(0xE6E6FA), 0);
            lv_obj_set_style_bg_opa(moonGlow, LV_OPA_20, 0);
            lv_obj_set_style_radius(moonGlow, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(moonGlow, 0, 0);
            disableAllScrolling(moonGlow);
            
            lv_obj_t *moon = lv_obj_create(bgCard);
            lv_obj_set_size(moon, 35, 35);
            lv_obj_align(moon, LV_ALIGN_TOP_RIGHT, -25, 40);
            lv_obj_set_style_bg_color(moon, lv_color_hex(0xFFFACD), 0);
            lv_obj_set_style_radius(moon, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(moon, 0, 0);
            disableAllScrolling(moon);
            
            // Ocean waves
            lv_obj_t *ocean = lv_obj_create(bgCard);
            lv_obj_set_size(ocean, LCD_WIDTH, LCD_HEIGHT / 2 + 20);
            lv_obj_align(ocean, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(ocean, lv_color_hex(0x191970), 0);
            lv_obj_set_style_bg_grad_color(ocean, lv_color_hex(0x000033), 0);
            lv_obj_set_style_bg_grad_dir(ocean, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_border_width(ocean, 0, 0);
            lv_obj_set_style_radius(ocean, 0, 0);
            disableAllScrolling(ocean);
            
            // Moon reflection
            lv_obj_t *moonReflect = lv_obj_create(bgCard);
            lv_obj_set_size(moonReflect, 6, 100);
            lv_obj_align(moonReflect, LV_ALIGN_BOTTOM_RIGHT, -40, -20);
            lv_obj_set_style_bg_color(moonReflect, lv_color_hex(0xFFFACD), 0);
            lv_obj_set_style_bg_opa(moonReflect, LV_OPA_30, 0);
            lv_obj_set_style_border_width(moonReflect, 0, 0);
            lv_obj_set_style_radius(moonReflect, 3, 0);
            disableAllScrolling(moonReflect);
            
            // Wave lines
            for (int i = 0; i < 4; i++) {
                lv_obj_t *wave = lv_obj_create(bgCard);
                lv_obj_set_size(wave, LCD_WIDTH - 40, 3);
                lv_obj_align(wave, LV_ALIGN_BOTTOM_MID, 0, -30 - i * 25);
                lv_obj_set_style_bg_color(wave, lv_color_hex(0x4169E1), 0);
                lv_obj_set_style_bg_opa(wave, LV_OPA_30 - i * 5, 0);
                lv_obj_set_style_border_width(wave, 0, 0);
                lv_obj_set_style_radius(wave, 2, 0);
                disableAllScrolling(wave);
            }
        }
    } else {
        // Solid theme gradient (no wallpaper)
        lv_obj_set_style_bg_color(bgCard, theme->color1, 0);
        lv_obj_set_style_bg_grad_color(bgCard, theme->color2, 0);
        lv_obj_set_style_bg_grad_dir(bgCard, LV_GRAD_DIR_VER, 0);
    }

    // Main card container (transparent overlay for content)
    lv_obj_t *card = lv_obj_create(bgCard);
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    disableAllScrolling(card);

    // ═══════════════════════════════════════════════════════════════════════
    // DISPATCH TO SELECTED WATCH FACE
    // ═══════════════════════════════════════════════════════════════════════
    
    // Nike face requires pure black background
    if (userData.watchFaceIndex == 3) {
        lv_obj_set_style_bg_color(bgCard, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(bgCard, LV_OPA_COVER, 0);
    }

    // Photo face: black bg when photo loaded (photo covers it), gradient theme when no photos
    if (userData.watchFaceIndex == 6) {
        scanSDPhotos();  // Re-scan for new photos each time face renders
        if (numSDPhotos > 0) {
            lv_obj_set_style_bg_color(bgCard, lv_color_hex(0x000000), 0);
            lv_obj_set_style_bg_grad_color(bgCard, lv_color_hex(0x000000), 0);
            lv_obj_set_style_bg_opa(bgCard, LV_OPA_COVER, 0);
        }
    }
    
    switch (userData.watchFaceIndex) {
        case 0:
            createDigitalFace(card);
            break;
        case 1:
            createWordClockFace(card);
            break;
        case 2:
            createAnalogRingsFace(card);
            break;
        case 3:
            createNikeStyleFace(card);
            break;
        case 4:
            createMinimalDarkFace(card);
            break;
        case 5:
            createFitnessRingsFace(card);
            break;
        case 6:
            createPhotoFace(card);
            break;
        default:
            createDigitalFace(card);
            break;
    }
    
    // Add hint for Time Settings access (swipe down)
    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, LV_SYMBOL_DOWN " Set Time");
    lv_obj_set_style_text_color(hint, theme->secondary, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_opa(hint, 180, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// ═══════════════════════════════════════════════════════════════════════════
// COMPASS CARD - APPLE WATCH STYLE WITH SUNRISE/SUNSET
// ═══════════════════════════════════════════════════════════════════════════
void createCompassCard() {
    disableAllScrolling(lv_scr_act());
    calculateSunTimes();

    // Pure black background - Apple Watch style
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    disableAllScrolling(card);

    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    int outerRadius = 125;  // BIGGER COMPASS - was 95, now 125

    // Calibrated heading
    float calibratedHeading = compassHeadingSmooth - compassNorthOffset;
    if (calibratedHeading < 0) calibratedHeading += 360;
    if (calibratedHeading >= 360) calibratedHeading -= 360;
    float headingRad = calibratedHeading * M_PI / 180.0;

    // ═══ OUTER TICK MARKS - Rotate with compass ═══
    for (int deg = 0; deg < 360; deg += 6) {
        float actualDeg = deg - calibratedHeading;
        float rad = actualDeg * M_PI / 180.0;
        
        bool isCardinal = (deg == 0 || deg == 90 || deg == 180 || deg == 270);
        bool isMajor = (deg % 30 == 0);
        int tickLen = isCardinal ? 20 : (isMajor ? 14 : 7);  // Bigger ticks
        int tickWidth = isCardinal ? 4 : (isMajor ? 3 : 2);
        
        int outerR = outerRadius;
        int innerR = outerRadius - tickLen;
        
        int x1 = centerX + (int)(outerR * sin(rad));
        int y1 = centerY - (int)(outerR * cos(rad));
        int x2 = centerX + (int)(innerR * sin(rad));
        int y2 = centerY - (int)(innerR * cos(rad));

        lv_obj_t *tick = lv_obj_create(card);
        lv_obj_set_size(tick, tickWidth, tickLen);
        lv_obj_align(tick, LV_ALIGN_CENTER, (int)(innerR * sin(rad)), -(int)(innerR * cos(rad)) - tickLen/2);
        
        uint32_t tickColor = isCardinal ? 0xFFFFFF : (isMajor ? 0x8E8E93 : 0x48484A);
        lv_obj_set_style_bg_color(tick, lv_color_hex(tickColor), 0);
        lv_obj_set_style_radius(tick, 0, 0);
        lv_obj_set_style_border_width(tick, 0, 0);
        lv_obj_set_style_transform_pivot_x(tick, tickWidth/2, 0);
        lv_obj_set_style_transform_pivot_y(tick, tickLen/2, 0);
        lv_obj_set_style_transform_angle(tick, (int)(actualDeg * 10), 0);
        disableAllScrolling(tick);
    }

    // ═══ CARDINAL DIRECTIONS - Rotate with compass ═══
    const char* cardinals[] = {"N", "E", "S", "W"};
    int cardinalDegs[] = {0, 90, 180, 270};
    uint32_t cardinalColors[] = {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF};
    
    for (int i = 0; i < 4; i++) {
        float actualDeg = cardinalDegs[i] - calibratedHeading;
        float rad = actualDeg * M_PI / 180.0;
        int labelR = outerRadius - 38;  // Adjusted for bigger compass
        
        int lx = centerX + (int)(labelR * sin(rad));
        int ly = centerY - (int)(labelR * cos(rad));
        
        lv_obj_t *cardLabel = lv_label_create(card);
        lv_label_set_text(cardLabel, cardinals[i]);
        lv_obj_set_style_text_color(cardLabel, lv_color_hex(cardinalColors[i]), 0);
        lv_obj_set_style_text_font(cardLabel, &lv_font_montserrat_24, 0);  // Bigger font
        lv_obj_set_pos(cardLabel, lx - 10, ly - 14);
    }

    // ═══ COMPASS NEEDLE - Apple Watch Red/Blue style ═══
    // Red needle pointing UP (North)
    int needleLen = 75;  // Bigger needle - was 55
    int needleWidth = 14;  // Slightly wider
    
    // Red (North) half - pointing up
    lv_obj_t *redNeedle = lv_obj_create(card);
    lv_obj_set_size(redNeedle, needleWidth, needleLen);
    lv_obj_align(redNeedle, LV_ALIGN_CENTER, 0, -needleLen/2);
    lv_obj_set_style_bg_color(redNeedle, lv_color_hex(0xFF3B30), 0);  // Apple red
    lv_obj_set_style_radius(redNeedle, 2, 0);
    lv_obj_set_style_border_width(redNeedle, 0, 0);
    disableAllScrolling(redNeedle);

    // Pointy tip for red needle
    lv_obj_t *redTip = lv_obj_create(card);
    lv_obj_set_size(redTip, needleWidth + 8, 22);
    lv_obj_align(redTip, LV_ALIGN_CENTER, 0, -needleLen + 6);
    lv_obj_set_style_bg_color(redTip, lv_color_hex(0xFF3B30), 0);
    lv_obj_set_style_radius(redTip, 2, 0);
    lv_obj_set_style_border_width(redTip, 0, 0);
    disableAllScrolling(redTip);
    
    // Blue (South) half - pointing down
    lv_obj_t *blueNeedle = lv_obj_create(card);
    lv_obj_set_size(blueNeedle, needleWidth, needleLen);
    lv_obj_align(blueNeedle, LV_ALIGN_CENTER, 0, needleLen/2);
    lv_obj_set_style_bg_color(blueNeedle, lv_color_hex(0x007AFF), 0);  // Apple blue
    lv_obj_set_style_radius(blueNeedle, 2, 0);
    lv_obj_set_style_border_width(blueNeedle, 0, 0);
    disableAllScrolling(blueNeedle);

    // Pointy tip for blue needle
    lv_obj_t *blueTip = lv_obj_create(card);
    lv_obj_set_size(blueTip, needleWidth + 8, 22);
    lv_obj_align(blueTip, LV_ALIGN_CENTER, 0, needleLen - 6);
    lv_obj_set_style_bg_color(blueTip, lv_color_hex(0x007AFF), 0);
    lv_obj_set_style_radius(blueTip, 2, 0);
    lv_obj_set_style_border_width(blueTip, 0, 0);
    disableAllScrolling(blueTip);

    // Center white dot
    lv_obj_t *centerDot = lv_obj_create(card);
    lv_obj_set_size(centerDot, 22, 22);
    lv_obj_center(centerDot);
    lv_obj_set_style_bg_color(centerDot, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(centerDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(centerDot, 0, 0);
    disableAllScrolling(centerDot);

    // Heading display at top
    char headingBuf[16];
    snprintf(headingBuf, sizeof(headingBuf), "%d°", (int)calibratedHeading);
    lv_obj_t *headingLabel = lv_label_create(card);
    lv_label_set_text(headingLabel, headingBuf);
    lv_obj_set_style_text_color(headingLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(headingLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(headingLabel, LV_ALIGN_TOP_MID, 0, 8);

    // Calibration hint
    lv_obj_t *calibHint = lv_label_create(card);
    lv_label_set_text(calibHint, "TAP TO CALIBRATE");
    lv_obj_set_style_text_color(calibHint, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_text_font(calibHint, &lv_font_montserrat_12, 0);
    lv_obj_align(calibHint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// ═══════════════════════════════════════════════════════════════════════════
// TILT CARD
// ═══════════════════════════════════════════════════════════════════════════
void createTiltCard() {
    GradientTheme *theme = getSafeTheme();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, theme->color1, 0);
    lv_obj_set_style_bg_grad_color(card, theme->color2, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "TILT SENSOR");
    lv_obj_set_style_text_color(title, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    char tiltStr[32];
    snprintf(tiltStr, sizeof(tiltStr), "X: %.1f°", tiltX);
    lv_obj_t *tiltXLbl = lv_label_create(card);
    lv_label_set_text(tiltXLbl, tiltStr);
    lv_obj_set_style_text_color(tiltXLbl, theme->text, 0);
    lv_obj_set_style_text_font(tiltXLbl, &lv_font_montserrat_28, 0);
    lv_obj_align(tiltXLbl, LV_ALIGN_CENTER, 0, -30);

    snprintf(tiltStr, sizeof(tiltStr), "Y: %.1f°", tiltY);
    lv_obj_t *tiltYLbl = lv_label_create(card);
    lv_label_set_text(tiltYLbl, tiltStr);
    lv_obj_set_style_text_color(tiltYLbl, theme->text, 0);
    lv_obj_set_style_text_font(tiltYLbl, &lv_font_montserrat_28, 0);
    lv_obj_align(tiltYLbl, LV_ALIGN_CENTER, 0, 30);

    lv_obj_t *levelLbl = lv_label_create(card);
    bool isLevel = (abs(tiltX) < 3 && abs(tiltY) < 3);
    lv_label_set_text(levelLbl, isLevel ? "LEVEL" : "TILTED");
    lv_obj_set_style_text_color(levelLbl, isLevel ? lv_color_hex(0x34C759) : lv_color_hex(0xFF9500), 0);
    lv_obj_align(levelLbl, LV_ALIGN_BOTTOM_MID, 0, -40);
}

// ═══════════════════════════════════════════════════════════════════════════
// STEPS CARD - USBO GRADIENT STYLE (Purple/Cyan like reference image)
// ═══════════════════════════════════════════════════════════════════════════
void createStepsCard() {
    disableAllScrolling(lv_scr_act());

    // Deep dark background for contrast
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D1A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Main gradient card - USBO style with purple/cyan gradient feel
    lv_obj_t *mainCard = lv_obj_create(card);
    lv_obj_set_size(mainCard, LCD_WIDTH - 24, LCD_HEIGHT - 40);
    lv_obj_align(mainCard, LV_ALIGN_CENTER, 0, 0);
    // Use a purple-blue gradient base color
    lv_obj_set_style_bg_color(mainCard, lv_color_hex(0x6B4CE6), 0);  // Rich purple
    lv_obj_set_style_bg_grad_color(mainCard, lv_color_hex(0x00D4FF), 0);  // Cyan
    lv_obj_set_style_bg_grad_dir(mainCard, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(mainCard, 28, 0);
    lv_obj_set_style_border_width(mainCard, 0, 0);
    lv_obj_set_style_shadow_width(mainCard, 30, 0);
    lv_obj_set_style_shadow_color(mainCard, lv_color_hex(0x6B4CE6), 0);
    lv_obj_set_style_shadow_opa(mainCard, LV_OPA_40, 0);
    disableAllScrolling(mainCard);

    // App title badge - top left (like USBO App)
    lv_obj_t *badge = lv_obj_create(mainCard);
    lv_obj_set_size(badge, 100, 30);
    lv_obj_align(badge, LV_ALIGN_TOP_LEFT, 16, 16);
    lv_obj_set_style_bg_color(badge, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(badge, LV_OPA_20, 0);
    lv_obj_set_style_radius(badge, 8, 0);
    lv_obj_set_style_border_width(badge, 0, 0);
    disableAllScrolling(badge);

    lv_obj_t *badgeIcon = lv_label_create(badge);
    lv_label_set_text(badgeIcon, LV_SYMBOL_CHARGE " USBO App");
    lv_obj_set_style_text_color(badgeIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(badgeIcon, &lv_font_montserrat_12, 0);
    lv_obj_center(badgeIcon);

    // "Steps:" label
    lv_obj_t *stepsLabel = lv_label_create(mainCard);
    lv_label_set_text(stepsLabel, "Steps:");
    lv_obj_set_style_text_color(stepsLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(stepsLabel, LV_OPA_80, 0);
    lv_obj_set_style_text_font(stepsLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(stepsLabel, LV_ALIGN_TOP_LEFT, 18, 60);

    // HUGE step count - NIKE FONT bold white number (main focus like reference)
    char stepBuf[16];
    snprintf(stepBuf, sizeof(stepBuf), "%lu", (unsigned long)userData.steps);
    lv_obj_t *stepCount = lv_label_create(mainCard);
    lv_label_set_text(stepCount, stepBuf);
    lv_obj_set_style_text_color(stepCount, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(stepCount, &NIKE_FONT, 0);  // Use Nike font for bigger, bolder display
    lv_obj_align(stepCount, LV_ALIGN_CENTER, 0, -15);
    
    // Distance below steps - using Nike font at size 20
    char distCardBuf[24];
    snprintf(distCardBuf, sizeof(distCardBuf), "%.2f KM", userData.totalDistance);
    lv_obj_t *distanceOnCard = lv_label_create(mainCard);
    lv_label_set_text(distanceOnCard, distCardBuf);
    lv_obj_set_style_text_color(distanceOnCard, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_opa(distanceOnCard, LV_OPA_90, 0);
    lv_obj_set_style_text_font(distanceOnCard, &lv_font_montserrat_20, 0);  // Bigger distance text
    lv_obj_align(distanceOnCard, LV_ALIGN_CENTER, 0, 25);

    // Goal progress calculation
    int progress = (userData.steps * 100) / userData.dailyGoal;
    if (progress > 100) progress = 100;

    // Progress bar container - milestone style like reference (2000 4000 6000 8000)
    lv_obj_t *progressContainer = lv_obj_create(mainCard);
    lv_obj_set_size(progressContainer, LCD_WIDTH - 60, 50);
    lv_obj_align(progressContainer, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_opa(progressContainer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(progressContainer, 0, 0);
    disableAllScrolling(progressContainer);

    // Milestone labels
    const int milestones[] = {2000, 4000, 6000, 8000};
    const int numMilestones = 4;
    int barWidth = (LCD_WIDTH - 80) / numMilestones;

    for (int i = 0; i < numMilestones; i++) {
        // Milestone number label
        char mileBuf[8];
        snprintf(mileBuf, sizeof(mileBuf), "%d", milestones[i]);
        lv_obj_t *mileLabel = lv_label_create(progressContainer);
        lv_label_set_text(mileLabel, mileBuf);
        lv_obj_set_style_text_color(mileLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(mileLabel, LV_OPA_80, 0);
        lv_obj_set_style_text_font(mileLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(mileLabel, LV_ALIGN_TOP_LEFT, i * barWidth + 10, 0);

        // Progress bar segment
        lv_obj_t *barSegment = lv_obj_create(progressContainer);
        lv_obj_set_size(barSegment, barWidth - 8, 6);
        lv_obj_align(barSegment, LV_ALIGN_TOP_LEFT, i * barWidth + 5, 22);
        lv_obj_set_style_radius(barSegment, 3, 0);
        lv_obj_set_style_border_width(barSegment, 0, 0);
        disableAllScrolling(barSegment);

        // Color based on whether milestone is reached
        if ((int)userData.steps >= milestones[i]) {
            lv_obj_set_style_bg_color(barSegment, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_bg_opa(barSegment, LV_OPA_100, 0);
        } else if (i == 0 || (int)userData.steps >= milestones[i-1]) {
            // Partial progress in current segment
            lv_obj_set_style_bg_color(barSegment, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_bg_opa(barSegment, LV_OPA_40, 0);
        } else {
            lv_obj_set_style_bg_color(barSegment, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_bg_opa(barSegment, LV_OPA_20, 0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DISTANCE CARD - DARK MINIMAL STYLE (Like reference image with daily goal)
// ═══════════════════════════════════════════════════════════════════════════
void createDistanceCard() {
    disableAllScrolling(lv_scr_act());

    // Pure dark AMOLED background
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Main dark card container - rounded corners like reference
    lv_obj_t *mainCard = lv_obj_create(card);
    lv_obj_set_size(mainCard, LCD_WIDTH - 24, LCD_HEIGHT - 40);
    lv_obj_align(mainCard, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(mainCard, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_radius(mainCard, 28, 0);
    lv_obj_set_style_border_width(mainCard, 0, 0);
    disableAllScrolling(mainCard);

    // Top text section - "Today you've walked X steps, that's around X KM."
    lv_obj_t *todayLabel = lv_label_create(mainCard);
    lv_label_set_text(todayLabel, "Today");
    lv_obj_set_style_text_color(todayLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(todayLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(todayLabel, LV_ALIGN_TOP_LEFT, 20, 30);

    lv_obj_t *walkedLabel = lv_label_create(mainCard);
    lv_label_set_text(walkedLabel, "you've walked");
    lv_obj_set_style_text_color(walkedLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(walkedLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(walkedLabel, LV_ALIGN_TOP_LEFT, 75, 33);

    // Steps count - bold white
    char stepsBuf[24];
    snprintf(stepsBuf, sizeof(stepsBuf), "%lu steps,", (unsigned long)userData.steps);
    lv_obj_t *stepsCountLabel = lv_label_create(mainCard);
    lv_label_set_text(stepsCountLabel, stepsBuf);
    lv_obj_set_style_text_color(stepsCountLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(stepsCountLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(stepsCountLabel, LV_ALIGN_TOP_LEFT, 20, 58);

    // "that's around"
    lv_obj_t *aroundLabel = lv_label_create(mainCard);
    lv_label_set_text(aroundLabel, "that's around");
    lv_obj_set_style_text_color(aroundLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(aroundLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(aroundLabel, LV_ALIGN_TOP_LEFT, 20, 88);

    // Distance KM - BIGGER with Nike-inspired styling (size 20)
    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.1f KM", userData.totalDistance);
    lv_obj_t *distLabel = lv_label_create(mainCard);
    lv_label_set_text(distLabel, distBuf);
    lv_obj_set_style_text_color(distLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(distLabel, &lv_font_montserrat_20, 0);  // Bigger distance text (20pt)
    lv_obj_align(distLabel, LV_ALIGN_TOP_LEFT, 120, 83);

    // Divider line
    lv_obj_t *divider = lv_obj_create(mainCard);
    lv_obj_set_size(divider, LCD_WIDTH - 70, 1);
    lv_obj_align(divider, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(divider, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(divider, 0, 0);
    lv_obj_set_style_border_width(divider, 0, 0);
    disableAllScrolling(divider);

    // Bottom section - Daily goal with footprint icon
    // Footprint icon (using GPS as placeholder since LVGL doesn't have footprint)
    lv_obj_t *footIcon = lv_label_create(mainCard);
    lv_label_set_text(footIcon, LV_SYMBOL_SHUFFLE);  // Using shuffle as footsteps placeholder
    lv_obj_set_style_text_color(footIcon, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(footIcon, &lv_font_montserrat_32, 0);
    lv_obj_align(footIcon, LV_ALIGN_BOTTOM_LEFT, 25, -50);

    // "Your daily goal:" text
    lv_obj_t *goalTextLabel = lv_label_create(mainCard);
    lv_label_set_text(goalTextLabel, "Your");
    lv_obj_set_style_text_color(goalTextLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(goalTextLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(goalTextLabel, LV_ALIGN_BOTTOM_LEFT, 70, -75);

    lv_obj_t *dailyGoalLabel = lv_label_create(mainCard);
    lv_label_set_text(dailyGoalLabel, "daily goal:");
    lv_obj_set_style_text_color(dailyGoalLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dailyGoalLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(dailyGoalLabel, LV_ALIGN_BOTTOM_LEFT, 105, -72);

    // Daily goal steps value
    char goalBuf[24];
    // Format with comma: 3,500 steps
    if (userData.dailyGoal >= 1000) {
        snprintf(goalBuf, sizeof(goalBuf), "%d,%03d steps.", 
                 (int)(userData.dailyGoal / 1000), 
                 (int)(userData.dailyGoal % 1000));
    } else {
        snprintf(goalBuf, sizeof(goalBuf), "%lu steps.", (unsigned long)userData.dailyGoal);
    }
    lv_obj_t *goalValueLabel = lv_label_create(mainCard);
    lv_label_set_text(goalValueLabel, goalBuf);
    lv_obj_set_style_text_color(goalValueLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(goalValueLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(goalValueLabel, LV_ALIGN_BOTTOM_LEFT, 70, -45);
}

// ═══════════════════════════════════════════════════════════════════════════
// BLACKJACK HELPER FUNCTIONS - PREMIUM VISUAL CARDS
// ═══════════════════════════════════════════════════════════════════════════
void drawPlayingCard(lv_obj_t *parent, int cardValue, int x, int y, bool faceUp) {
    lv_obj_t *cardObj = lv_obj_create(parent);
    lv_obj_set_size(cardObj, 32, 45);
    lv_obj_set_pos(cardObj, x, y);
    lv_obj_set_style_radius(cardObj, 4, 0);
    lv_obj_set_style_border_width(cardObj, 1, 0);
    disableAllScrolling(cardObj);

    if (faceUp) {
        lv_obj_set_style_bg_color(cardObj, lv_color_hex(0xFFFFF0), 0);
        lv_obj_set_style_border_color(cardObj, lv_color_hex(0x000000), 0);

        // Card value
        char valStr[3];
        int displayVal = (cardValue % 13) + 1;
        if (displayVal == 1) strcpy(valStr, "A");
        else if (displayVal == 11) strcpy(valStr, "J");
        else if (displayVal == 12) strcpy(valStr, "Q");
        else if (displayVal == 13) strcpy(valStr, "K");
        else snprintf(valStr, sizeof(valStr), "%d", displayVal);

        lv_obj_t *valLabel = lv_label_create(cardObj);
        lv_label_set_text(valLabel, valStr);

        // Red for hearts/diamonds, black for clubs/spades
        int suit = cardValue / 13;
        lv_color_t suitColor = (suit < 2) ? lv_color_hex(0xFF0000) : lv_color_hex(0x000000);
        lv_obj_set_style_text_color(valLabel, suitColor, 0);
        lv_obj_set_style_text_font(valLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(valLabel, LV_ALIGN_TOP_LEFT, 2, 2);

        // Suit symbol
        const char* suits[] = {"H", "D", "C", "S"};  // Simplified
        lv_obj_t *suitLabel = lv_label_create(cardObj);
        lv_label_set_text(suitLabel, suits[suit]);
        lv_obj_set_style_text_color(suitLabel, suitColor, 0);
        lv_obj_set_style_text_font(suitLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(suitLabel, LV_ALIGN_BOTTOM_RIGHT, -2, -2);
    } else {
        // Face down - show pattern
        lv_obj_set_style_bg_color(cardObj, lv_color_hex(0x1E3A5F), 0);
        lv_obj_set_style_border_color(cardObj, lv_color_hex(0xFFD700), 0);

        lv_obj_t *pattern = lv_label_create(cardObj);
        lv_label_set_text(pattern, "?");
        lv_obj_set_style_text_color(pattern, lv_color_hex(0xFFD700), 0);
        lv_obj_center(pattern);
    }
}

int calculateHandValue(int *cards, int count) {
    int value = 0;
    int aces = 0;

    for (int i = 0; i < count; i++) {
        int cardVal = (cards[i] % 13) + 1;
        if (cardVal == 1) {
            aces++;
            value += 11;
        } else if (cardVal >= 10) {
            value += 10;
        } else {
            value += cardVal;
        }
    }

    // Convert aces from 11 to 1 if busting
    while (value > 21 && aces > 0) {
        value -= 10;
        aces--;
    }

    return value;
}

void dealCard(int *cards, int *count, bool toPlayer) {
    if (*count >= 10) return;
    cards[*count] = random(52);
    (*count)++;
}

// ═══════════════════════════════════════════════════════════════════════════
// BLACKJACK CARD - GREEN CASINO TABLE STYLE
// ═══════════════════════════════════════════════════════════════════════════
void createBlackjackCard() {
    disableAllScrolling(lv_scr_act());

    // Casino green felt table background
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D4A2B), 0);  // Deep casino green
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Felt texture overlay - darker green border effect
    lv_obj_t *feltBorder = lv_obj_create(card);
    lv_obj_set_size(feltBorder, LCD_WIDTH - 8, LCD_HEIGHT - 8);
    lv_obj_center(feltBorder);
    lv_obj_set_style_bg_color(feltBorder, lv_color_hex(0x1A6B40), 0);  // Lighter casino green felt
    lv_obj_set_style_radius(feltBorder, 20, 0);
    lv_obj_set_style_border_width(feltBorder, 3, 0);
    lv_obj_set_style_border_color(feltBorder, lv_color_hex(0x8B4513), 0);  // Wood brown border
    disableAllScrolling(feltBorder);

    // BLACKJACK title - gold on green
    lv_obj_t *titleLabel = lv_label_create(feltBorder);
    lv_label_set_text(titleLabel, "BLACKJACK");
    lv_obj_set_style_text_color(titleLabel, lv_color_hex(0xFFD700), 0);  // Gold
    lv_obj_set_style_text_font(titleLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(titleLabel, LV_ALIGN_TOP_MID, 0, 8);

    // Dealer section - semi-transparent darker area
    lv_obj_t *dealerSection = lv_obj_create(feltBorder);
    lv_obj_set_size(dealerSection, LCD_WIDTH - 32, 85);
    lv_obj_align(dealerSection, LV_ALIGN_TOP_MID, 0, 35);
    lv_obj_set_style_bg_color(dealerSection, lv_color_hex(0x0D4A2B), 0);
    lv_obj_set_style_bg_opa(dealerSection, LV_OPA_60, 0);
    lv_obj_set_style_radius(dealerSection, 12, 0);
    lv_obj_set_style_border_width(dealerSection, 1, 0);
    lv_obj_set_style_border_color(dealerSection, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_border_opa(dealerSection, LV_OPA_30, 0);
    disableAllScrolling(dealerSection);

    lv_obj_t *dealerLabel = lv_label_create(dealerSection);
    lv_label_set_text(dealerLabel, "DEALER");
    lv_obj_set_style_text_color(dealerLabel, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_text_font(dealerLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(dealerLabel, LV_ALIGN_TOP_LEFT, 10, 5);

    // Draw dealer cards
    for (int i = 0; i < dealerCount; i++) {
        bool showCard = (i == 0) || playerStand || !blackjackGameActive;
        drawPlayingCard(dealerSection, dealerCards[i], 10 + i * 36, 22, showCard);
    }

    // Show dealer value
    if ((playerStand && blackjackGameActive) || !blackjackGameActive) {
        int dealerVal = calculateHandValue(dealerCards, dealerCount);
        if (dealerCount > 0) {
            char dealerBuf[8];
            snprintf(dealerBuf, sizeof(dealerBuf), "%d", dealerVal);
            lv_obj_t *dealerValLabel = lv_label_create(dealerSection);
            lv_label_set_text(dealerValLabel, dealerBuf);
            lv_obj_set_style_text_color(dealerValLabel, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(dealerValLabel, &lv_font_montserrat_24, 0);
            lv_obj_align(dealerValLabel, LV_ALIGN_RIGHT_MID, -12, 5);
        }
    }

    // Player section
    lv_obj_t *playerSection = lv_obj_create(feltBorder);
    lv_obj_set_size(playerSection, LCD_WIDTH - 32, 85);
    lv_obj_align(playerSection, LV_ALIGN_TOP_MID, 0, 128);
    lv_obj_set_style_bg_color(playerSection, lv_color_hex(0x0D4A2B), 0);
    lv_obj_set_style_bg_opa(playerSection, LV_OPA_60, 0);
    lv_obj_set_style_radius(playerSection, 12, 0);
    lv_obj_set_style_border_width(playerSection, 1, 0);
    lv_obj_set_style_border_color(playerSection, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_border_opa(playerSection, LV_OPA_30, 0);
    disableAllScrolling(playerSection);

    lv_obj_t *playerLabel = lv_label_create(playerSection);
    lv_label_set_text(playerLabel, "YOUR HAND");
    lv_obj_set_style_text_color(playerLabel, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_text_font(playerLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(playerLabel, LV_ALIGN_TOP_LEFT, 10, 5);

    // Draw player cards
    for (int i = 0; i < playerCount; i++) {
        drawPlayingCard(playerSection, playerCards[i], 10 + i * 36, 22, true);
    }

    // Player value
    int playerVal = calculateHandValue(playerCards, playerCount);
    if (playerCount > 0) {
        char playerBuf[8];
        snprintf(playerBuf, sizeof(playerBuf), "%d", playerVal);
        lv_obj_t *playerValLabel = lv_label_create(playerSection);
        lv_label_set_text(playerValLabel, playerBuf);
        uint32_t valColor = (playerVal > 21) ? 0xFF453A : (playerVal == 21 ? 0xFFD700 : 0xFFFFFF);
        lv_obj_set_style_text_color(playerValLabel, lv_color_hex(valColor), 0);
        lv_obj_set_style_text_font(playerValLabel, &lv_font_montserrat_24, 0);
        lv_obj_align(playerValLabel, LV_ALIGN_RIGHT_MID, -12, 5);
    }

    // Bottom action area - on the felt
    lv_obj_t *actionArea = lv_obj_create(feltBorder);
    lv_obj_set_size(actionArea, LCD_WIDTH - 32, 75);
    lv_obj_align(actionArea, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_opa(actionArea, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(actionArea, 0, 0);
    disableAllScrolling(actionArea);

    if (!blackjackGameActive) {
        // New game state - gold deal button
        lv_obj_t *dealBtn = lv_obj_create(actionArea);
        lv_obj_set_size(dealBtn, LCD_WIDTH - 70, 42);
        lv_obj_align(dealBtn, LV_ALIGN_TOP_MID, 0, 5);
        lv_obj_set_style_bg_color(dealBtn, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_radius(dealBtn, 21, 0);
        lv_obj_set_style_border_width(dealBtn, 2, 0);
        lv_obj_set_style_border_color(dealBtn, lv_color_hex(0x8B4513), 0);
        lv_obj_set_style_shadow_width(dealBtn, 8, 0);
        lv_obj_set_style_shadow_color(dealBtn, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(dealBtn, LV_OPA_40, 0);
        disableAllScrolling(dealBtn);

        lv_obj_t *dealLabel = lv_label_create(dealBtn);
        lv_label_set_text(dealLabel, "TAP TO DEAL");
        lv_obj_set_style_text_color(dealLabel, lv_color_hex(0x0D4A2B), 0);
        lv_obj_set_style_text_font(dealLabel, &lv_font_montserrat_16, 0);
        lv_obj_center(dealLabel);

        // Stats row with Win/Loss ratio
        char statsBuf[48];
        int losses = userData.gamesPlayed - userData.gamesWon;
        float winRate = userData.gamesPlayed > 0 ? (float)userData.gamesWon / userData.gamesPlayed * 100.0f : 0.0f;
        snprintf(statsBuf, sizeof(statsBuf), "W:%d L:%d (%.0f%%)", userData.gamesWon, losses, winRate);
        lv_obj_t *statsLabel = lv_label_create(actionArea);
        lv_label_set_text(statsLabel, statsBuf);
        lv_obj_set_style_text_color(statsLabel, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_text_opa(statsLabel, LV_OPA_70, 0);
        lv_obj_set_style_text_font(statsLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(statsLabel, LV_ALIGN_BOTTOM_MID, 0, -5);

    } else if (playerVal > 21) {
        // Bust state - red text
        lv_obj_t *bustLabel = lv_label_create(actionArea);
        lv_label_set_text(bustLabel, "BUST!");
        lv_obj_set_style_text_color(bustLabel, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_text_font(bustLabel, &lv_font_montserrat_32, 0);
        lv_obj_align(bustLabel, LV_ALIGN_TOP_MID, 0, 8);

        lv_obj_t *tapLabel = lv_label_create(actionArea);
        lv_label_set_text(tapLabel, "Tap for new game");
        lv_obj_set_style_text_color(tapLabel, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_text_opa(tapLabel, LV_OPA_60, 0);
        lv_obj_align(tapLabel, LV_ALIGN_BOTTOM_MID, 0, -8);

    } else if (!playerStand) {
        // Action buttons - HIT (green) and STAND (red)
        lv_obj_t *hitBtn = lv_obj_create(actionArea);
        lv_obj_set_size(hitBtn, 85, 48);
        lv_obj_align(hitBtn, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_bg_color(hitBtn, lv_color_hex(0x228B22), 0);  // Forest green
        lv_obj_set_style_radius(hitBtn, 24, 0);
        lv_obj_set_style_border_width(hitBtn, 2, 0);
        lv_obj_set_style_border_color(hitBtn, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_shadow_width(hitBtn, 6, 0);
        lv_obj_set_style_shadow_color(hitBtn, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(hitBtn, LV_OPA_50, 0);
        disableAllScrolling(hitBtn);

        lv_obj_t *hitLabel = lv_label_create(hitBtn);
        lv_label_set_text(hitLabel, "HIT");
        lv_obj_set_style_text_color(hitLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(hitLabel, &lv_font_montserrat_16, 0);
        lv_obj_center(hitLabel);

        lv_obj_t *standBtn = lv_obj_create(actionArea);
        lv_obj_set_size(standBtn, 85, 48);
        lv_obj_align(standBtn, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(standBtn, lv_color_hex(0x8B0000), 0);  // Dark red
        lv_obj_set_style_radius(standBtn, 24, 0);
        lv_obj_set_style_border_width(standBtn, 2, 0);
        lv_obj_set_style_border_color(standBtn, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_shadow_width(standBtn, 6, 0);
        lv_obj_set_style_shadow_color(standBtn, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(standBtn, LV_OPA_50, 0);
        disableAllScrolling(standBtn);

        lv_obj_t *standLabel = lv_label_create(standBtn);
        lv_label_set_text(standLabel, "STAND");
        lv_obj_set_style_text_color(standLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(standLabel, &lv_font_montserrat_14, 0);
        lv_obj_center(standLabel);

    } else {
        // Result state
        int dealerVal = calculateHandValue(dealerCards, dealerCount);
        const char* result;
        uint32_t resultColor;

        if (dealerVal > 21 || playerVal > dealerVal) {
            result = "YOU WIN!";
            resultColor = 0xFFD700;  // Gold for win
        } else if (playerVal < dealerVal) {
            result = "DEALER WINS";
            resultColor = 0xFF453A;  // Red for loss
        } else {
            result = "PUSH";
            resultColor = 0xFFFFFF;  // White for tie
        }

        lv_obj_t *resultLabel = lv_label_create(actionArea);
        lv_label_set_text(resultLabel, result);
        lv_obj_set_style_text_color(resultLabel, lv_color_hex(resultColor), 0);
        lv_obj_set_style_text_font(resultLabel, &lv_font_montserrat_24, 0);
        lv_obj_align(resultLabel, LV_ALIGN_TOP_MID, 0, 12);

        lv_obj_t *tapLabel = lv_label_create(actionArea);
        lv_label_set_text(tapLabel, "Tap for new game");
        lv_obj_set_style_text_color(tapLabel, lv_color_hex(0xFFD700), 0);
        lv_obj_set_style_text_opa(tapLabel, LV_OPA_60, 0);
        lv_obj_align(tapLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
    }
    
    // Hint to access game selector
    lv_obj_t *moreGamesHint = lv_label_create(card);
    lv_label_set_text(moreGamesHint, LV_SYMBOL_DOWN " MORE GAMES");
    lv_obj_set_style_text_color(moreGamesHint, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_text_opa(moreGamesHint, LV_OPA_50, 0);
    lv_obj_set_style_text_font(moreGamesHint, &lv_font_montserrat_12, 0);
    lv_obj_align(moreGamesHint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// Blackjack game logic handlers
void startNewBlackjackGame() {
    playerCount = 0;
    dealerCount = 0;
    playerStand = false;
    blackjackGameActive = true;

    // Deal initial cards
    dealCard(playerCards, &playerCount, true);
    dealCard(dealerCards, &dealerCount, false);
    dealCard(playerCards, &playerCount, true);
    dealCard(dealerCards, &dealerCount, false);

    userData.gamesPlayed++;
}

void playerHit() {
    if (!blackjackGameActive || playerStand) return;

    dealCard(playerCards, &playerCount, true);

    int playerVal = calculateHandValue(playerCards, playerCount);
    if (playerVal > 21) {
        // Bust - game over
        blackjackGameActive = false;
    }
}

void playerStandAction() {
    if (!blackjackGameActive || playerStand) return;

    playerStand = true;

    // Dealer draws until 17+
    int dealerVal = calculateHandValue(dealerCards, dealerCount);
    while (dealerVal < 17) {
        dealCard(dealerCards, &dealerCount, false);
        dealerVal = calculateHandValue(dealerCards, dealerCount);
    }

    // Determine winner
    int playerVal = calculateHandValue(playerCards, playerCount);
    if (dealerVal > 21 || playerVal > dealerVal) {
        userData.gamesWon++;
        userData.blackjackStreak++;
    } else if (playerVal < dealerVal) {
        userData.blackjackStreak = 0;
    }

    blackjackGameActive = false;
}

// ═══════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════
// WEATHER CARD - BERLIN MINIMALIST STYLE (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void createWeatherCard() {
    disableAllScrolling(lv_scr_act());

    // Deep matte black background
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    disableAllScrolling(card);

    // Subtle top gradient for depth
    lv_obj_t *topGradient = lv_obj_create(card);
    lv_obj_set_size(topGradient, LCD_WIDTH, 150);
    lv_obj_align(topGradient, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(topGradient, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(topGradient, LV_OPA_30, 0);
    lv_obj_set_style_radius(topGradient, 0, 0);
    lv_obj_set_style_border_width(topGradient, 0, 0);
    disableAllScrolling(topGradient);

    // Large temperature with shadow
    lv_obj_t *tempLabel = lv_label_create(card);
    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%.0f°", weatherTemp);
    lv_label_set_text(tempLabel, tempStr);
    lv_obj_set_style_text_color(tempLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(tempLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(tempLabel, LV_ALIGN_TOP_LEFT, 32, 80);

    // City name in CAPS with letter spacing
    lv_obj_t *cityLabel = lv_label_create(card);
    char cityUpper[64];
    strncpy(cityUpper, weatherCity, sizeof(cityUpper));
    for (int i = 0; cityUpper[i]; i++) cityUpper[i] = toupper(cityUpper[i]);
    lv_label_set_text(cityLabel, cityUpper);
    lv_obj_set_style_text_color(cityLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(cityLabel, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_letter_space(cityLabel, 3, 0);
    lv_obj_align(cityLabel, LV_ALIGN_TOP_LEFT, 32, 145);

    // Accent line under city
    lv_obj_t *accentLine = lv_obj_create(card);
    lv_obj_set_size(accentLine, 60, 3);
    lv_obj_align(accentLine, LV_ALIGN_TOP_LEFT, 32, 175);
    lv_obj_set_style_bg_color(accentLine, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_radius(accentLine, 2, 0);
    lv_obj_set_style_border_width(accentLine, 0, 0);
    disableAllScrolling(accentLine);

    // High/Low container with glass effect
    lv_obj_t *rangeContainer = lv_obj_create(card);
    lv_obj_set_size(rangeContainer, 180, 50);
    lv_obj_align(rangeContainer, LV_ALIGN_TOP_LEFT, 32, 195);
    lv_obj_set_style_bg_color(rangeContainer, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(rangeContainer, LV_OPA_40, 0);
    lv_obj_set_style_radius(rangeContainer, 12, 0);
    lv_obj_set_style_border_width(rangeContainer, 0, 0);
    disableAllScrolling(rangeContainer);

    lv_obj_t *rangeLabel = lv_label_create(rangeContainer);
    char rangeStr[32];
    snprintf(rangeStr, sizeof(rangeStr), "H: %.0f°  L: %.0f°", weatherHigh, weatherLow);
    lv_label_set_text(rangeLabel, rangeStr);
    lv_obj_set_style_text_color(rangeLabel, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_text_font(rangeLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(rangeLabel);

    // Weather icon with glow effect
    lv_obj_t *iconContainer = lv_obj_create(card);
    lv_obj_set_size(iconContainer, 80, 80);
    lv_obj_align(iconContainer, LV_ALIGN_TOP_LEFT, 32, 270);
    lv_obj_set_style_bg_color(iconContainer, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_bg_opa(iconContainer, LV_OPA_20, 0);
    lv_obj_set_style_radius(iconContainer, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(iconContainer, 2, 0);
    lv_obj_set_style_border_color(iconContainer, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_border_opa(iconContainer, LV_OPA_50, 0);
    disableAllScrolling(iconContainer);

    lv_obj_t *icon = lv_label_create(iconContainer);
    // Get proper weather icon based on condition
    uint32_t iconColor = getWeatherIconColor(weatherDesc.c_str());
    lv_obj_set_style_bg_color(iconContainer, lv_color_hex(iconColor), 0);
    lv_obj_set_style_bg_opa(iconContainer, LV_OPA_20, 0);
    lv_obj_set_style_border_color(iconContainer, lv_color_hex(iconColor), 0);
    
    // Weather icon symbols mapped to conditions
    const char* iconSymbol;
    if (weatherDesc.indexOf("Clear") >= 0 || weatherDesc.indexOf("Sunny") >= 0) {
        iconSymbol = LV_SYMBOL_IMAGE;  // Sun icon
    } else if (weatherDesc.indexOf("Cloud") >= 0 || weatherDesc.indexOf("Overcast") >= 0) {
        iconSymbol = LV_SYMBOL_WIFI;  // Cloud icon
    } else if (weatherDesc.indexOf("Rain") >= 0 || weatherDesc.indexOf("Drizzle") >= 0) {
        iconSymbol = LV_SYMBOL_REFRESH;  // Rain icon
    } else if (weatherDesc.indexOf("Snow") >= 0 || weatherDesc.indexOf("Sleet") >= 0) {
        iconSymbol = LV_SYMBOL_OK;  // Snow icon
    } else if (weatherDesc.indexOf("Thunder") >= 0 || weatherDesc.indexOf("Storm") >= 0) {
        iconSymbol = LV_SYMBOL_CHARGE;  // Storm icon
    } else if (weatherDesc.indexOf("Mist") >= 0 || weatherDesc.indexOf("Fog") >= 0 || weatherDesc.indexOf("Haze") >= 0) {
        iconSymbol = LV_SYMBOL_LIST;  // Fog icon
    } else {
        iconSymbol = LV_SYMBOL_EYE_OPEN;  // Default weather
    }
    lv_label_set_text(icon, iconSymbol);
    lv_obj_set_style_text_color(icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_32, 0);
    lv_obj_center(icon);

    // Weather description
    lv_obj_t *descLabel = lv_label_create(card);
    lv_label_set_text(descLabel, weatherDesc.c_str());
    lv_obj_set_style_text_color(descLabel, lv_color_hex(0xB0B0B0), 0);
    lv_obj_set_style_text_font(descLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(descLabel, LV_ALIGN_TOP_LEFT, 130, 295);

    // Bottom hint bar
    lv_obj_t *bottomBar = lv_obj_create(card);
    lv_obj_set_size(bottomBar, LCD_WIDTH, 60);
    lv_obj_align(bottomBar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bottomBar, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(bottomBar, LV_OPA_30, 0);
    lv_obj_set_style_radius(bottomBar, 0, 0);
    lv_obj_set_style_border_width(bottomBar, 0, 0);
    disableAllScrolling(bottomBar);

    lv_obj_t *hint = lv_label_create(bottomBar);
    lv_label_set_text(hint, LV_SYMBOL_UP " Swipe for more");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_center(hint);
}

// ═══════════════════════════════════════════════════════════════════════════
// FORECAST CARD
// ═══════════════════════════════════════════════════════════════════════════
void createForecastCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "5-DAY FORECAST");
    lv_obj_set_style_text_color(title, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    // City name
    lv_obj_t *cityLabel = lv_label_create(card);
    char cityUpper[64];
    strncpy(cityUpper, weatherCity, sizeof(cityUpper));
    for (int i = 0; cityUpper[i]; i++) cityUpper[i] = toupper(cityUpper[i]);
    lv_label_set_text(cityLabel, cityUpper);
    lv_obj_set_style_text_color(cityLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(cityLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(cityLabel, LV_ALIGN_TOP_MID, 0, 32);

    // 5-day forecast rows
    int startY = 58;
    int rowHeight = 58;

    for (int i = 0; i < 5; i++) {
        // Day row container
        lv_obj_t *dayRow = lv_obj_create(card);
        lv_obj_set_size(dayRow, LCD_WIDTH - 30, 50);
        lv_obj_align(dayRow, LV_ALIGN_TOP_MID, 0, startY + i * rowHeight);
        lv_obj_set_style_bg_color(dayRow, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(dayRow, 12, 0);
        lv_obj_set_style_border_width(dayRow, 0, 0);
        disableAllScrolling(dayRow);

        // Day name
        lv_obj_t *dayLabel = lv_label_create(dayRow);
        lv_label_set_text(dayLabel, forecast5Day[i].dayName);
        lv_obj_set_style_text_color(dayLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(dayLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(dayLabel, LV_ALIGN_LEFT_MID, 15, 0);

        // Weather icon
        lv_obj_t *iconLabel = lv_label_create(dayRow);
        const char* iconSymbol;
        uint32_t iconColor;
        
        // Map icon string to proper LVGL symbols and colors
        if (strcmp(forecast5Day[i].icon, "sun") == 0 || strcmp(forecast5Day[i].icon, "clear") == 0) {
            iconSymbol = LV_SYMBOL_IMAGE;  // Sun
            iconColor = 0xFFD60A;  // Yellow/Gold
        } else if (strcmp(forecast5Day[i].icon, "cloud") == 0 || strcmp(forecast5Day[i].icon, "overcast") == 0) {
            iconSymbol = LV_SYMBOL_WIFI;  // Cloud
            iconColor = 0xAEAEB2;  // Gray
        } else if (strcmp(forecast5Day[i].icon, "rain") == 0 || strcmp(forecast5Day[i].icon, "drizzle") == 0) {
            iconSymbol = LV_SYMBOL_REFRESH;  // Rain drops
            iconColor = 0x0A84FF;  // Blue
        } else if (strcmp(forecast5Day[i].icon, "snow") == 0 || strcmp(forecast5Day[i].icon, "sleet") == 0) {
            iconSymbol = LV_SYMBOL_OK;  // Snowflake
            iconColor = 0xE0F7FA;  // Light cyan
        } else if (strcmp(forecast5Day[i].icon, "storm") == 0 || strcmp(forecast5Day[i].icon, "thunder") == 0) {
            iconSymbol = LV_SYMBOL_CHARGE;  // Lightning bolt
            iconColor = 0xFFD60A;  // Yellow
        } else if (strcmp(forecast5Day[i].icon, "mist") == 0 || strcmp(forecast5Day[i].icon, "fog") == 0) {
            iconSymbol = LV_SYMBOL_LIST;  // Horizontal lines (fog)
            iconColor = 0x9E9E9E;  // Light gray
        } else if (strcmp(forecast5Day[i].icon, "wind") == 0) {
            iconSymbol = LV_SYMBOL_NEXT;  // Wind arrow
            iconColor = 0xB3E5FC;  // Light blue
        } else {
            iconSymbol = LV_SYMBOL_EYE_OPEN;  // Default - partial cloud
            iconColor = 0x8E8E93;  // Gray
        }
        
        lv_label_set_text(iconLabel, iconSymbol);
        lv_obj_set_style_text_color(iconLabel, lv_color_hex(iconColor), 0);
        lv_obj_set_style_text_font(iconLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(iconLabel, LV_ALIGN_CENTER, -15, 0);

        // High temp
        char highBuf[8];
        snprintf(highBuf, sizeof(highBuf), "%.0f°", forecast5Day[i].tempHigh);
        lv_obj_t *highLabel = lv_label_create(dayRow);
        lv_label_set_text(highLabel, highBuf);
        lv_obj_set_style_text_color(highLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(highLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(highLabel, LV_ALIGN_RIGHT_MID, -55, 0);

        // Low temp
        char lowBuf[8];
        snprintf(lowBuf, sizeof(lowBuf), "%.0f°", forecast5Day[i].tempLow);
        lv_obj_t *lowLabel = lv_label_create(dayRow);
        lv_label_set_text(lowLabel, lowBuf);
        lv_obj_set_style_text_color(lowLabel, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(lowLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(lowLabel, LV_ALIGN_RIGHT_MID, -12, 0);
    }

    // Refresh hint
    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, forecastLoaded ? (weatherDataLoaded ? "Cached data" : "Updated from API") : "Sample data");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// STOPWATCH CARD - PREMIUM DESIGN WITH LAP TIMES
// ═══════════════════════════════════════════════════════════════════════════
void createStopwatchCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "STOPWATCH");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Calculate elapsed time
    unsigned long elapsed = stopwatchElapsedMs;
    if (stopwatchRunning) {
        elapsed += (millis() - stopwatchStartMs);
    }

    int mins = elapsed / 60000;
    int secs = (elapsed / 1000) % 60;
    int ms = (elapsed % 1000) / 10;

    // Main time display container
    lv_obj_t *timeCard = lv_obj_create(card);
    lv_obj_set_size(timeCard, LCD_WIDTH - 40, 100);
    lv_obj_align(timeCard, LV_ALIGN_TOP_MID, 0, 45);
    lv_obj_set_style_bg_color(timeCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(timeCard, 20, 0);
    lv_obj_set_style_border_width(timeCard, 0, 0);
    disableAllScrolling(timeCard);

    // Large time display
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", mins, secs);
    lv_obj_t *timeLabel = lv_label_create(timeCard);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, -15, 0);

    // Milliseconds
    char msBuf[8];
    snprintf(msBuf, sizeof(msBuf), ".%02d", ms);
    lv_obj_t *msLabel = lv_label_create(timeCard);
    lv_label_set_text(msLabel, msBuf);
    lv_obj_set_style_text_color(msLabel, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(msLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(msLabel, LV_ALIGN_CENTER, 70, 10);

    // Status indicator
    lv_obj_t *statusDot = lv_obj_create(card);
    lv_obj_set_size(statusDot, 12, 12);
    lv_obj_align(statusDot, LV_ALIGN_TOP_MID, 0, 155);
    lv_obj_set_style_bg_color(statusDot, stopwatchRunning ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_radius(statusDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(statusDot, 0, 0);
    disableAllScrolling(statusDot);

    // Status text
    lv_obj_t *statusLabel = lv_label_create(card);
    lv_label_set_text(statusLabel, stopwatchRunning ? "RUNNING" : (elapsed > 0 ? "PAUSED" : "READY"));
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(statusLabel, LV_ALIGN_TOP_MID, 15, 153);

    // Lap times panel (if any laps recorded)
    if (lapCount > 0) {
        lv_obj_t *lapCard = lv_obj_create(card);
        lv_obj_set_size(lapCard, LCD_WIDTH - 40, 100);
        lv_obj_align(lapCard, LV_ALIGN_CENTER, 0, 50);
        lv_obj_set_style_bg_color(lapCard, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(lapCard, 15, 0);
        lv_obj_set_style_border_width(lapCard, 0, 0);
        disableAllScrolling(lapCard);

        lv_obj_t *lapTitle = lv_label_create(lapCard);
        lv_label_set_text(lapTitle, "LAP TIMES");
        lv_obj_set_style_text_color(lapTitle, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(lapTitle, &lv_font_montserrat_12, 0);
        lv_obj_align(lapTitle, LV_ALIGN_TOP_MID, 0, 8);

        // Show last 3 laps
        int startLap = max(0, lapCount - 3);
        for (int i = startLap; i < lapCount && i < startLap + 3; i++) {
            int lapMins = lapTimes[i] / 60000;
            int lapSecs = (lapTimes[i] / 1000) % 60;
            int lapMs = (lapTimes[i] % 1000) / 10;

            char lapBuf[32];
            snprintf(lapBuf, sizeof(lapBuf), "Lap %d: %02d:%02d.%02d", i + 1, lapMins, lapSecs, lapMs);
            lv_obj_t *lapLabel = lv_label_create(lapCard);
            lv_label_set_text(lapLabel, lapBuf);
            lv_obj_set_style_text_color(lapLabel, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(lapLabel, &lv_font_montserrat_12, 0);
            lv_obj_align(lapLabel, LV_ALIGN_TOP_LEFT, 15, 25 + (i - startLap) * 22);
        }
    }

    // Control buttons row
    lv_obj_t *btnRow = lv_obj_create(card);
    lv_obj_set_size(btnRow, LCD_WIDTH - 40, 55);
    lv_obj_align(btnRow, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(btnRow, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(btnRow, 27, 0);
    lv_obj_set_style_border_width(btnRow, 0, 0);
    disableAllScrolling(btnRow);

    // Start/Stop indicator
    lv_obj_t *actionLabel = lv_label_create(btnRow);
    lv_label_set_text(actionLabel, stopwatchRunning ? "TAP TO STOP" : "TAP TO START");
    lv_obj_set_style_text_color(actionLabel, stopwatchRunning ? lv_color_hex(0xFF453A) : lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(actionLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(actionLabel);

    // Reset hint
    lv_obj_t *resetHint = lv_label_create(card);
    lv_label_set_text(resetHint, "DOUBLE TAP TO RESET");
    lv_obj_set_style_text_color(resetHint, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(resetHint, &lv_font_montserrat_12, 0);
    lv_obj_align(resetHint, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ═══════════════════════════════════════════════════════════════════════════
// TORCH CARD - PREMIUM DESIGN
// ═══════════════════════════════════════════════════════════════════════════
void createTorchCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);

    // Background changes based on torch state
    if (torchOn) {
        // Bright background matching torch color
        lv_obj_set_style_bg_color(card, lv_color_hex(torchColors[torchColorIndex]), 0);
    } else {
        // Dark background when off
        lv_obj_set_style_bg_color(card, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_bg_grad_color(card, lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    }

    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "TORCH");
    lv_obj_set_style_text_color(title, torchOn ? lv_color_hex(0x1C1C1E) : lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Large power button icon
    lv_obj_t *powerBtn = lv_obj_create(card);
    lv_obj_set_size(powerBtn, 150, 150);
    lv_obj_center(powerBtn);
    lv_obj_set_style_radius(powerBtn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(powerBtn, 4, 0);
    disableAllScrolling(powerBtn);

    if (torchOn) {
        lv_obj_set_style_bg_color(powerBtn, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_bg_opa(powerBtn, LV_OPA_30, 0);
        lv_obj_set_style_border_color(powerBtn, lv_color_hex(0x1C1C1E), 0);
    } else {
        lv_obj_set_style_bg_color(powerBtn, lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_border_color(powerBtn, lv_color_hex(0x0A84FF), 0);
    }

    // Power icon
    lv_obj_t *powerIcon = lv_label_create(powerBtn);
    lv_label_set_text(powerIcon, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(powerIcon, torchOn ? lv_color_hex(0x1C1C1E) : lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(powerIcon, &lv_font_montserrat_48, 0);
    lv_obj_center(powerIcon);

    // Status text
    lv_obj_t *statusLabel = lv_label_create(card);
    lv_label_set_text(statusLabel, torchOn ? "ON" : "OFF");
    lv_obj_set_style_text_color(statusLabel, torchOn ? lv_color_hex(0x1C1C1E) : lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(statusLabel, LV_ALIGN_BOTTOM_MID, 0, -100);

    // Color indicator (when on)
    if (torchOn) {
        lv_obj_t *colorLabel = lv_label_create(card);
        lv_label_set_text(colorLabel, torchColorNames[torchColorIndex]);
        lv_obj_set_style_text_color(colorLabel, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_text_font(colorLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(colorLabel, LV_ALIGN_BOTTOM_MID, 0, -70);
    }

    // Hint
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_label_set_text(hintLabel, "TAP TO TOGGLE");
    lv_obj_set_style_text_color(hintLabel, torchOn ? lv_color_hex(0x1C1C1E) : lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -30);

    lv_obj_t *hint2Label = lv_label_create(card);
    lv_label_set_text(hint2Label, "SWIPE DOWN FOR SETTINGS");
    lv_obj_set_style_text_color(hint2Label, torchOn ? lv_color_hex(0x1C1C1E) : lv_color_hex(0x5E5E5E), 0);
    lv_obj_set_style_text_font(hint2Label, &lv_font_montserrat_12, 0);
    lv_obj_align(hint2Label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ═══════════════════════════════════════════════════════════════════════════
// TORCH SETTINGS CARD - PREMIUM DESIGN
// ═══════════════════════════════════════════════════════════════════════════
void createTorchSettingsCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, theme->color1, 0);
    lv_obj_set_style_bg_grad_color(card, theme->color2, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "TORCH SETTINGS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Color selection
    lv_obj_t *colorTitle = lv_label_create(card);
    lv_label_set_text(colorTitle, "COLOR");
    lv_obj_set_style_text_color(colorTitle, theme->text, 0);
    lv_obj_align(colorTitle, LV_ALIGN_TOP_MID, 0, 50);

    // Color swatches
    int swatchSize = 50;
    int startX = (LCD_WIDTH - (NUM_TORCH_COLORS * swatchSize + (NUM_TORCH_COLORS - 1) * 10)) / 2;

    for (int i = 0; i < NUM_TORCH_COLORS; i++) {
        lv_obj_t *swatch = lv_obj_create(card);
        lv_obj_set_size(swatch, swatchSize, swatchSize);
        lv_obj_set_pos(swatch, startX + i * (swatchSize + 10), 80);
        lv_obj_set_style_bg_color(swatch, lv_color_hex(torchColors[i]), 0);
        lv_obj_set_style_radius(swatch, LV_RADIUS_CIRCLE, 0);

        // Highlight selected
        if (i == torchColorIndex) {
            lv_obj_set_style_border_width(swatch, 3, 0);
            lv_obj_set_style_border_color(swatch, theme->accent, 0);
        } else {
            lv_obj_set_style_border_width(swatch, 1, 0);
            lv_obj_set_style_border_color(swatch, lv_color_hex(0x3A3A3C), 0);
        }

        disableAllScrolling(swatch);
    }

    // Selected color name
    lv_obj_t *colorName = lv_label_create(card);
    lv_label_set_text(colorName, torchColorNames[torchColorIndex]);
    lv_obj_set_style_text_color(colorName, theme->accent, 0);
    lv_obj_set_style_text_font(colorName, &lv_font_montserrat_16, 0);
    lv_obj_align(colorName, LV_ALIGN_TOP_MID, 0, 145);

    // Brightness section
    lv_obj_t *brightTitle = lv_label_create(card);
    lv_label_set_text(brightTitle, "BRIGHTNESS");
    lv_obj_set_style_text_color(brightTitle, theme->text, 0);
    lv_obj_align(brightTitle, LV_ALIGN_TOP_MID, 0, 190);

    // Brightness bar
    lv_obj_t *brightBar = lv_bar_create(card);
    lv_obj_set_size(brightBar, LCD_WIDTH - 60, 20);
    lv_obj_align(brightBar, LV_ALIGN_TOP_MID, 0, 220);
    lv_bar_set_range(brightBar, 50, 255);
    lv_bar_set_value(brightBar, torchBrightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(brightBar, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brightBar, theme->accent, LV_PART_INDICATOR);
    lv_obj_set_style_radius(brightBar, 10, LV_PART_MAIN);
    lv_obj_set_style_radius(brightBar, 10, LV_PART_INDICATOR);

    // Brightness value
    char brightBuf[16];
    int brightPercent = ((torchBrightness - 50) * 100) / 205;
    snprintf(brightBuf, sizeof(brightBuf), "%d%%", brightPercent);
    lv_obj_t *brightVal = lv_label_create(card);
    lv_label_set_text(brightVal, brightBuf);
    lv_obj_set_style_text_color(brightVal, theme->secondary, 0);
    lv_obj_align(brightVal, LV_ALIGN_TOP_MID, 0, 250);

    // Tap hint
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_label_set_text(hintLabel, "TAP COLORS TO SELECT");
    lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ═══════════════════════════════════════════════════════════════════════════
// TOOLS SELECTOR CARD - Icons to select tools
// ═══════════════════════════════════════════════════════════════════════════
void createToolsSelectorCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_SETTINGS " TOOLS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9500), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Grid layout: 2x2 icons
    int iconSize = 90;
    int spacing = 20;
    int startX = (LCD_WIDTH - (iconSize * 2 + spacing)) / 2;
    int startY = 80;

    // Tool data
    const char* toolNames[] = {"CALC", "TALLY", "VOICE", "FILES"};
    const char* toolSymbols[] = {LV_SYMBOL_PLUS, LV_SYMBOL_UP, LV_SYMBOL_AUDIO, LV_SYMBOL_FILE};
    uint32_t toolColors[] = {0xFF9500, 0x30D158, 0xFF2D55, 0x5AC8FA};

    for (int i = 0; i < 4; i++) {
        int row = i / 2;
        int col = i % 2;
        int posX = startX + col * (iconSize + spacing);
        int posY = startY + row * (iconSize + spacing);

        // Icon button
        lv_obj_t *btn = lv_obj_create(card);
        lv_obj_set_size(btn, iconSize, iconSize);
        lv_obj_set_pos(btn, posX, posY);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(btn, 20, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(toolColors[i]), 0);
        lv_obj_set_style_shadow_width(btn, 12, 0);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(toolColors[i]), 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        disableAllScrolling(btn);

        // Icon symbol
        lv_obj_t *icon = lv_label_create(btn);
        lv_label_set_text(icon, toolSymbols[i]);
        lv_obj_set_style_text_color(icon, lv_color_hex(toolColors[i]), 0);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
        lv_obj_align(icon, LV_ALIGN_CENTER, 0, -10);

        // Label
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, toolNames[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -8);
    }

    // Bottom hint
    lv_obj_t *hintTap = lv_label_create(card);
    lv_label_set_text(hintTap, "TAP TO OPEN");
    lv_obj_set_style_text_color(hintTap, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(hintTap, &lv_font_montserrat_12, 0);
    lv_obj_align(hintTap, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// ═══════════════════════════════════════════════════════════════════════════
// CALCULATOR CARD - Enhanced with Memory and Scientific functions
// ═══════════════════════════════════════════════════════════════════════════
void createCalculatorCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Display area
    lv_obj_t *displayBg = lv_obj_create(card);
    lv_obj_set_size(displayBg, LCD_WIDTH - 16, 75);
    lv_obj_align(displayBg, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_bg_color(displayBg, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(displayBg, 16, 0);
    lv_obj_set_style_border_width(displayBg, 0, 0);
    disableAllScrolling(displayBg);

    // Memory indicator
    if (calcMemory != 0) {
        lv_obj_t *memLabel = lv_label_create(displayBg);
        lv_label_set_text(memLabel, "M");
        lv_obj_set_style_text_color(memLabel, lv_color_hex(0x30D158), 0);
        lv_obj_set_style_text_font(memLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(memLabel, LV_ALIGN_TOP_LEFT, 10, 5);
    }

    // Operation indicator
    if (calcOperator != ' ') {
        char opBuf[4];
        snprintf(opBuf, sizeof(opBuf), "%c", calcOperator);
        lv_obj_t *opLabel = lv_label_create(displayBg);
        lv_label_set_text(opLabel, opBuf);
        lv_obj_set_style_text_color(opLabel, lv_color_hex(0xFF9F0A), 0);
        lv_obj_set_style_text_font(opLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(opLabel, LV_ALIGN_TOP_RIGHT, -10, 5);
    }

    // Display number
    lv_obj_t *displayLabel = lv_label_create(displayBg);
    lv_label_set_text(displayLabel, calcDisplay);
    lv_obj_set_style_text_color(displayLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(displayLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(displayLabel, LV_ALIGN_BOTTOM_RIGHT, -10, -8);

    // Button grid - 6 rows x 4 cols (added memory row)
    int btnW = 50, btnH = 44;
    int btnSpacing = 4;
    int startY = 88;
    int startX = (LCD_WIDTH - (4 * btnW + 3 * btnSpacing)) / 2;

    // Row definitions: text, bgColor, textColor
    const char* btnTexts[6][4] = {
        {"MC", "MR", "M+", "M-"},
        {"AC", "±", "%", "÷"},
        {"7", "8", "9", "×"},
        {"4", "5", "6", "-"},
        {"1", "2", "3", "+"},
        {"0", ".", "⌫", "="}
    };
    
    uint32_t btnColors[6][4] = {
        {0x2C2C2E, 0x2C2C2E, 0x2C2C2E, 0x2C2C2E},  // Memory row - dark
        {0xA5A5A5, 0xA5A5A5, 0xA5A5A5, 0xFF9F0A},  // Functions row
        {0x333333, 0x333333, 0x333333, 0xFF9F0A},  // Numbers
        {0x333333, 0x333333, 0x333333, 0xFF9F0A},
        {0x333333, 0x333333, 0x333333, 0xFF9F0A},
        {0x333333, 0x333333, 0x333333, 0x30D158}   // Last row with =
    };
    
    uint32_t txtColors[6][4] = {
        {0x30D158, 0x30D158, 0x30D158, 0x30D158},  // Memory - green
        {0x000000, 0x000000, 0x000000, 0xFFFFFF},  // Functions
        {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF},  // Numbers
        {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF},
        {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF},
        {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF}
    };

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 4; col++) {
            lv_obj_t *btn = lv_obj_create(card);
            lv_obj_set_size(btn, btnW, btnH);
            lv_obj_set_pos(btn, startX + col * (btnW + btnSpacing), startY + row * (btnH + btnSpacing));
            lv_obj_set_style_bg_color(btn, lv_color_hex(btnColors[row][col]), 0);
            lv_obj_set_style_radius(btn, 12, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            disableAllScrolling(btn);

            lv_obj_t *btnLabel = lv_label_create(btn);
            lv_label_set_text(btnLabel, btnTexts[row][col]);
            lv_obj_set_style_text_color(btnLabel, lv_color_hex(txtColors[row][col]), 0);
            lv_obj_set_style_text_font(btnLabel, row == 0 ? &lv_font_montserrat_14 : &lv_font_montserrat_16, 0);
            lv_obj_center(btnLabel);
        }
    }

    // Back hint
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_label_set_text(hintLabel, LV_SYMBOL_UP " TOOLS");
    lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -3);
}

// ═══════════════════════════════════════════════════════════════════════════
// TALLY COUNTER CARD - Enhanced with 4 counters
// ═══════════════════════════════════════════════════════════════════════════
void createTallyCounterCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Counter selector tabs
    uint32_t tabColors[] = {0x30D158, 0x5E5CE6, 0xFF9F0A, 0xFF2D55};
    int tabW = 90, tabH = 35;
    int tabStartX = (LCD_WIDTH - (tabW * 4 + 12)) / 2;
    
    for (int i = 0; i < 4; i++) {
        lv_obj_t *tab = lv_obj_create(card);
        lv_obj_set_size(tab, tabW, tabH);
        lv_obj_set_pos(tab, tabStartX + i * (tabW + 4), 10);
        lv_obj_set_style_bg_color(tab, lv_color_hex(i == currentTallyCounter ? tabColors[i] : 0x2C2C2E), 0);
        lv_obj_set_style_radius(tab, 8, 0);
        lv_obj_set_style_border_width(tab, i == currentTallyCounter ? 0 : 1, 0);
        lv_obj_set_style_border_color(tab, lv_color_hex(tabColors[i]), 0);
        disableAllScrolling(tab);
        
        char tabBuf[8];
        snprintf(tabBuf, sizeof(tabBuf), "#%d", i + 1);
        lv_obj_t *tabLabel = lv_label_create(tab);
        lv_label_set_text(tabLabel, tabBuf);
        lv_obj_set_style_text_color(tabLabel, lv_color_hex(i == currentTallyCounter ? 0xFFFFFF : tabColors[i]), 0);
        lv_obj_set_style_text_font(tabLabel, &lv_font_montserrat_14, 0);
        lv_obj_center(tabLabel);
    }

    // Main count display area
    lv_obj_t *countArea = lv_obj_create(card);
    lv_obj_set_size(countArea, LCD_WIDTH - 30, 180);
    lv_obj_align(countArea, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_color(countArea, lv_color_hex(tabColors[currentTallyCounter]), 0);
    lv_obj_set_style_bg_opa(countArea, LV_OPA_20, 0);
    lv_obj_set_style_radius(countArea, 20, 0);
    lv_obj_set_style_border_width(countArea, 2, 0);
    lv_obj_set_style_border_color(countArea, lv_color_hex(tabColors[currentTallyCounter]), 0);
    disableAllScrolling(countArea);

    // Large count number
    char countBuf[16];
    snprintf(countBuf, sizeof(countBuf), "%d", tallyCounters[currentTallyCounter]);
    lv_obj_t *countLabel = lv_label_create(countArea);
    lv_label_set_text(countLabel, countBuf);
    lv_obj_set_style_text_color(countLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(countLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(countLabel, LV_ALIGN_CENTER, 0, -10);

    // Tap hint
    lv_obj_t *tapHint = lv_label_create(countArea);
    lv_label_set_text(tapHint, "TAP TO COUNT");
    lv_obj_set_style_text_color(tapHint, lv_color_hex(tabColors[currentTallyCounter]), 0);
    lv_obj_set_style_text_font(tapHint, &lv_font_montserrat_12, 0);
    lv_obj_align(tapHint, LV_ALIGN_BOTTOM_MID, 0, -10);

    // Bottom buttons - 3 columns
    int btnW = (LCD_WIDTH - 40) / 3;
    int btnH = 55;
    int btnY = LCD_HEIGHT - btnH - 45;

    // Minus button
    lv_obj_t *minusBtn = lv_obj_create(card);
    lv_obj_set_size(minusBtn, btnW - 5, btnH);
    lv_obj_set_pos(minusBtn, 15, btnY);
    lv_obj_set_style_bg_color(minusBtn, lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_radius(minusBtn, 12, 0);
    lv_obj_set_style_border_width(minusBtn, 0, 0);
    disableAllScrolling(minusBtn);
    lv_obj_t *minusLabel = lv_label_create(minusBtn);
    lv_label_set_text(minusLabel, "-1");
    lv_obj_set_style_text_color(minusLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(minusLabel, &lv_font_montserrat_20, 0);
    lv_obj_center(minusLabel);

    // Reset current button
    lv_obj_t *resetBtn = lv_obj_create(card);
    lv_obj_set_size(resetBtn, btnW - 5, btnH);
    lv_obj_set_pos(resetBtn, 15 + btnW + 5, btnY);
    lv_obj_set_style_bg_color(resetBtn, lv_color_hex(0x636366), 0);
    lv_obj_set_style_radius(resetBtn, 12, 0);
    lv_obj_set_style_border_width(resetBtn, 0, 0);
    disableAllScrolling(resetBtn);
    lv_obj_t *resetLabel = lv_label_create(resetBtn);
    lv_label_set_text(resetLabel, "CLR");
    lv_obj_set_style_text_color(resetLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(resetLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(resetLabel);

    // Reset all button
    lv_obj_t *resetAllBtn = lv_obj_create(card);
    lv_obj_set_size(resetAllBtn, btnW - 5, btnH);
    lv_obj_set_pos(resetAllBtn, 15 + (btnW + 5) * 2, btnY);
    lv_obj_set_style_bg_color(resetAllBtn, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(resetAllBtn, 12, 0);
    lv_obj_set_style_border_width(resetAllBtn, 0, 0);
    disableAllScrolling(resetAllBtn);
    lv_obj_t *resetAllLabel = lv_label_create(resetAllBtn);
    lv_label_set_text(resetAllLabel, "ALL");
    lv_obj_set_style_text_color(resetAllLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(resetAllLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(resetAllLabel);

    // Total across all counters
    int total = 0;
    for (int i = 0; i < 4; i++) total += tallyCounters[i];
    char totalBuf[24];
    snprintf(totalBuf, sizeof(totalBuf), "Total: %d  " LV_SYMBOL_UP " TOOLS", total);
    lv_obj_t *totalLabel = lv_label_create(card);
    lv_label_set_text(totalLabel, totalBuf);
    lv_obj_set_style_text_color(totalLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(totalLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(totalLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// VOICE MEMO CARD - ES8311 Recording
// ═══════════════════════════════════════════════════════════════════════════

// Initialize ES8311 I2S for recording
bool initVoiceI2S() {
    if (voiceI2SInitialized) return true;
    
    // Enable PA
    pinMode(ES8311_PA_PIN, OUTPUT);
    digitalWrite(ES8311_PA_PIN, HIGH);
    
    // Initialize I2S
    voiceI2S.setPins(I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_DIN_PIN, I2S_MCLK_PIN);
    if (!voiceI2S.begin(I2S_MODE_STD, VOICE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
        USBSerial.println("[VOICE] Failed to init I2S!");
        return false;
    }
    
    // Initialize ES8311 codec
    es8311_handle_t es_handle = es8311_create(0, ES8311_ADDRESS_0);
    if (es_handle) {
        es8311_clock_config_t es_clk = {
            .mclk_inverted = false,
            .sclk_inverted = false,
            .mclk_from_mclk_pin = true,
            .mclk_frequency = VOICE_SAMPLE_RATE * 256,
            .sample_frequency = VOICE_SAMPLE_RATE
        };
        
        es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
        es8311_microphone_config(es_handle, false);
        es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_SETTING);
        es8311_voice_volume_set(es_handle, 85, NULL);
        USBSerial.println("[VOICE] ES8311 codec initialized");
    } else {
        USBSerial.println("[VOICE] ES8311 not found, using raw I2S");
    }
    
    voiceI2SInitialized = true;
    USBSerial.println("[VOICE] I2S initialized for recording");
    return true;
}

// Start recording voice memo to SD card
void startVoiceRecording() {
    if (!initVoiceI2S()) return;
    
    // Create memo folder if needed
    if (!SD_MMC.exists(VOICE_MEMO_FOLDER)) {
        SD_MMC.mkdir(VOICE_MEMO_FOLDER);
        USBSerial.println("[VOICE] Created memo folder");
    }
    
    // Generate filename with timestamp
    char filename[64];
    snprintf(filename, sizeof(filename), "%s/memo_%lu.raw", VOICE_MEMO_FOLDER, millis());
    
    voiceMemoFile = SD_MMC.open(filename, FILE_WRITE);
    if (!voiceMemoFile) {
        USBSerial.println("[VOICE] Failed to create file!");
        return;
    }
    
    voiceMemoRecording = true;
    voiceRecordStart = millis();
    voiceRecordDuration = 0;
    USBSerial.printf("[VOICE] Recording to: %s\n", filename);
}

// Stop and save recording
void stopVoiceRecording() {
    if (!voiceMemoRecording) return;
    
    voiceMemoRecording = false;
    voiceRecordDuration = (millis() - voiceRecordStart) / 1000;
    
    if (voiceMemoFile) {
        voiceMemoFile.close();
    }
    
    voiceMemoCount++;
    USBSerial.printf("[VOICE] Recording saved (%ds)\n", voiceRecordDuration);
}

// Record audio samples (call in loop when recording)
void recordVoiceSamples() {
    if (!voiceMemoRecording || !voiceMemoFile) return;
    
    static uint8_t audioBuffer[VOICE_BUF_SIZE];
    size_t bytesRead = voiceI2S.readBytes((char*)audioBuffer, VOICE_BUF_SIZE);
    
    if (bytesRead > 0) {
        voiceMemoFile.write(audioBuffer, bytesRead);
    }
}

// Count existing memos on SD
void countVoiceMemos() {
    if (!SD_MMC.exists(VOICE_MEMO_FOLDER)) {
        voiceMemoCount = 0;
        return;
    }
    
    File dir = SD_MMC.open(VOICE_MEMO_FOLDER);
    voiceMemoCount = 0;
    while (File entry = dir.openNextFile()) {
        if (!entry.isDirectory()) {
            voiceMemoCount++;
        }
        entry.close();
    }
    dir.close();
}

void createVoiceMemoCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_AUDIO " VOICE MEMO");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Recording indicator
    if (voiceMemoRecording) {
        // Recording animation - pulsing red circle
        lv_obj_t *recCircle = lv_obj_create(card);
        lv_obj_set_size(recCircle, 80, 80);
        lv_obj_align(recCircle, LV_ALIGN_CENTER, 0, -30);
        lv_obj_set_style_bg_color(recCircle, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_radius(recCircle, 40, 0);
        lv_obj_set_style_border_width(recCircle, 0, 0);
        disableAllScrolling(recCircle);

        // Duration
        int elapsed = (millis() - voiceRecordStart) / 1000;
        char durBuf[16];
        snprintf(durBuf, sizeof(durBuf), "%02d:%02d", elapsed / 60, elapsed % 60);
        lv_obj_t *durLabel = lv_label_create(card);
        lv_label_set_text(durLabel, durBuf);
        lv_obj_set_style_text_color(durLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(durLabel, &lv_font_montserrat_32, 0);
        lv_obj_align(durLabel, LV_ALIGN_CENTER, 0, 40);

        // Stop button
        lv_obj_t *stopBtn = lv_obj_create(card);
        lv_obj_set_size(stopBtn, LCD_WIDTH - 60, 50);
        lv_obj_align(stopBtn, LV_ALIGN_BOTTOM_MID, 0, -20);
        lv_obj_set_style_bg_color(stopBtn, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_radius(stopBtn, 25, 0);
        lv_obj_set_style_border_width(stopBtn, 0, 0);
        disableAllScrolling(stopBtn);

        lv_obj_t *stopLabel = lv_label_create(stopBtn);
        lv_label_set_text(stopLabel, LV_SYMBOL_STOP " STOP & SAVE");
        lv_obj_set_style_text_color(stopLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(stopLabel, &lv_font_montserrat_14, 0);
        lv_obj_center(stopLabel);
    } else {
        // Not recording - show record button
        
        // Mic icon
        lv_obj_t *micCircle = lv_obj_create(card);
        lv_obj_set_size(micCircle, 100, 100);
        lv_obj_align(micCircle, LV_ALIGN_CENTER, 0, -30);
        lv_obj_set_style_bg_color(micCircle, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(micCircle, 50, 0);
        lv_obj_set_style_border_width(micCircle, 3, 0);
        lv_obj_set_style_border_color(micCircle, lv_color_hex(0xFF453A), 0);
        disableAllScrolling(micCircle);

        lv_obj_t *micIcon = lv_label_create(micCircle);
        lv_label_set_text(micIcon, LV_SYMBOL_AUDIO);
        lv_obj_set_style_text_color(micIcon, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_text_font(micIcon, &lv_font_montserrat_32, 0);
        lv_obj_center(micIcon);

        // Memo count
        char memoBuf[32];
        snprintf(memoBuf, sizeof(memoBuf), "%d memo%s saved", voiceMemoCount, voiceMemoCount == 1 ? "" : "s");
        lv_obj_t *memoLabel = lv_label_create(card);
        lv_label_set_text(memoLabel, memoBuf);
        lv_obj_set_style_text_color(memoLabel, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(memoLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(memoLabel, LV_ALIGN_CENTER, 0, 50);

        // Record button
        lv_obj_t *recBtn = lv_obj_create(card);
        lv_obj_set_size(recBtn, LCD_WIDTH - 60, 50);
        lv_obj_align(recBtn, LV_ALIGN_BOTTOM_MID, 0, -20);
        lv_obj_set_style_bg_color(recBtn, lv_color_hex(0x30D158), 0);
        lv_obj_set_style_radius(recBtn, 25, 0);
        lv_obj_set_style_border_width(recBtn, 0, 0);
        disableAllScrolling(recBtn);

        lv_obj_t *recLabel = lv_label_create(recBtn);
        lv_label_set_text(recLabel, LV_SYMBOL_PLAY " TAP TO RECORD");
        lv_obj_set_style_text_color(recLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(recLabel, &lv_font_montserrat_14, 0);
        lv_obj_center(recLabel);
    }

    // SD folder path + back hint
    lv_obj_t *pathLabel = lv_label_create(card);
    lv_label_set_text(pathLabel, LV_SYMBOL_UP " TOOLS");
    lv_obj_set_style_text_color(pathLabel, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_text_font(pathLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(pathLabel, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// ═══════════════════════════════════════════════════════════════════════════
// DICE ROLLER CARD
// ═══════════════════════════════════════════════════════════════════════════
void createDiceRollerCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_REFRESH " DICE ROLLER");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Dice display boxes
    int diceSize = 80;
    int spacing = 20;

    // Dice 1
    lv_obj_t *dice1 = lv_obj_create(card);
    lv_obj_set_size(dice1, diceSize, diceSize);
    lv_obj_align(dice1, LV_ALIGN_CENTER, -(diceSize/2 + spacing/2), -20);
    lv_obj_set_style_bg_color(dice1, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(dice1, 15, 0);
    lv_obj_set_style_border_width(dice1, 0, 0);
    disableAllScrolling(dice1);

    char d1Buf[4];
    snprintf(d1Buf, sizeof(d1Buf), "%d", diceValue1);
    lv_obj_t *d1Label = lv_label_create(dice1);
    lv_label_set_text(d1Label, d1Buf);
    lv_obj_set_style_text_color(d1Label, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(d1Label, &lv_font_montserrat_48, 0);
    lv_obj_center(d1Label);

    // Dice 2
    lv_obj_t *dice2 = lv_obj_create(card);
    lv_obj_set_size(dice2, diceSize, diceSize);
    lv_obj_align(dice2, LV_ALIGN_CENTER, (diceSize/2 + spacing/2), -20);
    lv_obj_set_style_bg_color(dice2, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(dice2, 15, 0);
    lv_obj_set_style_border_width(dice2, 0, 0);
    disableAllScrolling(dice2);

    char d2Buf[4];
    snprintf(d2Buf, sizeof(d2Buf), "%d", diceValue2);
    lv_obj_t *d2Label = lv_label_create(dice2);
    lv_label_set_text(d2Label, d2Buf);
    lv_obj_set_style_text_color(d2Label, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(d2Label, &lv_font_montserrat_48, 0);
    lv_obj_center(d2Label);

    // Total
    char totalBuf[16];
    snprintf(totalBuf, sizeof(totalBuf), "Total: %d", diceValue1 + diceValue2);
    lv_obj_t *totalLabel = lv_label_create(card);
    lv_label_set_text(totalLabel, totalBuf);
    lv_obj_set_style_text_color(totalLabel, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(totalLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(totalLabel, LV_ALIGN_CENTER, 0, 70);

    // Hint
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_label_set_text(hintLabel, "TAP/SHAKE TO ROLL  " LV_SYMBOL_UP " BACK");
    lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ═══════════════════════════════════════════════════════════════════════════
// MAGIC 8 BALL CARD
// ═══════════════════════════════════════════════════════════════════════════
void createMagic8BallCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "MAGIC 8 BALL");
    lv_obj_set_style_text_color(title, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // 8 Ball circle
    lv_obj_t *ball = lv_obj_create(card);
    lv_obj_set_size(ball, 180, 180);
    lv_obj_align(ball, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_bg_color(ball, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(ball, 90, 0);
    lv_obj_set_style_border_width(ball, 4, 0);
    lv_obj_set_style_border_color(ball, lv_color_hex(0x3A3A3C), 0);
    disableAllScrolling(ball);

    // Inner triangle/window
    lv_obj_t *window = lv_obj_create(ball);
    lv_obj_set_size(window, 100, 100);
    lv_obj_center(window);
    lv_obj_set_style_bg_color(window, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_radius(window, 50, 0);
    lv_obj_set_style_border_width(window, 0, 0);
    disableAllScrolling(window);

    // Answer text
    lv_obj_t *answerLabel = lv_label_create(window);
    lv_label_set_text(answerLabel, magic8BallAnswer.c_str());
    lv_obj_set_style_text_color(answerLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(answerLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(answerLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(answerLabel);

    // Hint
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_label_set_text(hintLabel, "TAP/SHAKE TO ASK  " LV_SYMBOL_UP " BACK");
    lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ═══════════════════════════════════════════════════════════════════════════
// TILT MAZE GAME - Tilt wrist to roll ball through maze
// ═══════════════════════════════════════════════════════════════════════════
void resetTiltMaze() {
    mazeBallX = MAZE_START_X + 20;
    mazeBallY = MAZE_START_Y + 20;
    mazeBallVelX = 0;
    mazeBallVelY = 0;
    mazeGameWon = false;
    mazeStartTime = millis();
}

bool checkMazeCollision(float newX, float newY) {
    // Check screen boundaries
    if (newX < MAZE_BALL_RADIUS || newX > LCD_WIDTH - MAZE_BALL_RADIUS ||
        newY < 80 + MAZE_BALL_RADIUS || newY > LCD_HEIGHT - 30 - MAZE_BALL_RADIUS) {
        return true;
    }
    
    // Check wall collisions
    for (int i = 0; i < MAZE_WALL_COUNT; i++) {
        int wx = mazeWalls[i][0];
        int wy = mazeWalls[i][1];
        int ww = mazeWalls[i][2];
        int wh = mazeWalls[i][3];
        
        // Simple AABB collision with ball radius
        if (newX + MAZE_BALL_RADIUS > wx && newX - MAZE_BALL_RADIUS < wx + ww &&
            newY + MAZE_BALL_RADIUS > wy && newY - MAZE_BALL_RADIUS < wy + wh) {
            return true;
        }
    }
    return false;
}

bool checkMazeGoal() {
    // Check if ball is in goal area
    return (mazeBallX > MAZE_GOAL_X - MAZE_GOAL_SIZE/2 && 
            mazeBallX < MAZE_GOAL_X + MAZE_GOAL_SIZE/2 &&
            mazeBallY > MAZE_GOAL_Y - MAZE_GOAL_SIZE/2 && 
            mazeBallY < MAZE_GOAL_Y + MAZE_GOAL_SIZE/2);
}

void updateTiltMaze() {
    if (!mazeGameActive || mazeGameWon) return;
    
    // Read accelerometer for tilt
    if (hasIMU && qmi.getDataReady()) {
        qmi.getAccelerometer(acc.x, acc.y, acc.z);
        
        // Apply tilt to velocity (acc.x for left/right, acc.y for forward/back)
        mazeBallVelX += acc.x * MAZE_TILT_SENSITIVITY;
        mazeBallVelY += acc.y * MAZE_TILT_SENSITIVITY;
    }
    
    // Apply friction
    mazeBallVelX *= MAZE_FRICTION;
    mazeBallVelY *= MAZE_FRICTION;
    
    // Limit max speed
    float maxSpeed = 8.0f;
    if (mazeBallVelX > maxSpeed) mazeBallVelX = maxSpeed;
    if (mazeBallVelX < -maxSpeed) mazeBallVelX = -maxSpeed;
    if (mazeBallVelY > maxSpeed) mazeBallVelY = maxSpeed;
    if (mazeBallVelY < -maxSpeed) mazeBallVelY = -maxSpeed;
    
    // Calculate new position
    float newX = mazeBallX + mazeBallVelX;
    float newY = mazeBallY + mazeBallVelY;
    
    // Check collisions and update position
    if (!checkMazeCollision(newX, mazeBallY)) {
        mazeBallX = newX;
    } else {
        mazeBallVelX = -mazeBallVelX * 0.3f;  // Bounce back
    }
    
    if (!checkMazeCollision(mazeBallX, newY)) {
        mazeBallY = newY;
    } else {
        mazeBallVelY = -mazeBallVelY * 0.3f;  // Bounce back
    }
    
    // Check if reached goal
    if (checkMazeGoal()) {
        mazeGameWon = true;
        unsigned long finishTime = millis() - mazeStartTime;
        if (mazeBestTime == 0 || finishTime < mazeBestTime) {
            mazeBestTime = finishTime;
        }
    }
}

void createTiltMazeCard() {
    disableAllScrolling(lv_scr_act());
    
    // Update maze physics
    updateTiltMaze();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1A1A2E), 0);  // Dark blue background
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_GPS " TILT MAZE");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00D4FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Timer display
    unsigned long elapsed = mazeGameActive ? (millis() - mazeStartTime) : 0;
    int secs = (elapsed / 1000) % 60;
    int ms = (elapsed / 10) % 100;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d.%02d", secs, ms);
    
    lv_obj_t *timeLabel = lv_label_create(card);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(timeLabel, LV_ALIGN_TOP_MID, 0, 35);

    // Best time
    if (mazeBestTime > 0) {
        char bestBuf[24];
        snprintf(bestBuf, sizeof(bestBuf), "Best: %d.%02ds", (int)(mazeBestTime/1000), (int)((mazeBestTime/10)%100));
        lv_obj_t *bestLabel = lv_label_create(card);
        lv_label_set_text(bestLabel, bestBuf);
        lv_obj_set_style_text_color(bestLabel, lv_color_hex(0x30D158), 0);
        lv_obj_set_style_text_font(bestLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(bestLabel, LV_ALIGN_TOP_MID, 0, 55);
    }

    // Draw maze walls
    for (int i = 0; i < MAZE_WALL_COUNT; i++) {
        lv_obj_t *wall = lv_obj_create(card);
        lv_obj_set_size(wall, mazeWalls[i][2], mazeWalls[i][3]);
        lv_obj_set_pos(wall, mazeWalls[i][0], mazeWalls[i][1]);
        lv_obj_set_style_bg_color(wall, lv_color_hex(0x4A4A6A), 0);
        lv_obj_set_style_radius(wall, 2, 0);
        lv_obj_set_style_border_width(wall, 0, 0);
        disableAllScrolling(wall);
    }

    // Draw goal area
    lv_obj_t *goal = lv_obj_create(card);
    lv_obj_set_size(goal, MAZE_GOAL_SIZE, MAZE_GOAL_SIZE);
    lv_obj_set_pos(goal, MAZE_GOAL_X - MAZE_GOAL_SIZE/2, MAZE_GOAL_Y - MAZE_GOAL_SIZE/2);
    lv_obj_set_style_bg_color(goal, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_bg_opa(goal, LV_OPA_70, 0);
    lv_obj_set_style_radius(goal, 4, 0);
    lv_obj_set_style_border_width(goal, 2, 0);
    lv_obj_set_style_border_color(goal, lv_color_hex(0x30D158), 0);
    disableAllScrolling(goal);

    // Goal label
    lv_obj_t *goalLabel = lv_label_create(goal);
    lv_label_set_text(goalLabel, "GOAL");
    lv_obj_set_style_text_color(goalLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(goalLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(goalLabel);

    // Draw start area
    lv_obj_t *start = lv_obj_create(card);
    lv_obj_set_size(start, 30, 30);
    lv_obj_set_pos(start, MAZE_START_X, MAZE_START_Y);
    lv_obj_set_style_bg_color(start, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_bg_opa(start, LV_OPA_50, 0);
    lv_obj_set_style_radius(start, 4, 0);
    lv_obj_set_style_border_width(start, 1, 0);
    lv_obj_set_style_border_color(start, lv_color_hex(0xFF9F0A), 0);
    disableAllScrolling(start);

    // Draw ball
    lv_obj_t *ball = lv_obj_create(card);
    lv_obj_set_size(ball, MAZE_BALL_RADIUS * 2, MAZE_BALL_RADIUS * 2);
    lv_obj_set_pos(ball, (int)mazeBallX - MAZE_BALL_RADIUS, (int)mazeBallY - MAZE_BALL_RADIUS);
    lv_obj_set_style_bg_color(ball, lv_color_hex(0xFF3B30), 0);
    lv_obj_set_style_radius(ball, MAZE_BALL_RADIUS, 0);
    lv_obj_set_style_border_width(ball, 0, 0);
    lv_obj_set_style_shadow_width(ball, 8, 0);
    lv_obj_set_style_shadow_color(ball, lv_color_hex(0xFF3B30), 0);
    lv_obj_set_style_shadow_opa(ball, LV_OPA_50, 0);
    disableAllScrolling(ball);

    // Win message or hint
    lv_obj_t *hintLabel = lv_label_create(card);
    if (mazeGameWon) {
        char winBuf[48];
        snprintf(winBuf, sizeof(winBuf), "WIN! %d.%02ds  " LV_SYMBOL_UP " BACK", (int)(elapsed/1000), (int)((elapsed/10)%100));
        lv_label_set_text(hintLabel, winBuf);
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x30D158), 0);
    } else if (!mazeGameActive) {
        lv_label_set_text(hintLabel, "TAP TO START  " LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x636366), 0);
    } else {
        lv_label_set_text(hintLabel, "TILT WRIST  " LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x636366), 0);
    }
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ═══════════════════════════════════════════════════════════════════════════
// PONG GAME - Classic 2-paddle Pong: Player (LEFT) vs AI (RIGHT)
// ═══════════════════════════════════════════════════════════════════════════
void resetPongGame() {
    pongBallX = LCD_WIDTH / 2;
    pongBallY = LCD_HEIGHT / 2;
    pongBallVelX = (random(2) == 0 ? 6.0f : -6.0f);  // Fast ball
    pongBallVelY = (random(100) - 50) / 15.0f;       // Random vertical angle
    pongPaddleY = LCD_HEIGHT / 2 - PONG_PADDLE_HEIGHT / 2;  // Player (LEFT)
    pongCpuPaddleY = LCD_HEIGHT / 2 - PONG_PADDLE_HEIGHT / 2;  // AI (RIGHT)
    pongPlayerScore = 0;
    pongCpuScore = 0;
}

void updatePongGame() {
    if (!pongGameActive) return;
    
    // Move ball
    pongBallX += pongBallVelX;
    pongBallY += pongBallVelY;
    
    // Top/bottom wall collision
    if (pongBallY <= 50 + PONG_BALL_SIZE/2) {
        pongBallY = 50 + PONG_BALL_SIZE/2;
        pongBallVelY = -pongBallVelY;
    }
    if (pongBallY >= LCD_HEIGHT - 30 - PONG_BALL_SIZE/2) {
        pongBallY = LCD_HEIGHT - 30 - PONG_BALL_SIZE/2;
        pongBallVelY = -pongBallVelY;
    }
    
    // Player paddle collision (LEFT side) - player controls this
    int playerPaddleX = 20;
    if (pongBallX - PONG_BALL_SIZE/2 <= playerPaddleX + PONG_PADDLE_WIDTH && 
        pongBallX - PONG_BALL_SIZE/2 >= playerPaddleX &&
        pongBallY >= pongPaddleY && pongBallY <= pongPaddleY + PONG_PADDLE_HEIGHT) {
        pongBallX = playerPaddleX + PONG_PADDLE_WIDTH + PONG_BALL_SIZE/2;
        pongBallVelX = abs(pongBallVelX) + PONG_BALL_SPEED_INCREASE;  // Always go right
        // Add angle based on where ball hits paddle
        float hitPos = (pongBallY - pongPaddleY) / PONG_PADDLE_HEIGHT;
        pongBallVelY = (hitPos - 0.5f) * 8.0f;
    }
    
    // AI paddle collision (RIGHT side) - AI controls this
    int aiPaddleX = LCD_WIDTH - 20 - PONG_PADDLE_WIDTH;
    if (pongBallX + PONG_BALL_SIZE/2 >= aiPaddleX && 
        pongBallX + PONG_BALL_SIZE/2 <= aiPaddleX + PONG_PADDLE_WIDTH &&
        pongBallY >= pongCpuPaddleY && pongBallY <= pongCpuPaddleY + PONG_PADDLE_HEIGHT) {
        pongBallX = aiPaddleX - PONG_BALL_SIZE/2;
        pongBallVelX = -(abs(pongBallVelX) + PONG_BALL_SPEED_INCREASE);  // Always go left
        float hitPos = (pongBallY - pongCpuPaddleY) / PONG_PADDLE_HEIGHT;
        pongBallVelY = (hitPos - 0.5f) * 8.0f;
    }
    
    // AI movement - follows ball with some prediction
    float aiTarget = pongBallY - PONG_PADDLE_HEIGHT / 2;
    // Add some prediction when ball is coming towards AI
    if (pongBallVelX > 0) {
        float timeToReach = (aiPaddleX - pongBallX) / pongBallVelX;
        aiTarget = pongBallY + pongBallVelY * timeToReach * 0.5f - PONG_PADDLE_HEIGHT / 2;
    }
    
    if (pongCpuPaddleY < aiTarget - 3) {
        pongCpuPaddleY += PONG_CPU_SPEED;
    } else if (pongCpuPaddleY > aiTarget + 3) {
        pongCpuPaddleY -= PONG_CPU_SPEED;
    }
    
    // Keep AI paddle in bounds
    if (pongCpuPaddleY < 55) pongCpuPaddleY = 55;
    if (pongCpuPaddleY > LCD_HEIGHT - 35 - PONG_PADDLE_HEIGHT) pongCpuPaddleY = LCD_HEIGHT - 35 - PONG_PADDLE_HEIGHT;
    
    // Scoring - ball past paddles
    if (pongBallX < 10) {
        // AI scores (ball went past player on left)
        pongCpuScore++;
        pongBallX = LCD_WIDTH / 2;
        pongBallY = LCD_HEIGHT / 2;
        pongBallVelX = 6.0f;  // Ball goes right (towards AI)
        pongBallVelY = (random(100) - 50) / 15.0f;
    }
    if (pongBallX > LCD_WIDTH - 10) {
        // Player scores (ball went past AI on right)
        pongPlayerScore++;
        pongBallX = LCD_WIDTH / 2;
        pongBallY = LCD_HEIGHT / 2;
        pongBallVelX = -6.0f;  // Ball goes left (towards player)
        pongBallVelY = (random(100) - 50) / 15.0f;
    }
    
    // Game over at 5 points
    if (pongPlayerScore >= 5 || pongCpuScore >= 5) {
        pongGameActive = false;
    }
}

void createPongCard() {
    disableAllScrolling(lv_scr_act());
    
    // Update game state
    updatePongGame();
    
    // Handle touch for player paddle movement (LEFT paddle)
    if (pongGameActive && touchActive) {
        pongPaddleY = touchCurrentY - PONG_PADDLE_HEIGHT / 2;
        // Keep paddle in bounds
        if (pongPaddleY < 55) pongPaddleY = 55;
        if (pongPaddleY > LCD_HEIGHT - 35 - PONG_PADDLE_HEIGHT) pongPaddleY = LCD_HEIGHT - 35 - PONG_PADDLE_HEIGHT;
    }

    // Pure black background - classic Pong style
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Score display at top - Player score LEFT, AI score RIGHT
    // Player score (left side)
    char playerScoreBuf[8];
    snprintf(playerScoreBuf, sizeof(playerScoreBuf), "%d", pongPlayerScore);
    lv_obj_t *playerScoreLabel = lv_label_create(card);
    lv_label_set_text(playerScoreLabel, playerScoreBuf);
    lv_obj_set_style_text_color(playerScoreLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(playerScoreLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_pos(playerScoreLabel, LCD_WIDTH / 4 - 10, 12);
    
    // AI score (right side)
    char aiScoreBuf[8];
    snprintf(aiScoreBuf, sizeof(aiScoreBuf), "%d", pongCpuScore);
    lv_obj_t *aiScoreLabel = lv_label_create(card);
    lv_label_set_text(aiScoreLabel, aiScoreBuf);
    lv_obj_set_style_text_color(aiScoreLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(aiScoreLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_pos(aiScoreLabel, LCD_WIDTH * 3 / 4 - 10, 12);

    // Center dotted line - classic white segments
    int dashHeight = 15;
    int dashGap = 12;
    for (int y = 50; y < LCD_HEIGHT - 30; y += dashHeight + dashGap) {
        lv_obj_t *dash = lv_obj_create(card);
        lv_obj_set_size(dash, 4, dashHeight);
        lv_obj_set_pos(dash, LCD_WIDTH / 2 - 2, y);
        lv_obj_set_style_bg_color(dash, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_radius(dash, 0, 0);
        lv_obj_set_style_border_width(dash, 0, 0);
        disableAllScrolling(dash);
    }

    // Player Paddle (LEFT) - white with glow
    lv_obj_t *playerPaddle = lv_obj_create(card);
    lv_obj_set_size(playerPaddle, PONG_PADDLE_WIDTH, PONG_PADDLE_HEIGHT);
    lv_obj_set_pos(playerPaddle, 20, (int)pongPaddleY);
    lv_obj_set_style_bg_color(playerPaddle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(playerPaddle, 2, 0);
    lv_obj_set_style_border_width(playerPaddle, 0, 0);
    lv_obj_set_style_shadow_width(playerPaddle, 10, 0);
    lv_obj_set_style_shadow_color(playerPaddle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_shadow_opa(playerPaddle, LV_OPA_50, 0);
    disableAllScrolling(playerPaddle);

    // AI Paddle (RIGHT) - white with glow
    lv_obj_t *aiPaddle = lv_obj_create(card);
    lv_obj_set_size(aiPaddle, PONG_PADDLE_WIDTH, PONG_PADDLE_HEIGHT);
    lv_obj_set_pos(aiPaddle, LCD_WIDTH - 20 - PONG_PADDLE_WIDTH, (int)pongCpuPaddleY);
    lv_obj_set_style_bg_color(aiPaddle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(aiPaddle, 2, 0);
    lv_obj_set_style_border_width(aiPaddle, 0, 0);
    lv_obj_set_style_shadow_width(aiPaddle, 10, 0);
    lv_obj_set_style_shadow_color(aiPaddle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_shadow_opa(aiPaddle, LV_OPA_50, 0);
    disableAllScrolling(aiPaddle);

    // Ball - white square (classic Pong style)
    lv_obj_t *ball = lv_obj_create(card);
    lv_obj_set_size(ball, PONG_BALL_SIZE, PONG_BALL_SIZE);
    lv_obj_set_pos(ball, (int)pongBallX - PONG_BALL_SIZE/2, (int)pongBallY - PONG_BALL_SIZE/2);
    lv_obj_set_style_bg_color(ball, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(ball, 0, 0);  // Square ball - classic!
    lv_obj_set_style_border_width(ball, 0, 0);
    lv_obj_set_style_shadow_width(ball, 8, 0);
    lv_obj_set_style_shadow_color(ball, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_shadow_opa(ball, LV_OPA_60, 0);
    disableAllScrolling(ball);

    // Show UI elements when game is not active
    if (!pongGameActive) {
        lv_obj_t *hintLabel = lv_label_create(card);
        if (pongPlayerScore >= 5) {
            lv_label_set_text(hintLabel, "YOU WIN!");
            lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x30D158), 0);
        } else if (pongCpuScore >= 5) {
            lv_label_set_text(hintLabel, "AI WINS!");
            lv_obj_set_style_text_color(hintLabel, lv_color_hex(0xFF453A), 0);
        } else {
            lv_label_set_text(hintLabel, "TAP TO START");
            lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x888888), 0);
        }
        lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(hintLabel, LV_ALIGN_CENTER, 0, 0);
        
        // Back hint
        lv_obj_t *backHint = lv_label_create(card);
        lv_label_set_text(backHint, LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(backHint, lv_color_hex(0x444444), 0);
        lv_obj_set_style_text_font(backHint, &lv_font_montserrat_12, 0);
        lv_obj_align(backHint, LV_ALIGN_BOTTOM_MID, 0, -8);
    } else {
        // Show "YOU" and "AI" labels during gameplay
        lv_obj_t *youLabel = lv_label_create(card);
        lv_label_set_text(youLabel, "YOU");
        lv_obj_set_style_text_color(youLabel, lv_color_hex(0x555555), 0);
        lv_obj_set_style_text_font(youLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(youLabel, LV_ALIGN_BOTTOM_LEFT, 20, -8);
        
        lv_obj_t *aiLabel = lv_label_create(card);
        lv_label_set_text(aiLabel, "AI");
        lv_obj_set_style_text_color(aiLabel, lv_color_hex(0x555555), 0);
        lv_obj_set_style_text_font(aiLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(aiLabel, LV_ALIGN_BOTTOM_RIGHT, -25, -8);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GAME SELECTOR CARD - Icons to select games
// ═══════════════════════════════════════════════════════════════════════════
void createGameSelectorCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    // Premium dark gradient background
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_bg_grad_color(card, lv_color_hex(0x1A1A2E), 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Premium title with glow effect
    lv_obj_t *titleGlow = lv_label_create(card);
    lv_label_set_text(titleGlow, "GAMES");
    lv_obj_set_style_text_color(titleGlow, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(titleGlow, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_opa(titleGlow, LV_OPA_30, 0);
    lv_obj_align(titleGlow, LV_ALIGN_TOP_MID, 1, 11);
    
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "GAMES");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Navigation hint with subtle styling
    lv_obj_t *hintUp = lv_label_create(card);
    lv_label_set_text(hintUp, LV_SYMBOL_UP " BLACKJACK");
    lv_obj_set_style_text_color(hintUp, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_text_font(hintUp, &lv_font_montserrat_12, 0);
    lv_obj_align(hintUp, LV_ALIGN_TOP_MID, 0, 38);

    // Premium grid layout: 3x3 icons with glass-morphism effect
    int iconSize = 90;
    int spacing = 10;
    int startX = (LCD_WIDTH - (iconSize * 3 + spacing * 2)) / 2;
    int startY = 58;

    // Game data: 9 games with premium colors
    const char* gameNames[] = {"DICE", "8-BALL", "MAZE", "PONG", "TTT", "2048", "WORDLE", "BREAK", "TETRIS"};
    const char* gameSymbols[] = {LV_SYMBOL_REFRESH, LV_SYMBOL_EYE_OPEN, LV_SYMBOL_GPS, LV_SYMBOL_PLAY, LV_SYMBOL_CLOSE, LV_SYMBOL_PLUS, "W", LV_SYMBOL_STOP, LV_SYMBOL_LIST};
    // Premium neon color palette
    uint32_t gameColors[] = {
        0x7C3AED,  // Violet
        0x0EA5E9,  // Cyan
        0x10B981,  // Emerald
        0xF59E0B,  // Amber
        0xEF4444,  // Red
        0xEC4899,  // Pink
        0x6366F1,  // Indigo (Wordle)
        0xF97316,  // Orange
        0x8B5CF6   // Purple
    };

    for (int i = 0; i < 9; i++) {
        int row = i / 3;
        int col = i % 3;
        int posX = startX + col * (iconSize + spacing);
        int posY = startY + row * (iconSize + spacing);

        // Premium glass-morphism button with gradient
        lv_obj_t *btn = lv_obj_create(card);
        lv_obj_set_size(btn, iconSize, iconSize);
        lv_obj_set_pos(btn, posX, posY);
        
        // Glass effect background
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_bg_grad_color(btn, lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_90, 0);
        lv_obj_set_style_radius(btn, 18, 0);
        
        // Subtle colored border
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(gameColors[i]), 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_60, 0);
        
        // Premium shadow with color tint
        lv_obj_set_style_shadow_width(btn, 15, 0);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(gameColors[i]), 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_40, 0);
        lv_obj_set_style_shadow_spread(btn, 2, 0);
        disableAllScrolling(btn);

        // Icon with glow effect (shadow behind)
        lv_obj_t *iconGlow = lv_label_create(btn);
        lv_label_set_text(iconGlow, gameSymbols[i]);
        lv_obj_set_style_text_color(iconGlow, lv_color_hex(gameColors[i]), 0);
        lv_obj_set_style_text_font(iconGlow, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_opa(iconGlow, LV_OPA_40, 0);
        lv_obj_align(iconGlow, LV_ALIGN_CENTER, 1, -7);
        
        // Main icon
        lv_obj_t *icon = lv_label_create(btn);
        lv_label_set_text(icon, gameSymbols[i]);
        lv_obj_set_style_text_color(icon, lv_color_hex(gameColors[i]), 0);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_24, 0);
        lv_obj_align(icon, LV_ALIGN_CENTER, 0, -8);

        // Label with clean styling
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, gameNames[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xE5E5E5), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -6);
    }

    // Premium bottom hint
    lv_obj_t *hintTap = lv_label_create(card);
    lv_label_set_text(hintTap, "TAP TO PLAY");
    lv_obj_set_style_text_color(hintTap, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_text_font(hintTap, &lv_font_montserrat_12, 0);
    lv_obj_align(hintTap, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// TIC-TAC-TOE GAME - Player vs AI
// ═══════════════════════════════════════════════════════════════════════════
void resetTicTacToe() {
    for (int i = 0; i < 9; i++) tttBoard[i] = 0;
    tttPlayerTurn = true;
    tttGameOver = false;
    tttWinner = 0;
}

int checkTTTWinner() {
    // Check rows, cols, diagonals
    int lines[8][3] = {
        {0,1,2}, {3,4,5}, {6,7,8},  // rows
        {0,3,6}, {1,4,7}, {2,5,8},  // cols
        {0,4,8}, {2,4,6}            // diagonals
    };
    for (int i = 0; i < 8; i++) {
        int a = lines[i][0], b = lines[i][1], c = lines[i][2];
        if (tttBoard[a] != 0 && tttBoard[a] == tttBoard[b] && tttBoard[b] == tttBoard[c]) {
            return tttBoard[a];  // 1=player wins, 2=AI wins
        }
    }
    // Check for draw
    bool full = true;
    for (int i = 0; i < 9; i++) {
        if (tttBoard[i] == 0) { full = false; break; }
    }
    return full ? -1 : 0;  // -1=draw, 0=game continues
}

int tttMinimax(bool isMaximizing) {
    int winner = checkTTTWinner();
    if (winner == 2) return 10;   // AI wins
    if (winner == 1) return -10;  // Player wins
    if (winner == -1) return 0;   // Draw
    
    if (isMaximizing) {
        int best = -1000;
        for (int i = 0; i < 9; i++) {
            if (tttBoard[i] == 0) {
                tttBoard[i] = 2;
                best = max(best, tttMinimax(false));
                tttBoard[i] = 0;
            }
        }
        return best;
    } else {
        int best = 1000;
        for (int i = 0; i < 9; i++) {
            if (tttBoard[i] == 0) {
                tttBoard[i] = 1;
                best = min(best, tttMinimax(true));
                tttBoard[i] = 0;
            }
        }
        return best;
    }
}

void tttAiMove() {
    int bestScore = -1000;
    int bestMove = -1;
    
    for (int i = 0; i < 9; i++) {
        if (tttBoard[i] == 0) {
            tttBoard[i] = 2;
            int score = tttMinimax(false);
            tttBoard[i] = 0;
            if (score > bestScore) {
                bestScore = score;
                bestMove = i;
            }
        }
    }
    
    if (bestMove != -1) {
        tttBoard[bestMove] = 2;
    }
}

void createTicTacToeCard() {
    disableAllScrolling(lv_scr_act());

    // Light/clean background like the reference image
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0xF5F5F7), 0);  // Light gray
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title / Status
    lv_obj_t *title = lv_label_create(card);
    if (tttGameOver) {
        if (tttWinner == 1) lv_label_set_text(title, "You Win!");
        else if (tttWinner == 2) lv_label_set_text(title, "AI Wins!");
        else lv_label_set_text(title, "Draw!");
    } else {
        lv_label_set_text(title, tttPlayerTurn ? "Your turn" : "AI thinking...");
    }
    lv_obj_set_style_text_color(title, lv_color_hex(0x333333), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Score display
    char scoreBuf[24];
    snprintf(scoreBuf, sizeof(scoreBuf), "You: %d  AI: %d", tttPlayerWins, tttAiWins);
    lv_obj_t *scoreLabel = lv_label_create(card);
    lv_label_set_text(scoreLabel, scoreBuf);
    lv_obj_set_style_text_color(scoreLabel, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(scoreLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(scoreLabel, LV_ALIGN_TOP_MID, 0, 45);

    // 3x3 Grid
    int cellSize = 85;
    int cellGap = 8;
    int gridStartX = (LCD_WIDTH - (cellSize * 3 + cellGap * 2)) / 2;
    int gridStartY = 90;

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            int idx = row * 3 + col;
            int posX = gridStartX + col * (cellSize + cellGap);
            int posY = gridStartY + row * (cellSize + cellGap);

            // Cell background (rounded card style)
            lv_obj_t *cell = lv_obj_create(card);
            lv_obj_set_size(cell, cellSize, cellSize);
            lv_obj_set_pos(cell, posX, posY);
            lv_obj_set_style_bg_color(cell, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_radius(cell, 12, 0);
            lv_obj_set_style_border_width(cell, 0, 0);
            lv_obj_set_style_shadow_width(cell, 8, 0);
            lv_obj_set_style_shadow_color(cell, lv_color_hex(0x000000), 0);
            lv_obj_set_style_shadow_opa(cell, LV_OPA_10, 0);
            disableAllScrolling(cell);

            // X or O symbol
            if (tttBoard[idx] != 0) {
                lv_obj_t *symbol = lv_label_create(cell);
                if (tttBoard[idx] == 1) {
                    lv_label_set_text(symbol, "X");
                    lv_obj_set_style_text_color(symbol, lv_color_hex(0x5B7FFF), 0);  // Blue X
                } else {
                    lv_label_set_text(symbol, "O");
                    lv_obj_set_style_text_color(symbol, lv_color_hex(0xFF6B9D), 0);  // Pink O
                }
                lv_obj_set_style_text_font(symbol, &lv_font_montserrat_48, 0);
                lv_obj_center(symbol);
            }
        }
    }

    // Bottom hint
    lv_obj_t *hintLabel = lv_label_create(card);
    if (tttGameOver) {
        lv_label_set_text(hintLabel, "TAP TO PLAY AGAIN");
    } else {
        lv_label_set_text(hintLabel, LV_SYMBOL_UP " BACK");
    }
    lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// ═══════════════════════════════════════════════════════════════════════════
// 2048 GAME - Swipe to combine tiles
// ═══════════════════════════════════════════════════════════════════════════
uint32_t getTileColor(int value) {
    switch(value) {
        case 2:    return 0xEEE4DA;
        case 4:    return 0xEDE0C8;
        case 8:    return 0xF2B179;
        case 16:   return 0xF59563;
        case 32:   return 0xF67C5F;
        case 64:   return 0xF65E3B;
        case 128:  return 0xEDCF72;
        case 256:  return 0xEDCC61;
        case 512:  return 0xEDC850;
        case 1024: return 0xEDC53F;
        case 2048: return 0xEDC22E;
        default:   return 0xCDC1B4;  // Empty
    }
}

uint32_t getTileTextColor(int value) {
    return (value <= 4) ? 0x776E65 : 0xF9F6F2;
}

void reset2048() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            grid2048[i][j] = 0;
        }
    }
    score2048 = 0;
    game2048Over = false;
    game2048Won = false;
    add2048Tile();
    add2048Tile();
}

void add2048Tile() {
    int empty[16][2];
    int count = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (grid2048[i][j] == 0) {
                empty[count][0] = i;
                empty[count][1] = j;
                count++;
            }
        }
    }
    if (count > 0) {
        int idx = random(count);
        grid2048[empty[idx][0]][empty[idx][1]] = (random(10) < 9) ? 2 : 4;
    }
}

bool move2048(int direction) {
    bool moved = false;
    int temp[4][4];
    
    // Copy grid
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            temp[i][j] = grid2048[i][j];
    
    if (direction == 0) {  // Up
        for (int j = 0; j < 4; j++) {
            int merged[4] = {0};
            for (int i = 1; i < 4; i++) {
                if (grid2048[i][j] != 0) {
                    int k = i;
                    while (k > 0 && grid2048[k-1][j] == 0) {
                        grid2048[k-1][j] = grid2048[k][j];
                        grid2048[k][j] = 0;
                        k--;
                    }
                    if (k > 0 && grid2048[k-1][j] == grid2048[k][j] && !merged[k-1]) {
                        grid2048[k-1][j] *= 2;
                        score2048 += grid2048[k-1][j];
                        if (grid2048[k-1][j] == 2048) game2048Won = true;
                        grid2048[k][j] = 0;
                        merged[k-1] = 1;
                    }
                }
            }
        }
    } else if (direction == 2) {  // Down
        for (int j = 0; j < 4; j++) {
            int merged[4] = {0};
            for (int i = 2; i >= 0; i--) {
                if (grid2048[i][j] != 0) {
                    int k = i;
                    while (k < 3 && grid2048[k+1][j] == 0) {
                        grid2048[k+1][j] = grid2048[k][j];
                        grid2048[k][j] = 0;
                        k++;
                    }
                    if (k < 3 && grid2048[k+1][j] == grid2048[k][j] && !merged[k+1]) {
                        grid2048[k+1][j] *= 2;
                        score2048 += grid2048[k+1][j];
                        if (grid2048[k+1][j] == 2048) game2048Won = true;
                        grid2048[k][j] = 0;
                        merged[k+1] = 1;
                    }
                }
            }
        }
    } else if (direction == 3) {  // Left
        for (int i = 0; i < 4; i++) {
            int merged[4] = {0};
            for (int j = 1; j < 4; j++) {
                if (grid2048[i][j] != 0) {
                    int k = j;
                    while (k > 0 && grid2048[i][k-1] == 0) {
                        grid2048[i][k-1] = grid2048[i][k];
                        grid2048[i][k] = 0;
                        k--;
                    }
                    if (k > 0 && grid2048[i][k-1] == grid2048[i][k] && !merged[k-1]) {
                        grid2048[i][k-1] *= 2;
                        score2048 += grid2048[i][k-1];
                        if (grid2048[i][k-1] == 2048) game2048Won = true;
                        grid2048[i][k] = 0;
                        merged[k-1] = 1;
                    }
                }
            }
        }
    } else if (direction == 1) {  // Right
        for (int i = 0; i < 4; i++) {
            int merged[4] = {0};
            for (int j = 2; j >= 0; j--) {
                if (grid2048[i][j] != 0) {
                    int k = j;
                    while (k < 3 && grid2048[i][k+1] == 0) {
                        grid2048[i][k+1] = grid2048[i][k];
                        grid2048[i][k] = 0;
                        k++;
                    }
                    if (k < 3 && grid2048[i][k+1] == grid2048[i][k] && !merged[k+1]) {
                        grid2048[i][k+1] *= 2;
                        score2048 += grid2048[i][k+1];
                        if (grid2048[i][k+1] == 2048) game2048Won = true;
                        grid2048[i][k] = 0;
                        merged[k+1] = 1;
                    }
                }
            }
        }
    }
    
    // Check if moved
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (temp[i][j] != grid2048[i][j]) moved = true;
    
    if (score2048 > bestScore2048) bestScore2048 = score2048;
    return moved;
}

bool canMove2048() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (grid2048[i][j] == 0) return true;
            if (i < 3 && grid2048[i][j] == grid2048[i+1][j]) return true;
            if (j < 3 && grid2048[i][j] == grid2048[i][j+1]) return true;
        }
    }
    return false;
}

void create2048Card() {
    disableAllScrolling(lv_scr_act());

    // Initialize game if empty
    bool isEmpty = true;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (grid2048[i][j] != 0) isEmpty = false;
    if (isEmpty) reset2048();

    // Warm beige background
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0xFAF8EF), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "2048");
    lv_obj_set_style_text_color(title, lv_color_hex(0x776E65), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    // Score
    char scoreBuf[32];
    snprintf(scoreBuf, sizeof(scoreBuf), "SCORE\n%d", score2048);
    lv_obj_t *scoreBox = lv_obj_create(card);
    lv_obj_set_size(scoreBox, 70, 40);
    lv_obj_align(scoreBox, LV_ALIGN_TOP_RIGHT, -85, 10);
    lv_obj_set_style_bg_color(scoreBox, lv_color_hex(0xBBADA0), 0);
    lv_obj_set_style_radius(scoreBox, 6, 0);
    lv_obj_set_style_border_width(scoreBox, 0, 0);
    disableAllScrolling(scoreBox);
    
    lv_obj_t *scoreLabel = lv_label_create(scoreBox);
    lv_label_set_text(scoreLabel, scoreBuf);
    lv_obj_set_style_text_color(scoreLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(scoreLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(scoreLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(scoreLabel);

    // Best score
    char bestBuf[32];
    snprintf(bestBuf, sizeof(bestBuf), "BEST\n%d", bestScore2048);
    lv_obj_t *bestBox = lv_obj_create(card);
    lv_obj_set_size(bestBox, 70, 40);
    lv_obj_align(bestBox, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(bestBox, lv_color_hex(0xBBADA0), 0);
    lv_obj_set_style_radius(bestBox, 6, 0);
    lv_obj_set_style_border_width(bestBox, 0, 0);
    disableAllScrolling(bestBox);
    
    lv_obj_t *bestLabel = lv_label_create(bestBox);
    lv_label_set_text(bestLabel, bestBuf);
    lv_obj_set_style_text_color(bestLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(bestLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(bestLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(bestLabel);

    // Grid container - SMALLER to fit fully on screen
    int gridSize = 240;    // Smaller grid (was 260, originally 280)
    int cellSize = 55;     // Smaller cells (was 60)
    int cellGap = 5;       // Gap between cells

    lv_obj_t *gridBg = lv_obj_create(card);
    lv_obj_set_size(gridBg, gridSize, gridSize);
    lv_obj_align(gridBg, LV_ALIGN_CENTER, 0, 15);  // Center with slight offset down
    lv_obj_set_style_bg_color(gridBg, lv_color_hex(0xBBADA0), 0);
    lv_obj_set_style_radius(gridBg, 8, 0);
    lv_obj_set_style_border_width(gridBg, 0, 0);
    disableAllScrolling(gridBg);

    // Draw tiles
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            int posX = cellGap + col * (cellSize + cellGap);
            int posY = cellGap + row * (cellSize + cellGap);
            int val = grid2048[row][col];

            lv_obj_t *tile = lv_obj_create(gridBg);
            lv_obj_set_size(tile, cellSize, cellSize);
            lv_obj_set_pos(tile, posX, posY);
            lv_obj_set_style_bg_color(tile, lv_color_hex(getTileColor(val)), 0);
            lv_obj_set_style_radius(tile, 6, 0);
            lv_obj_set_style_border_width(tile, 0, 0);
            disableAllScrolling(tile);

            if (val > 0) {
                lv_obj_t *numLabel = lv_label_create(tile);
                char numBuf[8];
                snprintf(numBuf, sizeof(numBuf), "%d", val);
                lv_label_set_text(numLabel, numBuf);
                lv_obj_set_style_text_color(numLabel, lv_color_hex(getTileTextColor(val)), 0);
                // Smaller font for larger numbers
                if (val >= 1024) {
                    lv_obj_set_style_text_font(numLabel, &lv_font_montserrat_16, 0);
                } else if (val >= 128) {
                    lv_obj_set_style_text_font(numLabel, &lv_font_montserrat_20, 0);
                } else {
                    lv_obj_set_style_text_font(numLabel, &lv_font_montserrat_24, 0);
                }
                lv_obj_center(numLabel);
            }
        }
    }

    // Status / hint - CENTERED
    lv_obj_t *hintLabel = lv_label_create(card);
    lv_obj_set_width(hintLabel, LCD_WIDTH);  // Full width for proper centering
    lv_obj_set_style_text_align(hintLabel, LV_TEXT_ALIGN_CENTER, 0);  // Center text
    if (game2048Over) {
        lv_label_set_text(hintLabel, "GAME OVER - TAP TO RESTART");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x776E65), 0);
    } else if (game2048Won) {
        lv_label_set_text(hintLabel, "YOU WIN! KEEP GOING?");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x30D158), 0);
    } else {
        lv_label_set_text(hintLabel, "SWIPE TO PLAY  " LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x888888), 0);
    }
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// ═══════════════════════════════════════════════════════════════════════════
// WORDLE GAME - 5-letter word guessing
// ═══════════════════════════════════════════════════════════════════════════
const char* defaultWordList[] = {
    "WATCH", "SMART", "GAMES", "PLAYS", "CLOCK", "TIMER", "ALARM", "HEART", "STEPS", "SLEEP",
    "MUSIC", "PHONE", "LIGHT", "POWER", "QUICK", "SPEED", "TOUCH", "SWIPE", "PRESS", "CLICK",
    "WORLD", "EARTH", "OCEAN", "RIVER", "MOUNT", "CLOUD", "STORM", "SUNNY", "RAINY", "WINDY",
    "APPLE", "GRAPE", "LEMON", "MANGO", "PEACH", "BERRY", "MELON", "OLIVE", "ONION", "BREAD",
    "HOUSE", "CHAIR", "TABLE", "FLOOR", "DOORS", "WALLS", "ROOMS", "BATHS", "COUCH", "SHELF",
    "HAPPY", "SMILE", "LAUGH", "CHEER", "DREAM", "THINK", "LEARN", "TEACH", "WRITE", "SPEAK",
    "BLACK", "WHITE", "GREEN", "BROWN", "CORAL", "CREAM", "FROST", "AMBER", "BEIGE", "IVORY",
    "AUDIO", "VIDEO", "RADIO", "MEDIA", "SHARE", "STORY", "PHOTO", "IMAGE", "ALBUM", "FRAME",
    "MONEY", "COINS", "CARDS", "FUNDS", "TRADE", "STOCK", "PRIZE", "BONUS", "SCORE", "POINT",
    "TRAIN", "PLANE", "SHIPS", "BIKES", "BUSES", "TRUCK", "WHEEL", "DRIVE", "ROADS", "PATHS"
};
const int DEFAULT_WORD_COUNT = 100;

void loadWordleWords() {
    if (wordleWordsLoaded) return;
    
    // Try to load from SD card
    String path = "/WATCH/GAMES/words.txt";
    if (SD_MMC.exists(path)) {
        File f = SD_MMC.open(path, FILE_READ);
        if (f) {
            wordleWordCount = 0;
            while (f.available() && wordleWordCount < 1000) {
                String word = f.readStringUntil('\n');
                word.trim();
                word.toUpperCase();
                if (word.length() == 5) {
                    wordleWordList[wordleWordCount++] = word;
                }
            }
            f.close();
        }
    }
    
    // If no words loaded, create default file and use defaults
    if (wordleWordCount == 0) {
        // Create directory and file with defaults
        if (!SD_MMC.exists("/WATCH/GAMES")) {
            SD_MMC.mkdir("/WATCH/GAMES");
        }
        File f = SD_MMC.open(path, FILE_WRITE);
        if (f) {
            for (int i = 0; i < DEFAULT_WORD_COUNT; i++) {
                f.println(defaultWordList[i]);
            }
            f.close();
        }
        // Use default words
        for (int i = 0; i < DEFAULT_WORD_COUNT; i++) {
            wordleWordList[i] = String(defaultWordList[i]);
        }
        wordleWordCount = DEFAULT_WORD_COUNT;
    }
    
    wordleWordsLoaded = true;
}

void resetWordle() {
    loadWordleWords();
    
    // Pick random word
    if (wordleWordCount > 0) {
        String word = wordleWordList[random(wordleWordCount)];
        strncpy(wordleTarget, word.c_str(), 5);
        wordleTarget[5] = '\0';
    }
    
    // Clear guesses
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            wordleGuesses[i][j] = '\0';
        }
    }
    wordleCurrentRow = 0;
    wordleCurrentCol = 0;
    wordleGameOver = false;
    wordleWon = false;
}

void wordleAddLetter(char c) {
    if (wordleGameOver || wordleCurrentCol >= 5) return;
    wordleGuesses[wordleCurrentRow][wordleCurrentCol++] = c;
}

void wordleDeleteLetter() {
    if (wordleGameOver || wordleCurrentCol <= 0) return;
    wordleGuesses[wordleCurrentRow][--wordleCurrentCol] = '\0';
}

void wordleSubmitGuess() {
    if (wordleGameOver || wordleCurrentCol < 5) return;
    
    // Check if won
    bool won = true;
    for (int i = 0; i < 5; i++) {
        if (wordleGuesses[wordleCurrentRow][i] != wordleTarget[i]) {
            won = false;
            break;
        }
    }
    
    if (won) {
        wordleWon = true;
        wordleGameOver = true;
        wordleGamesWon++;
        wordleGamesPlayed++;
    } else {
        wordleCurrentRow++;
        wordleCurrentCol = 0;
        if (wordleCurrentRow >= 6) {
            wordleGameOver = true;
            wordleGamesPlayed++;
        }
    }
}

int getWordleLetterState(int row, int col) {
    if (row >= wordleCurrentRow && !wordleGameOver) return 0;  // Not revealed yet
    
    char letter = wordleGuesses[row][col];
    if (letter == '\0') return 0;
    
    // Green - correct position
    if (letter == wordleTarget[col]) return 3;
    
    // Yellow - in word but wrong position
    for (int i = 0; i < 5; i++) {
        if (wordleTarget[i] == letter) return 2;
    }
    
    // Gray - not in word
    return 1;
}

// Get keyboard key color based on letter's best state across all guesses
uint32_t getWordleKeyColor(char letter) {
    int bestState = 0;  // 0=unused, 1=gray, 2=yellow, 3=green
    
    for (int row = 0; row < wordleCurrentRow; row++) {
        for (int col = 0; col < 5; col++) {
            if (wordleGuesses[row][col] == letter) {
                int state = getWordleLetterState(row, col);
                if (state > bestState) bestState = state;
            }
        }
    }
    
    switch (bestState) {
        case 3: return 0x538D4E;  // Green - correct
        case 2: return 0xB59F3B;  // Yellow - wrong position  
        case 1: return 0x3A3A3C;  // Gray - not in word
        default: return 0x818384; // Unused - default gray
    }
}

void createWordleCard() {
    disableAllScrolling(lv_scr_act());
    
    if (!wordleWordsLoaded) {
        loadWordleWords();
        resetWordle();
    }

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x121213), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "WORDLE");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // Stats
    char statsBuf[24];
    snprintf(statsBuf, sizeof(statsBuf), "%d/%d", wordleGamesWon, wordleGamesPlayed);
    lv_obj_t *statsLabel = lv_label_create(card);
    lv_label_set_text(statsLabel, statsBuf);
    lv_obj_set_style_text_color(statsLabel, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(statsLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(statsLabel, LV_ALIGN_TOP_RIGHT, -10, 12);

    // Letter grid (6 rows x 5 cols)
    int cellSize = 42;
    int cellGap = 4;
    int gridStartX = (LCD_WIDTH - (cellSize * 5 + cellGap * 4)) / 2;
    int gridStartY = 35;

    uint32_t stateColors[] = {0x3A3A3C, 0x3A3A3C, 0xB59F3B, 0x538D4E};  // empty, gray, yellow, green

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 5; col++) {
            int posX = gridStartX + col * (cellSize + cellGap);
            int posY = gridStartY + row * (cellSize + cellGap);
            int state = getWordleLetterState(row, col);

            lv_obj_t *cell = lv_obj_create(card);
            lv_obj_set_size(cell, cellSize, cellSize);
            lv_obj_set_pos(cell, posX, posY);
            lv_obj_set_style_bg_color(cell, lv_color_hex(stateColors[state]), 0);
            lv_obj_set_style_radius(cell, 4, 0);
            lv_obj_set_style_border_width(cell, state == 0 ? 2 : 0, 0);
            lv_obj_set_style_border_color(cell, lv_color_hex(0x3A3A3C), 0);
            disableAllScrolling(cell);

            char letter = wordleGuesses[row][col];
            if (letter != '\0') {
                lv_obj_t *letterLabel = lv_label_create(cell);
                char buf[2] = {letter, '\0'};
                lv_label_set_text(letterLabel, buf);
                lv_obj_set_style_text_color(letterLabel, lv_color_hex(0xFFFFFF), 0);
                lv_obj_set_style_text_font(letterLabel, &lv_font_montserrat_24, 0);
                lv_obj_center(letterLabel);
            }
        }
    }

    // Keyboard (3 rows) - LARGER KEYS, BETTER SPACING
    int keyW = 36, keyH = 48, keyGap = 5;  // Increased from 32x40 to 36x48
    int row1Y = 295, row2Y = 348, row3Y = 401;  // Adjusted positions
    const char* row1 = "QWERTYUIOP";
    const char* row2 = "ASDFGHJKL";
    const char* row3 = "ZXCVBNM";
    
    // Row 1 - Full width centered with larger keys
    int row1X = (LCD_WIDTH - (10 * keyW + 9 * keyGap)) / 2;
    for (int i = 0; i < 10; i++) {
        lv_obj_t *key = lv_obj_create(card);
        lv_obj_set_size(key, keyW, keyH);
        lv_obj_set_pos(key, row1X + i * (keyW + keyGap), row1Y);
        lv_obj_set_style_bg_color(key, lv_color_hex(getWordleKeyColor(row1[i])), 0);
        lv_obj_set_style_radius(key, 8, 0);  // More rounded
        lv_obj_set_style_border_width(key, 0, 0);
        lv_obj_set_style_shadow_width(key, 4, 0);
        lv_obj_set_style_shadow_color(key, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(key, LV_OPA_50, 0);
        disableAllScrolling(key);
        
        lv_obj_t *keyLabel = lv_label_create(key);
        char buf[2] = {row1[i], '\0'};
        lv_label_set_text(keyLabel, buf);
        lv_obj_set_style_text_color(keyLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(keyLabel, &lv_font_montserrat_20, 0);  // Larger font
        lv_obj_center(keyLabel);
    }
    
    // Row 2 - Centered with larger keys
    int row2X = (LCD_WIDTH - (9 * keyW + 8 * keyGap)) / 2;
    for (int i = 0; i < 9; i++) {
        lv_obj_t *key = lv_obj_create(card);
        lv_obj_set_size(key, keyW, keyH);
        lv_obj_set_pos(key, row2X + i * (keyW + keyGap), row2Y);
        lv_obj_set_style_bg_color(key, lv_color_hex(getWordleKeyColor(row2[i])), 0);
        lv_obj_set_style_radius(key, 8, 0);
        lv_obj_set_style_border_width(key, 0, 0);
        lv_obj_set_style_shadow_width(key, 4, 0);
        lv_obj_set_style_shadow_color(key, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(key, LV_OPA_50, 0);
        disableAllScrolling(key);
        
        lv_obj_t *keyLabel = lv_label_create(key);
        char buf[2] = {row2[i], '\0'};
        lv_label_set_text(keyLabel, buf);
        lv_obj_set_style_text_color(keyLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(keyLabel, &lv_font_montserrat_20, 0);
        lv_obj_center(keyLabel);
    }
    
    // Row 3 with ENTER and DEL - Larger action keys
    // ENTER key - wider and more prominent
    lv_obj_t *enterKey = lv_obj_create(card);
    lv_obj_set_size(enterKey, 56, keyH);  // Wider
    lv_obj_set_pos(enterKey, 6, row3Y);
    lv_obj_set_style_bg_color(enterKey, lv_color_hex(0x538D4E), 0);
    lv_obj_set_style_radius(enterKey, 8, 0);
    lv_obj_set_style_border_width(enterKey, 0, 0);
    lv_obj_set_style_shadow_width(enterKey, 4, 0);
    lv_obj_set_style_shadow_color(enterKey, lv_color_hex(0x3A6B35), 0);
    lv_obj_set_style_shadow_opa(enterKey, LV_OPA_70, 0);
    disableAllScrolling(enterKey);
    lv_obj_t *enterLabel = lv_label_create(enterKey);
    lv_label_set_text(enterLabel, LV_SYMBOL_OK);
    lv_obj_set_style_text_color(enterLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(enterLabel, &lv_font_montserrat_20, 0);
    lv_obj_center(enterLabel);
    
    // Letter keys - Row 3 with adjusted spacing
    int row3X = 66;
    for (int i = 0; i < 7; i++) {
        lv_obj_t *key = lv_obj_create(card);
        lv_obj_set_size(key, keyW, keyH);
        lv_obj_set_pos(key, row3X + i * (keyW + keyGap), row3Y);
        lv_obj_set_style_bg_color(key, lv_color_hex(getWordleKeyColor(row3[i])), 0);
        lv_obj_set_style_radius(key, 8, 0);
        lv_obj_set_style_border_width(key, 0, 0);
        lv_obj_set_style_shadow_width(key, 4, 0);
        lv_obj_set_style_shadow_color(key, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(key, LV_OPA_50, 0);
        disableAllScrolling(key);
        
        lv_obj_t *keyLabel = lv_label_create(key);
        char buf[2] = {row3[i], '\0'};
        lv_label_set_text(keyLabel, buf);
        lv_obj_set_style_text_color(keyLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(keyLabel, &lv_font_montserrat_20, 0);
        lv_obj_center(keyLabel);
    }
    
    // DEL key - wider and more visible
    lv_obj_t *delKey = lv_obj_create(card);
    lv_obj_set_size(delKey, 56, keyH);  // Wider
    lv_obj_set_pos(delKey, LCD_WIDTH - 62, row3Y);
    lv_obj_set_style_bg_color(delKey, lv_color_hex(0x666666), 0);
    lv_obj_set_style_radius(delKey, 8, 0);
    lv_obj_set_style_border_width(delKey, 0, 0);
    lv_obj_set_style_shadow_width(delKey, 4, 0);
    lv_obj_set_style_shadow_color(delKey, lv_color_hex(0x333333), 0);
    lv_obj_set_style_shadow_opa(delKey, LV_OPA_70, 0);
    disableAllScrolling(delKey);
    lv_obj_t *delLabel = lv_label_create(delKey);
    lv_label_set_text(delLabel, LV_SYMBOL_BACKSPACE);
    lv_obj_set_style_text_color(delLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(delLabel, &lv_font_montserrat_20, 0);
    lv_obj_center(delLabel);

    // Status message
    lv_obj_t *statusLabel = lv_label_create(card);
    if (wordleGameOver) {
        if (wordleWon) {
            lv_label_set_text(statusLabel, "NICE! TAP TO PLAY AGAIN");
            lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x538D4E), 0);
        } else {
            char loseBuf[32];
            snprintf(loseBuf, sizeof(loseBuf), "WORD: %s - TAP AGAIN", wordleTarget);
            lv_label_set_text(statusLabel, loseBuf);
            lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xB59F3B), 0);
        }
    } else {
        lv_label_set_text(statusLabel, LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x555555), 0);
    }
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(statusLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// BREAKOUT GAME - Touch to move paddle, break bricks - 5 LEVELS!
// ═══════════════════════════════════════════════════════════════════════════

// Level configurations: rows, cols, ball speed multiplier, brick pattern
struct BreakoutLevel {
    int rows;
    int cols; 
    float speedMultiplier;
    int pattern;  // 0=full, 1=checkerboard, 2=pyramid, 3=inverted, 4=random holes
};

const BreakoutLevel breakoutLevels[BREAKOUT_MAX_LEVELS] = {
    {4, 8, 1.0f, 0},    // Level 1: 4 rows, normal speed, full bricks
    {5, 9, 1.2f, 1},    // Level 2: 5 rows, 20% faster, checkerboard
    {5, 10, 1.4f, 2},   // Level 3: 5 rows, 40% faster, pyramid
    {6, 10, 1.6f, 3},   // Level 4: 6 rows, 60% faster, inverted pyramid
    {6, 10, 2.0f, 4}    // Level 5: 6 rows, 2x speed, random holes
};

void initBreakoutLevel(int level) {
    if (level < 1) level = 1;
    if (level > BREAKOUT_MAX_LEVELS) level = BREAKOUT_MAX_LEVELS;
    
    breakoutCurrentLevel = level;
    const BreakoutLevel& lvl = breakoutLevels[level - 1];
    
    // Reset ball position
    breakoutPaddleX = LCD_WIDTH / 2 - BREAKOUT_PADDLE_WIDTH / 2;
    breakoutBallX = LCD_WIDTH / 2;
    breakoutBallY = 380;
    
    // Set ball speed based on level
    float speed = BREAKOUT_BASE_SPEED * lvl.speedMultiplier;
    breakoutBallVelX = speed * 0.5f;
    breakoutBallVelY = -speed * 0.7f;
    
    breakoutLevelComplete = false;
    breakoutShowNextLevel = false;
    breakoutGameOver = false;
    
    // Initialize bricks based on pattern
    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 10; c++) {
            breakoutBricks[r][c] = false;
            
            if (r < lvl.rows && c < lvl.cols) {
                switch (lvl.pattern) {
                    case 0: // Full
                        breakoutBricks[r][c] = true;
                        break;
                    case 1: // Checkerboard
                        breakoutBricks[r][c] = ((r + c) % 2 == 0);
                        break;
                    case 2: // Pyramid
                        if (c >= (lvl.cols/2 - r - 1) && c <= (lvl.cols/2 + r)) {
                            breakoutBricks[r][c] = true;
                        }
                        break;
                    case 3: // Inverted pyramid
                        if (c >= r && c < (lvl.cols - r)) {
                            breakoutBricks[r][c] = true;
                        }
                        break;
                    case 4: // Random holes (70% filled)
                        breakoutBricks[r][c] = (random(100) < 70);
                        break;
                }
            }
        }
    }
    
    // Save progress
    breakoutSavedLevel = breakoutCurrentLevel;
    breakoutSavedScore = breakoutScore;
}

void resetBreakout() {
    // If we have saved progress, restore it
    if (breakoutSavedLevel > 1 && breakoutSavedScore > 0) {
        breakoutScore = breakoutSavedScore;
        initBreakoutLevel(breakoutSavedLevel);
    } else {
        breakoutScore = 0;
        breakoutLives = 3;
        initBreakoutLevel(1);
    }
}

void resetBreakoutFull() {
    // Full reset - start from level 1
    breakoutScore = 0;
    breakoutLives = 3;
    breakoutSavedLevel = 1;
    breakoutSavedScore = 0;
    initBreakoutLevel(1);
}

void updateBreakout() {
    if (!breakoutGameActive || breakoutGameOver || breakoutShowNextLevel) return;
    
    // TOUCH control for paddle (no gyro)
    if (touchActive) {
        // Move paddle to follow touch X position
        breakoutPaddleX = touchCurrentX - BREAKOUT_PADDLE_WIDTH / 2;
    }
    
    // Clamp paddle position
    if (breakoutPaddleX < 5) breakoutPaddleX = 5;
    if (breakoutPaddleX > LCD_WIDTH - BREAKOUT_PADDLE_WIDTH - 5) 
        breakoutPaddleX = LCD_WIDTH - BREAKOUT_PADDLE_WIDTH - 5;
    
    // Move ball (DOUBLED speed)
    breakoutBallX += breakoutBallVelX;
    breakoutBallY += breakoutBallVelY;
    
    // Wall collisions
    if (breakoutBallX <= BREAKOUT_BALL_SIZE/2) {
        breakoutBallX = BREAKOUT_BALL_SIZE/2;
        breakoutBallVelX = -breakoutBallVelX;
    }
    if (breakoutBallX >= LCD_WIDTH - BREAKOUT_BALL_SIZE/2) {
        breakoutBallX = LCD_WIDTH - BREAKOUT_BALL_SIZE/2;
        breakoutBallVelX = -breakoutBallVelX;
    }
    if (breakoutBallY <= 55 + BREAKOUT_BALL_SIZE/2) {
        breakoutBallY = 55 + BREAKOUT_BALL_SIZE/2;
        breakoutBallVelY = -breakoutBallVelY;
    }
    
    // Paddle collision
    int paddleTop = 420;
    if (breakoutBallY + BREAKOUT_BALL_SIZE/2 >= paddleTop &&
        breakoutBallY - BREAKOUT_BALL_SIZE/2 <= paddleTop + BREAKOUT_PADDLE_HEIGHT &&
        breakoutBallX >= breakoutPaddleX &&
        breakoutBallX <= breakoutPaddleX + BREAKOUT_PADDLE_WIDTH) {
        breakoutBallY = paddleTop - BREAKOUT_BALL_SIZE/2;
        breakoutBallVelY = -abs(breakoutBallVelY);
        // Angle based on hit position (increased for faster gameplay)
        float hitPos = (breakoutBallX - breakoutPaddleX) / BREAKOUT_PADDLE_WIDTH;
        breakoutBallVelX = (hitPos - 0.5f) * 10.0f;  // Increased angle effect
    }
    
    // Bottom - lose life
    if (breakoutBallY > LCD_HEIGHT - 20) {
        breakoutLives--;
        if (breakoutLives <= 0) {
            breakoutGameOver = true;
            // Reset saved progress on game over
            breakoutSavedLevel = 1;
            breakoutSavedScore = 0;
        } else {
            // Reset ball position but keep speed based on level
            breakoutBallX = LCD_WIDTH / 2;
            breakoutBallY = 380;
            float speed = BREAKOUT_BASE_SPEED * breakoutLevels[breakoutCurrentLevel - 1].speedMultiplier;
            breakoutBallVelY = -speed * 0.7f;
        }
    }
    
    // Brick collisions
    const BreakoutLevel& lvl = breakoutLevels[breakoutCurrentLevel - 1];
    int brickStartY = 65;
    int brickGap = 3;
    int brickAreaWidth = lvl.cols * (BREAKOUT_BRICK_WIDTH + brickGap) - brickGap;
    int brickStartX = (LCD_WIDTH - brickAreaWidth) / 2;
    
    for (int r = 0; r < lvl.rows; r++) {
        for (int c = 0; c < lvl.cols; c++) {
            if (!breakoutBricks[r][c]) continue;
            
            int bx = brickStartX + c * (BREAKOUT_BRICK_WIDTH + brickGap);
            int by = brickStartY + r * (BREAKOUT_BRICK_HEIGHT + brickGap);
            
            if (breakoutBallX + BREAKOUT_BALL_SIZE/2 >= bx &&
                breakoutBallX - BREAKOUT_BALL_SIZE/2 <= bx + BREAKOUT_BRICK_WIDTH &&
                breakoutBallY + BREAKOUT_BALL_SIZE/2 >= by &&
                breakoutBallY - BREAKOUT_BALL_SIZE/2 <= by + BREAKOUT_BRICK_HEIGHT) {
                breakoutBricks[r][c] = false;
                breakoutScore += (lvl.rows - r) * 10 * breakoutCurrentLevel;  // More points for higher levels
                breakoutBallVelY = -breakoutBallVelY;
                break;
            }
        }
    }
    
    // Check level complete
    bool allClear = true;
    for (int r = 0; r < lvl.rows; r++) {
        for (int c = 0; c < lvl.cols; c++) {
            if (breakoutBricks[r][c]) allClear = false;
        }
    }
    
    if (allClear) {
        if (breakoutCurrentLevel < BREAKOUT_MAX_LEVELS) {
            // Show "Next Level?" prompt
            breakoutShowNextLevel = true;
            breakoutLevelComplete = true;
            // Save progress
            breakoutSavedLevel = breakoutCurrentLevel + 1;
            breakoutSavedScore = breakoutScore;
        } else {
            // Beat all 5 levels - game complete!
            breakoutGameOver = true;
            breakoutLevelComplete = true;
        }
    }
}

void advanceBreakoutLevel() {
    if (breakoutCurrentLevel < BREAKOUT_MAX_LEVELS) {
        breakoutCurrentLevel++;
        initBreakoutLevel(breakoutCurrentLevel);
    }
}

void createBreakoutCard() {
    disableAllScrolling(lv_scr_act());
    
    updateBreakout();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with level and score
    lv_obj_t *title = lv_label_create(card);
    char titleBuf[48];
    snprintf(titleBuf, sizeof(titleBuf), "LEVEL %d  SCORE: %d", breakoutCurrentLevel, breakoutScore);
    lv_label_set_text(title, titleBuf);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF5722), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // Lives
    char livesBuf[16];
    snprintf(livesBuf, sizeof(livesBuf), "%s %d", LV_SYMBOL_CHARGE, breakoutLives);
    lv_obj_t *livesLabel = lv_label_create(card);
    lv_label_set_text(livesLabel, livesBuf);
    lv_obj_set_style_text_color(livesLabel, lv_color_hex(0xFF5722), 0);
    lv_obj_set_style_text_font(livesLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(livesLabel, LV_ALIGN_TOP_RIGHT, -10, 10);

    // Level indicator dots
    for (int i = 0; i < BREAKOUT_MAX_LEVELS; i++) {
        lv_obj_t *dot = lv_obj_create(card);
        lv_obj_set_size(dot, 8, 8);
        lv_obj_set_pos(dot, 10 + i * 14, 12);
        lv_obj_set_style_radius(dot, 4, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        if (i < breakoutCurrentLevel) {
            lv_obj_set_style_bg_color(dot, lv_color_hex(0x30D158), 0);  // Completed - green
        } else if (i == breakoutCurrentLevel - 1) {
            lv_obj_set_style_bg_color(dot, lv_color_hex(0xFF9F0A), 0);  // Current - orange
        } else {
            lv_obj_set_style_bg_color(dot, lv_color_hex(0x3A3A3C), 0);  // Future - gray
        }
        disableAllScrolling(dot);
    }

    // Show "Next Level?" overlay if level complete
    if (breakoutShowNextLevel) {
        // Dark overlay
        lv_obj_t *overlay = lv_obj_create(card);
        lv_obj_set_size(overlay, LCD_WIDTH, LCD_HEIGHT);
        lv_obj_set_pos(overlay, 0, 0);
        lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(overlay, LV_OPA_80, 0);
        lv_obj_set_style_border_width(overlay, 0, 0);
        disableAllScrolling(overlay);
        
        // Level complete message
        lv_obj_t *completeLabel = lv_label_create(card);
        char completeBuf[48];
        snprintf(completeBuf, sizeof(completeBuf), "LEVEL %d COMPLETE!", breakoutCurrentLevel);
        lv_label_set_text(completeLabel, completeBuf);
        lv_obj_set_style_text_color(completeLabel, lv_color_hex(0x30D158), 0);
        lv_obj_set_style_text_font(completeLabel, &lv_font_montserrat_20, 0);
        lv_obj_align(completeLabel, LV_ALIGN_CENTER, 0, -40);
        
        // Score display
        lv_obj_t *scoreLabel = lv_label_create(card);
        char scoreBuf[32];
        snprintf(scoreBuf, sizeof(scoreBuf), "SCORE: %d", breakoutScore);
        lv_label_set_text(scoreLabel, scoreBuf);
        lv_obj_set_style_text_color(scoreLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(scoreLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(scoreLabel, LV_ALIGN_CENTER, 0, 0);
        
        // Next level prompt
        lv_obj_t *nextLabel = lv_label_create(card);
        char nextBuf[48];
        snprintf(nextBuf, sizeof(nextBuf), "TAP FOR LEVEL %d", breakoutCurrentLevel + 1);
        lv_label_set_text(nextLabel, nextBuf);
        lv_obj_set_style_text_color(nextLabel, lv_color_hex(0xFF9F0A), 0);
        lv_obj_set_style_text_font(nextLabel, &lv_font_montserrat_16, 0);
        lv_obj_align(nextLabel, LV_ALIGN_CENTER, 0, 50);
        
        return;  // Don't draw game elements when showing next level prompt
    }

    // Draw bricks based on current level
    const BreakoutLevel& lvl = breakoutLevels[breakoutCurrentLevel - 1];
    uint32_t brickColors[] = {0xFF0000, 0xFF6600, 0xFFCC00, 0x30D158, 0x0A84FF, 0x5E5CE6};
    int brickStartY = 65;
    int brickGap = 3;
    int brickAreaWidth = lvl.cols * (BREAKOUT_BRICK_WIDTH + brickGap) - brickGap;
    int brickStartX = (LCD_WIDTH - brickAreaWidth) / 2;
    
    for (int r = 0; r < lvl.rows; r++) {
        for (int c = 0; c < lvl.cols; c++) {
            if (!breakoutBricks[r][c]) continue;
            
            int bx = brickStartX + c * (BREAKOUT_BRICK_WIDTH + brickGap);
            int by = brickStartY + r * (BREAKOUT_BRICK_HEIGHT + brickGap);
            
            lv_obj_t *brick = lv_obj_create(card);
            lv_obj_set_size(brick, BREAKOUT_BRICK_WIDTH, BREAKOUT_BRICK_HEIGHT);
            lv_obj_set_pos(brick, bx, by);
            lv_obj_set_style_bg_color(brick, lv_color_hex(brickColors[r % 6]), 0);
            lv_obj_set_style_radius(brick, 2, 0);
            lv_obj_set_style_border_width(brick, 0, 0);
            disableAllScrolling(brick);
        }
    }

    // Paddle
    lv_obj_t *paddle = lv_obj_create(card);
    lv_obj_set_size(paddle, BREAKOUT_PADDLE_WIDTH, BREAKOUT_PADDLE_HEIGHT);
    lv_obj_set_pos(paddle, (int)breakoutPaddleX, 420);
    lv_obj_set_style_bg_color(paddle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(paddle, 3, 0);
    lv_obj_set_style_border_width(paddle, 0, 0);
    disableAllScrolling(paddle);

    // Ball
    lv_obj_t *ball = lv_obj_create(card);
    lv_obj_set_size(ball, BREAKOUT_BALL_SIZE, BREAKOUT_BALL_SIZE);
    lv_obj_set_pos(ball, (int)breakoutBallX - BREAKOUT_BALL_SIZE/2, (int)breakoutBallY - BREAKOUT_BALL_SIZE/2);
    lv_obj_set_style_bg_color(ball, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(ball, BREAKOUT_BALL_SIZE/2, 0);
    lv_obj_set_style_border_width(ball, 0, 0);
    disableAllScrolling(ball);

    // Status
    lv_obj_t *hintLabel = lv_label_create(card);
    if (breakoutGameOver) {
        if (breakoutLives <= 0) {
            lv_label_set_text(hintLabel, "GAME OVER - TAP TO RESTART");
        } else if (breakoutLevelComplete && breakoutCurrentLevel >= BREAKOUT_MAX_LEVELS) {
            lv_label_set_text(hintLabel, "ALL LEVELS COMPLETE! TAP TO RESTART");
            lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x30D158), 0);
        } else {
            lv_label_set_text(hintLabel, "YOU WIN! TAP TO RESTART");
        }
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0xFF5722), 0);
    } else if (!breakoutGameActive) {
        if (breakoutSavedLevel > 1) {
            char continueBuf[48];
            snprintf(continueBuf, sizeof(continueBuf), "TAP TO CONTINUE LV%d  " LV_SYMBOL_UP " BACK", breakoutSavedLevel);
            lv_label_set_text(hintLabel, continueBuf);
        } else {
            lv_label_set_text(hintLabel, "TAP TO START  " LV_SYMBOL_UP " BACK");
        }
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x666666), 0);
    } else {
        lv_label_set_text(hintLabel, "TOUCH TO MOVE  " LV_SYMBOL_UP " BACK");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x666666), 0);
    }
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// TETRIS GAME - Tilt to move, tap to rotate
// ═══════════════════════════════════════════════════════════════════════════
// Tetris piece definitions (I, O, T, S, Z, J, L)
const int tetrisPieces[7][4][4] = {
    {{0,0,0,0}, {1,1,1,1}, {0,0,0,0}, {0,0,0,0}},  // I
    {{0,0,0,0}, {0,1,1,0}, {0,1,1,0}, {0,0,0,0}},  // O
    {{0,0,0,0}, {0,1,0,0}, {1,1,1,0}, {0,0,0,0}},  // T
    {{0,0,0,0}, {0,1,1,0}, {1,1,0,0}, {0,0,0,0}},  // S
    {{0,0,0,0}, {1,1,0,0}, {0,1,1,0}, {0,0,0,0}},  // Z
    {{0,0,0,0}, {1,0,0,0}, {1,1,1,0}, {0,0,0,0}},  // J
    {{0,0,0,0}, {0,0,1,0}, {1,1,1,0}, {0,0,0,0}}   // L
};

uint32_t tetrisColors[] = {0x00F0F0, 0xF0F000, 0xA000F0, 0x00F000, 0xF00000, 0x0000F0, 0xF0A000};

void tetrisSpawnPiece() {
    tetrisCurrentType = random(7);
    tetrisCurrentX = 3;
    tetrisCurrentY = 0;
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tetrisCurrentPiece[i][j] = tetrisPieces[tetrisCurrentType][i][j];
        }
    }
    
    // Check game over
    if (!tetrisCanMove(0, 0)) {
        tetrisGameOver = true;
    }
}

bool tetrisCanMove(int dx, int dy) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (tetrisCurrentPiece[i][j]) {
                int newX = tetrisCurrentX + j + dx;
                int newY = tetrisCurrentY + i + dy;
                
                if (newX < 0 || newX >= 10 || newY >= 20) return false;
                if (newY >= 0 && tetrisBoard[newY][newX]) return false;
            }
        }
    }
    return true;
}

bool tetrisCanRotate() {
    int rotated[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            rotated[j][3-i] = tetrisCurrentPiece[i][j];
        }
    }
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (rotated[i][j]) {
                int newX = tetrisCurrentX + j;
                int newY = tetrisCurrentY + i;
                
                if (newX < 0 || newX >= 10 || newY >= 20) return false;
                if (newY >= 0 && tetrisBoard[newY][newX]) return false;
            }
        }
    }
    return true;
}

void tetrisRotatePiece() {
    if (!tetrisCanRotate()) return;
    
    int rotated[4][4] = {{0}};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            rotated[j][3-i] = tetrisCurrentPiece[i][j];
        }
    }
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tetrisCurrentPiece[i][j] = rotated[i][j];
        }
    }
}

void tetrisLockPiece() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (tetrisCurrentPiece[i][j]) {
                int bx = tetrisCurrentX + j;
                int by = tetrisCurrentY + i;
                if (by >= 0 && by < 20 && bx >= 0 && bx < 10) {
                    tetrisBoard[by][bx] = tetrisCurrentType + 1;
                }
            }
        }
    }
}

void tetrisClearLines() {
    int linesCleared = 0;
    
    for (int y = 19; y >= 0; y--) {
        bool full = true;
        for (int x = 0; x < 10; x++) {
            if (!tetrisBoard[y][x]) { full = false; break; }
        }
        
        if (full) {
            linesCleared++;
            // Move all rows above down
            for (int yy = y; yy > 0; yy--) {
                for (int x = 0; x < 10; x++) {
                    tetrisBoard[yy][x] = tetrisBoard[yy-1][x];
                }
            }
            // Clear top row
            for (int x = 0; x < 10; x++) {
                tetrisBoard[0][x] = 0;
            }
            y++;  // Check same row again
        }
    }
    
    // Score based on lines cleared
    int points[] = {0, 100, 300, 500, 800};
    tetrisScore += points[linesCleared] * tetrisLevel;
    tetrisLines += linesCleared;
    
    // Level up every 10 lines
    tetrisLevel = 1 + tetrisLines / 10;
    tetrisDropSpeed = max(100, 500 - (tetrisLevel - 1) * 50);
}

void resetTetris() {
    for (int y = 0; y < 20; y++) {
        for (int x = 0; x < 10; x++) {
            tetrisBoard[y][x] = 0;
        }
    }
    tetrisScore = 0;
    tetrisLevel = 1;
    tetrisLines = 0;
    tetrisDropSpeed = 500;
    tetrisGameOver = false;
    tetrisSpawnPiece();
    tetrisLastDrop = millis();
}

void updateTetris() {
    if (!tetrisGameActive || tetrisGameOver) return;
    
    // Swipe control is handled in handleSwipe() - one swipe = one block movement
    // No tilt control needed
    
    // Auto drop
    if (millis() - tetrisLastDrop > (unsigned long)tetrisDropSpeed) {
        if (tetrisCanMove(0, 1)) {
            tetrisCurrentY++;
        } else {
            tetrisLockPiece();
            tetrisClearLines();
            tetrisSpawnPiece();
        }
        tetrisLastDrop = millis();
    }
}

void createTetrisCard() {
    disableAllScrolling(lv_scr_act());
    
    updateTetris();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title and score
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "TETRIS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x9C27B0), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 8);

    char scoreBuf[32];
    snprintf(scoreBuf, sizeof(scoreBuf), "%d", tetrisScore);
    lv_obj_t *scoreLabel = lv_label_create(card);
    lv_label_set_text(scoreLabel, scoreBuf);
    lv_obj_set_style_text_color(scoreLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(scoreLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(scoreLabel, LV_ALIGN_TOP_RIGHT, -10, 10);

    char levelBuf[16];
    snprintf(levelBuf, sizeof(levelBuf), "LV%d", tetrisLevel);
    lv_obj_t *levelLabel = lv_label_create(card);
    lv_label_set_text(levelLabel, levelBuf);
    lv_obj_set_style_text_color(levelLabel, lv_color_hex(0x9C27B0), 0);
    lv_obj_set_style_text_font(levelLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(levelLabel, LV_ALIGN_TOP_RIGHT, -10, 25);

    // Game board
    int cellSize = 20;
    int boardX = (LCD_WIDTH - 10 * cellSize) / 2;
    int boardY = 45;

    // Board background
    lv_obj_t *boardBg = lv_obj_create(card);
    lv_obj_set_size(boardBg, 10 * cellSize + 4, 20 * cellSize + 4);
    lv_obj_set_pos(boardBg, boardX - 2, boardY - 2);
    lv_obj_set_style_bg_color(boardBg, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_radius(boardBg, 4, 0);
    lv_obj_set_style_border_width(boardBg, 1, 0);
    lv_obj_set_style_border_color(boardBg, lv_color_hex(0x333333), 0);
    disableAllScrolling(boardBg);

    // Draw locked pieces
    for (int y = 0; y < 20; y++) {
        for (int x = 0; x < 10; x++) {
            if (tetrisBoard[y][x]) {
                lv_obj_t *cell = lv_obj_create(card);
                lv_obj_set_size(cell, cellSize - 1, cellSize - 1);
                lv_obj_set_pos(cell, boardX + x * cellSize, boardY + y * cellSize);
                lv_obj_set_style_bg_color(cell, lv_color_hex(tetrisColors[tetrisBoard[y][x] - 1]), 0);
                lv_obj_set_style_radius(cell, 2, 0);
                lv_obj_set_style_border_width(cell, 0, 0);
                disableAllScrolling(cell);
            }
        }
    }

    // Draw current piece
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (tetrisCurrentPiece[i][j]) {
                int px = tetrisCurrentX + j;
                int py = tetrisCurrentY + i;
                if (py >= 0) {
                    lv_obj_t *cell = lv_obj_create(card);
                    lv_obj_set_size(cell, cellSize - 1, cellSize - 1);
                    lv_obj_set_pos(cell, boardX + px * cellSize, boardY + py * cellSize);
                    lv_obj_set_style_bg_color(cell, lv_color_hex(tetrisColors[tetrisCurrentType]), 0);
                    lv_obj_set_style_radius(cell, 2, 0);
                    lv_obj_set_style_border_width(cell, 0, 0);
                    disableAllScrolling(cell);
                }
            }
        }
    }

    // Status
    lv_obj_t *hintLabel = lv_label_create(card);
    if (tetrisGameOver) {
        lv_label_set_text(hintLabel, "GAME OVER - TAP TO RESTART");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x9C27B0), 0);
    } else if (!tetrisGameActive) {
        lv_label_set_text(hintLabel, "TAP TO START");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x666666), 0);
    } else {
        lv_label_set_text(hintLabel, "TILT:MOVE TAP:ROTATE");
        lv_obj_set_style_text_color(hintLabel, lv_color_hex(0x666666), 0);
    }
    lv_obj_set_style_text_font(hintLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(hintLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
}
void createRunningCard() {
    disableAllScrolling(lv_scr_act());
    GradientTheme *theme = getSafeTheme();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_SHUFFLE " RUNNING");
    lv_obj_set_style_text_color(title, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Timer display
    unsigned long elapsed = 0;
    if (runningModeActive) {
        elapsed = millis() - runningStartTime;
    }
    int mins = (elapsed / 60000) % 60;
    int secs = (elapsed / 1000) % 60;
    int ms = (elapsed % 1000) / 10;

    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d.%02d", mins, secs, ms);
    lv_obj_t *timeLabel = lv_label_create(card);
    lv_label_set_text(timeLabel, timeBuf);
    lv_obj_set_style_text_color(timeLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(timeLabel, LV_ALIGN_TOP_MID, 0, 40);

    // Stats row
    int currentSteps = userData.steps;
    int runSteps = runningModeActive ? (currentSteps - runningStartSteps) : 0;
    
    // Calculate pace (steps per minute)
    if (runningModeActive && elapsed > 10000) {
        runningPace = (float)runSteps / (elapsed / 60000.0);
    }
    
    // Distance (assuming 0.7m per step)
    runningDistance = runSteps * 0.0007;  // km

    // Steps during run
    char stepsBuf[24];
    snprintf(stepsBuf, sizeof(stepsBuf), "%d", runSteps);
    lv_obj_t *stepsVal = lv_label_create(card);
    lv_label_set_text(stepsVal, stepsBuf);
    lv_obj_set_style_text_color(stepsVal, theme->accent, 0);
    lv_obj_set_style_text_font(stepsVal, &lv_font_montserrat_24, 0);
    lv_obj_align(stepsVal, LV_ALIGN_CENTER, -60, 20);

    lv_obj_t *stepsLbl = lv_label_create(card);
    lv_label_set_text(stepsLbl, "STEPS");
    lv_obj_set_style_text_color(stepsLbl, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(stepsLbl, &lv_font_montserrat_12, 0);
    lv_obj_align(stepsLbl, LV_ALIGN_CENTER, -60, 45);

    // Pace
    char paceBuf[16];
    snprintf(paceBuf, sizeof(paceBuf), "%.0f", runningPace);
    lv_obj_t *paceVal = lv_label_create(card);
    lv_label_set_text(paceVal, paceBuf);
    lv_obj_set_style_text_color(paceVal, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(paceVal, &lv_font_montserrat_24, 0);
    lv_obj_align(paceVal, LV_ALIGN_CENTER, 60, 20);

    lv_obj_t *paceLbl = lv_label_create(card);
    lv_label_set_text(paceLbl, "PACE/MIN");
    lv_obj_set_style_text_color(paceLbl, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(paceLbl, &lv_font_montserrat_12, 0);
    lv_obj_align(paceLbl, LV_ALIGN_CENTER, 60, 45);

    // Distance
    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.2f km", runningDistance);
    lv_obj_t *distLabel = lv_label_create(card);
    lv_label_set_text(distLabel, distBuf);
    lv_obj_set_style_text_color(distLabel, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(distLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(distLabel, LV_ALIGN_CENTER, 0, 80);

    // Start/Stop button
    lv_obj_t *actionBtn = lv_obj_create(card);
    lv_obj_set_size(actionBtn, LCD_WIDTH - 60, 50);
    lv_obj_align(actionBtn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(actionBtn, runningModeActive ? lv_color_hex(0xFF453A) : lv_color_hex(0x30D158), 0);
    lv_obj_set_style_radius(actionBtn, 25, 0);
    lv_obj_set_style_border_width(actionBtn, 0, 0);
    disableAllScrolling(actionBtn);

    lv_obj_t *actionLabel = lv_label_create(actionBtn);
    lv_label_set_text(actionLabel, runningModeActive ? "STOP RUN" : "START RUN");
    lv_obj_set_style_text_color(actionLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(actionLabel, &lv_font_montserrat_16, 0);
    lv_obj_center(actionLabel);
}

// ═══════════════════════════════════════════════════════════════════════════
// SPORT + MOVEMENT TRACKER CARD (Activity subcard 3)
// Under Steps card - Sprint detection, session timer, activity streak
// ═══════════════════════════════════════════════════════════════════════════
void createSportTrackerCard() {
    disableAllScrolling(lv_scr_act());
    GradientTheme *theme = getSafeTheme();

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with gradient effect
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_SHUFFLE " SPORT TRACKER");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF6B35), 0);  // Orange
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // ═══ ACTIVITY STREAK BADGE (Top right) ═══
    lv_obj_t *streakBadge = lv_obj_create(card);
    lv_obj_set_size(streakBadge, 65, 28);
    lv_obj_align(streakBadge, LV_ALIGN_TOP_RIGHT, -10, 5);
    lv_obj_set_style_bg_color(streakBadge, lv_color_hex(0xFF9500), 0);
    lv_obj_set_style_radius(streakBadge, 14, 0);
    lv_obj_set_style_border_width(streakBadge, 0, 0);
    disableAllScrolling(streakBadge);

    char streakBuf[16];
    snprintf(streakBuf, sizeof(streakBuf), LV_SYMBOL_CHARGE " %d", activityStreakDays);
    lv_obj_t *streakLabel = lv_label_create(streakBadge);
    lv_label_set_text(streakLabel, streakBuf);
    lv_obj_set_style_text_color(streakLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(streakLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(streakLabel);

    // ═══ SESSION TIMER (Large center display) ═══
    unsigned long sessionElapsed = 0;
    if (sportTrackerActive) {
        sessionElapsed = millis() - sportSessionStart;
    }
    int mins = (sessionElapsed / 60000) % 60;
    int secs = (sessionElapsed / 1000) % 60;

    char timerBuf[16];
    snprintf(timerBuf, sizeof(timerBuf), "%02d:%02d", mins, secs);
    lv_obj_t *timerLabel = lv_label_create(card);
    lv_label_set_text(timerLabel, timerBuf);
    lv_obj_set_style_text_color(timerLabel, sportTrackerActive ? lv_color_hex(0x30D158) : lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(timerLabel, &lv_font_montserrat_32, 0);
    lv_obj_align(timerLabel, LV_ALIGN_TOP_MID, 0, 38);

    lv_obj_t *sessionLabel = lv_label_create(card);
    lv_label_set_text(sessionLabel, sportTrackerActive ? "SESSION ACTIVE" : "TAP TO START");
    lv_obj_set_style_text_color(sessionLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(sessionLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(sessionLabel, LV_ALIGN_TOP_MID, 0, 75);

    // ═══ STATS ROW 1: Steps + Distance ═══
    int sessionSteps = sportTrackerActive ? ((int)userData.steps - sportSessionStartSteps) : sportSessionSteps;
    if (sessionSteps < 0) sessionSteps = 0;
    float sessionDist = sessionSteps * 0.0007;  // km

    // Steps
    lv_obj_t *stepsBox = lv_obj_create(card);
    lv_obj_set_size(stepsBox, 90, 55);
    lv_obj_align(stepsBox, LV_ALIGN_CENTER, -55, 15);
    lv_obj_set_style_bg_color(stepsBox, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(stepsBox, 12, 0);
    lv_obj_set_style_border_width(stepsBox, 0, 0);
    disableAllScrolling(stepsBox);

    char stepsBuf[16];
    snprintf(stepsBuf, sizeof(stepsBuf), "%d", sessionSteps);
    lv_obj_t *stepsVal = lv_label_create(stepsBox);
    lv_label_set_text(stepsVal, stepsBuf);
    lv_obj_set_style_text_color(stepsVal, theme->accent, 0);
    lv_obj_set_style_text_font(stepsVal, &lv_font_montserrat_20, 0);
    lv_obj_align(stepsVal, LV_ALIGN_CENTER, 0, -5);

    lv_obj_t *stepsLbl = lv_label_create(stepsBox);
    lv_label_set_text(stepsLbl, "STEPS");
    lv_obj_set_style_text_color(stepsLbl, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(stepsLbl, &lv_font_montserrat_10, 0);
    lv_obj_align(stepsLbl, LV_ALIGN_CENTER, 0, 15);

    // Distance
    lv_obj_t *distBox = lv_obj_create(card);
    lv_obj_set_size(distBox, 90, 55);
    lv_obj_align(distBox, LV_ALIGN_CENTER, 55, 15);
    lv_obj_set_style_bg_color(distBox, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(distBox, 12, 0);
    lv_obj_set_style_border_width(distBox, 0, 0);
    disableAllScrolling(distBox);

    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.2f", sessionDist);
    lv_obj_t *distVal = lv_label_create(distBox);
    lv_label_set_text(distVal, distBuf);
    lv_obj_set_style_text_color(distVal, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(distVal, &lv_font_montserrat_20, 0);
    lv_obj_align(distVal, LV_ALIGN_CENTER, 0, -5);

    lv_obj_t *distLbl = lv_label_create(distBox);
    lv_label_set_text(distLbl, "KM");
    lv_obj_set_style_text_color(distLbl, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(distLbl, &lv_font_montserrat_10, 0);
    lv_obj_align(distLbl, LV_ALIGN_CENTER, 0, 15);

    // ═══ SPRINT COUNTER (Bottom stats) ═══
    lv_obj_t *sprintBox = lv_obj_create(card);
    lv_obj_set_size(sprintBox, LCD_WIDTH - 40, 45);
    lv_obj_align(sprintBox, LV_ALIGN_CENTER, 0, 75);
    lv_obj_set_style_bg_color(sprintBox, lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_bg_opa(sprintBox, LV_OPA_30, 0);
    lv_obj_set_style_radius(sprintBox, 12, 0);
    lv_obj_set_style_border_width(sprintBox, 0, 0);
    disableAllScrolling(sprintBox);

    char sprintBuf[32];
    snprintf(sprintBuf, sizeof(sprintBuf), LV_SYMBOL_WARNING " SPRINTS: %d", sportSprintCount);
    lv_obj_t *sprintLabel = lv_label_create(sprintBox);
    lv_label_set_text(sprintLabel, sprintBuf);
    lv_obj_set_style_text_color(sprintLabel, lv_color_hex(0xFF6B6B), 0);
    lv_obj_set_style_text_font(sprintLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(sprintLabel);

    // ═══ START/STOP BUTTON ═══
    lv_obj_t *actionBtn = lv_obj_create(card);
    lv_obj_set_size(actionBtn, LCD_WIDTH - 50, 48);
    lv_obj_align(actionBtn, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_bg_color(actionBtn, sportTrackerActive ? lv_color_hex(0xFF453A) : lv_color_hex(0x30D158), 0);
    lv_obj_set_style_radius(actionBtn, 24, 0);
    lv_obj_set_style_border_width(actionBtn, 0, 0);
    disableAllScrolling(actionBtn);

    lv_obj_t *actionLabel = lv_label_create(actionBtn);
    lv_label_set_text(actionLabel, sportTrackerActive ? LV_SYMBOL_STOP " STOP SESSION" : LV_SYMBOL_PLAY " START SESSION");
    lv_obj_set_style_text_color(actionLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(actionLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(actionLabel);
}

// ═══════════════════════════════════════════════════════════════════════════
// QUICK ACTIONS DOCK (Control Center) - Apple-style icons
// Swipe DOWN from clock to open, swipe UP to close
// ═══════════════════════════════════════════════════════════════════════════
void createQuickActionsDock() {
    disableAllScrolling(lv_scr_act());

    // ═══════════════════════════════════════════════════════════════════════
    // iPHONE iOS 17+ CONTROL CENTER STYLE
    // Glassmorphism background with blur effect simulation
    // ═══════════════════════════════════════════════════════════════════════
    
    lv_obj_t *dock = lv_obj_create(lv_scr_act());
    lv_obj_set_size(dock, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(dock);
    // Deep translucent background (simulating blur)
    lv_obj_set_style_bg_color(dock, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(dock, LV_OPA_90, 0);
    lv_obj_set_style_radius(dock, 0, 0);
    lv_obj_set_style_border_width(dock, 0, 0);
    disableAllScrolling(dock);

    // ═══ TOP BAR - Subtle grabber pill ═══
    lv_obj_t *grabber = lv_obj_create(dock);
    lv_obj_set_size(grabber, 36, 5);
    lv_obj_align(grabber, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_bg_color(grabber, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(grabber, 3, 0);
    lv_obj_set_style_border_width(grabber, 0, 0);
    disableAllScrolling(grabber);

    // ═══ CONNECTIVITY MODULE (Left pill - BLE + WiFi) ═══
    int moduleY = 22;
    int moduleHeight = 90;
    int moduleWidth = (LCD_WIDTH / 2) - 12;
    
    lv_obj_t *connectModule = lv_obj_create(dock);
    lv_obj_set_size(connectModule, moduleWidth, moduleHeight);
    lv_obj_align(connectModule, LV_ALIGN_TOP_LEFT, 8, moduleY);
    lv_obj_set_style_bg_color(connectModule, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(connectModule, 18, 0);
    lv_obj_set_style_border_width(connectModule, 0, 0);
    disableAllScrolling(connectModule);

    // BLE button (top-left of module)
    lv_obj_t *bleBtn = lv_obj_create(connectModule);
    lv_obj_set_size(bleBtn, 44, 44);
    lv_obj_align(bleBtn, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_obj_set_style_bg_color(bleBtn, bleEnabled ? lv_color_hex(0x0A84FF) : lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(bleBtn, 22, 0);  // Perfect circle
    lv_obj_set_style_border_width(bleBtn, 0, 0);
    disableAllScrolling(bleBtn);
    
    lv_obj_t *bleIcon = lv_label_create(bleBtn);
    lv_label_set_text(bleIcon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(bleIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(bleIcon, &lv_font_montserrat_18, 0);
    lv_obj_center(bleIcon);

    // WiFi button (top-right of module)
    lv_obj_t *wifiBtn = lv_obj_create(connectModule);
    lv_obj_set_size(wifiBtn, 44, 44);
    lv_obj_align(wifiBtn, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_set_style_bg_color(wifiBtn, wifiConnected ? lv_color_hex(0x0A84FF) : lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(wifiBtn, 22, 0);
    lv_obj_set_style_border_width(wifiBtn, 0, 0);
    disableAllScrolling(wifiBtn);
    
    lv_obj_t *wifiIcon = lv_label_create(wifiBtn);
    lv_label_set_text(wifiIcon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifiIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(wifiIcon, &lv_font_montserrat_18, 0);
    lv_obj_center(wifiIcon);

    // Connection status text
    lv_obj_t *connStatus = lv_label_create(connectModule);
    if (bleDeviceConnected) {
        lv_label_set_text(connStatus, "Connected");
        lv_obj_set_style_text_color(connStatus, lv_color_hex(0x30D158), 0);
    } else if (bleEnabled) {
        lv_label_set_text(connStatus, "Searching...");
        lv_obj_set_style_text_color(connStatus, lv_color_hex(0xFFCC00), 0);
    } else {
        lv_label_set_text(connStatus, "Not Connected");
        lv_obj_set_style_text_color(connStatus, lv_color_hex(0x8E8E93), 0);
    }
    lv_obj_set_style_text_font(connStatus, &lv_font_montserrat_10, 0);
    lv_obj_align(connStatus, LV_ALIGN_BOTTOM_MID, 0, -8);

    // ═══ MEDIA/QUICK MODULE (Right pill - Torch + Games) ═══
    lv_obj_t *mediaModule = lv_obj_create(dock);
    lv_obj_set_size(mediaModule, moduleWidth, moduleHeight);
    lv_obj_align(mediaModule, LV_ALIGN_TOP_RIGHT, -8, moduleY);
    lv_obj_set_style_bg_color(mediaModule, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(mediaModule, 18, 0);
    lv_obj_set_style_border_width(mediaModule, 0, 0);
    disableAllScrolling(mediaModule);

    // Torch button (top-left of module)
    lv_obj_t *torchBtn = lv_obj_create(mediaModule);
    lv_obj_set_size(torchBtn, 44, 44);
    lv_obj_align(torchBtn, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_obj_set_style_bg_color(torchBtn, torchEnabled ? lv_color_hex(0xFFD60A) : lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(torchBtn, 22, 0);
    lv_obj_set_style_border_width(torchBtn, 0, 0);
    disableAllScrolling(torchBtn);
    
    lv_obj_t *torchIcon = lv_label_create(torchBtn);
    lv_label_set_text(torchIcon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_color(torchIcon, torchEnabled ? lv_color_hex(0x000000) : lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(torchIcon, &lv_font_montserrat_18, 0);
    lv_obj_center(torchIcon);

    // Games button (top-right - REPLACES STEPS)
    lv_obj_t *gamesBtn = lv_obj_create(mediaModule);
    lv_obj_set_size(gamesBtn, 44, 44);
    lv_obj_align(gamesBtn, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_set_style_bg_color(gamesBtn, lv_color_hex(0xBF5AF2), 0);  // iOS purple
    lv_obj_set_style_radius(gamesBtn, 22, 0);
    lv_obj_set_style_border_width(gamesBtn, 0, 0);
    disableAllScrolling(gamesBtn);
    
    lv_obj_t *gamesIcon = lv_label_create(gamesBtn);
    lv_label_set_text(gamesIcon, LV_SYMBOL_PLAY);  // Play icon for games
    lv_obj_set_style_text_color(gamesIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(gamesIcon, &lv_font_montserrat_18, 0);
    lv_obj_center(gamesIcon);

    // Module label
    lv_obj_t *mediaLabel = lv_label_create(mediaModule);
    lv_label_set_text(mediaLabel, "Quick Access");
    lv_obj_set_style_text_color(mediaLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(mediaLabel, &lv_font_montserrat_10, 0);
    lv_obj_align(mediaLabel, LV_ALIGN_BOTTOM_MID, 0, -8);

    // ═══ BOTTOM ROW - Brightness Slider + Notification ═══
    int row2Y = moduleY + moduleHeight + 10;
    int sliderWidth = LCD_WIDTH - 80;
    
    // Brightness slider module - INTERACTIVE DRAG CONTROL
    lv_obj_t *brightModule = lv_obj_create(dock);
    lv_obj_set_size(brightModule, sliderWidth, 50);
    lv_obj_align(brightModule, LV_ALIGN_TOP_LEFT, 8, row2Y);
    lv_obj_set_style_bg_color(brightModule, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(brightModule, 14, 0);
    lv_obj_set_style_border_width(brightModule, 0, 0);
    disableAllScrolling(brightModule);

    // Sun icon (low brightness)
    lv_obj_t *sunIconLow = lv_label_create(brightModule);
    lv_label_set_text(sunIconLow, LV_SYMBOL_IMAGE);
    lv_obj_set_style_text_color(sunIconLow, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(sunIconLow, &lv_font_montserrat_12, 0);
    lv_obj_align(sunIconLow, LV_ALIGN_LEFT_MID, 8, 0);

    // Sun icon (high brightness)
    lv_obj_t *sunIconHigh = lv_label_create(brightModule);
    lv_label_set_text(sunIconHigh, LV_SYMBOL_IMAGE);
    lv_obj_set_style_text_color(sunIconHigh, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(sunIconHigh, &lv_font_montserrat_16, 0);
    lv_obj_align(sunIconHigh, LV_ALIGN_RIGHT_MID, -8, 0);

    // Interactive brightness bar - LARGER for touch
    int barWidth = sliderWidth - 70;
    dockBrightnessBarWidth = barWidth;  // Store for touch handling
    dockBrightnessBarX = 8 + 30;  // Module X (8) + icon space (30)
    dockBrightnessBarY = row2Y;   // Store Y position
    dockBrightness = saverModes[batterySaverLevel].brightness;  // Sync with current
    int fillWidth = (dockBrightness * barWidth) / 255;
    
    lv_obj_t *brightBar = lv_obj_create(brightModule);
    lv_obj_set_size(brightBar, barWidth, 20);  // Taller for easier touch
    lv_obj_align(brightBar, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(brightBar, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_radius(brightBar, 10, 0);
    lv_obj_set_style_border_width(brightBar, 0, 0);
    disableAllScrolling(brightBar);

    // Filled portion (white glow effect)
    lv_obj_t *brightFill = lv_obj_create(brightBar);
    lv_obj_set_size(brightFill, max(fillWidth, 20), 20);  // Min 20px for knob
    lv_obj_align(brightFill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(brightFill, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_radius(brightFill, 10, 0);
    lv_obj_set_style_border_width(brightFill, 0, 0);
    disableAllScrolling(brightFill);

    // Drag hint label
    lv_obj_t *dragHint = lv_label_create(brightModule);
    char brightPct[8];
    snprintf(brightPct, sizeof(brightPct), "%d%%", (dockBrightness * 100) / 255);
    lv_label_set_text(dragHint, brightPct);
    lv_obj_set_style_text_color(dragHint, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(dragHint, &lv_font_montserrat_10, 0);
    lv_obj_align(dragHint, LV_ALIGN_BOTTOM_MID, 0, -2);

    // Notification button (standalone circle)
    lv_obj_t *notifBtn = lv_obj_create(dock);
    lv_obj_set_size(notifBtn, 50, 50);
    lv_obj_align(notifBtn, LV_ALIGN_TOP_RIGHT, -8, row2Y);
    lv_obj_set_style_bg_color(notifBtn, notificationCount > 0 ? lv_color_hex(0xFF453A) : lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(notifBtn, 14, 0);
    lv_obj_set_style_border_width(notifBtn, 0, 0);
    disableAllScrolling(notifBtn);

    lv_obj_t *notifIcon = lv_label_create(notifBtn);
    lv_label_set_text(notifIcon, LV_SYMBOL_BELL);
    lv_obj_set_style_text_color(notifIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(notifIcon, &lv_font_montserrat_18, 0);
    lv_obj_center(notifIcon);

    // Badge if notifications
    if (notificationCount > 0) {
        lv_obj_t *badge = lv_obj_create(notifBtn);
        lv_obj_set_size(badge, 16, 16);
        lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 4, -4);
        lv_obj_set_style_bg_color(badge, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_radius(badge, 8, 0);
        lv_obj_set_style_border_width(badge, 0, 0);
        disableAllScrolling(badge);

        char countBuf[4];
        snprintf(countBuf, sizeof(countBuf), "%d", min(notificationCount, 9));
        lv_obj_t *countLbl = lv_label_create(badge);
        lv_label_set_text(countLbl, countBuf);
        lv_obj_set_style_text_color(countLbl, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_text_font(countLbl, &lv_font_montserrat_10, 0);
        lv_obj_center(countLbl);
    }

    // ═══ BOTTOM UTILITY ROW - Focus + Battery Saver ═══
    int row3Y = row2Y + 60;
    int utilWidth = (LCD_WIDTH / 2) - 12;

    // Battery Saver button
    lv_obj_t *saverBtn = lv_obj_create(dock);
    lv_obj_set_size(saverBtn, utilWidth, 42);
    lv_obj_align(saverBtn, LV_ALIGN_TOP_LEFT, 8, row3Y);
    lv_obj_set_style_bg_color(saverBtn, batterySaverLevel > 0 ? lv_color_hex(0x30D158) : lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(saverBtn, 12, 0);
    lv_obj_set_style_border_width(saverBtn, 0, 0);
    disableAllScrolling(saverBtn);

    lv_obj_t *saverIcon = lv_label_create(saverBtn);
    lv_label_set_text(saverIcon, LV_SYMBOL_BATTERY_FULL);
    lv_obj_set_style_text_color(saverIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(saverIcon, &lv_font_montserrat_14, 0);
    lv_obj_align(saverIcon, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t *saverLabel = lv_label_create(saverBtn);
    lv_label_set_text(saverLabel, saverModes[batterySaverLevel].name);
    lv_obj_set_style_text_color(saverLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(saverLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(saverLabel, LV_ALIGN_LEFT_MID, 35, 0);

    // Settings shortcut
    lv_obj_t *settingsBtn = lv_obj_create(dock);
    lv_obj_set_size(settingsBtn, utilWidth, 42);
    lv_obj_align(settingsBtn, LV_ALIGN_TOP_RIGHT, -8, row3Y);
    lv_obj_set_style_bg_color(settingsBtn, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(settingsBtn, 12, 0);
    lv_obj_set_style_border_width(settingsBtn, 0, 0);
    disableAllScrolling(settingsBtn);

    lv_obj_t *settingsIcon = lv_label_create(settingsBtn);
    lv_label_set_text(settingsIcon, LV_SYMBOL_SETTINGS);
    lv_obj_set_style_text_color(settingsIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(settingsIcon, &lv_font_montserrat_14, 0);
    lv_obj_align(settingsIcon, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t *settingsLabel = lv_label_create(settingsBtn);
    lv_label_set_text(settingsLabel, "Settings");
    lv_obj_set_style_text_color(settingsLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(settingsLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(settingsLabel, LV_ALIGN_LEFT_MID, 35, 0);

    // ═══ SWIPE HINT at bottom ═══
    lv_obj_t *hint = lv_label_create(dock);
    lv_label_set_text(hint, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x48484A), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_16, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// Handle taps on Quick Actions Dock - iOS Control Center Style
void handleQuickActionsTap(int x, int y) {
    if (!quickActionsDockVisible) return;

    // ═══════════════════════════════════════════════════════════════════════
    // NEW iOS CONTROL CENTER LAYOUT HIT DETECTION
    // ═══════════════════════════════════════════════════════════════════════
    
    int moduleY = 22;
    int moduleHeight = 90;
    int moduleWidth = (LCD_WIDTH / 2) - 12;
    int row2Y = moduleY + moduleHeight + 10;
    int row3Y = row2Y + 60;

    // ═══ CONNECTIVITY MODULE (Left side) ═══
    if (x >= 8 && x <= 8 + moduleWidth && y >= moduleY && y <= moduleY + moduleHeight) {
        // Inside connectivity module
        // BLE button (top-left, 44x44 starting at module x+8, y+8)
        if (x >= 16 && x <= 60 && y >= moduleY + 8 && y <= moduleY + 52) {
            USBSerial.println("[DOCK] BLE Toggle");
            if (bleEnabled) {
                stopBLE();
            } else {
                startBLE();
            }
            createQuickActionsDock();
            return;
        }
        // WiFi button (top-right of module)
        if (x >= moduleWidth - 36 && x <= moduleWidth + 8 && y >= moduleY + 8 && y <= moduleY + 52) {
            USBSerial.println("[DOCK] WiFi Scan");
            if (!wifiConnected) {
                scanWiFiNetworks();
            }
            createQuickActionsDock();
            return;
        }
    }

    // ═══ MEDIA MODULE (Right side) ═══
    int rightModuleX = LCD_WIDTH - 8 - moduleWidth;
    if (x >= rightModuleX && x <= LCD_WIDTH - 8 && y >= moduleY && y <= moduleY + moduleHeight) {
        // Torch button (left side of right module)
        if (x >= rightModuleX + 8 && x <= rightModuleX + 52 && y >= moduleY + 8 && y <= moduleY + 52) {
            USBSerial.println("[DOCK] Torch Toggle");
            torchEnabled = !torchEnabled;
            if (torchEnabled) {
                gfx->setBrightness(255);
            } else {
                gfx->setBrightness(saverModes[batterySaverLevel].brightness);
            }
            createQuickActionsDock();
            return;
        }
        // Games button (right side of right module) - NAVIGATES TO GAMES
        if (x >= rightModuleX + moduleWidth - 52 && x <= rightModuleX + moduleWidth - 8 && y >= moduleY + 8 && y <= moduleY + 52) {
            USBSerial.println("[DOCK] Opening Games");
            quickActionsDockVisible = false;
            navigateTo(CAT_GAMES, 0);  // Navigate to games menu
            return;
        }
    }

    // ═══ NOTIFICATION BUTTON (standalone) ═══
    if (x >= LCD_WIDTH - 58 && x <= LCD_WIDTH - 8 && y >= row2Y && y <= row2Y + 50) {
        USBSerial.println("[DOCK] Opening Notifications");
        quickActionsDockVisible = false;
        navigateTo(CAT_BLUETOOTH, 1);  // Notifications subcard
        return;
    }

    // ═══ BATTERY SAVER BUTTON ═══
    int utilWidth = (LCD_WIDTH / 2) - 12;
    if (x >= 8 && x <= 8 + utilWidth && y >= row3Y && y <= row3Y + 42) {
        USBSerial.println("[DOCK] Cycling Battery Saver");
        BatterySaverLevel newLevel = (BatterySaverLevel)((batterySaverLevel + 1) % 3);
        applyBatterySaverMode(newLevel);
        createQuickActionsDock();
        return;
    }

    // ═══ SETTINGS BUTTON ═══
    if (x >= LCD_WIDTH - 8 - utilWidth && x <= LCD_WIDTH - 8 && y >= row3Y && y <= row3Y + 42) {
        USBSerial.println("[DOCK] Opening Settings");
        quickActionsDockVisible = false;
        navigateTo(CAT_SETTINGS, 0);
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETTINGS CARD - WITH INDIVIDUAL TAP BUTTONS + 12/24HR TOGGLE
// ═══════════════════════════════════════════════════════════════════════════
void createSettingsCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_SETTINGS " SETTINGS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    int yPos = 32;
    int btnHeight = 42;
    int btnGap = 6;

    // ═══ WATCH FACE BUTTON ═══
    lv_obj_t *faceBtn = lv_obj_create(card);
    lv_obj_set_size(faceBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(faceBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(faceBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(faceBtn, 12, 0);
    lv_obj_set_style_border_width(faceBtn, 0, 0);
    disableAllScrolling(faceBtn);

    lv_obj_t *faceLabel = lv_label_create(faceBtn);
    lv_label_set_text(faceLabel, "WATCH FACE");
    lv_obj_set_style_text_color(faceLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(faceLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(faceLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *faceVal = lv_label_create(faceBtn);
    lv_label_set_text(faceVal, watchFaces[userData.watchFaceIndex].name);
    lv_obj_set_style_text_color(faceVal, theme->accent, 0);
    lv_obj_set_style_text_font(faceVal, &lv_font_montserrat_16, 0);
    lv_obj_align(faceVal, LV_ALIGN_LEFT_MID, 15, 10);

    lv_obj_t *faceArrow = lv_label_create(faceBtn);
    lv_label_set_text(faceArrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(faceArrow, lv_color_hex(0x636366), 0);
    lv_obj_align(faceArrow, LV_ALIGN_RIGHT_MID, -10, 0);

    yPos += btnHeight + btnGap;

    // ═══ WALLPAPER BUTTON ═══
    lv_obj_t *wallBtn = lv_obj_create(card);
    lv_obj_set_size(wallBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(wallBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(wallBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(wallBtn, 12, 0);
    lv_obj_set_style_border_width(wallBtn, 0, 0);
    disableAllScrolling(wallBtn);

    lv_obj_t *wallLabel = lv_label_create(wallBtn);
    lv_label_set_text(wallLabel, "WALLPAPER");
    lv_obj_set_style_text_color(wallLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(wallLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(wallLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *wallVal = lv_label_create(wallBtn);
    lv_label_set_text(wallVal, gradientWallpapers[userData.wallpaperIndex].name);
    lv_obj_set_style_text_color(wallVal, theme->accent, 0);
    lv_obj_set_style_text_font(wallVal, &lv_font_montserrat_16, 0);
    lv_obj_align(wallVal, LV_ALIGN_LEFT_MID, 15, 10);

    lv_obj_t *wallArrow = lv_label_create(wallBtn);
    lv_label_set_text(wallArrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(wallArrow, lv_color_hex(0x636366), 0);
    lv_obj_align(wallArrow, LV_ALIGN_RIGHT_MID, -10, 0);

    yPos += btnHeight + btnGap;

    // ═══ THEME BUTTON ═══
    lv_obj_t *themeBtn = lv_obj_create(card);
    lv_obj_set_size(themeBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(themeBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(themeBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(themeBtn, 12, 0);
    lv_obj_set_style_border_width(themeBtn, 0, 0);
    disableAllScrolling(themeBtn);

    lv_obj_t *themeLabel = lv_label_create(themeBtn);
    lv_label_set_text(themeLabel, "THEME");
    lv_obj_set_style_text_color(themeLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(themeLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(themeLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *themeVal = lv_label_create(themeBtn);
    lv_label_set_text(themeVal, theme->name);
    lv_obj_set_style_text_color(themeVal, theme->accent, 0);
    lv_obj_set_style_text_font(themeVal, &lv_font_montserrat_16, 0);
    lv_obj_align(themeVal, LV_ALIGN_LEFT_MID, 15, 10);

    // Theme color preview
    lv_obj_t *themePreview = lv_obj_create(themeBtn);
    lv_obj_set_size(themePreview, 24, 24);
    lv_obj_align(themePreview, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(themePreview, theme->accent, 0);
    lv_obj_set_style_radius(themePreview, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(themePreview, 0, 0);
    disableAllScrolling(themePreview);

    yPos += btnHeight + btnGap;

    // ═══ BRIGHTNESS BUTTON ═══
    lv_obj_t *brightBtn = lv_obj_create(card);
    lv_obj_set_size(brightBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(brightBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(brightBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(brightBtn, 12, 0);
    lv_obj_set_style_border_width(brightBtn, 0, 0);
    disableAllScrolling(brightBtn);

    lv_obj_t *brightLabel = lv_label_create(brightBtn);
    lv_label_set_text(brightLabel, "BRIGHTNESS");
    lv_obj_set_style_text_color(brightLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(brightLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(brightLabel, LV_ALIGN_LEFT_MID, 15, -8);

    char brightBuf[16];
    int brightPercent = ((userData.brightness - 50) * 100) / 205;
    snprintf(brightBuf, sizeof(brightBuf), "%d%%", brightPercent);
    lv_obj_t *brightVal = lv_label_create(brightBtn);
    lv_label_set_text(brightVal, brightBuf);
    lv_obj_set_style_text_color(brightVal, theme->accent, 0);
    lv_obj_set_style_text_font(brightVal, &lv_font_montserrat_16, 0);
    lv_obj_align(brightVal, LV_ALIGN_LEFT_MID, 15, 10);

    // Brightness bar mini preview
    lv_obj_t *brightBar = lv_obj_create(brightBtn);
    lv_obj_set_size(brightBar, 60, 8);
    lv_obj_align(brightBar, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(brightBar, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_radius(brightBar, 4, 0);
    lv_obj_set_style_border_width(brightBar, 0, 0);
    disableAllScrolling(brightBar);

    // Filled portion
    int fillWidth = (userData.brightness - 50) * 60 / 205;
    lv_obj_t *brightFill = lv_obj_create(brightBar);
    lv_obj_set_size(brightFill, fillWidth, 8);
    lv_obj_align(brightFill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(brightFill, theme->accent, 0);
    lv_obj_set_style_radius(brightFill, 4, 0);
    lv_obj_set_style_border_width(brightFill, 0, 0);
    disableAllScrolling(brightFill);

    yPos += btnHeight + btnGap;

    // ═══ 12/24 HOUR FORMAT TOGGLE ═══
    lv_obj_t *timeFormatBtn = lv_obj_create(card);
    lv_obj_set_size(timeFormatBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(timeFormatBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(timeFormatBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(timeFormatBtn, 12, 0);
    lv_obj_set_style_border_width(timeFormatBtn, 0, 0);
    disableAllScrolling(timeFormatBtn);

    lv_obj_t *timeFormatLabel = lv_label_create(timeFormatBtn);
    lv_label_set_text(timeFormatLabel, "TIME FORMAT");
    lv_obj_set_style_text_color(timeFormatLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(timeFormatLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(timeFormatLabel, LV_ALIGN_LEFT_MID, 15, 0);

    lv_obj_t *timeFormatVal = lv_label_create(timeFormatBtn);
    lv_label_set_text(timeFormatVal, use24HourFormat ? "24HR" : "12HR");
    lv_obj_set_style_text_color(timeFormatVal, theme->accent, 0);
    lv_obj_set_style_text_font(timeFormatVal, &lv_font_montserrat_16, 0);
    lv_obj_align(timeFormatVal, LV_ALIGN_RIGHT_MID, -15, 0);

    yPos += btnHeight + btnGap;

    // ═══ BATTERY SAVER BUTTON ═══
    lv_obj_t *saverBtn = lv_obj_create(card);
    lv_obj_set_size(saverBtn, LCD_WIDTH - 30, 40);
    lv_obj_align(saverBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(saverBtn, batterySaverMode ? lv_color_hex(0x30D158) : lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(saverBtn, 12, 0);
    lv_obj_set_style_border_width(saverBtn, 0, 0);
    disableAllScrolling(saverBtn);

    lv_obj_t *saverLabel = lv_label_create(saverBtn);
    lv_label_set_text(saverLabel, batterySaverMode ? LV_SYMBOL_OK " SAVER ON" : "BATTERY SAVER");
    lv_obj_set_style_text_color(saverLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(saverLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(saverLabel);
}

// ═══════════════════════════════════════════════════════════════════════════
// HANDLE TAP IN SETTINGS - Individual buttons for each setting
// ═══════════════════════════════════════════════════════════════════════════
void handleSettingsTap(int x, int y) {
    // Button layout: btnHeight=42, btnGap=6, starting at y=32
    // Button 1 (Watch Face): y=32 to y=74
    // Button 2 (Wallpaper): y=80 to y=122
    // Button 3 (Theme): y=128 to y=170
    // Button 4 (Brightness): y=176 to y=218
    // Button 5 (Time Format): y=224 to y=266
    // Button 6 (Battery Saver): y=272 to y=312
    
    if (y >= 32 && y < 74) {
        // WATCH FACE button - cycle watch face
        userData.watchFaceIndex = (userData.watchFaceIndex + 1) % NUM_WATCH_FACES;
        USBSerial.printf("[SETTINGS] Watch face: %s\n", watchFaces[userData.watchFaceIndex].name);
    } 
    else if (y >= 80 && y < 122) {
        // WALLPAPER button - cycle wallpaper
        userData.wallpaperIndex = (userData.wallpaperIndex + 1) % NUM_GRADIENT_WALLPAPERS;
        USBSerial.printf("[SETTINGS] Wallpaper: %s\n", gradientWallpapers[userData.wallpaperIndex].name);
    } 
    else if (y >= 128 && y < 170) {
        // THEME button - cycle theme
        userData.themeIndex = (userData.themeIndex + 1) % NUM_THEMES;
        USBSerial.printf("[SETTINGS] Theme: %s\n", gradientThemes[userData.themeIndex].name);
    } 
    else if (y >= 176 && y < 218) {
        // BRIGHTNESS button - tap left to decrease, right to increase
        if (x < LCD_WIDTH / 2) {
            userData.brightness = max(50, userData.brightness - 25);
        } else {
            userData.brightness = min(255, userData.brightness + 25);
        }
        gfx->setBrightness(batterySaverMode ? 50 : userData.brightness);
        USBSerial.printf("[SETTINGS] Brightness: %d%%\n", ((userData.brightness - 50) * 100) / 205);
    } 
    else if (y >= 224 && y < 266) {
        // TIME FORMAT button - toggle 12/24 hour
        use24HourFormat = !use24HourFormat;
        USBSerial.printf("[SETTINGS] Time format: %s\n", use24HourFormat ? "24HR" : "12HR");
    }
    else if (y >= 272 && y < 320) {
        // BATTERY SAVER button - toggle
        toggleBatterySaver();
        USBSerial.printf("[SETTINGS] Battery saver: %s\n", batterySaverMode ? "ON" : "OFF");
    }

    saveUserData();
    navigateTo(currentCategory, currentSubCard);
}

// ═══════════════════════════════════════════════════════════════════════════
// BLUETOOTH SETTINGS CARD
// ═══════════════════════════════════════════════════════════════════════════
void createBluetoothCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with Bluetooth icon
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_BLUETOOTH " BLUETOOTH");
    lv_obj_set_style_text_color(title, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    int yPos = 35;
    int btnHeight = 42;
    int btnGap = 6;

    // ═══ BLUETOOTH TOGGLE BUTTON ═══
    lv_obj_t *bleBtn = lv_obj_create(card);
    lv_obj_set_size(bleBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(bleBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(bleBtn, bleEnabled ? lv_color_hex(0x0A84FF) : lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(bleBtn, 12, 0);
    lv_obj_set_style_border_width(bleBtn, 0, 0);
    disableAllScrolling(bleBtn);

    lv_obj_t *bleLabel = lv_label_create(bleBtn);
    lv_label_set_text(bleLabel, "BLUETOOTH");
    lv_obj_set_style_text_color(bleLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(bleLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(bleLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *bleStatus = lv_label_create(bleBtn);
    // Show status with timer
    if (bleEnabled && bleDeviceConnected) {
        lv_label_set_text(bleStatus, "Connected");
        lv_obj_set_style_text_color(bleStatus, lv_color_hex(0x30D158), 0);
    } else if (bleEnabled) {
        // Show countdown timer
        unsigned long elapsed = millis() - bleLastActivityTime;
        unsigned long remaining = (BLE_AUTO_OFF_MS > elapsed) ? (BLE_AUTO_OFF_MS - elapsed) : 0;
        int secsLeft = remaining / 1000;
        int mins = secsLeft / 60;
        int secs = secsLeft % 60;
        char timerBuf[24];
        snprintf(timerBuf, sizeof(timerBuf), "ON - %d:%02d left", mins, secs);
        lv_label_set_text(bleStatus, timerBuf);
        lv_obj_set_style_text_color(bleStatus, lv_color_hex(0xFF9F0A), 0);
    } else {
        lv_label_set_text(bleStatus, "OFF - Tap to enable");
        lv_obj_set_style_text_color(bleStatus, lv_color_hex(0x8E8E93), 0);
    }
    lv_obj_set_style_text_font(bleStatus, &lv_font_montserrat_12, 0);
    lv_obj_align(bleStatus, LV_ALIGN_LEFT_MID, 15, 8);

    yPos += btnHeight + btnGap;

    // ═══ CONNECTION STATUS ═══
    lv_obj_t *connBtn = lv_obj_create(card);
    lv_obj_set_size(connBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(connBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(connBtn, bleDeviceConnected ? lv_color_hex(0x30D158) : lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(connBtn, 12, 0);
    lv_obj_set_style_border_width(connBtn, 0, 0);
    disableAllScrolling(connBtn);

    lv_obj_t *connLabel = lv_label_create(connBtn);
    lv_label_set_text(connLabel, "STATUS");
    lv_obj_set_style_text_color(connLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(connLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(connLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *connStatus = lv_label_create(connBtn);
    if (bleDeviceConnected) {
        lv_label_set_text(connStatus, bleConnectedDeviceName.length() > 0 ? bleConnectedDeviceName.c_str() : "Connected");
    } else if (bleEnabled) {
        lv_label_set_text(connStatus, "Searching...");
    } else {
        lv_label_set_text(connStatus, "Not Connected");
    }
    lv_obj_set_style_text_color(connStatus, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(connStatus, &lv_font_montserrat_12, 0);
    lv_obj_align(connStatus, LV_ALIGN_LEFT_MID, 15, 8);

    yPos += btnHeight + btnGap;

    // ═══ TIME SYNC STATUS ═══
    lv_obj_t *syncBtn = lv_obj_create(card);
    lv_obj_set_size(syncBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(syncBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(syncBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(syncBtn, 12, 0);
    lv_obj_set_style_border_width(syncBtn, 0, 0);
    disableAllScrolling(syncBtn);

    lv_obj_t *syncLabel = lv_label_create(syncBtn);
    lv_label_set_text(syncLabel, "TIME SYNC");
    lv_obj_set_style_text_color(syncLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(syncLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(syncLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *syncStatus = lv_label_create(syncBtn);
    lv_label_set_text(syncStatus, bleTimeSynced ? "Synced via BLE" : "Not synced");
    lv_obj_set_style_text_color(syncStatus, bleTimeSynced ? lv_color_hex(0x30D158) : lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(syncStatus, &lv_font_montserrat_12, 0);
    lv_obj_align(syncStatus, LV_ALIGN_LEFT_MID, 15, 8);

    yPos += btnHeight + btnGap;

    // ═══ DEVICE NAME ═══
    lv_obj_t *nameBtn = lv_obj_create(card);
    lv_obj_set_size(nameBtn, LCD_WIDTH - 30, btnHeight);
    lv_obj_align(nameBtn, LV_ALIGN_TOP_MID, 0, yPos);
    lv_obj_set_style_bg_color(nameBtn, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(nameBtn, 12, 0);
    lv_obj_set_style_border_width(nameBtn, 0, 0);
    disableAllScrolling(nameBtn);

    lv_obj_t *nameLabel = lv_label_create(nameBtn);
    lv_label_set_text(nameLabel, "DEVICE NAME");
    lv_obj_set_style_text_color(nameLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(nameLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 15, -8);

    lv_obj_t *nameVal = lv_label_create(nameBtn);
    lv_label_set_text(nameVal, BLE_DEVICE_NAME);
    lv_obj_set_style_text_color(nameVal, theme->accent, 0);
    lv_obj_set_style_text_font(nameVal, &lv_font_montserrat_12, 0);
    lv_obj_align(nameVal, LV_ALIGN_LEFT_MID, 15, 8);

    yPos += btnHeight + btnGap;

    // ═══ HINT ═══
    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, "Tap top button to toggle BLE");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// HANDLE BLUETOOTH CARD TAP
// ═══════════════════════════════════════════════════════════════════════════
void handleBluetoothTap(int x, int y) {
    // Button layout: btnHeight=42, btnGap=6, starting at y=35
    // Button 1 (BLE Toggle): y=35 to y=77
    
    if (y >= 35 && y < 77) {
        // Toggle BLE on/off
        if (bleEnabled) {
            stopBLE();
            USBSerial.println("[BLE] Manually disabled");
        } else {
            initGadgetbridgeBLE();
            USBSerial.println("[BLE] Manually enabled (3min timer)");
        }
    }
    
    navigateTo(currentCategory, currentSubCard);
}

// ═══════════════════════════════════════════════════════════════════════════
// NOTIFICATIONS CARD - PHONE NOTIFICATIONS VIA BLE
// ═══════════════════════════════════════════════════════════════════════════
int notifScrollOffset = 0;

// Update unique app filters list
void updateNotifAppFilters() {
    notifAppFilters[0] = "All";
    notifAppFilterCount = 1;
    
    for (int i = 0; i < notificationCount; i++) {
        bool found = false;
        for (int j = 1; j < notifAppFilterCount; j++) {
            if (notifAppFilters[j] == notifications[i].app) {
                found = true;
                break;
            }
        }
        if (!found && notifAppFilterCount < MAX_NOTIFICATIONS + 1) {
            notifAppFilters[notifAppFilterCount++] = notifications[i].app;
        }
    }
}

void addNotification(String app, String title, String body) {
    // Shift all notifications down
    for (int i = MAX_NOTIFICATIONS - 1; i > 0; i--) {
        notifications[i] = notifications[i-1];
    }
    // Add new notification at top
    notifications[0].app = app;
    notifications[0].title = title;
    notifications[0].body = body;
    notifications[0].timestamp = millis();
    notifications[0].read = false;
    
    if (notificationCount < MAX_NOTIFICATIONS) notificationCount++;
    
    // Update app filter list
    updateNotifAppFilters();
    
    USBSerial.printf("[NOTIF] New: %s - %s\n", app.c_str(), title.c_str());
    
    // Show popup if screen is on
    if (screenOn) {
        showNotificationPopup(0);
    }
}

void showNotificationPopup(int index) {
    if (index < 0 || index >= notificationCount) return;
    
    // Create popup overlay
    lv_obj_t *popup = lv_obj_create(lv_scr_act());
    lv_obj_set_size(popup, LCD_WIDTH - 20, 80);
    lv_obj_align(popup, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(popup, LV_OPA_90, 0);
    lv_obj_set_style_radius(popup, 16, 0);
    lv_obj_set_style_border_width(popup, 1, 0);
    lv_obj_set_style_border_color(popup, lv_color_hex(0x3A3A3C), 0);
    
    // App name
    lv_obj_t *appLabel = lv_label_create(popup);
    lv_label_set_text(appLabel, notifications[index].app.c_str());
    lv_obj_set_style_text_color(appLabel, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(appLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(appLabel, LV_ALIGN_TOP_LEFT, 10, 5);
    
    // Title
    lv_obj_t *titleLabel = lv_label_create(popup);
    lv_label_set_text(titleLabel, notifications[index].title.c_str());
    lv_obj_set_style_text_color(titleLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(titleLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(titleLabel, LV_ALIGN_TOP_LEFT, 10, 20);
    
    // Body (truncated)
    lv_obj_t *bodyLabel = lv_label_create(popup);
    String truncBody = notifications[index].body;
    if (truncBody.length() > 40) truncBody = truncBody.substring(0, 40) + "...";
    lv_label_set_text(bodyLabel, truncBody.c_str());
    lv_obj_set_style_text_color(bodyLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(bodyLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(bodyLabel, LV_ALIGN_TOP_LEFT, 10, 38);
    
    notifications[index].read = true;
}

// Get filtered notification count
int getFilteredNotifCount() {
    if (notificationFilter == 0) return notificationCount;  // "All"
    
    int count = 0;
    String filterApp = notifAppFilters[notificationFilter];
    for (int i = 0; i < notificationCount; i++) {
        if (notifications[i].app == filterApp) count++;
    }
    return count;
}

// Get notification index by filtered position
int getFilteredNotifIndex(int filteredPos) {
    if (notificationFilter == 0) return filteredPos;  // "All" - direct mapping
    
    String filterApp = notifAppFilters[notificationFilter];
    int count = 0;
    for (int i = 0; i < notificationCount; i++) {
        if (notifications[i].app == filterApp) {
            if (count == filteredPos) return i;
            count++;
        }
    }
    return -1;
}

void createNotificationsCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // ═══ FULL NOTIFICATION VIEW ═══
    if (selectedNotifIndex >= 0 && selectedNotifIndex < notificationCount) {
        // Back button
        lv_obj_t *backBtn = lv_obj_create(card);
        lv_obj_set_size(backBtn, LCD_WIDTH - 30, 32);
        lv_obj_align(backBtn, LV_ALIGN_TOP_MID, 0, 8);
        lv_obj_set_style_bg_color(backBtn, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(backBtn, 8, 0);
        lv_obj_set_style_border_width(backBtn, 0, 0);
        disableAllScrolling(backBtn);

        lv_obj_t *backLabel = lv_label_create(backBtn);
        lv_label_set_text(backLabel, LV_SYMBOL_LEFT " Back to list");
        lv_obj_set_style_text_color(backLabel, lv_color_hex(0x0A84FF), 0);
        lv_obj_set_style_text_font(backLabel, &lv_font_montserrat_12, 0);
        lv_obj_center(backLabel);

        // App name header
        lv_obj_t *appHeader = lv_label_create(card);
        lv_label_set_text(appHeader, notifications[selectedNotifIndex].app.c_str());
        lv_obj_set_style_text_color(appHeader, lv_color_hex(0x0A84FF), 0);
        lv_obj_set_style_text_font(appHeader, &lv_font_montserrat_14, 0);
        lv_obj_align(appHeader, LV_ALIGN_TOP_LEFT, 15, 50);

        // Time ago
        lv_obj_t *timeLabel = lv_label_create(card);
        unsigned long ago = (millis() - notifications[selectedNotifIndex].timestamp) / 1000;
        char timeBuf[20];
        if (ago < 60) snprintf(timeBuf, sizeof(timeBuf), "%lu seconds ago", ago);
        else if (ago < 3600) snprintf(timeBuf, sizeof(timeBuf), "%lu minutes ago", ago / 60);
        else snprintf(timeBuf, sizeof(timeBuf), "%lu hours ago", ago / 3600);
        lv_label_set_text(timeLabel, timeBuf);
        lv_obj_set_style_text_color(timeLabel, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(timeLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(timeLabel, LV_ALIGN_TOP_RIGHT, -15, 52);

        // Title (full)
        lv_obj_t *titleFull = lv_label_create(card);
        lv_label_set_text(titleFull, notifications[selectedNotifIndex].title.c_str());
        lv_obj_set_style_text_color(titleFull, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(titleFull, &lv_font_montserrat_14, 0);
        lv_obj_set_width(titleFull, LCD_WIDTH - 30);
        lv_label_set_long_mode(titleFull, LV_LABEL_LONG_WRAP);
        lv_obj_align(titleFull, LV_ALIGN_TOP_LEFT, 15, 75);

        // Body (full, scrollable area)
        lv_obj_t *bodyArea = lv_obj_create(card);
        lv_obj_set_size(bodyArea, LCD_WIDTH - 20, LCD_HEIGHT - 120);
        lv_obj_align(bodyArea, LV_ALIGN_TOP_MID, 0, 110);
        lv_obj_set_style_bg_color(bodyArea, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(bodyArea, 12, 0);
        lv_obj_set_style_border_width(bodyArea, 0, 0);
        lv_obj_set_style_pad_all(bodyArea, 10, 0);
        lv_obj_set_scroll_dir(bodyArea, LV_DIR_VER);

        lv_obj_t *bodyFull = lv_label_create(bodyArea);
        lv_label_set_text(bodyFull, notifications[selectedNotifIndex].body.c_str());
        lv_obj_set_style_text_color(bodyFull, lv_color_hex(0xE5E5E7), 0);
        lv_obj_set_style_text_font(bodyFull, &lv_font_montserrat_12, 0);
        lv_obj_set_width(bodyFull, LCD_WIDTH - 50);
        lv_label_set_long_mode(bodyFull, LV_LABEL_LONG_WRAP);

        notifications[selectedNotifIndex].read = true;
        return;
    }

    // ═══ LIST VIEW ═══
    // Title with bell icon
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_BELL " NOTIFICATIONS");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // App filter button (tap to cycle through apps)
    lv_obj_t *filterBtn = lv_obj_create(card);
    lv_obj_set_size(filterBtn, LCD_WIDTH - 30, 36);
    lv_obj_align(filterBtn, LV_ALIGN_TOP_MID, 0, 32);
    lv_obj_set_style_bg_color(filterBtn, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_radius(filterBtn, 10, 0);
    lv_obj_set_style_border_width(filterBtn, 0, 0);
    disableAllScrolling(filterBtn);

    lv_obj_t *filterLabel = lv_label_create(filterBtn);
    char filterBuf[40];
    if (notificationFilter < notifAppFilterCount) {
        snprintf(filterBuf, sizeof(filterBuf), LV_SYMBOL_LEFT " %s " LV_SYMBOL_RIGHT, notifAppFilters[notificationFilter].c_str());
    } else {
        snprintf(filterBuf, sizeof(filterBuf), LV_SYMBOL_LEFT " All " LV_SYMBOL_RIGHT);
    }
    lv_label_set_text(filterLabel, filterBuf);
    lv_obj_set_style_text_color(filterLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(filterLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(filterLabel);

    // Synced steps display
    lv_obj_t *stepsRow = lv_obj_create(card);
    lv_obj_set_size(stepsRow, LCD_WIDTH - 30, 32);
    lv_obj_align(stepsRow, LV_ALIGN_TOP_MID, 0, 72);
    lv_obj_set_style_bg_color(stepsRow, stepsSynced ? lv_color_hex(0x30D158) : lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(stepsRow, 10, 0);
    lv_obj_set_style_border_width(stepsRow, 0, 0);
    disableAllScrolling(stepsRow);

    lv_obj_t *stepsLabel = lv_label_create(stepsRow);
    char stepsBuf[50];
    if (stepsSynced) {
        snprintf(stepsBuf, sizeof(stepsBuf), LV_SYMBOL_REFRESH " %d steps | %d cal", syncedSteps, syncedCalories);
    } else {
        snprintf(stepsBuf, sizeof(stepsBuf), LV_SYMBOL_REFRESH " Steps: Connect to sync");
    }
    lv_label_set_text(stepsLabel, stepsBuf);
    lv_obj_set_style_text_color(stepsLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(stepsLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(stepsLabel);

    // Notification count for current filter
    int filteredCount = getFilteredNotifCount();
    lv_obj_t *countLabel = lv_label_create(card);
    char countBuf[40];
    snprintf(countBuf, sizeof(countBuf), "%d notification%s", filteredCount, filteredCount == 1 ? "" : "s");
    lv_label_set_text(countLabel, countBuf);
    lv_obj_set_style_text_color(countLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(countLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(countLabel, LV_ALIGN_TOP_LEFT, 18, 108);

    // Notification list area
    int yPos = 124;
    int itemHeight = 52;
    int maxVisible = 3;
    
    int displayedCount = 0;
    for (int filteredIdx = notifScrollOffset; filteredIdx < filteredCount && displayedCount < maxVisible; filteredIdx++) {
        int realIdx = getFilteredNotifIndex(filteredIdx);
        if (realIdx < 0) continue;
        
        lv_obj_t *notifItem = lv_obj_create(card);
        lv_obj_set_size(notifItem, LCD_WIDTH - 20, itemHeight - 4);
        lv_obj_align(notifItem, LV_ALIGN_TOP_MID, 0, yPos + (displayedCount * itemHeight));
        lv_obj_set_style_bg_color(notifItem, notifications[realIdx].read ? lv_color_hex(0x1C1C1E) : lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_radius(notifItem, 12, 0);
        lv_obj_set_style_border_width(notifItem, 0, 0);
        disableAllScrolling(notifItem);

        // App name (top left, colored)
        lv_obj_t *appLbl = lv_label_create(notifItem);
        lv_label_set_text(appLbl, notifications[realIdx].app.c_str());
        lv_obj_set_style_text_color(appLbl, lv_color_hex(0x0A84FF), 0);
        lv_obj_set_style_text_font(appLbl, &lv_font_montserrat_12, 0);
        lv_obj_align(appLbl, LV_ALIGN_TOP_LEFT, 10, 3);

        // Time ago (top right)
        lv_obj_t *timeLbl = lv_label_create(notifItem);
        unsigned long ago = (millis() - notifications[realIdx].timestamp) / 1000;
        char timeBuf[16];
        if (ago < 60) snprintf(timeBuf, sizeof(timeBuf), "%lus", ago);
        else if (ago < 3600) snprintf(timeBuf, sizeof(timeBuf), "%lum", ago / 60);
        else snprintf(timeBuf, sizeof(timeBuf), "%luh", ago / 3600);
        lv_label_set_text(timeLbl, timeBuf);
        lv_obj_set_style_text_color(timeLbl, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(timeLbl, &lv_font_montserrat_12, 0);
        lv_obj_align(timeLbl, LV_ALIGN_TOP_RIGHT, -10, 3);

        // Title
        lv_obj_t *titleLbl = lv_label_create(notifItem);
        String titleTrunc = notifications[realIdx].title;
        if (titleTrunc.length() > 28) titleTrunc = titleTrunc.substring(0, 28) + "...";
        lv_label_set_text(titleLbl, titleTrunc.c_str());
        lv_obj_set_style_text_color(titleLbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(titleLbl, &lv_font_montserrat_12, 0);
        lv_obj_align(titleLbl, LV_ALIGN_TOP_LEFT, 10, 17);

        // Body preview
        lv_obj_t *bodyLbl = lv_label_create(notifItem);
        String bodyTrunc = notifications[realIdx].body;
        if (bodyTrunc.length() > 38) bodyTrunc = bodyTrunc.substring(0, 38) + "...";
        lv_label_set_text(bodyLbl, bodyTrunc.c_str());
        lv_obj_set_style_text_color(bodyLbl, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(bodyLbl, &lv_font_montserrat_12, 0);
        lv_obj_align(bodyLbl, LV_ALIGN_TOP_LEFT, 10, 32);

        displayedCount++;
    }

    // Empty state
    if (filteredCount == 0) {
        lv_obj_t *emptyLabel = lv_label_create(card);
        if (notificationCount == 0) {
            lv_label_set_text(emptyLabel, "No notifications\nConnect via Gadgetbridge");
        } else {
            lv_label_set_text(emptyLabel, "No notifications\nfrom this app");
        }
        lv_obj_set_style_text_color(emptyLabel, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(emptyLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(emptyLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(emptyLabel, LV_ALIGN_CENTER, 0, 30);
    }

    // Scroll hint
    if (filteredCount > maxVisible) {
        lv_obj_t *scrollHint = lv_label_create(card);
        char scrollBuf[30];
        snprintf(scrollBuf, sizeof(scrollBuf), LV_SYMBOL_DOWN " %d more", filteredCount - maxVisible - notifScrollOffset);
        lv_label_set_text(scrollHint, scrollBuf);
        lv_obj_set_style_text_color(scrollHint, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(scrollHint, &lv_font_montserrat_12, 0);
        lv_obj_align(scrollHint, LV_ALIGN_BOTTOM_MID, 0, -8);
    }
}

void handleNotificationsTap(int x, int y) {
    // ═══ FULL VIEW - Back button ═══
    if (selectedNotifIndex >= 0) {
        if (y < 45) {
            // Back button tapped
            selectedNotifIndex = -1;
            navigateTo(currentCategory, currentSubCard);
            return;
        }
        return;  // In full view, only back button works
    }

    // ═══ LIST VIEW ═══
    // Filter button area (32-68) - tap LEFT or RIGHT side to change filter
    if (y >= 32 && y < 68) {
        if (notifAppFilterCount <= 1) return;  // No apps to filter
        
        if (x < LCD_WIDTH / 2) {
            // LEFT tap - go to previous filter
            notificationFilter--;
            if (notificationFilter < 0) notificationFilter = notifAppFilterCount - 1;
        } else {
            // RIGHT tap - go to next filter
            notificationFilter++;
            if (notificationFilter >= notifAppFilterCount) notificationFilter = 0;
        }
        notifScrollOffset = 0;  // Reset scroll when changing filter
        USBSerial.printf("[NOTIF] Filter: %s\n", notifAppFilters[notificationFilter].c_str());
        navigateTo(currentCategory, currentSubCard);
        return;
    }
    
    // Notification list area (124+)
    if (y >= 124) {
        int itemHeight = 52;
        int tappedPos = (y - 124) / itemHeight;
        int filteredIdx = tappedPos + notifScrollOffset;
        int filteredCount = getFilteredNotifCount();
        
        // Scroll down if tapping bottom area
        if (y > LCD_HEIGHT - 40 && filteredCount > 3) {
            notifScrollOffset = min(notifScrollOffset + 1, max(0, filteredCount - 3));
            navigateTo(currentCategory, currentSubCard);
            return;
        }
        
        // Tap notification to view full details
        if (filteredIdx >= 0 && filteredIdx < filteredCount) {
            int realIdx = getFilteredNotifIndex(filteredIdx);
            if (realIdx >= 0) {
                selectedNotifIndex = realIdx;
                USBSerial.printf("[NOTIF] Viewing notification %d\n", realIdx);
                navigateTo(currentCategory, currentSubCard);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY CARD - PREMIUM DESIGN (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void createBatteryCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "BATTERY");
    lv_obj_set_style_text_color(title, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 16, 12);

    // Large percentage
    char percBuf[8];
    snprintf(percBuf, sizeof(percBuf), "%d%%", batteryPercent);
    lv_obj_t *percLbl = lv_label_create(card);
    lv_label_set_text(percLbl, percBuf);
    lv_obj_set_style_text_color(percLbl, batteryPercent > 20 ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_text_font(percLbl, &lv_font_montserrat_48, 0);
    lv_obj_align(percLbl, LV_ALIGN_TOP_MID, 0, 40);

    // Charging/estimate status
    lv_obj_t *statusLbl = lv_label_create(card);
    if (isCharging) {
        lv_label_set_text(statusLbl, LV_SYMBOL_CHARGE " Charging");
        lv_obj_set_style_text_color(statusLbl, lv_color_hex(0x30D158), 0);
    } else {
        char estBuf[32];
        uint32_t hrs = batteryStats.combinedEstimateMins / 60;
        uint32_t mins = batteryStats.combinedEstimateMins % 60;
        snprintf(estBuf, sizeof(estBuf), "~%luh %lum remaining", hrs, mins);
        lv_label_set_text(statusLbl, estBuf);
        lv_obj_set_style_text_color(statusLbl, lv_color_hex(0x8E8E93), 0);
    }
    lv_obj_set_style_text_font(statusLbl, &lv_font_montserrat_14, 0);
    lv_obj_align(statusLbl, LV_ALIGN_TOP_MID, 0, 100);

    // Voltage
    char voltBuf[16];
    snprintf(voltBuf, sizeof(voltBuf), "%dmV", batteryVoltage);
    lv_obj_t *voltLbl = lv_label_create(card);
    lv_label_set_text(voltLbl, voltBuf);
    lv_obj_set_style_text_color(voltLbl, lv_color_hex(0x636366), 0);
    lv_obj_align(voltLbl, LV_ALIGN_TOP_MID, 0, 125);

    // Battery saver toggle row
    lv_obj_t *saverRow = lv_obj_create(card);
    lv_obj_set_size(saverRow, LCD_WIDTH - 40, 50);
    lv_obj_align(saverRow, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(saverRow, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(saverRow, 14, 0);
    lv_obj_set_style_border_width(saverRow, 0, 0);
    disableAllScrolling(saverRow);

    lv_obj_t *saverLbl = lv_label_create(saverRow);
    lv_label_set_text(saverLbl, "Battery Saver");
    lv_obj_set_style_text_color(saverLbl, theme->text, 0);
    lv_obj_align(saverLbl, LV_ALIGN_LEFT_MID, 15, 0);

    lv_obj_t *saverStatus = lv_label_create(saverRow);
    lv_label_set_text(saverStatus, batterySaverMode ? "ON" : "OFF");
    lv_obj_set_style_text_color(saverStatus, batterySaverMode ? lv_color_hex(0xFF9F0A) : lv_color_hex(0x636366), 0);
    lv_obj_align(saverStatus, LV_ALIGN_RIGHT_MID, -15, 0);

    // System info row
    lv_obj_t *infoRow = lv_obj_create(card);
    lv_obj_set_size(infoRow, LCD_WIDTH - 40, 80);
    lv_obj_align(infoRow, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(infoRow, lv_color_hex(0x2C2C2E), 0);
    lv_obj_set_style_radius(infoRow, 14, 0);
    lv_obj_set_style_border_width(infoRow, 0, 0);
    disableAllScrolling(infoRow);

    // WiFi status
    char wifiBuf[48];
    if (wifiConnected) {
        snprintf(wifiBuf, sizeof(wifiBuf), "WiFi: %s", wifiNetworks[0].ssid);
    } else {
        snprintf(wifiBuf, sizeof(wifiBuf), "WiFi: Disconnected");
    }
    lv_obj_t *wifiLbl = lv_label_create(infoRow);
    lv_label_set_text(wifiLbl, wifiBuf);
    lv_obj_set_style_text_color(wifiLbl, wifiConnected ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_text_font(wifiLbl, &lv_font_montserrat_12, 0);
    lv_obj_align(wifiLbl, LV_ALIGN_TOP_LEFT, 15, 12);

    // SD status
    char sdBuf[32];
    snprintf(sdBuf, sizeof(sdBuf), "SD: %s", hasSD ? "Mounted" : "Not Found");
    lv_obj_t *sdLbl = lv_label_create(infoRow);
    lv_label_set_text(sdLbl, sdBuf);
    lv_obj_set_style_text_color(sdLbl, hasSD ? lv_color_hex(0x30D158) : lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(sdLbl, &lv_font_montserrat_12, 0);
    lv_obj_align(sdLbl, LV_ALIGN_TOP_LEFT, 15, 32);

    // Free RAM
    char ramBuf[32];
    snprintf(ramBuf, sizeof(ramBuf), "Free RAM: %luKB", freeRAM / 1024);
    lv_obj_t *ramLbl = lv_label_create(infoRow);
    lv_label_set_text(ramLbl, ramBuf);
    lv_obj_set_style_text_color(ramLbl, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(ramLbl, &lv_font_montserrat_12, 0);
    lv_obj_align(ramLbl, LV_ALIGN_TOP_LEFT, 15, 52);
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY STATS CARD - PREMIUM DESIGN
// ═══════════════════════════════════════════════════════════════════════════
void createBatteryStatsCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with accent
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "BATTERY STATS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    unsigned long screenOnMins = batteryStats.screenOnTimeMs / 60000;
    unsigned long screenOffMins = batteryStats.screenOffTimeMs / 60000;
    
    // Screen On Time Card
    lv_obj_t *onCard = lv_obj_create(card);
    lv_obj_set_size(onCard, LCD_WIDTH - 40, 70);
    lv_obj_align(onCard, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_color(onCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(onCard, 15, 0);
    lv_obj_set_style_border_width(onCard, 0, 0);
    disableAllScrolling(onCard);

    lv_obj_t *onIcon = lv_label_create(onCard);
    lv_label_set_text(onIcon, LV_SYMBOL_EYE_OPEN);
    lv_obj_set_style_text_color(onIcon, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(onIcon, &lv_font_montserrat_24, 0);
    lv_obj_align(onIcon, LV_ALIGN_LEFT_MID, 15, 0);

    lv_obj_t *onLabel = lv_label_create(onCard);
    lv_label_set_text(onLabel, "Screen On");
    lv_obj_set_style_text_color(onLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(onLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(onLabel, LV_ALIGN_LEFT_MID, 55, -12);

    char onBuf[16];
    snprintf(onBuf, sizeof(onBuf), "%lu min", screenOnMins);
    lv_obj_t *onValue = lv_label_create(onCard);
    lv_label_set_text(onValue, onBuf);
    lv_obj_set_style_text_color(onValue, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(onValue, &lv_font_montserrat_20, 0);
    lv_obj_align(onValue, LV_ALIGN_LEFT_MID, 55, 12);

    // Screen Off Time Card  
    lv_obj_t *offCard = lv_obj_create(card);
    lv_obj_set_size(offCard, LCD_WIDTH - 40, 70);
    lv_obj_align(offCard, LV_ALIGN_TOP_MID, 0, 130);
    lv_obj_set_style_bg_color(offCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(offCard, 15, 0);
    lv_obj_set_style_border_width(offCard, 0, 0);
    disableAllScrolling(offCard);

    lv_obj_t *offIcon = lv_label_create(offCard);
    lv_label_set_text(offIcon, LV_SYMBOL_EYE_CLOSE);
    lv_obj_set_style_text_color(offIcon, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(offIcon, &lv_font_montserrat_24, 0);
    lv_obj_align(offIcon, LV_ALIGN_LEFT_MID, 15, 0);

    lv_obj_t *offLabel = lv_label_create(offCard);
    lv_label_set_text(offLabel, "Screen Off");
    lv_obj_set_style_text_color(offLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(offLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(offLabel, LV_ALIGN_LEFT_MID, 55, -12);

    char offBuf[16];
    snprintf(offBuf, sizeof(offBuf), "%lu min", screenOffMins);
    lv_obj_t *offValue = lv_label_create(offCard);
    lv_label_set_text(offValue, offBuf);
    lv_obj_set_style_text_color(offValue, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(offValue, &lv_font_montserrat_20, 0);
    lv_obj_align(offValue, LV_ALIGN_LEFT_MID, 55, 12);

    // Voltage & Estimate Card
    lv_obj_t *voltCard = lv_obj_create(card);
    lv_obj_set_size(voltCard, LCD_WIDTH - 40, 100);
    lv_obj_align(voltCard, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(voltCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(voltCard, 15, 0);
    lv_obj_set_style_border_width(voltCard, 0, 0);
    disableAllScrolling(voltCard);

    char voltBuf[32];
    snprintf(voltBuf, sizeof(voltBuf), "%d mV", batteryVoltage);
    lv_obj_t *voltLabel = lv_label_create(voltCard);
    lv_label_set_text(voltLabel, voltBuf);
    lv_obj_set_style_text_color(voltLabel, lv_color_hex(0xFFD60A), 0);
    lv_obj_set_style_text_font(voltLabel, &lv_font_montserrat_24, 0);
    lv_obj_align(voltLabel, LV_ALIGN_TOP_MID, 0, 15);

    char estBuf[48];
    uint32_t hrs = batteryStats.combinedEstimateMins / 60;
    uint32_t mins = batteryStats.combinedEstimateMins % 60;
    snprintf(estBuf, sizeof(estBuf), "~%lu hrs %lu min remaining", hrs, mins);
    lv_obj_t *estLabel = lv_label_create(voltCard);
    lv_label_set_text(estLabel, estBuf);
    lv_obj_set_style_text_color(estLabel, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(estLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(estLabel, LV_ALIGN_BOTTOM_MID, 0, -15);
}

// ═══════════════════════════════════════════════════════════════════════════
// USAGE PATTERNS CARD - PREMIUM SYSTEM STATS DESIGN
// ═══════════════════════════════════════════════════════════════════════════
void createUsagePatternsCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "SYSTEM STATS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // ═══ RAM USAGE GAUGE ═══
    lv_obj_t *ramCard = lv_obj_create(card);
    lv_obj_set_size(ramCard, (LCD_WIDTH - 50) / 2, 120);
    lv_obj_align(ramCard, LV_ALIGN_TOP_LEFT, 15, 50);
    lv_obj_set_style_bg_color(ramCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(ramCard, 15, 0);
    lv_obj_set_style_border_width(ramCard, 0, 0);
    disableAllScrolling(ramCard);

    lv_obj_t *ramTitle = lv_label_create(ramCard);
    lv_label_set_text(ramTitle, "RAM");
    lv_obj_set_style_text_color(ramTitle, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(ramTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(ramTitle, LV_ALIGN_TOP_MID, 0, 8);

    // RAM arc gauge
    uint32_t totalRAM = 512 * 1024;  // 512KB total
    int ramPercent = ((totalRAM - freeRAM) * 100) / totalRAM;
    
    lv_obj_t *ramArc = lv_arc_create(ramCard);
    lv_obj_set_size(ramArc, 70, 70);
    lv_obj_align(ramArc, LV_ALIGN_CENTER, 0, 5);
    lv_arc_set_rotation(ramArc, 135);
    lv_arc_set_bg_angles(ramArc, 0, 270);
    lv_arc_set_range(ramArc, 0, 100);
    lv_arc_set_value(ramArc, ramPercent);
    lv_obj_set_style_arc_width(ramArc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ramArc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ramArc, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ramArc, lv_color_hex(0x30D158), LV_PART_INDICATOR);
    lv_obj_remove_style(ramArc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ramArc, LV_OBJ_FLAG_CLICKABLE);

    char ramBuf[16];
    snprintf(ramBuf, sizeof(ramBuf), "%d%%", ramPercent);
    lv_obj_t *ramPerc = lv_label_create(ramArc);
    lv_label_set_text(ramPerc, ramBuf);
    lv_obj_set_style_text_color(ramPerc, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ramPerc, &lv_font_montserrat_14, 0);
    lv_obj_center(ramPerc);

    char freeRamBuf[16];
    snprintf(freeRamBuf, sizeof(freeRamBuf), "%luKB free", freeRAM / 1024);
    lv_obj_t *ramFree = lv_label_create(ramCard);
    lv_label_set_text(ramFree, freeRamBuf);
    lv_obj_set_style_text_color(ramFree, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(ramFree, &lv_font_montserrat_12, 0);
    lv_obj_align(ramFree, LV_ALIGN_BOTTOM_MID, 0, -5);

    // ═══ CPU USAGE GAUGE ═══
    lv_obj_t *cpuCard = lv_obj_create(card);
    lv_obj_set_size(cpuCard, (LCD_WIDTH - 50) / 2, 120);
    lv_obj_align(cpuCard, LV_ALIGN_TOP_RIGHT, -15, 50);
    lv_obj_set_style_bg_color(cpuCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(cpuCard, 15, 0);
    lv_obj_set_style_border_width(cpuCard, 0, 0);
    disableAllScrolling(cpuCard);

    lv_obj_t *cpuTitle = lv_label_create(cpuCard);
    lv_label_set_text(cpuTitle, "CPU");
    lv_obj_set_style_text_color(cpuTitle, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(cpuTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(cpuTitle, LV_ALIGN_TOP_MID, 0, 8);

    // Simulated CPU load (based on screen state)
    int cpuPercent = screenOn ? random(15, 35) : random(5, 15);
    
    lv_obj_t *cpuArc = lv_arc_create(cpuCard);
    lv_obj_set_size(cpuArc, 70, 70);
    lv_obj_align(cpuArc, LV_ALIGN_CENTER, 0, 5);
    lv_arc_set_rotation(cpuArc, 135);
    lv_arc_set_bg_angles(cpuArc, 0, 270);
    lv_arc_set_range(cpuArc, 0, 100);
    lv_arc_set_value(cpuArc, cpuPercent);
    lv_obj_set_style_arc_width(cpuArc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(cpuArc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(cpuArc, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_arc_color(cpuArc, lv_color_hex(0x0A84FF), LV_PART_INDICATOR);
    lv_obj_remove_style(cpuArc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(cpuArc, LV_OBJ_FLAG_CLICKABLE);

    char cpuBuf[16];
    snprintf(cpuBuf, sizeof(cpuBuf), "%d%%", cpuPercent);
    lv_obj_t *cpuPerc = lv_label_create(cpuArc);
    lv_label_set_text(cpuPerc, cpuBuf);
    lv_obj_set_style_text_color(cpuPerc, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(cpuPerc, &lv_font_montserrat_14, 0);
    lv_obj_center(cpuPerc);

    lv_obj_t *cpuSpeed = lv_label_create(cpuCard);
    lv_label_set_text(cpuSpeed, "240MHz");
    lv_obj_set_style_text_color(cpuSpeed, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(cpuSpeed, &lv_font_montserrat_12, 0);
    lv_obj_align(cpuSpeed, LV_ALIGN_BOTTOM_MID, 0, -5);

    // ═══ SESSION INFO ═══
    lv_obj_t *sessionCard = lv_obj_create(card);
    lv_obj_set_size(sessionCard, LCD_WIDTH - 30, 70);
    lv_obj_align(sessionCard, LV_ALIGN_TOP_MID, 0, 180);
    lv_obj_set_style_bg_color(sessionCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(sessionCard, 15, 0);
    lv_obj_set_style_border_width(sessionCard, 0, 0);
    disableAllScrolling(sessionCard);

    unsigned long sessionMins = (millis() - batteryStats.sessionStartMs) / 60000;
    char sessionBuf[32];
    snprintf(sessionBuf, sizeof(sessionBuf), "Session: %lu min", sessionMins);
    lv_obj_t *sessionLabel = lv_label_create(sessionCard);
    lv_label_set_text(sessionLabel, sessionBuf);
    lv_obj_set_style_text_color(sessionLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(sessionLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(sessionLabel, LV_ALIGN_TOP_MID, 0, 12);

    char drainBuf[32];
    snprintf(drainBuf, sizeof(drainBuf), "Drain: %.1f%%/hr", batteryStats.avgDrainPerHour);
    lv_obj_t *drainLabel = lv_label_create(sessionCard);
    lv_label_set_text(drainLabel, drainBuf);
    lv_obj_set_style_text_color(drainLabel, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(drainLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(drainLabel, LV_ALIGN_BOTTOM_MID, 0, -12);

    // ═══ MEMORY BREAKDOWN ═══
    lv_obj_t *memCard = lv_obj_create(card);
    lv_obj_set_size(memCard, LCD_WIDTH - 30, 80);
    lv_obj_align(memCard, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(memCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(memCard, 15, 0);
    lv_obj_set_style_border_width(memCard, 0, 0);
    disableAllScrolling(memCard);

    lv_obj_t *memTitle = lv_label_create(memCard);
    lv_label_set_text(memTitle, "MEMORY BREAKDOWN");
    lv_obj_set_style_text_color(memTitle, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(memTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(memTitle, LV_ALIGN_TOP_MID, 0, 8);

    // Progress bar showing memory usage
    lv_obj_t *memBar = lv_bar_create(memCard);
    lv_obj_set_size(memBar, LCD_WIDTH - 70, 12);
    lv_obj_align(memBar, LV_ALIGN_CENTER, 0, 5);
    lv_bar_set_range(memBar, 0, 100);
    lv_bar_set_value(memBar, ramPercent, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(memBar, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(memBar, lv_color_hex(0x5E5CE6), LV_PART_INDICATOR);
    lv_obj_set_style_radius(memBar, 6, LV_PART_MAIN);
    lv_obj_set_style_radius(memBar, 6, LV_PART_INDICATOR);

    char memInfoBuf[48];
    snprintf(memInfoBuf, sizeof(memInfoBuf), "Used: %luKB | Total: %luKB", (totalRAM - freeRAM) / 1024, totalRAM / 1024);
    lv_obj_t *memInfo = lv_label_create(memCard);
    lv_label_set_text(memInfo, memInfoBuf);
    lv_obj_set_style_text_color(memInfo, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(memInfo, &lv_font_montserrat_12, 0);
    lv_obj_align(memInfo, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ═══════════════════════════════════════════════════════════════════════════
// SD CARD HEALTH CARD (ENHANCED FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void updateSDCardHealth() {
    if (hasSD) {
        sdHealth.totalBytes = SD_MMC.totalBytes();
        sdHealth.usedBytes = SD_MMC.usedBytes();
        sdHealth.freeBytes = sdHealth.totalBytes - sdHealth.usedBytes;
        sdHealth.usedPercent = (sdHealth.usedBytes * 100.0) / sdHealth.totalBytes;
        sdHealth.mounted = true;
        sdHealth.healthy = (sdHealth.freeBytes > 1024 * 1024);
    } else {
        sdHealth.mounted = false;
        sdHealth.healthy = false;
    }
}

void createSDCardHealthCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "SD CARD");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    updateSDCardHealth();

    if (sdHealth.mounted) {
        // Status badge
        lv_obj_t *statusBadge = lv_obj_create(card);
        lv_obj_set_size(statusBadge, 100, 28);
        lv_obj_align(statusBadge, LV_ALIGN_TOP_MID, 0, 45);
        lv_obj_set_style_bg_color(statusBadge, sdHealth.healthy ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_radius(statusBadge, 14, 0);
        lv_obj_set_style_border_width(statusBadge, 0, 0);
        disableAllScrolling(statusBadge);

        lv_obj_t *statusText = lv_label_create(statusBadge);
        lv_label_set_text(statusText, sdHealth.healthy ? "HEALTHY" : "LOW SPACE");
        lv_obj_set_style_text_color(statusText, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(statusText, &lv_font_montserrat_12, 0);
        lv_obj_center(statusText);

        // Storage gauge
        lv_obj_t *storageCard = lv_obj_create(card);
        lv_obj_set_size(storageCard, LCD_WIDTH - 40, 110);
        lv_obj_align(storageCard, LV_ALIGN_TOP_MID, 0, 85);
        lv_obj_set_style_bg_color(storageCard, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(storageCard, 15, 0);
        lv_obj_set_style_border_width(storageCard, 0, 0);
        disableAllScrolling(storageCard);

        // Arc gauge for storage
        lv_obj_t *storageArc = lv_arc_create(storageCard);
        lv_obj_set_size(storageArc, 80, 80);
        lv_obj_align(storageArc, LV_ALIGN_LEFT_MID, 15, 0);
        lv_arc_set_rotation(storageArc, 135);
        lv_arc_set_bg_angles(storageArc, 0, 270);
        lv_arc_set_range(storageArc, 0, 100);
        lv_arc_set_value(storageArc, (int)sdHealth.usedPercent);
        lv_obj_set_style_arc_width(storageArc, 10, LV_PART_MAIN);
        lv_obj_set_style_arc_width(storageArc, 10, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(storageArc, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
        lv_obj_set_style_arc_color(storageArc, lv_color_hex(0xFF9F0A), LV_PART_INDICATOR);
        lv_obj_remove_style(storageArc, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(storageArc, LV_OBJ_FLAG_CLICKABLE);

        char percBuf[8];
        snprintf(percBuf, sizeof(percBuf), "%.0f%%", sdHealth.usedPercent);
        lv_obj_t *percLabel = lv_label_create(storageArc);
        lv_label_set_text(percLabel, percBuf);
        lv_obj_set_style_text_color(percLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(percLabel, &lv_font_montserrat_16, 0);
        lv_obj_center(percLabel);

        // Storage details
        char totalBuf[24];
        snprintf(totalBuf, sizeof(totalBuf), "Total: %llu MB", sdHealth.totalBytes / (1024 * 1024));
        lv_obj_t *totalLabel = lv_label_create(storageCard);
        lv_label_set_text(totalLabel, totalBuf);
        lv_obj_set_style_text_color(totalLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(totalLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(totalLabel, LV_ALIGN_RIGHT_MID, -15, -25);

        char usedBuf[24];
        snprintf(usedBuf, sizeof(usedBuf), "Used: %llu MB", sdHealth.usedBytes / (1024 * 1024));
        lv_obj_t *usedLabel = lv_label_create(storageCard);
        lv_label_set_text(usedLabel, usedBuf);
        lv_obj_set_style_text_color(usedLabel, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(usedLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(usedLabel, LV_ALIGN_RIGHT_MID, -15, 0);

        char freeBuf[24];
        snprintf(freeBuf, sizeof(freeBuf), "Free: %llu MB", sdHealth.freeBytes / (1024 * 1024));
        lv_obj_t *freeLabel = lv_label_create(storageCard);
        lv_label_set_text(freeLabel, freeBuf);
        lv_obj_set_style_text_color(freeLabel, lv_color_hex(0x30D158), 0);
        lv_obj_set_style_text_font(freeLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(freeLabel, LV_ALIGN_RIGHT_MID, -15, 25);

        // Backup info card
        lv_obj_t *backupCard = lv_obj_create(card);
        lv_obj_set_size(backupCard, LCD_WIDTH - 40, 70);
        lv_obj_align(backupCard, LV_ALIGN_TOP_MID, 0, 205);
        lv_obj_set_style_bg_color(backupCard, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(backupCard, 15, 0);
        lv_obj_set_style_border_width(backupCard, 0, 0);
        disableAllScrolling(backupCard);

        lv_obj_t *backupTitle = lv_label_create(backupCard);
        lv_label_set_text(backupTitle, "AUTO BACKUP");
        lv_obj_set_style_text_color(backupTitle, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(backupTitle, &lv_font_montserrat_12, 0);
        lv_obj_align(backupTitle, LV_ALIGN_TOP_LEFT, 15, 10);

        lv_obj_t *backupStatus = lv_label_create(backupCard);
        lv_label_set_text(backupStatus, autoBackupEnabled ? "ENABLED" : "DISABLED");
        lv_obj_set_style_text_color(backupStatus, autoBackupEnabled ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_text_font(backupStatus, &lv_font_montserrat_14, 0);
        lv_obj_align(backupStatus, LV_ALIGN_LEFT_MID, 15, 8);

        char countBuf[24];
        snprintf(countBuf, sizeof(countBuf), "Backups: %d", totalBackups);
        lv_obj_t *countLabel = lv_label_create(backupCard);
        lv_label_set_text(countLabel, countBuf);
        lv_obj_set_style_text_color(countLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(countLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(countLabel, LV_ALIGN_RIGHT_MID, -15, 0);

        // Card type info
        lv_obj_t *typeCard = lv_obj_create(card);
        lv_obj_set_size(typeCard, LCD_WIDTH - 40, 45);
        lv_obj_align(typeCard, LV_ALIGN_BOTTOM_MID, 0, -20);
        lv_obj_set_style_bg_color(typeCard, lv_color_hex(0x1C1C1E), 0);
        lv_obj_set_style_radius(typeCard, 12, 0);
        lv_obj_set_style_border_width(typeCard, 0, 0);
        disableAllScrolling(typeCard);

        char typeBuf[32];
        snprintf(typeBuf, sizeof(typeBuf), "Type: %s | %llu MB", sdCardType.c_str(), sdCardSizeMB);
        lv_obj_t *typeLabel = lv_label_create(typeCard);
        lv_label_set_text(typeLabel, typeBuf);
        lv_obj_set_style_text_color(typeLabel, lv_color_hex(0x636366), 0);
        lv_obj_set_style_text_font(typeLabel, &lv_font_montserrat_12, 0);
        lv_obj_center(typeLabel);

    } else {
        // No SD card
        lv_obj_t *noSDIcon = lv_label_create(card);
        lv_label_set_text(noSDIcon, LV_SYMBOL_SD_CARD);
        lv_obj_set_style_text_color(noSDIcon, lv_color_hex(0xFF453A), 0);
        lv_obj_set_style_text_font(noSDIcon, &lv_font_montserrat_48, 0);
        lv_obj_align(noSDIcon, LV_ALIGN_CENTER, 0, -30);

        lv_obj_t *noSDLabel = lv_label_create(card);
        lv_label_set_text(noSDLabel, "NO SD CARD");
        lv_obj_set_style_text_color(noSDLabel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(noSDLabel, &lv_font_montserrat_20, 0);
        lv_obj_align(noSDLabel, LV_ALIGN_CENTER, 0, 30);

        lv_obj_t *insertLabel = lv_label_create(card);
        lv_label_set_text(insertLabel, "Insert SD card to enable\nbackups and storage");
        lv_obj_set_style_text_color(insertLabel, lv_color_hex(0x8E8E93), 0);
        lv_obj_set_style_text_font(insertLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_align(insertLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(insertLabel, LV_ALIGN_BOTTOM_MID, 0, -60);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// POWER STATS CARD - DISPLAYS LOGGED POWER CONSUMPTION DATA
// ═══════════════════════════════════════════════════════════════════════════
void createPowerStatsCard() {
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0D0D0D), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with power icon
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_CHARGE " POWER STATS");
    lv_obj_set_style_text_color(title, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    // Calculate current session stats
    uint32_t sessionMs = millis() - currentPowerSession.sessionStartMs;
    float sessionHours = sessionMs / 3600000.0;
    float drainPercent = currentPowerSession.startBatteryPercent - batteryPercent;
    float drainRatePerHour = (sessionHours > 0.1) ? (drainPercent / sessionHours) : 0;
    float remainingHours = (drainRatePerHour > 0.5) ? (batteryPercent / drainRatePerHour) : 99;

    // Current drain rate gauge
    lv_obj_t *gaugeCard = lv_obj_create(card);
    lv_obj_set_size(gaugeCard, LCD_WIDTH - 30, 95);
    lv_obj_align(gaugeCard, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_bg_color(gaugeCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(gaugeCard, 15, 0);
    lv_obj_set_style_border_width(gaugeCard, 0, 0);
    disableAllScrolling(gaugeCard);

    // Drain rate arc
    lv_obj_t *drainArc = lv_arc_create(gaugeCard);
    lv_obj_set_size(drainArc, 70, 70);
    lv_obj_align(drainArc, LV_ALIGN_LEFT_MID, 10, 0);
    lv_arc_set_rotation(drainArc, 135);
    lv_arc_set_bg_angles(drainArc, 0, 270);
    lv_arc_set_range(drainArc, 0, 50);  // 0-50%/hour range
    int drainVal = (drainRatePerHour > 50) ? 50 : (int)drainRatePerHour;
    lv_arc_set_value(drainArc, drainVal);
    lv_obj_set_style_arc_width(drainArc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(drainArc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(drainArc, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    
    // Color based on drain rate (green = good, orange = moderate, red = high)
    uint32_t drainColor = (drainRatePerHour < 10) ? 0x30D158 : 
                          (drainRatePerHour < 20) ? 0xFF9F0A : 0xFF453A;
    lv_obj_set_style_arc_color(drainArc, lv_color_hex(drainColor), LV_PART_INDICATOR);
    lv_obj_remove_style(drainArc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(drainArc, LV_OBJ_FLAG_CLICKABLE);

    // Drain rate value in center
    char drainBuf[16];
    snprintf(drainBuf, sizeof(drainBuf), "%.1f%%", drainRatePerHour);
    lv_obj_t *drainLabel = lv_label_create(drainArc);
    lv_label_set_text(drainLabel, drainBuf);
    lv_obj_set_style_text_color(drainLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(drainLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(drainLabel);

    lv_obj_t *perHrLabel = lv_label_create(drainArc);
    lv_label_set_text(perHrLabel, "/hr");
    lv_obj_set_style_text_color(perHrLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(perHrLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(perHrLabel, LV_ALIGN_BOTTOM_MID, 0, -5);

    // Stats on the right
    lv_obj_t *statsTitle = lv_label_create(gaugeCard);
    lv_label_set_text(statsTitle, "CURRENT SESSION");
    lv_obj_set_style_text_color(statsTitle, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(statsTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(statsTitle, LV_ALIGN_TOP_RIGHT, -10, 8);

    char sessionBuf[24];
    snprintf(sessionBuf, sizeof(sessionBuf), "Runtime: %.1fh", sessionHours);
    lv_obj_t *sessionLabel = lv_label_create(gaugeCard);
    lv_label_set_text(sessionLabel, sessionBuf);
    lv_obj_set_style_text_color(sessionLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(sessionLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(sessionLabel, LV_ALIGN_TOP_RIGHT, -10, 28);

    char usedBuf[24];
    snprintf(usedBuf, sizeof(usedBuf), "Used: %.1f%%", drainPercent);
    lv_obj_t *usedLabel = lv_label_create(gaugeCard);
    lv_label_set_text(usedLabel, usedBuf);
    lv_obj_set_style_text_color(usedLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(usedLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(usedLabel, LV_ALIGN_TOP_RIGHT, -10, 48);

    char estBuf[24];
    if (remainingHours > 48) {
        snprintf(estBuf, sizeof(estBuf), "Est: 48h+");
    } else {
        snprintf(estBuf, sizeof(estBuf), "Est: %.1fh left", remainingHours);
    }
    lv_obj_t *estLabel = lv_label_create(gaugeCard);
    lv_label_set_text(estLabel, estBuf);
    lv_obj_set_style_text_color(estLabel, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(estLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(estLabel, LV_ALIGN_TOP_RIGHT, -10, 68);

    // Mode & logging status card
    lv_obj_t *modeCard = lv_obj_create(card);
    lv_obj_set_size(modeCard, LCD_WIDTH - 30, 55);
    lv_obj_align(modeCard, LV_ALIGN_TOP_MID, 0, 145);
    lv_obj_set_style_bg_color(modeCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(modeCard, 12, 0);
    lv_obj_set_style_border_width(modeCard, 0, 0);
    disableAllScrolling(modeCard);

    lv_obj_t *modeTitle = lv_label_create(modeCard);
    lv_label_set_text(modeTitle, "Power Saver Mode:");
    lv_obj_set_style_text_color(modeTitle, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(modeTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(modeTitle, LV_ALIGN_TOP_LEFT, 12, 8);

    lv_obj_t *modeVal = lv_label_create(modeCard);
    lv_label_set_text(modeVal, saverModes[batterySaverLevel].name);
    lv_obj_set_style_text_color(modeVal, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_text_font(modeVal, &lv_font_montserrat_16, 0);
    lv_obj_align(modeVal, LV_ALIGN_LEFT_MID, 12, 8);

    // CPU frequency
    char cpuBuf[16];
    snprintf(cpuBuf, sizeof(cpuBuf), "%dMHz", getCpuFrequencyMhz());
    lv_obj_t *cpuLabel = lv_label_create(modeCard);
    lv_label_set_text(cpuLabel, cpuBuf);
    lv_obj_set_style_text_color(cpuLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(cpuLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(cpuLabel, LV_ALIGN_RIGHT_MID, -12, 0);

    // Logging status card
    lv_obj_t *logCard = lv_obj_create(card);
    lv_obj_set_size(logCard, LCD_WIDTH - 30, 55);
    lv_obj_align(logCard, LV_ALIGN_TOP_MID, 0, 210);
    lv_obj_set_style_bg_color(logCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(logCard, 12, 0);
    lv_obj_set_style_border_width(logCard, 0, 0);
    disableAllScrolling(logCard);

    lv_obj_t *logIcon = lv_label_create(logCard);
    lv_label_set_text(logIcon, LV_SYMBOL_FILE);
    lv_obj_set_style_text_color(logIcon, powerLoggingEnabled && hasSD ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_text_font(logIcon, &lv_font_montserrat_20, 0);
    lv_obj_align(logIcon, LV_ALIGN_LEFT_MID, 12, 0);

    lv_obj_t *logTitle = lv_label_create(logCard);
    lv_label_set_text(logTitle, "SD Card Logging");
    lv_obj_set_style_text_color(logTitle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(logTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(logTitle, LV_ALIGN_LEFT_MID, 45, -10);

    lv_obj_t *logStatus = lv_label_create(logCard);
    if (!hasSD) {
        lv_label_set_text(logStatus, "No SD Card");
        lv_obj_set_style_text_color(logStatus, lv_color_hex(0xFF453A), 0);
    } else if (powerLoggingEnabled) {
        lv_label_set_text(logStatus, "Active - /WATCH/POWER_LOGS/");
        lv_obj_set_style_text_color(logStatus, lv_color_hex(0x30D158), 0);
    } else {
        lv_label_set_text(logStatus, "Disabled");
        lv_obj_set_style_text_color(logStatus, lv_color_hex(0x8E8E93), 0);
    }
    lv_obj_set_style_text_font(logStatus, &lv_font_montserrat_12, 0);
    lv_obj_align(logStatus, LV_ALIGN_LEFT_MID, 45, 8);

    // Mode changes count
    char changesBuf[24];
    snprintf(changesBuf, sizeof(changesBuf), "%lu changes", currentPowerSession.modeChanges);
    lv_obj_t *changesLabel = lv_label_create(logCard);
    lv_label_set_text(changesLabel, changesBuf);
    lv_obj_set_style_text_color(changesLabel, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(changesLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(changesLabel, LV_ALIGN_RIGHT_MID, -12, 0);

    // Efficiency rating bar
    lv_obj_t *effCard = lv_obj_create(card);
    lv_obj_set_size(effCard, LCD_WIDTH - 30, 40);
    lv_obj_align(effCard, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_bg_color(effCard, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_radius(effCard, 10, 0);
    lv_obj_set_style_border_width(effCard, 0, 0);
    disableAllScrolling(effCard);

    const char* effRating;
    uint32_t effColor;
    if (drainRatePerHour < 5) { effRating = "EXCELLENT"; effColor = 0x30D158; }
    else if (drainRatePerHour < 10) { effRating = "GOOD"; effColor = 0x30D158; }
    else if (drainRatePerHour < 20) { effRating = "MODERATE"; effColor = 0xFF9F0A; }
    else if (drainRatePerHour < 30) { effRating = "HIGH USAGE"; effColor = 0xFF9F0A; }
    else { effRating = "VERY HIGH"; effColor = 0xFF453A; }

    lv_obj_t *effLabel = lv_label_create(effCard);
    char effBuf[32];
    snprintf(effBuf, sizeof(effBuf), "Efficiency: %s", effRating);
    lv_label_set_text(effLabel, effBuf);
    lv_obj_set_style_text_color(effLabel, lv_color_hex(effColor), 0);
    lv_obj_set_style_text_font(effLabel, &lv_font_montserrat_12, 0);
    lv_obj_center(effLabel);
}

// ═══════════════════════════════════════════════════════════════════════════
// ABOUT/SOFTWARE INFO CARD - WIGET OS INFO
// ═══════════════════════════════════════════════════════════════════════════
void createAboutCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    // Deep dark background with subtle gradient
    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0F), 0);
    lv_obj_set_style_bg_grad_color(card, lv_color_hex(0x1A1A2E), 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Ambient glow effect at top (gradient accent)
    lv_obj_t *glowTop = lv_obj_create(card);
    lv_obj_set_size(glowTop, LCD_WIDTH, 120);
    lv_obj_align(glowTop, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(glowTop, theme->accent, 0);
    lv_obj_set_style_bg_opa(glowTop, LV_OPA_10, 0);
    lv_obj_set_style_radius(glowTop, 0, 0);
    lv_obj_set_style_border_width(glowTop, 0, 0);
    disableAllScrolling(glowTop);

    // Main glass-morphism card container
    lv_obj_t *glassCard = lv_obj_create(card);
    lv_obj_set_size(glassCard, LCD_WIDTH - 28, LCD_HEIGHT - 50);
    lv_obj_align(glassCard, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_bg_color(glassCard, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(glassCard, LV_OPA_10, 0);  // Glass transparency
    lv_obj_set_style_radius(glassCard, 28, 0);
    lv_obj_set_style_border_width(glassCard, 1, 0);
    lv_obj_set_style_border_color(glassCard, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(glassCard, LV_OPA_20, 0);
    lv_obj_set_style_shadow_width(glassCard, 40, 0);
    lv_obj_set_style_shadow_color(glassCard, theme->accent, 0);
    lv_obj_set_style_shadow_opa(glassCard, LV_OPA_20, 0);
    lv_obj_set_style_shadow_spread(glassCard, 2, 0);
    disableAllScrolling(glassCard);

    // Inner glass highlight (top edge glow)
    lv_obj_t *innerGlow = lv_obj_create(glassCard);
    lv_obj_set_size(innerGlow, LCD_WIDTH - 60, 3);
    lv_obj_align(innerGlow, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_bg_color(innerGlow, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(innerGlow, LV_OPA_30, 0);
    lv_obj_set_style_radius(innerGlow, 2, 0);
    lv_obj_set_style_border_width(innerGlow, 0, 0);
    disableAllScrolling(innerGlow);

    // Logo/Icon circle with gradient
    lv_obj_t *logoCircle = lv_obj_create(glassCard);
    lv_obj_set_size(logoCircle, 70, 70);
    lv_obj_align(logoCircle, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_style_bg_color(logoCircle, theme->accent, 0);
    lv_obj_set_style_bg_grad_color(logoCircle, theme->secondary, 0);
    lv_obj_set_style_bg_grad_dir(logoCircle, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_radius(logoCircle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(logoCircle, 2, 0);
    lv_obj_set_style_border_color(logoCircle, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_opa(logoCircle, LV_OPA_40, 0);
    lv_obj_set_style_shadow_width(logoCircle, 20, 0);
    lv_obj_set_style_shadow_color(logoCircle, theme->accent, 0);
    lv_obj_set_style_shadow_opa(logoCircle, LV_OPA_50, 0);
    disableAllScrolling(logoCircle);

    // Logo letter "W" for Widget
    lv_obj_t *logoLetter = lv_label_create(logoCircle);
    lv_label_set_text(logoLetter, "W");
    lv_obj_set_style_text_color(logoLetter, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(logoLetter, &lv_font_montserrat_32, 0);
    lv_obj_center(logoLetter);

    // OS Name - WIDGET OS with letter spacing
    lv_obj_t *osNameLabel = lv_label_create(glassCard);
    lv_label_set_text(osNameLabel, "WIDGET OS");
    lv_obj_set_style_text_color(osNameLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(osNameLabel, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_letter_space(osNameLabel, 4, 0);
    lv_obj_align(osNameLabel, LV_ALIGN_TOP_MID, 0, 105);

    // Version badge with gradient accent
    lv_obj_t *versionBadge = lv_obj_create(glassCard);
    lv_obj_set_size(versionBadge, 90, 32);
    lv_obj_align(versionBadge, LV_ALIGN_TOP_MID, 0, 140);
    lv_obj_set_style_bg_color(versionBadge, theme->accent, 0);
    lv_obj_set_style_bg_opa(versionBadge, LV_OPA_80, 0);
    lv_obj_set_style_radius(versionBadge, 16, 0);
    lv_obj_set_style_border_width(versionBadge, 0, 0);
    disableAllScrolling(versionBadge);

    lv_obj_t *versionLabel = lv_label_create(versionBadge);
    lv_label_set_text(versionLabel, "v7.2.0");
    lv_obj_set_style_text_color(versionLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(versionLabel, &lv_font_montserrat_14, 0);
    lv_obj_center(versionLabel);

    // Divider with gradient dots
    int dotY = 185;
    for (int i = 0; i < 5; i++) {
        lv_obj_t *dot = lv_obj_create(glassCard);
        lv_obj_set_size(dot, 4, 4);
        lv_obj_align(dot, LV_ALIGN_TOP_MID, -32 + i * 16, dotY);
        lv_obj_set_style_bg_color(dot, theme->accent, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_40 + i * 15, 0);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        disableAllScrolling(dot);
    }

    // Info rows container
    lv_obj_t *infoContainer = lv_obj_create(glassCard);
    lv_obj_set_size(infoContainer, LCD_WIDTH - 60, 90);
    lv_obj_align(infoContainer, LV_ALIGN_TOP_MID, 0, 200);
    lv_obj_set_style_bg_color(infoContainer, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(infoContainer, LV_OPA_30, 0);
    lv_obj_set_style_radius(infoContainer, 16, 0);
    lv_obj_set_style_border_width(infoContainer, 0, 0);
    disableAllScrolling(infoContainer);

    // Device ID row
    lv_obj_t *deviceLabel = lv_label_create(infoContainer);
    lv_label_set_text(deviceLabel, "Device");
    lv_obj_set_style_text_color(deviceLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(deviceLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(deviceLabel, LV_ALIGN_TOP_LEFT, 15, 12);

    lv_obj_t *deviceValue = lv_label_create(infoContainer);
    lv_label_set_text(deviceValue, DEVICE_ID);
    lv_obj_set_style_text_color(deviceValue, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(deviceValue, &lv_font_montserrat_14, 0);
    lv_obj_align(deviceValue, LV_ALIGN_TOP_RIGHT, -15, 10);

    // Screen row
    lv_obj_t *screenLabel = lv_label_create(infoContainer);
    lv_label_set_text(screenLabel, "Display");
    lv_obj_set_style_text_color(screenLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(screenLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(screenLabel, LV_ALIGN_TOP_LEFT, 15, 38);

    lv_obj_t *screenValue = lv_label_create(infoContainer);
    lv_label_set_text(screenValue, "2.06\" AMOLED");
    lv_obj_set_style_text_color(screenValue, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(screenValue, &lv_font_montserrat_14, 0);
    lv_obj_align(screenValue, LV_ALIGN_TOP_RIGHT, -15, 36);

    // Build row
    lv_obj_t *buildLabel = lv_label_create(infoContainer);
    lv_label_set_text(buildLabel, "Build");
    lv_obj_set_style_text_color(buildLabel, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_text_font(buildLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(buildLabel, LV_ALIGN_TOP_LEFT, 15, 64);

    lv_obj_t *buildValue = lv_label_create(infoContainer);
    lv_label_set_text(buildValue, "Nike Enhanced");
    lv_obj_set_style_text_color(buildValue, theme->accent, 0);
    lv_obj_set_style_text_font(buildValue, &lv_font_montserrat_14, 0);
    lv_obj_align(buildValue, LV_ALIGN_TOP_RIGHT, -15, 62);

    // Footer credit with glow
    lv_obj_t *creditLabel = lv_label_create(glassCard);
    lv_label_set_text(creditLabel, "Fusion Labs 2026");
    lv_obj_set_style_text_color(creditLabel, theme->accent, 0);
    lv_obj_set_style_text_font(creditLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_letter_space(creditLabel, 2, 0);
    lv_obj_align(creditLabel, LV_ALIGN_BOTTOM_MID, 0, -15);

    // Bottom accent line
    lv_obj_t *accentLine = lv_obj_create(card);
    lv_obj_set_size(accentLine, 60, 4);
    lv_obj_align(accentLine, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_color(accentLine, theme->accent, 0);
    lv_obj_set_style_radius(accentLine, 2, 0);
    lv_obj_set_style_border_width(accentLine, 0, 0);
    disableAllScrolling(accentLine);
}

// ═══════════════════════════════════════════════════════════════════════════
// CREATE BATTERY SAVER MODES CARD (NEW!)
// ═══════════════════════════════════════════════════════════════════════════
void createBatterySaverCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, theme->color1, 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Title with power icon
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_CHARGE " POWER SAVER");
    lv_obj_set_style_text_color(title, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Subtitle showing power savings
    lv_obj_t *subtitle = lv_label_create(card);
    lv_label_set_text(subtitle, "Tap to select power mode");
    lv_obj_set_style_text_color(subtitle, theme->secondary, 0);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_12, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 32);

    // 3 MODE OPTIONS ONLY: Off, Medium, Extreme
    const char* modeNames[] = {"NORMAL", "MEDIUM", "EXTREME"};
    const char* modeDescs[] = {
        "240MHz | Full sync | ~3hrs",      // Normal/Off
        "80MHz | Slow poll | ~5hrs",       // Medium
        "80MHz | No WiFi | ~10+hrs"        // Extreme with light sleep
    };
    const char* savingsInfo[] = {
        "Full performance",
        "~50% power saving", 
        "~75% power saving"
    };
    uint32_t modeColors[] = {0x8E8E93, 0xFF9F0A, 0xFF453A};

    int yStart = 55;
    int rowHeight = 75;  // Larger rows for 3 options

    for (int i = 0; i < 3; i++) {
        lv_obj_t *modeRow = lv_obj_create(card);
        lv_obj_set_size(modeRow, LCD_WIDTH - 24, 65);
        lv_obj_align(modeRow, LV_ALIGN_TOP_MID, 0, yStart + i * rowHeight);
        
        // Highlight selected mode
        bool isSelected = (i == (int)batterySaverLevel);
        if (isSelected) {
            lv_obj_set_style_bg_color(modeRow, lv_color_hex(modeColors[i]), 0);
            lv_obj_set_style_bg_opa(modeRow, 60, 0);
            lv_obj_set_style_border_width(modeRow, 2, 0);
            lv_obj_set_style_border_color(modeRow, lv_color_hex(modeColors[i]), 0);
        } else {
            lv_obj_set_style_bg_color(modeRow, theme->color2, 0);
            lv_obj_set_style_border_width(modeRow, 1, 0);
            lv_obj_set_style_border_color(modeRow, theme->secondary, 0);
            lv_obj_set_style_border_opa(modeRow, 50, 0);
        }
        lv_obj_set_style_radius(modeRow, 12, 0);
        disableAllScrolling(modeRow);

        // Mode name with color indicator
        lv_obj_t *modeName = lv_label_create(modeRow);
        lv_label_set_text(modeName, modeNames[i]);
        lv_obj_set_style_text_color(modeName, isSelected ? theme->text : lv_color_hex(modeColors[i]), 0);
        lv_obj_set_style_text_font(modeName, &lv_font_montserrat_14, 0);
        lv_obj_align(modeName, LV_ALIGN_LEFT_MID, 12, -12);

        // Technical details
        lv_obj_t *modeDesc = lv_label_create(modeRow);
        lv_label_set_text(modeDesc, modeDescs[i]);
        lv_obj_set_style_text_color(modeDesc, theme->secondary, 0);
        lv_obj_set_style_text_font(modeDesc, &lv_font_montserrat_12, 0);
        lv_obj_align(modeDesc, LV_ALIGN_LEFT_MID, 12, 8);

        // Power savings percentage on right
        lv_obj_t *savingsLabel = lv_label_create(modeRow);
        lv_label_set_text(savingsLabel, savingsInfo[i]);
        lv_obj_set_style_text_color(savingsLabel, lv_color_hex(modeColors[i]), 0);
        lv_obj_set_style_text_font(savingsLabel, &lv_font_montserrat_12, 0);
        lv_obj_align(savingsLabel, LV_ALIGN_RIGHT_MID, -10, -10);

        // Checkmark for selected
        if (isSelected) {
            lv_obj_t *check = lv_label_create(modeRow);
            lv_label_set_text(check, LV_SYMBOL_OK);
            lv_obj_set_style_text_color(check, theme->text, 0);
            lv_obj_set_style_text_font(check, &lv_font_montserrat_16, 0);
            lv_obj_align(check, LV_ALIGN_RIGHT_MID, -10, 10);
        }
    }

    // Current CPU frequency display
    lv_obj_t *cpuLabel = lv_label_create(card);
    char cpuBuf[48];
    snprintf(cpuBuf, sizeof(cpuBuf), "CPU: %d MHz | Poll: %ldms", 
             getCpuFrequencyMhz(), sensorPollInterval);
    lv_label_set_text(cpuLabel, cpuBuf);
    lv_obj_set_style_text_color(cpuLabel, theme->accent, 0);
    lv_obj_set_style_text_font(cpuLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(cpuLabel, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ═══════════════════════════════════════════════════════════════════════════
// TXT FILE FUNCTIONS (NEW!)
// ═══════════════════════════════════════════════════════════════════════════
void loadTxtFileList() {
    numTxtFiles = 0;
    
    if (!hasSD) return;
    
    // Create TXT folder if it doesn't exist
    if (!SD_MMC.exists(SD_TXT_PATH)) {
        SD_MMC.mkdir(SD_TXT_PATH);
        USBSerial.println("[SD] Created /WATCH/TXT folder");
        
        // Create sample file
        createSampleTxtFile();
    }
    
    File dir = SD_MMC.open(SD_TXT_PATH);
    if (!dir || !dir.isDirectory()) {
        USBSerial.println("[SD] Failed to open TXT directory");
        return;
    }
    
    File file = dir.openNextFile();
    while (file && numTxtFiles < MAX_TXT_FILES) {
        String name = file.name();
        if (name.endsWith(".txt") || name.endsWith(".TXT")) {
            strncpy(txtFiles[numTxtFiles].filename, file.name(), sizeof(txtFiles[numTxtFiles].filename) - 1);
            txtFiles[numTxtFiles].fileSize = file.size();
            numTxtFiles++;
        }
        file = dir.openNextFile();
    }
    dir.close();
    
    USBSerial.printf("[TXT] Found %d text files\n", numTxtFiles);
}

void createSampleTxtFile() {
    File sample = SD_MMC.open(SD_SAMPLE_TXT, FILE_WRITE);
    if (sample) {
        sample.println("===================================");
        sample.println("  WIGET OS - Sample Text File");
        sample.println("===================================");
        sample.println("");
        sample.println("Welcome to the Text File Reader!");
        sample.println("");
        sample.println("This is a sample text file that was");
        sample.println("automatically created by Widget OS.");
        sample.println("");
        sample.println("You can add your own .txt files to:");
        sample.println("  /WATCH/TXT/");
        sample.println("");
        sample.println("Features:");
        sample.println("- Tap a file to view it");
        sample.println("- Tap top to scroll up");
        sample.println("- Tap bottom to scroll down");
        sample.println("- Tap header bar to go back");
        sample.println("");
        sample.println("This file reader supports plain text");
        sample.println("files up to 4KB in size.");
        sample.println("");
        sample.println("===================================");
        sample.println("  Fusion Labs 2026");
        sample.println("===================================");
        sample.close();
        USBSerial.println("[SD] Created sample.txt");
    }
}

void loadTxtFileContent(int index) {
    if (index < 0 || index >= numTxtFiles || !hasSD) {
        txtFileLoaded = false;
        return;
    }
    
    selectedTxtFile = index;
    txtScrollOffset = 0;
    
    // Allocate buffer from PSRAM if not already allocated (for large files!)
    if (txtFileContent == NULL) {
        txtFileContent = (char*)heap_caps_malloc(TXT_FILE_MAX_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (txtFileContent == NULL) {
            // Fallback to smaller internal RAM buffer
            txtFileContent = (char*)malloc(8192);  // 8KB fallback
            if (txtFileContent == NULL) {
                USBSerial.println("[TXT] Failed to allocate buffer!");
                txtFileLoaded = false;
                return;
            }
            USBSerial.println("[TXT] Using 8KB internal buffer (PSRAM unavailable)");
        } else {
            USBSerial.println("[TXT] Allocated 64KB PSRAM buffer");
        }
    }
    
    String fullPath = String(SD_TXT_PATH) + "/" + txtFiles[index].filename;
    File file = SD_MMC.open(fullPath.c_str(), FILE_READ);
    
    if (file) {
        int maxRead = min((int)file.size(), TXT_FILE_MAX_SIZE - 1);
        int bytesRead = file.readBytes(txtFileContent, maxRead);
        txtFileContent[bytesRead] = 0;  // null terminator
        file.close();
        txtFileLoaded = true;
        USBSerial.printf("[TXT] Loaded: %s (%d bytes)\n", txtFiles[index].filename, bytesRead);
    } else {
        txtFileLoaded = false;
        USBSerial.println("[TXT] Failed to open file");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TIME BACKUP FOR WATCHDOG RESET
// Saves current time + offset to SD before reset, restores when WiFi fails
// FIX: Changed from 5 to 10 seconds to account for actual reset duration
// ═══════════════════════════════════════════════════════════════════════════

#define WATCHDOG_RESET_SECONDS 10  // FIX: Increased from 5 to 10 seconds for accurate time restore

void saveTimeBackup() {
    if (!hasSD || !hasRTC) return;
    
    // Create TIME folder if needed
    if (!SD_MMC.exists(SD_TIME_BACKUP_PATH)) {
        SD_MMC.mkdir(SD_TIME_BACKUP_PATH);
    }
    
    RTC_DateTime dt = rtc.getDateTime();
    
    // Add a few seconds for the watchdog reset duration
    int newSecond = dt.getSecond() + WATCHDOG_RESET_SECONDS;
    int newMinute = dt.getMinute();
    int newHour = dt.getHour();
    int newDay = dt.getDay();
    int newMonth = dt.getMonth();
    int newYear = dt.getYear();
    
    // Handle overflow
    if (newSecond >= 60) {
        newSecond -= 60;
        newMinute++;
    }
    if (newMinute >= 60) {
        newMinute -= 60;
        newHour++;
    }
    if (newHour >= 24) {
        newHour -= 24;
        newDay++;
    }
    // Simple month overflow (not handling all months perfectly)
    if (newDay > 28) {
        // Just reset to 1 for simplicity - user can adjust
        newDay = 1;
        newMonth++;
    }
    if (newMonth > 12) {
        newMonth = 1;
        newYear++;
    }
    
    // Write to backup file
    File f = SD_MMC.open(SD_TIME_BACKUP_FILE, FILE_WRITE);
    if (f) {
        f.printf("%04d\n%02d\n%02d\n%02d\n%02d\n%02d\n", 
                newYear, newMonth, newDay, newHour, newMinute, newSecond);
        f.close();
        USBSerial.printf("[TIME BACKUP] Saved: %04d-%02d-%02d %02d:%02d:%02d\n",
                        newYear, newMonth, newDay, newHour, newMinute, newSecond);
    }
}

bool hasTimeBackup() {
    return hasSD && SD_MMC.exists(SD_TIME_BACKUP_FILE);
}

void restoreTimeBackup() {
    if (!hasSD || !hasRTC) return;
    if (!SD_MMC.exists(SD_TIME_BACKUP_FILE)) {
        USBSerial.println("[TIME BACKUP] No backup found");
        return;
    }
    
    File f = SD_MMC.open(SD_TIME_BACKUP_FILE, FILE_READ);
    if (f) {
        int year = f.parseInt();
        int month = f.parseInt();
        int day = f.parseInt();
        int hour = f.parseInt();
        int minute = f.parseInt();
        int second = f.parseInt();
        f.close();
        
        // Validate values
        if (year >= 2020 && year <= 2099 && month >= 1 && month <= 12 &&
            day >= 1 && day <= 31 && hour >= 0 && hour <= 23 &&
            minute >= 0 && minute <= 59 && second >= 0 && second <= 59) {
            
            rtc.setDateTime(year, month, day, hour, minute, second);
            USBSerial.printf("[TIME BACKUP] Restored: %04d-%02d-%02d %02d:%02d:%02d\n",
                            year, month, day, hour, minute, second);
            
            // Delete backup after successful restore
            SD_MMC.remove(SD_TIME_BACKUP_FILE);
        } else {
            USBSerial.println("[TIME BACKUP] Invalid backup data");
        }
    }
}

void deleteTimeBackup() {
    if (hasSD && SD_MMC.exists(SD_TIME_BACKUP_FILE)) {
        SD_MMC.remove(SD_TIME_BACKUP_FILE);
        USBSerial.println("[TIME BACKUP] Deleted after WiFi sync");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// POWER CONSUMPTION LOGGING FUNCTIONS (NEW!)
// Logs battery drain, power modes, and usage patterns to SD card
// ═══════════════════════════════════════════════════════════════════════════

void initPowerLogging() {
    if (!hasSD) return;
    
    // Create power logs folder if it doesn't exist
    if (!SD_MMC.exists(SD_POWER_LOGS_PATH)) {
        SD_MMC.mkdir(SD_POWER_LOGS_PATH);
        USBSerial.println("[POWER] Created /WATCH/POWER_LOGS folder");
    }
    
    // Initialize session
    currentPowerSession.sessionStartMs = millis();
    currentPowerSession.startBatteryPercent = batteryPercent;
    currentPowerSession.currentBatteryPercent = batteryPercent;
    currentPowerSession.totalScreenOnMs = 0;
    currentPowerSession.totalScreenOffMs = 0;
    currentPowerSession.modeChanges = 0;
    currentPowerSession.avgDrainRate = 0;
    currentPowerSession.dominantMode = (uint8_t)batterySaverLevel;
    
    // Write session start header
    File logFile = SD_MMC.open(SD_POWER_LOG_CURRENT, FILE_WRITE);
    if (logFile) {
        RTC_DateTime dt = rtc.getDateTime();
        logFile.printf("═══════════════════════════════════════════════════\n");
        logFile.printf("POWER LOG SESSION STARTED\n");
        logFile.printf("Date: %04d-%02d-%02d %02d:%02d:%02d\n", 
                      dt.getYear(), dt.getMonth(), dt.getDay(),
                      dt.getHour(), dt.getMinute(), dt.getSecond());
        logFile.printf("Starting Battery: %d%% (%dmV)\n", batteryPercent, batteryVoltage);
        logFile.printf("Initial Mode: %s\n", saverModes[batterySaverLevel].name);
        logFile.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
        logFile.printf("═══════════════════════════════════════════════════\n\n");
        logFile.printf("TIME       | BAT%% | mV   | MODE    | CPU  | ON/OFF  | DRAIN\n");
        logFile.printf("-----------|------|------|---------|------|---------|-------\n");
        logFile.close();
        USBSerial.println("[POWER] Log session started");
    }
    
    lastPowerLogMs = millis();
    lastPowerSummaryMs = millis();
}

String formatPowerLogEntry(PowerLogEntry &entry) {
    char buf[128];
    int hrs = entry.timestamp / 3600;
    int mins = (entry.timestamp % 3600) / 60;
    int secs = entry.timestamp % 60;
    
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d | %3d%% | %4d | %-7s | %3d  | %4lu/%4lu | %.2f%%/h",
             hrs, mins, secs,
             entry.batteryPercent,
             entry.batteryVoltage,
             saverModes[entry.saverLevel].name,
             entry.cpuFreqMHz,
             entry.screenOnSecs / 60,
             entry.screenOffSecs / 60,
             entry.drainRatePerHour);
    
    return String(buf);
}

void logPowerConsumption() {
    if (!hasSD || !powerLoggingEnabled) return;
    if (millis() - lastPowerLogMs < POWER_LOG_INTERVAL_MS) return;
    
    // Calculate drain rate
    uint32_t elapsedMs = millis() - currentPowerSession.sessionStartMs;
    float elapsedHours = elapsedMs / 3600000.0;
    float drainPercent = currentPowerSession.startBatteryPercent - batteryPercent;
    float drainRatePerHour = (elapsedHours > 0.1) ? (drainPercent / elapsedHours) : 0;
    
    // Create log entry
    PowerLogEntry entry;
    entry.timestamp = elapsedMs / 1000;
    entry.batteryPercent = batteryPercent;
    entry.batteryVoltage = batteryVoltage;
    entry.saverLevel = (uint8_t)batterySaverLevel;
    entry.cpuFreqMHz = getCpuFrequencyMhz();
    entry.screenOnSecs = batteryStats.screenOnTimeMs / 1000;
    entry.screenOffSecs = batteryStats.screenOffTimeMs / 1000;
    entry.drainRatePerHour = drainRatePerHour;
    
    // Update session
    currentPowerSession.currentBatteryPercent = batteryPercent;
    currentPowerSession.avgDrainRate = drainRatePerHour;
    
    // Append to log file
    File logFile = SD_MMC.open(SD_POWER_LOG_CURRENT, FILE_APPEND);
    if (logFile) {
        logFile.println(formatPowerLogEntry(entry));
        logFile.close();
    }
    
    lastPowerLogMs = millis();
    USBSerial.printf("[POWER] Logged: %d%%, %.2f%%/hr drain\n", batteryPercent, drainRatePerHour);
}


void writePowerSummary() {
    if (!hasSD || !powerLoggingEnabled) return;
    if (millis() - lastPowerSummaryMs < POWER_SUMMARY_INTERVAL_MS) return;
    
    uint32_t elapsedMs = millis() - currentPowerSession.sessionStartMs;
    float elapsedHours = elapsedMs / 3600000.0;
    float drainPercent = currentPowerSession.startBatteryPercent - batteryPercent;
    float drainRatePerHour = (elapsedHours > 0.1) ? (drainPercent / elapsedHours) : 0;
    
    // Calculate estimated remaining time
    float remainingHours = (drainRatePerHour > 0.5) ? (batteryPercent / drainRatePerHour) : 99;
    
    // Calculate screen on percentage
    uint32_t totalMs = batteryStats.screenOnTimeMs + batteryStats.screenOffTimeMs;
    float screenOnPercent = (totalMs > 0) ? (batteryStats.screenOnTimeMs * 100.0 / totalMs) : 0;
    
    // Write/update summary file
    File summaryFile = SD_MMC.open(SD_POWER_SUMMARY, FILE_WRITE);
    if (summaryFile) {
        RTC_DateTime dt = rtc.getDateTime();
        
        summaryFile.println("╔═══════════════════════════════════════════════════╗");
        summaryFile.println("║         POWER CONSUMPTION SUMMARY                 ║");
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.printf("║ Last Update: %04d-%02d-%02d %02d:%02d                   ║\n",
                          dt.getYear(), dt.getMonth(), dt.getDay(),
                          dt.getHour(), dt.getMinute());
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.println("║ CURRENT SESSION                                   ║");
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.printf("║ Session Duration:     %.1f hours                   \n", elapsedHours);
        summaryFile.printf("║ Battery Start:        %d%%                         \n", currentPowerSession.startBatteryPercent);
        summaryFile.printf("║ Battery Current:      %d%% (%dmV)                  \n", batteryPercent, batteryVoltage);
        summaryFile.printf("║ Battery Drained:      %.1f%%                       \n", drainPercent);
        summaryFile.printf("║ Drain Rate:           %.2f%%/hour                  \n", drainRatePerHour);
        summaryFile.printf("║ Est. Remaining:       %.1f hours                   \n", remainingHours);
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.println("║ USAGE BREAKDOWN                                   ║");
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.printf("║ Screen ON Time:       %lu min (%.0f%%)             \n", 
                          batteryStats.screenOnTimeMs / 60000, screenOnPercent);
        summaryFile.printf("║ Screen OFF Time:      %lu min                      \n", 
                          batteryStats.screenOffTimeMs / 60000);
        summaryFile.printf("║ Mode Changes:         %lu                          \n", 
                          currentPowerSession.modeChanges);
        summaryFile.printf("║ Current Mode:         %s                          \n", 
                          saverModes[batterySaverLevel].name);
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        summaryFile.println("║ POWER EFFICIENCY                                  ║");
        summaryFile.println("╠═══════════════════════════════════════════════════╣");
        
        // Power efficiency rating
        const char* efficiencyRating;
        if (drainRatePerHour < 5) efficiencyRating = "EXCELLENT - Deep Sleep Working!";
        else if (drainRatePerHour < 10) efficiencyRating = "GOOD - Saver Mode Effective";
        else if (drainRatePerHour < 20) efficiencyRating = "MODERATE - Normal Usage";
        else if (drainRatePerHour < 30) efficiencyRating = "HIGH - Consider Saver Mode";
        else efficiencyRating = "VERY HIGH - Enable Extreme Saver";
        
        summaryFile.printf("║ Rating: %s                                        \n", efficiencyRating);
        summaryFile.printf("║ CPU Frequency:        %d MHz                       \n", getCpuFrequencyMhz());
        summaryFile.printf("║ WiFi Status:          %s                          \n", 
                          wifiConnected ? "Connected" : "Disconnected");
        summaryFile.println("╚═══════════════════════════════════════════════════╝");
        summaryFile.println("");
        summaryFile.println("TIP: Check current.txt for detailed timeline logs");
        summaryFile.println("TIP: Lower drain rate = better battery optimization");
        summaryFile.close();
        
        USBSerial.println("[POWER] Summary updated");
    }
    
    lastPowerSummaryMs = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// POWER LOG ROTATION & DAILY ARCHIVING
// ═══════════════════════════════════════════════════════════════════════════

// Rotate current log file if it exceeds max size
void rotatePowerLogIfNeeded() {
    if (!hasSD || !powerLoggingEnabled) return;
    if (millis() - lastLogRotationCheck < LOG_ROTATION_CHECK_MS) return;
    lastLogRotationCheck = millis();
    
    File logFile = SD_MMC.open(SD_POWER_LOG_CURRENT, FILE_READ);
    if (!logFile) return;
    
    size_t fileSize = logFile.size();
    logFile.close();
    
    if (fileSize > MAX_LOG_FILE_SIZE) {
        // Archive current log with timestamp
        RTC_DateTime dt = rtc.getDateTime();
        char archivePath[64];
        snprintf(archivePath, sizeof(archivePath), 
                "/WATCH/POWER_LOGS/archive_%04d%02d%02d_%02d%02d.txt",
                dt.getYear(), dt.getMonth(), dt.getDay(),
                dt.getHour(), dt.getMinute());
        
        // Rename current to archive
        SD_MMC.rename(SD_POWER_LOG_CURRENT, archivePath);
        USBSerial.printf("[POWER] Log rotated to %s\n", archivePath);
        
        // Start a new log session
        initPowerLogging();
    }
}

// Archive daily power log at midnight
void archiveDailyPowerLog() {
    if (!hasSD || !powerLoggingEnabled) return;
    
    // Skip on first few seconds of boot to avoid WDT issues
    static bool firstRunSkipped = false;
    if (!firstRunSkipped) {
        if (millis() < 10000) return;  // Skip first 10 seconds
        firstRunSkipped = true;
    }
    
    RTC_DateTime dt = rtc.getDateTime();
    uint8_t currentDay = dt.getDay();
    
    // Check if day has changed
    if (lastLogDay != 0 && lastLogDay != currentDay) {
        // Create daily summary before archiving
        writeDailyStats();
        
        // Archive current log with yesterday's date
        char archivePath[64];
        // Get yesterday's date (simplified - doesn't handle month boundaries perfectly)
        uint8_t prevDay = (lastLogDay > 0) ? lastLogDay : currentDay;
        snprintf(archivePath, sizeof(archivePath), 
                "/WATCH/POWER_LOGS/daily_%04d%02d%02d.txt",
                dt.getYear(), dt.getMonth(), prevDay);
        
        // Copy current to daily archive if it exists - USE BUFFERED COPY
        if (SD_MMC.exists(SD_POWER_LOG_CURRENT)) {
            File srcFile = SD_MMC.open(SD_POWER_LOG_CURRENT, FILE_READ);
            if (srcFile) {
                File dstFile = SD_MMC.open(archivePath, FILE_WRITE);
                if (dstFile) {
                    uint8_t buf[512];  // Buffered copy - much faster!
                    size_t bytesRead;
                    while ((bytesRead = srcFile.read(buf, sizeof(buf))) > 0) {
                        dstFile.write(buf, bytesRead);
                        esp_task_wdt_reset();  // Feed watchdog during long operations
                    }
                    dstFile.close();
                    USBSerial.printf("[POWER] Daily log archived: %s\n", archivePath);
                }
                srcFile.close();
            }
        }
        
        // Start fresh log for new day
        initPowerLogging();
    }
    
    lastLogDay = currentDay;
}

// Write daily statistics summary
void writeDailyStats() {
    if (!hasSD) return;
    
    RTC_DateTime dt = rtc.getDateTime();
    
    // Calculate session stats
    uint32_t elapsedMs = millis() - currentPowerSession.sessionStartMs;
    float elapsedHours = elapsedMs / 3600000.0;
    float drainPercent = currentPowerSession.startBatteryPercent - batteryPercent;
    float drainRatePerHour = (elapsedHours > 0.1) ? (drainPercent / elapsedHours) : 0;
    
    // Calculate screen on percentage
    uint32_t totalMs = batteryStats.screenOnTimeMs + batteryStats.screenOffTimeMs;
    float screenOnPercent = (totalMs > 0) ? (batteryStats.screenOnTimeMs * 100.0 / totalMs) : 0;
    
    // Append to history file
    char historyPath[64];
    snprintf(historyPath, sizeof(historyPath), "/WATCH/POWER_LOGS/history_%04d%02d.txt",
            dt.getYear(), dt.getMonth());
    
    File histFile = SD_MMC.open(historyPath, FILE_APPEND);
    if (histFile) {
        histFile.printf("─────────────────────────────────────\n");
        histFile.printf("DATE: %04d-%02d-%02d\n", dt.getYear(), dt.getMonth(), dt.getDay());
        histFile.printf("─────────────────────────────────────\n");
        histFile.printf("Runtime:        %.1f hours\n", elapsedHours);
        histFile.printf("Battery Used:   %.1f%%\n", drainPercent);
        histFile.printf("Drain Rate:     %.2f%%/hour\n", drainRatePerHour);
        histFile.printf("Screen ON:      %.0f%% of time\n", screenOnPercent);
        histFile.printf("Mode Changes:   %lu\n", currentPowerSession.modeChanges);
        histFile.printf("Final Battery:  %d%%\n", batteryPercent);
        histFile.printf("\n");
        histFile.close();
        
        USBSerial.printf("[POWER] Daily stats saved to history\n");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CREATE TXT FILES CARD - ENHANCED UI
// ═══════════════════════════════════════════════════════════════════════════
void createTxtFilesCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, theme->color1, 0);  // Use theme color instead of hardcoded
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    if (!txtFileLoaded) {
        // ═══ FILE LIST VIEW - ENHANCED UI ═══
        
        // Title bar with gradient effect
        lv_obj_t *titleBar = lv_obj_create(card);
        lv_obj_set_size(titleBar, LCD_WIDTH, 48);
        lv_obj_align(titleBar, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_color(titleBar, theme->color2, 0);
        lv_obj_set_style_radius(titleBar, 0, 0);
        lv_obj_set_style_border_width(titleBar, 0, 0);
        lv_obj_set_style_shadow_width(titleBar, 8, 0);
        lv_obj_set_style_shadow_color(titleBar, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(titleBar, 100, 0);
        disableAllScrolling(titleBar);

        // Title icon and text
        lv_obj_t *titleIcon = lv_label_create(titleBar);
        lv_label_set_text(titleIcon, LV_SYMBOL_FILE);
        lv_obj_set_style_text_color(titleIcon, theme->accent, 0);
        lv_obj_set_style_text_font(titleIcon, &lv_font_montserrat_16, 0);
        lv_obj_align(titleIcon, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t *title = lv_label_create(titleBar);
        lv_label_set_text(title, "TEXT FILES");
        lv_obj_set_style_text_color(title, theme->text, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 42, 0);

        // Load file list
        loadTxtFileList();

        if (numTxtFiles == 0) {
            // Empty state with better styling
            lv_obj_t *emptyIcon = lv_label_create(card);
            lv_label_set_text(emptyIcon, LV_SYMBOL_DIRECTORY);
            lv_obj_set_style_text_color(emptyIcon, theme->secondary, 0);
            lv_obj_set_style_text_font(emptyIcon, &lv_font_montserrat_24, 0);
            lv_obj_align(emptyIcon, LV_ALIGN_CENTER, 0, -30);

            lv_obj_t *noFiles = lv_label_create(card);
            lv_label_set_text(noFiles, "No text files found");
            lv_obj_set_style_text_color(noFiles, theme->secondary, 0);
            lv_obj_set_style_text_font(noFiles, &lv_font_montserrat_14, 0);
            lv_obj_align(noFiles, LV_ALIGN_CENTER, 0, 10);

            lv_obj_t *hint = lv_label_create(card);
            lv_label_set_text(hint, "Add .txt files to:\n/WATCH/TXT/");
            lv_obj_set_style_text_color(hint, theme->accent, 0);
            lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_align(hint, LV_ALIGN_CENTER, 0, 55);
        } else {
            // ═══ BEAUTIFUL FILE LIST ═══
            int yOffset = 56;
            int itemHeight = 52;
            int maxVisible = 5;
            
            // Clamp scroll offset
            int maxScroll = max(0, numTxtFiles - maxVisible);
            if (txtFileListOffset > maxScroll) txtFileListOffset = maxScroll;
            if (txtFileListOffset < 0) txtFileListOffset = 0;
            
            // Scroll indicators with theme colors
            if (numTxtFiles > maxVisible) {
                if (txtFileListOffset > 0) {
                    lv_obj_t *upArrow = lv_label_create(card);
                    lv_label_set_text(upArrow, LV_SYMBOL_UP);
                    lv_obj_set_style_text_color(upArrow, theme->accent, 0);
                    lv_obj_set_style_text_font(upArrow, &lv_font_montserrat_16, 0);
                    lv_obj_align(upArrow, LV_ALIGN_TOP_RIGHT, -12, 52);
                }
                if (txtFileListOffset < maxScroll) {
                    lv_obj_t *downArrow = lv_label_create(card);
                    lv_label_set_text(downArrow, LV_SYMBOL_DOWN);
                    lv_obj_set_style_text_color(downArrow, theme->accent, 0);
                    lv_obj_set_style_text_font(downArrow, &lv_font_montserrat_16, 0);
                    lv_obj_align(downArrow, LV_ALIGN_BOTTOM_RIGHT, -12, -30);
                }
            }
            
            // Display visible files with enhanced styling
            for (int i = 0; i < min(numTxtFiles - txtFileListOffset, maxVisible); i++) {
                int fileIdx = i + txtFileListOffset;
                
                lv_obj_t *fileRow = lv_obj_create(card);
                lv_obj_set_size(fileRow, LCD_WIDTH - 24, 46);
                lv_obj_align(fileRow, LV_ALIGN_TOP_MID, 0, yOffset + i * itemHeight);
                lv_obj_set_style_bg_color(fileRow, theme->color2, 0);
                lv_obj_set_style_radius(fileRow, 12, 0);
                lv_obj_set_style_border_width(fileRow, 1, 0);
                lv_obj_set_style_border_color(fileRow, theme->secondary, 0);
                lv_obj_set_style_border_opa(fileRow, 50, 0);
                disableAllScrolling(fileRow);

                // File icon with theme accent
                lv_obj_t *icon = lv_label_create(fileRow);
                lv_label_set_text(icon, LV_SYMBOL_FILE);
                lv_obj_set_style_text_color(icon, theme->accent, 0);
                lv_obj_set_style_text_font(icon, &lv_font_montserrat_14, 0);
                lv_obj_align(icon, LV_ALIGN_LEFT_MID, 12, 0);

                // Filename
                char displayName[22];
                strncpy(displayName, txtFiles[fileIdx].filename, 18);
                displayName[18] = '\0';
                if (strlen(txtFiles[fileIdx].filename) > 18) {
                    displayName[15] = '.';
                    displayName[16] = '.';
                    displayName[17] = '.';
                }
                
                lv_obj_t *nameLabel = lv_label_create(fileRow);
                lv_label_set_text(nameLabel, displayName);
                lv_obj_set_style_text_color(nameLabel, theme->text, 0);
                lv_obj_set_style_text_font(nameLabel, &lv_font_montserrat_12, 0);
                lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 35, -6);

                // File size with subtle color
                char sizeBuf[16];
                if (txtFiles[fileIdx].fileSize < 1024) {
                    snprintf(sizeBuf, sizeof(sizeBuf), "%lu bytes", (unsigned long)txtFiles[fileIdx].fileSize);
                } else {
                    snprintf(sizeBuf, sizeof(sizeBuf), "%.1f KB", txtFiles[fileIdx].fileSize / 1024.0);
                }
                lv_obj_t *sizeLabel = lv_label_create(fileRow);
                lv_label_set_text(sizeLabel, sizeBuf);
                lv_obj_set_style_text_color(sizeLabel, theme->secondary, 0);
                lv_obj_set_style_text_font(sizeLabel, &lv_font_montserrat_12, 0);
                lv_obj_align(sizeLabel, LV_ALIGN_LEFT_MID, 35, 10);

                // Chevron arrow
                lv_obj_t *arrow = lv_label_create(fileRow);
                lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
                lv_obj_set_style_text_color(arrow, theme->secondary, 0);
                lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -8, 0);
            }

            // Bottom hint
            lv_obj_t *hint = lv_label_create(card);
            lv_label_set_text(hint, numTxtFiles > maxVisible ? 
                            "Tap edges to scroll" : "Tap file to open");
            lv_obj_set_style_text_color(hint, theme->secondary, 0);
            lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
            lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
        }
    } else {
        // ═══ FILE CONTENT VIEW - ENHANCED READER ═══
        
        // Header bar
        lv_obj_t *headerBar = lv_obj_create(card);
        lv_obj_set_size(headerBar, LCD_WIDTH, 48);
        lv_obj_align(headerBar, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_color(headerBar, theme->color2, 0);
        lv_obj_set_style_radius(headerBar, 0, 0);
        lv_obj_set_style_border_width(headerBar, 0, 0);
        lv_obj_set_style_shadow_width(headerBar, 8, 0);
        lv_obj_set_style_shadow_color(headerBar, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(headerBar, 100, 0);
        disableAllScrolling(headerBar);

        // Back button
        lv_obj_t *backArrow = lv_label_create(headerBar);
        lv_label_set_text(backArrow, LV_SYMBOL_LEFT);
        lv_obj_set_style_text_color(backArrow, theme->accent, 0);
        lv_obj_set_style_text_font(backArrow, &lv_font_montserrat_16, 0);
        lv_obj_align(backArrow, LV_ALIGN_LEFT_MID, 12, 0);

        // Filename in header
        char headerName[18];
        strncpy(headerName, txtFiles[selectedTxtFile].filename, 14);
        headerName[14] = '\0';
        if (strlen(txtFiles[selectedTxtFile].filename) > 14) {
            headerName[11] = '.';
            headerName[12] = '.';
            headerName[13] = '.';
        }
        lv_obj_t *fileName = lv_label_create(headerBar);
        lv_label_set_text(fileName, headerName);
        lv_obj_set_style_text_color(fileName, theme->text, 0);
        lv_obj_set_style_text_font(fileName, &lv_font_montserrat_14, 0);
        lv_obj_center(fileName);

        // Content area with nice styling
        lv_obj_t *contentArea = lv_obj_create(card);
        lv_obj_set_size(contentArea, LCD_WIDTH - 16, LCD_HEIGHT - 85);
        lv_obj_align(contentArea, LV_ALIGN_TOP_MID, 0, 52);
        lv_obj_set_style_bg_color(contentArea, theme->color2, 0);
        lv_obj_set_style_radius(contentArea, 16, 0);
        lv_obj_set_style_border_width(contentArea, 0, 0);
        lv_obj_set_style_pad_all(contentArea, 14, 0);
        
        // Enable smooth scrolling for content
        lv_obj_set_scroll_dir(contentArea, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(contentArea, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_style_bg_color(contentArea, theme->accent, LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(contentArea, LV_OPA_70, LV_PART_SCROLLBAR);
        lv_obj_set_style_width(contentArea, 4, LV_PART_SCROLLBAR);

        // Text content with readable styling
        lv_obj_t *textLabel = lv_label_create(contentArea);
        lv_label_set_text(textLabel, txtFileContent);
        lv_obj_set_style_text_color(textLabel, theme->text, 0);
        lv_obj_set_style_text_font(textLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_line_space(textLabel, 4, 0);  // Better line spacing
        lv_obj_set_width(textLabel, LCD_WIDTH - 50);
        lv_label_set_long_mode(textLabel, LV_LABEL_LONG_WRAP);
        lv_obj_align(textLabel, LV_ALIGN_TOP_LEFT, 0, 0);

        // Scroll progress bar at bottom
        lv_obj_t *progressBg = lv_obj_create(card);
        lv_obj_set_size(progressBg, LCD_WIDTH - 80, 4);
        lv_obj_align(progressBg, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(progressBg, theme->secondary, 0);
        lv_obj_set_style_bg_opa(progressBg, 100, 0);
        lv_obj_set_style_radius(progressBg, 2, 0);
        lv_obj_set_style_border_width(progressBg, 0, 0);

        // Scroll hint
        lv_obj_t *scrollHint = lv_label_create(card);
        lv_label_set_text(scrollHint, "Tap header to go back");
        lv_obj_set_style_text_color(scrollHint, theme->secondary, 0);
        lv_obj_set_style_text_font(scrollHint, &lv_font_montserrat_12, 0);
        lv_obj_align(scrollHint, LV_ALIGN_BOTTOM_MID, 0, -18);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI NETWORK SCANNING
// ═══════════════════════════════════════════════════════════════════════════
void scanWiFiNetworks() {
    numScannedNetworks = 0;
    
    int n = WiFi.scanNetworks(false, false);  // Synchronous scan
    
    if (n > 0) {
        for (int i = 0; i < min(n, MAX_SCANNED_NETWORKS); i++) {
            strncpy(scannedNetworks[numScannedNetworks].ssid, WiFi.SSID(i).c_str(), 63);
            scannedNetworks[numScannedNetworks].ssid[63] = '\0';
            scannedNetworks[numScannedNetworks].rssi = WiFi.RSSI(i);
            scannedNetworks[numScannedNetworks].isOpen = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
            scannedNetworks[numScannedNetworks].isConnected = (WiFi.status() == WL_CONNECTED && 
                                                              WiFi.SSID() == WiFi.SSID(i));
            numScannedNetworks++;
        }
    }
    
    wifiScanComplete = true;
    WiFi.scanDelete();  // Free memory
    USBSerial.printf("[WIFI] Scan complete: %d networks found\n", numScannedNetworks);
}

// ═══════════════════════════════════════════════════════════════════════════
// CONNECT TO HARDCODED WIFI + TIME SYNC (NEW!)
// ═══════════════════════════════════════════════════════════════════════════
void connectToHardcodedWiFi() {
    USBSerial.printf("[WIFI] Connecting to hardcoded: %s\n", hardcodedSSID);
    
    WiFi.begin(hardcodedSSID, hardcodedPass);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        attempts++;
        USBSerial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        USBSerial.printf("\n[WIFI] Connected to: %s\n", hardcodedSSID);
        
        // Sync time via NTP
        USBSerial.println("[NTP] Syncing time...");
        configTime(gmtOffsetSec, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
        delay(2000);
        
        // Update RTC from NTP
        if (hasRTC) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                rtc.setDateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                ntpSyncedOnce = true;
                lastNTPSync = millis();
                USBSerial.printf("[NTP] Time synced: %04d-%02d-%02d %02d:%02d:%02d\n",
                                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            }
        }
        
        // Fetch weather for location
        fetchLocationFromIP();
    } else {
        wifiConnected = false;
        USBSerial.println("\n[WIFI] Failed to connect to hardcoded network");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI CARD - NETWORK MANAGER (NOW IN SETTINGS)
// ═══════════════════════════════════════════════════════════════════════════
void createWiFiCard() {
    GradientTheme *theme = getSafeTheme();
    disableAllScrolling(lv_scr_act());

    lv_obj_t *card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_radius(card, 0, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    disableAllScrolling(card);

    // Top accent bar - WiFi blue
    lv_obj_t *accentBar = lv_obj_create(card);
    lv_obj_set_size(accentBar, LCD_WIDTH, 4);
    lv_obj_align(accentBar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(accentBar, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_radius(accentBar, 0, 0);
    lv_obj_set_style_border_width(accentBar, 0, 0);
    disableAllScrolling(accentBar);

    // Title badge
    lv_obj_t *titleBadge = lv_obj_create(card);
    lv_obj_set_size(titleBadge, 130, 32);
    lv_obj_align(titleBadge, LV_ALIGN_TOP_MID, 0, 12);
    lv_obj_set_style_bg_color(titleBadge, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_bg_opa(titleBadge, LV_OPA_20, 0);
    lv_obj_set_style_radius(titleBadge, 16, 0);
    lv_obj_set_style_border_width(titleBadge, 1, 0);
    lv_obj_set_style_border_color(titleBadge, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_border_opa(titleBadge, LV_OPA_50, 0);
    disableAllScrolling(titleBadge);

    lv_obj_t *title = lv_label_create(titleBadge);
    lv_label_set_text(title, LV_SYMBOL_WIFI " WiFi");
    lv_obj_set_style_text_color(title, lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_center(title);

    // Connection status card
    lv_obj_t *statusCard = lv_obj_create(card);
    lv_obj_set_size(statusCard, LCD_WIDTH - 24, 70);
    lv_obj_align(statusCard, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_bg_color(statusCard, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_radius(statusCard, 16, 0);
    lv_obj_set_style_border_width(statusCard, 1, 0);
    lv_obj_set_style_border_color(statusCard, wifiConnected ? lv_color_hex(0x30D158) : lv_color_hex(0x2A2A2A), 0);
    disableAllScrolling(statusCard);

    lv_obj_t *statusIcon = lv_label_create(statusCard);
    lv_label_set_text(statusIcon, wifiConnected ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(statusIcon, wifiConnected ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    lv_obj_set_style_text_font(statusIcon, &lv_font_montserrat_24, 0);
    lv_obj_align(statusIcon, LV_ALIGN_LEFT_MID, 15, 0);

    lv_obj_t *statusLabel = lv_label_create(statusCard);
    if (wifiConnected) {
        char ssidBuf[24];
        strncpy(ssidBuf, WiFi.SSID().c_str(), 20);
        ssidBuf[20] = '\0';
        lv_label_set_text(statusLabel, ssidBuf);
        lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xFFFFFF), 0);
    } else {
        lv_label_set_text(statusLabel, "Not Connected");
        lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x8E8E93), 0);
    }
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_16, 0);
    lv_obj_align(statusLabel, LV_ALIGN_LEFT_MID, 50, -8);

    lv_obj_t *infoLabel = lv_label_create(statusCard);
    if (wifiConnected && ntpSyncedOnce) {
        lv_label_set_text(infoLabel, "Time synced " LV_SYMBOL_OK);
        lv_obj_set_style_text_color(infoLabel, lv_color_hex(0x30D158), 0);
    } else {
        lv_label_set_text(infoLabel, "Tap below to connect");
        lv_obj_set_style_text_color(infoLabel, lv_color_hex(0x636366), 0);
    }
    lv_obj_set_style_text_font(infoLabel, &lv_font_montserrat_12, 0);
    lv_obj_align(infoLabel, LV_ALIGN_LEFT_MID, 50, 10);

    // Connect button with glow effect
    lv_obj_t *connectBtn = lv_obj_create(card);
    lv_obj_set_size(connectBtn, LCD_WIDTH - 24, 55);
    lv_obj_align(connectBtn, LV_ALIGN_TOP_MID, 0, 130);
    lv_obj_set_style_bg_color(connectBtn, wifiConnected ? lv_color_hex(0x30D158) : lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_radius(connectBtn, 16, 0);
    lv_obj_set_style_border_width(connectBtn, 0, 0);
    lv_obj_set_style_shadow_width(connectBtn, 12, 0);
    lv_obj_set_style_shadow_color(connectBtn, wifiConnected ? lv_color_hex(0x30D158) : lv_color_hex(0x0A84FF), 0);
    lv_obj_set_style_shadow_opa(connectBtn, LV_OPA_30, 0);
    disableAllScrolling(connectBtn);

    lv_obj_t *connectIcon = lv_label_create(connectBtn);
    lv_label_set_text(connectIcon, wifiConnected ? LV_SYMBOL_OK : LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(connectIcon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(connectIcon, &lv_font_montserrat_20, 0);
    lv_obj_align(connectIcon, LV_ALIGN_LEFT_MID, 20, 0);

    lv_obj_t *connectLabel = lv_label_create(connectBtn);
    char btnBuf[32];
    snprintf(btnBuf, sizeof(btnBuf), wifiConnected ? "CONNECTED" : "Connect: %s", hardcodedSSID);
    lv_label_set_text(connectLabel, btnBuf);
    lv_obj_set_style_text_color(connectLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(connectLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(connectLabel, LV_ALIGN_LEFT_MID, 55, 0);

    // Saved networks section
    lv_obj_t *savedTitle = lv_label_create(card);
    lv_label_set_text(savedTitle, "SAVED NETWORKS");
    lv_obj_set_style_text_color(savedTitle, lv_color_hex(0x636366), 0);
    lv_obj_set_style_text_font(savedTitle, &lv_font_montserrat_12, 0);
    lv_obj_align(savedTitle, LV_ALIGN_TOP_LEFT, 18, 195);

    lv_obj_t *savedContainer = lv_obj_create(card);
    lv_obj_set_size(savedContainer, LCD_WIDTH - 24, 95);
    lv_obj_align(savedContainer, LV_ALIGN_TOP_MID, 0, 212);
    lv_obj_set_style_bg_color(savedContainer, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_radius(savedContainer, 16, 0);
    lv_obj_set_style_border_width(savedContainer, 0, 0);
    disableAllScrolling(savedContainer);

    if (numWifiNetworks > 0) {
        int yOff = 10;
        for (int i = 0; i < min(numWifiNetworks, 2); i++) {
            lv_obj_t *netIcon = lv_label_create(savedContainer);
            lv_label_set_text(netIcon, LV_SYMBOL_WIFI);
            bool isConnected = wifiConnected && (String(wifiNetworks[i].ssid) == WiFi.SSID());
            lv_obj_set_style_text_color(netIcon, isConnected ? lv_color_hex(0x30D158) : lv_color_hex(0x636366), 0);
            lv_obj_align(netIcon, LV_ALIGN_TOP_LEFT, 15, yOff + 5);

            char ssidDisplay[24];
            strncpy(ssidDisplay, wifiNetworks[i].ssid, 20);
            ssidDisplay[20] = '\0';
            lv_obj_t *ssidLabel = lv_label_create(savedContainer);
            lv_label_set_text(ssidLabel, ssidDisplay);
            lv_obj_set_style_text_color(ssidLabel, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(ssidLabel, &lv_font_montserrat_14, 0);
            lv_obj_align(ssidLabel, LV_ALIGN_TOP_LEFT, 45, yOff + 5);
            yOff += 40;
        }
    } else {
        lv_obj_t *noNetLabel = lv_label_create(savedContainer);
        lv_label_set_text(noNetLabel, "No saved networks");
        lv_obj_set_style_text_color(noNetLabel, lv_color_hex(0x636366), 0);
        lv_obj_center(noNetLabel);
    }

    lv_obj_t *hint = lv_label_create(card);
    lv_label_set_text(hint, "TAP BUTTON TO CONNECT & SYNC");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x4A4A4A), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -10);
}

// ═══════════════════════════════════════════════════════════════════════════
// USER DATA PERSISTENCE
// ═══════════════════════════════════════════════════════════════════════════
void saveUserData() {
    prefs.begin("minios", false);
    prefs.putUInt("steps", userData.steps);
    prefs.putUInt("dailyGoal", userData.dailyGoal);
    prefs.putInt("brightness", userData.brightness);
    prefs.putInt("themeIndex", userData.themeIndex);
    prefs.putInt("wallpaper", userData.wallpaperIndex);
    prefs.putInt("watchFace", userData.watchFaceIndex);  // Save watch face
    prefs.putInt("nikeColor", currentNikeColor);  // Save Nike color variant
    prefs.putInt("photoIdx", currentPhotoIndex);  // Save Photo face index
    prefs.putFloat("distance", userData.totalDistance);
    prefs.putFloat("calories", userData.totalCalories);
    prefs.putInt("gamesWon", userData.gamesWon);
    prefs.putInt("gamesPlayed", userData.gamesPlayed);
    prefs.putInt("batterySaverLvl", (int)batterySaverLevel);  // Save battery saver level
    prefs.putBool("use24Hr", use24HourFormat);  // NEW: Save 24hr format preference
    
    // Save weather data (persists until next WiFi sync)
    prefs.putFloat("weatherTemp", weatherTemp);
    prefs.putFloat("weatherHigh", weatherHigh);
    prefs.putFloat("weatherLow", weatherLow);
    prefs.putString("weatherDesc", weatherDesc);
    prefs.putBool("weatherLoaded", true);
    
    // Save 5-day forecast
    for (int i = 0; i < 5; i++) {
        char keyHigh[16], keyLow[16], keyIcon[16], keyDay[16];
        snprintf(keyHigh, sizeof(keyHigh), "fcHigh%d", i);
        snprintf(keyLow, sizeof(keyLow), "fcLow%d", i);
        snprintf(keyIcon, sizeof(keyIcon), "fcIcon%d", i);
        snprintf(keyDay, sizeof(keyDay), "fcDay%d", i);
        prefs.putFloat(keyHigh, forecast5Day[i].tempHigh);
        prefs.putFloat(keyLow, forecast5Day[i].tempLow);
        prefs.putString(keyIcon, forecast5Day[i].icon);
        prefs.putString(keyDay, forecast5Day[i].dayName);
    }
    prefs.end();
    lastSaveTime = millis();
    
    // Also backup time to SD for watchdog recovery
    saveTimeBackup();
}

void loadUserData() {
    prefs.begin("minios", true);
    userData.steps = prefs.getUInt("steps", 0);
    userData.dailyGoal = prefs.getUInt("dailyGoal", 10000);
    userData.brightness = prefs.getInt("brightness", 200);
    userData.themeIndex = prefs.getInt("themeIndex", 0);
    userData.wallpaperIndex = prefs.getInt("wallpaper", 0);
    userData.watchFaceIndex = prefs.getInt("watchFace", 0);  // Load watch face
    currentNikeColor = prefs.getInt("nikeColor", 0);  // Load Nike color variant
    currentPhotoIndex = prefs.getInt("photoIdx", 0);  // Load Photo face index
    userData.totalDistance = prefs.getFloat("distance", 0.0);
    userData.totalCalories = prefs.getFloat("calories", 0.0);
    userData.gamesWon = prefs.getInt("gamesWon", 0);
    userData.gamesPlayed = prefs.getInt("gamesPlayed", 0);
    batterySaverLevel = (BatterySaverLevel)prefs.getInt("batterySaverLvl", 0);  // Default to OFF mode
    use24HourFormat = prefs.getBool("use24Hr", true);  // NEW: Load 24hr format (default true)
    
    // Load cached weather data (persists until WiFi sync)
    weatherDataLoaded = prefs.getBool("weatherLoaded", false);
    if (weatherDataLoaded) {
        weatherTemp = prefs.getFloat("weatherTemp", 24.0);
        weatherHigh = prefs.getFloat("weatherHigh", 28.0);
        weatherLow = prefs.getFloat("weatherLow", 18.0);
        weatherDesc = prefs.getString("weatherDesc", "Sunny");
        
        // Load 5-day forecast
        for (int i = 0; i < 5; i++) {
            char keyHigh[16], keyLow[16], keyIcon[16], keyDay[16];
            snprintf(keyHigh, sizeof(keyHigh), "fcHigh%d", i);
            snprintf(keyLow, sizeof(keyLow), "fcLow%d", i);
            snprintf(keyIcon, sizeof(keyIcon), "fcIcon%d", i);
            snprintf(keyDay, sizeof(keyDay), "fcDay%d", i);
            forecast5Day[i].tempHigh = prefs.getFloat(keyHigh, forecast5Day[i].tempHigh);
            forecast5Day[i].tempLow = prefs.getFloat(keyLow, forecast5Day[i].tempLow);
            String iconStr = prefs.getString(keyIcon, forecast5Day[i].icon);
            strncpy(forecast5Day[i].icon, iconStr.c_str(), 15);
            String dayStr = prefs.getString(keyDay, forecast5Day[i].dayName);
            strncpy(forecast5Day[i].dayName, dayStr.c_str(), 3);
        }
        forecastLoaded = true;
        USBSerial.println("[WEATHER] Loaded cached weather data from Preferences");
    }
    prefs.end();
    
    // Validate indices
    if (userData.watchFaceIndex < 0 || userData.watchFaceIndex >= NUM_WATCH_FACES) {
        userData.watchFaceIndex = 0;
    }
    if (currentNikeColor >= NUM_NIKE_COLORS) {
        currentNikeColor = 0;
    }
    if (currentPhotoIndex >= MAX_SD_PHOTOS) {
        currentPhotoIndex = 0;
    }
    if ((int)batterySaverLevel < 0 || (int)batterySaverLevel > 2) {
        batterySaverLevel = BATTERY_SAVER_OFF;  // 3 modes: 0, 1, 2
    }
    batterySaverMode = (batterySaverLevel != BATTERY_SAVER_OFF);
    getSafeThemeIndex();
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR FUSION
// ═══════════════════════════════════════════════════════════════════════════
void updateSensorFusion() {
    if (!hasIMU) return;
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

    tiltX = atan2(acc.y, acc.z) * 180.0 / M_PI;
    tiltY = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0 / M_PI;

    float heading = atan2(acc.y, acc.x) * 180.0 / M_PI;
    if (heading < 0) heading += 360.0;
    heading += compassNorthOffset;
    if (heading >= 360) heading -= 360;
    if (heading < 0) heading += 360;

    compassHeadingSmooth = compassHeadingSmooth * 0.9 + heading * 0.1;
}

void updateStepCount() {
    if (!hasIMU) return;
    static float lastMag = 0;
    static bool stepDet = false;
    static unsigned long lastStepTime = 0;

    float mag = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    if (mag > 1.2 && lastMag <= 1.2 && !stepDet && millis() - lastStepTime > 250) {
        userData.steps++;
        userData.totalDistance += 0.0007;
        userData.totalCalories += 0.04;
        lastStepTime = millis();
        stepDet = true;
    }
    if (mag < 1.2) stepDet = false;
    lastMag = mag;
}

// ═══════════════════════════════════════════════════════════════════════════
// WEATHER FETCH
// ═══════════════════════════════════════════════════════════════════════════
void fetchWeatherData() {
    if (!wifiConnected) return;

    HTTPClient http;
    String url = "http://api.openweathermap.org/data/2.5/weather?q=";
    url += weatherCity;
    url += ",";
    url += weatherCountry;
    url += "&units=metric&appid=";
    url += OPENWEATHER_API;

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        DynamicJsonDocument doc(2048);
        deserializeJson(doc, payload);

        weatherTemp = doc["main"]["temp"];
        weatherHigh = doc["main"]["temp_max"];
        weatherLow = doc["main"]["temp_min"];
        weatherDesc = doc["weather"][0]["main"].as<String>();
    }

    http.end();
    lastWeatherUpdate = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// FETCH 5-DAY FORECAST FROM OPENWEATHERMAP API
// ═══════════════════════════════════════════════════════════════════════════
void fetch5DayForecast() {
    if (!wifiConnected) return;

    HTTPClient http;
    String url = "http://api.openweathermap.org/data/2.5/forecast?q=";
    url += weatherCity;
    url += ",";
    url += weatherCountry;
    url += "&units=metric&cnt=40&appid=";
    url += OPENWEATHER_API;

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        DynamicJsonDocument doc(16384);  // Larger buffer for forecast data
        DeserializationError error = deserializeJson(doc, payload);

        if (!error) {
            JsonArray list = doc["list"];
            const char* dayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
            
            // Get one forecast per day (every 8th entry = 24 hours)
            for (int i = 0; i < 5; i++) {
                int idx = i * 8;  // Every 24 hours (8 * 3 hour intervals)
                if (idx < (int)list.size()) {
                    JsonObject entry = list[idx];
                    
                    // Get day of week from timestamp
                    long dt = entry["dt"];
                    time_t timestamp = dt;
                    struct tm *timeinfo = localtime(&timestamp);
                    
                    strncpy(forecast5Day[i].dayName, dayNames[timeinfo->tm_wday], 3);
                    forecast5Day[i].dayName[3] = '\0';
                    
                    // Temperature
                    forecast5Day[i].tempHigh = entry["main"]["temp_max"];
                    forecast5Day[i].tempLow = entry["main"]["temp_min"];
                    
                    // Condition
                    const char* main = entry["weather"][0]["main"];
                    strncpy(forecast5Day[i].condition, main, 15);
                    forecast5Day[i].condition[15] = '\0';
                    
                    // Icon mapping
                    if (strstr(main, "Clear") || strstr(main, "Sun")) {
                        strcpy(forecast5Day[i].icon, "sun");
                    } else if (strstr(main, "Cloud")) {
                        strcpy(forecast5Day[i].icon, "cloud");
                    } else if (strstr(main, "Rain") || strstr(main, "Drizzle")) {
                        strcpy(forecast5Day[i].icon, "rain");
                    } else if (strstr(main, "Snow")) {
                        strcpy(forecast5Day[i].icon, "snow");
                    } else if (strstr(main, "Thunder")) {
                        strcpy(forecast5Day[i].icon, "storm");
                    } else {
                        strcpy(forecast5Day[i].icon, "cloud");
                    }
                }
            }
            forecastLoaded = true;
            USBSerial.println("[WEATHER] 5-day forecast loaded from API");
        }
    }

    http.end();
}

// ═══════════════════════════════════════════════════════════════════════════
// AUTO LOCATION FETCH FROM IP (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void fetchLocationFromIP() {
    if (!wifiConnected) return;

    HTTPClient http;
    http.begin("http://ip-api.com/json/?fields=city,country,timezone");
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        DynamicJsonDocument doc(512);
        deserializeJson(doc, payload);

        if (doc.containsKey("city")) {
            strncpy(weatherCity, doc["city"].as<const char*>(), sizeof(weatherCity) - 1);
        }
        if (doc.containsKey("country")) {
            strncpy(weatherCountry, doc["country"].as<const char*>(), sizeof(weatherCountry) - 1);
        }

        USBSerial.printf("[LOCATION] Detected: %s, %s\n", weatherCity, weatherCountry);
    }

    http.end();
    
    // Now fetch weather for detected location
    fetchWeatherData();
    fetch5DayForecast();  // Also fetch 5-day forecast
}

// ═══════════════════════════════════════════════════════════════════════════
// SD CARD FOLDER STRUCTURE (FROM 206q - FUSION LABS COMPATIBLE)
// ═══════════════════════════════════════════════════════════════════════════
bool createDirectoryIfNotExists(const char* path) {
    if (!SD_MMC.exists(path)) {
        if (SD_MMC.mkdir(path)) {
            USBSerial.printf("[SD] Created: %s\n", path);
            return true;
        } else {
            USBSerial.printf("[SD] Failed to create: %s\n", path);
            return false;
        }
    }
    return true;
}

void logToBootLog(const char* message) {
    if (!hasSD) return;
    File file = SD_MMC.open(SD_BOOT_LOG, FILE_APPEND);
    if (file) {
        file.printf("[%lu] %s\n", millis(), message);
        file.close();
    }
}

bool createWidgetOSFolderStructure() {
    bool success = true;

    // Root
    success &= createDirectoryIfNotExists(SD_ROOT_PATH);

    // SYSTEM
    success &= createDirectoryIfNotExists(SD_SYSTEM_PATH);
    success &= createDirectoryIfNotExists(SD_SYSTEM_LOGS_PATH);

    // CONFIG
    success &= createDirectoryIfNotExists(SD_CONFIG_PATH);

    // FACES
    success &= createDirectoryIfNotExists(SD_FACES_PATH);
    success &= createDirectoryIfNotExists(SD_FACES_CUSTOM_PATH);
    success &= createDirectoryIfNotExists(SD_FACES_IMPORTED_PATH);

    // IMAGES
    success &= createDirectoryIfNotExists(SD_IMAGES_PATH);

    // PHOTOS (for Photo clockface)
    success &= createDirectoryIfNotExists(SD_PHOTOS_PATH);

    // MUSIC
    success &= createDirectoryIfNotExists(SD_MUSIC_PATH);

    // CACHE
    success &= createDirectoryIfNotExists(SD_CACHE_PATH);
    success &= createDirectoryIfNotExists(SD_CACHE_TEMP_PATH);

    // UPDATE
    success &= createDirectoryIfNotExists(SD_UPDATE_PATH);

    // WiFi
    success &= createDirectoryIfNotExists(SD_WIFI_PATH);

    // BACKUP (Fusion Labs)
    success &= createDirectoryIfNotExists(SD_BACKUP_PATH);

    // FIRMWARE (Fusion Labs)
    success &= createDirectoryIfNotExists(SD_FIRMWARE_PATH);

    // LOGS
    success &= createDirectoryIfNotExists(SD_LOGS_PATH);

    // WALLPAPERS
    success &= createDirectoryIfNotExists(SD_WALLPAPERS_PATH);
    
    // NEW: Power consumption logs folder
    success &= createDirectoryIfNotExists(SD_POWER_LOGS_PATH);

    if (!success) {
        sdCardStatus = SD_STATUS_CORRUPT;
        return false;
    }

    // Create boot log
    File bootLog = SD_MMC.open(SD_BOOT_LOG, FILE_WRITE);
    if (bootLog) {
        bootLog.println("═══════════════════════════════════════════════════════════════════");
        bootLog.printf("  %s %s - Boot Log\n", WIDGET_OS_NAME, WIDGET_OS_VERSION);
        bootLog.printf("  Device: %s\n", DEVICE_ID);
        bootLog.println("═══════════════════════════════════════════════════════════════════");
        bootLog.close();
    }

    // Create default config.txt if not exists (Fusion Labs compatible)
    if (!SD_MMC.exists(SD_CONFIG_TXT)) {
        File cfg = SD_MMC.open(SD_CONFIG_TXT, FILE_WRITE);
        if (cfg) {
            cfg.println("# Widget OS Configuration");
            cfg.println("# Fusion Labs Compatible v1.0");
            cfg.println("");
            cfg.println("[device]");
            cfg.println("name=Widget OS Watch");
            cfg.println("board=2.06");
            cfg.println("");
            cfg.println("[backup]");
            cfg.println("auto_backup=true");
            cfg.println("backup_interval_hours=24");
            cfg.println("");
            cfg.println("[display]");
            cfg.printf("brightness=%d\n", userData.brightness);
            cfg.println("screen_timeout=30");
            cfg.println("");
            cfg.println("[wifi]");
            cfg.println("# Add WiFi networks here:");
            cfg.println("# ssid=YourNetwork");
            cfg.println("# password=YourPassword");
            cfg.close();
            USBSerial.println("[SD] Created config.txt");
        }
    }

    // Create WiFi config template with 5 network slots
    if (!SD_MMC.exists(SD_WIFI_CONFIG)) {
        File wifiCfg = SD_MMC.open(SD_WIFI_CONFIG, FILE_WRITE);
        if (wifiCfg) {
            wifiCfg.println("# Widget OS WiFi Configuration");
            wifiCfg.println("# Supports up to 5 WiFi networks");
            wifiCfg.println("");
            wifiCfg.println("# Network 1 (Primary)");
            wifiCfg.println("SSID1=YourHomeWiFi");
            wifiCfg.println("PASSWORD1=YourPassword");
            wifiCfg.println("OPEN1=false");
            wifiCfg.println("");
            wifiCfg.println("# Network 2");
            wifiCfg.println("SSID2=");
            wifiCfg.println("PASSWORD2=");
            wifiCfg.println("OPEN2=false");
            wifiCfg.println("");
            wifiCfg.println("# Network 3");
            wifiCfg.println("SSID3=");
            wifiCfg.println("PASSWORD3=");
            wifiCfg.println("OPEN3=false");
            wifiCfg.println("");
            wifiCfg.println("# Network 4");
            wifiCfg.println("SSID4=");
            wifiCfg.println("PASSWORD4=");
            wifiCfg.println("OPEN4=false");
            wifiCfg.println("");
            wifiCfg.println("# Network 5");
            wifiCfg.println("SSID5=");
            wifiCfg.println("PASSWORD5=");
            wifiCfg.println("OPEN5=false");
            wifiCfg.println("");
            wifiCfg.println("# Weather Location");
            wifiCfg.println("CITY=Perth");
            wifiCfg.println("COUNTRY=AU");
            wifiCfg.println("");
            wifiCfg.println("# Timezone (hours from GMT)");
            wifiCfg.println("GMT_OFFSET=8");
            wifiCfg.close();
            USBSerial.println("[SD] Created wifi/config.txt with 5 network slots");
        }
    }

    sdStructureCreated = true;
    logToBootLog("Widget OS folder structure created");
    return true;
}

bool initWidgetOSSDCard() {
    sdCardStatus = SD_STATUS_INIT_IN_PROGRESS;

    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);

    if (!SD_MMC.begin("/sdcard", true, true)) {
        sdCardStatus = SD_STATUS_MOUNT_FAILED;
        sdErrorMessage = "SD card mount failed";
        hasSD = false;
        return false;
    }

    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        sdCardStatus = SD_STATUS_NOT_PRESENT;
        sdErrorMessage = "No SD card detected";
        hasSD = false;
        return false;
    }

    switch(cardType) {
        case CARD_MMC:  sdCardType = "MMC"; break;
        case CARD_SD:   sdCardType = "SDSC"; break;
        case CARD_SDHC: sdCardType = "SDHC"; break;
        default:        sdCardType = "UNKNOWN"; break;
    }

    sdCardSizeMB = SD_MMC.cardSize() / (1024 * 1024);
    sdCardInitialized = true;
    hasSD = true;
    sdCardStatus = SD_STATUS_MOUNTED_OK;

    createWidgetOSFolderStructure();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI CONFIG FROM SD (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
bool loadWiFiConfigFromSD() {
    if (!hasSD || !sdCardInitialized) return false;

    if (!SD_MMC.exists(SD_WIFI_CONFIG)) return false;

    File file = SD_MMC.open(SD_WIFI_CONFIG, FILE_READ);
    if (!file) return false;

    // DON'T reset networks - keep hardcoded in slot 0!
    // SD networks will be added starting from slot 1
    int sdNetworkCount = 0;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();

        if (line.length() == 0 || line.startsWith("#")) continue;

        int eqPos = line.indexOf('=');
        if (eqPos <= 0) continue;

        String key = line.substring(0, eqPos);
        String value = line.substring(eqPos + 1);
        key.trim();
        value.trim();

        // Support for SD WiFi networks: SSID1-SSID4, PASSWORD1-PASSWORD4
        // These map to slots 1-4 (slot 0 is hardcoded)
        for (int slot = 1; slot <= 4; slot++) {
            char ssidKey[16], passKey[16], openKey[16];
            snprintf(ssidKey, sizeof(ssidKey), "SSID%d", slot);
            snprintf(passKey, sizeof(passKey), "PASSWORD%d", slot);
            snprintf(openKey, sizeof(openKey), "OPEN%d", slot);
            
            // Map SD slot 1-4 to array index 1-4 (0 is hardcoded)
            int idx = slot;
            
            if (key == ssidKey && idx < MAX_WIFI_NETWORKS) {
                strncpy(wifiNetworks[idx].ssid, value.c_str(), sizeof(wifiNetworks[idx].ssid) - 1);
                wifiNetworks[idx].valid = true;
                sdNetworkCount++;
                if (idx >= numWifiNetworks) numWifiNetworks = idx + 1;
            }
            else if (key == passKey && idx < MAX_WIFI_NETWORKS) {
                strncpy(wifiNetworks[idx].password, value.c_str(), sizeof(wifiNetworks[idx].password) - 1);
            }
            else if (key == openKey && idx < MAX_WIFI_NETWORKS) {
                wifiNetworks[idx].isOpen = (value == "1" || value.equalsIgnoreCase("true"));
            }
        }
        
        // Legacy single network support (SSID, PASSWORD without number) -> slot 1
        if (key == "SSID") {
            strncpy(wifiNetworks[1].ssid, value.c_str(), sizeof(wifiNetworks[1].ssid) - 1);
            wifiNetworks[1].valid = true;
            sdNetworkCount++;
            if (numWifiNetworks < 2) numWifiNetworks = 2;
        }
        else if (key == "PASSWORD") {
            strncpy(wifiNetworks[1].password, value.c_str(), sizeof(wifiNetworks[1].password) - 1);
        }
        else if (key == "CITY") {
            strncpy(weatherCity, value.c_str(), sizeof(weatherCity) - 1);
        }
        else if (key == "COUNTRY") {
            strncpy(weatherCountry, value.c_str(), sizeof(weatherCountry) - 1);
        }
        else if (key == "GMT_OFFSET") {
            gmtOffsetSec = value.toInt() * 3600;
        }
    }

    file.close();

    if (sdNetworkCount > 0) {
        wifiConfigFromSD = true;
        logToBootLog("Loaded WiFi config from SD card");
        char logMsg[64];
        snprintf(logMsg, sizeof(logMsg), "[SD] Loaded %d WiFi networks from SD (slots 1-%d)", 
                 sdNetworkCount, numWifiNetworks - 1);
        USBSerial.println(logMsg);
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// BACKUP SYSTEM (FROM 206q - FUSION LABS COMPATIBLE)
// ═══════════════════════════════════════════════════════════════════════════
bool createBackup(bool isAuto) {
    if (!hasSD) return false;

    // Create timestamp for backup folder
    char backupDir[64];
    RTC_DateTime dt = rtc.getDateTime();
    snprintf(backupDir, sizeof(backupDir), "%s/%04d%02d%02d_%02d%02d%02d%s",
             SD_BACKUP_PATH,
             dt.getYear(), dt.getMonth(), dt.getDay(),
             dt.getHour(), dt.getMinute(), dt.getSecond(),
             isAuto ? "_auto" : "_manual");

    if (!SD_MMC.mkdir(backupDir)) {
        sdHealth.writeErrors++;
        return false;
    }

    // Save user data
    String userDataPath = String(backupDir) + "/user_data.json";
    File userFile = SD_MMC.open(userDataPath.c_str(), FILE_WRITE);
    if (userFile) {
        DynamicJsonDocument doc(1024);
        doc["steps"] = userData.steps;
        doc["daily_goal"] = userData.dailyGoal;
        doc["distance"] = userData.totalDistance;
        doc["calories"] = userData.totalCalories;
        doc["games_won"] = userData.gamesWon;
        doc["games_played"] = userData.gamesPlayed;
        doc["brightness"] = userData.brightness;
        doc["theme"] = userData.themeIndex;
        doc["wallpaper"] = userData.wallpaperIndex;
        doc["backup_time"] = millis();
        doc["is_auto"] = isAuto;

        serializeJsonPretty(doc, userFile);
        userFile.close();
    }

    // Save compass calibration
    String compassPath = String(backupDir) + "/compass_calibration.txt";
    File compassFile = SD_MMC.open(compassPath.c_str(), FILE_WRITE);
    if (compassFile) {
        compassFile.println(compassNorthOffset);
        compassFile.close();
    }

    totalBackups++;
    lastBackupTime = millis();
    logToBootLog(isAuto ? "Auto backup created" : "Manual backup created");
    USBSerial.printf("[BACKUP] Created: %s\n", backupDir);
    return true;
}

bool restoreFromBackup(const String& backupName) {
    if (!hasSD) return false;

    String backupDir = String(SD_BACKUP_PATH) + "/" + backupName;

    if (!SD_MMC.exists(backupDir.c_str())) {
        USBSerial.println("[RESTORE] Backup not found");
        return false;
    }

    // Restore user data
    String userDataPath = backupDir + "/user_data.json";
    File userFile = SD_MMC.open(userDataPath.c_str(), FILE_READ);
    if (userFile) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, userFile);
        userFile.close();

        if (!error) {
            userData.steps = doc["steps"] | 0;
            userData.dailyGoal = doc["daily_goal"] | 10000;
            userData.totalDistance = doc["distance"] | 0.0;
            userData.totalCalories = doc["calories"] | 0.0;
            userData.gamesWon = doc["games_won"] | 0;
            userData.gamesPlayed = doc["games_played"] | 0;
            userData.brightness = doc["brightness"] | 200;
            userData.themeIndex = doc["theme"] | 0;
            userData.wallpaperIndex = doc["wallpaper"] | 0;

            gfx->setBrightness(userData.brightness);
            saveUserData();
        }
    }

    // Restore compass calibration
    String compassPath = backupDir + "/compass_calibration.txt";
    File compassFile = SD_MMC.open(compassPath.c_str(), FILE_READ);
    if (compassFile) {
        String line = compassFile.readStringUntil('\n');
        compassNorthOffset = line.toFloat();
        compassFile.close();

        prefs.begin("minios", false);
        prefs.putFloat("compassOffset", compassNorthOffset);
        prefs.end();
    }

    logToBootLog("Restored from backup");
    USBSerial.printf("[RESTORE] Restored from: %s\n", backupName.c_str());
    return true;
}

void checkAutoBackup() {
    if (!autoBackupEnabled || !hasSD) return;

    unsigned long currentTime = millis();
    if (currentTime - lastAutoBackup >= AUTO_BACKUP_INTERVAL_MS) {
        if (createBackup(true)) {
            lastAutoBackup = currentTime;
        }
    }
}

String listBackups() {
    String result = "";
    if (!hasSD) return "NO_SD";

    File dir = SD_MMC.open(SD_BACKUP_PATH);
    if (!dir || !dir.isDirectory()) return "NO_BACKUPS";

    File file = dir.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            result += file.name();
            result += "\n";
        }
        file = dir.openNextFile();
    }
    dir.close();

    return result.length() > 0 ? result : "NO_BACKUPS";
}

// ═══════════════════════════════════════════════════════════════════════════
// FUSION LABS WEB SERIAL PROTOCOL (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void sendDeviceStatus() {
    DynamicJsonDocument doc(1024);
    doc["type"] = "WIDGET_STATUS_RESPONSE";
    doc["firmware"] = WIDGET_OS_VERSION;
    doc["protocol"] = FUSION_PROTOCOL_VERSION;
    doc["board"] = DEVICE_SCREEN;
    doc["display"] = "CO5300";
    doc["battery"] = hasPMU ? (int)power.getBatteryPercent() : -1;
    doc["charging"] = hasPMU ? power.isCharging() : false;
    doc["wifi"] = wifiConnected;
    doc["sd_mounted"] = hasSD;
    doc["brightness"] = userData.brightness;
    doc["theme"] = userData.themeIndex;
    doc["wallpaper"] = userData.wallpaperIndex;
    doc["steps"] = userData.steps;
    doc["uptime_ms"] = millis();

    String output;
    serializeJson(doc, output);
    USBSerial.println(output);
}

void sendSDHealth() {
    updateSDCardHealth();

    DynamicJsonDocument doc(512);
    doc["type"] = "WIDGET_SD_HEALTH_RESPONSE";
    doc["mounted"] = sdHealth.mounted;
    doc["healthy"] = sdHealth.healthy;
    doc["total_mb"] = sdHealth.totalBytes / (1024 * 1024);
    doc["used_mb"] = sdHealth.usedBytes / (1024 * 1024);
    doc["free_mb"] = sdHealth.freeBytes / (1024 * 1024);
    doc["used_percent"] = sdHealth.usedPercent;
    doc["write_errors"] = sdHealth.writeErrors;
    doc["read_errors"] = sdHealth.readErrors;
    doc["auto_backup"] = autoBackupEnabled;
    doc["total_backups"] = totalBackups;

    String output;
    serializeJson(doc, output);
    USBSerial.println(output);
}

String readConfigFromSD() {
    if (!hasSD) return "NO_SD";

    File cfg = SD_MMC.open(SD_CONFIG_TXT, FILE_READ);
    if (!cfg) return "NO_CONFIG";

    String content = cfg.readString();
    cfg.close();
    return content;
}

bool saveConfigToSD(const String& configData) {
    if (!hasSD) return false;

    File cfg = SD_MMC.open(SD_CONFIG_TXT, FILE_WRITE);
    if (!cfg) {
        sdHealth.writeErrors++;
        return false;
    }

    cfg.print(configData);
    cfg.close();
    return true;
}

void processWebSerialCommand(const String& cmd) {
    String trimmedCmd = cmd;
    trimmedCmd.trim();

    if (trimmedCmd == "WIDGET_PING") {
        USBSerial.println("WIDGET_PONG");
    }
    else if (trimmedCmd == "WIDGET_STATUS") {
        sendDeviceStatus();
    }
    else if (trimmedCmd == "WIDGET_SD_HEALTH") {
        sendSDHealth();
    }
    else if (trimmedCmd == "WIDGET_READ_CONFIG") {
        String config = readConfigFromSD();
        USBSerial.println(config);
    }
    else if (trimmedCmd == "WIDGET_BACKUP") {
        bool success = createBackup(false);
        USBSerial.println(success ? "BACKUP_OK" : "BACKUP_FAIL");
    }
    else if (trimmedCmd.startsWith("WIDGET_RESTORE:")) {
        String backupName = trimmedCmd.substring(15);
        bool success = restoreFromBackup(backupName);
        USBSerial.println(success ? "RESTORE_OK" : "RESTORE_FAIL");
    }
    else if (trimmedCmd == "WIDGET_LIST_BACKUPS") {
        String backups = listBackups();
        USBSerial.println(backups);
    }
    else if (trimmedCmd == "WIDGET_TOGGLE_AUTO_BACKUP") {
        autoBackupEnabled = !autoBackupEnabled;
        USBSerial.printf("AUTO_BACKUP_%s\n", autoBackupEnabled ? "ON" : "OFF");
    }
    else if (trimmedCmd == "WIDGET_REBOOT") {
        USBSerial.println("REBOOTING...");
        delay(100);
        ESP.restart();
    }
}

void handleFusionLabsProtocol() {
    while (USBSerial.available()) {
        char c = USBSerial.read();

        if (c == '\n') {
            webSerialBuffer[webSerialBufferIndex] = '\0';
            String cmd = String(webSerialBuffer);
            webSerialBufferIndex = 0;

            processWebSerialCommand(cmd);
        } else if (webSerialBufferIndex < WEB_SERIAL_BUFFER_SIZE - 1) {
            webSerialBuffer[webSerialBufferIndex++] = c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// UI EVENT HANDLER
// ═══════════════════════════════════════════════════════════════════════════
void handle_ui_event(UIEventType event, int param1, int param2) {
    if (event != UI_EVENT_NONE && event != UI_EVENT_REFRESH) {
        USBSerial.printf("[UI_EVENT] %d (canNav=%d, isTrans=%d)\n", event, canNavigate(), isTransitioning);
    }
    switch (event) {
        case UI_EVENT_NAV_LEFT: if (canNavigate()) handleSwipe(50, 0); break;
        case UI_EVENT_NAV_RIGHT: if (canNavigate()) handleSwipe(-50, 0); break;
        case UI_EVENT_NAV_UP: if (canNavigate()) handleSwipe(0, 50); break;
        case UI_EVENT_NAV_DOWN: if (canNavigate()) handleSwipe(0, -50); break;
        case UI_EVENT_TAP: handleTap(param1, param2); break;
        case UI_EVENT_SCREEN_ON: screenOnFunc(); break;
        case UI_EVENT_SCREEN_OFF: screenOff(); break;
        case UI_EVENT_REFRESH: if (screenOn) navigateTo(currentCategory, currentSubCard); break;
        case UI_EVENT_SHUTDOWN: shutdownDevice(); break;
        default: break;
    }
    last_lvgl_response = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// UI TASK (FROM FIXED - STABLE)
// ═══════════════════════════════════════════════════════════════════════════
void ui_task(void *pvParameters) {
    USBSerial.println("[UI_TASK] Started on Core 1");

    esp_err_t wdt_err = esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    if (wdt_err != ESP_OK) {
        USBSerial.printf("[WDT] Warning: Failed to add ui_task: %d\n", wdt_err);
    } else {
        USBSerial.println("[WDT] ui_task registered with watchdog");
    }

    while (true) {
        esp_task_wdt_reset();
        last_lvgl_response = millis();

        lv_task_handler();

        if (ui_event != UI_EVENT_NONE) {
            UIEventType evt = ui_event;
            int p1 = ui_event_param1, p2 = ui_event_param2;

            ui_event = UI_EVENT_NONE;
            ui_event_param1 = ui_event_param2 = 0;

            handle_ui_event(evt, p1, p2);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void panic_recover() {
    USBSerial.println("[PANIC] Attempting LVGL recovery...");

    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1C1C1E), 0);
    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text(lbl, "Recovering...");
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFF6B6B), 0);
    lv_obj_center(lbl);
    lv_scr_load(scr);

    isTransitioning = false;
    navigationLocked = false;
    currentCategory = CAT_CLOCK;
    currentSubCard = 0;

    USBSerial.println("[PANIC] Recovery complete, returning to clock");

    lv_timer_create([](lv_timer_t *t) {
        navigateTo(CAT_CLOCK, 0);
        lv_timer_del(t);
    }, 500, NULL);
}

void backlight_manager() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 500) return;
    lastCheck = millis();

    // Use timeout from current battery saver mode
    unsigned long timeout = saverModes[batterySaverLevel].screenTimeoutMs;
    if (screenOn && millis() - last_ui_activity > timeout) {
        ui_event = UI_EVENT_SCREEN_OFF;
    }
}

void check_lvgl_stall() {
    if (millis() - last_lvgl_response > LVGL_STALL_TIMEOUT_MS) {
        USBSerial.println("[STALL] LVGL stall detected! Initiating recovery...");
        panic_recover();
        last_lvgl_response = millis();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GADGETBRIDGE BLE - SERVER CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════
class GadgetbridgeServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        bleDeviceConnected = true;
        bleLastActivity = millis();
        // Get connected device info
        bleConnectedDeviceName = "Gadgetbridge";  // Default name
        USBSerial.println("[BLE] ✓ Device connected!");
        USBSerial.println("[BLE] Auto-off timer cancelled (device connected)");
        
        // Flash screen to confirm connection (no vibration motor)
        for (int i = 0; i < 2; i++) {
            gfx->setBrightness(255);
            delay(80);
            gfx->setBrightness(100);
            delay(80);
        }
        gfx->setBrightness(saverModes[batterySaverLevel].brightness);
    }

    void onDisconnect(BLEServer* pServer) {
        bleDeviceConnected = false;
        bleConnectedDeviceName = "";
        USBSerial.println("[BLE] Device disconnected");
        // Always restart advertising for auto-reconnect
        if (bleEnabled) {
            delay(500);
            pServer->startAdvertising();
            USBSerial.println("[BLE] Advertising restarted (always-on mode)");
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// GADGETBRIDGE BLE - CHARACTERISTIC CALLBACKS (Receive data)
// ═══════════════════════════════════════════════════════════════════════════
class GadgetbridgeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str();
        
        if (rxValue.length() > 0) {
            bleLastActivity = millis();
            
            // Accumulate data (messages may come in chunks)
            for (int i = 0; i < rxValue.length(); i++) {
                char c = rxValue[i];
                if (c == '\n' || c == '\r') {
                    if (bleIncomingBuffer.length() > 0) {
                        USBSerial.printf("[BLE] Received: %s\n", bleIncomingBuffer.c_str());
                        handleGadgetbridgeMessage(bleIncomingBuffer);
                        bleIncomingBuffer = "";
                    }
                } else {
                    bleIncomingBuffer += c;
                }
            }
            
            // Handle message without newline (some clients don't send it)
            if (bleIncomingBuffer.length() > 0 && bleIncomingBuffer.startsWith("{")) {
                // Check if it's a complete JSON
                int braceCount = 0;
                bool inString = false;
                for (int i = 0; i < bleIncomingBuffer.length(); i++) {
                    char c = bleIncomingBuffer[i];
                    if (c == '"' && (i == 0 || bleIncomingBuffer[i-1] != '\\')) inString = !inString;
                    if (!inString) {
                        if (c == '{') braceCount++;
                        if (c == '}') braceCount--;
                    }
                }
                if (braceCount == 0) {
                    USBSerial.printf("[BLE] Received JSON: %s\n", bleIncomingBuffer.c_str());
                    handleGadgetbridgeMessage(bleIncomingBuffer);
                    bleIncomingBuffer = "";
                }
            }
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// GADGETBRIDGE PROTOCOL HANDLER (Bangle.js Compatible)
// Protocol: JSON over Nordic UART, wrapped in GB({...})\n
// Ref: https://www.espruino.com/Gadgetbridge
// ═══════════════════════════════════════════════════════════════════════════

// Music state storage
struct MusicState {
    String artist = "";
    String album = "";
    String track = "";
    int duration = 0;
    int position = 0;
    bool playing = false;
    int trackCount = 0;
    int trackNum = 0;
} musicState;

// Call state storage
struct CallState {
    String name = "";
    String number = "";
    bool incoming = false;
    bool active = false;
} callState;

// Weather storage (improved)
struct WeatherData {
    int temp = 0;           // Current temp
    int tempHigh = 0;       // Day high
    int tempLow = 0;        // Day low
    int humidity = 0;       // Humidity %
    int rain = 0;           // Rain probability %
    int uvIndex = 0;        // UV index
    int code = 0;           // Weather code
    String text = "";       // Condition text
    float windSpeed = 0;    // Wind speed
    String windDir = "";    // Wind direction
    String location = "";   // Location name
    bool valid = false;
} weatherData;

// Activity tracking flags for realtime
bool realtimeHRM = false;
bool realtimeSteps = false;
int realtimeInterval = 60;  // Default 60 seconds

// Calendar events storage (max 10)
#define MAX_CALENDAR_EVENTS 10
struct CalendarEvent {
    int id = -1;
    unsigned long timestamp = 0;
    int duration = 0;
    String title = "";
    String description = "";
    String location = "";
    bool allDay = false;
    bool valid = false;
} calendarEvents[MAX_CALENDAR_EVENTS];
int calendarEventCount = 0;

// Navigation state
struct NavState {
    String instruction = "";
    String distance = "";
    String action = "";
    String eta = "";
    bool active = false;
} navState;

// GPS state from phone
struct PhoneGPS {
    float lat = 0;
    float lon = 0;
    float alt = 0;
    float speed = 0;
    float course = 0;
    int satellites = 0;
    bool valid = false;
    bool enabled = false;
} phoneGPS;

// Alarm storage (max 5)
#define MAX_ALARMS 5
struct AlarmData {
    int hour = 0;
    int minute = 0;
    int repeatMask = 0;  // Binary mask: bit0=Sun, bit1=Mon...bit6=Sat
    bool enabled = true;
    bool valid = false;
} alarms[MAX_ALARMS];
int alarmCount = 0;

void handleGadgetbridgeMessage(String message) {
    message.trim();
    
    // ═══ EXTRACT JSON FROM GB() WRAPPER ═══
    // Gadgetbridge sends: GB({...})\n
    if (message.startsWith("GB(") || message.startsWith("\x10GB(")) {
        int start = message.indexOf("{");
        int end = message.lastIndexOf("}");
        if (start >= 0 && end > start) {
            message = message.substring(start, end + 1);
        }
        USBSerial.printf("[GB] Received: %s\n", message.substring(0, min((int)message.length(), 100)).c_str());
    }
    
    // ═══ HANDLE PLAIN TEXT COMMANDS ═══
    if (message.indexOf("getHealthStatus") >= 0) {
        char response[256];
        snprintf(response, sizeof(response), 
            "{\"steps\":%lu,\"bpm\":%d,\"movement\":%d,\"bpmConfidence\":100}", 
            (unsigned long)userData.steps, lastHeartRate, activityIntensity);
        sendBLEResponse(response);
        return;
    }
    
    if (message.indexOf("getBattery") >= 0) {
        char response[64];
        snprintf(response, sizeof(response), "{\"level\":%d,\"charging\":%s}", 
            batteryPercent, isCharging ? "true" : "false");
        sendBLEResponse(response);
        return;
    }
    
    // ═══ PARSE JSON MESSAGE ═══
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        return;  // Not valid JSON
    }
    
    const char* msgType = doc["t"];
    if (!msgType) return;
    
    USBSerial.printf("[GB] Type: %s\n", msgType);
    
    // ═══════════════════════════════════════════════════════════════════════
    // TIME SYNC - t:"setTime"
    // ═══════════════════════════════════════════════════════════════════════
    if (strcmp(msgType, "setTime") == 0) {
        unsigned long unixTime = doc["sec"] | 0;
        int tzOffset = doc["tz"] | 0;
        
        if (unixTime > 0 && hasRTC) {
            time_t rawTime = unixTime;
            struct tm *ti = localtime(&rawTime);
            
            rtc.setDateTime(ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
                           ti->tm_hour, ti->tm_min, ti->tm_sec);
            
            clockHour = ti->tm_hour;
            clockMinute = ti->tm_min;
            clockSecond = ti->tm_sec;
            bleTimeSynced = true;
            bleLastActivityTime = millis();
            
            saveTimeBackup();
            USBSerial.printf("[GB] Time synced: %02d:%02d:%02d (tz:%d)\n", 
                clockHour, clockMinute, clockSecond, tzOffset);
            
            // Send firmware version as confirmation (per Bangle.js protocol)
            sendBLEResponse("{\"t\":\"ver\",\"fw\":\"MiniOS 7.4.1\",\"hw\":\"ESP32-S3\"}");
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // NOTIFICATIONS - t:"notify", t:"notify-", t:"notify~"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "notify") == 0) {
        int notifyId = doc["id"] | 0;
        const char* src = doc["src"] | "App";
        const char* title = doc["title"] | "";
        const char* body = doc["body"] | "";
        const char* sender = doc["sender"] | "";
        const char* tel = doc["tel"] | "";
        bool canReply = doc["reply"] | false;
        
        // Use sender if available, otherwise title
        String displayTitle = strlen(sender) > 0 ? String(sender) : String(title);
        addNotification(String(src), displayTitle, String(body));
        
        bleLastActivityTime = millis();
        wakeScreenForNotification();
        
        USBSerial.printf("[GB] Notify [%d]: %s - %s\n", notifyId, src, title);
    }
    else if (strcmp(msgType, "notify-") == 0) {
        // Delete notification
        int notifyId = doc["id"] | 0;
        USBSerial.printf("[GB] Delete notification: %d\n", notifyId);
        // Could implement notification removal by ID here
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // ALARMS - t:"alarm"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "alarm") == 0) {
        JsonArray alarmArray = doc["d"].as<JsonArray>();
        alarmCount = 0;
        
        for (JsonObject alarm : alarmArray) {
            if (alarmCount >= MAX_ALARMS) break;
            
            alarms[alarmCount].hour = alarm["h"] | 0;
            alarms[alarmCount].minute = alarm["m"] | 0;
            alarms[alarmCount].repeatMask = alarm["rep"] | 0;
            alarms[alarmCount].enabled = alarm["on"] | true;
            alarms[alarmCount].valid = true;
            alarmCount++;
        }
        
        USBSerial.printf("[GB] Received %d alarms\n", alarmCount);
        // Save alarms to preferences here if needed
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // FIND DEVICE - t:"find"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "find") == 0) {
        bool findOn = doc["n"] | false;
        
        if (findOn) {
            USBSerial.println("[GB] Find device activated!");
            // Wake screen and flash brightly
            screenOn = true;
            gfx->displayOn();
            
            for (int i = 0; i < 5; i++) {
                gfx->setBrightness(255);
                delay(300);
                gfx->setBrightness(30);
                delay(300);
            }
            gfx->setBrightness(saverModes[batterySaverLevel].brightness);
            
            // Send findPhone back to phone (optional - makes phone ring too)
            // sendBLEResponse("{\"t\":\"findPhone\",\"n\":true}");
        }
        sendBLEResponse("{\"t\":\"find\",\"n\":false}");
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // VIBRATE - t:"vibrate"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "vibrate") == 0) {
        int pattern = doc["n"] | 1;
        
        // Flash screen since we have no vibration motor
        for (int i = 0; i < pattern && i < 5; i++) {
            gfx->setBrightness(255);
            delay(100);
            gfx->setBrightness(50);
            delay(100);
        }
        gfx->setBrightness(saverModes[batterySaverLevel].brightness);
        USBSerial.printf("[GB] Vibrate pattern: %d (flashed screen)\n", pattern);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // WEATHER - t:"weather"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "weather") == 0) {
        weatherData.temp = doc["temp"] | 0;
        weatherData.tempHigh = doc["hi"] | 0;
        weatherData.tempLow = doc["lo"] | 0;
        weatherData.humidity = doc["hum"] | 0;
        weatherData.rain = doc["rain"] | 0;
        weatherData.uvIndex = doc["uv"] | 0;
        weatherData.code = doc["code"] | 0;
        weatherData.text = doc["txt"] | "Unknown";
        weatherData.windSpeed = doc["wind"] | 0.0f;
        weatherData.windDir = doc["wdir"] | "";
        weatherData.location = doc["loc"] | "";
        weatherData.valid = true;
        
        USBSerial.printf("[GB] Weather: %d°, %s, %s\n", 
            weatherData.temp, weatherData.text.c_str(), weatherData.location.c_str());
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // MUSIC STATE - t:"musicstate"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "musicstate") == 0) {
        const char* state = doc["state"] | "";
        musicState.playing = (strcmp(state, "play") == 0);
        musicState.position = doc["position"] | 0;
        
        USBSerial.printf("[GB] Music: %s, pos=%d\n", state, musicState.position);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // MUSIC INFO - t:"musicinfo"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "musicinfo") == 0) {
        musicState.artist = doc["artist"] | "";
        musicState.album = doc["album"] | "";
        musicState.track = doc["track"] | "";
        musicState.duration = doc["dur"] | 0;
        musicState.trackCount = doc["c"] | 0;
        musicState.trackNum = doc["n"] | 0;
        
        USBSerial.printf("[GB] Now playing: %s - %s\n", 
            musicState.artist.c_str(), musicState.track.c_str());
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // CALL - t:"call"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "call") == 0) {
        const char* cmd = doc["cmd"] | "";
        callState.name = doc["name"] | "Unknown";
        callState.number = doc["number"] | "";
        
        if (strcmp(cmd, "incoming") == 0) {
            callState.incoming = true;
            callState.active = false;
            addNotification("Phone", "Incoming Call", callState.name);
            wakeScreenForNotification();
            USBSerial.printf("[GB] Incoming call: %s\n", callState.name.c_str());
        }
        else if (strcmp(cmd, "outgoing") == 0) {
            callState.incoming = false;
            callState.active = true;
            USBSerial.printf("[GB] Outgoing call: %s\n", callState.name.c_str());
        }
        else if (strcmp(cmd, "start") == 0) {
            callState.active = true;
            USBSerial.println("[GB] Call started");
        }
        else if (strcmp(cmd, "end") == 0 || strcmp(cmd, "reject") == 0) {
            callState.active = false;
            callState.incoming = false;
            USBSerial.println("[GB] Call ended");
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // ACTIVITY TRACKING - t:"act"
    // Enable/disable realtime HR and step counting
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "act") == 0) {
        realtimeHRM = doc["hrm"] | false;
        realtimeSteps = doc["stp"] | false;
        realtimeInterval = doc["int"] | 60;
        
        USBSerial.printf("[GB] Activity tracking: HRM=%d, Steps=%d, Interval=%ds\n", 
            realtimeHRM, realtimeSteps, realtimeInterval);
        
        // If realtime enabled, start sending activity data periodically
        if (realtimeHRM || realtimeSteps) {
            sendActivityData(false);  // Send initial data, not realtime flag
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // ACTIVITY FETCH - t:"actfetch"
    // Request historical activity data
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "actfetch") == 0) {
        unsigned long since = doc["ts"] | 0;
        USBSerial.printf("[GB] Activity fetch since: %lu\n", since);
        // Send current activity data (we don't store historical data yet)
        sendActivityData(false);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // CALENDAR - t:"calendar", t:"calendar-"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "calendar") == 0) {
        int eventId = doc["id"] | -1;
        if (eventId >= 0 && calendarEventCount < MAX_CALENDAR_EVENTS) {
            CalendarEvent& event = calendarEvents[calendarEventCount];
            event.id = eventId;
            event.timestamp = doc["timestamp"] | 0;
            event.duration = doc["durationInSeconds"] | 0;
            event.title = doc["title"] | "";
            event.description = doc["description"] | "";
            event.location = doc["location"] | "";
            event.allDay = doc["allDay"] | false;
            event.valid = true;
            calendarEventCount++;
            
            USBSerial.printf("[GB] Calendar event: %s at %lu\n", 
                event.title.c_str(), event.timestamp);
        }
    }
    else if (strcmp(msgType, "calendar-") == 0) {
        int eventId = doc["id"] | -1;
        // Remove calendar event by ID
        for (int i = 0; i < calendarEventCount; i++) {
            if (calendarEvents[i].id == eventId) {
                calendarEvents[i].valid = false;
                USBSerial.printf("[GB] Removed calendar event: %d\n", eventId);
                break;
            }
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // GPS DATA FROM PHONE - t:"gps"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "gps") == 0) {
        phoneGPS.lat = doc["lat"] | 0.0f;
        phoneGPS.lon = doc["lon"] | 0.0f;
        phoneGPS.alt = doc["alt"] | 0.0f;
        phoneGPS.speed = doc["speed"] | 0.0f;  // km/h
        phoneGPS.course = doc["course"] | 0.0f;
        phoneGPS.satellites = doc["satellites"] | 0;
        phoneGPS.valid = true;
        
        USBSerial.printf("[GB] GPS: %.6f, %.6f, alt=%.1f\n", 
            phoneGPS.lat, phoneGPS.lon, phoneGPS.alt);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // GPS ACTIVE CHECK - t:"is_gps_active"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "is_gps_active") == 0) {
        sendBLEResponse(phoneGPS.enabled ? 
            "{\"t\":\"gps_power\",\"status\":true}" : 
            "{\"t\":\"gps_power\",\"status\":false}");
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // NAVIGATION - t:"nav"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "nav") == 0) {
        if (doc.containsKey("instr")) {
            navState.instruction = doc["instr"] | "";
            navState.distance = doc["distance"] | "";
            navState.action = doc["action"] | "";
            navState.eta = doc["eta"] | "";
            navState.active = true;
            
            // Show navigation notification
            addNotification("Navigation", navState.instruction, 
                navState.distance + " - ETA: " + navState.eta);
            wakeScreenForNotification();
            
            USBSerial.printf("[GB] Nav: %s, %s, ETA=%s\n", 
                navState.instruction.c_str(), navState.distance.c_str(), navState.eta.c_str());
        } else {
            // Navigation stopped
            navState.active = false;
            USBSerial.println("[GB] Navigation stopped");
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // VERSION/INFO REQUEST - t:"ver", t:"info", t:"status"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "ver") == 0) {
        sendBLEResponse("{\"t\":\"ver\",\"fw\":\"MiniOS 7.4.1\",\"hw\":\"ESP32-S3\"}");
    }
    else if (strcmp(msgType, "info") == 0) {
        char response[256];
        snprintf(response, sizeof(response), 
            "{\"t\":\"info\",\"msg\":\"MiniOS 7.4.1 | Bat:%d%% | Steps:%lu\"}",
            batteryPercent, (unsigned long)userData.steps);
        sendBLEResponse(response);
    }
    else if (strcmp(msgType, "status") == 0) {
        sendStatusUpdate();
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // CUSTOM: ASTRO DATA - t:"astro"
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "astro") == 0) {
        moonPhase = doc["moon"] | 0.0;
        moonPhaseName = doc["moonName"] | "---";
        sunriseTime = doc["sunrise"] | "--:--";
        sunsetTime = doc["sunset"] | "--:--";
        astroDataSynced = true;
        
        USBSerial.printf("[GB] Astro: Moon=%s, Rise=%s, Set=%s\n", 
            moonPhaseName.c_str(), sunriseTime.c_str(), sunsetTime.c_str());
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // HTTP RESPONSE - t:"http" (response from phone)
    // ═══════════════════════════════════════════════════════════════════════
    else if (strcmp(msgType, "http") == 0) {
        const char* resp = doc["resp"] | "";
        const char* err = doc["err"] | "";
        const char* id = doc["id"] | "";
        
        if (strlen(err) > 0) {
            USBSerial.printf("[GB] HTTP error: %s\n", err);
        } else {
            USBSerial.printf("[GB] HTTP response [%s]: %s\n", id, 
                String(resp).substring(0, 100).c_str());
        }
    }
    
    // Update last activity time for all messages
    bleLastActivityTime = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND STATUS UPDATE TO GADGETBRIDGE
// ═══════════════════════════════════════════════════════════════════════════
void sendStatusUpdate() {
    char response[128];
    snprintf(response, sizeof(response), 
        "{\"t\":\"status\",\"bat\":%d,\"volt\":%.2f,\"chg\":%d}",
        batteryPercent, batteryVoltage / 1000.0f, isCharging ? 1 : 0);
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND ACTIVITY DATA TO GADGETBRIDGE
// Format: {t:"act", ts:long, hrm:int, stp:int, mov:int, rt:int}
// ═══════════════════════════════════════════════════════════════════════════
void sendActivityData(bool realtime) {
    char response[256];
    unsigned long timestamp = 0;
    
    // Get current Unix timestamp if RTC available
    if (hasRTC) {
        RTC_DateTime dt = rtc.getDateTime();
        struct tm ti = {0};
        ti.tm_year = dt.getYear() - 1900;
        ti.tm_mon = dt.getMonth() - 1;
        ti.tm_mday = dt.getDay();
        ti.tm_hour = dt.getHour();
        ti.tm_min = dt.getMinute();
        ti.tm_sec = dt.getSecond();
        timestamp = mktime(&ti) * 1000ULL;  // Convert to milliseconds
    }
    
    snprintf(response, sizeof(response), 
        "{\"t\":\"act\",\"ts\":%lu,\"hrm\":%d,\"stp\":%lu,\"mov\":%d,\"rt\":%d}",
        timestamp, lastHeartRate, (unsigned long)userData.steps, 
        activityIntensity, realtime ? 1 : 0);
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND FIND PHONE REQUEST TO GADGETBRIDGE
// Makes the connected phone ring
// ═══════════════════════════════════════════════════════════════════════════
void sendFindPhone(bool enable) {
    char response[64];
    snprintf(response, sizeof(response), "{\"t\":\"findPhone\",\"n\":%s}", 
        enable ? "true" : "false");
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND MUSIC CONTROL TO GADGETBRIDGE
// Options: play, pause, next, previous, volumeup, volumedown
// ═══════════════════════════════════════════════════════════════════════════
void sendMusicControl(const char* action) {
    char response[64];
    snprintf(response, sizeof(response), "{\"t\":\"music\",\"n\":\"%s\"}", action);
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND CALL CONTROL TO GADGETBRIDGE
// Options: ACCEPT, END, REJECT, IGNORE
// ═══════════════════════════════════════════════════════════════════════════
void sendCallControl(const char* action) {
    char response[64];
    snprintf(response, sizeof(response), "{\"t\":\"call\",\"n\":\"%s\"}", action);
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND NOTIFICATION ACTION TO GADGETBRIDGE
// Options: DISMISS, DISMISS_ALL, OPEN, MUTE, REPLY
// ═══════════════════════════════════════════════════════════════════════════
void sendNotificationAction(int id, const char* action, const char* replyMsg) {
    char response[256];
    if (replyMsg && strlen(replyMsg) > 0) {
        snprintf(response, sizeof(response), 
            "{\"t\":\"notify\",\"id\":%d,\"n\":\"%s\",\"msg\":\"%s\"}", 
            id, action, replyMsg);
    } else {
        snprintf(response, sizeof(response), 
            "{\"t\":\"notify\",\"id\":%d,\"n\":\"%s\"}", id, action);
    }
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// REQUEST GPS FROM PHONE
// ═══════════════════════════════════════════════════════════════════════════
void requestPhoneGPS(bool enable) {
    phoneGPS.enabled = enable;
    char response[64];
    snprintf(response, sizeof(response), "{\"t\":\"gps_power\",\"status\":%s}", 
        enable ? "true" : "false");
    sendBLEResponse(response);
}

// ═══════════════════════════════════════════════════════════════════════════
// WAKE SCREEN FOR NOTIFICATION
// ═══════════════════════════════════════════════════════════════════════════
void wakeScreenForNotification() {
    if (!screenOn) {
        screenOn = true;
        gfx->displayOn();
        gfx->setBrightness(saverModes[batterySaverLevel].brightness);
        lastActivityMs = millis();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND BLE RESPONSE (with newline terminator per Bangle.js protocol)
// ═══════════════════════════════════════════════════════════════════════════
void sendBLEResponse(String response) {
    if (bleDeviceConnected && pTxCharacteristic) {
        // Bangle.js protocol requires newline-separated JSON
        String fullResponse = response + "\n";
        
        // Handle MTU chunking for large responses
        int mtu = 20;  // Default BLE MTU, could be negotiated higher
        int len = fullResponse.length();
        int offset = 0;
        
        while (offset < len) {
            int chunkSize = min(mtu, len - offset);
            String chunk = fullResponse.substring(offset, offset + chunkSize);
            pTxCharacteristic->setValue(chunk.c_str());
            pTxCharacteristic->notify();
            offset += chunkSize;
            if (offset < len) delay(10);  // Small delay between chunks
        }
        
        USBSerial.printf("[GB] Sent: %s\n", response.substring(0, min((int)response.length(), 80)).c_str());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PERIODIC ACTIVITY REPORTING (call from main loop)
// ═══════════════════════════════════════════════════════════════════════════
unsigned long lastActivityReport = 0;

void updateGadgetbridgeActivity() {
    if (!bleDeviceConnected) return;
    
    // Send periodic activity data if realtime tracking is enabled
    if ((realtimeHRM || realtimeSteps) && 
        (millis() - lastActivityReport > (unsigned long)realtimeInterval * 1000)) {
        sendActivityData(true);  // realtime=true
        lastActivityReport = millis();
    }
    
    // Send status update every 5 minutes
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 300000) {
        sendStatusUpdate();
        lastStatusUpdate = millis();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// STOP BLE - Disable Bluetooth to save battery
// ═══════════════════════════════════════════════════════════════════════════
void stopBLE() {
    if (bleEnabled) {
        BLEDevice::deinit(true);
        bleEnabled = false;
        bleDeviceConnected = false;
        bleConnectedDeviceName = "";
        USBSerial.println("[BLE] ✓ Bluetooth disabled to save battery");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// START BLE - Enable Bluetooth
// ═══════════════════════════════════════════════════════════════════════════
void startBLE() {
    if (!bleEnabled) {
        initGadgetbridgeBLE();
        USBSerial.println("[BLE] ✓ Bluetooth enabled via Quick Actions");
    }
}

// Track if we have bonded (for auto-reconnect logic)
bool bleBonded = false;

// ═══════════════════════════════════════════════════════════════════════════
// BLE SECURITY CALLBACKS CLASS (for bonding/auto-reconnect)
// ═══════════════════════════════════════════════════════════════════════════
class GadgetbridgeSecurityCallbacks : public BLESecurityCallbacks {
    bool onConfirmPIN(uint32_t pin) override { 
        USBSerial.printf("[BLE] Confirm PIN: %d\n", pin);
        return true; 
    }
    
    uint32_t onPassKeyRequest() override { 
        USBSerial.println("[BLE] PassKey requested");
        return 123456; 
    }
    
    void onPassKeyNotify(uint32_t pass_key) override {
        USBSerial.printf("[BLE] PassKey notify: %d\n", pass_key);
    }
    
    bool onSecurityRequest() override { 
        USBSerial.println("[BLE] Security request - accepting");
        return true; 
    }
    
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
        if (cmpl.success) {
            USBSerial.println("[BLE] ✓ Bonding successful - auto-reconnect enabled!");
            bleBonded = true;  // Set flag for auto-reconnect logic
        } else {
            USBSerial.printf("[BLE] Bonding failed: %d\n", cmpl.fail_reason);
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// INITIALIZE GADGETBRIDGE BLE (with bonding for auto-reconnect)
// ═══════════════════════════════════════════════════════════════════════════
void initGadgetbridgeBLE() {
    USBSerial.println("[BLE] ═══ Initializing Gadgetbridge BLE ═══");
    
    // Create BLE Device
    BLEDevice::init(BLE_DEVICE_NAME);
    
    // ═══ ENABLE BONDING FOR AUTO-RECONNECT ═══
    // This is critical for Gadgetbridge to auto-connect after initial pairing
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new GadgetbridgeSecurityCallbacks());
    
    // Security settings for bonding
    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);  // Secure + Bond
    pSecurity->setCapability(ESP_IO_CAP_NONE);  // No PIN required (just works)
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    
    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new GadgetbridgeServerCallbacks());
    
    // Create Nordic UART Service (NUS)
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Create TX Characteristic (Notify - watch to phone)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());
    
    // Create RX Characteristic (Write - phone to watch)
    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new GadgetbridgeCallbacks());
    
    // Start Service
    pService->start();
    
    // Start Advertising with bonding info
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // For iPhone compatibility
    pAdvertising->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
    
    // Mark BLE as enabled and start auto-off timer
    bleEnabled = true;
    bleStartTime = millis();
    bleLastActivityTime = millis();
    
    USBSerial.println("[BLE] ✓ Gadgetbridge BLE ready!");
    USBSerial.printf("[BLE] Device name: %s\n", BLE_DEVICE_NAME);
    USBSerial.println("[BLE] Bonding enabled - will auto-reconnect after pairing");
    USBSerial.println("[BLE] Auto-off in 3 minutes (if no activity)");
}

// ═══════════════════════════════════════════════════════════════════════════
// SMART WIFI CONNECT (FROM 206q)
// ═══════════════════════════════════════════════════════════════════════════
void smartWiFiConnect() {
    USBSerial.println("[WiFi] ═══ Starting Smart WiFi Connect ═══");
    USBSerial.printf("[WiFi] Networks to try: %d\n", numWifiNetworks);
    
    // Try networks in order (slot 0 = hardcoded, slots 1+ = SD card)
    for (int i = 0; i < numWifiNetworks; i++) {
        if (!wifiNetworks[i].valid) continue;

        USBSerial.printf("[WiFi] Trying [%d]: %s %s\n", i, 
                        wifiNetworks[i].ssid,
                        i == 0 ? "(hardcoded)" : "(SD card)");
        
        WiFi.begin(wifiNetworks[i].ssid, wifiNetworks[i].password);

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            USBSerial.print(".");
            attempts++;
        }
        USBSerial.println();

        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            connectedNetworkIndex = i;
            USBSerial.printf("[WiFi] ✓ Connected to: %s (IP: %s)\n", 
                            wifiNetworks[i].ssid, 
                            WiFi.localIP().toString().c_str());
            
            // WiFi success - delete old time backup, NTP will provide fresh time
            deleteTimeBackup();
            USBSerial.println("[WiFi] ═══ WiFi Connected Successfully ═══");
            return;
        } else {
            USBSerial.printf("[WiFi] ✗ Failed to connect to: %s\n", wifiNetworks[i].ssid);
        }
    }

    wifiConnected = false;
    USBSerial.println("[WiFi] ✗ Failed to connect to any network");
    
    // WiFi FAILED - restore time from SD backup!
    if (hasTimeBackup()) {
        USBSerial.println("[TIME] WiFi failed - restoring time from SD backup...");
        restoreTimeBackup();
    } else {
        USBSerial.println("[TIME] No time backup available on SD card");
    }
    USBSerial.println("[WiFi] ═══ WiFi Connection Failed ═══");
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP - WITH EXTREME SLEEP WAKE DETECTION
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    USBSerial.begin(115200);
    delay(100);

    USBSerial.println("═══════════════════════════════════════════════════════════════");
    USBSerial.println("  S3 MiniOS v7.4 - EXTREME SLEEP EDITION");
    USBSerial.println("  Full Features + 2-Second Extreme Sleep Mode");
    USBSerial.println("═══════════════════════════════════════════════════════════════");

    // ═══════════════════════════════════════════════════════════════════════
    // DUAL BOOT INITIALIZATION - Must be early, before other init!
    // Double-tap BOOT button (GPIO 0) within 500ms to switch OS
    // Only check on fresh power-on (not wake from sleep)
    // ═══════════════════════════════════════════════════════════════════════
    {
        esp_sleep_wakeup_cause_t earlyWake = esp_sleep_get_wakeup_cause();
        if (earlyWake == ESP_SLEEP_WAKEUP_UNDEFINED) {
            // Fresh power-on - check for dual boot double-tap
            DualBoot.begin();                    // Initialize boot manager
            DualBoot.beginSD();                  // Mount SD card via SD_MMC
            DualBoot.checkDoubleTapBoot();       // Check for OS switch
            USBSerial.printf("[BOOT] Running: %s\n", DualBoot.getCurrentOSName());
            USBSerial.printf("[BOOT] Double-tap BOOT to switch to: %s\n", DualBoot.getOtherOSName());
        } else {
            // Waking from sleep - just init boot manager for status info
            DualBoot.begin();
            USBSerial.printf("[BOOT] Running: %s (woke from sleep)\n", DualBoot.getCurrentOSName());
        }
    }
    // ═══════════════════════════════════════════════════════════════════════

    // ═══════════════════════════════════════════════════════════════════════
    // EXTREME SLEEP: Check wake cause FIRST (before any hardware init)
    // ═══════════════════════════════════════════════════════════════════════
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    bool wokeFromSleep = (wakeup_cause != ESP_SLEEP_WAKEUP_UNDEFINED);
    
    if (wokeFromSleep) {
        USBSerial.println("════════════════════════════════════════════════════════════");
        USBSerial.println("  WAKING FROM EXTREME DEEP SLEEP");
        USBSerial.printf("  Sleep count: %lu\n", (unsigned long)extremeSleepCount);
        USBSerial.printf("  Saved battery saver level: %d\n", savedBatterySaverLevel);
        
        // Determine which GPIO caused wake
        bool wokenByPowerButton = false;
        bool wokenByTouch = false;
        bool wokenByTimer = false;
        
        if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT1) {
            uint64_t wakeup_status = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_status & (1ULL << PWR_BUTTON)) {
                USBSerial.println("  Wake source: PWR Button (GPIO10) → FULL BOOT");
                wokenByPowerButton = true;
            } else if (wakeup_status & (1ULL << TP_INT)) {
                USBSerial.println("  Wake source: Touch (GPIO38)");
                wokenByTouch = true;
            } else {
                USBSerial.printf("  Wake source: EXT1 (status: 0x%llX)\n", wakeup_status);
            }
        } else if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0) {
            USBSerial.println("  Wake source: EXT0 (PWR Button fallback) → FULL BOOT");
            wokenByPowerButton = true;
        } else if (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER) {
            USBSerial.println("  Wake source: Timer (30min backup) → FULL BOOT + NTP SYNC");
            wokenByTimer = true;
        } else {
            USBSerial.printf("  Wake source: Unknown (%d)\n", wakeup_cause);
        }
        
        // Restore RTC time from saved values
        USBSerial.printf("  Saved time before sleep: %02d:%02d:%02d\n",
                        savedRtcTime.hours, savedRtcTime.minutes, savedRtcTime.seconds);
        USBSerial.println("════════════════════════════════════════════════════════════");
        
        // Release GPIO hold (from ultra-low power config)
        gpio_hold_dis((gpio_num_t)PWR_BUTTON);
        gpio_hold_dis((gpio_num_t)TP_INT);
        
        // Deinit RTC GPIOs (return to normal function)
        rtc_gpio_deinit((gpio_num_t)PWR_BUTTON);
        rtc_gpio_deinit((gpio_num_t)TP_INT);
        
        // ═══════════════════════════════════════════════════════════════════════
        // POWER BUTTON WAKE: Always do FULL BOOT with WiFi/NTP sync
        // This ensures proper time sync when user explicitly presses power
        // ═══════════════════════════════════════════════════════════════════════
        if (wokenByPowerButton || wokenByTimer) {
            USBSerial.println("[BOOT] Power button/Timer wake → Full boot sequence");
            // Skip glance mode, proceed to full boot with WiFi/NTP sync
            // The code continues below to full hardware init
        }
        // ═══════════════════════════════════════════════════════════════════════
        // TOUCH WAKE: Use glance mode for quick time check (battery saving)
        // Only for MEDIUM and EXTREME modes
        // ═══════════════════════════════════════════════════════════════════════
        else if (wokenByTouch && glanceModeEnabled && 
            (savedBatterySaverLevel == 1 ||   // BATTERY_SAVER_MEDIUM
             savedBatterySaverLevel == 2)) {  // BATTERY_SAVER_EXTREME
            
            USBSerial.println("[GLANCE] Touch wake - Glance mode active");
            
            // Minimal hardware init for glance mode
            Wire.begin(IIC_SDA, IIC_SCL);
            Wire.setClock(400000);
            
            pinMode(LCD_RESET, OUTPUT);
            pinMode(TP_RESET, OUTPUT);
            pinMode(PWR_BUTTON, INPUT_PULLUP);
            pinMode(TP_INT, INPUT);
            
            // Reset display
            digitalWrite(LCD_RESET, LOW);
            delay(20);
            digitalWrite(LCD_RESET, HIGH);
            delay(50);
            
            // Read FRESH time from RTC for glance display
            int glanceHour = savedRtcTime.hours;
            int glanceMin = savedRtcTime.minutes;
            if (hasRTC) {
                // Try to read from RTC chip directly for most accurate time
                Wire.beginTransmission(0x51);  // PCF85063 address
                if (Wire.endTransmission() == 0) {
                    if (rtc.begin(Wire, IIC_SDA, IIC_SCL)) {
                        RTC_DateTime dt = rtc.getDateTime();
                        glanceHour = dt.getHour();
                        glanceMin = dt.getMinute();
                        USBSerial.printf("[GLANCE] Fresh RTC time: %02d:%02d\n", glanceHour, glanceMin);
                    }
                }
            }
            
            // Run glance mode with fresh RTC time
            bool userWantsFullBoot = runGlanceMode(glanceHour, glanceMin);
            
            if (!userWantsFullBoot) {
                // User didn't interact - go back to sleep immediately
                USBSerial.println("[GLANCE] Returning to deep sleep...");
                
                // Re-configure wake sources with HOLD for ultra-low power
                gpio_reset_pin((gpio_num_t)PWR_BUTTON);
                gpio_set_direction((gpio_num_t)PWR_BUTTON, GPIO_MODE_INPUT);
                gpio_pullup_en((gpio_num_t)PWR_BUTTON);
                gpio_pulldown_dis((gpio_num_t)PWR_BUTTON);
                gpio_hold_en((gpio_num_t)PWR_BUTTON);
                
                gpio_reset_pin((gpio_num_t)TP_INT);
                gpio_set_direction((gpio_num_t)TP_INT, GPIO_MODE_INPUT);
                gpio_pullup_en((gpio_num_t)TP_INT);
                gpio_pulldown_dis((gpio_num_t)TP_INT);
                gpio_hold_en((gpio_num_t)TP_INT);
                
                // CRITICAL: Ensure touch chip is in monitor mode for wake
                // Touch chip must stay powered to generate wake interrupt
                pinMode(TP_RESET, OUTPUT);
                digitalWrite(TP_RESET, LOW);
                delay(10);
                digitalWrite(TP_RESET, HIGH);
                delay(50);
                
                Wire.beginTransmission(FT3168_DEVICE_ADDRESS);
                if (Wire.endTransmission() == 0) {
                    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
                    USBSerial.println("[GLANCE] Touch chip in MONITOR mode");
                }
                
                uint64_t wakeup_pin_mask = (1ULL << PWR_BUTTON) | (1ULL << TP_INT);
                esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW);
                esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);  // 30 min backup
                
                // Configure ultra-low power domains
                esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
                esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
                esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST,   ESP_PD_OPTION_OFF);
                esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
                
                USBSerial.println("[GLANCE] Entering deep sleep...");
                USBSerial.println("[GLANCE] Touch chip powered for wake interrupt");
                USBSerial.flush();
                delay(10);
                esp_deep_sleep_start();
                // Never returns
            }
            
            USBSerial.println("[GLANCE] User interacted → proceeding to full boot");
            // Turn off display temporarily for full init
            gfx->displayOff();
        }
    } else {
        USBSerial.println("[BOOT] Fresh power on (not from sleep)");
    }

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_err_t wdt_init_err = esp_task_wdt_init(&wdt_config);
    if (wdt_init_err == ESP_ERR_INVALID_STATE) {
        USBSerial.println("[WDT] Already initialized, reconfiguring...");
        esp_task_wdt_deinit();
        esp_task_wdt_init(&wdt_config);
    }
    USBSerial.println("[WDT] Watchdog initialized (10s timeout)");

    pinMode(PWR_BUTTON, INPUT_PULLUP);
    pinMode(BOOT_BUTTON, INPUT_PULLUP);

    Wire.begin(IIC_SDA, IIC_SCL);
    Wire.setClock(400000);

    pinMode(LCD_RESET, OUTPUT);
    pinMode(TP_RESET, OUTPUT);
    digitalWrite(LCD_RESET, LOW);
    digitalWrite(TP_RESET, LOW);
    delay(20);
    digitalWrite(LCD_RESET, HIGH);
    digitalWrite(TP_RESET, HIGH);
    delay(50);

    gfx->begin();
    gfx->setBrightness(100);  // Start at medium brightness to save power
    gfx->fillScreen(0x0000);

    pinMode(TP_INT, INPUT_PULLUP);
    while (!FT3168->begin()) delay(1000);
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
    attachInterrupt(digitalPinToInterrupt(TP_INT), Arduino_IIC_Touch_Interrupt, FALLING);

    if (qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        // LOW POWER: 62.5Hz is enough for step counting, disable gyro
        qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_62_5Hz);
        qmi.enableAccelerometer();
        // DISABLE gyroscope to save power (only need accel for steps)
        qmi.disableGyroscope();
        hasIMU = true;
        USBSerial.println("[IMU] QMI8658 initialized (low power mode)");
        
        // Initialize step counter for low-power mode
        initStepCounter();
    }

    if (rtc.begin(Wire, IIC_SDA, IIC_SCL)) {
        hasRTC = true;
        
        // ═══════════════════════════════════════════════════════════════════════
        // EXTERNAL RTC: PCF85063 runs continuously and independently
        // 
        // The RTC chip is powered separately and maintains time through:
        // ✓ Deep sleep (ESP32 off, RTC still running)
        // ✓ Light sleep (ESP32 paused, RTC still running)
        // ✓ Watchdog resets (ESP32 restarts, RTC still running)
        // ✓ Power cycles (if RTC has battery backup like CR1220)
        // 
        // On every boot/wake, we read FRESH time from the RTC chip.
        // ═══════════════════════════════════════════════════════════════════════
        
        // Read FRESH time from RTC chip - it ALWAYS has accurate time
        RTC_DateTime dt = rtc.getDateTime();
        clockHour = dt.getHour();
        clockMinute = dt.getMinute();
        clockSecond = dt.getSecond();
        
        if (wokeFromSleep) {
            USBSerial.printf("[RTC] Fresh time from chip: %02d:%02d:%02d\n", 
                            clockHour, clockMinute, clockSecond);
            USBSerial.printf("[RTC] Backup comparison - saved: %02d:%02d:%02d, actual: %02d:%02d:%02d\n",
                            savedRtcTime.hours, savedRtcTime.minutes, savedRtcTime.seconds,
                            clockHour, clockMinute, clockSecond);
        } else {
            // Fresh boot - check if RTC has valid time (year > 2024)
            if (dt.getYear() < 2024) {
                // RTC lost power or first boot - set default time
                rtc.setDateTime(2025, 1, 26, 12, 0, 0);
                USBSerial.println("[RTC] Set default time (will sync with NTP)");
            } else {
                USBSerial.printf("[RTC] Valid time found: %02d:%02d:%02d %02d/%02d/%04d\n",
                                clockHour, clockMinute, clockSecond,
                                dt.getDay(), dt.getMonth(), dt.getYear());
            }
        }
        USBSerial.println("[RTC] PCF85063 ready - runs independently during all sleep modes");
    }

    if (power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        hasPMU = true;
        power.disableTSPinMeasure();
        power.enableBattVoltageMeasure();
        USBSerial.println("[PMU] AXP2101 initialized");
    }

    // Initialize SD card with full Widget OS folder structure (FROM 206q)
    if (initWidgetOSSDCard()) {
        USBSerial.printf("[SD] %s card mounted (%lluMB)\n", sdCardType.c_str(), sdCardSizeMB);
        updateSDCardHealth();
        USBSerial.printf("[SD] Total: %lluMB, Used: %lluMB (%.1f%%)\n", 
                        sdHealth.totalBytes / (1024*1024),
                        sdHealth.usedBytes / (1024*1024),
                        sdHealth.usedPercent);
    } else {
        USBSerial.println("[SD] No SD card or mount failed");
    }

    loadUserData();

    // Scan SD card for photos (for Photo clockface)
    if (hasSD) {
        scanSDPhotos();
        // Validate photo index against actual photo count
        if (currentPhotoIndex >= numSDPhotos) {
            currentPhotoIndex = 0;
        }
    }

    prefs.begin("minios", true);
    compassNorthOffset = prefs.getFloat("compassOffset", 0.0);
    prefs.end();

    // ═══ WIFI CONNECTION PRIORITY ═══
    // 1. Hardcoded WiFi (primary - always add first)
    // 2. SD card WiFi networks (additional options)
    
    // Add hardcoded WiFi as primary (slot 0)
    strncpy(wifiNetworks[0].ssid, "Optus_9D2E3D", sizeof(wifiNetworks[0].ssid) - 1);
    strncpy(wifiNetworks[0].password, "snucktemptGLeQU", sizeof(wifiNetworks[0].password) - 1);
    wifiNetworks[0].valid = true;
    numWifiNetworks = 1;
    USBSerial.println("[WiFi] Hardcoded network added as primary");
    
    // Load additional WiFi networks from SD card (slots 1-4)
    if (hasSD) {
        loadWiFiConfigFromSD();  // This appends to existing networks
        USBSerial.printf("[WiFi] Total networks available: %d\n", numWifiNetworks);
    }

    // Smart WiFi connect - tries all networks in order
    // If connected: deletes time backup (NTP will sync fresh time)
    // If failed: restores time from SD backup
    smartWiFiConnect();

    if (wifiConnected) {
        USBSerial.println("[WiFi] Connected - syncing time and weather");
        
        // Sync time via NTP
        configTime(gmtOffsetSec, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
        delay(2000);

        // Update RTC from NTP
        if (hasRTC) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                rtc.setDateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                ntpSyncedOnce = true;
                lastNTPSync = millis();
                USBSerial.println("[NTP] Time synced to RTC");
                
                // Save fresh time backup after NTP sync
                saveTimeBackup();
                USBSerial.println("[TIME] Fresh backup saved after NTP sync");
            }
        }

        // Auto-detect location and fetch weather (FROM 206q)
        fetchLocationFromIP();
        
        // POWER SAVE: Disconnect WiFi after initial sync
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        wifiConnected = false;
        USBSerial.println("[WiFi] Disconnected after sync (power save)");
    }

    // ═══ SD CARD TIME OFFSET (if WiFi failed) ═══
    if (!wifiConnected && hasTimeBackup()) {
        restoreTimeBackup();
        USBSerial.println("[TIME] Restored from SD card backup");
    }

    // ═══ GADGETBRIDGE BLE - START AFTER WiFi/SD ═══
    // Boot order: WiFi sync → SD card backup → BLE (3 min timer)
    USBSerial.println("[BLE] Starting Bluetooth (3 min auto-off)...");
    initGadgetbridgeBLE();
    
    // Initialize deep sleep timer
    lastTouchTime = millis();

    lv_init();

    size_t buf_size = LCD_WIDTH * 50 * sizeof(lv_color_t);
    buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * 50);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    const esp_timer_create_args_t timer_args = { .callback = &lvgl_tick_cb, .name = "lvgl_tick" };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, LVGL_TICK_PERIOD_MS * 1000);

    lastActivityMs = millis();
    screenOnStartMs = millis();
    last_ui_activity = millis();
    last_lvgl_response = millis();
    batteryStats.sessionStartMs = millis();
    lastAutoBackup = millis();  // Reset auto backup timer

    // Initialize power consumption logging to SD card - DISABLED (causes freezing)
    // initPowerLogging();
    // USBSerial.println("[POWER] Power consumption logging initialized");

    // Apply saved battery saver level
    applyBatterySaverMode(batterySaverLevel);

    xTaskCreatePinnedToCore(ui_task, "ui_task", 10240, NULL, 2, &ui_task_handle, 1);
    USBSerial.println("[UI_TASK] Created with 10KB stack on Core 1");

    // ═══ BOOT: Check if battery is below 20% (only if NOT charging) ═══
    if (hasPMU) {
        batteryPercent = power.getBatteryPercent();
        isCharging = power.isCharging();
        
        if (batteryPercent < 20 && !isCharging) {
            USBSerial.println("[BOOT] Battery < 20% - Entering POWER RESERVE mode");
            enterPowerReserveMode();
        }
    }

    ui_event = UI_EVENT_REFRESH;

    USBSerial.println("═══════════════════════════════════════════════════════════════");
    USBSerial.printf("  Battery: %d%% | Saver: %s | Light Sleep: %s\n", 
        batteryPercent, saverModes[batterySaverLevel].name,
        shouldEnableLightSleep() ? "READY" : "STANDBY");
    USBSerial.println("  Light sleep activates when: EXTREME mode enabled");
    USBSerial.println("═══════════════════════════════════════════════════════════════");
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // DUAL BOOT: Check for double-tap on BOOT button (GPIO 0) to switch OS
    // Non-blocking — polls button state each loop iteration
    DualBoot.checkRuntimeDoubleTap();

    // ═══ VOICE MEMO RECORDING ═══
    if (voiceMemoRecording) {
        recordVoiceSamples();
    }

    // ═══ BLE AUTO-OFF CHECK (3 minute timeout if no activity) ═══
    // NOTE: When bonded with Gadgetbridge, we keep BLE on for auto-reconnect
    if (bleEnabled && bleAutoOffEnabled) {
        unsigned long timeSinceActivity = millis() - bleLastActivityTime;
        
        // Only auto-off if:
        // 1. Not connected AND
        // 2. No recent activity AND
        // 3. Not bonded (use our tracking variable instead of unsupported API)
        
        if (!bleDeviceConnected && timeSinceActivity > BLE_AUTO_OFF_MS) {
            if (bleBonded) {
                // Keep advertising for bonded device reconnection
                // But reset timer so we don't spam this log
                bleLastActivityTime = millis() - (BLE_AUTO_OFF_MS - 60000);  // Check again in 1 min
                USBSerial.println("[BLE] Keeping alive for bonded device auto-reconnect...");
            } else {
                // No bonded devices - safe to turn off
                USBSerial.println("[BLE] Auto-off: 3 minute timeout (no bonded devices)");
                stopBLE();
            }
        }
    }

    // ═══ POWER BUTTON - FIXED: Simple debounced toggle ═══
    static bool lastPwrState = HIGH;
    static unsigned long lastPwrToggle = 0;
    const unsigned long BUTTON_DEBOUNCE_MS = 200;

    bool currentPwrState = digitalRead(PWR_BUTTON);

    // Detect button press with debounce
    if (lastPwrState == HIGH && currentPwrState == LOW) {
        if (millis() - lastPwrToggle > BUTTON_DEBOUNCE_MS) {
            lastPwrToggle = millis();
            lastTouchTime = millis();  // Reset deep sleep timer
            if (screenOn) {
                screenOff();
            } else {
                screenOnFunc();
            }
        }
    }
    lastPwrState = currentPwrState;

    // ═══ BOOT BUTTON HANDLING (optional secondary button) ═══
    static bool lastBootState = HIGH;
    bool currentBootState = digitalRead(BOOT_BUTTON);
    
    if (lastBootState == HIGH && currentBootState == LOW) {
        lastTouchTime = millis();  // Reset deep sleep timer
        // Boot button pressed - wake screen if off, or go to clock if on
        if (!screenOn) {
            ui_event = UI_EVENT_SCREEN_ON;
        } else {
            // Navigate to clock
            currentCategory = CAT_CLOCK;
            currentSubCard = 0;
            navigateTo(currentCategory, currentSubCard);
        }
    }
    lastBootState = currentBootState;

    // ═══ TOUCH TO WAKE - Wake screen on any touch ═══
    // Only check touch wake periodically to avoid false wakes from touch noise
    static unsigned long lastTouchWakeCheck = 0;
    if (!screenOn && millis() - lastTouchWakeCheck > 200) {  // Check every 200ms
        lastTouchWakeCheck = millis();
        uint8_t fingers = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
        // Require sustained touch (check twice with delay) to prevent false wakes
        if (fingers > 0) {
            delay(50);  // Short delay
            fingers = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
            if (fingers > 0) {
                USBSerial.println("[WAKE] Touch detected - waking screen");
                lastTouchTime = millis();  // Reset deep sleep timer
                ui_event = UI_EVENT_SCREEN_ON;
            }
        }
    }

    // Handle Fusion Labs Web Serial Protocol (FROM 206q)
    handleFusionLabsProtocol();

    // Check 24-hour auto backup (FROM 206q)
    checkAutoBackup();

    check_lvgl_stall();
    backlight_manager();

    if (hasRTC && millis() - lastClockUpdate >= 1000) {
        lastClockUpdate = millis();
        RTC_DateTime dt = rtc.getDateTime();
        clockHour = dt.getHour();
        clockMinute = dt.getMinute();
        clockSecond = dt.getSecond();
    }

    if (hasIMU && millis() - lastStepUpdate >= sensorPollInterval) {
        lastStepUpdate = millis();
        updateSensorFusion();
        updateStepCount();
    }

    if (hasPMU && millis() - lastBatteryUpdate >= 3000) {
        lastBatteryUpdate = millis();
        batteryVoltage = power.getBattVoltage();
        batteryPercent = power.getBatteryPercent();
        isCharging = power.isCharging();
        freeRAM = ESP.getFreeHeap();
        
        // ═══ POWER RESERVE: Reboot when charging detected ═══
        // If in power reserve mode and charger is connected, reboot to exit reserve mode
        if (powerReserveMode && isCharging) {
            USBSerial.println("[POWER RESERVE] Charging detected - REBOOTING...");
            delay(500);  // Brief delay to show message
            ESP.restart();
        }
    }

    if (millis() - lastSaveTime >= SAVE_INTERVAL_MS) saveUserData();

    // Weather update - SCHEDULED SYNC (hourly in saver modes for ~15% power saving)
    unsigned long weatherInterval = (batterySaverMode) ? WEATHER_SYNC_INTERVAL_MS : 1800000;  // 1hr vs 30min
    if (wifiConnected && !saverModes[batterySaverLevel].disableWifiSync && 
        millis() - lastWeatherUpdate >= weatherInterval) {
        fetchWeatherData();
        lastWeatherSync = millis();
    }

    // NTP resync every hour (FROM 206q)
    if (wifiConnected && millis() - lastNTPSync >= 3600000) {
        configTime(gmtOffsetSec, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
        if (hasRTC) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                rtc.setDateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            }
        }
        lastNTPSync = millis();
    }

    // Clock refresh
    if (screenOn && currentCategory == CAT_CLOCK && !isTransitioning) {
        static unsigned long lastRef = 0;
        if (millis() - lastRef >= 5000) {
            lastRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // ═══ POWER RESERVE MODE - Auto screen off after 0.5s ═══
    checkPowerReserveTimeout();

    // Compass refresh - FEATURE GATED
    if (shouldUpdateForCategory(CAT_COMPASS, -1)) {
        static unsigned long lastRef = 0;
        if (millis() - lastRef >= 500) {
            lastRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }

    // Stopwatch refresh - FEATURE GATED
    if (stopwatchRunning && shouldUpdateForCategory(CAT_TIMER, -1)) {
        static unsigned long lastRef = 0;
        if (millis() - lastRef >= 100) {
            lastRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // Running mode refresh - FEATURE GATED
    if (runningModeActive && shouldUpdateForCategory(CAT_ACTIVITY, 2)) {
        static unsigned long lastRef = 0;
        if (millis() - lastRef >= 500) {
            lastRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // Tilt Maze game refresh - FEATURE GATED (paused when not on screen)
    if (mazeGameActive && !mazeGameWon && isGameActive(4)) {
        static unsigned long lastMazeRef = 0;
        if (millis() - lastMazeRef >= 33) {  // ~30 FPS
            lastMazeRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // Pong game refresh - FEATURE GATED
    if (pongGameActive && isGameActive(5)) {
        static unsigned long lastPongRef = 0;
        if (millis() - lastPongRef >= 33) {  // ~30 FPS
            lastPongRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // Breakout game refresh - FEATURE GATED
    if (breakoutGameActive && !breakoutGameOver && isGameActive(9)) {
        static unsigned long lastBreakoutRef = 0;
        if (millis() - lastBreakoutRef >= 33) {  // ~30 FPS
            lastBreakoutRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }
    
    // Tetris game refresh - FEATURE GATED
    if (tetrisGameActive && !tetrisGameOver && isGameActive(10)) {
        static unsigned long lastTetrisRef = 0;
        if (millis() - lastTetrisRef >= 50) {  // ~20 FPS
            lastTetrisRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }

    // ═══ AMOLED BURN-IN PREVENTION ═══
    if (screenOn) {
        updateBurnInOffset();
    }
    
    // ═══ DEEP SLEEP CHECK (4 min no touch) ═══
    if (!screenOn) {
        checkDeepSleepTimeout();
    }

    // ═══ POWER CONSUMPTION LOGGING - DISABLED (was causing device freezes) ═══
    // logPowerConsumption();
    // writePowerSummary();
    
    // Log rotation and daily archiving - DISABLED
    // rotatePowerLogIfNeeded();
    // archiveDailyPowerLog();

    // ═══════════════════════════════════════════════════════════════════════
    // SPORT TRACKER - Sprint detection via IMU acceleration - FEATURE GATED
    // Only runs when sport tracker screen is active
    // ═══════════════════════════════════════════════════════════════════════
    if (sportTrackerActive && hasIMU && shouldUpdateForCategory(CAT_ACTIVITY, 3)) {
        static unsigned long lastSprintCheck = 0;
        if (millis() - lastSprintCheck >= 100) {  // Check at 10Hz
            lastSprintCheck = millis();
            
            // Get acceleration data
            float ax, ay, az;
            if (qmi.getAccelerometer(ax, ay, az)) {
                float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
                
                // Sprint detection: sudden acceleration spike > threshold
                if (accelMagnitude > SPRINT_THRESHOLD && !sprintDetected) {
                    sprintDetected = true;
                    sportSprintCount++;
                    USBSerial.printf("[SPORT] Sprint detected! Total: %d (accel: %.1f)\n", sportSprintCount, accelMagnitude);
                } else if (accelMagnitude < SPRINT_THRESHOLD * 0.7) {
                    sprintDetected = false;  // Reset when acceleration drops
                }
                lastAccelMagnitude = accelMagnitude;
            }
        }
    }
    
    // Sport Tracker screen refresh - FEATURE GATED
    if (sportTrackerActive && shouldUpdateForCategory(CAT_ACTIVITY, 3)) {
        static unsigned long lastSportRef = 0;
        if (millis() - lastSportRef >= 1000) {  // Update every second
            lastSportRef = millis();
            ui_event = UI_EVENT_REFRESH;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ADAPTIVE BATTERY SAVING - Normal Mode Only
    // Dynamic loop delay based on user activity
    // ═══════════════════════════════════════════════════════════════════════
    if (batterySaverLevel == BATTERY_SAVER_OFF) {
        // Normal mode - apply adaptive battery saving
        unsigned long idleTime = millis() - lastUserInteraction;
        
        if (idleTime > ADAPTIVE_IDLE_THRESHOLD_3) {
            adaptiveLoopDelay = 50;  // Deep idle - max battery saving
        } else if (idleTime > ADAPTIVE_IDLE_THRESHOLD_2) {
            adaptiveLoopDelay = 30;  // Moderate idle
        } else if (idleTime > ADAPTIVE_IDLE_THRESHOLD_1) {
            adaptiveLoopDelay = 20;  // Light idle
        } else {
            adaptiveLoopDelay = 10;  // Active - responsive
        }
        
        delay(adaptiveLoopDelay);
    } else {
        // Battery saver modes use their own delays
        delay(10);
    }
}
