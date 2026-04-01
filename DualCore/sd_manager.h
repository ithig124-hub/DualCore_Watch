/*
 * sd_manager.h - SD Card Management & WiFi Configuration
 * Based on S3_MiniOS_206.ino and Fusion Labs Protocol
 * 
 * Features:
 * - Auto-create folder structure on SD card
 * - WiFi config.txt with multiple networks
 * - NTP time sync
 * - Auto-backup system
 * - Fusion Labs Web Serial Protocol support
 * - OTA update preparation
 */

#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <Arduino.h>
#include <SD_MMC.h>
#include <FS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// Forward declaration for TouchGesture (defined in touch.h)
struct TouchGesture;

// =============================================================================
// SD CARD PATHS (Compatible with S3_MiniOS_206)
// =============================================================================

#define SD_ROOT_PATH            "/WATCH"
#define SD_USER_DATA_PATH       "/WATCH/data"
#define SD_MUSIC_PATH           "/WATCH/music"
#define SD_WIFI_PATH            "/WATCH/wifi"
#define SD_BACKUP_PATH          "/WATCH/BACKUPS"
#define SD_FIRMWARE_PATH        "/WATCH/FIRMWARE"
#define SD_LOGS_PATH            "/WATCH/LOGS"
#define SD_WALLPAPERS_PATH      "/WATCH/WALLPAPERS"
#define SD_GACHA_PATH           "/WATCH/gacha"
#define SD_TRAINING_PATH        "/WATCH/training"
#define SD_BOSS_PATH            "/WATCH/boss_rush"
#define SD_THEMES_PATH          "/WATCH/themes"
#define SD_ASSETS_PATH          "/WATCH/assets"

#define SD_WIFI_CONFIG          "/WATCH/wifi/config.txt"
#define SD_PLAYER_DATA          "/WATCH/data/player.dat"
#define SD_GACHA_DATA           "/WATCH/gacha/cards.dat"
#define SD_BOSS_DATA            "/WATCH/boss_rush/progress.dat"
#define SD_BOOT_LOG             "/WATCH/LOGS/boot.log"

// =============================================================================
// SD CARD STATUS
// =============================================================================

enum SDCardStatus { 
  SD_STATUS_NOT_PRESENT = 0, 
  SD_STATUS_MOUNTED_OK, 
  SD_STATUS_MOUNT_FAILED,
  SD_STATUS_CORRUPT,
  SD_STATUS_READ_ONLY,
  SD_STATUS_INIT_IN_PROGRESS
};

struct SDCardHealth {
  uint64_t totalBytes, usedBytes, freeBytes;
  float usedPercent;
  bool mounted, healthy;
  String lastError;
  int writeErrors, readErrors;
};

extern SDCardStatus sdCardStatus;
extern SDCardHealth sdHealth;
extern bool sdCardInitialized;
extern bool sdStructureCreated;

// =============================================================================
// WIFI CONFIGURATION
// =============================================================================

#define MAX_WIFI_NETWORKS 5

struct WiFiNetwork {
  char ssid[64];
  char password[64];
  bool valid;
  bool isOpen;
  int32_t rssi;
};

extern WiFiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
extern int numWifiNetworks;
extern int connectedNetworkIndex;
extern bool wifiConnected;
extern bool wifiConfigFromSD;

// Weather/Location
extern char weatherCity[64];
extern char weatherCountry[8];
extern long gmtOffsetSec;

// NTP
#define NTP_SERVER "pool.ntp.org"
#define DAYLIGHT_OFFSET_SEC 0
extern bool ntpSyncedOnce;
extern unsigned long lastNTPSync;

// =============================================================================
// SD CARD FUNCTIONS
// =============================================================================

bool initSDCard();
bool createSDFolderStructure();
void updateSDHealth();
void logToBootLog(const char* message);

// Config files
bool createDefaultWiFiConfig();
bool loadWiFiFromSD();
bool saveWiFiToSD();
bool loadPlayerDataFromSD();
bool savePlayerDataToSD();
bool loadGachaDataFromSD();
bool saveGachaDataToSD();
bool loadBossDataFromSD();
bool saveBossDataToSD();

// =============================================================================
// WIFI FUNCTIONS
// =============================================================================

bool initWiFiFromSD();
bool connectToWiFi();
bool connectToNetwork(int networkIndex);
void disconnectWiFi();
void scanWiFiNetworks();
int getWiFiSignalStrength();
const char* getWiFiStatusString();

// =============================================================================
// NTP TIME SYNC
// =============================================================================

bool syncTimeFromNTP();
void updateRTCFromNTP();
bool isNTPSynced();
unsigned long getTimeSinceNTPSync();

// =============================================================================
// BACKUP SYSTEM
// =============================================================================

#define AUTO_BACKUP_INTERVAL_MS (24UL * 60UL * 60UL * 1000UL)  // 24 hours
extern bool autoBackupEnabled;
extern unsigned long lastAutoBackup;

bool performAutoBackup();
bool createBackup(const char* backupName);
bool restoreFromBackup(const char* backupName);
void listBackups();
void cleanOldBackups(int keepCount);

// =============================================================================
// FUSION LABS PROTOCOL - COMPLETE WEB SERIAL INTEGRATION
// =============================================================================

#define FUSION_OS_NAME          "Fusion Watch"
#define FUSION_OS_VERSION       "1.0.0"
#define FUSION_OS_BUILD         "anime_edition"
#define DEVICE_ID               "FW-180A"
#define FUSION_PROTOCOL_VERSION "1.0"
#define FUSION_LABS_URL         "https://ithig124-hub.github.io/fusion-labs"
#define FIRMWARE_UPDATE_URL     "https://ithig124-hub.github.io/fusion-labs/fusion-labs/firmware/fusion-watch/180A/latest.json"

// Serial buffer for Web Serial communication
extern String serialBuffer;
extern bool receivingWifiConfig;
extern String wifiConfigBuffer;
extern bool receivingFaceData;
extern String faceIdBuffer;
extern String faceDataBuffer;
extern bool waitingForFaceId;
extern char activeWatchFaceId[32];

// OTA Update state
struct OTAUpdateInfo {
  String version;
  String bin_url;
  String checksum;
  String release_notes;
  size_t file_size;
  bool update_available;
  bool download_in_progress;
  int download_progress;
  bool ready_to_apply;
};

extern OTAUpdateInfo otaInfo;

// OTA Update Functions
bool checkForUpdates();
bool downloadFirmware(const char* url);
bool downloadFirmwareToSD();
bool applyFirmwareUpdate();
void cancelOTAUpdate();
int getOTAProgress();

// Web Serial Protocol - Core Functions
void setupFusionLabsSerial();
void handleSerialConfig();
void processSerialCommand(const String& command);

// Web Serial Protocol - Status Commands
void sendDeviceStatus();
void sendWifiConfig();
void saveWifiConfigFromSerial(const String& config);

// Web Serial Protocol - Watch Face Commands
void listInstalledFaces();
void saveFaceFromSerial(const String& faceId, const String& faceData);
void setActiveFace(const String& command);
void deleteFace(const String& command);
void handleFaceSerialData(const String& data);

// Web Serial Protocol - Response
void sendFusionLabsResponse(const char* response);

// Web Serial Protocol - Gacha/Boss Data Commands  
void sendGachaCollectionData();
void sendBossProgressData();
void sendPlayerStatsData();

// =============================================================================
// UI FUNCTIONS
// =============================================================================

void drawSDCardStatus();
void drawWiFiStatus();
void drawWiFiManagerScreen();
void handleWiFiManagerTouch(TouchGesture& gesture);
void drawBackupScreen();
void handleBackupTouch(TouchGesture& gesture);

// OTA Update UI
void drawOTAUpdateScreen();
void handleOTAUpdateTouch(TouchGesture& gesture);
void drawUpdateProgressBar(int x, int y, int w, int h, int progress);

// Fusion Labs Connection UI  
void drawFusionLabsScreen();
void handleFusionLabsTouch(TouchGesture& gesture);

#endif // SD_MANAGER_H
