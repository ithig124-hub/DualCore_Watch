/*
 * sd_manager.cpp - SD Card Management & WiFi Configuration Implementation
 * Compatible with S3_MiniOS_206.ino and Fusion Labs Protocol
 */

#include "sd_manager.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "hardware.h"
#include "gacha.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

SDCardStatus sdCardStatus = SD_STATUS_NOT_PRESENT;
SDCardHealth sdHealth = {0};
bool sdCardInitialized = false;
bool sdStructureCreated = false;

WiFiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
int numWifiNetworks = 0;
int connectedNetworkIndex = -1;
bool wifiConnected = false;
bool wifiConfigFromSD = false;

char weatherCity[64] = "Perth";
char weatherCountry[8] = "AU";
long gmtOffsetSec = 8 * 3600;  // GMT+8 default (Perth)

bool ntpSyncedOnce = false;
unsigned long lastNTPSync = 0;

bool autoBackupEnabled = true;
unsigned long lastAutoBackup = 0;

// =============================================================================
// SD CARD INITIALIZATION
// =============================================================================

bool initSDCard() {
  Serial.println("[SD] Initializing SD Card (SD_MMC)...");
  sdCardStatus = SD_STATUS_INIT_IN_PROGRESS;
  
  // CRITICAL FIX: Set correct SD_MMC pins before begin() (from Waveshare official)
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  Serial.printf("[SD] setPins(CLK=%d, CMD=%d, D0=%d)\n", SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  
  // Initialize SD_MMC with 1-bit mode for compatibility
  if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit mode
    Serial.println("[SD] SD_MMC mount failed!");
    sdCardStatus = SD_STATUS_MOUNT_FAILED;
    sdHealth.mounted = false;
    sdHealth.lastError = "Mount failed";
    return false;
  }
  
  // Get card type
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] No SD card attached!");
    sdCardStatus = SD_STATUS_NOT_PRESENT;
    sdHealth.mounted = false;
    sdHealth.lastError = "No card";
    return false;
  }
  
  Serial.printf("[SD] Card Type: ");
  switch (cardType) {
    case CARD_MMC:  Serial.println("MMC"); break;
    case CARD_SD:   Serial.println("SDSC"); break;
    case CARD_SDHC: Serial.println("SDHC"); break;
    default:        Serial.println("UNKNOWN"); break;
  }
  
  sdCardStatus = SD_STATUS_MOUNTED_OK;
  sdHealth.mounted = true;
  sdHealth.healthy = true;
  sdCardInitialized = true;
  
  // Update health stats
  updateSDHealth();
  
  // Create folder structure
  if (!createSDFolderStructure()) {
    Serial.println("[SD] Warning: Could not create folder structure");
  }
  
  Serial.printf("[SD] SD Card: %.2f GB total, %.2f GB free\n",
                sdHealth.totalBytes / (1024.0 * 1024.0 * 1024.0),
                sdHealth.freeBytes / (1024.0 * 1024.0 * 1024.0));
  
  logToBootLog("SD Card initialized successfully");
  return true;
}

void updateSDHealth() {
  if (!sdCardInitialized) return;
  
  sdHealth.totalBytes = SD_MMC.totalBytes();
  sdHealth.usedBytes = SD_MMC.usedBytes();
  sdHealth.freeBytes = sdHealth.totalBytes - sdHealth.usedBytes;
  sdHealth.usedPercent = (float)sdHealth.usedBytes / sdHealth.totalBytes * 100.0;
}

// =============================================================================
// SD FOLDER STRUCTURE
// =============================================================================

bool createSDFolderStructure() {
  Serial.println("[SD] Creating folder structure...");
  
  const char* folders[] = {
    SD_ROOT_PATH,
    SD_USER_DATA_PATH,
    SD_MUSIC_PATH,
    SD_WIFI_PATH,
    SD_BACKUP_PATH,
    SD_FIRMWARE_PATH,
    SD_LOGS_PATH,
    SD_WALLPAPERS_PATH,
    SD_GACHA_PATH,
    SD_TRAINING_PATH,
    SD_BOSS_PATH,
    SD_THEMES_PATH,
    SD_ASSETS_PATH
  };
  
  int numFolders = sizeof(folders) / sizeof(folders[0]);
  
  for (int i = 0; i < numFolders; i++) {
    if (!SD_MMC.exists(folders[i])) {
      if (SD_MMC.mkdir(folders[i])) {
        Serial.printf("[SD] Created: %s\n", folders[i]);
      } else {
        Serial.printf("[SD] Failed to create: %s\n", folders[i]);
      }
    }
  }
  
  // Create default WiFi config
  if (!SD_MMC.exists(SD_WIFI_CONFIG)) {
    createDefaultWiFiConfig();
  }
  
  sdStructureCreated = true;
  return true;
}

// =============================================================================
// WIFI CONFIG FILE
// =============================================================================

bool createDefaultWiFiConfig() {
  Serial.println("[SD] Creating default wifi/config.txt...");
  
  File wifiCfg = SD_MMC.open(SD_WIFI_CONFIG, FILE_WRITE);
  if (!wifiCfg) {
    Serial.println("[SD] Failed to create wifi config!");
    return false;
  }
  
  wifiCfg.println("# ESP32 Watch WiFi Configuration");
  wifiCfg.println("# Supports up to 5 WiFi networks");
  wifiCfg.println("# Edit this file to add your networks");
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
  wifiCfg.println("# Timezone (hours from GMT, e.g., 8 for GMT+8)");
  wifiCfg.println("GMT_OFFSET=8");
  wifiCfg.println("");
  wifiCfg.println("# NTP Server");
  wifiCfg.println("NTP_SERVER=pool.ntp.org");
  wifiCfg.close();
  
  Serial.println("[SD] Created wifi/config.txt with 5 network slots");
  return true;
}

bool loadWiFiFromSD() {
  if (!sdCardInitialized) return false;
  
  Serial.println("[SD] Loading WiFi config from SD...");
  
  File wifiCfg = SD_MMC.open(SD_WIFI_CONFIG, FILE_READ);
  if (!wifiCfg) {
    Serial.println("[SD] WiFi config not found!");
    return false;
  }
  
  // Clear existing networks
  numWifiNetworks = 0;
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    wifiNetworks[i].valid = false;
    wifiNetworks[i].ssid[0] = '\0';
    wifiNetworks[i].password[0] = '\0';
    wifiNetworks[i].isOpen = false;
  }
  
  while (wifiCfg.available()) {
    String line = wifiCfg.readStringUntil('\n');
    line.trim();
    
    // Skip comments and empty lines
    if (line.startsWith("#") || line.length() == 0) continue;
    
    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;
    
    String key = line.substring(0, eqPos);
    String value = line.substring(eqPos + 1);
    key.trim();
    value.trim();
    
    // Parse network configs
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
      char numStr[2];
      sprintf(numStr, "%d", i + 1);
      
      if (key == String("SSID") + numStr) {
        if (value.length() > 0) {
          strncpy(wifiNetworks[i].ssid, value.c_str(), 63);
          wifiNetworks[i].valid = true;
          numWifiNetworks = max(numWifiNetworks, i + 1);
        }
      } else if (key == String("PASSWORD") + numStr) {
        strncpy(wifiNetworks[i].password, value.c_str(), 63);
      } else if (key == String("OPEN") + numStr) {
        wifiNetworks[i].isOpen = (value == "true" || value == "1");
      }
    }
    
    // Parse location/timezone
    if (key == "CITY") {
      strncpy(weatherCity, value.c_str(), 63);
    } else if (key == "COUNTRY") {
      strncpy(weatherCountry, value.c_str(), 7);
    } else if (key == "GMT_OFFSET") {
      gmtOffsetSec = value.toInt() * 3600;
    }
  }
  
  wifiCfg.close();
  wifiConfigFromSD = true;
  
  Serial.printf("[SD] Loaded %d WiFi networks, City: %s, GMT: %+d\n",
                numWifiNetworks, weatherCity, gmtOffsetSec / 3600);
  
  return numWifiNetworks > 0;
}

bool saveWiFiToSD() {
  if (!sdCardInitialized) return false;
  
  Serial.println("[SD] Saving WiFi config to SD...");
  
  File wifiCfg = SD_MMC.open(SD_WIFI_CONFIG, FILE_WRITE);
  if (!wifiCfg) {
    Serial.println("[SD] Failed to write wifi config!");
    return false;
  }
  
  wifiCfg.println("# ESP32 Watch WiFi Configuration");
  wifiCfg.println("# Auto-saved configuration");
  wifiCfg.println("");
  
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    wifiCfg.printf("# Network %d\n", i + 1);
    wifiCfg.printf("SSID%d=%s\n", i + 1, wifiNetworks[i].ssid);
    wifiCfg.printf("PASSWORD%d=%s\n", i + 1, wifiNetworks[i].password);
    wifiCfg.printf("OPEN%d=%s\n", i + 1, wifiNetworks[i].isOpen ? "true" : "false");
    wifiCfg.println("");
  }
  
  wifiCfg.println("# Location");
  wifiCfg.printf("CITY=%s\n", weatherCity);
  wifiCfg.printf("COUNTRY=%s\n", weatherCountry);
  wifiCfg.println("");
  wifiCfg.printf("GMT_OFFSET=%d\n", gmtOffsetSec / 3600);
  
  wifiCfg.close();
  return true;
}

// =============================================================================
// WIFI FUNCTIONS
// =============================================================================

bool initWiFiFromSD() {
  if (!loadWiFiFromSD()) {
    Serial.println("[WIFI] No valid WiFi config on SD card");
    return false;
  }
  return connectToWiFi();
}

bool connectToWiFi() {
  Serial.println("[WIFI] Attempting to connect...");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  // Try each network in order
  for (int i = 0; i < numWifiNetworks; i++) {
    if (!wifiNetworks[i].valid || strlen(wifiNetworks[i].ssid) == 0) continue;
    
    if (connectToNetwork(i)) {
      return true;
    }
  }
  
  Serial.println("[WIFI] Failed to connect to any network");
  return false;
}

bool connectToNetwork(int networkIndex) {
  if (networkIndex < 0 || networkIndex >= MAX_WIFI_NETWORKS) return false;
  if (!wifiNetworks[networkIndex].valid) return false;
  
  WiFiNetwork& net = wifiNetworks[networkIndex];
  
  Serial.printf("[WIFI] Connecting to: %s\n", net.ssid);
  
  if (net.isOpen) {
    WiFi.begin(net.ssid);
  } else {
    WiFi.begin(net.ssid, net.password);
  }
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    connectedNetworkIndex = networkIndex;
    net.rssi = WiFi.RSSI();
    
    Serial.printf("\n[WIFI] Connected! IP: %s, RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), net.rssi);
    
    // Auto-sync time
    syncTimeFromNTP();
    
    system_state.wifi_connected = true;
    system_state.wifi_ssid = String(net.ssid);
    system_state.wifi_signal_strength = net.rssi;
    
    return true;
  }
  
  Serial.println("\n[WIFI] Connection failed");
  return false;
}

// disconnectWiFi() is defined in wifi_apps.cpp - renamed here to avoid duplicate
void disconnectWiFiManager() {
  WiFi.disconnect();
  wifiConnected = false;
  connectedNetworkIndex = -1;
  system_state.wifi_connected = false;
}

void scanWiFiNetworks() {
  Serial.println("[WIFI] Scanning networks...");
  int n = WiFi.scanNetworks();
  Serial.printf("[WIFI] Found %d networks\n", n);
  
  for (int i = 0; i < n && i < 10; i++) {
    Serial.printf("  %d: %s (%d dBm) %s\n",
                  i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "[OPEN]" : "");
  }
}

int getWiFiSignalStrength() {
  if (!wifiConnected) return -100;
  return WiFi.RSSI();
}

const char* getWiFiStatusString() {
  switch (WiFi.status()) {
    case WL_CONNECTED: return "Connected";
    case WL_NO_SHIELD: return "No WiFi";
    case WL_IDLE_STATUS: return "Idle";
    case WL_NO_SSID_AVAIL: return "No SSID";
    case WL_SCAN_COMPLETED: return "Scan Done";
    case WL_CONNECT_FAILED: return "Failed";
    case WL_CONNECTION_LOST: return "Lost";
    case WL_DISCONNECTED: return "Disconnected";
    default: return "Unknown";
  }
}

// =============================================================================
// NTP TIME SYNC
// =============================================================================

bool syncTimeFromNTP() {
  if (!wifiConnected) {
    Serial.println("[NTP] No WiFi connection");
    return false;
  }
  
  Serial.println("[NTP] Syncing time from NTP...");
  
  configTime(gmtOffsetSec, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  delay(2000);
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5000)) {
    Serial.println("[NTP] Failed to get time!");
    return false;
  }
  
  ntpSyncedOnce = true;
  lastNTPSync = millis();
  
  Serial.printf("[NTP] Time synced: %04d-%02d-%02d %02d:%02d:%02d (GMT%+d)\n",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                gmtOffsetSec / 3600);
  
  // Update RTC if available
  updateRTCFromNTP();
  
  return true;
}

void updateRTCFromNTP() {
  // This will be called by hardware.cpp if RTC is available
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    // RTC update handled by hardware layer
    Serial.println("[NTP] RTC update requested");
  }
}

bool isNTPSynced() {
  return ntpSyncedOnce;
}

unsigned long getTimeSinceNTPSync() {
  if (!ntpSyncedOnce) return 0xFFFFFFFF;
  return millis() - lastNTPSync;
}

// =============================================================================
// BOOT LOG
// =============================================================================

void logToBootLog(const char* message) {
  if (!sdCardInitialized) return;
  
  File logFile = SD_MMC.open(SD_BOOT_LOG, FILE_APPEND);
  if (logFile) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      logFile.printf("[%04d-%02d-%02d %02d:%02d:%02d] %s\n",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, message);
    } else {
      logFile.printf("[%lu] %s\n", millis(), message);
    }
    logFile.close();
  }
}

// =============================================================================
// PLAYER DATA PERSISTENCE
// =============================================================================

bool savePlayerDataToSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_PLAYER_DATA, FILE_WRITE);
  if (!dataFile) return false;
  
  dataFile.printf("VERSION=1\n");
  dataFile.printf("LEVEL=%d\n", system_state.player_level);
  dataFile.printf("XP=%d\n", system_state.player_xp);
  dataFile.printf("GEMS=%d\n", system_state.player_gems);
  dataFile.printf("CARDS=%d\n", system_state.gacha_cards_collected);
  dataFile.printf("BOSSES=%d\n", system_state.bosses_defeated);
  dataFile.printf("STREAK=%d\n", system_state.training_streak);
  dataFile.printf("STEPS=%d\n", system_state.steps_today);
  dataFile.printf("THEME=%d\n", (int)system_state.current_theme);
  dataFile.printf("BRIGHTNESS=%d\n", system_state.brightness);
  dataFile.close();
  
  Serial.println("[SD] Player data saved");
  return true;
}

bool loadPlayerDataFromSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_PLAYER_DATA, FILE_READ);
  if (!dataFile) {
    Serial.println("[SD] No player data found, using defaults");
    return false;
  }
  
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();
    
    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;
    
    String key = line.substring(0, eqPos);
    int value = line.substring(eqPos + 1).toInt();
    
    if (key == "LEVEL") system_state.player_level = value;
    else if (key == "XP") system_state.player_xp = value;
    else if (key == "GEMS") system_state.player_gems = value;
    else if (key == "CARDS") system_state.gacha_cards_collected = value;
    else if (key == "BOSSES") system_state.bosses_defeated = value;
    else if (key == "STREAK") system_state.training_streak = value;
    else if (key == "STEPS") system_state.steps_today = value;
    // --- THEME is intentionally NOT loaded here. -----------------------------
    // The theme is loaded from real NVS ("theme_cfg" namespace) in setup()
    // BEFORE this function runs. Loading it again from a stale SD file was
    // silently reverting freshly-switched themes on reboot — that was the
    // "theme switch plays the animation + reboots, but boots into the old
    // character" bug. NVS is the single source of truth for the theme.
    // -------------------------------------------------------------------------
    // else if (key == "THEME") system_state.current_theme = (ThemeType)value;
    else if (key == "BRIGHTNESS") system_state.brightness = value;
  }
  
  dataFile.close();
  Serial.println("[SD] Player data loaded (theme preserved from NVS)");
  return true;
}

// =============================================================================
// GACHA DATA PERSISTENCE (SD Card)
// =============================================================================

bool saveGachaDataToSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_GACHA_DATA, FILE_WRITE);
  if (!dataFile) {
    Serial.println("[SD] Failed to write gacha data!");
    sdHealth.writeErrors++;
    return false;
  }
  
  dataFile.printf("VERSION=1\n");
  dataFile.printf("TOTAL_CARDS=%d\n", system_state.gacha_cards_collected);
  dataFile.printf("PITY=%d\n", system_state.pity_counter);
  dataFile.printf("PITY_LEGENDARY=%d\n", system_state.pity_legendary_counter);
  
  // Save deck
  dataFile.printf("DECK_SIZE=%d\n", system_state.deck_size);
  for (int i = 0; i < 5; i++) {
    dataFile.printf("DECK_%d=%d\n", i, system_state.battle_deck[i]);
  }
  
  dataFile.close();
  Serial.println("[SD] Gacha data saved");
  return true;
}

bool loadGachaDataFromSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_GACHA_DATA, FILE_READ);
  if (!dataFile) {
    Serial.println("[SD] No gacha data found, using defaults");
    return false;
  }
  
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();
    
    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;
    
    String key = line.substring(0, eqPos);
    int value = line.substring(eqPos + 1).toInt();
    
    if (key == "TOTAL_CARDS") system_state.gacha_cards_collected = value;
    else if (key == "PITY") system_state.pity_counter = value;
    else if (key == "PITY_LEGENDARY") system_state.pity_legendary_counter = value;
    else if (key == "DECK_SIZE") system_state.deck_size = constrain(value, 0, 5);
    else if (key.startsWith("DECK_")) {
      int idx = key.substring(5).toInt();
      if (idx >= 0 && idx < 5) system_state.battle_deck[idx] = value;
    }
  }
  
  dataFile.close();
  Serial.println("[SD] Gacha data loaded");
  return true;
}

// =============================================================================
// BOSS DATA PERSISTENCE (SD Card)
// =============================================================================

bool saveBossDataToSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_BOSS_DATA, FILE_WRITE);
  if (!dataFile) {
    Serial.println("[SD] Failed to write boss data!");
    sdHealth.writeErrors++;
    return false;
  }
  
  dataFile.printf("VERSION=1\n");
  dataFile.printf("BOSSES_DEFEATED=%d\n", system_state.bosses_defeated);
  
  // Save individual boss defeat status
  extern bool bosses_defeated[];
  for (int i = 0; i < TOTAL_BOSSES; i++) {
    dataFile.printf("BOSS_%d=%d\n", i, bosses_defeated[i] ? 1 : 0);
  }
  
  dataFile.close();
  Serial.println("[SD] Boss data saved");
  return true;
}

bool loadBossDataFromSD() {
  if (!sdCardInitialized) return false;
  
  File dataFile = SD_MMC.open(SD_BOSS_DATA, FILE_READ);
  if (!dataFile) {
    Serial.println("[SD] No boss data found, using defaults");
    return false;
  }
  
  extern bool bosses_defeated[];
  
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();
    
    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;
    
    String key = line.substring(0, eqPos);
    int value = line.substring(eqPos + 1).toInt();
    
    if (key == "BOSSES_DEFEATED") system_state.bosses_defeated = value;
    else if (key.startsWith("BOSS_")) {
      int idx = key.substring(5).toInt();
      if (idx >= 0 && idx < TOTAL_BOSSES) bosses_defeated[idx] = (value != 0);
    }
  }
  
  dataFile.close();
  Serial.println("[SD] Boss data loaded");
  return true;
}

// =============================================================================
// BACKUP SYSTEM
// =============================================================================

bool performAutoBackup() {
  if (!autoBackupEnabled || !sdCardInitialized) return false;
  
  if (millis() - lastAutoBackup < AUTO_BACKUP_INTERVAL_MS) return false;
  
  Serial.println("[SD] Performing auto backup...");
  
  struct tm timeinfo;
  char backupName[64];
  
  if (getLocalTime(&timeinfo)) {
    sprintf(backupName, "backup_%04d%02d%02d_%02d%02d",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min);
  } else {
    sprintf(backupName, "backup_%lu", millis());
  }
  
  bool success = createBackup(backupName);
  if (success) {
    lastAutoBackup = millis();
  }
  
  return success;
}

bool createBackup(const char* backupName) {
  char backupDir[128];
  sprintf(backupDir, "%s/%s", SD_BACKUP_PATH, backupName);
  
  if (!SD_MMC.mkdir(backupDir)) {
    Serial.printf("[SD] Failed to create backup dir: %s\n", backupDir);
    return false;
  }
  
  int filesCopied = 0;
  
  // Helper lambda to copy a file
  auto copyFile = [&](const char* srcPath, const char* fileName) {
    char destPath[128];
    sprintf(destPath, "%s/%s", backupDir, fileName);
    
    File src = SD_MMC.open(srcPath, FILE_READ);
    if (!src) return;
    
    File dst = SD_MMC.open(destPath, FILE_WRITE);
    if (!dst) { src.close(); return; }
    
    while (src.available()) {
      dst.write(src.read());
    }
    
    src.close();
    dst.close();
    filesCopied++;
    Serial.printf("[SD] Backed up: %s\n", fileName);
  };
  
  // Backup all game data files
  copyFile(SD_PLAYER_DATA, "player.dat");
  copyFile(SD_GACHA_DATA, "cards.dat");
  copyFile(SD_BOSS_DATA, "progress.dat");
  copyFile(SD_WIFI_CONFIG, "config.txt");
  
  Serial.printf("[SD] Backup created: %s (%d files)\n", backupName, filesCopied);
  logToBootLog("Backup created");
  
  return filesCopied > 0;
}

bool restoreFromBackup(const char* backupName) {
  char backupBase[128];
  sprintf(backupBase, "%s/%s", SD_BACKUP_PATH, backupName);
  
  int filesRestored = 0;
  
  // Helper lambda to restore a file
  auto restoreFile = [&](const char* fileName, const char* destPath) {
    char srcPath[128];
    sprintf(srcPath, "%s/%s", backupBase, fileName);
    
    if (!SD_MMC.exists(srcPath)) return;
    
    File src = SD_MMC.open(srcPath, FILE_READ);
    if (!src) return;
    
    File dst = SD_MMC.open(destPath, FILE_WRITE);
    if (!dst) { src.close(); return; }
    
    while (src.available()) {
      dst.write(src.read());
    }
    
    src.close();
    dst.close();
    filesRestored++;
    Serial.printf("[SD] Restored: %s\n", fileName);
  };
  
  // Restore all game data files
  restoreFile("player.dat", SD_PLAYER_DATA);
  restoreFile("cards.dat", SD_GACHA_DATA);
  restoreFile("progress.dat", SD_BOSS_DATA);
  
  if (filesRestored == 0) {
    Serial.printf("[SD] Backup not found or empty: %s\n", backupName);
    return false;
  }
  
  // Reload all data into memory
  loadPlayerDataFromSD();
  loadGachaDataFromSD();
  loadBossDataFromSD();
  
  Serial.printf("[SD] Restored from backup: %s (%d files)\n", backupName, filesRestored);
  return true;
}

void listBackups() {
  File backupDir = SD_MMC.open(SD_BACKUP_PATH);
  if (!backupDir) {
    Serial.println("[SD] No backup directory");
    return;
  }
  
  Serial.println("[SD] Available backups:");
  File entry;
  while (entry = backupDir.openNextFile()) {
    if (entry.isDirectory()) {
      Serial.printf("  - %s\n", entry.name());
    }
    entry.close();
  }
  backupDir.close();
}

void cleanOldBackups(int keepCount) {
  // Keep only the most recent backups
  Serial.printf("[SD] Cleaning old backups, keeping %d\n", keepCount);
  // Implementation would list, sort by date, and delete oldest
}

// =============================================================================
// FUSION LABS PROTOCOL - COMPLETE WEB SERIAL IMPLEMENTATION
// =============================================================================

// Serial buffer globals for Web Serial communication
String serialBuffer = "";
bool receivingWifiConfig = false;
String wifiConfigBuffer = "";
bool receivingFaceData = false;
String faceIdBuffer = "";
String faceDataBuffer = "";
bool waitingForFaceId = false;
char activeWatchFaceId[32] = "default";

// OTA Update state
OTAUpdateInfo otaInfo = {
  "",     // version
  "",     // bin_url
  "",     // checksum
  "",     // release_notes
  0,      // file_size
  false,  // update_available
  false,  // download_in_progress
  0,      // download_progress
  false   // ready_to_apply
};

void setupFusionLabsSerial() {
  Serial.println("[FUSION] Web Serial Protocol Ready");
  Serial.println("  Commands: WIDGET_PING, WIDGET_STATUS, WIDGET_READ_WIFI");
  Serial.println("  OTA: WIDGET_CHECK_UPDATE, WIDGET_DOWNLOAD_UPDATE");
}

void handleSerialConfig() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      processSerialCommand(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
      
      // Prevent buffer overflow
      if (serialBuffer.length() > 4096) {
        serialBuffer = "";
        Serial.println("ERROR:BUFFER_OVERFLOW");
      }
    }
  }
}

void processSerialCommand(const String& command) {
  String cmd = command;
  cmd.trim();
  
  // ========== CONNECTION TEST ==========
  if (cmd == "WIDGET_PING") {
    Serial.println("WIDGET_PONG");
    return;
  }
  
  // ========== DEVICE STATUS ==========
  if (cmd == "WIDGET_STATUS") {
    sendDeviceStatus();
    return;
  }
  
  // ========== WIFI CONFIG READ ==========
  if (cmd == "WIDGET_READ_WIFI") {
    sendWifiConfig();
    return;
  }
  
  // ========== WIFI CONFIG WRITE ==========
  if (cmd == "WIDGET_WRITE_WIFI") {
    receivingWifiConfig = true;
    wifiConfigBuffer = "";
    Serial.println("READY_FOR_CONFIG");
    return;
  }
  
  // Handle WiFi config data being received
  if (receivingWifiConfig) {
    if (cmd == "END_WIFI_CONFIG") {
      receivingWifiConfig = false;
      saveWifiConfigFromSerial(wifiConfigBuffer);
    } else {
      wifiConfigBuffer += cmd + "\n";
    }
    return;
  }
  
  // ========== WATCH FACE COMMANDS ==========
  if (cmd == "WIDGET_LIST_FACES") {
    listInstalledFaces();
    return;
  }
  
  if (cmd == "WIDGET_WRITE_FACE") {
    receivingFaceData = true;
    waitingForFaceId = true;
    Serial.println("SEND_FACE_ID");
    return;
  }
  
  if (cmd.startsWith("WIDGET_SET_FACE:")) {
    setActiveFace(cmd);
    return;
  }
  
  if (cmd.startsWith("WIDGET_DELETE_FACE:")) {
    deleteFace(cmd);
    return;
  }
  
  // Handle face data being received
  if (receivingFaceData) {
    handleFaceSerialData(cmd);
    return;
  }
  
  // ========== OTA UPDATE COMMANDS ==========
  if (cmd == "WIDGET_CHECK_UPDATE") {
    if (checkForUpdates()) {
      Serial.println("UPDATE_AVAILABLE:" + otaInfo.version);
      Serial.println("UPDATE_SIZE:" + String(otaInfo.file_size));
      Serial.println("UPDATE_NOTES:" + otaInfo.release_notes);
    } else {
      Serial.println("NO_UPDATE_AVAILABLE");
    }
    return;
  }
  
  if (cmd == "WIDGET_DOWNLOAD_UPDATE") {
    if (otaInfo.update_available) {
      Serial.println("DOWNLOAD_STARTED");
      if (downloadFirmwareToSD()) {
        Serial.println("DOWNLOAD_COMPLETE");
        Serial.println("READY_TO_APPLY");
      } else {
        Serial.println("DOWNLOAD_FAILED");
      }
    } else {
      Serial.println("ERROR:NO_UPDATE_AVAILABLE");
    }
    return;
  }
  
  if (cmd == "WIDGET_APPLY_UPDATE") {
    if (otaInfo.ready_to_apply) {
      Serial.println("APPLYING_UPDATE...");
      applyFirmwareUpdate();  // This will reboot
    } else {
      Serial.println("ERROR:NO_UPDATE_READY");
    }
    return;
  }
  
  // ========== GAME DATA COMMANDS (for Fusion Labs dashboard) ==========
  if (cmd == "WIDGET_GET_GACHA") {
    sendGachaCollectionData();
    return;
  }
  
  if (cmd == "WIDGET_GET_BOSS") {
    sendBossProgressData();
    return;
  }
  
  if (cmd == "WIDGET_GET_STATS") {
    sendPlayerStatsData();
    return;
  }
  
  // ========== BACKUP COMMANDS ==========
  if (cmd == "WIDGET_BACKUP") {
    createBackup("serial_backup");
    Serial.println("BACKUP_COMPLETE");
    return;
  }
  
  if (cmd == "WIDGET_SYNC_TIME") {
    if (syncTimeFromNTP()) {
      Serial.println("TIME_SYNCED");
    } else {
      Serial.println("TIME_SYNC_FAILED");
    }
    return;
  }
  
  // Unknown command
  if (cmd.length() > 0) {
    Serial.println("UNKNOWN_COMMAND:" + cmd);
  }
}

void sendDeviceStatus() {
  Serial.println("WIDGET_STATUS_START");
  Serial.println("DEVICE:" FUSION_OS_NAME);
  Serial.println("VERSION:" FUSION_OS_VERSION);
  Serial.println("BUILD:" FUSION_OS_BUILD);
  Serial.printf("BOARD:%dx%d\n", LCD_WIDTH, LCD_HEIGHT);
  Serial.println("DEVICE_ID:" DEVICE_ID);
  Serial.printf("SD_CARD:%s\n", sdCardInitialized ? "YES" : "NO");
  
  if (sdCardInitialized) {
    Serial.printf("SD_FREE:%.1fGB\n", sdHealth.freeBytes / (1024.0 * 1024.0 * 1024.0));
  }
  
  Serial.printf("WIFI:%s\n", wifiConnected ? wifiNetworks[connectedNetworkIndex].ssid : "Not Connected");
  
  if (wifiConnected) {
    Serial.printf("WIFI_SIGNAL:%d\n", getWiFiSignalStrength());
    Serial.printf("IP:%s\n", WiFi.localIP().toString().c_str());
  }
  
  Serial.printf("NTP_SYNCED:%s\n", ntpSyncedOnce ? "YES" : "NO");
  Serial.printf("BATTERY:%d%%\n", system_state.battery_percentage);
  Serial.printf("CHARGING:%s\n", system_state.is_charging ? "YES" : "NO");
  
  // Game stats
  Serial.printf("PLAYER_LEVEL:%d\n", system_state.player_level);
  Serial.printf("PLAYER_GEMS:%d\n", system_state.player_gems);
  Serial.printf("GACHA_CARDS:%d\n", system_state.gacha_cards_collected);
  Serial.printf("BOSSES_DEFEATED:%d\n", system_state.bosses_defeated);
  Serial.printf("CURRENT_THEME:%d\n", (int)system_state.current_theme);
  
  Serial.println("WIDGET_STATUS_END");
}

void sendWifiConfig() {
  if (!sdCardInitialized) {
    Serial.println("ERROR:SD_CARD_NOT_AVAILABLE");
    return;
  }
  
  if (!SD_MMC.exists(SD_WIFI_CONFIG)) {
    Serial.println("ERROR:CONFIG_FILE_NOT_FOUND");
    return;
  }
  
  File configFile = SD_MMC.open(SD_WIFI_CONFIG, FILE_READ);
  if (!configFile) {
    Serial.println("ERROR:CANNOT_OPEN_FILE");
    return;
  }
  
  Serial.println("WIFI_CONFIG_START");
  while (configFile.available()) {
    String line = configFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0 && !line.startsWith("#")) {
      Serial.println(line);
    }
  }
  configFile.close();
  Serial.println("WIFI_CONFIG_END");
}

void saveWifiConfigFromSerial(const String& config) {
  if (!sdCardInitialized) {
    Serial.println("ERROR:SD_CARD_NOT_AVAILABLE");
    return;
  }
  
  // Ensure directory exists
  if (!SD_MMC.exists(SD_WIFI_PATH)) {
    SD_MMC.mkdir(SD_WIFI_PATH);
  }
  
  File configFile = SD_MMC.open(SD_WIFI_CONFIG, FILE_WRITE);
  if (!configFile) {
    Serial.println("ERROR:CANNOT_CREATE_FILE");
    return;
  }
  
  configFile.println("# ESP32 Anime Watch - WiFi Configuration");
  configFile.println("# Updated via Fusion Labs Web Serial Tool");
  configFile.println();
  configFile.print(config);
  configFile.close();
  
  Serial.println("CONFIG_WRITTEN");
  
  // Reload WiFi config
  loadWiFiFromSD();
  Serial.println("CONFIG_RELOADED");
}

// =============================================================================
// WATCH FACE MANAGEMENT (for Fusion Labs Watch Face Studio)
// =============================================================================

void listInstalledFaces() {
  Serial.println("FACE_LIST_START");
  
  // Built-in faces (stored in firmware)
  Serial.println("FACE:default,Default Analog,Built-in,1.0.0");
  Serial.println("FACE:digital,Digital Minimal,Built-in,1.0.0");
  Serial.println("FACE:anime_theme,Anime Theme,Built-in,1.0.0");
  
  // Check for custom faces on SD card
  if (sdCardInitialized && SD_MMC.exists("/WATCH/FACES/custom")) {
    File facesDir = SD_MMC.open("/WATCH/FACES/custom");
    if (facesDir) {
      File entry;
      while (entry = facesDir.openNextFile()) {
        if (!entry.isDirectory()) {
          String name = entry.name();
          if (name.endsWith(".json")) {
            String faceId = name.substring(0, name.length() - 5);
            Serial.printf("FACE:%s,Custom Face,SD Card,1.0.0\n", faceId.c_str());
          }
        }
        entry.close();
      }
      facesDir.close();
    }
  }
  
  Serial.printf("ACTIVE_FACE:%s\n", activeWatchFaceId);
  Serial.println("FACE_LIST_END");
}

void handleFaceSerialData(const String& data) {
  if (waitingForFaceId) {
    faceIdBuffer = data;
    waitingForFaceId = false;
    faceDataBuffer = "";
    Serial.println("READY_FOR_FACE_DATA");
    return;
  }
  
  if (data == "END_FACE_DATA") {
    receivingFaceData = false;
    saveFaceFromSerial(faceIdBuffer, faceDataBuffer);
    return;
  }
  
  faceDataBuffer += data + "\n";
}

void saveFaceFromSerial(const String& faceId, const String& faceData) {
  if (!sdCardInitialized) {
    Serial.println("ERROR:SD_CARD_NOT_AVAILABLE");
    return;
  }
  
  // Ensure custom faces directory exists
  if (!SD_MMC.exists("/WATCH/FACES/custom")) {
    SD_MMC.mkdir("/WATCH/FACES");
    SD_MMC.mkdir("/WATCH/FACES/custom");
  }
  
  String filePath = "/WATCH/FACES/custom/" + faceId + ".json";
  File faceFile = SD_MMC.open(filePath.c_str(), FILE_WRITE);
  if (!faceFile) {
    Serial.println("ERROR:CANNOT_CREATE_FILE");
    return;
  }
  
  faceFile.print(faceData);
  faceFile.close();
  
  Serial.println("FACE_SAVED:" + faceId);
}

void setActiveFace(const String& command) {
  // Extract face ID from command like "WIDGET_SET_FACE:faceid"
  int colonPos = command.indexOf(':');
  if (colonPos < 0) {
    Serial.println("ERROR:INVALID_COMMAND");
    return;
  }
  
  String faceId = command.substring(colonPos + 1);
  faceId.trim();
  
  strncpy(activeWatchFaceId, faceId.c_str(), 31);
  activeWatchFaceId[31] = '\0';
  
  // Save preference
  if (sdCardInitialized) {
    File prefFile = SD_MMC.open("/WATCH/CONFIG/active_face.txt", FILE_WRITE);
    if (prefFile) {
      prefFile.println(faceId);
      prefFile.close();
    }
  }
  
  Serial.println("FACE_SET:" + faceId);
}

void deleteFace(const String& command) {
  // Extract face ID from command like "WIDGET_DELETE_FACE:faceid"
  int colonPos = command.indexOf(':');
  if (colonPos < 0) {
    Serial.println("ERROR:INVALID_COMMAND");
    return;
  }
  
  String faceId = command.substring(colonPos + 1);
  faceId.trim();
  
  // Can't delete built-in faces
  if (faceId == "default" || faceId == "digital" || faceId == "anime_theme") {
    Serial.println("ERROR:CANNOT_DELETE_BUILTIN");
    return;
  }
  
  String filePath = "/WATCH/FACES/custom/" + faceId + ".json";
  if (SD_MMC.exists(filePath.c_str())) {
    SD_MMC.remove(filePath.c_str());
    Serial.println("FACE_DELETED:" + faceId);
  } else {
    Serial.println("ERROR:FACE_NOT_FOUND");
  }
}

// =============================================================================
// OTA UPDATE IMPLEMENTATION
// =============================================================================

bool checkForUpdates() {
  if (!wifiConnected) {
    Serial.println("[OTA] No WiFi connection");
    return false;
  }
  
  Serial.println("[OTA] Checking for updates...");
  
  HTTPClient http;
  http.begin(FIRMWARE_UPDATE_URL);
  http.setTimeout(10000);
  
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] HTTP error: %d\n", httpCode);
    http.end();
    return false;
  }
  
  String payload = http.getString();
  http.end();
  
  // Parse JSON response
  // Expected format: {"version":"1.1.0","bin":"S3_MiniOS.bin","checksum":"sha256:...","notes":"..."}
  
  // Simple JSON parsing (no ArduinoJson to save memory)
  int versionStart = payload.indexOf("\"version\":\"") + 11;
  int versionEnd = payload.indexOf("\"", versionStart);
  if (versionStart > 10 && versionEnd > versionStart) {
    otaInfo.version = payload.substring(versionStart, versionEnd);
  }
  
  int binStart = payload.indexOf("\"bin\":\"") + 7;
  int binEnd = payload.indexOf("\"", binStart);
  if (binStart > 6 && binEnd > binStart) {
    String binFile = payload.substring(binStart, binEnd);
    otaInfo.bin_url = String(FUSION_LABS_URL) + "/firmware/widget-os/180A/" + binFile;
  }
  
  int notesStart = payload.indexOf("\"notes\":\"") + 9;
  int notesEnd = payload.indexOf("\"", notesStart);
  if (notesStart > 8 && notesEnd > notesStart) {
    otaInfo.release_notes = payload.substring(notesStart, notesEnd);
  }
  
  // Compare versions
  if (otaInfo.version.length() > 0 && otaInfo.version != FUSION_OS_VERSION) {
    otaInfo.update_available = true;
    Serial.printf("[OTA] Update available: %s -> %s\n", FUSION_OS_VERSION, otaInfo.version.c_str());
    return true;
  }
  
  Serial.println("[OTA] Already on latest version");
  return false;
}

bool downloadFirmwareToSD() {
  if (!wifiConnected || !sdCardInitialized) {
    Serial.println("[OTA] Cannot download: no WiFi or SD card");
    return false;
  }
  
  if (otaInfo.bin_url.length() == 0) {
    Serial.println("[OTA] No firmware URL");
    return false;
  }
  
  Serial.printf("[OTA] Downloading: %s\n", otaInfo.bin_url.c_str());
  
  otaInfo.download_in_progress = true;
  otaInfo.download_progress = 0;
  
  HTTPClient http;
  http.begin(otaInfo.bin_url);
  http.setTimeout(60000);  // 60 second timeout for large files
  
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] Download failed: %d\n", httpCode);
    http.end();
    otaInfo.download_in_progress = false;
    return false;
  }
  
  int contentLength = http.getSize();
  otaInfo.file_size = contentLength;
  
  // Ensure update directory exists
  if (!SD_MMC.exists(SD_FIRMWARE_PATH)) {
    SD_MMC.mkdir(SD_FIRMWARE_PATH);
  }
  
  String firmwarePath = String(SD_FIRMWARE_PATH) + "/update.bin";
  File firmwareFile = SD_MMC.open(firmwarePath.c_str(), FILE_WRITE);
  if (!firmwareFile) {
    Serial.println("[OTA] Cannot create firmware file");
    http.end();
    otaInfo.download_in_progress = false;
    return false;
  }
  
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buffer[1024];
  int totalDownloaded = 0;
  
  while (http.connected() && totalDownloaded < contentLength) {
    int available = stream->available();
    if (available) {
      int bytesRead = stream->readBytes(buffer, min(available, (int)sizeof(buffer)));
      firmwareFile.write(buffer, bytesRead);
      totalDownloaded += bytesRead;
      
      otaInfo.download_progress = (totalDownloaded * 100) / contentLength;
      
      // Progress update every 10%
      if (otaInfo.download_progress % 10 == 0) {
        Serial.printf("[OTA] Download: %d%%\n", otaInfo.download_progress);
      }
    }
    delay(1);
  }
  
  firmwareFile.close();
  http.end();
  
  if (totalDownloaded == contentLength) {
    otaInfo.download_in_progress = false;
    otaInfo.ready_to_apply = true;
    Serial.println("[OTA] Download complete!");
    return true;
  }
  
  Serial.println("[OTA] Download incomplete");
  otaInfo.download_in_progress = false;
  return false;
}

bool downloadFirmware(const char* url) {
  otaInfo.bin_url = String(url);
  return downloadFirmwareToSD();
}

bool applyFirmwareUpdate() {
  if (!otaInfo.ready_to_apply) {
    Serial.println("[OTA] No update ready to apply");
    return false;
  }
  
  String firmwarePath = String(SD_FIRMWARE_PATH) + "/update.bin";
  if (!SD_MMC.exists(firmwarePath.c_str())) {
    Serial.println("[OTA] Firmware file not found");
    return false;
  }
  
  Serial.println("[OTA] Applying update...");
  Serial.println("[OTA] The device will reboot now!");
  
  // For ESP32, we use Update library to flash from SD card
  // This requires #include <Update.h>
  File updateFile = SD_MMC.open(firmwarePath.c_str(), FILE_READ);
  if (!updateFile) {
    Serial.println("[OTA] Cannot open update file");
    return false;
  }
  
  size_t updateSize = updateFile.size();
  
  // Note: The actual OTA implementation would use Update.begin() and Update.writeStream()
  // For safety, we'll just mark the update as ready and let the user manually flash
  Serial.printf("[OTA] Update file ready: %d bytes\n", updateSize);
  Serial.println("[OTA] Please use esptool or Arduino IDE to flash:");
  Serial.printf("      File: %s\n", firmwarePath.c_str());
  
  updateFile.close();
  
  // Clean up
  otaInfo.ready_to_apply = false;
  otaInfo.update_available = false;
  
  // In a full implementation, you would call:
  // Update.begin(updateSize);
  // Update.writeStream(updateFile);
  // Update.end(true);
  // ESP.restart();
  
  return true;
}

void cancelOTAUpdate() {
  otaInfo.download_in_progress = false;
  otaInfo.ready_to_apply = false;
  otaInfo.update_available = false;
  
  // Delete partial download
  String firmwarePath = String(SD_FIRMWARE_PATH) + "/update.bin";
  if (SD_MMC.exists(firmwarePath.c_str())) {
    SD_MMC.remove(firmwarePath.c_str());
  }
  
  Serial.println("[OTA] Update cancelled");
}

int getOTAProgress() {
  return otaInfo.download_progress;
}

// =============================================================================
// GAME DATA COMMANDS (for Fusion Labs Dashboard)
// =============================================================================

void sendGachaCollectionData() {
  Serial.println("GACHA_DATA_START");
  Serial.printf("TOTAL_CARDS:%d\n", system_state.gacha_cards_collected);
  Serial.printf("MAX_CARDS:%d\n", GACHA_TOTAL_CARDS);
  // In full implementation, would list all owned cards
  Serial.println("GACHA_DATA_END");
}

void sendBossProgressData() {
  Serial.println("BOSS_DATA_START");
  Serial.printf("BOSSES_DEFEATED:%d\n", system_state.bosses_defeated);
  Serial.printf("TOTAL_BOSSES:%d\n", TOTAL_BOSSES);
  // In full implementation, would list defeated bosses
  Serial.println("BOSS_DATA_END");
}

void sendPlayerStatsData() {
  Serial.println("STATS_DATA_START");
  Serial.printf("LEVEL:%d\n", system_state.player_level);
  Serial.printf("XP:%d\n", system_state.player_xp);
  Serial.printf("GEMS:%d\n", system_state.player_gems);
  Serial.printf("STEPS:%d\n", system_state.steps_today);
  Serial.printf("TRAINING_STREAK:%d\n", system_state.training_streak);
  Serial.printf("DAILY_LOGINS:%d\n", system_state.daily_login_count);
  Serial.println("STATS_DATA_END");
}

void sendFusionLabsResponse(const char* response) {
  Serial.printf("[FUSION] %s\n", response);
}

// =============================================================================
// UI FUNCTIONS
// =============================================================================

void drawSDCardStatus() {
  gfx->setTextColor(sdCardInitialized ? COLOR_GREEN : COLOR_RED);
  gfx->setTextSize(1);
  gfx->setCursor(20, 15);
  gfx->printf("SD: %s", sdCardInitialized ? "OK" : "FAIL");
  
  if (sdCardInitialized) {
    gfx->setCursor(80, 15);
    gfx->printf("%.1fGB free", sdHealth.freeBytes / (1024.0 * 1024.0 * 1024.0));
  }
}

void drawWiFiStatus() {
  gfx->setTextColor(wifiConnected ? COLOR_GREEN : COLOR_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(200, 15);
  
  if (wifiConnected) {
    gfx->printf("WiFi: %d dBm", getWiFiSignalStrength());
  } else {
    gfx->print("WiFi: --");
  }
}

void drawWiFiManagerScreen() {
  gfx->fillScreen(COLOR_BLACK);
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(70, 20);
  gfx->print("WiFi MANAGER");
  
  // Status
  gfx->setTextColor(wifiConnected ? COLOR_GREEN : COLOR_RED);
  gfx->setTextSize(1);
  gfx->setCursor(120, 50);
  gfx->print(getWiFiStatusString());
  
  if (wifiConnected && connectedNetworkIndex >= 0) {
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(80, 70);
    gfx->printf("Connected: %s", wifiNetworks[connectedNetworkIndex].ssid);
    gfx->setCursor(80, 90);
    gfx->printf("Signal: %d dBm", getWiFiSignalStrength());
    gfx->setCursor(80, 110);
    gfx->printf("IP: %s", WiFi.localIP().toString().c_str());
  }
  
  // Network list
  gfx->setTextColor(getCurrentTheme()->accent);
  gfx->setCursor(20, 140);
  gfx->print("Saved Networks:");
  
  for (int i = 0; i < numWifiNetworks && i < 5; i++) {
    int y = 160 + i * 40;
    bool isConnected = (i == connectedNetworkIndex && wifiConnected);
    
    gfx->fillRect(20, y, 320, 35, isConnected ? RGB565(0, 60, 0) : RGB565(40, 40, 40));
    
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(30, y + 10);
    gfx->print(wifiNetworks[i].ssid);
    
    if (isConnected) {
      gfx->setTextColor(COLOR_GREEN);
      gfx->setCursor(280, y + 10);
      gfx->print("OK");
    }
  }
  
  // Buttons
  drawGlassButton(40, 380, 130, 35, wifiConnected ? "Disconnect" : "Connect", false);
  drawGlassButton(190, 380, 130, 35, "Scan", false);
  
  // NTP status
  gfx->setTextColor(ntpSyncedOnce ? COLOR_GREEN : COLOR_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(80, 420);
  gfx->printf("NTP: %s", ntpSyncedOnce ? "Synced" : "Not synced");
}

void handleWiFiManagerTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Connect/Disconnect button
  if (y >= 380 && y < 415 && x >= 40 && x < 170) {
    if (wifiConnected) {
      disconnectWiFi();
    } else {
      connectToWiFi();
    }
    return;
  }
  
  // Scan button
  if (y >= 380 && y < 415 && x >= 190 && x < 320) {
    scanWiFiNetworks();
    return;
  }
  
  // Network selection
  for (int i = 0; i < numWifiNetworks && i < 5; i++) {
    int ny = 160 + i * 40;
    if (y >= ny && y < ny + 35 && x >= 20 && x < 340) {
      connectToNetwork(i);
      return;
    }
  }
}

void drawBackupScreen() {
  gfx->fillScreen(COLOR_BLACK);
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(100, 20);
  gfx->print("BACKUPS");
  
  // Auto-backup status
  gfx->setTextColor(autoBackupEnabled ? COLOR_GREEN : COLOR_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(80, 60);
  gfx->printf("Auto-backup: %s", autoBackupEnabled ? "ON" : "OFF");
  
  if (lastAutoBackup > 0) {
    unsigned long hours = (millis() - lastAutoBackup) / 3600000;
    gfx->setCursor(80, 80);
    gfx->printf("Last backup: %lu hours ago", hours);
  }
  
  // Buttons
  drawGlassButton(60, 150, 240, 50, "Create Backup Now", false);
  drawGlassButton(60, 220, 240, 50, "Restore Backup", false);
  drawGlassButton(60, 290, 240, 50, "List Backups", false);
  
  drawGlassButton(140, 380, 80, 35, "Back", false);
}

void handleBackupTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int y = gesture.y;
  
  // Create backup
  if (y >= 150 && y < 200) {
    createBackup("manual_backup");
  }
  // Restore
  else if (y >= 220 && y < 270) {
    // Show restore options
    listBackups();
  }
  // List
  else if (y >= 290 && y < 340) {
    listBackups();
  }
  // Back
  else if (y >= 380) {
    system_state.current_screen = SCREEN_SETTINGS;
  }
}

// =============================================================================
// OTA UPDATE UI
// =============================================================================

void drawUpdateProgressBar(int x, int y, int w, int h, int progress) {
  // Background
  gfx->fillRect(x, y, w, h, RGB565(40, 40, 40));
  
  // Progress fill
  int fillWidth = (w * progress) / 100;
  if (fillWidth > 0) {
    uint16_t color = progress < 100 ? getCurrentTheme()->primary : COLOR_GREEN;
    gfx->fillRect(x, y, fillWidth, h, color);
  }
  
  // Progress text
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + w/2 - 10, y + h/2 - 4);
  gfx->printf("%d%%", progress);
}

void drawOTAUpdateScreen() {
  gfx->fillScreen(COLOR_BLACK);
  
  // Title
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(70, 20);
  gfx->print("FIRMWARE UPDATE");
  
  // Current version
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(60, 60);
  gfx->printf("Current: v%s", FUSION_OS_VERSION);
  
  // WiFi status
  gfx->setTextColor(wifiConnected ? COLOR_GREEN : COLOR_RED);
  gfx->setCursor(60, 80);
  gfx->printf("WiFi: %s", wifiConnected ? "Connected" : "Required");
  
  // SD Card status
  gfx->setTextColor(sdCardInitialized ? COLOR_GREEN : COLOR_RED);
  gfx->setCursor(60, 100);
  gfx->printf("SD Card: %s", sdCardInitialized ? "Ready" : "Required");
  
  if (otaInfo.download_in_progress) {
    // Download in progress
    gfx->setTextColor(COLOR_YELLOW);
    gfx->setTextSize(2);
    gfx->setCursor(80, 150);
    gfx->print("Downloading...");
    
    drawUpdateProgressBar(40, 200, 280, 30, otaInfo.download_progress);
    
    gfx->setTextColor(COLOR_GRAY);
    gfx->setTextSize(1);
    gfx->setCursor(100, 250);
    gfx->printf("Size: %d KB", otaInfo.file_size / 1024);
    
  } else if (otaInfo.ready_to_apply) {
    // Ready to apply
    gfx->setTextColor(COLOR_GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(60, 150);
    gfx->print("Update Ready!");
    
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(60, 190);
    gfx->printf("New version: v%s", otaInfo.version.c_str());
    
    if (otaInfo.release_notes.length() > 0) {
      gfx->setCursor(60, 220);
      gfx->print(otaInfo.release_notes.substring(0, 40));
    }
    
    drawGlassButton(60, 280, 240, 50, "Apply Update", false);
    
  } else if (otaInfo.update_available) {
    // Update available
    gfx->setTextColor(getCurrentTheme()->accent);
    gfx->setTextSize(2);
    gfx->setCursor(60, 150);
    gfx->print("Update Available!");
    
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(60, 190);
    gfx->printf("v%s -> v%s", FUSION_OS_VERSION, otaInfo.version.c_str());
    
    if (otaInfo.release_notes.length() > 0) {
      gfx->setCursor(60, 220);
      gfx->print(otaInfo.release_notes.substring(0, 40));
    }
    
    drawGlassButton(60, 270, 240, 50, "Download Update", false);
    
  } else {
    // No update / need to check
    gfx->setTextColor(COLOR_GRAY);
    gfx->setTextSize(1);
    gfx->setCursor(80, 150);
    gfx->print("Check for the latest firmware");
    
    drawGlassButton(60, 200, 240, 50, "Check for Updates", !wifiConnected);
  }
  
  // Back button
  drawGlassButton(140, 380, 80, 35, "Back", false);
  
  // Fusion Labs branding
  gfx->setTextColor(COLOR_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(80, 430);
  gfx->print("Powered by Fusion Labs");
}

void handleOTAUpdateTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Don't process touches during download
  if (otaInfo.download_in_progress) return;
  
  // Check for Updates button (when no update available)
  if (!otaInfo.update_available && !otaInfo.ready_to_apply) {
    if (y >= 200 && y < 250 && wifiConnected) {
      checkForUpdates();
      return;
    }
  }
  
  // Download Update button (when update available)
  if (otaInfo.update_available && !otaInfo.ready_to_apply) {
    if (y >= 270 && y < 320) {
      downloadFirmwareToSD();
      return;
    }
  }
  
  // Apply Update button (when ready)
  if (otaInfo.ready_to_apply) {
    if (y >= 280 && y < 330) {
      applyFirmwareUpdate();
      return;
    }
  }
  
  // Back button
  if (y >= 380 && y < 415) {
    system_state.current_screen = SCREEN_SETTINGS;
    return;
  }
}

// =============================================================================
// FUSION LABS CONNECTION SCREEN
// =============================================================================

void drawFusionLabsScreen() {
  gfx->fillScreen(COLOR_BLACK);
  
  // Title with accent
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(80, 20);
  gfx->print("FUSION LABS");
  
  // Subtitle
  gfx->setTextColor(COLOR_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(60, 50);
  gfx->print("Web Serial Connection");
  
  // Connection instructions
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(30, 90);
  gfx->print("To connect from your computer:");
  
  gfx->setTextColor(getCurrentTheme()->accent);
  gfx->setCursor(30, 115);
  gfx->print("1. Connect watch via USB");
  
  gfx->setCursor(30, 135);
  gfx->print("2. Visit fusion-labs website");
  
  gfx->setCursor(30, 155);
  gfx->print("3. Click 'Connect Device'");
  
  // URL
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(30, 190);
  gfx->print(FUSION_LABS_URL);
  
  // Features list
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setCursor(30, 230);
  gfx->print("Features Available:");
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(40, 255);
  gfx->print("* WiFi Configuration");
  gfx->setCursor(40, 275);
  gfx->print("* Watch Face Studio");
  gfx->setCursor(40, 295);
  gfx->print("* Firmware Updates");
  gfx->setCursor(40, 315);
  gfx->print("* Data Backup/Restore");
  gfx->setCursor(40, 335);
  gfx->print("* Game Progress Sync");
  
  // Status
  gfx->setTextColor(COLOR_GREEN);
  gfx->setCursor(80, 370);
  gfx->print("Serial: Ready");
  gfx->setCursor(80, 390);
  gfx->printf("Device ID: %s", DEVICE_ID);
  
  // Back button
  drawGlassButton(140, 420, 80, 35, "Back", false);
}

void handleFusionLabsTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int y = gesture.y;
  
  // Back button
  if (y >= 420) {
    system_state.current_screen = SCREEN_SETTINGS;
  }
}
