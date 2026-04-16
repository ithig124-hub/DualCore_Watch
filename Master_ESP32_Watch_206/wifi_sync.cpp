/*
 * wifi_sync.cpp - WiFi Boot Sync Implementation (FIXED)
 * FUSION OS Feature - With SD Card WiFi Support
 *
 * NOTE: SD Card is initialized by sd_manager.cpp - we just use it here
 */

#include "wifi_sync.h"
#include "config.h"
#include "pin_config.h"
#include "hardware.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <SD_MMC.h>
#include <FS.h>

// =============================================================================
// SD CARD TOGGLE - Set to false if hardware conflict with display
// =============================================================================
#define SD_CARD_ENABLED true

// =============================================================================
// CONFIGURATION
// =============================================================================

// Timezone configuration (auto-detected or manual)
static int timezone_offset_hours = 8;    // Default GMT+8 (will be auto-detected)
static int timezone_offset_minutes = 0;
static bool daylight_saving = false;

// Detected location info
static char detectedCity[64] = "Unknown";
static char detectedCountry[8] = "??";
static bool locationDetected = false;
static float detectedLat = 0.0f;
static float detectedLon = 0.0f;

// Cached weather data (fetched via Open-Meteo)
static float weatherTemp = 0.0f;
static float weatherHumidity = 0.0f;
static float weatherWindSpeed = 0.0f;
static int weatherPressure = 0;
static char weatherDescription[32] = "";
static char weatherIcon[8] = "";
static bool weatherValid = false;

// SD Card state - USE THE ONE FROM sd_manager.cpp
extern bool sdCardInitialized;  // From sd_manager.cpp
#define sdCardMounted sdCardInitialized  // Alias for compatibility

// External declarations
extern void feedWatchdog();
extern WiFiState wifi_state;
extern SystemState system_state;

// =============================================================================
// GLOBAL STATE
// =============================================================================

WiFiSyncState wifi_sync_state = {
    .sync_enabled = true,
    .last_sync_success = false,
    .last_sync_time = 0,
    .networks_loaded = 0
};

// Hardcoded WiFi network (PRIMARY - always slot 0)
const char* HARDCODED_SSID = "Optus_9D2E3D";
const char* HARDCODED_PASSWORD = "snucktemptGLeQU";

// =============================================================================
// CONSTANTS - SAFE TIMEOUTS (prevent boot loop)
// Total worst case: scan(4s) + 6 networks × 1 retry × 4s = 28s + NTP(4s) + HTTP(4s) = 36s
// Watchdog set to 40s during WiFi phase to cover this
// =============================================================================
#define WIFI_RETRY_PER_NETWORK 1       // 1 attempt per network (fast fail, move to next)
#define WIFI_CONNECT_TIMEOUT 4000      // 4 seconds per network attempt
#define NTP_TIMEOUT 4000               // 4 seconds for NTP
#define QUICK_SCAN_TIMEOUT 200         // 200ms per WiFi channel
#define HTTP_TIMEOUT 4000              // 4 seconds for HTTP requests
#define TOTAL_WIFI_BOOT_TIMEOUT 30000  // 30 seconds MAX for entire WiFi boot phase

// SD Card paths
#define SD_WIFI_PATH "/WATCH/wifi"
#define SD_WIFI_CONFIG_FILE "/WATCH/wifi/config.txt"
#define SD_TIME_BACKUP_PATH "/WATCH/TIME"
#define SD_TIME_BACKUP_FILE "/WATCH/TIME/backup.txt"

// =============================================================================
// SD CARD FOLDER MANAGEMENT (Updated for SD_MMC)
// =============================================================================

bool createWiFiFolders() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) {
        Serial.println("[WiFiSync] SD card not mounted, skipping folder creation");
        return false;
    }
    
    Serial.println("[WiFiSync] Checking/creating WiFi folders on SD card...");
    
    // Create WATCH root folder
    if (!SD_MMC.exists("/WATCH")) {
        if (SD_MMC.mkdir("/WATCH")) {
            Serial.println("[WiFiSync] Created /WATCH folder");
        }
    }
    
    // Create wifi folder
    if (!SD_MMC.exists(SD_WIFI_PATH)) {
        if (SD_MMC.mkdir(SD_WIFI_PATH)) {
            Serial.println("[WiFiSync] Created /WATCH/wifi folder");
        }
    }
    
    // Create TIME folder for backups
    if (!SD_MMC.exists(SD_TIME_BACKUP_PATH)) {
        if (SD_MMC.mkdir(SD_TIME_BACKUP_PATH)) {
            Serial.println("[WiFiSync] Created /WATCH/TIME folder");
        }
    }
    
    return true;
#else
    return false;
#endif
}

bool createExampleConfig() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) return false;
    
    // Only create if config doesn't exist
    if (SD_MMC.exists(SD_WIFI_CONFIG_FILE)) {
        Serial.println("[WiFiSync] Config file already exists, skipping creation");
        return true;
    }
    
    Serial.println("[WiFiSync] Creating example config file...");
    
    File configFile = SD_MMC.open(SD_WIFI_CONFIG_FILE, FILE_WRITE);
    if (!configFile) {
        Serial.println("[WiFiSync] Failed to create config file");
        return false;
    }
    
    // Write example config
    // NOTE: Use same format as sd_manager.cpp: SSID1=, PASSWORD1=
    configFile.println("# WiFi Configuration File for ESP32 Watch");
    configFile.println("# Add your WiFi networks below (up to 5 networks)");
    configFile.println("# Format: SSIDn=YourNetworkName");
    configFile.println("#         PASSWORDn=YourPassword");
    configFile.println("# Leave password blank for open networks");
    configFile.println("#");
    configFile.println("# Timezone is AUTO-DETECTED from your IP location!");
    configFile.println("# But you can override it here if needed:");
    configFile.println("# GMT_OFFSET=8");
    configFile.println("#");
    configFile.println("# Example:");
    configFile.println("# SSID1=HomeNetwork");
    configFile.println("# PASSWORD1=secretpassword");
    configFile.println("");
    configFile.println("# Network 1 (Primary)");
    configFile.println("SSID1=");
    configFile.println("PASSWORD1=");
    configFile.println("");
    configFile.println("# Network 2");
    configFile.println("SSID2=");
    configFile.println("PASSWORD2=");
    configFile.println("");
    configFile.println("# Network 3");
    configFile.println("SSID3=");
    configFile.println("PASSWORD3=");
    configFile.println("");
    configFile.println("# Network 4");
    configFile.println("SSID4=");
    configFile.println("PASSWORD4=");
    configFile.println("");
    configFile.println("# Network 5");
    configFile.println("SSID5=");
    configFile.println("PASSWORD5=");
    
    configFile.close();
    
    Serial.println("[WiFiSync] Example config file created");
    return true;
#else
    return false;
#endif
}

// =============================================================================
// LOAD WIFI NETWORKS FROM SD CARD (Updated for SD_MMC)
// =============================================================================

int loadWiFiNetworksFromSD() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) {
        Serial.println("[WiFiSync] SD card not mounted, skipping SD WiFi load");
        return 0;
    }
    
    // First, create folders if they don't exist
    createWiFiFolders();
    
    // Create example config if none exists
    createExampleConfig();
    
    // Now try to read the config file
    if (!SD_MMC.exists(SD_WIFI_CONFIG_FILE)) {
        Serial.println("[WiFiSync] No config file found on SD card");
        return 0;
    }
    
    File configFile = SD_MMC.open(SD_WIFI_CONFIG_FILE, FILE_READ);
    if (!configFile) {
        Serial.println("[WiFiSync] Failed to open config file");
        return 0;
    }
    
    Serial.println("[WiFiSync] Reading WiFi config from SD card...");
    
    int networksAdded = 0;
    char line[128];
    char currentSSID[64] = "";
    char currentPass[64] = "";
    bool hasSSID = false;
    
    // Parse the config file (up to 5 networks from SD)
    // ==========================================================================
    // FIX: Support BOTH config formats:
    //   Format A (sd_manager.cpp): SSID1=MyNetwork, PASSWORD1=MyPass
    //   Format B (wifi_sync.cpp):  WIFI1_SSID=MyNetwork, WIFI1_PASS=MyPass
    //   Format C (legacy):         SSID=MyNetwork, PASS=MyPass
    // ==========================================================================
    while (configFile.available() && networksAdded < 5) {
        // Read a line
        int lineLen = 0;
        while (configFile.available() && lineLen < 127) {
            char c = configFile.read();
            if (c == '\n' || c == '\r') break;
            line[lineLen++] = c;
        }
        line[lineLen] = (char)0;
        
        // Skip empty lines and comments
        if (lineLen == 0 || line[0] == '#') continue;
        
        // Find the '=' separator
        char* eqSign = strchr(line, '=');
        if (!eqSign) continue;
        
        // Extract key and value
        *eqSign = '\0';  // Split at '='
        char* key = line;
        char* value = eqSign + 1;
        
        // Trim leading whitespace from value
        while (*value == ' ') value++;
        
        // =================================================================
        // Match SSID patterns: "SSID1", "WIFI1_SSID", "SSID", etc.
        // =================================================================
        bool isSSID = false;
        bool isPASS = false;
        
        // Check for SSID patterns (case: SSID1, SSID2, ..., SSID, WIFI1_SSID, etc.)
        if (strstr(key, "SSID") != NULL && strstr(key, "PASS") == NULL) {
            isSSID = true;
        }
        // Check for PASSWORD/PASS patterns (case: PASSWORD1, PASS, WIFI1_PASS, etc.)
        else if (strstr(key, "PASS") != NULL) {
            isPASS = true;
        }
        // Parse manual timezone override
        else if (strcmp(key, "GMT_OFFSET") == 0) {
            timezone_offset_hours = atoi(value);
            Serial.printf("[WiFiSync] Manual timezone override: GMT%+d\n", timezone_offset_hours);
            *eqSign = '=';  // Restore line
            continue;
        }
        
        *eqSign = '=';  // Restore line
        
        if (isSSID && strlen(value) > 0) {
            strncpy(currentSSID, value, sizeof(currentSSID) - 1);
            currentSSID[sizeof(currentSSID) - 1] = (char)0;
            hasSSID = true;
            Serial.printf("[WiFiSync] Found SSID: %s\n", currentSSID);
        }
        else if (isPASS) {
            strncpy(currentPass, value, sizeof(currentPass) - 1);
            currentPass[sizeof(currentPass) - 1] = (char)0;
            
            // If we have an SSID, add the network
            if (hasSSID && strlen(currentSSID) > 0) {
                // Find next available slot (slot 0 is hardcoded)
                int slot = wifi_sync_state.networks_loaded;
                if (slot < MAX_WIFI_NETWORKS) {
                    strncpy(wifi_sync_state.networks[slot].ssid, currentSSID, sizeof(wifi_sync_state.networks[slot].ssid) - 1);
                    strncpy(wifi_sync_state.networks[slot].password, currentPass, sizeof(wifi_sync_state.networks[slot].password) - 1);
                    wifi_sync_state.networks[slot].valid = true;
                    wifi_sync_state.networks[slot].isFromSD = true;
                    wifi_sync_state.networks_loaded++;
                    networksAdded++;
                    Serial.printf("[WiFiSync] Added SD network [%d]: %s\n", slot, currentSSID);
                }
                
                // Reset for next network
                hasSSID = false;
                currentSSID[0] = (char)0;
                currentPass[0] = (char)0;
            }
        }
    }
    
    configFile.close();
    
    Serial.printf("[WiFiSync] Loaded %d networks from SD card\n", networksAdded);
    return networksAdded;
#else
    return 0;
#endif
}

// =============================================================================
// LOAD ALL WIFI NETWORKS (Hardcoded + SD)
// =============================================================================

int loadWiFiNetworks() {
    int count = 0;
    
    // Clear all networks first
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        wifi_sync_state.networks[i].valid = false;
        wifi_sync_state.networks[i].ssid[0] = (char)0;
        wifi_sync_state.networks[i].password[0] = (char)0;
        wifi_sync_state.networks[i].isFromSD = false;
    }
    
    // Add hardcoded network as PRIMARY (slot 0)
    strncpy(wifi_sync_state.networks[0].ssid, HARDCODED_SSID, sizeof(wifi_sync_state.networks[0].ssid) - 1);
    strncpy(wifi_sync_state.networks[0].password, HARDCODED_PASSWORD, sizeof(wifi_sync_state.networks[0].password) - 1);
    wifi_sync_state.networks[0].valid = true;
    wifi_sync_state.networks[0].isFromSD = false;
    wifi_sync_state.networks_loaded = 1;
    count = 1;
    
    Serial.println("[WiFiSync] OK Hardcoded WiFi added as primary network");
    Serial.printf("[WiFiSync]   Primary: %s\n", HARDCODED_SSID);
    
#if SD_CARD_ENABLED
    // Load additional networks from SD card (slots 1+)
    int sdNetworks = loadWiFiNetworksFromSD();
    count += sdNetworks;
#else
    Serial.println("[WiFiSync] INFO SD Card disabled (hardware conflict with display)");
    Serial.println("[WiFiSync] -> To add more networks, update wifi_sync.cpp hardcoded credentials");
#endif
    
    Serial.printf("[WiFiSync] Total networks available: %d\n", wifi_sync_state.networks_loaded);
    
    return count;
}

// =============================================================================
// AUTO TIMEZONE DETECTION VIA IP GEOLOCATION
// =============================================================================

bool fetchLocationFromIP() {
    if (WiFi.status() != WL_CONNECTED) return false;
    
    Serial.println("[WiFiSync] === AUTO TIMEZONE DETECTION ===");
    Serial.println("[WiFiSync] Fetching location from IP...");
    feedWatchdog();
    
    HTTPClient http;
    
    // Use ip-api.com (free, no API key needed, includes timezone + lat/lon)
    http.begin("http://ip-api.com/json/?fields=status,country,countryCode,city,timezone,offset,lat,lon");
    http.setTimeout(HTTP_TIMEOUT);
    
    int httpCode = http.GET();
    feedWatchdog();
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[WiFiSync] IP geolocation failed: HTTP %d\n", httpCode);
        http.end();
        return false;
    }
    
    String payload = http.getString();
    http.end();
    feedWatchdog();
    
    // Parse JSON response
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        Serial.printf("[WiFiSync] JSON parse error: %s\n", error.c_str());
        return false;
    }
    
    // Check status
    const char* status = doc["status"] | "fail";
    if (strcmp(status, "success") != 0) {
        Serial.println("[WiFiSync] IP geolocation returned fail status");
        return false;
    }
    
    // Extract location data
    const char* city = doc["city"] | "Unknown";
    const char* countryCode = doc["countryCode"] | "??";
    const char* timezone = doc["timezone"] | "Unknown";
    int offsetSeconds = doc["offset"] | 0;  // Offset in seconds from UTC
    float lat = doc["lat"] | 0.0f;
    float lon = doc["lon"] | 0.0f;
    
    // Store detected location
    strncpy(detectedCity, city, sizeof(detectedCity) - 1);
    strncpy(detectedCountry, countryCode, sizeof(detectedCountry) - 1);
    detectedLat = lat;
    detectedLon = lon;
    locationDetected = true;
    
    // Convert offset from seconds to hours/minutes
    timezone_offset_hours = offsetSeconds / 3600;
    timezone_offset_minutes = (abs(offsetSeconds) % 3600) / 60;
    
    Serial.println("[WiFiSync] =======================================");
    Serial.println("[WiFiSync] OK LOCATION DETECTED!");
    Serial.printf("[WiFiSync]   City: %s, %s\n", detectedCity, detectedCountry);
    Serial.printf("[WiFiSync]   Lat/Lon: %.4f, %.4f\n", detectedLat, detectedLon);
    Serial.printf("[WiFiSync]   Timezone: %s\n", timezone);
    Serial.printf("[WiFiSync]   UTC Offset: GMT%+d:%02d (%d seconds)\n", 
        timezone_offset_hours, timezone_offset_minutes, offsetSeconds);
    Serial.println("[WiFiSync] =======================================");
    
    return true;
}

const char* getDetectedCity() {
    return detectedCity;
}

const char* getDetectedCountry() {
    return detectedCountry;
}

// =============================================================================
// TIME BACKUP/RESTORE (FOR OFFLINE BOOT) - Updated for SD_MMC
// =============================================================================

bool saveTimeBackup() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) return false;
    
    // Get current time from RTC
    WatchTime current_time = getCurrentTime();
    
    // Ensure folder exists
    if (!SD_MMC.exists(SD_TIME_BACKUP_PATH)) {
        SD_MMC.mkdir(SD_TIME_BACKUP_PATH);
    }
    
    File backupFile = SD_MMC.open(SD_TIME_BACKUP_FILE, FILE_WRITE);
    if (!backupFile) {
        Serial.println("[WiFiSync] Failed to create time backup file");
        return false;
    }
    
    // Save time and timezone info
    backupFile.printf("%04d-%02d-%02d %02d:%02d:%02d\n",
        current_time.year, current_time.month, current_time.day,
        current_time.hour, current_time.minute, current_time.second);
    backupFile.printf("TZ_OFFSET=%d\n", timezone_offset_hours);
    backupFile.printf("TZ_MINUTES=%d\n", timezone_offset_minutes);
    backupFile.printf("CITY=%s\n", detectedCity);
    backupFile.printf("COUNTRY=%s\n", detectedCountry);
    
    backupFile.close();
    
    Serial.printf("[WiFiSync] Time backup saved: %04d-%02d-%02d %02d:%02d:%02d (GMT%+d)\n",
        current_time.year, current_time.month, current_time.day,
        current_time.hour, current_time.minute, current_time.second,
        timezone_offset_hours);
    
    return true;
#else
    return false;
#endif
}

bool hasTimeBackup() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) return false;
    return SD_MMC.exists(SD_TIME_BACKUP_FILE);
#else
    return false;
#endif
}

bool restoreTimeBackup() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) return false;
    
    if (!SD_MMC.exists(SD_TIME_BACKUP_FILE)) {
        Serial.println("[WiFiSync] No time backup file found");
        return false;
    }
    
    File backupFile = SD_MMC.open(SD_TIME_BACKUP_FILE, FILE_READ);
    if (!backupFile) {
        Serial.println("[WiFiSync] Failed to open time backup file");
        return false;
    }
    
    char line[128];
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    
    while (backupFile.available()) {
        int lineLen = 0;
        while (backupFile.available() && lineLen < 127) {
            char c = backupFile.read();
            if (c == '\n' || c == '\r') break;
            line[lineLen++] = c;
        }
        line[lineLen] = (char)0;
        
        if (lineLen == 0) continue;
        
        // Parse time line (first line)
        if (sscanf(line, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
            // Time parsed
        }
        // Parse timezone offset
        else if (strstr(line, "TZ_OFFSET=") != NULL) {
            timezone_offset_hours = atoi(line + 10);
        }
        else if (strstr(line, "TZ_MINUTES=") != NULL) {
            timezone_offset_minutes = atoi(line + 11);
        }
        else if (strstr(line, "CITY=") != NULL) {
            strncpy(detectedCity, line + 5, sizeof(detectedCity) - 1);
        }
        else if (strstr(line, "COUNTRY=") != NULL) {
            strncpy(detectedCountry, line + 8, sizeof(detectedCountry) - 1);
        }
    }
    
    backupFile.close();
    
    if (year > 2020) {
        WatchTime restored_time;
        restored_time.year = year;
        restored_time.month = month;
        restored_time.day = day;
        restored_time.hour = hour;
        restored_time.minute = minute;
        restored_time.second = second;
        restored_time.weekday = 0;
        
        setCurrentTime(restored_time);
        
        Serial.printf("[WiFiSync] Time restored: %04d-%02d-%02d %02d:%02d:%02d\n",
            year, month, day, hour, minute, second);
        Serial.printf("[WiFiSync] Timezone restored: GMT%+d (%s, %s)\n",
            timezone_offset_hours, detectedCity, detectedCountry);
        
        return true;
    }
    
    Serial.println("[WiFiSync] Failed to parse time backup");
    return false;
#else
    return false;
#endif
}

void deleteTimeBackup() {
#if SD_CARD_ENABLED
    if (!sdCardMounted) return;
    
    if (SD_MMC.exists(SD_TIME_BACKUP_FILE)) {
        SD_MMC.remove(SD_TIME_BACKUP_FILE);
        Serial.println("[WiFiSync] Old time backup deleted (fresh sync)");
    }
#endif
}

// =============================================================================
// QUICK NETWORK SCAN - Check if any known network is visible
// =============================================================================

bool quickNetworkScan() {
    Serial.println("[WiFiSync] Quick scan: Checking for available networks...");
    feedWatchdog();
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(50);
    feedWatchdog();
    
    // Use ASYNC scan to avoid blocking the watchdog
    Serial.println("[WiFiSync] Starting async scan...");
    WiFi.scanNetworks(true);  // true = async
    
    // Wait for scan with watchdog feeding (max 4 seconds)
    unsigned long scanStart = millis();
    int scanResult = WIFI_SCAN_RUNNING;
    while (scanResult == WIFI_SCAN_RUNNING && (millis() - scanStart) < 4000) {
        delay(100);
        feedWatchdog();
        scanResult = WiFi.scanComplete();
    }
    
    feedWatchdog();
    
    if (scanResult <= 0) {
        Serial.println("[WiFiSync] Quick scan: NO NETWORKS FOUND");
        Serial.println("[WiFiSync] -> Skipping WiFi sync (no networks in range)");
        WiFi.scanDelete();
        WiFi.mode(WIFI_OFF);
        return false;
    }
    
    Serial.printf("[WiFiSync] Quick scan: Found %d networks\n", scanResult);
    
    bool anyTargetFound = false;
    for (int net = 0; net < wifi_sync_state.networks_loaded; net++) {
        if (!wifi_sync_state.networks[net].valid) continue;
        
        for (int i = 0; i < scanResult; i++) {
            feedWatchdog();
            String foundSSID = WiFi.SSID(i);
            if (foundSSID.equals(wifi_sync_state.networks[net].ssid)) {
                anyTargetFound = true;
                Serial.printf("[WiFiSync] Quick scan: FOUND '%s' (RSSI: %d dBm) [%s]\n",
                    wifi_sync_state.networks[net].ssid,
                    WiFi.RSSI(i),
                    wifi_sync_state.networks[net].isFromSD ? "SD" : "hardcoded");
            }
        }
    }
    
    WiFi.scanDelete();
    
    if (!anyTargetFound) {
        Serial.println("[WiFiSync] Quick scan: No known networks visible");
        Serial.println("[WiFiSync] -> Skipping WiFi sync (known networks not in range)");
        WiFi.mode(WIFI_OFF);
        return false;
    }
    
    Serial.println("[WiFiSync] Quick scan: Known network(s) available, proceeding with sync");
    return true;
}

// =============================================================================
// INITIALIZE WIFI SYNC - SD Card is already initialized by sd_manager.cpp
// =============================================================================

void initWiFiSync() {
    Serial.println("[WiFiSync] === Initializing WiFi Sync System ===");
    Serial.println("[WiFiSync] Auto-timezone detection: ENABLED");
    Serial.printf("[WiFiSync] Default timezone: GMT%+d\n", timezone_offset_hours);
    
    feedWatchdog();  // FIX: Feed before WiFi.mode() which can block
    WiFi.mode(WIFI_OFF);
    delay(100);
    feedWatchdog();
    
    // NOTE: SD Card is already initialized by sd_manager.cpp
    // We just check if it's available
#if SD_CARD_ENABLED
    Serial.printf("[WiFiSync] SD Card: %s\n", sdCardMounted ? "Available" : "Not available");
#else
    Serial.println("[WiFiSync] SD Card: DISABLED (hardware conflict mode)");
#endif
    
    wifi_sync_state.networks_loaded = loadWiFiNetworks();
    
    Serial.printf("[WiFiSync] Loaded %d WiFi network(s)\n", wifi_sync_state.networks_loaded);
    Serial.println("[WiFiSync] === WiFi Sync System Ready ===");
}

// =============================================================================
// CONNECT TO WIFI
// =============================================================================

bool connectToWiFi(const char* ssid, const char* password, int timeout_ms) {
    feedWatchdog();  // FIX: Feed before WiFi.mode() which can block
    WiFi.mode(WIFI_STA);
    feedWatchdog();  // FIX: Feed before WiFi.begin() which can block
    WiFi.begin(ssid, password);
    feedWatchdog();
    
    unsigned long start = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(250);
        feedWatchdog();
        Serial.print(".");
    }
    Serial.println();
    
    feedWatchdog();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFiSync] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        return true;
    }
    
    feedWatchdog();  // FIX: Feed before WiFi.disconnect() which can block
    WiFi.disconnect();
    feedWatchdog();
    return false;
}

// =============================================================================
// PERFORM BOOT SYNC - SAFE: Never causes boot loop
// =============================================================================

bool performBootSync() {
    if (!wifi_sync_state.sync_enabled) {
        Serial.println("[WiFiSync] Sync disabled, skipping");
        feedWatchdog();
        return false;
    }
    
    Serial.println("[WiFiSync] =======================================");
    Serial.println("[WiFiSync]          BOOT SYNC START");
    Serial.println("[WiFiSync] =======================================");
    feedWatchdog();
    
    unsigned long bootSyncStart = millis();  // Track total time
    
    // CRITICAL: Quick scan first - skip if no known networks visible
    if (!quickNetworkScan()) {
        Serial.println("[WiFiSync] =======================================");
        Serial.println("[WiFiSync]    BOOT SYNC SKIPPED (No WiFi found)");
        Serial.println("[WiFiSync]    -> Boot continues using RTC time");
        Serial.println("[WiFiSync] =======================================");
        
        wifi_sync_state.last_sync_success = false;
        
        // Fallback 1: Try SD card time backup
        if (hasTimeBackup()) {
            Serial.println("[WiFiSync] Restoring timezone from SD backup...");
            restoreTimeBackup();
        }
        
        // Fallback 2: RTC chip always has time (PCF85063 runs on battery)
        WatchTime rtc_time = getCurrentTime();
        Serial.printf("[WiFiSync] RTC time available: %04d-%02d-%02d %02d:%02d:%02d\n",
            rtc_time.year, rtc_time.month, rtc_time.day,
            rtc_time.hour, rtc_time.minute, rtc_time.second);
        
        WiFi.mode(WIFI_OFF);
        return false;
    }
    
    Serial.printf("[WiFiSync] Attempting to connect to %d network(s)...\n",
        wifi_sync_state.networks_loaded);
    feedWatchdog();
    
    bool connected = false;
    
    for (int net = 0; net < wifi_sync_state.networks_loaded && !connected; net++) {
        // SAFETY: Check total boot timeout
        if ((millis() - bootSyncStart) > TOTAL_WIFI_BOOT_TIMEOUT) {
            Serial.println("[WiFiSync] TOTAL BOOT TIMEOUT reached - aborting WiFi");
            break;
        }
        
        WiFiCredential* cred = &wifi_sync_state.networks[net];
        if (!cred->valid) continue;
        
        Serial.printf("[WiFiSync] Trying [%d]: %s (%s)\n", 
            net, cred->ssid, cred->isFromSD ? "SD card" : "hardcoded");
        feedWatchdog();
        
        for (int attempt = 1; attempt <= WIFI_RETRY_PER_NETWORK && !connected; attempt++) {
            // SAFETY: Check total boot timeout inside retry loop too
            if ((millis() - bootSyncStart) > TOTAL_WIFI_BOOT_TIMEOUT) {
                Serial.println("[WiFiSync] TOTAL BOOT TIMEOUT reached - aborting WiFi");
                break;
            }
            
            Serial.printf("[WiFiSync]   Attempt %d/%d...\n", attempt, WIFI_RETRY_PER_NETWORK);
            feedWatchdog();
            
            if (connectToWiFi(cred->ssid, cred->password, WIFI_CONNECT_TIMEOUT)) {
                connected = true;
                Serial.printf("[WiFiSync] OK Connected to: %s\n", cred->ssid);
                break;
            } else {
                Serial.printf("[WiFiSync] X Failed (attempt %d)\n", attempt);
                feedWatchdog();
                delay(200);
            }
        }
    }
    
    feedWatchdog();
    
    if (!connected) {
        Serial.println("[WiFiSync] =======================================");
        Serial.println("[WiFiSync]    FAILED to connect to any network");
        Serial.println("[WiFiSync]    -> Boot continues using RTC time");
        Serial.println("[WiFiSync] =======================================");
        
        WiFi.mode(WIFI_OFF);
        wifi_sync_state.last_sync_success = false;
        
        // Fallback 1: Try SD card time backup for timezone info
        if (hasTimeBackup()) {
            Serial.println("[WiFiSync] Restoring timezone from SD backup...");
            restoreTimeBackup();
        }
        
        // Fallback 2: RTC chip always has time
        WatchTime rtc_time = getCurrentTime();
        Serial.printf("[WiFiSync] RTC time available: %04d-%02d-%02d %02d:%02d:%02d\n",
            rtc_time.year, rtc_time.month, rtc_time.day,
            rtc_time.hour, rtc_time.minute, rtc_time.second);
        
        return false;
    }
    
    // ===================================================================
    // CONNECTED! Now sync everything
    // ===================================================================
    Serial.println("[WiFiSync] OK WiFi connected, syncing...");
    
    // Step 1: Auto-detect timezone from IP (BEFORE NTP sync)
    bool location_detected = fetchLocationFromIP();
    feedWatchdog();
    
    // Step 2: Sync time via NTP (uses detected timezone)
    bool time_synced = syncNTPTime();
    feedWatchdog();
    
    // Step 3: Fetch weather (optional)
    bool weather_fetched = fetchWeather();
    feedWatchdog();
    
    // Save backup with timezone info
    if (time_synced) {
        deleteTimeBackup();
        saveTimeBackup();
    }
    
    wifi_sync_state.last_sync_success = (time_synced || weather_fetched);
    wifi_sync_state.last_sync_time = millis();
    
    // Disconnect to save power
    disconnectWiFi();
    
    Serial.println("[WiFiSync] =======================================");
    Serial.println("[WiFiSync]        BOOT SYNC COMPLETE");
    Serial.printf("[WiFiSync]   Location: %s (%s, %s)\n",
        location_detected ? "OK Detected" : "X Failed",
        detectedCity, detectedCountry);
    Serial.printf("[WiFiSync]   Timezone: GMT%+d:%02d\n", 
        timezone_offset_hours, timezone_offset_minutes);
    Serial.printf("[WiFiSync]   Time: %s | Weather: %s\n",
        time_synced ? "OK Synced" : "X Failed",
        weather_fetched ? "OK Fetched" : "X Failed");
    Serial.println("[WiFiSync] =======================================");
    
    return wifi_sync_state.last_sync_success;
}

// =============================================================================
// SYNC NTP TIME - With Timezone Support
// =============================================================================

bool syncNTPTime() {
    if (WiFi.status() != WL_CONNECTED) return false;
    
    Serial.println("[WiFiSync] Syncing NTP time from time.google.com...");
    Serial.printf("[WiFiSync] Using timezone: GMT%+d:%02d\n", 
        timezone_offset_hours, timezone_offset_minutes);
    
    long gmtOffset_sec = (timezone_offset_hours * 3600) + (timezone_offset_minutes * 60);
    int daylightOffset_sec = daylight_saving ? 3600 : 0;
    
    configTime(gmtOffset_sec, daylightOffset_sec,
        "time.google.com",
        "time1.google.com",
        "time2.google.com");
    
    struct tm timeinfo;
    unsigned long start = millis();
    int attempts = 0;
    
    Serial.println("[WiFiSync] Waiting for NTP response...");
    
    while (!getLocalTime(&timeinfo) && (millis() - start) < NTP_TIMEOUT) {
        delay(100);
        feedWatchdog();
        attempts++;
        if (attempts % 10 == 0) {
            Serial.printf("[WiFiSync] Still waiting... (%d ms)\n", (int)(millis() - start));
        }
    }
    
    feedWatchdog();
    
    if (getLocalTime(&timeinfo)) {
        WatchTime new_time;
        new_time.year = timeinfo.tm_year + 1900;
        new_time.month = timeinfo.tm_mon + 1;
        new_time.day = timeinfo.tm_mday;
        new_time.hour = timeinfo.tm_hour;
        new_time.minute = timeinfo.tm_min;
        new_time.second = timeinfo.tm_sec;
        new_time.weekday = timeinfo.tm_wday;
        
        setCurrentTime(new_time);
        
        Serial.println("[WiFiSync] ========== TIME SYNC SUCCESS ==========");
        Serial.printf("[WiFiSync] UTC Time would be: %02d:%02d:%02d\n",
                      (new_time.hour - timezone_offset_hours + 24) % 24,
                      new_time.minute,
                      new_time.second);
        Serial.printf("[WiFiSync] Local Time (GMT%+d): %04d-%02d-%02d %02d:%02d:%02d\n",
                      timezone_offset_hours,
                      new_time.year,
                      new_time.month,
                      new_time.day,
                      new_time.hour,
                      new_time.minute,
                      new_time.second);
        Serial.printf("[WiFiSync] Day of week: %d (0=Sun, 1=Mon, ...)\n", new_time.weekday);
        Serial.println("[WiFiSync] ==========================================");
        
        return true;
    }
    
    Serial.println("[WiFiSync] X NTP sync failed - timeout");
    Serial.println("[WiFiSync] Check: WiFi connected? Firewall blocking NTP?");
    return false;
}

// =============================================================================
// FETCH WEATHER VIA OPEN-METEO (Free, no API key needed)
// =============================================================================

// Convert WMO weather code to description and icon
static void wmoCodeToDescription(int code, char* desc, size_t descSize, char* icon, size_t iconSize) {
    switch (code) {
        case 0:  strncpy(desc, "Clear sky", descSize); strncpy(icon, "01d", iconSize); break;
        case 1:  strncpy(desc, "Mainly clear", descSize); strncpy(icon, "02d", iconSize); break;
        case 2:  strncpy(desc, "Partly cloudy", descSize); strncpy(icon, "03d", iconSize); break;
        case 3:  strncpy(desc, "Overcast", descSize); strncpy(icon, "04d", iconSize); break;
        case 45: case 48: strncpy(desc, "Foggy", descSize); strncpy(icon, "50d", iconSize); break;
        case 51: case 53: case 55: strncpy(desc, "Drizzle", descSize); strncpy(icon, "09d", iconSize); break;
        case 61: strncpy(desc, "Light rain", descSize); strncpy(icon, "10d", iconSize); break;
        case 63: strncpy(desc, "Moderate rain", descSize); strncpy(icon, "10d", iconSize); break;
        case 65: strncpy(desc, "Heavy rain", descSize); strncpy(icon, "09d", iconSize); break;
        case 66: case 67: strncpy(desc, "Freezing rain", descSize); strncpy(icon, "13d", iconSize); break;
        case 71: strncpy(desc, "Light snow", descSize); strncpy(icon, "13d", iconSize); break;
        case 73: strncpy(desc, "Moderate snow", descSize); strncpy(icon, "13d", iconSize); break;
        case 75: strncpy(desc, "Heavy snow", descSize); strncpy(icon, "13d", iconSize); break;
        case 77: strncpy(desc, "Snow grains", descSize); strncpy(icon, "13d", iconSize); break;
        case 80: case 81: case 82: strncpy(desc, "Rain showers", descSize); strncpy(icon, "09d", iconSize); break;
        case 85: case 86: strncpy(desc, "Snow showers", descSize); strncpy(icon, "13d", iconSize); break;
        case 95: strncpy(desc, "Thunderstorm", descSize); strncpy(icon, "11d", iconSize); break;
        case 96: case 99: strncpy(desc, "Thunderstorm+hail", descSize); strncpy(icon, "11d", iconSize); break;
        default: strncpy(desc, "Unknown", descSize); strncpy(icon, "01d", iconSize); break;
    }
}

bool fetchWeather() {
    if (WiFi.status() != WL_CONNECTED) return false;
    
    if (!locationDetected || (detectedLat == 0.0f && detectedLon == 0.0f)) {
        Serial.println("[WiFiSync] Weather: No location data, skipping");
        return false;
    }
    
    Serial.println("[WiFiSync] === FETCHING WEATHER (Open-Meteo) ===");
    Serial.printf("[WiFiSync] Location: %s, %s (%.4f, %.4f)\n", 
        detectedCity, detectedCountry, detectedLat, detectedLon);
    feedWatchdog();
    
    HTTPClient http;
    
    // Open-Meteo API - free, no key needed
    // Request current weather: temperature, humidity, wind, pressure, weather code
    char url[256];
    snprintf(url, sizeof(url),
        "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
        "&current=temperature_2m,relative_humidity_2m,wind_speed_10m,surface_pressure,weather_code",
        detectedLat, detectedLon);
    
    Serial.printf("[WiFiSync] URL: %s\n", url);
    
    http.begin(url);
    http.setTimeout(HTTP_TIMEOUT);
    
    int httpCode = http.GET();
    feedWatchdog();
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[WiFiSync] Weather fetch failed: HTTP %d\n", httpCode);
        http.end();
        return false;
    }
    
    String payload = http.getString();
    http.end();
    feedWatchdog();
    
    // Parse JSON response
    // Response format: { "current": { "temperature_2m": 22.5, "relative_humidity_2m": 65, ... } }
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        Serial.printf("[WiFiSync] Weather JSON parse error: %s\n", error.c_str());
        return false;
    }
    
    // Extract current weather data
    JsonObject current = doc["current"];
    if (current.isNull()) {
        Serial.println("[WiFiSync] Weather: No 'current' data in response");
        return false;
    }
    
    weatherTemp = current["temperature_2m"] | 0.0f;
    weatherHumidity = current["relative_humidity_2m"] | 0.0f;
    weatherWindSpeed = current["wind_speed_10m"] | 0.0f;
    weatherPressure = (int)(current["surface_pressure"] | 0.0f);
    int weatherCode = current["weather_code"] | 0;
    
    // Convert WMO weather code to human-readable description
    wmoCodeToDescription(weatherCode, weatherDescription, sizeof(weatherDescription),
                         weatherIcon, sizeof(weatherIcon));
    
    weatherValid = true;
    
    Serial.println("[WiFiSync] ========== WEATHER FETCHED ==========");
    Serial.printf("[WiFiSync]   Temperature: %.1f C\n", weatherTemp);
    Serial.printf("[WiFiSync]   Humidity:    %.0f%%\n", weatherHumidity);
    Serial.printf("[WiFiSync]   Wind:        %.1f km/h\n", weatherWindSpeed);
    Serial.printf("[WiFiSync]   Pressure:    %d hPa\n", weatherPressure);
    Serial.printf("[WiFiSync]   Condition:   %s (WMO code %d)\n", weatherDescription, weatherCode);
    Serial.println("[WiFiSync] ========================================");
    
    return true;
}

// =============================================================================
// DISCONNECT WIFI
// =============================================================================

void disconnectWiFi() {
    feedWatchdog();  // FIX: Feed before WiFi operations that can block
    WiFi.disconnect(true);
    feedWatchdog();
    WiFi.mode(WIFI_OFF);
    wifi_state = WIFI_DISCONNECTED;
    system_state.wifi_connected = false;
    Serial.println("[WiFiSync] WiFi disconnected (power save)");
}

// =============================================================================
// STATUS FUNCTIONS
// =============================================================================

unsigned long timeSinceLastSync() {
    if (wifi_sync_state.last_sync_time == 0) return 0xFFFFFFFF;
    return millis() - wifi_sync_state.last_sync_time;
}

int getNetworkCount() {
    return wifi_sync_state.networks_loaded;
}

// =============================================================================
// MANUAL SYNC TRIGGER
// =============================================================================

bool triggerManualSync() {
    Serial.println("[WiFiSync] Manual sync triggered");
    return performBootSync();
}

// =============================================================================
// TIMEZONE CONFIGURATION FUNCTIONS
// =============================================================================

void setTimezoneOffset(int hours, int minutes) {
    timezone_offset_hours = hours;
    timezone_offset_minutes = minutes;
    Serial.printf("[WiFiSync] Timezone set to GMT%+d:%02d\n", hours, minutes);
}

int getTimezoneOffsetHours() {
    return timezone_offset_hours;
}

int getTimezoneOffsetMinutes() {
    return timezone_offset_minutes;
}

void setDaylightSaving(bool enabled) {
    daylight_saving = enabled;
    Serial.printf("[WiFiSync] Daylight saving: %s\n", enabled ? "ON" : "OFF");
}

bool isDaylightSavingEnabled() {
    return daylight_saving;
}

void getTimezoneString(char* buffer, size_t bufferSize) {
    if (timezone_offset_minutes == 0) {
        snprintf(buffer, bufferSize, "GMT%+d", timezone_offset_hours);
    } else {
        snprintf(buffer, bufferSize, "GMT%+d:%02d", timezone_offset_hours, abs(timezone_offset_minutes));
    }
}


// =============================================================================
// WEATHER DATA GETTERS (for wifi_apps.cpp Weather App)
// =============================================================================

float getWeatherTemp() { return weatherTemp; }
float getWeatherHumidity() { return weatherHumidity; }
float getWeatherWindSpeed() { return weatherWindSpeed; }
int getWeatherPressure() { return weatherPressure; }
const char* getWeatherDescription() { return weatherDescription; }
const char* getWeatherIcon() { return weatherIcon; }
bool isWeatherValid() { return weatherValid; }

