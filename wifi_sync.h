/*
 * wifi_sync.h - WiFi Boot Sync Header (ENHANCED)
 * FUSION OS Feature - With SD Card WiFi Support
 * 
 * FIXES:
 * - Boot loop when WiFi unavailable
 * - SD card WiFi network loading
 * - Multi-network support (hardcoded + SD)
 * - Time backup/restore from SD
 * - Auto timezone detection via IP geolocation
 */

#ifndef WIFI_SYNC_H
#define WIFI_SYNC_H

#include <Arduino.h>

// Maximum WiFi networks (1 hardcoded + 5 from SD card = 6 total)
#define MAX_WIFI_NETWORKS 6

// SD Card paths for WiFi config
#define SD_WIFI_PATH "/WATCH/wifi"
#define SD_WIFI_CONFIG_FILE "/WATCH/wifi/config.txt"
#define SD_TIME_BACKUP_PATH "/WATCH/TIME"
#define SD_TIME_BACKUP_FILE "/WATCH/TIME/backup.txt"

// WiFi credential structure
typedef struct {
    char ssid[64];
    char password[64];
    bool valid;
    bool isFromSD;
} WiFiCredential;

// WiFi sync state
typedef struct {
    bool sync_enabled;
    bool last_sync_success;
    unsigned long last_sync_time;
    int networks_loaded;
    WiFiCredential networks[MAX_WIFI_NETWORKS];
} WiFiSyncState;

// External sync state
extern WiFiSyncState wifi_sync_state;

// =============================================================================
// MAIN FUNCTIONS
// =============================================================================

// Initialize WiFi sync system (call once in setup)
void initWiFiSync();

// Perform boot sync - SAFE: Never causes boot loop
// Returns true if sync successful, false otherwise (boot continues either way)
bool performBootSync();

// Manual sync trigger
bool triggerManualSync();

// =============================================================================
// NETWORK MANAGEMENT
// =============================================================================

// Load WiFi networks (hardcoded + SD card)
int loadWiFiNetworks();

// Load additional networks from SD card
int loadWiFiNetworksFromSD();

// Quick network scan - check if any known network is visible
bool quickNetworkScan();

// Connect to specific WiFi network
bool connectToWiFi(const char* ssid, const char* password, int timeout_ms);

// Disconnect WiFi to save power
void disconnectWiFi();

// =============================================================================
// TIME SYNC
// =============================================================================

// Sync time via NTP
bool syncNTPTime();

// Save time backup to SD card (for offline boot)
bool saveTimeBackup();

// Restore time from SD backup
bool restoreTimeBackup();

// Check if time backup exists
bool hasTimeBackup();

// Delete time backup (after successful NTP sync)
void deleteTimeBackup();

// =============================================================================
// AUTO TIMEZONE DETECTION
// =============================================================================

// Fetch location and timezone from IP address
bool fetchLocationFromIP();

// Get detected city name
const char* getDetectedCity();

// Get detected country code
const char* getDetectedCountry();

// =============================================================================
// WEATHER
// =============================================================================

// Fetch weather data via Open-Meteo API (free, no key needed)
bool fetchWeather();

// Weather data getters (populated after fetchWeather() succeeds)
float getWeatherTemp();
float getWeatherHumidity();
float getWeatherWindSpeed();
int getWeatherPressure();
const char* getWeatherDescription();
const char* getWeatherIcon();
bool isWeatherValid();

// =============================================================================
// STATUS FUNCTIONS
// =============================================================================

// Get time since last successful sync
unsigned long timeSinceLastSync();

// Check if WiFi is currently connected
bool isWiFiConnected();

// Get number of loaded networks
int getNetworkCount();

// =============================================================================
// TIMEZONE CONFIGURATION
// =============================================================================

void setTimezoneOffset(int hours, int minutes);
int getTimezoneOffsetHours();
int getTimezoneOffsetMinutes();
void setDaylightSaving(bool enabled);
bool isDaylightSavingEnabled();
void getTimezoneString(char* buffer, size_t bufferSize);

// =============================================================================
// SD CARD FOLDER MANAGEMENT
// =============================================================================

// Create required folders if they don't exist
bool createWiFiFolders();

// Create example config file if none exists
bool createExampleConfig();

#endif // WIFI_SYNC_H