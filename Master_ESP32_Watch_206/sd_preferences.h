/*
 * sd_preferences.h - SD Card-based Preferences replacement
 * 
 * Drop-in replacement for ESP32 Preferences library that stores
 * data on SD card at /WATCH/FusionData/<namespace>.txt
 * 
 * Uses same API as Preferences so migration is just:
 *   #include "sd_preferences.h"
 *   SDPreferences prefs;  // instead of: Preferences prefs;
 * 
 * Data format: Simple key=value text files (human-readable/editable)
 * Binary data (putBytes/getBytes): stored as hex strings
 */

#ifndef SD_PREFERENCES_H
#define SD_PREFERENCES_H

#include <Arduino.h>
#include <SD_MMC.h>
#include <FS.h>
#include <map>

// Base path for all FusionData on SD card
#define FUSION_DATA_ROOT "/WATCH/FusionData"

// Max entries per namespace (safety limit)
#define SD_PREFS_MAX_ENTRIES 128

class SDPreferences {
public:
    SDPreferences();
    ~SDPreferences();
    
    // Mimic Preferences API
    bool begin(const char* name, bool readOnly = false);
    void end();
    
    // Clear all keys in current namespace
    bool clear();
    
    // Remove a specific key
    bool remove(const char* key);
    
    // Integer types
    size_t putInt(const char* key, int32_t value);
    int32_t getInt(const char* key, int32_t defaultValue = 0);
    
    size_t putUInt(const char* key, uint32_t value);
    uint32_t getUInt(const char* key, uint32_t defaultValue = 0);
    
    size_t putShort(const char* key, int16_t value);
    int16_t getShort(const char* key, int16_t defaultValue = 0);
    
    size_t putLong(const char* key, int64_t value);
    int64_t getLong(const char* key, int64_t defaultValue = 0);
    
    size_t putUChar(const char* key, uint8_t value);
    uint8_t getUChar(const char* key, uint8_t defaultValue = 0);
    
    // Boolean
    size_t putBool(const char* key, bool value);
    bool getBool(const char* key, bool defaultValue = false);
    
    // Float/Double
    size_t putFloat(const char* key, float value);
    float getFloat(const char* key, float defaultValue = 0.0f);
    
    // String
    size_t putString(const char* key, const char* value);
    size_t putString(const char* key, const String& value);
    String getString(const char* key, const String& defaultValue = String());
    
    // Binary data (stored as hex string in the text file)
    size_t putBytes(const char* key, const void* value, size_t len);
    size_t getBytes(const char* key, void* buf, size_t maxLen);
    
    // Utility
    bool isKey(const char* key);
    
    // Static: ensure FusionData folder exists (call once at boot)
    static bool initFusionDataFolder();
    
    // Static: delete ALL FusionData (for reset)
    static bool resetAllFusionData();

private:
    String _namespace;
    String _filePath;
    bool _readOnly;
    bool _opened;
    std::map<String, String> _data;  // In-memory key-value store
    
    bool loadFromFile();
    bool saveToFile();
    String bytesToHex(const uint8_t* data, size_t len);
    size_t hexToBytes(const String& hex, uint8_t* buf, size_t maxLen);
};

#endif // SD_PREFERENCES_H
