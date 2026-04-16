/*
 * sd_preferences.cpp - SD Card-based Preferences replacement
 * 
 * Stores all data in /WATCH/FusionData/<namespace>.txt on the SD card.
 * Drop-in replacement for ESP32 Preferences library.
 */

#include "sd_preferences.h"

// =============================================================================
// CONSTRUCTOR / DESTRUCTOR
// =============================================================================

SDPreferences::SDPreferences() : _readOnly(false), _opened(false) {}

SDPreferences::~SDPreferences() {
    if (_opened) end();
}

// =============================================================================
// INIT / BEGIN / END
// =============================================================================

bool SDPreferences::initFusionDataFolder() {
    // Create folder structure if it doesn't exist
    if (!SD_MMC.exists("/WATCH")) {
        SD_MMC.mkdir("/WATCH");
    }
    if (!SD_MMC.exists(FUSION_DATA_ROOT)) {
        SD_MMC.mkdir(FUSION_DATA_ROOT);
        Serial.println("[FusionData] Created /WATCH/FusionData/");
    }
    return SD_MMC.exists(FUSION_DATA_ROOT);
}

bool SDPreferences::begin(const char* name, bool readOnly) {
    if (_opened) end();  // Close previous if still open
    
    _namespace = String(name);
    _filePath = String(FUSION_DATA_ROOT) + "/" + _namespace + ".txt";
    _readOnly = readOnly;
    _opened = true;
    _data.clear();
    
    // Ensure folder exists
    initFusionDataFolder();
    
    // Load existing data from file
    loadFromFile();
    
    return true;
}

void SDPreferences::end() {
    if (_opened && !_readOnly) {
        saveToFile();
    }
    _data.clear();
    _opened = false;
}

// =============================================================================
// CLEAR / REMOVE
// =============================================================================

bool SDPreferences::clear() {
    _data.clear();
    if (SD_MMC.exists(_filePath.c_str())) {
        SD_MMC.remove(_filePath.c_str());
        Serial.printf("[FusionData] Cleared namespace: %s\n", _namespace.c_str());
    }
    return true;
}

bool SDPreferences::remove(const char* key) {
    auto it = _data.find(String(key));
    if (it != _data.end()) {
        _data.erase(it);
        if (!_readOnly) saveToFile();
        return true;
    }
    return false;
}

// =============================================================================
// INTEGER TYPES
// =============================================================================

size_t SDPreferences::putInt(const char* key, int32_t value) {
    _data[String(key)] = String(value);
    if (!_readOnly) saveToFile();
    return sizeof(int32_t);
}

int32_t SDPreferences::getInt(const char* key, int32_t defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return it->second.toInt();
    return defaultValue;
}

size_t SDPreferences::putUInt(const char* key, uint32_t value) {
    _data[String(key)] = String(value);
    if (!_readOnly) saveToFile();
    return sizeof(uint32_t);
}

uint32_t SDPreferences::getUInt(const char* key, uint32_t defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return (uint32_t)strtoul(it->second.c_str(), NULL, 10);
    return defaultValue;
}

size_t SDPreferences::putShort(const char* key, int16_t value) {
    _data[String(key)] = String((int)value);
    if (!_readOnly) saveToFile();
    return sizeof(int16_t);
}

int16_t SDPreferences::getShort(const char* key, int16_t defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return (int16_t)it->second.toInt();
    return defaultValue;
}

size_t SDPreferences::putLong(const char* key, int64_t value) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%lld", (long long)value);
    _data[String(key)] = String(buf);
    if (!_readOnly) saveToFile();
    return sizeof(int64_t);
}

int64_t SDPreferences::getLong(const char* key, int64_t defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return strtoll(it->second.c_str(), NULL, 10);
    return defaultValue;
}

size_t SDPreferences::putUChar(const char* key, uint8_t value) {
    _data[String(key)] = String((int)value);
    if (!_readOnly) saveToFile();
    return sizeof(uint8_t);
}

uint8_t SDPreferences::getUChar(const char* key, uint8_t defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return (uint8_t)it->second.toInt();
    return defaultValue;
}

// =============================================================================
// BOOLEAN
// =============================================================================

size_t SDPreferences::putBool(const char* key, bool value) {
    _data[String(key)] = value ? "1" : "0";
    if (!_readOnly) saveToFile();
    return 1;
}

bool SDPreferences::getBool(const char* key, bool defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return it->second.toInt() != 0;
    return defaultValue;
}

// =============================================================================
// FLOAT
// =============================================================================

size_t SDPreferences::putFloat(const char* key, float value) {
    _data[String(key)] = String(value, 6);
    if (!_readOnly) saveToFile();
    return sizeof(float);
}

float SDPreferences::getFloat(const char* key, float defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return it->second.toFloat();
    return defaultValue;
}

// =============================================================================
// STRING
// =============================================================================

size_t SDPreferences::putString(const char* key, const char* value) {
    _data[String(key)] = String(value);
    if (!_readOnly) saveToFile();
    return strlen(value);
}

size_t SDPreferences::putString(const char* key, const String& value) {
    return putString(key, value.c_str());
}

String SDPreferences::getString(const char* key, const String& defaultValue) {
    auto it = _data.find(String(key));
    if (it != _data.end()) return it->second;
    return defaultValue;
}

// =============================================================================
// BINARY DATA (stored as hex string prefixed with "HEX:")
// =============================================================================

size_t SDPreferences::putBytes(const char* key, const void* value, size_t len) {
    String hex = "HEX:" + bytesToHex((const uint8_t*)value, len);
    _data[String(key)] = hex;
    if (!_readOnly) saveToFile();
    return len;
}

size_t SDPreferences::getBytes(const char* key, void* buf, size_t maxLen) {
    auto it = _data.find(String(key));
    if (it == _data.end()) return 0;
    
    String val = it->second;
    if (!val.startsWith("HEX:")) return 0;
    
    String hexStr = val.substring(4);
    return hexToBytes(hexStr, (uint8_t*)buf, maxLen);
}

// =============================================================================
// UTILITY
// =============================================================================

bool SDPreferences::isKey(const char* key) {
    return _data.find(String(key)) != _data.end();
}

// =============================================================================
// FILE I/O
// =============================================================================

bool SDPreferences::loadFromFile() {
    if (!SD_MMC.exists(_filePath.c_str())) {
        return false;
    }
    
    File file = SD_MMC.open(_filePath.c_str(), FILE_READ);
    if (!file) return false;
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        // Skip empty lines and comments
        if (line.length() == 0 || line.startsWith("#")) continue;
        
        int eqPos = line.indexOf('=');
        if (eqPos > 0) {
            String key = line.substring(0, eqPos);
            String value = line.substring(eqPos + 1);
            key.trim();
            value.trim();
            _data[key] = value;
        }
    }
    
    file.close();
    return true;
}

bool SDPreferences::saveToFile() {
    if (_readOnly) return false;
    
    // Ensure directory exists
    if (!SD_MMC.exists(FUSION_DATA_ROOT)) {
        SD_MMC.mkdir(FUSION_DATA_ROOT);
    }
    
    File file = SD_MMC.open(_filePath.c_str(), FILE_WRITE);
    if (!file) {
        Serial.printf("[FusionData] ERROR: Cannot write %s\n", _filePath.c_str());
        return false;
    }
    
    file.printf("# FusionData: %s\n", _namespace.c_str());
    
    for (auto& kv : _data) {
        file.printf("%s=%s\n", kv.first.c_str(), kv.second.c_str());
    }
    
    file.close();
    return true;
}

// =============================================================================
// HEX ENCODING FOR BINARY DATA
// =============================================================================

String SDPreferences::bytesToHex(const uint8_t* data, size_t len) {
    String hex;
    hex.reserve(len * 2);
    for (size_t i = 0; i < len; i++) {
        char buf[3];
        snprintf(buf, sizeof(buf), "%02X", data[i]);
        hex += buf;
    }
    return hex;
}

size_t SDPreferences::hexToBytes(const String& hex, uint8_t* buf, size_t maxLen) {
    size_t hexLen = hex.length();
    size_t byteCount = hexLen / 2;
    if (byteCount > maxLen) byteCount = maxLen;
    
    for (size_t i = 0; i < byteCount; i++) {
        char hi = hex.charAt(i * 2);
        char lo = hex.charAt(i * 2 + 1);
        
        uint8_t b = 0;
        b |= (hi >= 'A' ? (hi - 'A' + 10) : (hi - '0')) << 4;
        b |= (lo >= 'A' ? (lo - 'A' + 10) : (lo - '0'));
        buf[i] = b;
    }
    
    return byteCount;
}

// =============================================================================
// STATIC: RESET ALL FUSION DATA
// =============================================================================

bool SDPreferences::resetAllFusionData() {
    if (!SD_MMC.exists(FUSION_DATA_ROOT)) return true;
    
    File dir = SD_MMC.open(FUSION_DATA_ROOT);
    if (!dir || !dir.isDirectory()) return false;
    
    // Collect file paths first (can't delete while iterating)
    String files[32];
    int count = 0;
    
    File entry = dir.openNextFile();
    while (entry && count < 32) {
        if (!entry.isDirectory()) {
            files[count++] = String(FUSION_DATA_ROOT) + "/" + entry.name();
        }
        entry = dir.openNextFile();
    }
    dir.close();
    
    // Delete all files
    for (int i = 0; i < count; i++) {
        SD_MMC.remove(files[i].c_str());
        Serial.printf("[FusionData] Deleted: %s\n", files[i].c_str());
    }
    
    Serial.printf("[FusionData] Reset complete - %d files removed\n", count);
    return true;
}
