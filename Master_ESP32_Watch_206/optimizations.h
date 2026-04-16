/*
 * optimizations.h - Performance & Stability Enhancements
 * Memory management, watchdog, smooth animations, data persistence
 */

#ifndef OPTIMIZATIONS_H
#define OPTIMIZATIONS_H

#include <Arduino.h>
#include "sd_preferences.h"
#include <esp_task_wdt.h>

// =============================================================================
// MEMORY MANAGEMENT
// =============================================================================

// Check available memory
inline void printMemoryStatus() {
  Serial.printf("[MEM] Free Heap: %d bytes | Largest Block: %d bytes\n", 
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  Serial.printf("[MEM] PSRAM: %d / %d bytes free\n", 
                ESP.getFreePsram(), ESP.getPsramSize());
}

// Allocate from PSRAM if available, else internal RAM
inline void* smartMalloc(size_t size) {
  void* ptr = nullptr;
  if (ESP.getPsramSize() > 0) {
    ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!ptr) {
    ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  return ptr;
}

inline void smartFree(void* ptr) {
  if (ptr) heap_caps_free(ptr);
}

// Memory warning threshold
#define LOW_MEMORY_THRESHOLD 50000  // 50KB

inline bool isMemoryLow() {
  return ESP.getFreeHeap() < LOW_MEMORY_THRESHOLD;
}

// =============================================================================
// WATCHDOG TIMER
// =============================================================================

#define WDT_TIMEOUT_SECONDS 30

inline void initWatchdog() {
  // ESP-IDF 5.x API - uses config struct
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);  // Add current task
  Serial.println("[WDT] Watchdog initialized");
}

inline void feedWatchdog() {
  esp_task_wdt_reset();
}

// =============================================================================
// SMOOTH ANIMATION HELPERS
// =============================================================================

// Non-blocking delay using millis()
class NonBlockingDelay {
public:
  NonBlockingDelay(unsigned long interval = 0) : _interval(interval), _lastTime(0) {}
  
  void setInterval(unsigned long interval) { _interval = interval; }
  
  bool isReady() {
    unsigned long now = millis();
    if (now - _lastTime >= _interval) {
      _lastTime = now;
      return true;
    }
    return false;
  }
  
  void reset() { _lastTime = millis(); }
  
private:
  unsigned long _interval;
  unsigned long _lastTime;
};

// Smooth value interpolation (for animations)
// Renamed to avoid conflict with std::lerp in C++20
inline float lerpValue(float start, float end, float t) {
  return start + (end - start) * constrain(t, 0.0f, 1.0f);
}

// Ease-out animation curve
inline float easeOut(float t) {
  return 1.0f - (1.0f - t) * (1.0f - t);
}

// Ease-in-out animation curve
inline float easeInOut(float t) {
  return t < 0.5f ? 2.0f * t * t : 1.0f - pow(-2.0f * t + 2.0f, 2) / 2.0f;
}

// Frame rate limiter
class FrameLimiter {
public:
  FrameLimiter(int targetFPS = 30) : _frameTime(1000 / targetFPS), _lastFrame(0) {}
  
  bool shouldRender() {
    unsigned long now = millis();
    if (now - _lastFrame >= _frameTime) {
      _lastFrame = now;
      return true;
    }
    return false;
  }
  
  void setFPS(int fps) { _frameTime = 1000 / fps; }
  
private:
  unsigned long _frameTime;
  unsigned long _lastFrame;
};

// =============================================================================
// DATA PERSISTENCE (NVS)
// =============================================================================

class DataPersistence {
public:
  static SDPreferences prefs;
  
  static void begin() {
    prefs.begin("esp32watch", false);
    Serial.println("[SD] FusionData preferences initialized");
  }
  
  static void end() {
    prefs.end();
  }
  
  // Player data
  static void savePlayerData(int level, int xp, int gems) {
    prefs.putInt("p_level", level);
    prefs.putInt("p_xp", xp);
    prefs.putInt("p_gems", gems);
  }
  
  static void loadPlayerData(int& level, int& xp, int& gems) {
    level = prefs.getInt("p_level", 1);
    xp = prefs.getInt("p_xp", 0);
    gems = prefs.getInt("p_gems", 500);
  }
  
  // Theme preference
  static void saveTheme(int theme) {
    prefs.putInt("theme", theme);
  }
  
  static int loadTheme() {
    return prefs.getInt("theme", 0);
  }
  
  // Step data
  static void saveSteps(int steps, int day) {
    prefs.putInt("steps", steps);
    prefs.putInt("step_day", day);
  }
  
  static void loadSteps(int& steps, int& day) {
    steps = prefs.getInt("steps", 0);
    day = prefs.getInt("step_day", 0);
  }
  
  // Boss progress (bitmap for 20 bosses)
  static void saveBossProgress(uint32_t defeated) {
    prefs.putUInt("bosses", defeated);
  }
  
  static uint32_t loadBossProgress() {
    return prefs.getUInt("bosses", 0);
  }
  
  // Gacha cards (100 cards = 4 x uint32_t)
  static void saveGachaCards(uint32_t* cardBits) {
    prefs.putBytes("gacha", cardBits, 16);
  }
  
  static void loadGachaCards(uint32_t* cardBits) {
    prefs.getBytes("gacha", cardBits, 16);
  }
  
  // Training streak
  static void saveTrainingStreak(int streak, int lastDay) {
    prefs.putInt("t_streak", streak);
    prefs.putInt("t_day", lastDay);
  }
  
  static void loadTrainingStreak(int& streak, int& lastDay) {
    streak = prefs.getInt("t_streak", 0);
    lastDay = prefs.getInt("t_day", 0);
  }
  
  // Settings
  static void saveSettings(int brightness, bool sound) {
    prefs.putInt("brightness", brightness);
    prefs.putBool("sound", sound);
  }
  
  static void loadSettings(int& brightness, bool& sound) {
    brightness = prefs.getInt("brightness", 200);
    sound = prefs.getBool("sound", true);
  }
  
  // High scores
  static void saveHighScore(const char* game, int score) {
    prefs.putInt(game, score);
  }
  
  static int loadHighScore(const char* game) {
    return prefs.getInt(game, 0);
  }
  
  // Clear all data
  static void clearAll() {
    prefs.clear();
    Serial.println("[NVS] All data cleared");
  }
};

// Static member
SDPreferences DataPersistence::prefs;

// =============================================================================
// ERROR HANDLING
// =============================================================================

// Safe string copy (prevents buffer overflow)
inline void safeStrCopy(char* dest, const char* src, size_t destSize) {
  if (dest && src && destSize > 0) {
    strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0';
  }
}

// Null-safe string length
inline size_t safeStrLen(const char* str) {
  return str ? strlen(str) : 0;
}

// Safe array bounds check
template<typename T, size_t N>
inline T& safeArrayAccess(T (&arr)[N], size_t index, T& defaultVal) {
  return (index < N) ? arr[index] : defaultVal;
}

// =============================================================================
// BATTERY OPTIMIZATION
// =============================================================================

// CPU frequency scaling
inline void setCPUFrequencyLow() {
  setCpuFrequencyMhz(80);  // Low power mode
}

inline void setCPUFrequencyNormal() {
  setCpuFrequencyMhz(160);  // Normal operation
}

inline void setCPUFrequencyHigh() {
  setCpuFrequencyMhz(240);  // Maximum performance (games)
}

// Screen timeout helper
class ScreenTimeout {
public:
  ScreenTimeout(unsigned long timeout = 30000) : _timeout(timeout), _lastActivity(0) {}
  
  void resetTimer() { _lastActivity = millis(); }
  
  bool isTimedOut() {
    return (millis() - _lastActivity) > _timeout;
  }
  
  void setTimeout(unsigned long ms) { _timeout = ms; }
  
  unsigned long getIdleTime() { return millis() - _lastActivity; }
  
private:
  unsigned long _timeout;
  unsigned long _lastActivity;
};

// =============================================================================
// DEBUG HELPERS
// =============================================================================

#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

// Performance timing
class PerfTimer {
public:
  void start() { _startTime = micros(); }
  
  unsigned long stop() {
    _duration = micros() - _startTime;
    return _duration;
  }
  
  void printResult(const char* label) {
    Serial.printf("[PERF] %s: %lu us (%.2f ms)\n", label, _duration, _duration / 1000.0f);
  }
  
private:
  unsigned long _startTime;
  unsigned long _duration;
};

// =============================================================================
// TOUCH DEBOUNCING
// =============================================================================

class TouchDebouncer {
public:
  TouchDebouncer(unsigned long debounceTime = 50) : _debounceTime(debounceTime), _lastTouch(0) {}
  
  bool isValidTouch() {
    unsigned long now = millis();
    if (now - _lastTouch > _debounceTime) {
      _lastTouch = now;
      return true;
    }
    return false;
  }
  
  void setDebounceTime(unsigned long ms) { _debounceTime = ms; }
  
private:
  unsigned long _debounceTime;
  unsigned long _lastTouch;
};

// =============================================================================
// INITIALIZATION HELPER
// =============================================================================

inline void initOptimizations() {
  Serial.println("[OPT] Initializing optimizations...");
  
  // Initialize watchdog
  initWatchdog();
  
  // Initialize persistent storage
  DataPersistence::begin();
  
  // Print initial memory status
  printMemoryStatus();
  
  // Set normal CPU frequency
  setCPUFrequencyNormal();
  
  Serial.println("[OPT] Optimizations ready");
}

#endif // OPTIMIZATIONS_H
