/*
 * dual_boot_manager.h - FIXED Dual Boot Manager for ESP32-S3 Smartwatch
 * 
 * FIXES IN THIS VERSION:
 * - Proper GPIO 0 initialization for ESP32-S3 (needs explicit pullup)
 * - Added visual feedback on screen for button presses
 * - Added Power button (GPIO 10) as alternative trigger (triple-tap)
 * - Extensive debug logging to diagnose issues
 * - Handles ESP32-S3 strapping pin behavior
 *
 * TRIGGER OPTIONS:
 * 1. Double-tap BOOT button (GPIO 0) - 2 taps within 2 seconds
 * 2. Triple-tap Power button (GPIO 10) - 3 taps within 3 seconds (fallback)
 * 3. Long-press Power button (3 seconds) - alternative trigger
 *
 * Compatible with:
 * - ESP32_Watch_206 (Fusion OS)
 * - S3_MiniOS_206 (MiniOS)
 */

#ifndef DUAL_BOOT_MANAGER_H
#define DUAL_BOOT_MANAGER_H

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <Preferences.h>
#include <FS.h>
#include <SD_MMC.h>
#include <FFat.h>

// =============================================================================
// CONFIGURATION - ESP32-S3 SPECIFIC
// =============================================================================

// Boot button GPIO (ESP32-S3 BOOT/STRAPPING pin)
// NOTE: On ESP32-S3, GPIO 0 is a strapping pin - needs special handling
#define BOOT_BUTTON_GPIO 0

// Power button GPIO (alternative trigger via triple-tap or long-press)
#define POWER_BUTTON_GPIO 10

// Double-tap timing (milliseconds)
#define DOUBLE_TAP_WINDOW_MS     2000   // 2 seconds between taps (boot-time)
#define RUNTIME_TAP_WINDOW_MS    2000   // 2 seconds between taps (runtime)
#define TAP_DEBOUNCE_MS          50     // Button debounce time
#define BOOT_HOLD_TIME_MS        100    // Minimum hold time for valid tap
#define LONG_PRESS_MS            3000   // 3 second long press to switch

// Triple-tap for power button (alternative method)
#define TRIPLE_TAP_WINDOW_MS     3000   // 3 seconds for 3 taps
#define TRIPLE_TAP_COUNT         3      // Number of taps needed

// Boot configuration file on SD card
#define BOOT_CONFIG_PATH         "/WATCH/DUALOS/boot.txt"
#define BOOT_CONFIG_PATH_FAT     "/boot.txt"

// Dual OS folder
#define DUALOS_FOLDER            "/WATCH/DUALOS"

// OS identifiers
#define OS_FUSION_WATCH          0      // ESP32_Watch_206 (Fusion OS)
#define OS_MINI_OS               1      // S3_MiniOS_206

// Partition names
#define PARTITION_OS1            "app0"  // ota_0 - Fusion OS
#define PARTITION_OS2            "app1"  // ota_1 - MiniOS

// SD_MMC pins (Waveshare ESP32-S3-Touch-AMOLED-2.06")
#define DUALBOOT_SDMMC_CLK       2
#define DUALBOOT_SDMMC_CMD       1
#define DUALBOOT_SDMMC_DATA      3

// Visual feedback (set to true to show on-screen indicators)
#define VISUAL_FEEDBACK_ENABLED  true

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct BootConfig {
    uint8_t current_os;
    uint8_t next_os;
    uint8_t boot_count;
    bool pending_switch;
    uint32_t last_switch_time;
    char os_names[2][32];
};

struct DoubleTapState {
    bool first_tap_detected;
    unsigned long first_tap_time;
    bool button_was_pressed;
    unsigned long button_press_time;
    bool switch_triggered;
};

struct TripleTapState {
    int tap_count;
    unsigned long first_tap_time;
    unsigned long last_tap_time;
    bool button_was_pressed;
    unsigned long button_press_time;
};

struct LongPressState {
    bool button_held;
    unsigned long press_start_time;
    bool triggered;
};

// =============================================================================
// CLASS DEFINITION
// =============================================================================

class DualBootManager {
public:
    DualBootManager();
    
    // Initialization
    bool begin();
    bool beginSD();
    
    // Boot-time double-tap detection (blocking, call in setup)
    bool checkDoubleTapBoot();
    
    // RUNTIME detection methods (non-blocking, call from loop)
    void checkRuntimeDoubleTap();      // BOOT button double-tap
    void checkPowerButtonTrigger();    // Power button triple-tap or long-press
    void checkAllTriggers();           // Check all methods at once
    
    // Boot configuration
    bool loadBootConfig();
    bool saveBootConfig();
    
    // OS switching
    bool switchToOS(uint8_t os_id);
    bool switchToOtherOS();
    void triggerReboot();
    
    // Status
    uint8_t getCurrentOS();
    uint8_t getOtherOS();
    const char* getCurrentOSName();
    const char* getOtherOSName();
    bool isPendingSwitch();
    bool isInitialized() { return initialized; }
    
    // Debug
    void printStatus();
    void testButtonRead();  // Test if button can be read
    
    // Visual feedback callback (set by main code)
    void setVisualFeedbackCallback(void (*callback)(const char* message, int duration_ms));
    
private:
    BootConfig config;
    DoubleTapState tapState;      // Boot-time tap state
    DoubleTapState rtState;       // Runtime BOOT button tap state
    TripleTapState pwrTapState;   // Power button triple-tap state
    LongPressState pwrLongPress;  // Power button long-press state
    
    bool sd_available;
    bool fat_available;
    bool initialized;
    bool gpio0_working;           // Track if GPIO 0 reads work
    
    // Visual feedback
    void (*visualFeedback)(const char* message, int duration_ms);
    void showFeedback(const char* message, int duration_ms = 500);
    
    // Internal helpers
    bool readBootTxt();
    bool writeBootTxt();
    void detectCurrentPartition();
    bool setBootPartition(const char* partition_name);
    void resetTapState();
    bool detectTap();
    void initGPIO();              // Proper GPIO initialization
};

// Global instance
extern DualBootManager DualBoot;

#endif // DUAL_BOOT_MANAGER_H