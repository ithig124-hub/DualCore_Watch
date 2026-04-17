/*
 * dual_boot_manager.h - Dual Boot Manager for ESP32-S3 Smartwatch
 * 
 * Handles:
 * - Double-tap BOOT button detection for OS switching
 * - Runtime double-tap detection (call from loop())
 * - Reading/writing boot.txt to track which OS to load
 * - Managing the OTA partitions for dual-boot
 * 
 * Compatible with:
 * - ESP32_Watch_206 (Fusion OS)
 * - S3_MiniOS_206 (MiniOS)
 * 
 * MODIFIED: Uses SD_MMC instead of SD (SPI) to match both OS codebases
 * UPDATED:  Added runtime double-tap detection for switching while OS is running
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
// CONFIGURATION
// =============================================================================

// Boot button GPIO (ESP32-S3 standard BOOT button)
#define BOOT_BUTTON_GPIO       0

// Double-tap timing (milliseconds)
#define DOUBLE_TAP_WINDOW_MS   2000   // 2 seconds between taps (boot-time)
#define RUNTIME_TAP_WINDOW_MS  2000   // 2 seconds between taps (runtime)
#define TAP_DEBOUNCE_MS        50     // Button debounce time
#define BOOT_HOLD_TIME_MS      100    // Minimum hold time for valid tap

// Boot configuration file on SD card (inside existing WATCH folder)
#define BOOT_CONFIG_PATH       "/WATCH/DUALOS/boot.txt"
#define BOOT_CONFIG_PATH_FAT   "/boot.txt"  // If using internal FAT

// Dual OS folder (inside WATCH)
#define DUALOS_FOLDER          "/WATCH/DUALOS"

// OS identifiers
#define OS_FUSION_WATCH        0      // ESP32_Watch_206 (Fusion OS)
#define OS_MINI_OS             1      // S3_MiniOS_206

// Partition names matching dual_os_partitions.csv
#define PARTITION_OS1          "app0"  // ota_0 - Fusion OS
#define PARTITION_OS2          "app1"  // ota_1 - MiniOS

// SD_MMC pins (Waveshare ESP32-S3-Touch-AMOLED-2.06")
#define DUALBOOT_SDMMC_CLK    2
#define DUALBOOT_SDMMC_CMD    1
#define DUALBOOT_SDMMC_DATA   3

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct BootConfig {
    uint8_t current_os;           // Which OS is currently running
    uint8_t next_os;              // Which OS to boot next (after double-tap)
    uint8_t boot_count;           // Boot counter for debugging
    bool pending_switch;          // True if OS switch is pending
    uint32_t last_switch_time;    // Timestamp of last OS switch
    char os_names[2][32];         // Human-readable OS names
};

struct DoubleTapState {
    bool first_tap_detected;
    unsigned long first_tap_time;
    bool button_was_pressed;
    unsigned long button_press_time;
    bool switch_triggered;
};

// =============================================================================
// CLASS DEFINITION
// =============================================================================

class DualBootManager {
public:
    DualBootManager();
    
    // Initialization
    bool begin();
    bool beginSD();  // Uses SD_MMC with hardcoded pins for 2.06" board
    
    // Double-tap detection (call in setup() before other init)
    bool checkDoubleTapBoot();
    
    // RUNTIME double-tap detection (call from loop() every iteration)
    // Non-blocking: tracks button state across calls, triggers switch on double-tap
    void checkRuntimeDoubleTap();
    
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
    
    // Debug
    void printStatus();
    
private:
    BootConfig config;
    DoubleTapState tapState;
    DoubleTapState rtState;    // Runtime tap state (separate from boot-time)
    bool sd_available;
    bool fat_available;
    bool initialized;
    
    // Internal helpers
    bool readBootTxt();
    bool writeBootTxt();
    void detectCurrentPartition();
    bool setBootPartition(const char* partition_name);
    void resetTapState();
    bool detectTap();
};

// Global instance
extern DualBootManager DualBoot;

#endif // DUAL_BOOT_MANAGER_H
