/*
 * dual_boot_manager.cpp - Dual Boot Manager Implementation
 * 
 * Implements double-tap boot button detection and OS switching
 * for ESP32-S3 smartwatch dual-boot system.
 * 
 * MODIFIED: Uses SD_MMC instead of SD (SPI) to match both OS codebases
 * UPDATED:  Added runtime double-tap detection (checkRuntimeDoubleTap)
 *           so user can switch OS at any time, not just during boot.
 */

#include "dual_boot_manager.h"

// Global instance
DualBootManager DualBoot;

// =============================================================================
// CONSTRUCTOR
// =============================================================================

DualBootManager::DualBootManager() {
    memset(&config, 0, sizeof(config));
    memset(&tapState, 0, sizeof(tapState));
    memset(&rtState, 0, sizeof(rtState));
    
    // Default OS names
    strcpy(config.os_names[OS_FUSION_WATCH], "Fusion Watch OS");
    strcpy(config.os_names[OS_MINI_OS], "MiniOS");
    
    sd_available = false;
    fat_available = false;
    initialized = false;
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool DualBootManager::begin() {
    Serial.println("[DUAL-BOOT] Initializing dual boot manager...");
    
    // Setup boot button
    pinMode(BOOT_BUTTON_GPIO, INPUT_PULLUP);
    
    // Detect which partition we're running from
    detectCurrentPartition();
    
    // Try to mount internal FAT partition first
    if (FFat.begin(true, "/fat")) {
        fat_available = true;
        Serial.println("[DUAL-BOOT] Internal FAT mounted at /fat");
    }
    
    // Load boot configuration
    if (!loadBootConfig()) {
        Serial.println("[DUAL-BOOT] No boot.txt found - creating default config");
        
        // config.current_os was already set by detectCurrentPartition()
        config.next_os = config.current_os;
        config.boot_count = 1;
        config.pending_switch = false;
        
        // Auto-create the boot.txt file
        if (saveBootConfig()) {
            Serial.println("[DUAL-BOOT] boot.txt auto-created");
        } else {
            Serial.println("[DUAL-BOOT] WARNING: Could not auto-create boot.txt (SD may not be mounted yet)");
        }
    }
    
    // Update boot.txt with actual running partition (in case it's stale)
    config.current_os = config.current_os;  // Already set by detectCurrentPartition
    config.pending_switch = false;           // We booted successfully
    config.boot_count++;
    saveBootConfig();  // May fail if SD not mounted yet, that's OK
    
    initialized = true;
    printStatus();
    
    // Reset runtime tap state
    memset(&rtState, 0, sizeof(rtState));
    
    return true;
}

bool DualBootManager::beginSD() {
    Serial.println("[DUAL-BOOT] Initializing SD card via SD_MMC...");
    
    // Set SD_MMC pins for Waveshare ESP32-S3-Touch-AMOLED-2.06"
    SD_MMC.setPins(DUALBOOT_SDMMC_CLK, DUALBOOT_SDMMC_CMD, DUALBOOT_SDMMC_DATA);
    
    // Initialize SD_MMC in 1-bit mode for compatibility
    if (SD_MMC.begin("/sdcard", true)) {
        sd_available = true;
        Serial.println("[DUAL-BOOT] SD card mounted successfully via SD_MMC");
        
        // Create /WATCH/DUALOS folder if it doesn't exist
        if (!SD_MMC.exists("/WATCH")) {
            SD_MMC.mkdir("/WATCH");
        }
        if (!SD_MMC.exists("/WATCH/DUALOS")) {
            SD_MMC.mkdir("/WATCH/DUALOS");
            Serial.println("[DUAL-BOOT] Created /WATCH/DUALOS folder on SD");
        }
        
        // Now that SD is available, save boot config (may have failed in begin())
        saveBootConfig();
        
        return true;
    }
    
    Serial.println("[DUAL-BOOT] SD card mount failed");
    return false;
}

// =============================================================================
// BOOT-TIME DOUBLE-TAP DETECTION (blocking, runs in setup())
// =============================================================================

bool DualBootManager::checkDoubleTapBoot() {
    Serial.println("[DUAL-BOOT] Checking for double-tap boot switch...");
    Serial.println("[DUAL-BOOT] Double-tap BOOT button within 500ms to switch OS");
    
    resetTapState();
    
    unsigned long start_time = millis();
    unsigned long window_end = start_time + DOUBLE_TAP_WINDOW_MS;
    
    while (millis() < window_end) {
        if (detectTap()) {
            Serial.println("[DUAL-BOOT] Tap detected!");
            
            if (!tapState.first_tap_detected) {
                tapState.first_tap_detected = true;
                tapState.first_tap_time = millis();
                Serial.println("[DUAL-BOOT] First tap - waiting for second...");
            } else {
                unsigned long tap_interval = millis() - tapState.first_tap_time;
                
                if (tap_interval < DOUBLE_TAP_WINDOW_MS) {
                    Serial.println("[DUAL-BOOT] *** DOUBLE-TAP DETECTED! ***");
                    tapState.switch_triggered = true;
                    
                    if (switchToOtherOS()) {
                        Serial.println("[DUAL-BOOT] OS switch scheduled - rebooting...");
                        delay(100);
                        triggerReboot();
                        return true;
                    } else {
                        Serial.println("[DUAL-BOOT] OS switch failed!");
                        return false;
                    }
                } else {
                    Serial.println("[DUAL-BOOT] Taps too far apart, resetting");
                    tapState.first_tap_detected = true;
                    tapState.first_tap_time = millis();
                }
            }
        }
        
        delay(10);
    }
    
    Serial.println("[DUAL-BOOT] No double-tap detected, continuing normal boot");
    return false;
}

bool DualBootManager::detectTap() {
    bool pressed = (digitalRead(BOOT_BUTTON_GPIO) == LOW);
    
    if (pressed && !tapState.button_was_pressed) {
        tapState.button_was_pressed = true;
        tapState.button_press_time = millis();
        return false;
    }
    else if (!pressed && tapState.button_was_pressed) {
        unsigned long hold_time = millis() - tapState.button_press_time;
        tapState.button_was_pressed = false;
        
        if (hold_time >= TAP_DEBOUNCE_MS && hold_time < 1000) {
            return true;
        }
    }
    
    return false;
}

void DualBootManager::resetTapState() {
    tapState.first_tap_detected = false;
    tapState.first_tap_time = 0;
    tapState.button_was_pressed = false;
    tapState.button_press_time = 0;
    tapState.switch_triggered = false;
}

// =============================================================================
// RUNTIME DOUBLE-TAP DETECTION (non-blocking, call from loop())
// =============================================================================

void DualBootManager::checkRuntimeDoubleTap() {
    if (!initialized) return;
    
    bool pressed = (digitalRead(BOOT_BUTTON_GPIO) == LOW);
    unsigned long now = millis();
    
    // --- Detect a complete tap (press then release) ---
    if (pressed && !rtState.button_was_pressed) {
        // Button just pressed
        rtState.button_was_pressed = true;
        rtState.button_press_time = now;
    }
    else if (!pressed && rtState.button_was_pressed) {
        // Button just released — check if it was a valid short tap
        unsigned long hold_time = now - rtState.button_press_time;
        rtState.button_was_pressed = false;
        
        if (hold_time >= TAP_DEBOUNCE_MS && hold_time < 1000) {
            // Valid tap!
            if (!rtState.first_tap_detected) {
                // --- FIRST TAP ---
                rtState.first_tap_detected = true;
                rtState.first_tap_time = now;
                Serial.println("[DUAL-BOOT] Runtime: First tap detected, waiting for second...");
            } else {
                // --- SECOND TAP — check if within window ---
                unsigned long tap_interval = now - rtState.first_tap_time;
                
                if (tap_interval <= RUNTIME_TAP_WINDOW_MS) {
                    // *** DOUBLE-TAP DETECTED — SWITCH OS ***
                    Serial.println("[DUAL-BOOT] *** RUNTIME DOUBLE-TAP — SWITCHING OS! ***");
                    
                    if (switchToOtherOS()) {
                        Serial.printf("[DUAL-BOOT] Switching to %s — rebooting now!\n",
                                      getOtherOSName());
                        delay(150);  // Brief pause for serial output
                        triggerReboot();
                        // Won't reach here
                    } else {
                        Serial.println("[DUAL-BOOT] OS switch failed! Continuing...");
                    }
                }
                
                // Reset regardless (double-tap succeeded or taps too far apart)
                rtState.first_tap_detected = false;
            }
        }
    }
    
    // --- Timeout: if first tap happened but second didn't come in time, reset ---
    if (rtState.first_tap_detected && (now - rtState.first_tap_time > RUNTIME_TAP_WINDOW_MS)) {
        rtState.first_tap_detected = false;
    }
}

// =============================================================================
// BOOT CONFIGURATION
// =============================================================================

bool DualBootManager::loadBootConfig() {
    return readBootTxt();
}

bool DualBootManager::saveBootConfig() {
    return writeBootTxt();
}

bool DualBootManager::readBootTxt() {
    File file;
    
    if (sd_available && SD_MMC.exists(BOOT_CONFIG_PATH)) {
        file = SD_MMC.open(BOOT_CONFIG_PATH, FILE_READ);
        Serial.println("[DUAL-BOOT] Reading boot.txt from /WATCH/DUALOS/boot.txt");
    } 
    else if (fat_available && FFat.exists(BOOT_CONFIG_PATH_FAT)) {
        file = FFat.open(BOOT_CONFIG_PATH_FAT, FILE_READ);
        Serial.println("[DUAL-BOOT] Reading boot.txt from internal FAT");
    }
    else {
        return false;
    }
    
    if (!file) {
        return false;
    }
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        if (line.startsWith("CURRENT_OS=")) {
            config.current_os = line.substring(11).toInt();
        }
        else if (line.startsWith("NEXT_OS=")) {
            config.next_os = line.substring(8).toInt();
        }
        else if (line.startsWith("PENDING_SWITCH=")) {
            config.pending_switch = (line.substring(15).toInt() == 1);
        }
        else if (line.startsWith("BOOT_COUNT=")) {
            config.boot_count = line.substring(11).toInt();
        }
        else if (line.startsWith("OS0_NAME=")) {
            line.substring(9).toCharArray(config.os_names[0], 32);
        }
        else if (line.startsWith("OS1_NAME=")) {
            line.substring(9).toCharArray(config.os_names[1], 32);
        }
    }
    
    file.close();
    return true;
}

bool DualBootManager::writeBootTxt() {
    File file;
    
    if (sd_available) {
        if (!SD_MMC.exists("/WATCH/DUALOS")) {
            SD_MMC.mkdir("/WATCH");
            SD_MMC.mkdir("/WATCH/DUALOS");
        }
        file = SD_MMC.open(BOOT_CONFIG_PATH, FILE_WRITE);
    }
    else if (fat_available) {
        file = FFat.open(BOOT_CONFIG_PATH_FAT, FILE_WRITE);
    }
    else {
        return false;
    }
    
    if (!file) {
        return false;
    }
    
    file.printf("# ESP32 Dual Boot Configuration\n");
    file.printf("# Auto-generated by DualBootManager\n");
    file.printf("# Double-tap BOOT button (GPIO 0) to switch OS\n\n");
    file.printf("CURRENT_OS=%d\n", config.current_os);
    file.printf("NEXT_OS=%d\n", config.next_os);
    file.printf("PENDING_SWITCH=%d\n", config.pending_switch ? 1 : 0);
    file.printf("BOOT_COUNT=%d\n", config.boot_count);
    file.printf("OS0_NAME=%s\n", config.os_names[0]);
    file.printf("OS1_NAME=%s\n", config.os_names[1]);
    file.printf("LAST_SWITCH=%lu\n", config.last_switch_time);
    
    file.close();
    return true;
}

// =============================================================================
// OS SWITCHING
// =============================================================================

bool DualBootManager::switchToOS(uint8_t os_id) {
    if (os_id > 1) {
        Serial.printf("[DUAL-BOOT] Invalid OS ID: %d\n", os_id);
        return false;
    }
    
    const char* partition_name = (os_id == OS_FUSION_WATCH) ? PARTITION_OS1 : PARTITION_OS2;
    
    Serial.printf("[DUAL-BOOT] Switching to OS %d (%s) on partition %s\n",
                  os_id, config.os_names[os_id], partition_name);
    
    // Update boot config BEFORE setting partition
    config.next_os = os_id;
    config.pending_switch = true;
    config.last_switch_time = millis();
    
    // Write boot.txt so the other OS knows what happened
    if (!saveBootConfig()) {
        Serial.println("[DUAL-BOOT] WARNING: Could not save boot.txt (switch will still work via otadata)");
    }
    
    // Set the OTA boot partition — this is what actually controls which OS loads
    return setBootPartition(partition_name);
}

bool DualBootManager::switchToOtherOS() {
    uint8_t other = (config.current_os == OS_FUSION_WATCH) ? OS_MINI_OS : OS_FUSION_WATCH;
    return switchToOS(other);
}

bool DualBootManager::setBootPartition(const char* partition_name) {
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_ANY,
        partition_name
    );
    
    if (partition == NULL) {
        Serial.printf("[DUAL-BOOT] ERROR: Partition '%s' not found!\n", partition_name);
        Serial.println("[DUAL-BOOT] Make sure dual_os_partitions.csv is flashed!");
        return false;
    }
    
    esp_err_t err = esp_ota_set_boot_partition(partition);
    
    if (err != ESP_OK) {
        Serial.printf("[DUAL-BOOT] ERROR: esp_ota_set_boot_partition failed: %d\n", err);
        return false;
    }
    
    Serial.printf("[DUAL-BOOT] Boot partition set to: %s (0x%08x, size: 0x%08x)\n",
                  partition_name, partition->address, partition->size);
    return true;
}

void DualBootManager::triggerReboot() {
    Serial.println("[DUAL-BOOT] Rebooting in 100ms...");
    Serial.flush();
    delay(100);
    esp_restart();
}

void DualBootManager::detectCurrentPartition() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    
    if (running == NULL) {
        Serial.println("[DUAL-BOOT] WARNING: Could not detect running partition");
        config.current_os = OS_FUSION_WATCH;  // Default
        return;
    }
    
    Serial.printf("[DUAL-BOOT] Running from partition: %s @ 0x%08x (size: 0x%08x)\n",
                  running->label, running->address, running->size);
    
    // Determine which OS based on partition label
    if (strcmp(running->label, PARTITION_OS1) == 0 || 
        strcmp(running->label, "app0") == 0 ||
        strcmp(running->label, "factory") == 0) {
        config.current_os = OS_FUSION_WATCH;
    } else {
        config.current_os = OS_MINI_OS;
    }
}

// =============================================================================
// STATUS GETTERS
// =============================================================================

uint8_t DualBootManager::getCurrentOS() {
    return config.current_os;
}

uint8_t DualBootManager::getOtherOS() {
    return (config.current_os == OS_FUSION_WATCH) ? OS_MINI_OS : OS_FUSION_WATCH;
}

const char* DualBootManager::getCurrentOSName() {
    return config.os_names[config.current_os];
}

const char* DualBootManager::getOtherOSName() {
    return config.os_names[getOtherOS()];
}

bool DualBootManager::isPendingSwitch() {
    return config.pending_switch;
}

// =============================================================================
// DEBUG
// =============================================================================

void DualBootManager::printStatus() {
    Serial.println("\n===== DUAL BOOT STATUS =====");
    Serial.printf("Current OS:     %d (%s)\n", config.current_os, config.os_names[config.current_os]);
    Serial.printf("Other OS:       %d (%s)\n", getOtherOS(), config.os_names[getOtherOS()]);
    Serial.printf("Boot Count:     %d\n", config.boot_count);
    Serial.printf("Pending Switch: %s\n", config.pending_switch ? "YES" : "NO");
    Serial.printf("SD Card:        %s\n", sd_available ? "Available" : "Not mounted");
    Serial.printf("Internal FAT:   %s\n", fat_available ? "Available" : "Not mounted");
    Serial.printf("BOOT button:    GPIO %d (double-tap to switch)\n", BOOT_BUTTON_GPIO);
    Serial.println("============================\n");
}
