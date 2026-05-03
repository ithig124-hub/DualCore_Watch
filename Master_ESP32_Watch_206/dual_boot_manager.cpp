/*
 * dual_boot_manager.cpp - FIXED Dual Boot Manager Implementation
 *
 * FIXES IN THIS VERSION:
 * - Proper GPIO 0 initialization with explicit INPUT_PULLUP
 * - GPIO test function to verify button reads
 * - Visual feedback for debugging without Serial
 * - Power button triple-tap as fallback trigger
 * - Power button long-press (3 sec) as alternative trigger
 * - Extensive debug logging at every step
 * - Handles ESP32-S3 strapping pin quirks
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
    memset(&pwrTapState, 0, sizeof(pwrTapState));
    memset(&pwrLongPress, 0, sizeof(pwrLongPress));
    
    strcpy(config.os_names[OS_FUSION_WATCH], "Fusion Watch OS");
    strcpy(config.os_names[OS_MINI_OS], "MiniOS");
    
    sd_available = false;
    fat_available = false;
    initialized = false;
    gpio0_working = false;
    visualFeedback = nullptr;
}

// =============================================================================
// GPIO INITIALIZATION - CRITICAL FOR ESP32-S3
// =============================================================================

void DualBootManager::initGPIO() {
    Serial.println("[DUAL-BOOT] ====== GPIO INITIALIZATION ======");
    
    // =========================================================================
    // GPIO 0 (BOOT button) - ESP32-S3 Strapping Pin
    // On ESP32-S3, GPIO 0 is used for boot mode selection:
    // - LOW during reset = Download mode
    // - HIGH during reset = Normal boot
    // After boot, it can be used as a normal GPIO but needs explicit config
    // =========================================================================
    
    Serial.println("[DUAL-BOOT] Configuring GPIO 0 (BOOT button)...");
    
    // Disable any existing interrupt on GPIO 0
    detachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_GPIO));
    
    // Configure as input with internal pullup
    pinMode(BOOT_BUTTON_GPIO, INPUT_PULLUP);
    
    // Small delay for GPIO to stabilize
    delay(10);
    
    // Test if we can read the GPIO
    int testRead1 = digitalRead(BOOT_BUTTON_GPIO);
    delay(5);
    int testRead2 = digitalRead(BOOT_BUTTON_GPIO);
    delay(5);
    int testRead3 = digitalRead(BOOT_BUTTON_GPIO);
    
    Serial.printf("[DUAL-BOOT] GPIO 0 test reads: %d, %d, %d\n", testRead1, testRead2, testRead3);
    
    // Check if reads are consistent (should be HIGH when not pressed)
    if (testRead1 == HIGH && testRead2 == HIGH && testRead3 == HIGH) {
        gpio0_working = true;
        Serial.println("[DUAL-BOOT] GPIO 0 OK - reads HIGH (not pressed)");
    } else if (testRead1 == LOW && testRead2 == LOW && testRead3 == LOW) {
        // Button might be stuck or pressed during init
        Serial.println("[DUAL-BOOT] GPIO 0 reads LOW - button pressed or stuck?");
        gpio0_working = true;  // Still might work
    } else {
        Serial.println("[DUAL-BOOT] WARNING: GPIO 0 reads inconsistent!");
        gpio0_working = false;
    }
    
    // =========================================================================
    // GPIO 10 (Power button) - Fallback trigger
    // =========================================================================
    
    Serial.println("[DUAL-BOOT] Configuring GPIO 10 (Power button)...");
    pinMode(POWER_BUTTON_GPIO, INPUT_PULLUP);
    delay(5);
    
    int pwrTest = digitalRead(POWER_BUTTON_GPIO);
    Serial.printf("[DUAL-BOOT] GPIO 10 test read: %d (should be HIGH)\n", pwrTest);
    
    Serial.println("[DUAL-BOOT] ====== GPIO INIT COMPLETE ======");
    Serial.printf("[DUAL-BOOT] BOOT button (GPIO 0): %s\n", gpio0_working ? "WORKING" : "FAILED");
    Serial.println("[DUAL-BOOT] Power button (GPIO 10): Available as fallback");
    Serial.println("");
}

// =============================================================================
// TEST BUTTON READ - Call this to verify button works
// =============================================================================

void DualBootManager::testButtonRead() {
    Serial.println("\n[DUAL-BOOT] ====== BUTTON TEST ======");
    Serial.println("[DUAL-BOOT] Reading buttons for 5 seconds...");
    Serial.println("[DUAL-BOOT] Press BOOT (GPIO 0) or Power (GPIO 10) button");
    
    unsigned long startTime = millis();
    int lastBoot = -1;
    int lastPwr = -1;
    
    while (millis() - startTime < 5000) {
        int bootRead = digitalRead(BOOT_BUTTON_GPIO);
        int pwrRead = digitalRead(POWER_BUTTON_GPIO);
        
        if (bootRead != lastBoot || pwrRead != lastPwr) {
            Serial.printf("[DUAL-BOOT] BOOT=%d PWR=%d (LOW=pressed)\n", bootRead, pwrRead);
            lastBoot = bootRead;
            lastPwr = pwrRead;
            
            if (bootRead == LOW) {
                showFeedback("BOOT PRESSED!", 200);
            }
            if (pwrRead == LOW) {
                showFeedback("PWR PRESSED!", 200);
            }
        }
        delay(50);
    }
    
    Serial.println("[DUAL-BOOT] ====== TEST COMPLETE ======\n");
}

// =============================================================================
// VISUAL FEEDBACK
// =============================================================================

void DualBootManager::setVisualFeedbackCallback(void (*callback)(const char* message, int duration_ms)) {
    visualFeedback = callback;
}

void DualBootManager::showFeedback(const char* message, int duration_ms) {
    Serial.printf("[DUAL-BOOT] FEEDBACK: %s\n", message);
    if (visualFeedback != nullptr) {
        visualFeedback(message, duration_ms);
    }
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool DualBootManager::begin() {
    Serial.println("\n[DUAL-BOOT] ========================================");
    Serial.println("[DUAL-BOOT] INITIALIZING DUAL BOOT MANAGER v2.0");
    Serial.println("[DUAL-BOOT] ========================================\n");
    
    // Initialize GPIOs properly
    initGPIO();
    
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
        config.next_os = config.current_os;
        config.boot_count = 1;
        config.pending_switch = false;
        
        if (saveBootConfig()) {
            Serial.println("[DUAL-BOOT] boot.txt auto-created");
        }
    }
    
    config.pending_switch = false;
    config.boot_count++;
    saveBootConfig();
    
    initialized = true;
    
    // Reset all tap states
    memset(&rtState, 0, sizeof(rtState));
    memset(&pwrTapState, 0, sizeof(pwrTapState));
    memset(&pwrLongPress, 0, sizeof(pwrLongPress));
    
    printStatus();
    
    Serial.println("\n[DUAL-BOOT] ========================================");
    Serial.println("[DUAL-BOOT] HOW TO SWITCH OS:");
    Serial.println("[DUAL-BOOT]   Option 1: Double-tap BOOT button (GPIO 0)");
    Serial.println("[DUAL-BOOT]   Option 2: Triple-tap Power button (GPIO 10)");
    Serial.println("[DUAL-BOOT]   Option 3: Long-press Power button (3 sec)");
    Serial.println("[DUAL-BOOT] ========================================\n");
    
    return true;
}

bool DualBootManager::beginSD() {
    Serial.println("[DUAL-BOOT] Initializing SD card via SD_MMC...");
    
    SD_MMC.setPins(DUALBOOT_SDMMC_CLK, DUALBOOT_SDMMC_CMD, DUALBOOT_SDMMC_DATA);
    
    if (SD_MMC.begin("/sdcard", true)) {
        sd_available = true;
        Serial.println("[DUAL-BOOT] SD card mounted successfully via SD_MMC");
        
        if (!SD_MMC.exists("/WATCH")) {
            SD_MMC.mkdir("/WATCH");
        }
        if (!SD_MMC.exists("/WATCH/DUALOS")) {
            SD_MMC.mkdir("/WATCH/DUALOS");
            Serial.println("[DUAL-BOOT] Created /WATCH/DUALOS folder on SD");
        }
        
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
    Serial.println("\n[DUAL-BOOT] ====== BOOT-TIME DOUBLE-TAP CHECK ======");
    Serial.println("[DUAL-BOOT] Double-tap BOOT button within 2 seconds to switch OS");
    Serial.println("[DUAL-BOOT] Waiting for taps...\n");
    
    showFeedback("TAP BOOT BTN x2", 1000);
    
    resetTapState();
    
    unsigned long start_time = millis();
    unsigned long window_end = start_time + DOUBLE_TAP_WINDOW_MS;
    
    int loopCount = 0;
    
    while (millis() < window_end) {
        loopCount++;
        
        // Read button state
        bool pressed = (digitalRead(BOOT_BUTTON_GPIO) == LOW);
        
        // Debug every 500ms
        if (loopCount % 50 == 0) {
            Serial.printf("[DUAL-BOOT] Polling... GPIO0=%d (LOW=pressed) time=%lums\n", 
                         !pressed, millis() - start_time);
        }
        
        if (detectTap()) {
            Serial.println("[DUAL-BOOT] >>> TAP DETECTED! <<<");
            showFeedback("TAP!", 200);
            
            if (!tapState.first_tap_detected) {
                tapState.first_tap_detected = true;
                tapState.first_tap_time = millis();
                Serial.println("[DUAL-BOOT] First tap registered - waiting for second...");
                showFeedback("1st TAP - again!", 500);
            } else {
                unsigned long tap_interval = millis() - tapState.first_tap_time;
                Serial.printf("[DUAL-BOOT] Second tap! Interval: %lu ms\n", tap_interval);
                
                if (tap_interval < DOUBLE_TAP_WINDOW_MS) {
                    Serial.println("[DUAL-BOOT] *** DOUBLE-TAP DETECTED! ***");
                    showFeedback("SWITCHING OS!", 1000);
                    tapState.switch_triggered = true;
                    
                    if (switchToOtherOS()) {
                        Serial.println("[DUAL-BOOT] OS switch scheduled - rebooting...");
                        delay(500);
                        triggerReboot();
                        return true;
                    } else {
                        Serial.println("[DUAL-BOOT] OS switch failed!");
                        showFeedback("SWITCH FAILED!", 1000);
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
    Serial.println("[DUAL-BOOT] ====== END BOOT-TIME CHECK ======\n");
    return false;
}

bool DualBootManager::detectTap() {
    bool pressed = (digitalRead(BOOT_BUTTON_GPIO) == LOW);
    
    // Button just pressed
    if (pressed && !tapState.button_was_pressed) {
        tapState.button_was_pressed = true;
        tapState.button_press_time = millis();
        return false;
    }
    
    // Button just released
    if (!pressed && tapState.button_was_pressed) {
        unsigned long hold_time = millis() - tapState.button_press_time;
        tapState.button_was_pressed = false;
        
        // Valid tap: held between 50ms and 1000ms
        if (hold_time >= TAP_DEBOUNCE_MS && hold_time < 1000) {
            Serial.printf("[DUAL-BOOT] Valid tap detected (held %lu ms)\n", hold_time);
            return true;
        } else {
            Serial.printf("[DUAL-BOOT] Invalid tap (held %lu ms)\n", hold_time);
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
    
    // Button just pressed
    if (pressed && !rtState.button_was_pressed) {
        rtState.button_was_pressed = true;
        rtState.button_press_time = now;
        Serial.println("[DUAL-BOOT] RT: BOOT button pressed");
    }
    // Button just released
    else if (!pressed && rtState.button_was_pressed) {
        unsigned long hold_time = now - rtState.button_press_time;
        rtState.button_was_pressed = false;
        
        // Valid tap
        if (hold_time >= TAP_DEBOUNCE_MS && hold_time < 1000) {
            Serial.printf("[DUAL-BOOT] RT: Valid tap (held %lu ms)\n", hold_time);
            showFeedback("TAP!", 150);
            
            if (!rtState.first_tap_detected) {
                // First tap
                rtState.first_tap_detected = true;
                rtState.first_tap_time = now;
                Serial.println("[DUAL-BOOT] RT: First tap - waiting for second...");
                showFeedback("1st TAP", 300);
            } else {
                // Second tap
                unsigned long tap_interval = now - rtState.first_tap_time;
                
                if (tap_interval <= RUNTIME_TAP_WINDOW_MS) {
                    // DOUBLE-TAP DETECTED!
                    Serial.println("[DUAL-BOOT] *** RUNTIME DOUBLE-TAP - SWITCHING OS! ***");
                    showFeedback("SWITCHING!", 500);
                    
                    if (switchToOtherOS()) {
                        Serial.printf("[DUAL-BOOT] Switching to %s - rebooting!\n", getOtherOSName());
                        delay(300);
                        triggerReboot();
                    } else {
                        Serial.println("[DUAL-BOOT] OS switch failed!");
                        showFeedback("FAILED!", 500);
                    }
                }
                // Reset regardless
                rtState.first_tap_detected = false;
            }
        }
    }
    
    // Timeout: reset if first tap happened but second didn't come
    if (rtState.first_tap_detected && (now - rtState.first_tap_time > RUNTIME_TAP_WINDOW_MS)) {
        rtState.first_tap_detected = false;
        Serial.println("[DUAL-BOOT] RT: Tap timeout, reset");
    }
}

// =============================================================================
// POWER BUTTON TRIGGER (Triple-tap or Long-press)
// =============================================================================

void DualBootManager::checkPowerButtonTrigger() {
    if (!initialized) return;
    
    bool pressed = (digitalRead(POWER_BUTTON_GPIO) == LOW);
    unsigned long now = millis();
    
    // =========================================================================
    // LONG PRESS DETECTION (3 seconds)
    // =========================================================================
    
    if (pressed) {
        if (!pwrLongPress.button_held) {
            pwrLongPress.button_held = true;
            pwrLongPress.press_start_time = now;
            pwrLongPress.triggered = false;
        } else if (!pwrLongPress.triggered) {
            unsigned long hold_time = now - pwrLongPress.press_start_time;
            
            // Show progress feedback
            if (hold_time >= 1000 && hold_time < 1100) {
                showFeedback("Hold 2 more...", 200);
            } else if (hold_time >= 2000 && hold_time < 2100) {
                showFeedback("Hold 1 more...", 200);
            }
            
            if (hold_time >= LONG_PRESS_MS) {
                Serial.println("[DUAL-BOOT] *** LONG PRESS - SWITCHING OS! ***");
                showFeedback("SWITCHING!", 500);
                pwrLongPress.triggered = true;
                
                if (switchToOtherOS()) {
                    Serial.printf("[DUAL-BOOT] Switching to %s\n", getOtherOSName());
                    delay(300);
                    triggerReboot();
                }
            }
        }
    } else {
        // Button released
        if (pwrLongPress.button_held && !pwrLongPress.triggered) {
            // Short press - count as tap for triple-tap
            unsigned long hold_time = now - pwrLongPress.press_start_time;
            
            if (hold_time >= TAP_DEBOUNCE_MS && hold_time < 1000) {
                // Valid tap for triple-tap detection
                if (pwrTapState.tap_count == 0) {
                    pwrTapState.first_tap_time = now;
                }
                
                if (now - pwrTapState.first_tap_time <= TRIPLE_TAP_WINDOW_MS) {
                    pwrTapState.tap_count++;
                    pwrTapState.last_tap_time = now;
                    
                    Serial.printf("[DUAL-BOOT] Power tap %d/3\n", pwrTapState.tap_count);
                    
                    char msg[20];
                    snprintf(msg, sizeof(msg), "PWR TAP %d/3", pwrTapState.tap_count);
                    showFeedback(msg, 200);
                    
                    if (pwrTapState.tap_count >= TRIPLE_TAP_COUNT) {
                        Serial.println("[DUAL-BOOT] *** TRIPLE TAP - SWITCHING OS! ***");
                        showFeedback("SWITCHING!", 500);
                        
                        if (switchToOtherOS()) {
                            Serial.printf("[DUAL-BOOT] Switching to %s\n", getOtherOSName());
                            delay(300);
                            triggerReboot();
                        }
                        
                        pwrTapState.tap_count = 0;
                    }
                } else {
                    // Window expired, start new count
                    pwrTapState.tap_count = 1;
                    pwrTapState.first_tap_time = now;
                }
            }
        }
        
        pwrLongPress.button_held = false;
    }
    
    // Reset triple-tap if window expired
    if (pwrTapState.tap_count > 0 && (now - pwrTapState.first_tap_time > TRIPLE_TAP_WINDOW_MS)) {
        pwrTapState.tap_count = 0;
    }
}

// =============================================================================
// CHECK ALL TRIGGERS AT ONCE (convenience method)
// =============================================================================

void DualBootManager::checkAllTriggers() {
    checkRuntimeDoubleTap();     // BOOT button double-tap
    checkPowerButtonTrigger();   // Power button triple-tap or long-press
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
        Serial.println("[DUAL-BOOT] Reading boot.txt from SD");
    } else if (fat_available && FFat.exists(BOOT_CONFIG_PATH_FAT)) {
        file = FFat.open(BOOT_CONFIG_PATH_FAT, FILE_READ);
        Serial.println("[DUAL-BOOT] Reading boot.txt from internal FAT");
    } else {
        return false;
    }
    
    if (!file) return false;
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        if (line.startsWith("CURRENT_OS=")) {
            config.current_os = line.substring(11).toInt();
        } else if (line.startsWith("NEXT_OS=")) {
            config.next_os = line.substring(8).toInt();
        } else if (line.startsWith("PENDING_SWITCH=")) {
            config.pending_switch = (line.substring(15).toInt() == 1);
        } else if (line.startsWith("BOOT_COUNT=")) {
            config.boot_count = line.substring(11).toInt();
        } else if (line.startsWith("OS0_NAME=")) {
            line.substring(9).toCharArray(config.os_names[0], 32);
        } else if (line.startsWith("OS1_NAME=")) {
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
    } else if (fat_available) {
        file = FFat.open(BOOT_CONFIG_PATH_FAT, FILE_WRITE);
    } else {
        return false;
    }
    
    if (!file) return false;
    
    file.printf("# ESP32 Dual Boot Configuration\n");
    file.printf("# Auto-generated by DualBootManager v2.0\n");
    file.printf("# Triggers: Double-tap BOOT, Triple-tap PWR, Long-press PWR\n\n");
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
    
    config.next_os = os_id;
    config.pending_switch = true;
    config.last_switch_time = millis();
    
    if (!saveBootConfig()) {
        Serial.println("[DUAL-BOOT] WARNING: Could not save boot.txt");
    }
    
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
        showFeedback("PARTITION ERR!", 2000);
        return false;
    }
    
    esp_err_t err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        Serial.printf("[DUAL-BOOT] ERROR: esp_ota_set_boot_partition failed: %d\n", err);
        showFeedback("OTA ERROR!", 2000);
        return false;
    }
    
    Serial.printf("[DUAL-BOOT] Boot partition set to: %s (0x%08x)\n",
                  partition_name, partition->address);
    
    return true;
}

void DualBootManager::triggerReboot() {
    Serial.println("[DUAL-BOOT] Rebooting in 200ms...");
    showFeedback("REBOOTING...", 200);
    Serial.flush();
    delay(200);
    esp_restart();
}

void DualBootManager::detectCurrentPartition() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    
    if (running == NULL) {
        Serial.println("[DUAL-BOOT] WARNING: Could not detect running partition");
        config.current_os = OS_FUSION_WATCH;
        return;
    }
    
    Serial.printf("[DUAL-BOOT] Running from partition: %s @ 0x%08x (size: 0x%08x)\n",
                  running->label, running->address, running->size);
    
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
    Serial.println("\n===== DUAL BOOT STATUS (v2.0) =====");
    Serial.printf("Current OS: %d (%s)\n", config.current_os, config.os_names[config.current_os]);
    Serial.printf("Other OS: %d (%s)\n", getOtherOS(), config.os_names[getOtherOS()]);
    Serial.printf("Boot Count: %d\n", config.boot_count);
    Serial.printf("Pending Switch: %s\n", config.pending_switch ? "YES" : "NO");
    Serial.printf("SD Card: %s\n", sd_available ? "Available" : "Not mounted");
    Serial.printf("Internal FAT: %s\n", fat_available ? "Available" : "Not mounted");
    Serial.printf("GPIO 0 (BOOT): %s\n", gpio0_working ? "Working" : "FAILED");
    Serial.println("-----------------------------------");
    Serial.println("TRIGGERS:");
    Serial.printf("  BOOT button (GPIO %d): Double-tap within %dms\n", BOOT_BUTTON_GPIO, RUNTIME_TAP_WINDOW_MS);
    Serial.printf("  PWR button (GPIO %d): Triple-tap within %dms\n", POWER_BUTTON_GPIO, TRIPLE_TAP_WINDOW_MS);
    Serial.printf("  PWR button (GPIO %d): Long-press %dms\n", POWER_BUTTON_GPIO, LONG_PRESS_MS);
    Serial.println("====================================\n");
}

