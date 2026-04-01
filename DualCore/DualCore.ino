/**
 * ═══════════════════════════════════════════════════════════════════════════
 * DUAL CORE - ESP32 Unified Dual-OS Firmware
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Contains BOTH complete operating systems:
 *   - FUSION OS (Anime Gaming Watch) - FULL VERSION
 *   - WIDGET OS (S3 MiniOS v7.4) - FULL VERSION
 * 
 * Golden "ACTIVATE" button switches between them via soft reset.
 * Only ONE OS runs at a time - zero battery drain from inactive OS.
 * 
 * Hardware: ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>

// ═══════════════════════════════════════════════════════════════════════════
// DUAL CORE OS SELECTION
// ═══════════════════════════════════════════════════════════════════════════

#define OS_FUSION  0
#define OS_WIDGET  1

#define NVS_NAMESPACE "dualcore"
#define NVS_KEY       "activeOS"

Preferences osPrefs;
int activeOS = OS_FUSION;

// ═══════════════════════════════════════════════════════════════════════════
// OS SWITCHING FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

int getActiveOS() {
    osPrefs.begin(NVS_NAMESPACE, true);
    int os = osPrefs.getInt(NVS_KEY, OS_FUSION);
    osPrefs.end();
    return os;
}

void setActiveOS(int os) {
    osPrefs.begin(NVS_NAMESPACE, false);
    osPrefs.putInt(NVS_KEY, os);
    osPrefs.end();
}

void switchToWidgetOS() {
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println("  DUAL CORE: Switching to WIDGET OS");
    Serial.println("═══════════════════════════════════════════════════════════════");
    setActiveOS(OS_WIDGET);
    delay(100);
    ESP.restart();
}

void switchToFusionOS() {
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println("  DUAL CORE: Switching to FUSION OS");
    Serial.println("═══════════════════════════════════════════════════════════════");
    setActiveOS(OS_FUSION);
    delay(100);
    ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS FROM EACH OS
// ═══════════════════════════════════════════════════════════════════════════

// From fusion_os_full.cpp
extern void fusionOS_setup();
extern void fusionOS_loop();
extern bool dualCoreAnimActive;
extern bool updateDualCoreAnimation();

// From widget_os_full.cpp
extern void widgetOS_setup();
extern void widgetOS_loop();

// ═══════════════════════════════════════════════════════════════════════════
// ARDUINO SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println("  DUAL CORE - ESP32 Unified Dual-OS Firmware");
    Serial.println("  Fusion OS + Widget OS");
    Serial.println("═══════════════════════════════════════════════════════════════");
    
    // Get which OS to run
    activeOS = getActiveOS();
    
    Serial.printf("  Active OS: %s\n", activeOS == OS_FUSION ? "FUSION OS" : "WIDGET OS");
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println();
    
    // Initialize the selected OS
    if (activeOS == OS_FUSION) {
        fusionOS_setup();
    } else {
        widgetOS_setup();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ARDUINO LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    if (activeOS == OS_FUSION) {
        // Check for Dual Core animation
        if (dualCoreAnimActive) {
            updateDualCoreAnimation();
            delay(16);
            return;
        }
        fusionOS_loop();
    } else {
        widgetOS_loop();
    }
}
