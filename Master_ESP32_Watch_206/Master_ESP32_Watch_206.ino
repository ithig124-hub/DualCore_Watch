//✅ Timeout changed: 3 seconds ➔ 5 seconds
//✅ Power button changed: GPIO 0 ➔ GPIO 10
//✅ Screen timeout logic: Uses simple 5-second check (bypasses power manager)
/*
 * ESP32_Watch.ino - Main Firmware with Power Management
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 * 
 * POWER MANAGEMENT FEATURES:
 * - 5-second screen timeout (display off)
 * - Touch to wake (display on)
 * - Power button (GPIO 10) to toggle screen on/off
 * - Watchdog timer (10 sec) for system stability
 * 
 * NO LIGHT SLEEP / NO DEEP SLEEP - Simple screen on/off only
 * 
 * Hardware: ESP32-S3-Touch-AMOLED-2.06"
 * Display: 410x502 CO5300 AMOLED (QSPI)
 * Touch: FT3168 Capacitive (I2C)
 * PMU: AXP2101
 * RTC: PCF85063
 * IMU: QMI8658
 */

#include <esp_task_wdt.h>
#include <Preferences.h>
#include <esp_system.h>
#include <nvs_flash.h>

#include "config.h"
#include "display.h"
#include "hardware.h"
#include "touch.h"
#include "navigation.h"
#include "themes.h"
#include "apps.h"
#include "games.h"
#include "gacha.h"
#include "training.h"
#include "boss_rush.h"
#include "wifi_apps.h"
#include "filesystem.h"
#include "boboiboy_elements.h"
#include "ochobot.h"
#include "dynamic_bg.h"
#include "fusion_game.h"
#include "character_games.h"
#include "steps_tracker.h"
#include "daily_quests.h"
#include "ui.h"
#include "sd_manager.h"
#include "power_manager.h"
#include "standby_mode.h"
#include "battery_app.h"
#include "xp_system.h"
#include "wifi_sync.h"
#include "time_edit.h"
#include "storyline.h"
#include "companion.h"
#include "new_apps.h"

// ══════════════════════════════════════════════════════════════════════════════════════════
// DUAL BOOT SUPPORT - Double-tap BOOT button to switch OS
// ══════════════════════════════════════════════════════════════════════════════════════════
#include "dual_boot_manager.h"
#include "sd_preferences.h"

// =============================================================================
// POWER MANAGEMENT DEFINES
// =============================================================================
#define PWR_BUTTON               10      // Power button GPIO (CHANGED from 0)
#define BUTTON_DEBOUNCE_MS      50      // Button debounce time
#define SCREEN_OFF_TIMEOUT_MS   5000    // 5 seconds to turn screen off (CHANGED from 3000)
#define WATCHDOG_TIMEOUT_SEC    10      // Watchdog timeout in seconds
#define BOOT_PANIC_THRESHOLD    3       // Skip WiFi after N consecutive WDT panics

// =============================================================================
// DISPLAY SETUP - CO5300 for 2.06" AMOLED
// =============================================================================
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

// CO5300 display driver with column offset for 2.06" board
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0 /* rotation */, LCD_WIDTH, LCD_HEIGHT,
    22 /* col_offset1 */, 0 /* row_offset1 */,
    0 /* col_offset2 */, 0 /* row_offset2 */);

// =============================================================================
// POWER MANAGEMENT STATE
// =============================================================================
volatile bool screenOn = true;
volatile bool touchWakeFlag = false;
volatile bool buttonWakeFlag = false;
volatile unsigned long lastActivityMs = 0;
static bool lastPwrButtonState = HIGH;
static unsigned long lastPwrButtonChange = 0;

// =============================================================================
// GLOBAL SYSTEM STATE
// =============================================================================
SystemState system_state = {
  .current_screen = SCREEN_SPLASH,
  .current_theme = THEME_LUFFY_GEAR5,
  .current_app = APP_WATCHFACE,
  .brightness = 200,
  .display_available = false,
  .touch_available = false,
  .touch_active = false,
  .power_available = false,
  .battery_percentage = 85,
  .is_charging = false,
  .low_battery_warning = false,
  .lvgl_available = false,
  .sleep_timer = 0,
  .wifi_connected = false,
  .wifi_ssid = "",
  .steps_today = 3456,
  .step_goal = 10000,
  .player_gems = 500,
  .player_level = 1,
  .player_xp = 0,
  .gacha_cards_collected = 0,
  .bosses_defeated = 0,
  .training_streak = 0,
  .daily_login_count = 0,
  .music_playing = false,
  .total_mp3_files = 0,
  .total_pdf_files = 0,
  .filesystem_available = false,
  .current_wallpaper_path = "",
  .wallpaper_enabled = false,
  .low_power_mode = false,
  .power_saver_enabled = false
};

// Touch interrupt flag
volatile bool touch_interrupt = false;

// Boot guard state
static bool safeModeBoot = false;

// =============================================================================
// INTERRUPT SERVICE ROUTINES
// =============================================================================

void IRAM_ATTR touchWakeISR() {
    touchWakeFlag = true;
    touch_interrupt = true;   // Also trigger touch input handling
    lastActivityMs = millis();
}

void IRAM_ATTR powerButtonISR() {
    buttonWakeFlag = true;
}

// =============================================================================
// WATCHDOG FUNCTIONS
// =============================================================================

void initWatchdog() {
    Serial.println("[WDT] Initializing watchdog timer...");
    
    // ==============================================
    // FIX: On some ESP32-S3 boards with Arduino Core 3.x, the framework does NOT
    // auto-initialize the TWDT before setup(). esp_task_wdt_reconfigure() fails
    // with "TWDT was never initialized". We MUST call esp_task_wdt_init() ourselves.
    //
    // Strategy: deinit (clean slate) ➔ init fresh ➔ subscribe loopTask only.
    // Use a long timeout (120s) for the entire setup() phase.
    // Normal 10s timeout is set at end of setup() before loop() starts.
    // ==============================================
    
    // Step 1: Clean slate - remove any pre-existing WDT (OK to fail if none exists)
    esp_task_wdt_deinit();
    
    // Step 2: Initialize TWDT fresh with long timeout for setup phase
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 120000,      // 120s - covers entire setup() including WiFi
        .idle_core_mask = 0,       // Do NOT monitor idle tasks (they can't feed during setup)
        .trigger_panic = true
    };
    
    esp_err_t err = esp_task_wdt_init(&wdt_config);
    if (err != ESP_OK) {
        Serial.printf("[WDT] Init failed (%d) - trying reconfigure\n", err);
        err = esp_task_wdt_reconfigure(&wdt_config);
    }
    
    if (err != ESP_OK) {
        Serial.printf("[WDT] WARNING: Could not configure WDT (%d)\n", err);
        return;  // Continue boot without WDT rather than crash
    }
    
    // Step 3: Subscribe loopTask only
    err = esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    if (err == ESP_ERR_INVALID_ARG) {
        Serial.println("[WDT] Task already subscribed to watchdog");
    } else if (err != ESP_OK) {
        Serial.printf("[WDT] Add task result: %d\n", err);
    }
    
    // Step 4: Verify we can feed it
    err = esp_task_wdt_reset();
    if (err != ESP_OK) {
        Serial.printf("[WDT] WARNING: Feed failed (%d)\n", err);
    }
    
    Serial.println("[WDT] Watchdog initialized: 120s timeout (setup phase)");
}

void feedWatchdog() {
    esp_task_wdt_reset();
}

// =============================================================================
// BOOT GUARD - Skip WiFi after consecutive WDT panics
// =============================================================================

void checkBootPanic() {
    Preferences bootPrefs;
    bootPrefs.begin("bootguard", false);
    
    // Check what caused the last reboot
    esp_reset_reason_t reason = esp_reset_reason();
    bool was_panic = (reason == ESP_RST_TASK_WDT ||
                      reason == ESP_RST_INT_WDT ||
                      reason == ESP_RST_WDT ||
                      reason == ESP_RST_PANIC);
    
    int panicCount = bootPrefs.getInt("panics", 0);
    
    if (was_panic) {
        panicCount++;
        bootPrefs.putInt("panics", panicCount);
        Serial.printf("[BOOT GUARD] WDT/panic reboot detected! Count: %d/%d\n",
            panicCount, BOOT_PANIC_THRESHOLD);
    } else {
        // Clean boot (power on, manual reset, etc.)
        if (panicCount > 0) {
            Serial.printf("[BOOT GUARD] Clean boot - resetting panic counter (was %d)\n", panicCount);
            bootPrefs.putInt("panics", 0);
        }
        panicCount = 0;
    }
    
    if (panicCount >= BOOT_PANIC_THRESHOLD) {
        safeModeBoot = true;
        Serial.println("[BOOT GUARD] *** SAFE MODE ACTIVATED ***");
        Serial.printf("[BOOT GUARD] %d consecutive panics - WiFi SKIPPED this boot\n", panicCount);
    }
    
    bootPrefs.end();
    
    // Log reset reason for debugging
    const char* reasons[] = {
        "UNKNOWN", "POWERON", "EXT", "SW", "PANIC",
        "INT_WDT", "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO"
    };
    int r = (int)reason;
    Serial.printf("[BOOT GUARD] Reset reason: %s (%d)\n",
        (r >= 0 && r <= 10) ? reasons[r] : "?", r);
}

void clearBootPanicCounter() {
    Preferences bootPrefs;
    bootPrefs.begin("bootguard", false);
    int oldCount = bootPrefs.getInt("panics", 0);
    if (oldCount > 0) {
        bootPrefs.putInt("panics", 0);
        Serial.printf("[BOOT GUARD] Boot successful! Panic counter cleared (was %d)\n", oldCount);
    }
    bootPrefs.end();
    safeModeBoot = false;
}

// =============================================================================
// SCREEN CONTROL FUNCTIONS
// =============================================================================
void screenOff() {
    if (!screenOn) return;
    
    Serial.println("[POWER] Screen OFF - smooth fade");
    
    screenOn = false;
    
    int currentBrightness = system_state.brightness;
    for (int b = currentBrightness; b >= 0; b -= 20) {
        gfx->setBrightness(max(0, b));
        delay(8);
    }
    gfx->setBrightness(0);
    
    gfx->displayOff();
}

void screenOnFunc() {
    if (screenOn) return;
    
    Serial.println("[POWER] Screen ON - smooth fade");
    
    gfx->displayOn();
    gfx->setBrightness(0);
    
    drawCurrentScreen();
    
    delay(15);
    
    for (int b = 0; b <= system_state.brightness; b += 20) {
        gfx->setBrightness(min(b, (int)system_state.brightness));
        delay(8);
    }
    gfx->setBrightness(system_state.brightness);
    
    screenOn = true;
    lastActivityMs = millis();
}

void checkScreenTimeout() {
    if (!screenOn) return;
    
    unsigned long elapsed = millis() - lastActivityMs;
    if (elapsed >= SCREEN_OFF_TIMEOUT_MS) {
        screenOff();
    }
}

void checkPowerButton() {
    bool currentState = digitalRead(PWR_BUTTON);
    
    // Debounce
    if (currentState != lastPwrButtonState) {
        if (millis() - lastPwrButtonChange > BUTTON_DEBOUNCE_MS) {
            lastPwrButtonChange = millis();
            lastPwrButtonState = currentState;
            
            // Button pressed (LOW because of pullup)
            if (currentState == LOW) {
                Serial.println("[POWER] Power button pressed (GPIO 10)");
                
                // Toggle screen
                if (screenOn) {
                    screenOff();
                } else {
                    screenOnFunc();
                }
                
                lastActivityMs = millis();
            }
        }
    }
    
    // Also check button wake flag from ISR
    if (buttonWakeFlag) {
        buttonWakeFlag = false;
        if (!screenOn) {
            screenOnFunc();
        }
        lastActivityMs = millis();
    }
}

void checkTouchWake() {
    if (touchWakeFlag) {
        touchWakeFlag = false;
        if (!screenOn) {
            screenOnFunc();
        }
        lastActivityMs = millis();
    }
}

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void updateCurrentScreen();
void handleTouchGesture(TouchGesture& gesture);
void saveAllData();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n=====================");
  Serial.println(" ESP32 Anime Gaming Watch IMPROVED");
  Serial.println(" With Watchdog & Screen Timeout");
  Serial.println("=====================\n");

  // ══════════════════════════════════════════════════════════════════════════════════════════
  // DUAL BOOT INITIALIZATION - Must be early, before other init!
  // Double-tap BOOT button (GPIO 0) within 500ms to switch OS
  // ══════════════════════════════════════════════════════════════════════════════════════════
  DualBoot.begin();                    // Initialize boot manager
  DualBoot.beginSD();                  // Mount SD card via SD_MMC
  DualBoot.checkDoubleTapBoot();       // Check for OS switch
  Serial.printf("[BOOT] Running: %s\n", DualBoot.getCurrentOSName());
  Serial.printf("[BOOT] Double-tap BOOT to switch to: %s\n", DualBoot.getOtherOSName());
  
  // Create /WATCH/FusionData/ folder for SD-based preferences
  SDPreferences::initFusionDataFolder();
  // ══════════════════════════════════════════════════════════════════════════════════════════

  initWatchdog();
  feedWatchdog();
  
  checkBootPanic();
  feedWatchdog();
  
  initializeHardware();
  feedWatchdog();
  
  initDisplay();
  feedWatchdog();
  
  // =============================================================================
  // LOAD SAVED THEME from dedicated NVS namespace — BEFORE initializeThemes().
  // This reads from "theme_cfg" (isolated, nothing else writes to it).
  // Must happen before initializeThemes() so the correct theme is applied.
  // =============================================================================
  {
    ThemeType saved = loadThemeFromNVS();
    system_state.current_theme = saved;
    Serial.printf("[BOOT] Theme loaded from NVS: %d\n", (int)saved);
  }
  feedWatchdog();
  
  drawSplashScreen();
  delay(2000);
  feedWatchdog();
  
  if (initTouch()) {
    Serial.println("[INIT] Touch initialized");
    system_state.touch_available = true;
    
    // Single ISR handles both touch input and screen wake
    // (only ONE ISR allowed per pin - second attachInterrupt overwrites the first)
    attachInterrupt(digitalPinToInterrupt(TP_INT), touchWakeISR, FALLING);
  }
  
  // Initialize power button on GPIO 10
  pinMode(PWR_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PWR_BUTTON), powerButtonISR, FALLING);
  Serial.println("[INIT] Power button: GPIO 10");
  
  initializeThemes();
  feedWatchdog();
  
  loadAllGameData();
  feedWatchdog();
  
  initNavigation();
  
  initializeApps();
  feedWatchdog();
  
  initializeGames();
  feedWatchdog();
  
  initGachaSystem();
  feedWatchdog();
  
  initTrainingSystem();
  feedWatchdog();
  
  initBossRush();
  feedWatchdog();
  
  initXPSystem();
  checkDailyLoginBonus();
  feedWatchdog();
  
  Serial.println("[INIT] Initializing filesystem...");
  feedWatchdog();
  initFilesystem();
  feedWatchdog();
  
  Serial.println("[INIT] Initializing WiFi sync...");
  feedWatchdog();
  initWiFiSync();  // Always load SD card networks (even in safe mode)
  feedWatchdog();
  
  if (safeModeBoot) {
    // =============================================================================
    // SAFE MODE: Skip WiFi entirely - go straight to watchface with RTC time
    // =============================================================================
    Serial.println("\n[BOOT] *** SAFE MODE: Skipping WiFi sync ***");
    Serial.println("[BOOT] -> Using RTC chip time (PCF85063)");
    WatchTime rtc = getCurrentTime();
    Serial.printf("[BOOT] -> RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
      rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
    Serial.println("[BOOT] -> WiFi networks loaded from SD (available for manual sync)");
  } else {
    Serial.println("\n[BOOT] Starting WiFi boot sync...");
    feedWatchdog();
    
    // WDT already set to 120s in initWatchdog() — covers entire setup including WiFi
    feedWatchdog();
    
    unsigned long beforeSync = millis();
    bool wifi_synced = performBootSync();
    unsigned long syncDuration = millis() - beforeSync;
    
    feedWatchdog();
    
    Serial.printf("[WiFi] Sync attempt took %lu ms\n", syncDuration);
    
    if (wifi_synced) {
      Serial.println("[BOOT] WiFi sync successful!");
    } else {
      Serial.println("[BOOT] WiFi sync failed or skipped");
      Serial.println("[BOOT] -> Using RTC chip time (PCF85063)");
      WatchTime rtc = getCurrentTime();
      Serial.printf("[BOOT] -> RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
        rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
    }
  }
  
  feedWatchdog();
  
  initWiFiApps();
  feedWatchdog();
  
  initStepsTracker();
  feedWatchdog();
  initDailyQuests();
  feedWatchdog();
  initStorySystem();
  feedWatchdog();
  initCompanionSystem();
  feedWatchdog();
  
  lastActivityMs = millis();
  screenOn = true;
  
  initPowerManager();
  feedWatchdog();

  // Layered standby (IDLE -> AOD -> DEEP) + self-learning runtime estimator.
  // Keeps using the active character watchface as the AOD face.
  standbyInit();
  system_state.steps_today = (int)standby_rtc_steps;  // restore across deep sleep
  feedWatchdog();
  
  system_state.current_screen = SCREEN_WATCHFACE;
  drawWatchFace();
  drawNavigationIndicators();
  
  // =============================================================================
  // SETUP COMPLETE: Clear boot panic counter + tighten WDT to runtime
  // =============================================================================
  clearBootPanicCounter();
  
  {
    esp_task_wdt_config_t runtime_wdt = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&runtime_wdt);
    esp_task_wdt_reset();
  }
  Serial.printf("[WDT] Watchdog tightened to %d seconds for runtime\n", WATCHDOG_TIMEOUT_SEC);
  
  Serial.println("\n[INIT] ===== FUSION OS READY! =====");
  Serial.println("       - Screen Timeout: 5 seconds");
  Serial.println("       - Wake: Touch or GPIO 10 button");
  Serial.printf("       - Watchdog: %d seconds\n", WATCHDOG_TIMEOUT_SEC);
  Serial.printf("       - Boot guard: %d panic threshold\n", BOOT_PANIC_THRESHOLD);
  Serial.println("       Battery Target: 8-12 hours");
}

// =============================================================================
// MAIN LOOP - FUSION OS OPTIMIZED
// =============================================================================

void loop() {
  feedWatchdog();
  
  // DUAL BOOT: Check for double-tap on BOOT button (GPIO 0) to switch OS
  // Non-blocking — polls button state each loop iteration
  DualBoot.checkAllTriggers();

  
  static unsigned long lastAutoSave = 0;
  if (millis() - lastAutoSave > 300000) {
    lastAutoSave = millis();
    saveAllData();
    Serial.println("[AUTO-SAVE] All game data saved (5 min interval)");
  }
  
  updatePowerState();
  
  checkPowerButton();
  
  checkTouchWake();
  
  if (screenOn) {
    TouchGesture gesture = handleTouchInput();
    
    if (gesture.is_valid && gesture.event != TOUCH_NONE) {
      recordInteraction();
      standbyRecordInteraction();
      lastActivityMs = millis();  // Reset 5-second timer on touch
      
      extern bool time_edit_active;
      if (time_edit_active && gesture.event == TOUCH_TAP) {
        extern void handleTimeEditTouch(int x, int y);
        handleTimeEditTouch(gesture.x, gesture.y);
      }
      else if (gesture.event != TOUCH_PRESS && gesture.event != TOUCH_MOVE) {
        handleTouchGesture(gesture);
      }
    }
    
    updateCurrentScreen();
    
    static unsigned long lastStepUpdate = 0;
    int step_interval = getSensorPollInterval();
    if (millis() - lastStepUpdate > step_interval) {
      updateStepCount();
      lastStepUpdate = millis();
    }
    
    static unsigned long lastQuestCheck = 0;
    if (millis() - lastQuestCheck > 60000) {
      checkDailyReset();
      lastQuestCheck = millis();
    }
    
    // Update Pomodoro timer
    updatePomodoro();
    
    // SIMPLE 5-SECOND TIMEOUT CHECK (bypasses power manager)
    if (millis() - lastActivityMs >= SCREEN_OFF_TIMEOUT_MS) {
      screenOff();
    }
  }
  
  int loop_delay = getPowerLoopDelay();
  standbyTick();      // layered standby state machine + learning sampler
  delay(loop_delay);
}

// =============================================================================
// TOUCH GESTURE HANDLERS
// =============================================================================

void handleTouchGesture(TouchGesture& gesture) {
  Serial.printf("[MAIN] Gesture: %d at (%d, %d)\n", gesture.event, gesture.x, gesture.y);
  
  lastActivityMs = millis();
  
  if (gesture.event == TOUCH_SWIPE_UP) {
    if (system_state.current_screen == SCREEN_CHARACTER_GAME ||
        system_state.current_screen == SCREEN_BOSS_RUSH ||
        system_state.current_screen == SCREEN_GAMES) {
    }
    else {
      switch (system_state.current_screen) {
        case SCREEN_WATCHFACE:
        case SCREEN_STEPS_TRACKER:
          handleSwipeNavigation(gesture.dx, gesture.dy);
          break;
        
        case SCREEN_CHARACTER_STATS:
          system_state.current_screen = SCREEN_PROGRESSION;
          drawProgressionScreen();
          break;
        
        case SCREEN_APP_GRID:
          if (navState.appGridPage < APP_GRID_PAGES - 1) {
            navState.appGridPage++;
            navState.lastNavigationMs = millis();
            drawCurrentScreen();
          }
          break;
        
        case SCREEN_COLLECTION:
        case SCREEN_CARD_EVOLUTION:
        case SCREEN_DECK_BUILDER:
          system_state.current_screen = SCREEN_GACHA;
          drawGachaScreen();
          break;
        
        case SCREEN_THEME_SELECTOR:
          system_state.current_screen = SCREEN_SETTINGS;
          drawSettingsApp();
          break;
        
        default:
          returnToAppGrid();
          break;
      }
      return;
    }
  }
  
  if (gesture.event == TOUCH_SWIPE_DOWN) {
    if (system_state.current_screen == SCREEN_APP_GRID) {
      if (navState.appGridPage > 0) {
        navState.appGridPage--;
        navState.lastNavigationMs = millis();
        drawCurrentScreen();
      }
      return;
    }
  }
  
  switch (system_state.current_screen) {
    case SCREEN_WATCHFACE:
    case SCREEN_APP_GRID:
      if (gesture.event == TOUCH_TAP || gesture.event == TOUCH_RELEASE) {
        handleCurrentScreenTouch(gesture);
      } else if (gesture.event >= TOUCH_SWIPE_LEFT && gesture.event <= TOUCH_SWIPE_DOWN) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;
    
    case SCREEN_CHARACTER_STATS:
      if (gesture.event == TOUCH_TAP) {
        handleCurrentScreenTouch(gesture);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_RIGHT) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;

    case SCREEN_BATTERY:
      if (gesture.event == TOUCH_TAP) {
        batteryAppHandleTouch(gesture);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_RIGHT) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;
    
    case SCREEN_PROGRESSION:
      handleProgressionTouch(gesture);
      break;
    
    case SCREEN_STEPS_TRACKER:
      if (gesture.event == TOUCH_TAP) {
        handleStepsCardTouch(gesture);
      } else if (gesture.event >= TOUCH_SWIPE_LEFT && gesture.event <= TOUCH_SWIPE_DOWN) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;
    
    case SCREEN_GACHA:
      handleGachaTouch(gesture);
      break;
    
    case SCREEN_TRAINING:
      handleTrainingMenuTouch(gesture);
      break;
    
    case SCREEN_BOSS_RUSH:
      handleBossRushTouch(gesture);
      break;
    
    case SCREEN_GAMES:
      handleGameMenuTouch(gesture);
      break;
    
    case SCREEN_QUESTS:
      handleDailyQuestsTouch(gesture);
      break;
    
    case SCREEN_DAILY_QUESTS:
      handleDailyQuestsTouch(gesture);
      break;
    
    case SCREEN_ELEMENT_TREE:
      handleElementTreeTouch(gesture);
      break;
    
    case SCREEN_FUSION_GAME:
      handleFusionGameTouch(gesture);
      break;
    
    case SCREEN_CHARACTER_GAME:
      handleCharacterGameTouch(gesture);
      break;
    
    case SCREEN_SETTINGS:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        // Allow swiping back from settings/sub-screens to app grid
        returnToAppGrid();
      } else if (gesture.event == TOUCH_TAP || gesture.event == TOUCH_RELEASE) {
        handleSettingsTouch(gesture);
      }
      break;
    
    case SCREEN_THEME_SELECTOR:
      handleThemeSelectorTouch(gesture);
      break;
    
    case SCREEN_MUSIC:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleMusicTouch(gesture);
      }
      break;
    
    case SCREEN_WIFI_MANAGER:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleWifiManagerTouch(gesture);
      }
      break;
    
    case SCREEN_COLLECTION:
      handleCollectionTouch(gesture);
      break;
    
    case SCREEN_CARD_EVOLUTION:
      if (gesture.event == TOUCH_TAP) {
        handleCardEvolutionTap(gesture.x, gesture.y);
      }
      break;
    
    case SCREEN_DECK_BUILDER:
      if (gesture.event == TOUCH_TAP) {
        handleDeckBuilderTap(gesture.x, gesture.y);
      }
      break;
    
    case SCREEN_FILE_BROWSER:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleFileBrowserTouch(gesture);
      }
      break;
    
    case SCREEN_CALCULATOR:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleCalculatorTouch(gesture);
      }
      break;
    
    case SCREEN_FLASHLIGHT:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleFlashlightTouch(gesture);
      }
      break;
    
    case SCREEN_TIMER:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleTimerTouch(gesture);
      }
      break;
    
    case SCREEN_CONVERTER:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleConverterTouch(gesture);
      }
      break;
    
    case SCREEN_ACHIEVEMENTS:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleAchievementsTouch(gesture);
      }
      break;
    
    case SCREEN_SHOP:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleShopTouch(gesture);
      }
      break;
    
    case SCREEN_GALLERY:
      if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      } else {
        handleGalleryTouch(gesture);
      }
      break;

    case SCREEN_STORY_MENU:
      handleStoryMenuTouch(gesture);
      break;

    case SCREEN_CHAPTER_SELECT:
      handleChapterSelectTouch(gesture);
      break;

    case SCREEN_STORY_DIALOGUE:
      handleDialogueTouch(gesture);
      break;

    case SCREEN_STORY_BOSS:
      handleStoryBossTouch(gesture);
      break;

    case SCREEN_COMPANION:
    case SCREEN_COMPANION_GAME:
      if (gesture.event == TOUCH_TAP) {
        handleCareModeTouch(gesture.x, gesture.y);
        drawCompanionCareScreen();
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        exitCompanionCareMode();
        system_state.current_screen = SCREEN_APP_GRID;
      }
      break;
    
    case SCREEN_POMODORO:
      if (gesture.event == TOUCH_TAP) {
        handlePomodoroTouch(gesture.x, gesture.y);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      }
      break;
    
    case SCREEN_HABITS:
      if (gesture.event == TOUCH_TAP) {
        handleHabitsTouch(gesture.x, gesture.y);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      }
      break;
    
    case SCREEN_DUNGEON:
      if (gesture.event == TOUCH_TAP) {
        handleDungeonTouch(gesture.x, gesture.y);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        returnToAppGrid();
      }
      break;
    
    case SCREEN_SD_BACKUP:
      if (gesture.event == TOUCH_TAP) {
        if (in_backup_list_view) {
          handleBackupListTouch(gesture.x, gesture.y);
        } else {
          handleSDBackupTouch(gesture.x, gesture.y);
        }
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        system_state.current_screen = SCREEN_SETTINGS;
        drawSettingsApp();
      }
      break;
    
    default:
      break;
  }
}

// =============================================================================
// SCREEN UPDATE - FUSION OS OPTIMIZED
// =============================================================================

void updateCurrentScreen() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastTimerUpdate = 0;
  static unsigned long lastBatteryUpdate = 0;
  static int last_minute = -1;
  static ScreenType last_screen = SCREEN_SPLASH;
  
  bool screen_changed = (system_state.current_screen != last_screen);
  if (screen_changed) {
    last_screen = system_state.current_screen;
    Serial.printf("[SCREEN] Changed to: %d\n", system_state.current_screen);
    
    extern void forceWatchfaceRedraw();
    forceWatchfaceRedraw();
  }
  
  if (millis() - lastBatteryUpdate > 30000) {
    lastBatteryUpdate = millis();
    updateBatteryStatus();
  }
  
  if (system_state.current_screen == SCREEN_WATCHFACE && 
      navState.currentMain == MAIN_WATCHFACE) {
    WatchTime current_time = getCurrentTime();
    
    if (current_time.minute != last_minute) {
      last_minute = current_time.minute;
      
      updateWatchFaceTime();
      
      Serial.printf("[CLOCK] Minute changed ➔ %02d:%02d\n", 
                    current_time.hour, current_time.minute);
    }
  }
  
  if (system_state.current_screen == SCREEN_TIMER &&
      millis() - lastTimerUpdate > 50) {
    lastTimerUpdate = millis();
    updateTimerDisplay();
  }
  
  if (system_state.current_screen == SCREEN_GAMES && shouldAnimationsRun()) {
    AdvancedGameManager::updateGame();
  }
}

// =============================================================================
// SAVE ALL DATA - Persistent Storage using SD Card (/WATCH/FusionData/)
// =============================================================================

SDPreferences gamePrefs;

void saveAllGameData() {
  gamePrefs.begin("watchgame", false);
  
  // === UNIVERSAL DATA (shared across all themes) ===
  gamePrefs.putInt("theme", (int)system_state.current_theme);
  gamePrefs.putInt("steps", system_state.steps_today);
  gamePrefs.putInt("bright", system_state.brightness);
  gamePrefs.putInt("logins", system_state.daily_login_count);
  gamePrefs.putBool("pwrsaver", system_state.power_saver_enabled);
  
  // === PER-THEME DATA (saved with theme index prefix) ===
  int t = (int)system_state.current_theme;
  char key[16];
  
  snprintf(key, sizeof(key), "t%d_gems", t);
  gamePrefs.putInt(key, system_state.player_gems);
  
  snprintf(key, sizeof(key), "t%d_cards", t);
  gamePrefs.putInt(key, system_state.gacha_cards_collected);
  
  snprintf(key, sizeof(key), "t%d_boss", t);
  gamePrefs.putInt(key, system_state.bosses_defeated);
  
  snprintf(key, sizeof(key), "t%d_strk", t);
  gamePrefs.putInt(key, system_state.training_streak);
  
  snprintf(key, sizeof(key), "t%d_pity", t);
  gamePrefs.putInt(key, system_state.pity_counter);
  
  snprintf(key, sizeof(key), "t%d_pitl", t);
  gamePrefs.putInt(key, system_state.pity_legendary_counter);
  
  // Also save level/xp per theme (mirrors xp_system)
  snprintf(key, sizeof(key), "t%d_lvl", t);
  gamePrefs.putInt(key, system_state.player_level);
  
  snprintf(key, sizeof(key), "t%d_xp", t);
  gamePrefs.putInt(key, system_state.player_xp);
  
  gamePrefs.end();
  
  Serial.printf("[SAVE] Game data saved (theme %d)\n", t);
  Serial.printf("       Gems: %d, Cards: %d, Bosses: %d, Steps: %d\n", 
    system_state.player_gems, system_state.gacha_cards_collected,
    system_state.bosses_defeated, system_state.steps_today);
}

void loadAllGameData() {
  // Theme was already loaded from dedicated "theme_cfg" namespace in setup().
  // We keep that value and DON'T overwrite it from "watchgame".
  ThemeType already_loaded_theme = system_state.current_theme;
  
  gamePrefs.begin("watchgame", true);
  
  // === UNIVERSAL DATA (skip theme — already loaded from "theme_cfg") ===
  // We still read the "watchgame" theme for per-theme data key prefix,
  // but we DON'T assign it to system_state.current_theme.
  system_state.steps_today = gamePrefs.getInt("steps", 0);
  system_state.brightness = gamePrefs.getInt("bright", 200);
  system_state.daily_login_count = gamePrefs.getInt("logins", 0);
  system_state.power_saver_enabled = gamePrefs.getBool("pwrsaver", false);
  
  // === PER-THEME DATA — use the theme loaded from "theme_cfg" ===
  int t = (int)already_loaded_theme;
  char key[16];
  
  snprintf(key, sizeof(key), "t%d_gems", t);
  system_state.player_gems = gamePrefs.getInt(key, 500);
  
  snprintf(key, sizeof(key), "t%d_cards", t);
  system_state.gacha_cards_collected = gamePrefs.getInt(key, 0);
  
  snprintf(key, sizeof(key), "t%d_boss", t);
  system_state.bosses_defeated = gamePrefs.getInt(key, 0);
  
  snprintf(key, sizeof(key), "t%d_strk", t);
  system_state.training_streak = gamePrefs.getInt(key, 0);
  
  snprintf(key, sizeof(key), "t%d_pity", t);
  system_state.pity_counter = gamePrefs.getInt(key, 0);
  
  snprintf(key, sizeof(key), "t%d_pitl", t);
  system_state.pity_legendary_counter = gamePrefs.getInt(key, 0);
  
  snprintf(key, sizeof(key), "t%d_lvl", t);
  system_state.player_level = gamePrefs.getInt(key, 1);
  
  snprintf(key, sizeof(key), "t%d_xp", t);
  system_state.player_xp = gamePrefs.getInt(key, 0);
  
  gamePrefs.end();
  
  setTheme(system_state.current_theme);
  
  Serial.printf("[LOAD] Game data loaded (theme %d)\n", t);
  Serial.printf("       Gems: %d, Cards: %d, Bosses: %d, Steps: %d\n", 
    system_state.player_gems, system_state.gacha_cards_collected,
    system_state.bosses_defeated, system_state.steps_today);
}

void saveAllData() {
  saveAllGameData();
  saveGachaProgress();
  saveGameProgress();
  saveTrainingProgress();
  saveBossProgress();
  saveXPData();
  saveStepsData();
  saveStoryProgress();
  
  // Also save to SD card if available
  if (sdCardInitialized) {
    savePlayerDataToSD();
    saveGachaDataToSD();
    saveBossDataToSD();
  }
  
  Serial.println("[SAVE] All data saved (SD Card Fusion Data)");
}
