/*
 * ESP32_Watch.ino - Main Firmware with Power Management
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 * 
 * POWER MANAGEMENT FEATURES:
 * - 3-second screen timeout (display off)
 * - Touch to wake (display on)
 * - Power button (GPIO 0) to toggle screen on/off
 * - Watchdog timer (30 sec) for system stability
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
#include "rpg.h"
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

// =============================================================================
// POWER MANAGEMENT DEFINES
// =============================================================================
#define PWR_BUTTON              0       // Power/boot button GPIO
#define BUTTON_DEBOUNCE_MS      50      // Button debounce time
#define SCREEN_OFF_TIMEOUT_MS   3000    // 3 seconds to turn screen off
#define WATCHDOG_TIMEOUT_SEC    30      // Watchdog timeout in seconds

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
  .wallpaper_enabled = false
};

// Touch interrupt flag
volatile bool touch_interrupt = false;

// =============================================================================
// INTERRUPT SERVICE ROUTINES
// =============================================================================

void IRAM_ATTR touchWakeISR() {
    touchWakeFlag = true;
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
    
    // ESP-IDF 5.x / Arduino Core 3.x requires config struct
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,  // Convert to milliseconds
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Watch all cores
        .trigger_panic = true  // Reset on timeout
    };
    
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    
    Serial.printf("[WDT] Watchdog initialized: %d sec timeout\n", WATCHDOG_TIMEOUT_SEC);
}

void feedWatchdog() {
    esp_task_wdt_reset();
}

// =============================================================================
// SCREEN CONTROL FUNCTIONS
// =============================================================================

void screenOff() {
    if (!screenOn) return;
    
    Serial.println("[POWER] Screen OFF (3s timeout)");
    screenOn = false;
    
    // Turn off display backlight and display
    gfx->setBrightness(0);
    gfx->displayOff();
}

void screenOnFunc() {
    if (screenOn) return;
    
    Serial.println("[POWER] Screen ON");
    screenOn = true;
    lastActivityMs = millis();
    
    // Turn on display
    gfx->displayOn();
    gfx->setBrightness(system_state.brightness);
    
    // Refresh current screen
    drawCurrentScreen();
}

void checkScreenTimeout() {
    // Only check if screen is on
    if (!screenOn) return;
    
    // Check if 3 second timeout elapsed
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
                Serial.println("[POWER] Power button pressed");
                
                // Toggle screen
                if (screenOn) {
                    screenOff();
                } else {
                    screenOnFunc();
                }
                
                // Reset activity timer
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
    // Check if touch occurred while screen was off
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

void fusionOS_setup() {
  Serial.println("\n===================================");
  Serial.println(" FUSION OS - Anime Gaming Watch");
  Serial.println(" DUAL CORE MODE ENABLED");
  Serial.println("===================================\n");
  
  // Initialize watchdog FIRST
  initWatchdog();
  feedWatchdog();
  
  // Initialize hardware
  initializeHardware();
  feedWatchdog();
  
  // Initialize display
  initDisplay();
  feedWatchdog();
  
  // Show splash screen
  drawSplashScreen();
  delay(2000);
  feedWatchdog();
  
  // Initialize touch
  if (initTouch()) {
    Serial.println("[INIT] Touch initialized");
    system_state.touch_available = true;
    
    // Attach touch interrupt for normal touch AND wake
    attachInterrupt(digitalPinToInterrupt(TP_INT), touchISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(TP_INT), touchWakeISR, FALLING);
  }
  
  // Initialize power button
  pinMode(PWR_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PWR_BUTTON), powerButtonISR, FALLING);
  Serial.println("[INIT] Power button: GPIO 0");
  
  // Initialize themes
  initializeThemes();
  feedWatchdog();
  
  // LOAD SAVED GAME DATA FROM NVS (gems, level, XP, theme, etc.)
  loadAllGameData();
  feedWatchdog();
  
  // Initialize navigation system
  initNavigation();
  
  // Initialize apps
  initializeApps();
  feedWatchdog();
  
  // Initialize games
  initializeGames();
  feedWatchdog();
  
  // Initialize gacha
  initGachaSystem();
  feedWatchdog();
  
  // Initialize training
  initTrainingSystem();
  
  // Initialize boss rush
  initBossRush();
  
  // Initialize RPG
  initRPGSystem();
  feedWatchdog();
  
  // Initialize filesystem
  initFilesystem();
  feedWatchdog();
  
  // Initialize WiFi apps
  initWifiApps();
  feedWatchdog();
  
  // Initialize new features
  initStepsTracker();
  initDailyQuests();
  feedWatchdog();
  
  // Initialize power management timing
  lastActivityMs = millis();
  screenOn = true;
  
  // Go to watch face
  system_state.current_screen = SCREEN_WATCHFACE;
  drawWatchFace();
  drawNavigationIndicators();
  
  Serial.println("\n[INIT] Ready!");
  Serial.println("       - Screen timeout: 3 seconds");
  Serial.println("       - Touch or Button to wake");
  Serial.println("       - Watchdog: 30 seconds");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void fusionOS_loop() {
  // Feed watchdog at start of each loop
  feedWatchdog();
  
  // Check power button (toggle screen)
  checkPowerButton();
  
  // Check for touch wake
  checkTouchWake();
  
  // Only process touch and updates when screen is on
  if (screenOn) {
    // Process touch input
    TouchGesture gesture = handleTouchInput();
    
    // CRITICAL: Reset activity timer on ANY touch input (including press/move)
    // This ensures the 3-second timeout resets whenever user touches the screen
    if (gesture.is_valid && gesture.event != TOUCH_NONE) {
      lastActivityMs = millis();  // Reset timeout on ANY touch
      
      if (gesture.event != TOUCH_PRESS && gesture.event != TOUCH_MOVE) {
        // Only handle completed gestures (tap, swipe, release)
        handleTouchGesture(gesture);
      }
    }
    
    // Update dynamic content based on screen
    updateCurrentScreen();
    
    // Update step counter (background)
    static unsigned long lastStepUpdate = 0;
    if (millis() - lastStepUpdate > 500) {
      updateStepCount();
      lastStepUpdate = millis();
    }
    
    // Check daily quest reset
    static unsigned long lastQuestCheck = 0;
    if (millis() - lastQuestCheck > 60000) {
      checkDailyReset();
      lastQuestCheck = millis();
    }
    
    // Check screen timeout (3 seconds)
    checkScreenTimeout();
  }
  
  // Reduce CPU usage
  delay(16);  // ~60 FPS
}

// =============================================================================
// TOUCH GESTURE HANDLER
// =============================================================================

void handleTouchGesture(TouchGesture& gesture) {
  Serial.printf("[MAIN] Gesture: %d at (%d, %d)\n", gesture.event, gesture.x, gesture.y);
  
  // Reset activity timer on any gesture
  lastActivityMs = millis();
  
  // SWIPE UP = EXIT from any app back to app grid
  if (gesture.event == TOUCH_SWIPE_UP) {
    if (system_state.current_screen == SCREEN_CHARACTER_GAME ||
        system_state.current_screen == SCREEN_BOSS_RUSH ||
        system_state.current_screen == SCREEN_GAMES) {
      // Pass through to game handler
    }
    else {
      switch (system_state.current_screen) {
        case SCREEN_WATCHFACE:
        case SCREEN_STEPS_TRACKER:    // FIX: Added - allows swipe navigation from Activity screen
          handleSwipeNavigation(gesture.dx, gesture.dy);
          break;
        
        case SCREEN_CHARACTER_STATS:
          // Swipe UP from character stats goes to progression screen
          system_state.current_screen = SCREEN_PROGRESSION;
          drawProgressionScreen();
          break;
        
        case SCREEN_APP_GRID:
          // Swipe UP on app grid - go to next page (0->1->2)
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
  
  // SWIPE DOWN = Go to previous app grid page (when on app grid)
  if (gesture.event == TOUCH_SWIPE_DOWN) {
    if (system_state.current_screen == SCREEN_APP_GRID) {
      // Swipe DOWN on app grid - go to previous page (2->1->0)
      if (navState.appGridPage > 0) {
        navState.appGridPage--;
        navState.lastNavigationMs = millis();
        drawCurrentScreen();
      }
      return;
    }
  }
  
  // Handle other gestures per screen
  switch (system_state.current_screen) {
    case SCREEN_WATCHFACE:
    case SCREEN_APP_GRID:
      if (gesture.event == TOUCH_TAP) {
        handleCurrentScreenTouch(gesture);
      } else if (gesture.event >= TOUCH_SWIPE_LEFT && gesture.event <= TOUCH_SWIPE_DOWN) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;
    
    case SCREEN_CHARACTER_STATS:
      // Handle taps and horizontal swipes only (up goes to progression)
      if (gesture.event == TOUCH_TAP) {
        handleCurrentScreenTouch(gesture);
      } else if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_RIGHT) {
        handleSwipeNavigation(gesture.dx, gesture.dy);
      }
      break;
    
    case SCREEN_PROGRESSION:
      handleProgressionTouch(gesture);
      break;
    
    case SCREEN_STEPS_TRACKER:
      // Handle both taps (for steps card interaction) AND swipes (for navigation)
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
      handleQuestTouch(gesture);
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
      handleSettingsTouch(gesture);
      break;
    
    case SCREEN_THEME_SELECTOR:
      handleThemeSelectorTouch(gesture);
      break;
    
    case SCREEN_MUSIC:
      handleMusicTouch(gesture);
      break;
    
    case SCREEN_WIFI_MANAGER:
      handleWifiManagerTouch(gesture);
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
      handleFileBrowserTouch(gesture);
      break;
    
    case SCREEN_CALCULATOR:
      handleCalculatorTouch(gesture);
      break;
    
    case SCREEN_FLASHLIGHT:
      handleFlashlightTouch(gesture);
      break;
    
    case SCREEN_TIMER:
      handleTimerTouch(gesture);
      break;
    
    case SCREEN_CONVERTER:
      handleConverterTouch(gesture);
      break;
    
    case SCREEN_ACHIEVEMENTS:
      handleAchievementsTouch(gesture);
      break;
    
    case SCREEN_SHOP:
      handleShopTouch(gesture);
      break;
    
    case SCREEN_GALLERY:
      handleGalleryTouch(gesture);
      break;
    
    default:
      break;
  }
}

// =============================================================================
// SCREEN UPDATE
// =============================================================================

void updateCurrentScreen() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastTimerUpdate = 0;
  static unsigned long lastBatteryUpdate = 0;
  
  // Update battery status every 30 seconds
  if (millis() - lastBatteryUpdate > 30000) {
    lastBatteryUpdate = millis();
    updateBatteryStatus();
  }
  
  // Only update watchface time when on watchface
  if (system_state.current_screen == SCREEN_WATCHFACE && 
      navState.currentMain == MAIN_WATCHFACE &&
      millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    updateWatchFaceTime();
  }
  
  // Update timer display while running (every 50ms for smooth centiseconds)
  if (system_state.current_screen == SCREEN_TIMER &&
      millis() - lastTimerUpdate > 50) {
    lastTimerUpdate = millis();
    updateTimerDisplay();
  }
  
  // Update games
  if (system_state.current_screen == SCREEN_GAMES) {
    AdvancedGameManager::updateGame();
  }
}

// =============================================================================
// SAVE ALL DATA - Persistent Storage using Preferences (NVS)
// =============================================================================

#include <Preferences.h>
Preferences gamePrefs;

void saveAllGameData() {
  gamePrefs.begin("watchgame", false);  // Read-write mode
  
  // Save player stats
  gamePrefs.putInt("gems", system_state.player_gems);
  gamePrefs.putInt("level", system_state.player_level);
  gamePrefs.putInt("xp", system_state.player_xp);
  gamePrefs.putInt("theme", (int)system_state.current_theme);
  
  // Save game progress
  gamePrefs.putInt("cards", system_state.gacha_cards_collected);
  gamePrefs.putInt("bosses", system_state.bosses_defeated);
  gamePrefs.putInt("streak", system_state.training_streak);
  gamePrefs.putInt("logins", system_state.daily_login_count);
  gamePrefs.putInt("pity", system_state.pity_counter);
  gamePrefs.putInt("pityleg", system_state.pity_legendary_counter);
  
  // Save activity
  gamePrefs.putInt("steps", system_state.steps_today);
  
  // Save brightness setting
  gamePrefs.putInt("bright", system_state.brightness);
  
  gamePrefs.end();
  
  Serial.println("[SAVE] Game data saved to NVS");
  Serial.printf("       Gems: %d, Level: %d, XP: %d, Theme: %d\n", 
    system_state.player_gems, system_state.player_level, 
    system_state.player_xp, system_state.current_theme);
}

void loadAllGameData() {
  gamePrefs.begin("watchgame", true);  // Read-only mode
  
  // Load player stats with defaults
  system_state.player_gems = gamePrefs.getInt("gems", 500);  // Default 500 gems
  system_state.player_level = gamePrefs.getInt("level", 1);
  system_state.player_xp = gamePrefs.getInt("xp", 0);
  system_state.current_theme = (ThemeType)gamePrefs.getInt("theme", THEME_LUFFY_GEAR5);
  
  // Load game progress
  system_state.gacha_cards_collected = gamePrefs.getInt("cards", 0);
  system_state.bosses_defeated = gamePrefs.getInt("bosses", 0);
  system_state.training_streak = gamePrefs.getInt("streak", 0);
  system_state.daily_login_count = gamePrefs.getInt("logins", 0);
  system_state.pity_counter = gamePrefs.getInt("pity", 0);
  system_state.pity_legendary_counter = gamePrefs.getInt("pityleg", 0);
  
  // Load activity
  system_state.steps_today = gamePrefs.getInt("steps", 0);
  
  // Load brightness
  system_state.brightness = gamePrefs.getInt("bright", 200);
  
  gamePrefs.end();
  
  // Apply loaded theme
  setTheme(system_state.current_theme);
  
  Serial.println("[LOAD] Game data loaded from NVS");
  Serial.printf("       Gems: %d, Level: %d, XP: %d, Theme: %d\n", 
    system_state.player_gems, system_state.player_level, 
    system_state.player_xp, system_state.current_theme);
}

void saveAllData() {
  saveAllGameData();
  saveGachaProgress();
  saveGameProgress();
  saveTrainingProgress();
  saveBossProgress();
  saveRPGProgress();
  saveStepsData();
  Serial.println("[SAVE] All data saved");
}

// =============================================================================
// DUAL CORE MODE - Switch to Widget OS
// =============================================================================

// Animation state
bool dualCoreAnimActive = false;
unsigned long dualCoreAnimStart = 0;

void drawDualCoreScreen() {
    gfx->fillScreen(0x0000);
    
    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    
    // Title
    gfx->setTextColor(gfx->color565(255, 220, 100));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 95, 35);
    gfx->print("DUAL CORE MODE");
    
    // Decorative lines
    for (int i = 0; i < 5; i++) {
        uint16_t c = gfx->color565(255 - i*30, 200 - i*20, 50 + i*10);
        gfx->drawLine(30 + i*15, 70 + i*3, LCD_WIDTH - 30 - i*15, 70 + i*3, c);
    }
    
    int coreY = centerY - 50;
    
    // Fusion core (orange)
    for (int r = 70; r > 25; r -= 6) {
        uint8_t i = 255 - (70 - r) * 3;
        gfx->drawCircle(centerX - 90, coreY, r, gfx->color565(i, i/2, 0));
    }
    gfx->fillCircle(centerX - 90, coreY, 30, gfx->color565(255, 180, 80));
    gfx->setTextColor(gfx->color565(100, 50, 0));
    gfx->setTextSize(3);
    gfx->setCursor(centerX - 100, coreY - 12);
    gfx->print("F");
    gfx->setTextSize(1);
    gfx->setTextColor(gfx->color565(255, 200, 100));
    gfx->setCursor(centerX - 105, coreY + 50);
    gfx->print("FUSION");
    gfx->setTextColor(gfx->color565(100, 255, 100));
    gfx->setCursor(centerX - 108, coreY + 62);
    gfx->print("(Current)");
    
    // Connection
    for (int i = -2; i <= 2; i++) {
        gfx->drawLine(centerX - 50, coreY + i, centerX + 50, coreY + i, gfx->color565(150, 150, 255));
    }
    
    // Widget core (blue)
    for (int r = 70; r > 25; r -= 6) {
        uint8_t i = 255 - (70 - r) * 3;
        gfx->drawCircle(centerX + 90, coreY, r, gfx->color565(0, i/2, i));
    }
    gfx->fillCircle(centerX + 90, coreY, 30, gfx->color565(80, 180, 255));
    gfx->setTextColor(gfx->color565(0, 50, 100));
    gfx->setTextSize(3);
    gfx->setCursor(centerX + 77, coreY - 12);
    gfx->print("W");
    gfx->setTextSize(1);
    gfx->setTextColor(gfx->color565(100, 200, 255));
    gfx->setCursor(centerX + 73, coreY + 50);
    gfx->print("WIDGET");
    gfx->setTextColor(gfx->color565(150, 150, 150));
    gfx->setCursor(centerX + 75, coreY + 62);
    gfx->print("(Target)");
    
    // Golden button
    int btnX = centerX - 80, btnY = LCD_HEIGHT - 140, btnW = 160, btnH = 55;
    gfx->fillRoundRect(btnX + 3, btnY + 3, btnW, btnH, 10, gfx->color565(80, 60, 0));
    for (int i = 0; i < btnH; i++) {
        float r = (float)i / btnH;
        gfx->drawLine(btnX, btnY + i, btnX + btnW, btnY + i, 
            gfx->color565(255, 180 + r*60, r*50));
    }
    gfx->drawRoundRect(btnX, btnY, btnW, btnH, 10, gfx->color565(150, 120, 0));
    gfx->drawRoundRect(btnX+1, btnY+1, btnW-2, btnH-2, 9, gfx->color565(255, 220, 100));
    gfx->setTextColor(gfx->color565(60, 40, 0));
    gfx->setTextSize(2);
    gfx->setCursor(btnX + 22, btnY + 18);
    gfx->print("ACTIVATE");
    
    gfx->setTextColor(gfx->color565(100, 255, 100));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 85, LCD_HEIGHT - 70);
    gfx->print("Tap to switch to Widget OS");
    
    gfx->setTextColor(gfx->color565(120, 120, 120));
    gfx->setCursor(15, LCD_HEIGHT - 25);
    gfx->print("< Swipe to return");
}

bool handleDualCoreTouch(int x, int y) {
    int centerX = LCD_WIDTH / 2;
    int btnX = centerX - 80, btnY = LCD_HEIGHT - 140, btnW = 160, btnH = 55;
    
    if (x >= btnX && x <= btnX + btnW && y >= btnY && y <= btnY + btnH) {
        Serial.println("[DUAL_CORE] ACTIVATE pressed!");
        dualCoreAnimActive = true;
        dualCoreAnimStart = millis();
        return true;
    }
    return false;
}

void drawSwitchAnimation(float progress) {
    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    uint8_t frame = (uint8_t)(progress * 255);
    
    if (progress < 0.25f) {
        // Phase 1: Energy gathering
        gfx->fillScreen(0x0000);
        float p = progress / 0.25f;
        for (int i = 0; i < 16; i++) {
            float angle = (i * 22.5f + frame * 2) * PI / 180.0f;
            float dist = 250 * (1.0f - p);
            gfx->drawLine(centerX + cos(angle) * dist, centerY + sin(angle) * dist,
                         centerX + cos(angle) * 40, centerY + sin(angle) * 40,
                         gfx->color565(255, 150 + i*6, 50));
        }
        int sz = 20 + p * 40;
        for (int r = sz; r > 5; r -= 4) {
            gfx->drawCircle(centerX, centerY, r, gfx->color565(255 * r / sz, 128 * r / sz, 0));
        }
        gfx->fillCircle(centerX, centerY, 15, gfx->color565(255, 220, 100));
        gfx->setTextColor(gfx->color565(255, 200, 100));
        gfx->setTextSize(2);
        gfx->setCursor(centerX - 70, 40);
        gfx->print("CHARGING...");
    } else if (progress < 0.5f) {
        // Phase 2: Aura expansion
        gfx->fillScreen(0x0000);
        float p = (progress - 0.25f) / 0.25f;
        for (int ring = 0; ring < 6; ring++) {
            float rd = 40 + p * 180 + ring * 25;
            if (rd < 280) {
                uint8_t a = 255 - ring * 35;
                for (int t = 0; t < 4; t++) {
                    gfx->drawCircle(centerX, centerY, rd + t, gfx->color565(a, a/2, a/4));
                }
            }
        }
        int es = 30 + p * 60;
        gfx->fillCircle(centerX, centerY, es, gfx->color565(255, 200, 100));
        gfx->fillCircle(centerX, centerY, es * 0.7, gfx->color565(255, 255, 200));
        if ((frame / 8) % 2 == 0) {
            gfx->setTextColor(gfx->color565(255, 220, 100));
            gfx->setTextSize(2);
            gfx->setCursor(centerX - 115, 35);
            gfx->print("DUAL CORE ACTIVATE!");
        }
    } else if (progress < 0.75f) {
        // Phase 3: Fracture
        float p = (progress - 0.5f) / 0.25f;
        uint8_t fl = 255 * (1.0f - p);
        gfx->fillScreen(gfx->color565(fl, fl, fl));
        for (int i = 0; i < 10; i++) {
            float angle = (i * 36) * PI / 180.0f;
            float len = p * 300;
            for (int t = -2; t <= 2; t++) {
                gfx->drawLine(centerX, centerY + t, 
                    centerX + cos(angle) * len, centerY + sin(angle) * len + t, 0x0000);
            }
        }
        gfx->fillCircle(centerX, centerY, 15 + p * 80, 0x0000);
        gfx->setTextColor(0x0000);
        gfx->setTextSize(2);
        gfx->setCursor(centerX - 65, 40);
        gfx->print("SWITCHING...");
    } else {
        // Phase 4: Fade to Widget
        float p = (progress - 0.75f) / 0.25f;
        gfx->fillScreen(gfx->color565(50 * (1-p), 100 + 100*p, 180 + 75*p));
        if (p > 0.3f) {
            uint8_t li = ((p - 0.3f) / 0.7f) * 255;
            gfx->setTextColor(gfx->color565(li, li, li));
            gfx->setTextSize(5);
            gfx->setCursor(centerX - 30, centerY - 60);
            gfx->print("W");
        }
        gfx->setTextColor(0xFFFF);
        gfx->setTextSize(2);
        gfx->setCursor(centerX - 60, centerY + 30);
        gfx->print("LOADING...");
        int barW = 220, barX = centerX - 110, barY = centerY + 70;
        gfx->drawRect(barX, barY, barW, 12, 0xFFFF);
        gfx->fillRect(barX + 2, barY + 2, (barW - 4) * p, 8, gfx->color565(100, 200, 255));
        if (p > 0.6f) {
            uint8_t ti = ((p - 0.6f) / 0.4f) * 255;
            gfx->setTextColor(gfx->color565(ti, ti, ti));
            gfx->setTextSize(3);
            gfx->setCursor(centerX - 90, centerY + 100);
            gfx->print("WIDGET OS");
        }
    }
}

bool updateDualCoreAnimation() {
    if (!dualCoreAnimActive) return false;
    
    float progress = (float)(millis() - dualCoreAnimStart) / 3000.0f;
    
    if (progress >= 1.0f) {
        dualCoreAnimActive = false;
        extern void switchToWidgetOS();
        switchToWidgetOS();
        return false;
    }
    
    drawSwitchAnimation(progress);
    return true;
}

