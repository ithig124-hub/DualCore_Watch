/*
 * ui.cpp - User Interface Implementation (FIXED)
 * Screen management, navigation, and UI components
 */

#include "ui.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "games.h"
#include "apps.h"
#include "wifi_apps.h"
#include "daily_quests.h"  // FUSION OS: For drawDailyQuestsScreen()
#include "hardware.h"
#include "navigation.h"
#include "sd_preferences.h"
#include "dual_boot_manager.h"

// Forward declarations
void drawSplashScreen();

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Screen history stack
static ScreenType screen_history[10];
static int history_index = 0;

// =============================================================================
// UI INITIALIZATION
// =============================================================================

void initializeUI() {
  Serial.println("[UI] Initializing user interface...");
  initializeNavigation();
}

void ui_init() {
  initializeUI();
}

// =============================================================================
// SCREEN MANAGEMENT
// =============================================================================

void setCurrentScreen(ScreenType screen) {
  if (system_state.current_screen != screen) {
    pushScreen(system_state.current_screen);
  }
  system_state.current_screen = screen;
}

ScreenType getCurrentScreen() {
  return system_state.current_screen;
}

// drawCurrentScreen() is defined in navigation.cpp for swipe navigation
// This version handles app-specific screens when navigating to apps
void drawAppScreen() {
  switch(system_state.current_screen) {
    case SCREEN_SPLASH:
      drawSplashScreen();
      break;
    case SCREEN_WATCHFACE:
      drawWatchFace();
      break;
    case SCREEN_APP_GRID:
      drawAppGrid();
      break;
    case SCREEN_MUSIC:
      drawMusicApp();
      break;
    case SCREEN_NOTES:
      drawNotesApp();
      break;
    case SCREEN_QUESTS:
      drawDailyQuestsScreen();  // Fixed: correct function name
      break;
    case SCREEN_SETTINGS:
      drawSettingsApp();
      break;
    case SCREEN_GAMES:
      drawGameMenu();
      break;
    case SCREEN_WIFI_SETUP:         // FIXED: Now defined in config.h
      showWiFiSetupScreen();
      break;
    case SCREEN_WIFI_MANAGER:       // Handle both screen types
      showWiFiSetupScreen();
      break;
    case SCREEN_NETWORK_STATUS:     // FIXED: Now defined in config.h
      showNetworkStatusScreen();
      break;
    case SCREEN_WEATHER_APP:
      drawWeatherApp();
      break;
    case SCREEN_NEWS_APP:           // FIXED: Now defined in config.h
      drawNewsApp();
      break;
    case SCREEN_WALLPAPER_SELECTOR:
      drawWallpaperSelector();
      break;
    case SCREEN_CALCULATOR:
      drawCalculatorApp();
      break;
    case SCREEN_FLASHLIGHT:
      drawFlashlightApp();
      break;
    case SCREEN_FILE_BROWSER:
      drawFileBrowserApp();
      break;
    default:
      drawWatchFace();
      break;
  }
}

void drawSplashScreen() {
  gfx->fillScreen(0x0000);  // Pure AMOLED black
  
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  // Subtle accent ring
  for (int r = 80; r > 70; r--) {
    gfx->drawCircle(centerX, centerY - 30, r, RGB565(20, 20, 30));
  }
  
  // Title
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(centerX - 78, centerY - 55);
  gfx->print("ANIME");
  
  gfx->setTextColor(RGB565(255, 200, 60));
  gfx->setCursor(centerX - 78, centerY - 20);
  gfx->print("WATCH");
  
  // Subtitle
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 52, centerY + 20);
  gfx->print("Improved Edition");
  
  // Loading bar
  int barX = centerX - 70;
  int barY = centerY + 55;
  int barW = 140;
  int barH = 6;
  gfx->drawRect(barX, barY, barW, barH, RGB565(40, 42, 55));
  
  for (int i = 0; i <= barW - 4; i += 3) {
    gfx->fillRect(barX + 2, barY + 1, i, barH - 2, RGB565(100, 200, 150));
    delay(12);
  }
  
  // Feature list
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 82, 95));
  gfx->setCursor(centerX - 55, centerY + 80);
  gfx->print("11 Character Themes");
  gfx->setCursor(centerX - 50, centerY + 98);
  gfx->print("Gacha + Mini-Games");
}

void drawAboutScreen() {
  gfx->fillScreen(RGB565(8, 8, 12));
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 55, RGB565(16, 18, 24));
  gfx->drawFastHLine(0, 55, LCD_WIDTH, theme->primary);
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 30, 18);
  gfx->print("About");
  
  // Info card
  gfx->fillRect(20, 70, LCD_WIDTH - 40, 110, RGB565(22, 24, 32));
  gfx->drawRect(20, 70, LCD_WIDTH - 40, 110, RGB565(50, 52, 65));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(38, 85);
  gfx->print("Anime Watch");
  
  gfx->setTextSize(1);
  gfx->setTextColor(theme->accent);
  gfx->setCursor(38, 112);
  gfx->print("Version 2.0 + Dual Boot");
  
  gfx->setTextColor(RGB565(120, 120, 130));
  gfx->setCursor(38, 132);
  gfx->print("11 Themes | Gacha | Boss Rush");
  gfx->setCursor(38, 150);
  gfx->print("Data saved to SD: /WATCH/FusionData/");
  
  // Dual Boot info card
  gfx->fillRect(20, 195, LCD_WIDTH - 40, 75, RGB565(22, 24, 32));
  gfx->drawRect(20, 195, LCD_WIDTH - 40, 75, RGB565(50, 52, 65));
  
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  gfx->setCursor(38, 208);
  gfx->print("Dual Boot System");
  
  gfx->setTextColor(RGB565(160, 160, 170));
  gfx->setCursor(38, 228);
  gfx->printf("Running: %s", DualBoot.getCurrentOSName());
  gfx->setCursor(38, 248);
  gfx->printf("Switch to: %s (2x tap BOOT)", DualBoot.getOtherOSName());
  
  // Hardware card
  gfx->fillRect(20, 285, LCD_WIDTH - 40, 65, RGB565(22, 24, 32));
  gfx->drawRect(20, 285, LCD_WIDTH - 40, 65, RGB565(50, 52, 65));
  
  gfx->setTextColor(theme->primary);
  gfx->setCursor(38, 298);
  gfx->print("Hardware");
  
  gfx->setTextColor(RGB565(160, 160, 170));
  gfx->setCursor(38, 318);
  gfx->print("ESP32-S3 | CO5300 AMOLED | 410x502");
  gfx->setCursor(38, 335);
  gfx->print("AXP2101 | WiFi | SD Card | IMU");
  
  // Reset Data button
  int btnX = 20;
  int btnY = 365;
  int btnW = LCD_WIDTH - 40;
  int btnH = 42;
  gfx->fillRect(btnX, btnY, btnW, btnH, RGB565(80, 20, 20));
  gfx->drawRect(btnX, btnY, btnW, btnH, RGB565(200, 60, 60));
  gfx->setTextColor(RGB565(255, 100, 100));
  gfx->setTextSize(2);
  gfx->setCursor(btnX + btnW/2 - 72, btnY + 12);
  gfx->print("Reset Data");
  
  drawSwipeIndicator();
}

// Handle touch on About screen - check for Reset Data button tap
void handleAboutTouch(int touchX, int touchY) {
  int btnX = 20;
  int btnY = 365;
  int btnW = LCD_WIDTH - 40;
  int btnH = 42;
  
  if (touchX >= btnX && touchX <= btnX + btnW &&
      touchY >= btnY && touchY <= btnY + btnH) {
    
    // Confirm screen
    gfx->fillScreen(RGB565(8, 8, 12));
    gfx->setTextColor(RGB565(255, 80, 80));
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 90, LCD_HEIGHT/2 - 40);
    gfx->print("Resetting...");
    
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(160, 160, 170));
    gfx->setCursor(LCD_WIDTH/2 - 100, LCD_HEIGHT/2);
    gfx->print("Clearing /WATCH/FusionData/");
    
    // Delete all FusionData files
    SDPreferences::resetAllFusionData();
    
    delay(1500);
    
    gfx->setTextColor(RGB565(80, 200, 80));
    gfx->setCursor(LCD_WIDTH/2 - 50, LCD_HEIGHT/2 + 30);
    gfx->print("Done! Rebooting...");
    
    delay(1000);
    esp_restart();
  }
}

// =============================================================================
// COMPONENT DRAWING
// =============================================================================

void drawButton(UIComponent& button) {
  uint16_t bg = button.pressed ? getCurrentTheme()->accent : button.color;
  gfx->fillRect(button.x, button.y, button.width, button.height, bg);
  gfx->drawRect(button.x, button.y, button.width, button.height, COLOR_WHITE);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  int textW = button.text.length() * 6;
  gfx->setCursor(button.x + (button.width - textW) / 2, button.y + (button.height - 8) / 2);
  gfx->print(button.text);
}

void drawLabel(UIComponent& label) {
  gfx->setTextColor(label.color);
  gfx->setCursor(label.x, label.y);
  gfx->print(label.text);
}

void drawSlider(UIComponent& slider) {
  // Track
  gfx->fillRect(slider.x, slider.y + slider.height/2 - 3, slider.width, 6, RGB565(60, 60, 60));
  
  // Fill based on value (stored in color as percentage)
  int fillW = (slider.width * slider.color) / 100;
  gfx->fillRect(slider.x, slider.y + slider.height/2 - 3, fillW, 6, getCurrentTheme()->primary);
  
  // Knob
  int knobX = slider.x + fillW;
  gfx->fillCircle(knobX, slider.y + slider.height/2, 10, COLOR_WHITE);
}

void drawToggle(UIComponent& toggle) {
  bool on = toggle.pressed;
  uint16_t bg = on ? getCurrentTheme()->primary : RGB565(60, 60, 60);
  
  gfx->fillRect(toggle.x, toggle.y, toggle.width, toggle.height, bg);
  
  int knobX = on ? toggle.x + toggle.width - toggle.height/2 : toggle.x + toggle.height/2;
  gfx->fillCircle(knobX, toggle.y + toggle.height/2, toggle.height/2 - 4, COLOR_WHITE);
}

// =============================================================================
// TOUCH HANDLING
// =============================================================================

void handleUITouch(TouchGesture& gesture) {
  handleNavigation(gesture);
}

bool isTouchInComponent(TouchGesture& gesture, UIComponent& component) {
  return gesture.x >= component.x && gesture.x < component.x + component.width &&
         gesture.y >= component.y && gesture.y < component.y + component.height;
}

void handleButtonPress(UIComponent& button) {
  button.pressed = true;
  if (button.callback) button.callback();
}

// =============================================================================
// ANIMATION
// =============================================================================

void animateScreenTransition(ScreenType from, ScreenType to) {
  fadeOut(100);
  setCurrentScreen(to);
  drawCurrentScreen();
  fadeIn(100);
}

void animateButtonPress(UIComponent& button) {
  button.pressed = true;
  drawButton(button);
  delay(100);
  button.pressed = false;
  drawButton(button);
}

void animateSlideIn(int direction) {
  // Simple slide animation
}

// =============================================================================
// UI ELEMENTS
// =============================================================================

void drawDigitalCrown(int x, int y, int value) {
  gfx->drawCircle(x, y, 20, getCurrentTheme()->primary);
  gfx->drawCircle(x, y, 18, getCurrentTheme()->primary);
  
  // Value indicator
  float angle = (value % 100) * 3.6 * PI / 180;
  int indicatorX = x + sin(angle) * 15;
  int indicatorY = y - cos(angle) * 15;
  gfx->fillCircle(indicatorX, indicatorY, 3, getCurrentTheme()->accent);
}

void drawNavigationBar(const char* title, bool back_button) {
  gfx->fillRect(0, 0, LCD_WIDTH, 50, RGB565(20, 20, 20));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(back_button ? 60 : 20, 15);
  gfx->print(title);
  
  if (back_button) {
    gfx->setTextColor(getCurrentTheme()->primary);
    gfx->setCursor(15, 15);
    gfx->print("<");
  }
}

void drawStatusBar() {
  // Time
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(10, 5);
  gfx->print(timeStr);
  
  // Battery
  int bx = LCD_WIDTH - 35;
  gfx->drawRect(bx, 5, 25, 12, COLOR_WHITE);
  gfx->fillRect(bx + 25, 8, 2, 6, COLOR_WHITE);
  int fillW = (system_state.battery_percentage * 21) / 100;
  uint16_t color = system_state.battery_percentage > 20 ? COLOR_GREEN : COLOR_RED;
  gfx->fillRect(bx + 2, 7, fillW, 8, color);
  
  // WiFi indicator
  if (system_state.wifi_connected) {
    gfx->fillCircle(LCD_WIDTH - 50, 10, 5, COLOR_CYAN);
  }
}

void drawNotificationDot(int x, int y, uint16_t color) {
  gfx->fillCircle(x, y, 5, color);
}

// =============================================================================
// LIST VIEW
// =============================================================================

void drawListItem(int x, int y, int width, const char* title, const char* subtitle, uint16_t color) {
  gfx->fillRect(x, y, width, 50, RGB565(40, 40, 40));
  
  gfx->setTextColor(color);
  gfx->setTextSize(1);
  gfx->setCursor(x + 10, y + 10);
  gfx->print(title);
  
  gfx->setTextColor(COLOR_GRAY);
  gfx->setCursor(x + 10, y + 30);
  gfx->print(subtitle);
  
  // Chevron
  gfx->setTextColor(COLOR_GRAY);
  gfx->setCursor(x + width - 20, y + 18);
  gfx->print(">");
}

void handleListScroll(TouchGesture& gesture, int& scroll_offset) {
  if (gesture.event == TOUCH_SWIPE_UP) {
    scroll_offset += 50;
  } else if (gesture.event == TOUCH_SWIPE_DOWN) {
    scroll_offset -= 50;
    if (scroll_offset < 0) scroll_offset = 0;
  }
}

// =============================================================================
// MODAL DIALOGS
// =============================================================================

void showAlert(const char* title, const char* message) {
  int w = 280, h = 150;
  int x = (LCD_WIDTH - w) / 2;
  int y = (LCD_HEIGHT - h) / 2;
  
  gfx->fillRect(x, y, w, h, RGB565(30, 30, 30));
  gfx->drawRect(x, y, w, h, getCurrentTheme()->primary);
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(x + 20, y + 20);
  gfx->print(title);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 20, y + 60);
  gfx->print(message);
  
  drawThemeButton(x + (w - 80) / 2, y + h - 50, 80, 35, "OK", false);
}

void showConfirm(const char* title, const char* message, void (*yes_callback)(void)) {
  int w = 280, h = 180;
  int x = (LCD_WIDTH - w) / 2;
  int y = (LCD_HEIGHT - h) / 2;
  
  gfx->fillRect(x, y, w, h, RGB565(30, 30, 30));
  gfx->drawRect(x, y, w, h, getCurrentTheme()->primary);
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(x + 20, y + 20);
  gfx->print(title);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 20, y + 60);
  gfx->print(message);
  
  drawThemeButton(x + 30, y + h - 50, 100, 35, "Yes", false);
  drawThemeButton(x + 150, y + h - 50, 100, 35, "No", false);
}

// =============================================================================
// LOADING INDICATORS
// =============================================================================

void showLoadingSpinner(const char* message) {
  int x = LCD_WIDTH / 2;
  int y = LCD_HEIGHT / 2;
  
  gfx->fillCircle(x, y - 30, 25, getCurrentTheme()->primary);
  gfx->fillCircle(x, y - 30, 20, COLOR_BLACK);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  int textW = strlen(message) * 6;
  gfx->setCursor(x - textW / 2, y + 20);
  gfx->print(message);
}

void hideLoadingSpinner() {
  // Redraw current screen
  drawCurrentScreen();
}

void updateLoadingProgress(int percentage) {
  int x = (LCD_WIDTH - 200) / 2;
  int y = LCD_HEIGHT / 2 + 40;
  
  gfx->fillRect(x, y, 200, 20, RGB565(60, 60, 60));
  
  int fillW = (percentage * 196) / 100;
  if (fillW > 0) {
    gfx->fillRect(x + 2, y + 2, fillW, 16, getCurrentTheme()->primary);
  }
}

// =============================================================================
// COMPLICATIONS
// =============================================================================

void drawComplicationSlot(int x, int y, int w, int h, const char* data, uint16_t color) {
  gfx->fillRect(x, y, w, h, RGB565(30, 30, 30));
  gfx->setTextColor(color);
  gfx->setTextSize(1);
  gfx->setCursor(x + 5, y + h / 2 - 4);
  gfx->print(data);
}

void updateComplications() {
  // Update watch face complications with current data
}

// =============================================================================
// NAVIGATION
// =============================================================================

void initializeNavigation() {
  history_index = 0;
}

void handleNavigation(TouchGesture& gesture) {
  handleSwipeNavigation(gesture);
}

void navigateBack() {
  ScreenType prev = popScreen();
  if (prev != system_state.current_screen) {
    system_state.current_screen = prev;
  }
}

void navigateHome() {
  clearScreenHistory();
  system_state.current_screen = SCREEN_WATCHFACE;
}

void navigateToScreen(ScreenType screen) {
  setCurrentScreen(screen);
}

void pushScreen(ScreenType screen) {
  if (history_index < 9) {
    screen_history[history_index++] = screen;
  }
}

ScreenType popScreen() {
  if (history_index > 0) {
    return screen_history[--history_index];
  }
  return SCREEN_WATCHFACE;
}

void clearScreenHistory() {
  history_index = 0;
}

void handleSwipeNavigation(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_RIGHT) {
    navigateBack();
  } else if (gesture.event == TOUCH_SWIPE_LEFT && system_state.current_screen == SCREEN_WATCHFACE) {
    setCurrentScreen(SCREEN_APP_GRID);
  } else if (gesture.event == TOUCH_SWIPE_UP && system_state.current_screen == SCREEN_WATCHFACE) {
    setCurrentScreen(SCREEN_QUESTS);
  } else if (gesture.event == TOUCH_SWIPE_DOWN && system_state.current_screen == SCREEN_WATCHFACE) {
    setCurrentScreen(SCREEN_SETTINGS);
  }
}

void showNextScreen() {
  // Cycle to next screen
}

void showPreviousScreen() {
  navigateBack();
}
