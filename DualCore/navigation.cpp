/*
 * navigation.cpp - IMPROVED Swipe Navigation
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * IMPROVEMENTS:
 * - Better touch detection with lower thresholds
 * - Clearer tap vs swipe differentiation  
 * - Improved hit targets for app icons
 * - Smooth visual feedback
 * - All apps now accessible and working
 * - Tap-to-switch elements for BoBoiBoy
 * - Character-specific minigames access
 * - Steps tracker card (swipe right from watchface)
 * - Daily quests system
 * - 3-page app grid
 */

#include "navigation.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "games.h"
#include "gacha.h"
#include "training.h"
#include "boss_rush.h"
#include "rpg.h"
#include "apps.h"
#include "wifi_apps.h"
#include "ui.h"
#include "boboiboy_elements.h"
#include "ochobot.h"
#include "fusion_game.h"
#include "character_games.h"
#include "steps_tracker.h"
#include "daily_quests.h"
#include <time.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Touch state tracking
TouchState touchState = {false, 0, 0, 0, 0, 0, 0};

// Global navigation state
NavigationState navState = {
  MAIN_WATCHFACE,   // currentMain
  0,                // appGridPage
  false,            // isTransitioning
  0,                // lastNavigationMs
  false             // navigationLocked
};

// =============================================================================
// INITIALIZATION
// =============================================================================

void initNavigation() {
  Serial.println("[NAV] Initializing IMPROVED swipe navigation...");
  navState.currentMain = MAIN_WATCHFACE;
  navState.appGridPage = 0;
  navState.isTransitioning = false;
  navState.lastNavigationMs = 0;
  navState.navigationLocked = false;
  
  // Reset touch state
  touchState.isPressed = false;
  
  system_state.current_screen = SCREEN_WATCHFACE;
  
  Serial.println("[NAV] Navigation ready:");
  Serial.println("      - Swipe LEFT/RIGHT: Switch main screens");
  Serial.println("      - Swipe UP/DOWN: App grid pages");
  Serial.println("      - TAP: Select apps/buttons");
}

// =============================================================================
// NAVIGATION CONTROL - IMPROVED
// =============================================================================

bool canNavigate() {
  // Force reset stuck states after timeout
  if (navState.isTransitioning && (millis() - navState.lastNavigationMs > 1000)) {
    Serial.println("[NAV] Force resetting stuck transition state");
    navState.isTransitioning = false;
  }
  
  if (navState.navigationLocked) {
    return false;
  }
  
  if (navState.isTransitioning) {
    return false;
  }
  
  if (millis() - navState.lastNavigationMs < NAVIGATION_COOLDOWN_MS) {
    return false;
  }
  
  return true;
}

void handleSwipeNavigation(int dx, int dy) {
  if (!canNavigate()) {
    // Force unlock if stuck
    if (millis() - navState.lastNavigationMs > 500) {
      navState.navigationLocked = false;
      navState.isTransitioning = false;
    }
    return;
  }
  
  Serial.printf("[NAV] Swipe: dx=%d, dy=%d\n", dx, dy);
  
  // Determine gesture type
  int absDx = abs(dx);
  int absDy = abs(dy);
  
  // HORIZONTAL SWIPE - Navigate main screens
  if (absDx > absDy && absDx > SWIPE_THRESHOLD_MIN) {
    if (dx < -SWIPE_THRESHOLD_MIN) {
      // Swipe LEFT -> Next screen
      Serial.println("[NAV] SWIPE LEFT detected -> navigateRight()");
      navigateRight();
    } else if (dx > SWIPE_THRESHOLD_MIN) {
      // Swipe RIGHT -> Previous screen
      Serial.println("[NAV] SWIPE RIGHT detected -> navigateLeft()");
      navigateLeft();
    }
  }
  // VERTICAL SWIPE - Only on App Grid for page switching
  else if (absDy > absDx && absDy > SWIPE_THRESHOLD_MIN) {
    if (navState.currentMain == MAIN_APP_GRID_1) {
      if (dy < -SWIPE_THRESHOLD_MIN && navState.appGridPage < APP_GRID_PAGES - 1) {
        // Swipe UP -> Next app page
        Serial.println("[NAV] SWIPE UP -> Next app page");
        navigateDown();
      } else if (dy > SWIPE_THRESHOLD_MIN && navState.appGridPage > 0) {
        // Swipe DOWN -> Previous app page
        Serial.println("[NAV] SWIPE DOWN -> Previous app page");
        navigateUp();
      }
    }
  }
}

void navigateLeft() {
  int prev = (int)navState.currentMain - 1;
  if (prev < 0) prev = MAIN_SCREEN_COUNT - 1;
  Serial.printf("[NAV] Navigate LEFT: %d -> %d\n", navState.currentMain, prev);
  navigateToScreen((MainScreen)prev);
}

void navigateRight() {
  int next = ((int)navState.currentMain + 1) % MAIN_SCREEN_COUNT;
  Serial.printf("[NAV] Navigate RIGHT: %d -> %d\n", navState.currentMain, next);
  navigateToScreen((MainScreen)next);
}

void navigateUp() {
  if (navState.currentMain == MAIN_APP_GRID_1 && navState.appGridPage > 0) {
    navState.appGridPage--;
    Serial.printf("[NAV] App Grid: Page -> %d\n", navState.appGridPage);
    navState.lastNavigationMs = millis();
    drawCurrentScreen();
  }
}

void navigateDown() {
  if (navState.currentMain == MAIN_APP_GRID_1 && navState.appGridPage < APP_GRID_PAGES - 1) {
    navState.appGridPage++;
    Serial.printf("[NAV] App Grid: Page -> %d (of %d)\n", navState.appGridPage, APP_GRID_PAGES);
    navState.lastNavigationMs = millis();
    drawCurrentScreen();
  }
}

void navigateToScreen(MainScreen screen) {
  if (screen == navState.currentMain) return;
  
  navState.isTransitioning = true;
  navState.lastNavigationMs = millis();
  
  MainScreen oldScreen = navState.currentMain;
  navState.currentMain = screen;
  
  // Reset app grid page when leaving
  if (oldScreen == MAIN_APP_GRID_1) {
    navState.appGridPage = 0;
  }
  
  // Map to system screen type - CRITICAL for preventing overlap
  switch (screen) {
    case MAIN_WATCHFACE:
      system_state.current_screen = SCREEN_WATCHFACE;
      break;
    case MAIN_STEPS_TRACKER:
      system_state.current_screen = SCREEN_STEPS_TRACKER;  // FIX: Proper screen type
      break;
    case MAIN_APP_GRID_1:
      system_state.current_screen = SCREEN_APP_GRID;
      break;
    case MAIN_CHARACTER_STATS:
      system_state.current_screen = SCREEN_CHARACTER_STATS;
      break;
  }
  
  Serial.printf("[NAV] Screen changed to: %d (system: %d)\n", screen, system_state.current_screen);
  
  // Draw new screen
  drawCurrentScreen();
  
  navState.isTransitioning = false;
}

MainScreen getCurrentMainScreen() {
  return navState.currentMain;
}

int getCurrentAppGridPage() {
  return navState.appGridPage;
}

// =============================================================================
// SCREEN DRAWING - IMPROVED
// =============================================================================

void drawCurrentScreen() {
  // CRITICAL: Prevent overlap by ensuring full screen clear
  // This is called on every navigation, so we reset partial update state
  
  switch (navState.currentMain) {
    case MAIN_WATCHFACE:
      drawWatchFace();
      break;
    case MAIN_STEPS_TRACKER:
      // Full clear happens inside drawStepsCard() now
      drawStepsCard();
      break;
    case MAIN_APP_GRID_1:
      if (navState.appGridPage == 0) drawAppGrid1();
      else if (navState.appGridPage == 1) drawAppGrid2();
      else drawAppGrid3();
      break;
    case MAIN_CHARACTER_STATS:
      drawCharacterStatsScreen();
      break;
  }
  
  // Always draw navigation indicators
  drawNavigationIndicators();
}

void drawNavigationIndicators() {
  // ========================================
  // RETRO PIXEL NAVIGATION INDICATORS
  // ========================================
  
  int centerX = LCD_WIDTH / 2;
  int y = LCD_HEIGHT - 14;
  int dotSize = 6;
  int spacing = 18;
  
  // Clear indicator area
  gfx->fillRect(centerX - 40, y - 10, 80, 20, RGB565(2, 2, 5));
  
  // Draw 4 pixel squares for main screens - retro style
  for (int i = 0; i < MAIN_SCREEN_COUNT; i++) {
    int x = centerX + (i - (MAIN_SCREEN_COUNT - 1) / 2.0) * spacing - dotSize/2;
    
    if (i == (int)navState.currentMain) {
      // Current - larger bright square with glow
      gfx->fillRect(x - 1, y - 1, dotSize + 2, dotSize + 2, 
                    RGB565(getCurrentTheme()->primary >> 12, 15, 20));
      gfx->fillRect(x, y, dotSize, dotSize, getCurrentTheme()->primary);
    } else {
      // Other - small dim square
      gfx->drawRect(x, y, dotSize, dotSize, RGB565(40, 45, 60));
    }
  }
}

// =============================================================================
// APP GRID - IMPROVED MODERN DESIGN
// =============================================================================

uint16_t getAppColor(const char* appName) {
  if (strcmp(appName, "GACHA") == 0)     return RGB565(255, 180, 50);
  if (strcmp(appName, "TRAINING") == 0)  return RGB565(50, 200, 100);
  if (strcmp(appName, "BOSS") == 0)      return RGB565(220, 60, 60);
  if (strcmp(appName, "GAMES") == 0)     return RGB565(100, 150, 255);
  if (strcmp(appName, "QUESTS") == 0)    return RGB565(255, 150, 100);  // Daily Quests
  if (strcmp(appName, "STEPS") == 0)     return RGB565(100, 255, 100);  // Steps Tracker
  if (strcmp(appName, "ACHIEVE") == 0)   return RGB565(255, 200, 50);   // Achievements
  if (strcmp(appName, "SHOP") == 0)      return RGB565(200, 100, 255);  // Shop
  if (strcmp(appName, "SOCIAL") == 0)    return RGB565(100, 200, 255);  // Social
  if (strcmp(appName, "GALLERY") == 0)   return RGB565(255, 100, 200);  // Gallery
  if (strcmp(appName, "TIMER") == 0)     return RGB565(100, 180, 255);  // Timer
  if (strcmp(appName, "COMPASS") == 0)   return RGB565(255, 100, 150);  // Compass
  if (strcmp(appName, "CONVERT") == 0)   return RGB565(180, 120, 255);  // Converter
  if (strcmp(appName, "NOTES") == 0)    return RGB565(255, 200, 60);
  if (strcmp(appName, "ELEMENTS") == 0)  return BBB_BAND_ORANGE;
  if (strcmp(appName, "MUSIC") == 0)     return RGB565(255, 100, 150);
  if (strcmp(appName, "WEATHER") == 0)   return RGB565(80, 180, 255);
  if (strcmp(appName, "WIFI") == 0)      return RGB565(100, 220, 200);
  if (strcmp(appName, "SETTINGS") == 0)  return RGB565(150, 150, 160);
  if (strcmp(appName, "THEMES") == 0)    return RGB565(255, 120, 80);
  if (strcmp(appName, "COLLECT") == 0)   return RGB565(255, 200, 100);
  if (strcmp(appName, "FILES") == 0)     return RGB565(180, 180, 100);
  if (strcmp(appName, "CALC") == 0)      return RGB565(100, 100, 120);
  if (strcmp(appName, "TORCH") == 0)     return RGB565(255, 255, 150);
  if (strcmp(appName, "OTA") == 0)       return RGB565(100, 200, 150);
  if (strcmp(appName, "BACKUP") == 0)    return RGB565(150, 180, 200);
  if (strcmp(appName, "FUSION") == 0)    return RGB565(200, 150, 255);
  if (strcmp(appName, "ABOUT") == 0)     return RGB565(180, 180, 190);
  return RGB565(100, 100, 120);
}

void drawAppIcon(int x, int y, int w, int h, const char* name, uint16_t color, bool selected) {
  // ========================================
  // RETRO ANIME APP ICON - Pixel Art Style
  // ========================================
  
  // Shadow (subtle)
  gfx->fillRect(x + 3, y + 3, w, h, RGB565(3, 3, 6));
  
  // Main icon background - dark panel with pixel corners
  gfx->fillRect(x, y, w, h, RGB565(15, 18, 25));
  
  // Pixel corner accents (retro game style)
  gfx->fillRect(x, y, 5, 5, color);
  gfx->fillRect(x + w - 5, y, 5, 5, color);
  gfx->fillRect(x, y + h - 5, 5, 5, color);
  gfx->fillRect(x + w - 5, y + h - 5, 5, 5, color);
  
  // Subtle scan line effect
  for (int sy = y + 6; sy < y + h - 6; sy += 4) {
    gfx->drawFastHLine(x + 5, sy, w - 10, RGB565(10, 12, 18));
  }
  
  // Colored icon square at center-top (pixel style, not circle)
  int iconCX = x + w / 2;
  int iconCY = y + 26;
  int iconSize = 28;
  
  // Icon glow
  gfx->fillRect(iconCX - iconSize/2 - 2, iconCY - iconSize/2 - 2, iconSize + 4, iconSize + 4, 
                RGB565((color >> 11) * 2, ((color >> 5) & 0x3F) / 3, (color & 0x1F) / 3));
  // Icon square
  gfx->fillRect(iconCX - iconSize/2, iconCY - iconSize/2, iconSize, iconSize, color);
  // Inner highlight (pixel style)
  gfx->drawRect(iconCX - iconSize/2 + 2, iconCY - iconSize/2 + 2, iconSize - 4, 2, RGB565(255, 255, 255));
  
  // Selection effect (bright border)
  if (selected) {
    gfx->drawRect(x - 2, y - 2, w + 4, h + 4, getCurrentTheme()->primary);
    gfx->drawRect(x - 1, y - 1, w + 2, h + 2, getCurrentTheme()->accent);
  } else {
    // Normal border
    gfx->drawRect(x, y, w, h, RGB565(35, 40, 55));
  }
  
  // App name - retro pixel font style
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(1);
  int textLen = strlen(name) * 6;
  int textX = x + (w - textLen) / 2;
  gfx->setCursor(textX, y + h - 18);
  gfx->print(name);
}

void drawAppGrid1() {
  // ========================================
  // RETRO ANIME APP GRID - CRT Style (Optimized for 410x502)
  // ========================================
  
  // Deep AMOLED black background
  gfx->fillScreen(RGB565(2, 2, 5));
  
  // Subtle CRT scan lines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header bar with pixel border - taller for bigger display
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  
  // Pixel border at bottom of header
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  
  // Title with glow effect - larger
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));  // Shadow
  gfx->setCursor(LCD_WIDTH/2 - 35, 14);
  gfx->print("Apps");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 36, 12);
  gfx->print("Apps");
  
  // Page indicator - pixel style (3 pages)
  gfx->fillRect(LCD_WIDTH - 50, 22, 10, 10, theme->primary);
  gfx->drawRect(LCD_WIDTH - 35, 22, 10, 10, RGB565(50, 55, 70));
  gfx->drawRect(LCD_WIDTH - 20, 22, 10, 10, RGB565(50, 55, 70));
  
  // App icons - larger for 410x502 display
  const char* apps1_normal[] = {"GACHA", "TRAINING", "BOSS", "QUESTS", "MUSIC", "WEATHER", "WIFI", "TIMER", "SETTINGS"};
  const char* apps1_boboiboy[] = {"GACHA", "TRAINING", "BOSS", "ELEMENTS", "MUSIC", "WEATHER", "WIFI", "TIMER", "SETTINGS"};
  
  const char** apps1 = (system_state.current_theme == THEME_BOBOIBOY) ? apps1_boboiboy : apps1_normal;
  
  int cols = 3;
  int iconW = 115;  // Larger icons
  int iconH = 100;  // Taller icons
  int gapX = 15;
  int gapY = 12;
  int startX = (LCD_WIDTH - (cols * iconW + (cols-1) * gapX)) / 2;
  int startY = headerH + 12;
  int spacingX = iconW + gapX;
  int spacingY = iconH + gapY;
  
  for (int i = 0; i < 9; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * spacingX;
    int y = startY + row * spacingY;
    
    drawAppIcon(x, y, iconW, iconH, apps1[i], getAppColor(apps1[i]), false);
  }
  
  // Swipe hint - retro style
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
  gfx->print("< SWIPE >");
  
  drawSwipeIndicator();
}

void drawAppGrid2() {
  // ========================================
  // RETRO ANIME APP GRID PAGE 2 (Optimized for 410x502)
  // ========================================
  
  gfx->fillScreen(RGB565(2, 2, 5));
  
  // Subtle CRT scan lines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->accent);
  }
  
  // Title - larger
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 35, 14);
  gfx->print("More");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 36, 12);
  gfx->print("More");
  
  // Page indicator - larger
  gfx->drawRect(LCD_WIDTH - 50, 22, 10, 10, RGB565(50, 55, 70));
  gfx->fillRect(LCD_WIDTH - 35, 22, 10, 10, theme->accent);
  gfx->drawRect(LCD_WIDTH - 20, 22, 10, 10, RGB565(50, 55, 70));
  
  // App icons - larger
  const char* apps2[] = {"THEMES", "COLLECT", "FILES", "CALC", "TORCH", "OTA", "BACKUP", "FUSION", "ABOUT"};
  
  int cols = 3;
  int iconW = 115;
  int iconH = 100;
  int gapX = 15;
  int gapY = 12;
  int startX = (LCD_WIDTH - (cols * iconW + (cols-1) * gapX)) / 2;
  int startY = headerH + 12;
  int spacingX = iconW + gapX;
  int spacingY = iconH + gapY;
  
  for (int i = 0; i < 9; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * spacingX;
    int y = startY + row * spacingY;
    
    drawAppIcon(x, y, iconW, iconH, apps2[i], getAppColor(apps2[i]), false);
  }
  
  // Swipe hint
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
  gfx->print("< SWIPE >");
  
  drawSwipeIndicator();
}

void drawAppGrid3() {
  // ========================================
  // RETRO ANIME APP GRID PAGE 3 (Optimized for 410x502)
  // ========================================
  
  gfx->fillScreen(RGB565(2, 2, 5));
  
  // Subtle CRT scan lines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->effect1);
  }
  
  // Title - larger
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 70, 14);
  gfx->print("New Apps");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 72, 12);
  gfx->print("New Apps");
  
  // Page indicator - larger
  gfx->drawRect(LCD_WIDTH - 50, 22, 10, 10, RGB565(50, 55, 70));
  gfx->drawRect(LCD_WIDTH - 35, 22, 10, 10, RGB565(50, 55, 70));
  gfx->fillRect(LCD_WIDTH - 20, 22, 10, 10, theme->effect1);
  
  // NEW App icons
  const char* apps3[] = {"QUESTS", "STEPS", "ACHIEVE", "SHOP", "SOCIAL", "GALLERY", "TIMER", "GAMES", "CONVERT"};
  
  int cols = 3;
  int iconW = 115;
  int iconH = 100;
  int gapX = 15;
  int gapY = 12;
  int startX = (LCD_WIDTH - (cols * iconW + (cols-1) * gapX)) / 2;
  int startY = headerH + 12;
  int spacingX = iconW + gapX;
  int spacingY = iconH + gapY;
  
  for (int i = 0; i < 9; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * spacingX;
    int y = startY + row * spacingY;
    
    drawAppIcon(x, y, iconW, iconH, apps3[i], getAppColor(apps3[i]), false);
  }
  
  // Swipe hint (show up/down since we're at page 3 of app grid)
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
  gfx->print("< SWIPE >");
  
  drawSwipeIndicator();
}

// =============================================================================
// TOUCH HANDLING - IMPROVED
// =============================================================================

void handleCurrentScreenTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x;
  int y = gesture.y;
  
  Serial.printf("[NAV] TAP at (%d, %d) on screen %d\n", x, y, navState.currentMain);
  
  switch (navState.currentMain) {
    case MAIN_WATCHFACE:
      // Tap on watchface - switch element if BoBoiBoy theme
      if (system_state.current_theme == THEME_BOBOIBOY) {
        // Check if tap is in center zone
        if (isElementSwitchZoneTap(x, y)) {
          switchToNextElement();
          drawCurrentScreen();
        }
      }
      Serial.println("[NAV] Tap on watchface");
      break;
      
    case MAIN_APP_GRID_1:
      handleAppGridTap(x, y);
      break;
      
    case MAIN_CHARACTER_STATS:
      // Back button at bottom
      if (y >= 410 && x >= 140 && x < 220) {
        navigateToScreen(MAIN_WATCHFACE);
      }
      break;
  }
}

void handleAppGridTap(int x, int y) {
  Serial.printf("[NAV] handleAppGridTap: x=%d, y=%d, page=%d\n", x, y, navState.appGridPage);
  
  // App grid hit detection - larger icons for 410x502 display
  int cols = 3;
  int iconW = 115;  // Match drawAppGrid sizes
  int iconH = 100;
  int gapX = 15;
  int gapY = 12;
  int headerH = 55;
  int startX = (LCD_WIDTH - (cols * iconW + (cols-1) * gapX)) / 2;
  int startY = headerH + 12;
  int spacingX = iconW + gapX;
  int spacingY = iconH + gapY;
  
  for (int i = 0; i < 9; i++) {
    int col = i % cols;
    int row = i / cols;
    int appX = startX + col * spacingX;
    int appY = startY + row * spacingY;
    
    // Check if tap is within app icon bounds
    if (x >= appX && x < appX + iconW && 
        y >= appY && y < appY + iconH) {
      
      // In BoBoiBoy mode, replace QUESTS with ELEMENTS (NOTES removed)
      const char* apps1_normal[] = {"GACHA", "TRAINING", "BOSS", "QUESTS", "MUSIC", "WEATHER", "WIFI", "TIMER", "SETTINGS"};
      const char* apps1_boboiboy[] = {"GACHA", "TRAINING", "BOSS", "ELEMENTS", "MUSIC", "WEATHER", "WIFI", "TIMER", "SETTINGS"};
      const char* apps2[] = {"THEMES", "COLLECT", "FILES", "CALC", "TORCH", "OTA", "BACKUP", "FUSION", "ABOUT"};
      const char* apps3[] = {"QUESTS", "STEPS", "ACHIEVE", "SHOP", "SOCIAL", "GALLERY", "TIMER", "GAMES", "CONVERT"};
      
      const char** apps1 = (system_state.current_theme == THEME_BOBOIBOY) ? apps1_boboiboy : apps1_normal;
      const char* appName;
      if (navState.appGridPage == 0) appName = apps1[i];
      else if (navState.appGridPage == 1) appName = apps2[i];
      else appName = apps3[i];
      
      Serial.printf("[NAV] >>> OPENING APP: %s <<<\n", appName);
      openApp(appName);
      return;
    }
  }
  
  Serial.println("[NAV] Tap missed all app icons");
}

void openApp(const char* appName) {
  Serial.printf("[NAV] Opening app: %s\n", appName);
  
  // Lock navigation while in app
  navState.navigationLocked = true;
  
  // Route to correct app handler
  if (strcmp(appName, "GACHA") == 0) {
    system_state.current_screen = SCREEN_GACHA;
    drawGachaScreen();
  } 
  else if (strcmp(appName, "TRAINING") == 0) {
    system_state.current_screen = SCREEN_TRAINING;
    drawTrainingMenu();
  }
  else if (strcmp(appName, "BOSS") == 0) {
    system_state.current_screen = SCREEN_BOSS_RUSH;
    drawBossRushMenu();
  }
  else if (strcmp(appName, "GAMES") == 0) {
    system_state.current_screen = SCREEN_GAMES;
    drawGameMenu();
  }
  else if (strcmp(appName, "QUESTS") == 0) {
    system_state.current_screen = SCREEN_DAILY_QUESTS;
    drawDailyQuestsScreen();
  }
  else if (strcmp(appName, "STEPS") == 0) {
    system_state.current_screen = SCREEN_STEPS_TRACKER;
    drawStepsCard();
  }
  else if (strcmp(appName, "TIMER") == 0) {
    system_state.current_screen = SCREEN_TIMER;
    initTimerApp();
    drawTimerApp();
  }
  else if (strcmp(appName, "COMPASS") == 0) {
    // Compass not yet implemented - show coming soon
    system_state.current_screen = SCREEN_SETTINGS;
    navState.navigationLocked = true;
    gfx->fillScreen(RGB565(2, 2, 5));
    gfx->setTextColor(getCurrentTheme()->primary);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 42, 180);
    gfx->print("COMPASS");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(LCD_WIDTH/2 - 55, 220);
    gfx->print("Coming soon!");
    gfx->setCursor(LCD_WIDTH/2 - 70, 240);
    gfx->print("Requires magnetometer");
    drawSwipeIndicator();
  }
  else if (strcmp(appName, "CONVERT") == 0) {
    system_state.current_screen = SCREEN_CONVERTER;
    initConverterApp();
    drawConverterApp();
  }
  else if (strcmp(appName, "ACHIEVE") == 0) {
    system_state.current_screen = SCREEN_ACHIEVEMENTS;
    initAchievementsApp();
    drawAchievementsApp();
  }
  else if (strcmp(appName, "SHOP") == 0) {
    system_state.current_screen = SCREEN_SHOP;
    initShopApp();
    drawShopApp();
  }
  else if (strcmp(appName, "SOCIAL") == 0) {
    // Social not yet implemented - show coming soon
    system_state.current_screen = SCREEN_SETTINGS;
    navState.navigationLocked = true;
    gfx->fillScreen(RGB565(2, 2, 5));
    gfx->setTextColor(getCurrentTheme()->primary);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 36, 180);
    gfx->print("SOCIAL");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(LCD_WIDTH/2 - 55, 220);
    gfx->print("Coming soon!");
    gfx->setCursor(LCD_WIDTH/2 - 65, 240);
    gfx->print("Connect with friends");
    drawSwipeIndicator();
  }
  else if (strcmp(appName, "GALLERY") == 0) {
    system_state.current_screen = SCREEN_GALLERY;
    initGalleryApp();
    drawGalleryApp();
  }
  else if (strcmp(appName, "ELEMENTS") == 0) {
    // BoBoiBoy Element Tree - only works in BoBoiBoy mode
    if (system_state.current_theme == THEME_BOBOIBOY) {
      system_state.current_screen = SCREEN_ELEMENT_TREE;
      initBoboiboyElements();
      drawElementTree();
    } else {
      // Fallback to quests if not in BoBoiBoy mode
      system_state.current_screen = SCREEN_QUESTS;
      drawQuestScreen();
    }
  }
  else if (strcmp(appName, "MUSIC") == 0) {
    system_state.current_screen = SCREEN_MUSIC;
    drawMusicApp();
  }
  else if (strcmp(appName, "WEATHER") == 0) {
    system_state.current_screen = SCREEN_WEATHER_APP;
    drawWeatherApp();
  }
  else if (strcmp(appName, "WIFI") == 0) {
    system_state.current_screen = SCREEN_WIFI_MANAGER;
    drawNetworkListScreen();
  }
  else if (strcmp(appName, "SETTINGS") == 0) {
    system_state.current_screen = SCREEN_SETTINGS;
    drawSettingsApp();
  }
  else if (strcmp(appName, "THEMES") == 0) {
    system_state.current_screen = SCREEN_THEME_SELECTOR;
    drawThemeSelector();
  }
  else if (strcmp(appName, "COLLECT") == 0) {
    system_state.current_screen = SCREEN_COLLECTION;
    drawGachaCollection();
  }
  else if (strcmp(appName, "FILES") == 0) {
    system_state.current_screen = SCREEN_FILE_BROWSER;
    drawFileBrowserApp();
  }
  else if (strcmp(appName, "CALC") == 0) {
    system_state.current_screen = SCREEN_CALCULATOR;
    drawCalculatorApp();
  }
  else if (strcmp(appName, "TORCH") == 0) {
    system_state.current_screen = SCREEN_FLASHLIGHT;
    drawFlashlightApp();
  }
  else if (strcmp(appName, "FUSION") == 0) {
    // BoBoiBoy Fusion Minigame
    if (system_state.current_theme == THEME_BOBOIBOY) {
      system_state.current_screen = SCREEN_FUSION_GAME;
      initFusionGame();
      drawFusionGame();
    } else {
      // Show message for non-BBB themes
      gfx->fillScreen(COLOR_BLACK);
      gfx->setTextColor(getCurrentTheme()->primary);
      gfx->setTextSize(2);
      gfx->setCursor(60, 200);
      gfx->print("BoBoiBoy Only!");
      gfx->setTextSize(1);
      gfx->setCursor(70, 240);
      gfx->print("Switch to BoBoiBoy theme");
      drawSwipeIndicator();
    }
  }
  else if (strcmp(appName, "OTA") == 0) {
    system_state.current_screen = SCREEN_SETTINGS;
    gfx->fillScreen(COLOR_BLACK);
    
    ThemeColors* theme = getCurrentTheme();
    
    // Header - retro
    gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
      gfx->fillRect(x, 46, 6, 3, theme->primary);
    }
    gfx->setTextColor(theme->primary);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 60, 14);
    gfx->print("OTA UPDATE");
    
    // Status card - retro
    gfx->fillRect(24, 70, LCD_WIDTH - 48, 90, RGB565(12, 14, 20));
    gfx->drawRect(24, 70, LCD_WIDTH - 48, 90, RGB565(40, 45, 60));
    gfx->fillRect(24, 70, 5, 5, theme->primary);
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(40, 82);
    gfx->print("Firmware v2.0");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(0, 200, 80));
    gfx->setCursor(40, 110);
    gfx->print("Up to date");
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(40, 135);
    gfx->print("Connect WiFi to check updates");
    
    // Check button - retro
    gfx->fillRect(LCD_WIDTH/2 - 80, 185, 160, 45, theme->primary);
    gfx->drawRect(LCD_WIDTH/2 - 80, 185, 160, 45, COLOR_WHITE);
    gfx->fillRect(LCD_WIDTH/2 - 80, 185, 5, 5, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 55, 198);
    gfx->print("Check Now");
    
    drawSwipeIndicator();
  }
  else if (strcmp(appName, "BACKUP") == 0) {
    system_state.current_screen = SCREEN_SETTINGS;
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
      gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(4, 4, 7));
    }
    
    ThemeColors* theme = getCurrentTheme();
    
    // Header - retro
    gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
      gfx->fillRect(x, 46, 6, 3, theme->accent);
    }
    gfx->setTextColor(theme->accent);
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 36, 14);
    gfx->print("BACKUP");
    
    // Backup option - retro
    gfx->fillRect(24, 65, LCD_WIDTH - 48, 65, RGB565(12, 14, 20));
    gfx->drawRect(24, 65, LCD_WIDTH - 48, 65, RGB565(40, 45, 60));
    gfx->fillRect(24, 65, 5, 5, theme->accent);
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(40, 78);
    gfx->print("Save to SD");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(40, 108);
    gfx->print("Game progress, settings, themes");
    
    // Restore option - retro
    gfx->fillRect(24, 145, LCD_WIDTH - 48, 65, RGB565(12, 14, 20));
    gfx->drawRect(24, 145, LCD_WIDTH - 48, 65, RGB565(40, 45, 60));
    gfx->fillRect(24, 145, 5, 5, theme->accent);
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(40, 158);
    gfx->print("Restore");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(40, 188);
    gfx->print("Load backup from SD card");
    
    // Status
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setCursor(40, 230);
    gfx->printf("SD Card: %s", system_state.filesystem_available ? "Ready" : "Not found");
    
    drawSwipeIndicator();
  }
  else if (strcmp(appName, "ABOUT") == 0) {
    system_state.current_screen = SCREEN_SETTINGS;
    // drawAboutScreen() is defined in ui.cpp
    drawAboutScreen();
  }
  else {
    // Unknown app - show message and return
    Serial.printf("[NAV] Unknown app: %s\n", appName);
    system_state.current_screen = SCREEN_SETTINGS;
    navState.navigationLocked = true;
    gfx->fillScreen(RGB565(8, 8, 12));
    gfx->setTextColor(getCurrentTheme()->primary);
    gfx->setTextSize(2);
    gfx->setCursor(100, 200);
    gfx->print("Coming Soon");
    gfx->setTextSize(1);
    gfx->setCursor(110, 240);
    gfx->print(appName);
    drawSwipeIndicator();
  }
}

void returnToAppGrid() {
  Serial.println("[NAV] Returning to app grid");
  navState.navigationLocked = false;
  navState.currentMain = MAIN_APP_GRID_1;
  system_state.current_screen = SCREEN_APP_GRID;
  drawCurrentScreen();
}

void drawBackButton(int x, int y) {
  // Apple Watch style swipe indicator - replaces old back button
  drawSwipeIndicator();
}

// Retro pixel-style swipe indicator
void drawSwipeIndicator() {
  int barW = 40;
  int barH = 4;
  int barX = (LCD_WIDTH - barW) / 2;
  int barY = LCD_HEIGHT - 10;
  
  // Pixel style home bar
  gfx->fillRect(barX, barY, barW, barH, RGB565(50, 55, 70));
  // Highlight line
  gfx->drawFastHLine(barX + 2, barY, barW - 4, RGB565(80, 85, 100));
}

// drawAboutScreen() is defined in ui.cpp - removed from here to avoid duplicate

// handleTouchInput() is defined in touch.cpp - removed from here to avoid duplicate

// Helper functions
void drawStatBar(int x, int y, const char* name, int value, uint16_t color) {
  int maxWidth = 200;
  int barWidth = (value * maxWidth) / 100;
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x, y);
  gfx->print(name);
  
  // Bar background - retro pixel
  gfx->fillRect(x + 80, y - 2, maxWidth, 12, RGB565(8, 10, 14));
  gfx->drawRect(x + 80, y - 2, maxWidth, 12, RGB565(30, 35, 50));
  
  // Bar fill
  if (barWidth > 0) {
    gfx->fillRect(x + 81, y - 1, barWidth - 2, 10, color);
  }
  
  // Value text
  gfx->setCursor(x + 290, y);
  gfx->printf("%d", value);
}

int getXPForNextLevel() {
  return system_state.player_level * 100 + 100;
}
