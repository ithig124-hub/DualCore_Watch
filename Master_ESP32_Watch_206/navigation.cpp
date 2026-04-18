/*
 * navigation.cpp - IMPROVED Swipe Navigation (FIXED)
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 *
 * UPDATED: Story Mode + Companion Care apps added
 * - Page 1: MUSIC replaced with CARE (Companion)
 * - Page 3: SOCIAL replaced with STORY
 *
 * FIXES APPLIED:
 * 1. Steps Tracker now properly draws when navigating to it
 * 2. Character Stats redraws EVERY time you navigate to it (not just on XP change)
 * 3. Reduced navigation cooldown for smoother transitions
 * 4. Added force redraw for Steps Tracker screen
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
//#include "rpg.h"
#include "apps.h"
#include "wifi_apps.h"
#include "ui.h"
#include "boboiboy_elements.h"
#include "ochobot.h"
#include "fusion_game.h"
#include "character_games.h"
#include "steps_tracker.h"
#include "daily_quests.h"
#include "storyline.h"
#include "companion.h"
#include "xp_system.h"
#include <Arduino.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Touch state tracking
TouchState touchState = {false, 0, 0, 0, 0, 0, 0};

// Global navigation state
NavigationState navState = {
    MAIN_WATCHFACE,  // currentMain
    0,               // appGridPage
    false,           // isTransitioning
    0,               // lastNavigationMs
    false            // navigationLocked
};

// =============================================================================
// FIX: Force redraw flag for Steps Tracker
// =============================================================================
static bool g_force_steps_redraw = true;

void forceStepsRedraw() {
    g_force_steps_redraw = true;
    Serial.println("[NAV] Steps tracker redraw forced");
}

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
    Serial.println("  - Swipe LEFT/RIGHT: Switch main screens");
    Serial.println("  - Swipe UP/DOWN: App grid pages");
    Serial.println("  - TAP: Select apps/buttons");
}

// =============================================================================
// NAVIGATION CONTROL - FIXED
// =============================================================================

bool canNavigate() {
    // FIX: Reduced timeout from 1000ms to 500ms for faster recovery
    if (navState.isTransitioning && (millis() - navState.lastNavigationMs > 500)) {
        Serial.println("[NAV] Force resetting stuck transition state");
        navState.isTransitioning = false;
    }

    // FIX: Auto-unlock navigation if stuck locked for >2 seconds
    if (navState.navigationLocked && (millis() - navState.lastNavigationMs > 2000)) {
        Serial.println("[NAV] Force unlocking stuck navigation lock");
        navState.navigationLocked = false;
    }

    if (navState.navigationLocked) {
        return false;
    }

    if (navState.isTransitioning) {
        return false;
    }

    // FIX: Reduced cooldown for smoother navigation (was ~200ms)
    if (millis() - navState.lastNavigationMs < 150) {
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

// =============================================================================
// NAVIGATE TO SCREEN - FIXED
// =============================================================================
void navigateToScreen(MainScreen screen) {
    navState.isTransitioning = true;
    navState.lastNavigationMs = millis();

    MainScreen oldScreen = navState.currentMain;
    navState.currentMain = screen;

    switch (screen) {
        case MAIN_WATCHFACE: {
            extern void forceWatchfaceRedraw();
            forceWatchfaceRedraw();
            Serial.println("[NAV] Forced watchface redraw");
            break;
        }
        case MAIN_STEPS_TRACKER: {
            g_force_steps_redraw = true;
            forceStepsCardRedraw();  // FIX: Also reset the internal anti-flicker static vars
            Serial.println("[NAV] Forced steps tracker redraw");
            break;
        }
        case MAIN_CHARACTER_STATS: {
            extern void forceCharacterStatsRedraw();
            forceCharacterStatsRedraw();
            Serial.println("[NAV] Forced character stats redraw");
            break;
        }
        case MAIN_APP_GRID_1:
            break;
    }

    // Reset app grid page when leaving app grid
    if (oldScreen == MAIN_APP_GRID_1 && screen != MAIN_APP_GRID_1) {
        navState.appGridPage = 0;
    }

    // Map to system screen type
    switch (screen) {
        case MAIN_WATCHFACE:
            system_state.current_screen = SCREEN_WATCHFACE;
            break;
        case MAIN_STEPS_TRACKER:
            system_state.current_screen = SCREEN_STEPS_TRACKER;
            break;
        case MAIN_APP_GRID_1:
            system_state.current_screen = SCREEN_APP_GRID;
            break;
        case MAIN_CHARACTER_STATS:
            system_state.current_screen = SCREEN_CHARACTER_STATS;
            break;
    }

    Serial.printf("[NAV] Screen changed: %d -> %d (system: %d)\n", oldScreen, screen, system_state.current_screen);

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
// SCREEN DRAWING - FIXED
// =============================================================================

void drawCurrentScreen() {
    extern void forceWatchfaceRedraw();
    forceWatchfaceRedraw();

    static MainScreen last_screen = MAIN_WATCHFACE;

    if (navState.currentMain == MAIN_STEPS_TRACKER && last_screen != MAIN_STEPS_TRACKER) {
        g_force_steps_redraw = true;
        forceStepsCardRedraw();  // FIX: Reset anti-flicker statics too
        Serial.println("[NAV] Entering Steps Tracker - forcing redraw");
    }

    if (navState.currentMain == MAIN_CHARACTER_STATS) {
        extern void forceCharacterStatsRedraw();
        forceCharacterStatsRedraw();
        Serial.println("[NAV] Drawing Character Stats - forcing redraw");
    }

    last_screen = navState.currentMain;

    switch (navState.currentMain) {
        case MAIN_WATCHFACE:
            drawWatchFace();
            break;

        case MAIN_STEPS_TRACKER:
            if (g_force_steps_redraw) {
                gfx->fillScreen(RGB565(2, 2, 5));
                forceStepsCardRedraw();  // FIX: Ensure internal statics are reset
                Serial.println("[NAV] Steps Tracker: Full screen clear done");
                g_force_steps_redraw = false;
            }
            drawStepsCard();
            break;

        case MAIN_APP_GRID_1:
            if (navState.appGridPage == 0) drawAppGrid1();
            else if (navState.appGridPage == 1) drawAppGrid2();
            else if (navState.appGridPage == 2) drawAppGrid3();
            else drawAppGrid4();
            break;

        case MAIN_CHARACTER_STATS:
            drawCharacterStatsScreen();
            break;
    }

    drawNavigationIndicators();
}

void drawNavigationIndicators() {
    int centerX = LCD_WIDTH / 2;
    int y = LCD_HEIGHT - 14;
    int dotSize = 6;
    int spacing = 18;

    gfx->fillRect(centerX - 40, y - 10, 80, 20, RGB565(2, 2, 5));

    for (int i = 0; i < MAIN_SCREEN_COUNT; i++) {
        int x = centerX + (i - (MAIN_SCREEN_COUNT - 1) / 2.0) * spacing - dotSize/2;

        if (i == (int)navState.currentMain) {
            gfx->fillRect(x - 1, y - 1, dotSize + 2, dotSize + 2,
                RGB565(getCurrentTheme()->primary >> 12, 15, 20));
            gfx->fillRect(x, y, dotSize, dotSize, getCurrentTheme()->primary);
        } else {
            gfx->drawRect(x, y, dotSize, dotSize, RGB565(40, 45, 60));
        }
    }
}

// =============================================================================
// APP GRID - IMPROVED MODERN DESIGN
// =============================================================================

uint16_t getAppColor(const char* appName) {
    if (strcmp(appName, "GACHA") == 0) return RGB565(255, 180, 50);
    if (strcmp(appName, "TRAINING") == 0) return RGB565(50, 200, 100);
    if (strcmp(appName, "BOSS") == 0) return RGB565(220, 60, 60);
    if (strcmp(appName, "GAMES") == 0) return RGB565(100, 150, 255);
    if (strcmp(appName, "QUESTS") == 0) return RGB565(255, 150, 100);
    if (strcmp(appName, "STEPS") == 0) return RGB565(100, 255, 100);
    if (strcmp(appName, "ACHIEVE") == 0) return RGB565(255, 200, 50);
    if (strcmp(appName, "SHOP") == 0) return RGB565(200, 100, 255);
    if (strcmp(appName, "STORY") == 0) return RGB565(200, 100, 50);    // Bronze/adventure
    if (strcmp(appName, "CARE") == 0) return RGB565(255, 150, 200);    // Pink/caring
    if (strcmp(appName, "GALLERY") == 0) return RGB565(255, 100, 200);
    if (strcmp(appName, "TIMER") == 0) return RGB565(100, 180, 255);
    if (strcmp(appName, "COMPASS") == 0) return RGB565(255, 100, 150);
    if (strcmp(appName, "CONVERT") == 0) return RGB565(180, 120, 255);
    if (strcmp(appName, "NOTES") == 0) return RGB565(255, 200, 60);
    if (strcmp(appName, "ELEMENTS") == 0) return BBB_BAND_ORANGE;
    if (strcmp(appName, "MUSIC") == 0) return RGB565(255, 100, 150);
    if (strcmp(appName, "WEATHER") == 0) return RGB565(80, 180, 255);
    if (strcmp(appName, "WIFI") == 0) return RGB565(100, 220, 200);
    if (strcmp(appName, "SETTINGS") == 0) return RGB565(150, 150, 160);
    if (strcmp(appName, "THEMES") == 0) return RGB565(255, 120, 80);
    if (strcmp(appName, "COLLECT") == 0) return RGB565(255, 200, 100);
    if (strcmp(appName, "FILES") == 0) return RGB565(180, 180, 100);
    if (strcmp(appName, "CALC") == 0) return RGB565(100, 100, 120);
    if (strcmp(appName, "TORCH") == 0) return RGB565(255, 255, 150);
    if (strcmp(appName, "OTA") == 0) return RGB565(100, 200, 150);
    if (strcmp(appName, "BACKUP") == 0) return RGB565(150, 180, 200);
    if (strcmp(appName, "FUSION") == 0) return RGB565(200, 150, 255);
    if (strcmp(appName, "ABOUT") == 0) return RGB565(180, 180, 190);
    if (strcmp(appName, "WALLPAPER") == 0) return RGB565(200, 120, 180);
    if (strcmp(appName, "POMODORO") == 0) return RGB565(255, 80, 80);
    if (strcmp(appName, "HABITS") == 0) return RGB565(80, 220, 160);
    if (strcmp(appName, "DUNGEON") == 0) return RGB565(180, 60, 255);
    if (strcmp(appName, "STREAK") == 0) return RGB565(255, 200, 50);
    if (strcmp(appName, "CRAFT") == 0) return RGB565(200, 150, 100);
    return RGB565(100, 100, 120);
}

void drawAppIcon(int x, int y, int w, int h, const char* name, uint16_t color, bool selected) {
    // Shadow (subtle)
    gfx->fillRect(x + 3, y + 3, w, h, RGB565(3, 3, 6));

    // Main icon background
    gfx->fillRect(x, y, w, h, RGB565(15, 18, 25));

    // Pixel corner accents
    gfx->fillRect(x, y, 5, 5, color);
    gfx->fillRect(x + w - 5, y, 5, 5, color);
    gfx->fillRect(x, y + h - 5, 5, 5, color);
    gfx->fillRect(x + w - 5, y + h - 5, 5, 5, color);

    // Scan line effect
    for (int sy = y + 6; sy < y + h - 35; sy += 4) {
        gfx->drawFastHLine(x + 5, sy, w - 10, RGB565(10, 12, 18));
    }

    // Colored icon square at center-top
    int iconCX = x + w / 2;
    int iconCY = y + 30;
    int iconSize = 28;

    // Icon glow
    gfx->fillRect(iconCX - iconSize/2 - 2, iconCY - iconSize/2 - 2, iconSize + 4, iconSize + 4,
        RGB565((color >> 11) * 2, ((color >> 5) & 0x3F) / 3, (color & 0x1F) / 3));
    // Icon square
    gfx->fillRect(iconCX - iconSize/2, iconCY - iconSize/2, iconSize, iconSize, color);
    // Inner highlight
    gfx->drawRect(iconCX - iconSize/2 + 2, iconCY - iconSize/2 + 2, iconSize - 4, 2, 0xFFFF);

    // Selection effect
    if (selected) {
        gfx->drawRect(x - 2, y - 2, w + 4, h + 4, getCurrentTheme()->primary);
        gfx->drawRect(x - 1, y - 1, w + 2, h + 2, getCurrentTheme()->accent);
    } else {
        gfx->drawRect(x, y, w, h, RGB565(35, 40, 55));
    }

    // APP NAME TEXT
    int textAreaY = y + h - 28;
    gfx->fillRect(x + 4, textAreaY - 4, w - 8, 24, RGB565(10, 12, 18));

    int textLen = strlen(name);
    int textPixelWidth = textLen * 12;
    int textX = x + (w - textPixelWidth) / 2;
    int textY = textAreaY;

    // Shadow
    gfx->setTextSize(2);
    gfx->setTextColor(0x0000);
    gfx->setCursor(textX + 1, textY + 1);
    gfx->print(name);

    // Main text in white
    gfx->setTextColor(0xFFFF);
    gfx->setCursor(textX, textY);
    gfx->print(name);
}

void drawAppGrid1() {
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();

    int headerH = 55;
    gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
    }

    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(30, 35, 50));
    gfx->setCursor(LCD_WIDTH/2 - 50, 10);
    gfx->print("BATTLE");
    gfx->setTextColor(theme->primary);
    gfx->setCursor(LCD_WIDTH/2 - 51, 8);
    gfx->print("BATTLE");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 - 50, 30);
    gfx->print("RPG & Combat Apps");

    // Page indicator (4 pages)
    for (int p = 0; p < APP_GRID_PAGES; p++) {
        int px = LCD_WIDTH - 15 - (APP_GRID_PAGES - p) * 15;
        if (p == 0)
            gfx->fillRect(px, 22, 10, 10, theme->primary);
        else
            gfx->drawRect(px, 22, 10, 10, RGB565(50, 55, 70));
    }

    // Grid 1: RPG & Gaming - reorganized for best flow
    const char* apps1_normal[] = {"GACHA", "BOSS", "TRAINING", "STORY", "QUESTS", "GAMES", "CARE", "COLLECT", "FUSION"};
    const char* apps1_boboiboy[] = {"GACHA", "BOSS", "TRAINING", "STORY", "QUESTS", "GAMES", "ELEMENTS", "COLLECT", "FUSION"};

    const char** apps1 = (system_state.current_theme == THEME_BOBOIBOY) ? apps1_boboiboy : apps1_normal;

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

        drawAppIcon(x, y, iconW, iconH, apps1[i], getAppColor(apps1[i]), false);
    }

    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
    gfx->print("SWIPE UP >");

    drawSwipeIndicator();
}

void drawAppGrid2() {
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();

    int headerH = 55;
    gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, headerH - 3, 6, 3, theme->accent);
    }

    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(30, 35, 50));
    gfx->setCursor(LCD_WIDTH/2 - 40, 10);
    gfx->print("TOOLS");
    gfx->setTextColor(theme->accent);
    gfx->setCursor(LCD_WIDTH/2 - 41, 8);
    gfx->print("TOOLS");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 - 48, 30);
    gfx->print("Daily Use & Utils");

    for (int p = 0; p < APP_GRID_PAGES; p++) {
        int px = LCD_WIDTH - 15 - (APP_GRID_PAGES - p) * 15;
        if (p == 1)
            gfx->fillRect(px, 22, 10, 10, theme->accent);
        else
            gfx->drawRect(px, 22, 10, 10, RGB565(50, 55, 70));
    }

    // Grid 2: Daily Use - reorganized
    const char* apps2[] = {"STEPS", "TIMER", "CALC", "SHOP", "ACHIEVE", "GALLERY", "CONVERT", "TORCH", "WEATHER"};

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

    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
    gfx->print("< SWIPE >");

    drawSwipeIndicator();
}

void drawAppGrid3() {
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();

    int headerH = 55;
    gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, headerH - 3, 6, 3, theme->effect1);
    }

    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(30, 35, 50));
    gfx->setCursor(LCD_WIDTH/2 - 50, 10);
    gfx->print("SYSTEM");
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(LCD_WIDTH/2 - 51, 8);
    gfx->print("SYSTEM");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 - 58, 30);
    gfx->print("Settings & Media");

    for (int p = 0; p < APP_GRID_PAGES; p++) {
        int px = LCD_WIDTH - 15 - (APP_GRID_PAGES - p) * 15;
        if (p == 2)
            gfx->fillRect(px, 22, 10, 10, theme->effect1);
        else
            gfx->drawRect(px, 22, 10, 10, RGB565(50, 55, 70));
    }

    // Page 3: System & media
    const char* apps3[] = {"SETTINGS", "THEMES", "WIFI", "MUSIC", "FILES", "WALLPAPER", "BACKUP", "OTA", "ABOUT"};

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

    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
    gfx->print("< SWIPE >");

    drawSwipeIndicator();
}

void drawAppGrid4() {
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();

    int headerH = 55;
    gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, headerH - 3, 6, 3, RGB565(255, 180, 50));
    }

    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(30, 35, 50));
    gfx->setCursor(LCD_WIDTH/2 - 40, 10);
    gfx->print("EXTRA");
    gfx->setTextColor(RGB565(255, 200, 80));
    gfx->setCursor(LCD_WIDTH/2 - 41, 8);
    gfx->print("EXTRA");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 - 55, 30);
    gfx->print("Bonus & Lifestyle");

    // Page indicator (page 4 active)
    for (int p = 0; p < APP_GRID_PAGES; p++) {
        int px = LCD_WIDTH - 15 - (APP_GRID_PAGES - p) * 15;
        if (p == 3)
            gfx->fillRect(px, 22, 10, 10, RGB565(255, 180, 50));
        else
            gfx->drawRect(px, 22, 10, 10, RGB565(50, 55, 70));
    }

    // Grid 4: Extra features - DUNGEON only for Jinwoo theme
    const char* apps4_jinwoo[] = {"POMODORO", "HABITS", "DUNGEON", "STREAK", "CRAFT", "NOTES", "COMPASS", "THEMES", "ABOUT"};
    const char* apps4_other[]  = {"POMODORO", "HABITS", "NOTES",   "STREAK", "CRAFT", "COMPASS", "SETTINGS", "THEMES", "ABOUT"};
    
    const char** apps4 = (system_state.current_theme == THEME_SUNG_JINWOO) ? apps4_jinwoo : apps4_other;

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

        drawAppIcon(x, y, iconW, iconH, apps4[i], getAppColor(apps4[i]), false);
    }

    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 25);
    gfx->print("< SWIPE DOWN");

    drawSwipeIndicator();
}

// =============================================================================
// TOUCH HANDLING - IMPROVED
// =============================================================================

void handleCurrentScreenTouch(TouchGesture& gesture) {
    // FIX: Accept both TAP and RELEASE-with-small-movement as taps
    // On small watch screens, many taps get classified as RELEASE
    bool isTap = (gesture.event == TOUCH_TAP);
    if (!isTap && gesture.event == TOUCH_RELEASE) {
        int dx = abs(gesture.end_x - gesture.start_x);
        int dy = abs(gesture.end_y - gesture.start_y);
        if (dx < 25 && dy < 25 && gesture.duration < 600) {
            isTap = true;
            // Use start position for more accurate tap location
            gesture.x = gesture.start_x;
            gesture.y = gesture.start_y;
        }
    }
    if (!isTap) return;

    int x = gesture.x;
    int y = gesture.y;

    Serial.printf("[NAV] TAP at (%d, %d) on screen %d\n", x, y, navState.currentMain);

    switch (navState.currentMain) {
        case MAIN_WATCHFACE:
            if (system_state.current_theme == THEME_BOBOIBOY) {
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
            if (y >= 410 && x >= 140 && x < 220) {
                navigateToScreen(MAIN_WATCHFACE);
            }
            break;
    }
}

void handleAppGridTap(int x, int y) {
    Serial.printf("[NAV] handleAppGridTap: x=%d, y=%d, page=%d\n", x, y, navState.appGridPage);

    int cols = 3;
    int iconW = 115;
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

        if (x >= appX && x < appX + iconW &&
            y >= appY && y < appY + iconH) {

            // Grid 1: RPG, Grid 2: Daily, Grid 3: System
            const char* apps1_normal[] = {"GACHA", "BOSS", "TRAINING", "STORY", "QUESTS", "GAMES", "CARE", "COLLECT", "FUSION"};
            const char* apps1_boboiboy[] = {"GACHA", "BOSS", "TRAINING", "STORY", "QUESTS", "GAMES", "ELEMENTS", "COLLECT", "FUSION"};
            const char* apps2[] = {"STEPS", "TIMER", "CALC", "SHOP", "ACHIEVE", "GALLERY", "CONVERT", "TORCH", "WEATHER"};
            const char* apps3[] = {"SETTINGS", "THEMES", "WIFI", "MUSIC", "FILES", "WALLPAPER", "BACKUP", "OTA", "ABOUT"};

            const char** apps1 = (system_state.current_theme == THEME_BOBOIBOY) ? apps1_boboiboy : apps1_normal;
            const char* appName;
            if (navState.appGridPage == 0) appName = apps1[i];
            else if (navState.appGridPage == 1) appName = apps2[i];
            else if (navState.appGridPage == 2) appName = apps3[i];
            else {
                const char* apps4_jinwoo[] = {"POMODORO", "HABITS", "DUNGEON", "STREAK", "CRAFT", "NOTES", "COMPASS", "THEMES", "ABOUT"};
                const char* apps4_other[]  = {"POMODORO", "HABITS", "NOTES",   "STREAK", "CRAFT", "COMPASS", "SETTINGS", "THEMES", "ABOUT"};
                const char** apps4 = (system_state.current_theme == THEME_SUNG_JINWOO) ? apps4_jinwoo : apps4_other;
                appName = apps4[i];
            }

            Serial.printf("[NAV] >>> OPENING APP: %s <<<\n", appName);
            openApp(appName);
            return;
        }
    }

    Serial.println("[NAV] Tap missed all app icons");
}

void openApp(const char* appName) {
    Serial.printf("[NAV] Opening app: %s\n", appName);

    navState.navigationLocked = true;

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
    // =========================================================================
    // NEW: STORY MODE APP
    // =========================================================================
    else if (strcmp(appName, "STORY") == 0) {
        setCurrentStory(system_state.current_theme);
        system_state.current_screen = SCREEN_STORY_MENU;
        drawStoryMenu();
    }
    // =========================================================================
    // NEW: COMPANION CARE APP
    // =========================================================================
    else if (strcmp(appName, "CARE") == 0) {
        syncCompanionWithTheme();  // Make sure companion matches current theme
        enterCompanionCareMode();
        system_state.current_screen = SCREEN_COMPANION;
        drawCompanionCareScreen();
    }
    else if (strcmp(appName, "COMPASS") == 0) {
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
    else if (strcmp(appName, "GALLERY") == 0) {
        system_state.current_screen = SCREEN_GALLERY;
        initGalleryApp();
        drawGalleryApp();
    }
    else if (strcmp(appName, "ELEMENTS") == 0) {
        if (system_state.current_theme == THEME_BOBOIBOY) {
            system_state.current_screen = SCREEN_ELEMENT_TREE;
            initBoboiboyElements();
            drawElementTree();
        } else {
            system_state.current_screen = SCREEN_QUESTS;
            drawDailyQuestsScreen();
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
        forceThemeSelectorRedraw();  // Ensure clean first draw
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
        if (system_state.current_theme == THEME_BOBOIBOY) {
            system_state.current_screen = SCREEN_FUSION_GAME;
            initFusionGame();
            drawFusionGame();
        } else {
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
        gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
        for (int x = 0; x < LCD_WIDTH; x += 8) {
            gfx->fillRect(x, 46, 6, 3, theme->primary);
        }
        gfx->setTextColor(theme->primary);
        gfx->setTextSize(2);
        gfx->setCursor(LCD_WIDTH/2 - 60, 14);
        gfx->print("OTA UPDATE");

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

        gfx->fillRect(LCD_WIDTH/2 - 80, 185, 160, 45, theme->primary);
        gfx->drawRect(LCD_WIDTH/2 - 80, 185, 160, 45, 0xFFFF);
        gfx->fillRect(LCD_WIDTH/2 - 80, 185, 5, 5, 0xFFFF);
        gfx->setTextColor(0xFFFF);
        gfx->setTextSize(2);
        gfx->setCursor(LCD_WIDTH/2 - 55, 198);
        gfx->print("Check Now");

        drawSwipeIndicator();
    }
    else if (strcmp(appName, "BACKUP") == 0) {
        system_state.current_screen = SCREEN_SD_BACKUP;
        showSDBackupMenu();
    }
    else if (strcmp(appName, "ABOUT") == 0) {
        system_state.current_screen = SCREEN_SETTINGS;
        drawAboutScreen();
    }
    else if (strcmp(appName, "WALLPAPER") == 0) {
        initWallpaperSelector();
        drawWallpaperSelector();
    }
    // =========================================================================
    // NEW APPS: POMODORO, HABITS, DUNGEON, STREAK, CRAFT
    // =========================================================================
    else if (strcmp(appName, "POMODORO") == 0) {
        system_state.current_screen = SCREEN_POMODORO;
        extern void initPomodoroApp();
        extern void drawPomodoroApp();
        initPomodoroApp();
        drawPomodoroApp();
    }
    else if (strcmp(appName, "HABITS") == 0) {
        system_state.current_screen = SCREEN_HABITS;
        extern void initHabitsApp();
        extern void drawHabitsApp();
        initHabitsApp();
        drawHabitsApp();
    }
    else if (strcmp(appName, "DUNGEON") == 0) {
        system_state.current_screen = SCREEN_DUNGEON;
        extern void initDungeonApp();
        extern void drawDungeonApp();
        initDungeonApp();
        drawDungeonApp();
    }
    else if (strcmp(appName, "STREAK") == 0) {
        system_state.current_screen = SCREEN_SHOP;
        extern void drawStreakApp();
        drawStreakApp();
    }
    else if (strcmp(appName, "CRAFT") == 0) {
        system_state.current_screen = SCREEN_GACHA;
        extern void initCraftApp();
        extern void drawCraftApp();
        initCraftApp();
        drawCraftApp();
    }
    else if (strcmp(appName, "NOTES") == 0) {
        system_state.current_screen = SCREEN_SETTINGS;
        navState.navigationLocked = true;
        gfx->fillScreen(RGB565(2, 2, 5));
        gfx->setTextColor(getCurrentTheme()->primary);
        gfx->setTextSize(2);
        gfx->setCursor(LCD_WIDTH/2 - 42, 200);
        gfx->print("NOTES");
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(100, 105, 120));
        gfx->setCursor(LCD_WIDTH/2 - 45, 230);
        gfx->print("Coming soon!");
        drawSwipeIndicator();
    }
    else {
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
    drawSwipeIndicator();
}

void drawSwipeIndicator() {
    int barW = 40;
    int barH = 4;
    int barX = (LCD_WIDTH - barW) / 2;
    int barY = LCD_HEIGHT - 10;

    gfx->fillRect(barX, barY, barW, barH, RGB565(50, 55, 70));
    gfx->drawFastHLine(barX + 2, barY, barW - 4, RGB565(80, 85, 100));
}

// Helper functions
void drawStatBar(int x, int y, const char* name, int value, uint16_t color) {
    int maxWidth = 200;
    int barWidth = (value * maxWidth) / 100;

    gfx->setTextColor(0xFFFF);
    gfx->setTextSize(1);
    gfx->setCursor(x, y);
    gfx->print(name);

    gfx->fillRect(x + 80, y - 2, maxWidth, 12, RGB565(8, 10, 14));
    gfx->drawRect(x + 80, y - 2, maxWidth, 12, RGB565(30, 35, 50));

    if (barWidth > 0) {
        gfx->fillRect(x + 81, y - 1, barWidth - 2, 10, color);
    }

    gfx->setCursor(x + 290, y);
    gfx->printf("%d", value);
}

int getXPForNextLevel() {
    return system_state.player_level * 100 + 100;
}
