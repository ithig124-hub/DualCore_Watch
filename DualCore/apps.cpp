/*
 * apps.cpp - Application Implementation
 * Music, Notes, Files, Settings, Stopwatch, Wallpaper
 */

#include "apps.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "hardware.h"
#include "filesystem.h"
#include "navigation.h"
#include "wifi_apps.h"
#include "ui.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

int current_app_index = 0;
int num_registered_apps = 10;

// Stopwatch state
static bool stopwatch_mode = true;  // true = stopwatch, false = timer
static unsigned long lap_times[10];
static int lap_count = 0;

// Timer state (moved here for early access)
static unsigned long timer_start_ms = 0;
static unsigned long timer_elapsed_ms = 0;
static bool timer_running = false;
static bool timer_mode_stopwatch = true;
static unsigned long timer_countdown_set = 60000;

// Calculator state
static float calc_result = 0;
static String calc_display = "0";
static char calc_operator = 0;
static float calc_operand = 0;

// Flashlight state
static bool flashlight_on = false;

// Music state
static int current_track = 0;
static bool is_playing = false;

// App registry
WatchApp registered_apps[] = {
  {APP_WATCHFACE, "Watch", "", COLOR_WHITE, true, nullptr, nullptr, nullptr, nullptr},
  {APP_MUSIC, "Music", "", COLOR_GREEN, true, initMusicApp, drawMusicApp, handleMusicTouch, nullptr},
  {APP_NOTES, "Notes", "", COLOR_YELLOW, true, initNotesApp, drawNotesApp, handleNotesTouch, nullptr},
  {APP_FILES, "Files", "", COLOR_BLUE, true, initFileBrowserApp, drawFileBrowserApp, handleFileBrowserTouch, nullptr},
  {APP_SETTINGS, "Settings", "", COLOR_GRAY, true, initSettingsApp, drawSettingsApp, handleSettingsTouch, nullptr},
  {APP_WEATHER, "Weather", "", COLOR_CYAN, true, nullptr, nullptr, nullptr, nullptr},
  {APP_GAMES, "Games", "", COLOR_ORANGE, true, nullptr, nullptr, nullptr, nullptr},
  {APP_CALCULATOR, "Calc", "", COLOR_PURPLE, true, initCalculatorApp, drawCalculatorApp, handleCalculatorTouch, nullptr},
  {APP_FLASHLIGHT, "Light", "", COLOR_WHITE, true, initFlashlightApp, drawFlashlightApp, handleFlashlightTouch, nullptr},
  {APP_WALLPAPER_SELECTOR, "Wallpaper", "", COLOR_PINK, true, initWallpaperSelector, drawWallpaperSelector, handleWallpaperTouch, nullptr}
};

// =============================================================================
// APP MANAGEMENT
// =============================================================================

void initializeApps() {
  Serial.println("[Apps] Initializing app framework...");
}

void launchApp(AppType app) {
  system_state.current_app = app;
  for (int i = 0; i < num_registered_apps; i++) {
    if (registered_apps[i].type == app && registered_apps[i].init_func) {
      registered_apps[i].init_func();
      break;
    }
  }
}

void exitCurrentApp() {
  system_state.current_screen = SCREEN_APP_GRID;
}

void drawAppGrid() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header - optimized for 410x502
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 24, 18);
  gfx->print("APPS");
  
  // App grid - 3x3 with larger icons for bigger display (410x502)
  int iconSize = 115;  // Larger icons
  int gridStartX = (LCD_WIDTH - (3 * iconSize + 2 * 10)) / 2;  // Center the grid
  int gridStartY = headerH + 15;
  int gap = 10;
  
  for (int i = 0; i < min(num_registered_apps, 9); i++) {
    int x = (i % 3) * (iconSize + gap) + gridStartX;
    int y = (i / 3) * (iconSize + gap) + gridStartY;
    drawAppIcon(x, y, iconSize, registered_apps[i]);
  }
}

void drawAppIcon(int x, int y, int size, WatchApp& app) {
  // Enhanced app icon with better visuals
  gfx->fillRect(x, y, size, size, RGB565(12, 14, 20));
  gfx->drawRect(x, y, size, size, RGB565(40, 45, 60));
  
  // Corner accents
  gfx->fillRect(x, y, 6, 6, app.icon_color);
  gfx->fillRect(x + size - 6, y, 6, 6, app.icon_color);
  gfx->fillRect(x, y + size - 6, 6, 6, app.icon_color);
  gfx->fillRect(x + size - 6, y + size - 6, 6, 6, app.icon_color);
  
  // Top accent bar
  gfx->fillRect(x + 2, y + 2, size - 4, 4, app.icon_color);
  
  // Center icon area with glow
  int iconCenterY = y + size/2 - 10;
  gfx->fillCircle(x + size/2, iconCenterY, 20, RGB565(25, 28, 35));
  gfx->fillCircle(x + size/2, iconCenterY, 15, app.icon_color);
  
  // App name - larger text for bigger icons
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(size > 100 ? 2 : 1);  // Larger text for bigger icons
  int textW = app.name.length() * (size > 100 ? 12 : 6);
  gfx->setCursor(x + (size - textW) / 2, y + size - 25);
  gfx->print(app.name);
}

void handleAppGridTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Match the new grid layout
  int iconSize = 115;
  int gridStartX = (LCD_WIDTH - (3 * iconSize + 2 * 10)) / 2;
  int gridStartY = 70;  // headerH + 15
  int gap = 10;
  
  for (int i = 0; i < min(num_registered_apps, 9); i++) {
    int bx = (i % 3) * (iconSize + gap) + gridStartX;
    int by = (i / 3) * (iconSize + gap) + gridStartY;
    if (x >= bx && x < bx + iconSize && y >= by && y < by + iconSize) {
      launchApp(registered_apps[i].type);
      return;
    }
  }
}

void switchToApp(AppType app) {
  launchApp(app);
}

// =============================================================================
// MUSIC PLAYER
// =============================================================================

void initMusicApp() {
  system_state.current_screen = SCREEN_MUSIC;
}

void drawMusicApp() {
  // ========================================
  // RETRO ANIME MUSIC PLAYER - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 48 + 2, 14);
  gfx->print("MUSIC");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH/2 - 48, 12);
  gfx->print("MUSIC");
  
  // Album art - larger for bigger display
  int artSize = 180;
  int artX = (LCD_WIDTH - artSize) / 2;
  int artY = 75;
  gfx->fillRect(artX, artY, artSize, artSize, RGB565(12, 14, 20));
  gfx->drawRect(artX, artY, artSize, artSize, RGB565(40, 45, 60));
  gfx->fillRect(artX, artY, 8, 8, theme->primary);
  gfx->fillRect(artX + artSize - 8, artY, 8, 8, theme->primary);
  gfx->fillRect(artX, artY + artSize - 8, 8, 8, theme->primary);
  gfx->fillRect(artX + artSize - 8, artY + artSize - 8, 8, 8, theme->primary);
  
  // Vinyl disc - pixel style (larger)
  int discSize = 70;
  gfx->fillRect(LCD_WIDTH/2 - discSize/2, artY + (artSize - discSize)/2, discSize, discSize, theme->primary);
  gfx->fillRect(LCD_WIDTH/2 - 12, artY + (artSize - 24)/2, 24, 24, RGB565(12, 14, 20));
  
  // CRT lines on art
  for (int sy = artY + 2; sy < artY + artSize - 2; sy += 4) {
    gfx->drawFastHLine(artX + 2, sy, artSize - 4, RGB565(6, 6, 10));
  }
  
  // Track info - larger text
  int infoY = artY + artSize + 25;
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 60, infoY);
  gfx->printf("Track %d", current_track + 1);
  
  gfx->setTextSize(2);
  gfx->setTextColor(is_playing ? RGB565(0, 200, 80) : RGB565(80, 85, 100));
  gfx->setCursor(LCD_WIDTH/2 - 40, infoY + 35);
  gfx->print(is_playing ? "PLAYING" : "PAUSED");
  
  // Progress bar - wider
  int barY = infoY + 70;
  int barW = LCD_WIDTH - 60;
  int barX = 30;
  gfx->fillRect(barX, barY, barW, 10, RGB565(15, 18, 25));
  gfx->drawRect(barX, barY, barW, 10, RGB565(30, 35, 50));
  gfx->fillRect(42, 272, (LCD_WIDTH - 84) / 3, 2, theme->primary);
  
  // Controls - retro pixel buttons
  int btnY = 295;
  // Prev
  gfx->fillRect(40, btnY, 75, 40, RGB565(15, 18, 25));
  gfx->drawRect(40, btnY, 75, 40, RGB565(40, 45, 60));
  gfx->fillRect(40, btnY, 4, 4, theme->accent);
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(2);
  gfx->setCursor(58, btnY + 12);
  gfx->print("<<");
  
  // Play/Pause
  gfx->fillRect(LCD_WIDTH/2 - 40, btnY, 80, 40, theme->primary);
  gfx->drawRect(LCD_WIDTH/2 - 40, btnY, 80, 40, RGB565(60, 65, 80));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 18, btnY + 12);
  gfx->print(is_playing ? "||" : " >");
  
  // Next
  gfx->fillRect(LCD_WIDTH - 115, btnY, 75, 40, RGB565(15, 18, 25));
  gfx->drawRect(LCD_WIDTH - 115, btnY, 75, 40, RGB565(40, 45, 60));
  gfx->fillRect(LCD_WIDTH - 44, btnY, 4, 4, theme->accent);
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setCursor(LCD_WIDTH - 97, btnY + 12);
  gfx->print(">>");
  
  drawSwipeIndicator();
}

void handleMusicTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Control buttons at y=320, height=45
  if (y >= 320 && y < 365) {
    if (x < 130) { previousTrack(); drawMusicApp(); }
    else if (x < 250) { is_playing ? pauseMusic() : playMusic(); drawMusicApp(); }
    else { nextTrack(); drawMusicApp(); }
  }
}

void playMusic() { is_playing = true; system_state.music_playing = true; }
void pauseMusic() { is_playing = false; system_state.music_playing = false; }
void nextTrack() { current_track = (current_track + 1) % max(1, system_state.total_mp3_files); }
void previousTrack() { current_track = (current_track - 1 + max(1, system_state.total_mp3_files)) % max(1, system_state.total_mp3_files); }

// =============================================================================
// NOTES APP
// =============================================================================

static String notes[5] = {"Note 1", "Note 2", "Note 3", "Note 4", "Note 5"};
static int selected_note = 0;

void initNotesApp() {
  system_state.current_screen = SCREEN_NOTES;
}

void drawNotesApp() {
  // ========================================
  // RETRO ANIME NOTES - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(255, 200, 60));
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 45 + 2, 14);
  gfx->print("NOTES");
  gfx->setTextColor(RGB565(255, 200, 60));
  gfx->setCursor(LCD_WIDTH/2 - 45, 12);
  gfx->print("NOTES");
  
  // Note items - larger for 410x502
  int noteH = 70;
  int noteGap = 10;
  int startY = headerH + 12;
  
  for (int i = 0; i < 5; i++) {
    int y = startY + i * (noteH + noteGap);
    bool sel = (i == selected_note);
    uint16_t bg = sel ? RGB565(18, 22, 32) : RGB565(12, 14, 20);
    gfx->fillRect(20, y, LCD_WIDTH - 40, noteH, bg);
    gfx->drawRect(20, y, LCD_WIDTH - 40, noteH, sel ? RGB565(255, 200, 60) : RGB565(35, 40, 55));
    // Pixel corner accents
    uint16_t corner = sel ? RGB565(255, 200, 60) : RGB565(50, 55, 70);
    gfx->fillRect(20, y, 5, 5, corner);
    gfx->fillRect(LCD_WIDTH - 25, y, 5, 5, corner);
    
    gfx->setTextColor(sel ? RGB565(255, 200, 60) : RGB565(150, 155, 170));
    gfx->setTextSize(2);
    gfx->setCursor(40, y + 25);
    gfx->print(notes[i]);
  }
  
  drawSwipeIndicator();
}

void handleNotesTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int y = gesture.y;
  
  int noteH = 70;
  int noteGap = 10;
  int startY = 67;  // headerH + 12
  
  for (int i = 0; i < 5; i++) {
    int ny = startY + i * (noteH + noteGap);
    if (y >= ny && y < ny + noteH) {
      selected_note = i;
      drawNotesApp();
      return;
    }
  }
}

void saveNote() {}
void loadNote() {}

// =============================================================================
// FILE BROWSER
// =============================================================================

void initFileBrowserApp() {
  system_state.current_screen = SCREEN_FILE_BROWSER;
}

void drawFileBrowserApp() {
  // ========================================
  // RETRO ANIME FILE BROWSER - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  uint16_t fileBlue = RGB565(80, 180, 255);
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, fileBlue);
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 42 + 2, 14);
  gfx->print("FILES");
  gfx->setTextColor(fileBlue);
  gfx->setCursor(LCD_WIDTH/2 - 42, 12);
  gfx->print("FILES");
  
  // SD card status - larger retro card
  int cardY = headerH + 12;
  gfx->fillRect(20, cardY, LCD_WIDTH - 40, 90, RGB565(12, 14, 20));
  gfx->drawRect(20, cardY, LCD_WIDTH - 40, 90, RGB565(40, 45, 60));
  gfx->fillRect(20, cardY, 6, 6, fileBlue);
  gfx->fillRect(LCD_WIDTH - 26, cardY, 6, 6, fileBlue);
  
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(3);
  gfx->setCursor(40, cardY + 20);
  gfx->print("SD Card");
  
  gfx->setTextSize(2);
  gfx->setTextColor(system_state.filesystem_available ? RGB565(0, 200, 80) : RGB565(200, 60, 60));
  gfx->setCursor(40, cardY + 55);
  gfx->print(system_state.filesystem_available ? "CONNECTED" : "NOT FOUND");
  
  // File counts - larger retro card
  int countY = cardY + 105;
  gfx->fillRect(20, countY, LCD_WIDTH - 40, 120, RGB565(12, 14, 20));
  gfx->drawRect(20, countY, LCD_WIDTH - 40, 120, RGB565(40, 45, 60));
  gfx->fillRect(20, countY, 6, 6, theme->accent);
  gfx->fillRect(LCD_WIDTH - 26, countY, 6, 6, theme->accent);
  
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(2);
  gfx->setCursor(40, countY + 20);
  gfx->printf("MP3 FILES: %d", system_state.total_mp3_files);
  gfx->setCursor(40, countY + 50);
  gfx->printf("PDF FILES: %d", system_state.total_pdf_files);
  gfx->setCursor(40, countY + 80);
  gfx->print("IMAGES:    --");
  
  drawSwipeIndicator();
}

void handleFileBrowserTouch(TouchGesture& gesture) {
}

// =============================================================================
// PDF READER
// =============================================================================

void initPDFReaderApp() {
  system_state.current_screen = SCREEN_PDF_READER;
}

void drawPDFReaderApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(80, 14);
  gfx->print("PDF READER");
  
  gfx->fillRect(60, 180, 240, 40, RGB565(12, 14, 20));
  gfx->drawRect(60, 180, 240, 40, RGB565(40, 45, 60));
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(1);
  gfx->setCursor(110, 196);
  gfx->print("No PDF loaded");
  
  drawSwipeIndicator();
}

void handlePDFReaderTouch(TouchGesture& gesture) {
}

// =============================================================================
// SETTINGS
// =============================================================================

void initSettingsApp() {
  system_state.current_screen = SCREEN_SETTINGS;
}

void drawSettingsApp() {
  // ========================================
  // RETRO ANIME SETTINGS - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 72 + 2, 14);
  gfx->print("SETTINGS");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 72, 12);
  gfx->print("SETTINGS");
  
  // Settings items - larger retro cards for 410x502
  const char* options[] = {"Brightness", "Theme", "WiFi", "About"};
  const char* subtexts[] = {"Adjust display", "Change character", "Network setup", "Version info"};
  uint16_t icons[] = {RGB565(255, 200, 60), theme->primary, RGB565(80, 180, 255), RGB565(130, 135, 150)};
  
  int cardH = 85;
  int cardGap = 12;
  int startY = headerH + 12;
  
  for (int i = 0; i < 4; i++) {
    int y = startY + i * (cardH + cardGap);
    
    // Retro card - larger
    gfx->fillRect(20, y, LCD_WIDTH - 40, cardH, RGB565(12, 14, 20));
    gfx->drawRect(20, y, LCD_WIDTH - 40, cardH, RGB565(40, 45, 60));
    // Pixel corner with icon color
    gfx->fillRect(20, y, 6, 6, icons[i]);
    gfx->fillRect(LCD_WIDTH - 26, y, 6, 6, icons[i]);
    // Color stripe at left
    gfx->fillRect(20, y + 2, 5, cardH - 4, icons[i]);
    
    // Icon pixel square - larger
    gfx->fillRect(38, y + 20, 40, 40, RGB565(15, 18, 25));
    gfx->drawRect(38, y + 20, 40, 40, icons[i]);
    gfx->fillRect(48, y + 30, 20, 20, icons[i]);
    
    // Text - larger
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(3);
    gfx->setCursor(95, y + 18);
    gfx->print(options[i]);
    
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(95, y + 52);
    gfx->print(subtexts[i]);
    
    // Arrow - retro
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(3);
    gfx->setCursor(LCD_WIDTH - 55, y + 28);
    gfx->print(">");
  }
  
  drawSwipeIndicator();
}

void handleSettingsTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int y = gesture.y;
  
  int cardH = 85;
  int cardGap = 12;
  int startY = 67;  // headerH + 12
  
  // Cards at: y=67, y=164, y=261, y=358
  for (int i = 0; i < 4; i++) {
    int cardY = startY + i * (cardH + cardGap);
    if (y >= cardY && y < cardY + cardH) {
      switch(i) {
        case 0:  // Brightness - cycle
          system_state.brightness = (system_state.brightness + 50) % 256;
          setDisplayBrightness(system_state.brightness);
          drawSettingsApp();
          break;
        case 1:  // Theme selector
          system_state.current_screen = SCREEN_THEME_SELECTOR;
          drawThemeSelector();
          break;
        case 2:  // WiFi
          system_state.current_screen = SCREEN_WIFI_MANAGER;
          drawNetworkListScreen();
          break;
        case 3:  // About
          system_state.current_screen = SCREEN_SETTINGS;
          drawAboutScreen();
          break;
      }
      return;
    }
  }
}

// =============================================================================
// STOPWATCH/TIMER
// =============================================================================

void initStopwatchTimerApp() {
  lap_count = 0;
}

void drawStopwatchTimerApp() {
  if (stopwatch_mode) drawStopwatchMode();
  else drawTimerMode();
}

void handleStopwatchTimerTouch(TouchGesture& gesture) {
  if (stopwatch_mode) handleStopwatchTouch(gesture);
  else handleTimerTouch(gesture);
}

void updateStopwatchApp() {
  // Update timer countdown if running
}

void drawStopwatchMode() {
  // ========================================
  // RETRO ANIME STOPWATCH - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
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
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 82 + 2, 14);
  gfx->print("STOPWATCH");
  gfx->setTextColor(theme->accent);
  gfx->setCursor(LCD_WIDTH/2 - 82, 12);
  gfx->print("STOPWATCH");
  
  // Time display - larger retro pixel frame
  unsigned long t = getStopwatchTime();
  int mins = (t / 60000) % 60;
  int secs = (t / 1000) % 60;
  int ms = (t % 1000) / 10;
  
  int timeBoxY = headerH + 15;
  gfx->fillRect(20, timeBoxY, LCD_WIDTH - 40, 100, RGB565(12, 14, 20));
  gfx->drawRect(20, timeBoxY, LCD_WIDTH - 40, 100, RGB565(40, 45, 60));
  gfx->fillRect(20, timeBoxY, 8, 8, theme->accent);
  gfx->fillRect(LCD_WIDTH - 28, timeBoxY, 8, 8, theme->accent);
  gfx->fillRect(20, timeBoxY + 92, 8, 8, theme->accent);
  gfx->fillRect(LCD_WIDTH - 28, timeBoxY + 92, 8, 8, theme->accent);
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d.%02d", mins, secs, ms);
  gfx->setTextSize(5);
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setCursor(50, timeBoxY + 30);
  gfx->print(timeStr);
  
  // Lap times - larger retro terminal style
  int lapBoxY = timeBoxY + 115;
  gfx->fillRect(20, lapBoxY, LCD_WIDTH - 40, 150, RGB565(8, 10, 14));
  gfx->drawRect(20, lapBoxY, LCD_WIDTH - 40, 150, RGB565(30, 35, 50));
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(0, 200, 80));
  for (int i = 0; i < min(lap_count, 5); i++) {
    int lapMins = (lap_times[i] / 60000) % 60;
    int lapSecs = (lap_times[i] / 1000) % 60;
    gfx->setCursor(35, lapBoxY + 15 + i * 28);
    gfx->printf("> Lap %d: %02d:%02d", i + 1, lapMins, lapSecs);
  }
  
  // Control buttons - larger for 410x502
  int btnY = lapBoxY + 165;
  int btnW = 110;
  int btnH = 50;
  int btnGap = 15;
  int btnStartX = (LCD_WIDTH - (3 * btnW + 2 * btnGap)) / 2;
  
  // Start
  gfx->fillRect(btnStartX, btnY, btnW, btnH, RGB565(10, 50, 30));
  gfx->drawRect(btnStartX, btnY, btnW, btnH, RGB565(0, 200, 80));
  gfx->fillRect(btnStartX, btnY, 5, 5, RGB565(0, 200, 80));
  gfx->setTextColor(RGB565(0, 200, 80));
  gfx->setTextSize(2);
  gfx->setCursor(btnStartX + 22, btnY + 15);
  gfx->print("Start");
  
  // Lap
  int btn2X = btnStartX + btnW + btnGap;
  gfx->fillRect(btn2X, btnY, btnW, btnH, RGB565(15, 18, 25));
  gfx->drawRect(btn2X, btnY, btnW, btnH, RGB565(40, 45, 60));
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setCursor(btn2X + 35, btnY + 15);
  gfx->print("Lap");
  
  // Reset
  int btn3X = btn2X + btnW + btnGap;
  gfx->fillRect(btn3X, btnY, btnW, btnH, RGB565(50, 15, 15));
  gfx->drawRect(btn3X, btnY, btnW, btnH, RGB565(200, 60, 60));
  gfx->fillRect(btn3X + btnW - 5, btnY, 5, 5, RGB565(200, 60, 60));
  gfx->setTextColor(RGB565(200, 60, 60));
  gfx->setCursor(btn3X + 18, btnY + 15);
  gfx->print("Reset");
  
  drawSwipeIndicator();
}

void drawTimerMode() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 45, 14);
  gfx->print("TIMER");
  
  int remaining = getTimerRemaining();
  int mins = remaining / 60;
  int secs = remaining % 60;
  
  // Time display - larger retro framed
  int timeBoxY = 170;
  gfx->fillRect(30, timeBoxY, LCD_WIDTH - 60, 110, RGB565(12, 14, 20));
  gfx->drawRect(30, timeBoxY, LCD_WIDTH - 60, 110, RGB565(40, 45, 60));
  gfx->fillRect(30, timeBoxY, 8, 8, theme->primary);
  gfx->fillRect(LCD_WIDTH - 38, timeBoxY, 8, 8, theme->primary);
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", mins, secs);
  gfx->setTextSize(7);
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setCursor(70, timeBoxY + 25);
  gfx->print(timeStr);
  
  drawSwipeIndicator();
}

void handleStopwatchTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Updated button positions for 410x502
  int btnY = 405;  // lapBoxY + 165
  int btnH = 50;
  int btnW = 110;
  int btnGap = 15;
  int btnStartX = (LCD_WIDTH - (3 * btnW + 2 * btnGap)) / 2;
  
  if (y >= btnY && y < btnY + btnH) {
    static bool running = false;
    if (x >= btnStartX && x < btnStartX + btnW) {
      // Start/Stop
      if (running) pauseStopwatch();
      else startStopwatch();
      running = !running;
    }
    else if (x >= btnStartX + btnW + btnGap && x < btnStartX + 2*btnW + btnGap) {
      // Lap
      recordLapTime();
    }
    else if (x >= btnStartX + 2*(btnW + btnGap) && x < btnStartX + 3*btnW + 2*btnGap) {
      // Reset
      resetStopwatch();
    }
    drawStopwatchMode();
  }
}

void handleTimerTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  int centerX = LCD_WIDTH / 2;
  int btnY = 230;
  int btnW = 100;
  int btnH = 50;
  
  // Reset button
  if (x >= 30 && x < 130 && y >= btnY && y < btnY + btnH) {
    timer_running = false;
    timer_elapsed_ms = 0;
    lap_count = 0;
    drawTimerApp();
    return;
  }
  
  // Start/Stop button
  if (x >= centerX - btnW/2 && x < centerX + btnW/2 && y >= btnY && y < btnY + btnH) {
    if (timer_running) {
      timer_elapsed_ms += millis() - timer_start_ms;
      timer_running = false;
    } else {
      timer_start_ms = millis();
      timer_running = true;
    }
    drawTimerApp();
    return;
  }
  
  // Lap button
  if (x >= LCD_WIDTH - 130 && x < LCD_WIDTH - 30 && y >= btnY && y < btnY + btnH) {
    if (timer_running && lap_count < 10) {
      lap_times[lap_count++] = millis() - timer_start_ms + timer_elapsed_ms;
      drawTimerApp();
    }
    return;
  }
}

void recordLapTime() {
  if (lap_count < 10) {
    lap_times[lap_count++] = getStopwatchTime();
  }
}

// =============================================================================
// WALLPAPER SELECTOR
// =============================================================================

static int selected_wallpaper = 0;

void initWallpaperSelector() {
  system_state.current_screen = SCREEN_WALLPAPER_SELECTOR;
}

void drawWallpaperSelector() {
  // ========================================
  // RETRO ANIME WALLPAPERS - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_PINK);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 60 + 1, 14 + 1);
  gfx->print("WALLPAPERS");
  gfx->setTextColor(COLOR_PINK);
  gfx->setCursor(LCD_WIDTH/2 - 60, 14);
  gfx->print("WALLPAPERS");
  
  // Wallpaper options - retro pixel cards
  const char* wallpapers[] = {"Solid Black", "Gradient", "Grid", "Theme Color"};
  uint16_t colors[] = {RGB565(5, 5, 8), COLOR_BLUE, COLOR_GREEN, theme->primary};
  
  for (int i = 0; i < 4; i++) {
    int x = (i % 2) * 175 + 15;
    int y = (i / 2) * 140 + 60;
    
    // Preview area
    gfx->fillRect(x, y, 160, 120, colors[i]);
    gfx->drawRect(x, y, 160, 120, RGB565(40, 45, 60));
    // Pixel corners
    gfx->fillRect(x, y, 6, 6, COLOR_PINK);
    gfx->fillRect(x + 154, y, 6, 6, COLOR_PINK);
    gfx->fillRect(x, y + 114, 6, 6, COLOR_PINK);
    gfx->fillRect(x + 154, y + 114, 6, 6, COLOR_PINK);
    
    // CRT effect on preview
    for (int sy = y + 2; sy < y + 118; sy += 4) {
      gfx->drawFastHLine(x + 2, sy, 156, RGB565(0, 0, 0));
    }
    
    if (i == selected_wallpaper) {
      gfx->drawRect(x - 2, y - 2, 164, 124, COLOR_WHITE);
      gfx->drawRect(x - 1, y - 1, 162, 122, COLOR_WHITE);
    }
    
    // Label
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setTextSize(1);
    gfx->setCursor(x + 10, y + 105);
    gfx->print(wallpapers[i]);
  }
  
  drawSwipeIndicator();
}

void handleWallpaperTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  for (int i = 0; i < 4; i++) {
    int wx = (i % 2) * 175 + 15;
    int wy = (i / 2) * 150 + 80;
    if (x >= wx && x < wx + 160 && y >= wy && y < wy + 130) {
      selectWallpaper(i);
      return;
    }
  }
  
}

void selectWallpaper(int index) {
  selected_wallpaper = index;
  system_state.wallpaper_enabled = (index != 0);
}

void applyWallpaper(const String& path) {
  system_state.current_wallpaper_path = path;
}

// =============================================================================
// CALCULATOR
// =============================================================================

void initCalculatorApp() {}

void drawCalculatorApp() {
  // ========================================
  // RETRO ANIME CALCULATOR - CRT Style (Optimized for 410x502)
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(100, 100, 120));
  }
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 36, 14);
  gfx->print("CALC");
  
  // Display panel - larger retro terminal
  int displayY = headerH + 12;
  gfx->fillRect(16, displayY, LCD_WIDTH - 32, 70, RGB565(8, 10, 14));
  gfx->drawRect(16, displayY, LCD_WIDTH - 32, 70, RGB565(40, 45, 60));
  gfx->fillRect(16, displayY, 6, 6, theme->primary);
  gfx->fillRect(LCD_WIDTH - 22, displayY, 6, 6, theme->primary);
  gfx->setTextColor(RGB565(0, 200, 80));
  gfx->setTextSize(4);
  gfx->setCursor(30, displayY + 20);
  gfx->print(calc_display);
  
  // Calculator buttons - larger for 410x502
  const char* buttons[] = {"7", "8", "9", "/", "4", "5", "6", "*", "1", "2", "3", "-", "C", "0", "=", "+"};
  int btnStartY = displayY + 85;
  int btnW = 90;
  int btnH = 70;
  int btnGapX = 8;
  int btnGapY = 8;
  int btnStartX = (LCD_WIDTH - (4 * btnW + 3 * btnGapX)) / 2;
  
  for (int i = 0; i < 16; i++) {
    int col = i % 4;
    int row = i / 4;
    int bx = btnStartX + col * (btnW + btnGapX);
    int by = btnStartY + row * (btnH + btnGapY);
    
    bool isOp = (i % 4 == 3) || buttons[i][0] == 'C' || buttons[i][0] == '=';
    uint16_t bg = isOp ? RGB565(18, 15, 28) : RGB565(12, 14, 20);
    uint16_t border = isOp ? theme->primary : RGB565(35, 40, 55);
    
    gfx->fillRect(bx, by, btnW, btnH, bg);
    gfx->drawRect(bx, by, btnW, btnH, border);
    // Pixel corners on operator buttons
    if (isOp) {
      gfx->fillRect(bx, by, 5, 5, theme->primary);
      gfx->fillRect(bx + btnW - 5, by, 5, 5, theme->primary);
    }
    
    gfx->setTextColor(isOp ? theme->primary : RGB565(200, 205, 220));
    gfx->setTextSize(3);
    gfx->setCursor(bx + btnW/2 - 9, by + btnH/2 - 12);
    gfx->print(buttons[i]);
  }
  
  drawSwipeIndicator();
}

void handleCalculatorTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Check calculator buttons - updated for 410x502
  const char* buttons[] = {"7", "8", "9", "/", "4", "5", "6", "*", "1", "2", "3", "-", "C", "0", "=", "+"};
  int btnStartY = 152;  // displayY + 85
  int btnW = 90;
  int btnH = 70;
  int btnGapX = 8;
  int btnGapY = 8;
  int btnStartX = (LCD_WIDTH - (4 * btnW + 3 * btnGapX)) / 2;
  
  for (int i = 0; i < 16; i++) {
    int col = i % 4;
    int row = i / 4;
    int bx = btnStartX + col * (btnW + btnGapX);
    int by = btnStartY + row * (btnH + btnGapY);
    
    if (x >= bx && x < bx + btnW && y >= by && y < by + btnH) {
      char btn = buttons[i][0];
      if (btn >= '0' && btn <= '9') {
        if (calc_display == "0") calc_display = String(btn);
        else calc_display += btn;
      } else if (btn == 'C') {
        calc_display = "0";
        calc_result = 0;
        calc_operator = 0;
      } else if (btn == '=') {
        float num = calc_display.toFloat();
        switch(calc_operator) {
          case '+': calc_result += num; break;
          case '-': calc_result -= num; break;
          case '*': calc_result *= num; break;
          case '/': if (num != 0) calc_result /= num; break;
        }
        calc_display = String(calc_result);
        calc_operator = 0;
      } else {
        calc_operand = calc_display.toFloat();
        if (calc_operator == 0) calc_result = calc_operand;
        else {
          switch(calc_operator) {
            case '+': calc_result += calc_operand; break;
            case '-': calc_result -= calc_operand; break;
            case '*': calc_result *= calc_operand; break;
            case '/': if (calc_operand != 0) calc_result /= calc_operand; break;
          }
        }
        calc_operator = btn;
        calc_display = "0";
      }
      drawCalculatorApp();
      return;
    }
  }
}

// =============================================================================
// FLASHLIGHT
// =============================================================================

void initFlashlightApp() {}

void drawFlashlightApp() {
  int centerX = LCD_WIDTH / 2;
  int centerY = 220;  // Adjusted for taller display
  
  if (flashlight_on) {
    // ON state - bright white screen (functional, skip CRT for max brightness)
    gfx->fillScreen(COLOR_WHITE);
    gfx->setTextColor(RGB565(30, 30, 40));
    gfx->setTextSize(3);
    gfx->setCursor(centerX - 45, 35);
    gfx->print("TORCH");
    
    // OFF button - larger pixel square
    gfx->fillRect(centerX - 70, centerY - 40, 140, 80, RGB565(30, 32, 45));
    gfx->drawRect(centerX - 70, centerY - 40, 140, 80, RGB565(60, 62, 75));
    gfx->fillRect(centerX - 70, centerY - 40, 6, 6, RGB565(200, 60, 60));
    gfx->fillRect(centerX + 64, centerY - 40, 6, 6, RGB565(200, 60, 60));
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(centerX - 30, centerY - 15);
    gfx->print("OFF");
    
    gfx->setTextColor(RGB565(80, 80, 90));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 75, 440);
    gfx->print("> TAP TO TOGGLE <");
  } else {
    // OFF state - retro CRT dark
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
      gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }
    
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setTextSize(3);
    gfx->setCursor(centerX - 45, 35);
    gfx->print("TORCH");
    
    // Pixel glow effect around ON button - larger
    gfx->drawRect(centerX - 74, centerY - 44, 148, 88, RGB565(40, 35, 10));
    gfx->drawRect(centerX - 72, centerY - 42, 144, 84, RGB565(60, 50, 15));
    
    // ON button - larger retro pixel
    gfx->fillRect(centerX - 70, centerY - 40, 140, 80, RGB565(200, 160, 40));
    gfx->drawRect(centerX - 70, centerY - 40, 140, 80, COLOR_GOLD);
    gfx->fillRect(centerX - 70, centerY - 40, 8, 8, COLOR_GOLD);
    gfx->fillRect(centerX + 62, centerY - 40, 8, 8, COLOR_GOLD);
    gfx->fillRect(centerX - 70, centerY + 32, 8, 8, COLOR_GOLD);
    gfx->fillRect(centerX + 62, centerY + 32, 8, 8, COLOR_GOLD);
    gfx->setTextColor(RGB565(20, 20, 20));
    gfx->setTextSize(4);
    gfx->setCursor(centerX - 24, centerY - 15);
    gfx->print("ON");
    
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 75, 440);
    gfx->print("> TAP TO TOGGLE <");
  }
}

void handleFlashlightTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_TAP) {
    flashlight_on = !flashlight_on;
    if (flashlight_on) setDisplayBrightness(255);
    else setDisplayBrightness(system_state.brightness);
    drawFlashlightApp();
  }
}

// =============================================================================
// TIMER / STOPWATCH APP
// =============================================================================

void initTimerApp() {
  timer_elapsed_ms = 0;
  timer_running = false;
}

// Update only the time display (called from main loop when timer is running)
void updateTimerDisplay() {
  if (!timer_running) return;
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Calculate display time
  unsigned long display_ms = millis() - timer_start_ms + timer_elapsed_ms;
  
  int mins = (display_ms / 60000) % 60;
  int secs = (display_ms / 1000) % 60;
  int centis = (display_ms / 10) % 100;
  
  // Only update the time display area (not the whole screen)
  gfx->fillRect(31, 101, LCD_WIDTH - 62, 98, RGB565(12, 14, 20));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(6);
  char timeStr[12];
  sprintf(timeStr, "%02d:%02d", mins, secs);
  gfx->setCursor(60, 120);
  gfx->print(timeStr);
  
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(3);
  gfx->setCursor(260, 145);
  gfx->printf(".%02d", centis);
}

void drawTimerApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 48, 14);
  gfx->print(timer_mode_stopwatch ? "STOPWATCH" : "TIMER");
  
  // Calculate display time
  unsigned long display_ms = timer_elapsed_ms;
  if (timer_running) {
    display_ms = millis() - timer_start_ms + timer_elapsed_ms;
  }
  
  int mins = (display_ms / 60000) % 60;
  int secs = (display_ms / 1000) % 60;
  int centis = (display_ms / 10) % 100;
  
  // Big time display
  gfx->fillRect(30, 100, LCD_WIDTH - 60, 100, RGB565(12, 14, 20));
  gfx->drawRect(30, 100, LCD_WIDTH - 60, 100, RGB565(40, 45, 60));
  gfx->fillRect(30, 100, 5, 5, theme->primary);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(6);
  char timeStr[12];
  sprintf(timeStr, "%02d:%02d", mins, secs);
  gfx->setCursor(60, 120);
  gfx->print(timeStr);
  
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(3);
  gfx->setCursor(260, 145);
  gfx->printf(".%02d", centis);
  
  // Control buttons
  int btnY = 230;
  int btnW = 100;
  int btnH = 50;
  
  // Reset button
  gfx->fillRect(30, btnY, btnW, btnH, RGB565(15, 18, 25));
  gfx->drawRect(30, btnY, btnW, btnH, RGB565(60, 65, 80));
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(2);
  gfx->setCursor(52, btnY + 16);
  gfx->print("RESET");
  
  // Start/Stop button
  uint16_t startColor = timer_running ? RGB565(200, 60, 60) : RGB565(60, 200, 80);
  gfx->fillRect(centerX - btnW/2, btnY, btnW, btnH, startColor);
  gfx->drawRect(centerX - btnW/2, btnY, btnW, btnH, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(centerX - 24, btnY + 16);
  gfx->print(timer_running ? "STOP" : "START");
  
  // Lap button (stopwatch) or Mode button
  gfx->fillRect(LCD_WIDTH - 130, btnY, btnW, btnH, RGB565(15, 18, 25));
  gfx->drawRect(LCD_WIDTH - 130, btnY, btnW, btnH, theme->primary);
  gfx->fillRect(LCD_WIDTH - 130, btnY, 4, 4, theme->primary);
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH - 108, btnY + 16);
  gfx->print("LAP");
  
  // Lap times display
  if (lap_count > 0) {
    gfx->fillRect(30, 300, LCD_WIDTH - 60, 100, RGB565(10, 12, 18));
    gfx->drawRect(30, 300, LCD_WIDTH - 60, 100, RGB565(30, 35, 50));
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(150, 155, 170));
    for (int i = 0; i < min(lap_count, 4); i++) {
      int lapMins = (lap_times[i] / 60000) % 60;
      int lapSecs = (lap_times[i] / 1000) % 60;
      int lapCentis = (lap_times[i] / 10) % 100;
      gfx->setCursor(45, 310 + i * 22);
      gfx->printf("Lap %d:  %02d:%02d.%02d", lap_count - i, lapMins, lapSecs, lapCentis);
    }
  }
  
  drawSwipeIndicator();
}

// =============================================================================
// UNIT CONVERTER APP
// =============================================================================

static int converter_mode = 0;  // 0=length, 1=weight, 2=temp
static float converter_input = 1.0;
static int converter_unit_from = 0;
static int converter_unit_to = 1;

void initConverterApp() {
  converter_input = 1.0;
}

void drawConverterApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 48, 14);
  gfx->print("CONVERTER");
  
  // Mode tabs
  const char* modes[] = {"LENGTH", "WEIGHT", "TEMP"};
  for (int i = 0; i < 3; i++) {
    int tx = 25 + i * 115;
    bool selected = (i == converter_mode);
    gfx->fillRect(tx, 58, 105, 32, selected ? theme->primary : RGB565(15, 18, 25));
    gfx->drawRect(tx, 58, 105, 32, selected ? theme->accent : RGB565(40, 45, 60));
    gfx->setTextColor(selected ? COLOR_WHITE : RGB565(120, 125, 140));
    gfx->setTextSize(1);
    gfx->setCursor(tx + 20, 68);
    gfx->print(modes[i]);
  }
  
  // Units based on mode
  const char* length_units[] = {"m", "km", "ft", "mi", "cm", "in"};
  const char* weight_units[] = {"kg", "lb", "g", "oz"};
  const char* temp_units[] = {"C", "F", "K"};
  
  const char** units;
  int num_units;
  float result;
  
  if (converter_mode == 0) {
    units = length_units;
    num_units = 6;
    // Convert to meters first, then to target
    float to_meters[] = {1.0, 1000.0, 0.3048, 1609.34, 0.01, 0.0254};
    float in_meters = converter_input * to_meters[converter_unit_from];
    result = in_meters / to_meters[converter_unit_to];
  } else if (converter_mode == 1) {
    units = weight_units;
    num_units = 4;
    float to_kg[] = {1.0, 0.453592, 0.001, 0.0283495};
    float in_kg = converter_input * to_kg[converter_unit_from];
    result = in_kg / to_kg[converter_unit_to];
  } else {
    units = temp_units;
    num_units = 3;
    // Temperature conversion
    float celsius;
    if (converter_unit_from == 0) celsius = converter_input;
    else if (converter_unit_from == 1) celsius = (converter_input - 32) * 5.0 / 9.0;
    else celsius = converter_input - 273.15;
    
    if (converter_unit_to == 0) result = celsius;
    else if (converter_unit_to == 1) result = celsius * 9.0 / 5.0 + 32;
    else result = celsius + 273.15;
  }
  
  // Input display
  gfx->fillRect(30, 110, LCD_WIDTH - 60, 60, RGB565(12, 14, 20));
  gfx->drawRect(30, 110, LCD_WIDTH - 60, 60, RGB565(40, 45, 60));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(50, 125);
  gfx->printf("%.2f", converter_input);
  gfx->setTextSize(2);
  gfx->setTextColor(theme->primary);
  gfx->setCursor(250, 130);
  gfx->print(units[converter_unit_from % num_units]);
  
  // Arrow
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(3);
  gfx->setCursor(centerX - 12, 185);
  gfx->print("=");
  
  // Result display
  gfx->fillRect(30, 220, LCD_WIDTH - 60, 60, RGB565(15, 20, 15));
  gfx->drawRect(30, 220, LCD_WIDTH - 60, 60, theme->primary);
  gfx->fillRect(30, 220, 5, 5, theme->primary);
  gfx->setTextColor(RGB565(150, 230, 150));
  gfx->setTextSize(3);
  gfx->setCursor(50, 235);
  gfx->printf("%.2f", result);
  gfx->setTextSize(2);
  gfx->setTextColor(theme->accent);
  gfx->setCursor(250, 240);
  gfx->print(units[converter_unit_to % num_units]);
  
  // Unit selection buttons
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setCursor(50, 300);
  gfx->print("TAP numbers to change units");
  
  // +/- buttons
  gfx->fillRect(50, 330, 80, 45, RGB565(15, 18, 25));
  gfx->drawRect(50, 330, 80, 45, RGB565(60, 65, 80));
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(3);
  gfx->setCursor(80, 340);
  gfx->print("-");
  
  gfx->fillRect(centerX - 40, 330, 80, 45, RGB565(15, 18, 25));
  gfx->drawRect(centerX - 40, 330, 80, 45, theme->primary);
  gfx->setTextColor(theme->primary);
  gfx->setCursor(centerX - 18, 340);
  gfx->print("x10");
  
  gfx->fillRect(LCD_WIDTH - 130, 330, 80, 45, RGB565(15, 18, 25));
  gfx->drawRect(LCD_WIDTH - 130, 330, 80, 45, RGB565(60, 65, 80));
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setCursor(LCD_WIDTH - 100, 340);
  gfx->print("+");
  
  drawSwipeIndicator();
}

void handleConverterTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  int centerX = LCD_WIDTH / 2;
  
  // Mode tabs
  if (y >= 58 && y < 90) {
    for (int i = 0; i < 3; i++) {
      int tx = 25 + i * 115;
      if (x >= tx && x < tx + 105) {
        converter_mode = i;
        converter_unit_from = 0;
        converter_unit_to = 1;
        drawConverterApp();
        return;
      }
    }
  }
  
  // Input area - cycle from unit
  if (y >= 110 && y < 170) {
    int num_units = (converter_mode == 0) ? 6 : (converter_mode == 1) ? 4 : 3;
    converter_unit_from = (converter_unit_from + 1) % num_units;
    drawConverterApp();
    return;
  }
  
  // Result area - cycle to unit
  if (y >= 220 && y < 280) {
    int num_units = (converter_mode == 0) ? 6 : (converter_mode == 1) ? 4 : 3;
    converter_unit_to = (converter_unit_to + 1) % num_units;
    drawConverterApp();
    return;
  }
  
  // +/- buttons
  if (y >= 330 && y < 375) {
    if (x >= 50 && x < 130) {
      converter_input = max(0.01f, converter_input - 1.0f);
      drawConverterApp();
    } else if (x >= centerX - 40 && x < centerX + 40) {
      converter_input *= 10.0f;
      if (converter_input > 999999) converter_input = 1.0f;
      drawConverterApp();
    } else if (x >= LCD_WIDTH - 130 && x < LCD_WIDTH - 50) {
      converter_input += 1.0f;
      drawConverterApp();
    }
  }
}

// =============================================================================
// ACHIEVEMENTS APP
// =============================================================================

typedef struct {
  const char* name;
  const char* desc;
  int progress;
  int target;
  bool unlocked;
} Achievement;

static Achievement achievements[] = {
  {"First Steps", "Walk 1000 steps", 0, 1000, false},
  {"Marathon", "Walk 10000 steps in a day", 0, 10000, false},
  {"Card Collector", "Collect 10 cards", 0, 10, false},
  {"Boss Slayer", "Defeat 5 bosses", 0, 5, false},
  {"Training Pro", "Complete 10 training sessions", 0, 10, false},
  {"Quest Master", "Complete 20 daily quests", 0, 20, false},
  {"Level Up!", "Reach level 10", 0, 10, false},
  {"Gem Hoarder", "Collect 1000 gems", 0, 1000, false}
};
static int achievement_scroll = 0;

void initAchievementsApp() {
  // Update achievement progress from system state
  achievements[0].progress = system_state.steps_today;
  achievements[1].progress = system_state.steps_today;
  achievements[2].progress = system_state.gacha_cards_collected;
  achievements[3].progress = system_state.bosses_defeated;
  achievements[4].progress = system_state.training_streak;
  achievements[6].progress = system_state.player_level;
  achievements[7].progress = system_state.player_gems;
  
  for (int i = 0; i < 8; i++) {
    achievements[i].unlocked = (achievements[i].progress >= achievements[i].target);
  }
}

void drawAchievementsApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_GOLD);
  }
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 60, 14);
  gfx->print("ACHIEVEMENTS");
  
  // Count unlocked
  int unlocked_count = 0;
  for (int i = 0; i < 8; i++) {
    if (achievements[i].unlocked) unlocked_count++;
  }
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(LCD_WIDTH - 50, 18);
  gfx->printf("%d/8", unlocked_count);
  
  // Achievement cards
  for (int i = 0; i < 4; i++) {
    int idx = i + achievement_scroll;
    if (idx >= 8) break;
    
    int cardY = 60 + i * 95;
    bool unlocked = achievements[idx].unlocked;
    
    // Card background
    gfx->fillRect(20, cardY, LCD_WIDTH - 40, 85, unlocked ? RGB565(20, 25, 15) : RGB565(12, 14, 20));
    gfx->drawRect(20, cardY, LCD_WIDTH - 40, 85, unlocked ? COLOR_GOLD : RGB565(40, 45, 60));
    
    if (unlocked) {
      gfx->fillRect(20, cardY, 5, 5, COLOR_GOLD);
      gfx->fillRect(LCD_WIDTH - 25, cardY, 5, 5, COLOR_GOLD);
    }
    
    // Trophy icon for unlocked
    if (unlocked) {
      gfx->fillRect(30, cardY + 10, 30, 25, COLOR_GOLD);
      gfx->fillRect(35, cardY + 35, 20, 8, COLOR_GOLD);
      gfx->fillRect(30, cardY + 43, 30, 5, RGB565(180, 140, 40));
    }
    
    // Achievement name
    gfx->setTextColor(unlocked ? COLOR_GOLD : RGB565(180, 185, 200));
    gfx->setTextSize(2);
    gfx->setCursor(unlocked ? 75 : 35, cardY + 12);
    gfx->print(achievements[idx].name);
    
    // Description
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setTextSize(1);
    gfx->setCursor(unlocked ? 75 : 35, cardY + 38);
    gfx->print(achievements[idx].desc);
    
    // Progress bar
    int barX = 35;
    int barY = cardY + 58;
    int barW = LCD_WIDTH - 90;
    int barH = 12;
    float progress = (float)achievements[idx].progress / achievements[idx].target;
    if (progress > 1.0) progress = 1.0;
    
    gfx->fillRect(barX, barY, barW, barH, RGB565(8, 10, 14));
    gfx->drawRect(barX, barY, barW, barH, RGB565(30, 35, 50));
    int fillW = (int)(barW * progress);
    if (fillW > 0) {
      gfx->fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, unlocked ? COLOR_GOLD : theme->primary);
    }
    
    // Progress text
    gfx->setTextColor(RGB565(150, 155, 170));
    gfx->setCursor(barX + barW + 5, barY + 2);
    gfx->printf("%d/%d", min(achievements[idx].progress, achievements[idx].target), achievements[idx].target);
  }
  
  // Scroll indicator
  if (achievement_scroll > 0 || achievement_scroll < 4) {
    gfx->setTextColor(RGB565(60, 65, 80));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 20, LCD_HEIGHT - 30);
    gfx->print("SCROLL");
  }
  
  drawSwipeIndicator();
}

void handleAchievementsTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_DOWN && achievement_scroll > 0) {
    achievement_scroll--;
    drawAchievementsApp();
  } else if (gesture.event == TOUCH_SWIPE_UP && achievement_scroll < 4) {
    achievement_scroll++;
    drawAchievementsApp();
  }
}

// =============================================================================
// SHOP APP - Simple free gems button
// =============================================================================

void initShopApp() {}

void drawShopApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_GOLD);
  }
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 24, 14);
  gfx->print("SHOP");
  
  // Current gem balance - big display
  gfx->fillRect(60, 70, LCD_WIDTH - 120, 60, RGB565(25, 20, 10));
  gfx->drawRect(60, 70, LCD_WIDTH - 120, 60, COLOR_GOLD);
  gfx->fillRect(60, 70, 5, 5, COLOR_GOLD);
  gfx->fillRect(LCD_WIDTH - 65, 70, 5, 5, COLOR_GOLD);
  
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 30, 78);
  gfx->print("YOUR GEMS");
  
  gfx->setTextSize(4);
  gfx->setCursor(centerX - 60, 95);
  gfx->printf("%d", system_state.player_gems);
  
  // BIG FREE GEMS BUTTON
  int btnX = 50;
  int btnY = 180;
  int btnW = LCD_WIDTH - 100;
  int btnH = 120;
  
  // Button with gradient effect
  for (int i = 0; i < btnH; i++) {
    uint8_t r = 180 - i/2;
    uint8_t g = 120 + i/3;
    uint8_t b = 40;
    gfx->drawFastHLine(btnX, btnY + i, btnW, RGB565(r, g, b));
  }
  gfx->drawRect(btnX, btnY, btnW, btnH, COLOR_WHITE);
  gfx->drawRect(btnX + 2, btnY + 2, btnW - 4, btnH - 4, COLOR_GOLD);
  
  // Corner accents
  gfx->fillRect(btnX, btnY, 10, 10, COLOR_WHITE);
  gfx->fillRect(btnX + btnW - 10, btnY, 10, 10, COLOR_WHITE);
  gfx->fillRect(btnX, btnY + btnH - 10, 10, 10, COLOR_WHITE);
  gfx->fillRect(btnX + btnW - 10, btnY + btnH - 10, 10, 10, COLOR_WHITE);
  
  // Button text
  gfx->setTextColor(RGB565(40, 25, 5));
  gfx->setTextSize(3);
  gfx->setCursor(centerX - 30, btnY + 25);
  gfx->print("TAP");
  
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 24, btnY + 55);
  gfx->print("FOR");
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(4);
  gfx->setCursor(centerX - 72, btnY + 75);
  gfx->print("+1000");
  
  // Instruction
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 65, 320);
  gfx->print("Tap as many times as you want!");
  
  drawSwipeIndicator();
}

void handleShopTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_TAP) {
    int x = gesture.x, y = gesture.y;
    int centerX = LCD_WIDTH / 2;
    
    // Check if tapped on the big button
    int btnX = 50;
    int btnY = 180;
    int btnW = LCD_WIDTH - 100;
    int btnH = 120;
    
    if (x >= btnX && x < btnX + btnW && y >= btnY && y < btnY + btnH) {
      // Add 1000 gems!
      system_state.player_gems += 1000;
      
      // Quick flash effect
      gfx->fillRect(btnX + 5, btnY + 5, btnW - 10, btnH - 10, COLOR_WHITE);
      delay(50);
      
      // Save game data to persist gems
      extern void saveAllGameData();
      saveAllGameData();
      
      // Redraw
      drawShopApp();
    }
  }
}

// =============================================================================
// GALLERY APP (Simple photo viewer placeholder)
// =============================================================================

static int gallery_image_index = 0;

void initGalleryApp() {
  gallery_image_index = 0;
}

void drawGalleryApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, RGB565(255, 100, 200));
  }
  gfx->setTextColor(RGB565(255, 100, 200));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 42, 14);
  gfx->print("GALLERY");
  
  // Image placeholder area
  gfx->fillRect(30, 60, LCD_WIDTH - 60, LCD_HEIGHT - 140, RGB565(12, 14, 20));
  gfx->drawRect(30, 60, LCD_WIDTH - 60, LCD_HEIGHT - 140, RGB565(40, 45, 60));
  
  // Placeholder content
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 60, centerY - 30);
  gfx->print("No Images");
  
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 80, centerY + 10);
  gfx->print("Connect to WiFi to sync");
  gfx->setCursor(centerX - 70, centerY + 25);
  gfx->print("photos from your phone");
  
  // Image counter
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(centerX - 15, LCD_HEIGHT - 70);
  gfx->printf("%d / 0", gallery_image_index + 1);
  
  // Navigation arrows
  gfx->fillRect(30, LCD_HEIGHT - 55, 60, 40, RGB565(15, 18, 25));
  gfx->drawRect(30, LCD_HEIGHT - 55, 60, 40, RGB565(40, 45, 60));
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(2);
  gfx->setCursor(50, LCD_HEIGHT - 45);
  gfx->print("<");
  
  gfx->fillRect(LCD_WIDTH - 90, LCD_HEIGHT - 55, 60, 40, RGB565(15, 18, 25));
  gfx->drawRect(LCD_WIDTH - 90, LCD_HEIGHT - 55, 60, 40, RGB565(40, 45, 60));
  gfx->setCursor(LCD_WIDTH - 70, LCD_HEIGHT - 45);
  gfx->print(">");
  
  drawSwipeIndicator();
}

void handleGalleryTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_LEFT) {
    gallery_image_index++;
    drawGalleryApp();
  } else if (gesture.event == TOUCH_SWIPE_RIGHT) {
    if (gallery_image_index > 0) gallery_image_index--;
    drawGalleryApp();
  }
  
  if (gesture.event == TOUCH_TAP) {
    int x = gesture.x, y = gesture.y;
    if (y >= LCD_HEIGHT - 55 && y < LCD_HEIGHT - 15) {
      if (x < 100 && gallery_image_index > 0) {
        gallery_image_index--;
        drawGalleryApp();
      } else if (x > LCD_WIDTH - 100) {
        gallery_image_index++;
        drawGalleryApp();
      }
    }
  }
}
