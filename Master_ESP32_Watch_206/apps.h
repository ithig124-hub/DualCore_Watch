/*
 * apps.h - Application Framework
 * Consolidates: apps.h/cpp, music_app.cpp, stopwatch_app.cpp, wallpaper_selector.h/cpp
 */

#ifndef APPS_H
#define APPS_H

#include "config.h"

// App structure
struct WatchApp {
  AppType type;
  String name;
  String icon_file;
  uint16_t icon_color;
  bool installed;
  void (*init_func)();
  void (*draw_func)();
  void (*touch_func)(TouchGesture&);
  void (*cleanup_func)();
};

// =============================================================================
// APP MANAGEMENT
// =============================================================================

void initializeApps();
void launchApp(AppType app);
void exitCurrentApp();
void drawAppGrid();
void drawAppIcon(int x, int y, int size, WatchApp& app);
void handleAppGridTouch(TouchGesture& gesture);
void switchToApp(AppType app);

// Music Player
void initMusicApp();
void drawMusicApp();
void handleMusicTouch(TouchGesture& gesture);
void playMusic();
void pauseMusic();
void nextTrack();
void previousTrack();

// Notes
void initNotesApp();
void drawNotesApp();
void handleNotesTouch(TouchGesture& gesture);
void saveNote();
void loadNote();

// File Browser
void initFileBrowserApp();
void drawFileBrowserApp();
void handleFileBrowserTouch(TouchGesture& gesture);

// PDF Reader
void initPDFReaderApp();
void drawPDFReaderApp();
void handlePDFReaderTouch(TouchGesture& gesture);

// Settings
void initSettingsApp();
void drawSettingsApp();
void handleSettingsTouch(TouchGesture& gesture);

// Stopwatch/Timer
void initStopwatchTimerApp();
void drawStopwatchTimerApp();
void handleStopwatchTimerTouch(TouchGesture& gesture);
void updateStopwatchApp();
void drawStopwatchMode();
void drawTimerMode();
void handleStopwatchTouch(TouchGesture& gesture);
void handleTimerTouch(TouchGesture& gesture);
void recordLapTime();

// Wallpaper Selector
void initWallpaperSelector();
void drawWallpaperSelector();
void handleWallpaperTouch(TouchGesture& gesture);
void selectWallpaper(int index);
void applyWallpaper(const String& path);

// Utilities
void initCalculatorApp();
void drawCalculatorApp();
void handleCalculatorTouch(TouchGesture& gesture);

void initFlashlightApp();
void drawFlashlightApp();
void handleFlashlightTouch(TouchGesture& gesture);

// Timer/Stopwatch
void initTimerApp();
void drawTimerApp();
void handleTimerTouch(TouchGesture& gesture);
void updateTimerDisplay();  // Real-time update while running

// Unit Converter
void initConverterApp();
void drawConverterApp();
void handleConverterTouch(TouchGesture& gesture);

// Achievements
void initAchievementsApp();
void drawAchievementsApp();
void handleAchievementsTouch(TouchGesture& gesture);

// Shop
void initShopApp();
void drawShopApp();
void handleShopTouch(TouchGesture& gesture);

// Gallery
void initGalleryApp();
void drawGalleryApp();
void handleGalleryTouch(TouchGesture& gesture);

// App registry
extern WatchApp registered_apps[];
extern int num_registered_apps;
extern int current_app_index;

#endif // APPS_H
