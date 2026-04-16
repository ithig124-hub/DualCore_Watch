#ifndef TIME_EDIT_H
#define TIME_EDIT_H

#include <Arduino.h>
#include "config.h"      // WatchTime struct definition
#include "display.h"     // gfx, LCD dimensions
#include "themes.h"      // ThemeColors, getCurrentTheme()
#include "hardware.h"    // getCurrentTime(), setCurrentTime()

// Time edit state
extern bool time_edit_active;

// Time edit functions
void showTimeEditPopup();
void drawTimeEditPopup();
void handleTimeEditTouch(int x, int y);

#endif // TIME_EDIT_H
