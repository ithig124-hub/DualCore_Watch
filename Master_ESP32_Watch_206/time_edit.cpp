// =============================================================================
// TIME EDIT POPUP - FUSION OS
// Manual time adjustment with RTC save
// =============================================================================

#include "time_edit.h"
#include "config.h"      // system_state, SCREEN_SETTINGS
#include "hardware.h"    // getCurrentTime(), setCurrentTime()
#include "display.h"     // gfx, LCD_WIDTH, LCD_HEIGHT
#include "themes.h"      // getCurrentTheme(), ThemeColors
#include "apps.h"        // drawSettingsApp()

bool time_edit_active = false;
static int edit_hour = 0;
static int edit_minute = 0;
static bool editing_hour = true;  // true = hour, false = minute

void showTimeEditPopup() {
  time_edit_active = true;
  
  // Get current time as starting point
  WatchTime current = getCurrentTime();
  edit_hour = current.hour;
  edit_minute = current.minute;
  editing_hour = true;
  
  drawTimeEditPopup();
}

void drawTimeEditPopup() {
  // Semi-transparent overlay
  gfx->fillScreen(RGB565(5, 10, 20));
  
  ThemeColors* theme = getCurrentTheme();
  
  // Popup box
  int boxX = 40;
  int boxY = 120;
  int boxW = LCD_WIDTH - 80;
  int boxH = 280;
  
  gfx->fillRect(boxX, boxY, boxW, boxH, RGB565(15, 20, 35));
  gfx->drawRect(boxX, boxY, boxW, boxH, theme->primary);
  gfx->drawRect(boxX + 1, boxY + 1, boxW - 2, boxH - 2, theme->accent);
  
  // Header
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(boxX + 85, boxY + 20);
  gfx->print("EDIT TIME");
  
  // Instructions
  gfx->setTextColor(RGB565(150, 160, 180));
  gfx->setTextSize(1);
  gfx->setCursor(boxX + 60, boxY + 50);
  gfx->print("Tap +/- to adjust");
  
  // Time display
  int timeY = boxY + 90;
  
  // Hour
  int hourX = boxX + 70;
  uint16_t hourColor = editing_hour ? COLOR_WHITE : RGB565(100, 110, 130);
  gfx->fillRect(hourX - 10, timeY - 10, 80, 80, editing_hour ? theme->primary : RGB565(30, 35, 45));
  gfx->drawRect(hourX - 10, timeY - 10, 80, 80, editing_hour ? theme->accent : RGB565(60, 65, 75));
  
  gfx->setTextColor(hourColor);
  gfx->setTextSize(5);
  gfx->setCursor(hourX, timeY);
  gfx->printf("%02d", edit_hour);
  
  // Colon
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(4);
  gfx->setCursor(hourX + 85, timeY + 5);
  gfx->print(":");
  
  // Minute
  int minX = boxX + 180;
  uint16_t minColor = !editing_hour ? COLOR_WHITE : RGB565(100, 110, 130);
  gfx->fillRect(minX - 10, timeY - 10, 80, 80, !editing_hour ? theme->primary : RGB565(30, 35, 45));
  gfx->drawRect(minX - 10, timeY - 10, 80, 80, !editing_hour ? theme->accent : RGB565(60, 65, 75));
  
  gfx->setTextColor(minColor);
  gfx->setTextSize(5);
  gfx->setCursor(minX, timeY);
  gfx->printf("%02d", edit_minute);
  
  // +/- Buttons
  int btnY = boxY + 190;
  int btnW = 50;
  int btnH = 50;
  
  // + Button
  int plusX = boxX + 100;
  gfx->fillRect(plusX, btnY, btnW, btnH, theme->effect1);
  gfx->drawRect(plusX, btnY, btnW, btnH, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(plusX + 15, btnY + 12);
  gfx->print("+");
  
  // - Button
  int minusX = boxX + 180;
  gfx->fillRect(minusX, btnY, btnW, btnH, theme->effect2);
  gfx->drawRect(minusX, btnY, btnW, btnH, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(minusX + 18, btnY + 12);
  gfx->print("-");
  
  // Switch H/M button
  int switchY = boxY + 250;
  gfx->fillRect(boxX + 60, switchY, 210, 35, RGB565(40, 50, 70));
  gfx->drawRect(boxX + 60, switchY, 210, 35, theme->accent);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(boxX + 75, switchY + 10);
  gfx->print(editing_hour ? "TAP TO EDIT MINUTES" : "TAP TO EDIT HOURS");
  
  // Save & Cancel buttons
  int bottomY = boxY + boxH - 45;
  
  // Save button
  gfx->fillRect(boxX + 20, bottomY, 120, 35, RGB565(20, 120, 60));
  gfx->drawRect(boxX + 20, bottomY, 120, 35, RGB565(40, 200, 100));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(boxX + 45, bottomY + 8);
  gfx->print("SAVE");
  
  // Cancel button
  gfx->fillRect(boxX + 190, bottomY, 120, 35, RGB565(120, 20, 20));
  gfx->drawRect(boxX + 190, bottomY, 120, 35, RGB565(200, 40, 40));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(boxX + 198, bottomY + 8);
  gfx->print("CANCEL");
}

void handleTimeEditTouch(int x, int y) {
  int boxX = 40;
  int boxY = 120;
  
  // + Button (y: 310-360, x: 140-190)
  if (y >= 310 && y <= 360 && x >= 140 && x <= 190) {
    if (editing_hour) {
      edit_hour = (edit_hour + 1) % 24;
    } else {
      edit_minute = (edit_minute + 1) % 60;
    }
    drawTimeEditPopup();
    return;
  }
  
  // - Button (y: 310-360, x: 220-270)
  if (y >= 310 && y <= 360 && x >= 220 && x <= 270) {
    if (editing_hour) {
      edit_hour = (edit_hour - 1 + 24) % 24;
    } else {
      edit_minute = (edit_minute - 1 + 60) % 60;
    }
    drawTimeEditPopup();
    return;
  }
  
  // Switch H/M button (y: 370-405, x: 100-310)
  if (y >= 370 && y <= 405 && x >= 100 && x <= 310) {
    editing_hour = !editing_hour;
    drawTimeEditPopup();
    return;
  }
  
  // Save button (y: 355-390, x: 60-180)
  if (y >= 355 && y <= 390 && x >= 60 && x <= 180) {
    // Save time to RTC
    WatchTime new_time = getCurrentTime();
    new_time.hour = edit_hour;
    new_time.minute = edit_minute;
    new_time.second = 0;  // Reset seconds
    
    setCurrentTime(new_time);
    
    Serial.printf("[TIME EDIT] Time saved: %02d:%02d\n", edit_hour, edit_minute);
    
    time_edit_active = false;
    
    // Return to Settings
    system_state.current_screen = SCREEN_SETTINGS;
    drawSettingsApp();
    return;
  }
  
  // Cancel button (y: 355-390, x: 230-350)
  if (y >= 355 && y <= 390 && x >= 230 && x <= 350) {
    Serial.println("[TIME EDIT] Cancelled");
    time_edit_active = false;
    
    // Return to Settings
    system_state.current_screen = SCREEN_SETTINGS;
    drawSettingsApp();
    return;
  }
}
