/*
 * display.h - Display Management
 * Consolidates: display.h/cpp, lvgl_display.h/cpp, lvgl_ui.h/cpp
 * 
 * CO5300 AMOLED Driver with LVGL integration
 * PORTED for ESP32-S3-Touch-AMOLED-2.06" (Waveshare)
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"
#include "Arduino_GFX_Library.h"
#include <lvgl.h>

// Global display objects - using Arduino_GFX base class for CO5300
extern Arduino_DataBus *bus;
extern Arduino_CO5300 *gfx;

// RGB565 color definitions
#define RGB565_BLACK    0x0000
#define RGB565_WHITE    0xFFFF
#define RGB565_RED      0xF800
#define RGB565_GREEN    0x07E0
#define RGB565_BLUE     0x001F
#define RGB565_YELLOW   0xFFE0
#define RGB565_ORANGE   0xFD20
#define RGB565_CYAN     0x07FF
#define RGB565_MAGENTA  0xF81F

// TFT compatibility
#define TFT_BLACK   RGB565_BLACK
#define TFT_WHITE   RGB565_WHITE
#define TFT_RED     RGB565_RED
#define TFT_GREEN   RGB565_GREEN
#define TFT_BLUE    RGB565_BLUE
#define TFT_YELLOW  RGB565_YELLOW
#define TFT_ORANGE  RGB565_ORANGE
#define TFT_CYAN    RGB565_CYAN

// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================

// Initialization
bool initializeDisplay();
void initDisplay();  // Alias for initializeDisplay()
void clearDisplay();
void updateDisplay();
void setDisplayBrightness(int brightness);
int  getDisplayBrightness();
void enterDisplaySleep();
void wakeDisplay();

// Drawing primitives
void drawPixel(int x, int y, uint16_t color);
void drawLine(int x0, int y0, int x1, int y1, uint16_t color);
void drawRect(int x, int y, int w, int h, uint16_t color);
void fillRect(int x, int y, int w, int h, uint16_t color);
void drawCircle(int x, int y, int radius, uint16_t color);
void fillCircle(int x, int y, int radius, uint16_t color);
void drawRoundRect(int x, int y, int w, int h, int radius, uint16_t color);
void fillRoundRect(int x, int y, int w, int h, int radius, uint16_t color);

// Text rendering
void drawText(const char* text, int x, int y, uint16_t color, int size);
void drawCenteredText(const char* text, int x, int y, uint16_t color, int size);
int getTextWidth(const char* text, int size);

// Advanced graphics
void drawBitmap(int x, int y, int w, int h, const uint16_t* bitmap);
void drawGradient(int x, int y, int w, int h, uint16_t color1, uint16_t color2, bool vertical);
void drawProgressRing(int centerX, int centerY, int radius, float progress, uint16_t color, int thickness);
void drawActivityRing(int centerX, int centerY, int radius, float progress, uint16_t color, int thickness);

// Animation
void fadeIn(int duration);
void fadeOut(int duration);
void slideTransition(int direction, int duration);

// =============================================================================
// LVGL UI FUNCTIONS
// =============================================================================

typedef enum {
  LVGL_SCREEN_WATCHFACE, LVGL_SCREEN_APP_GRID, LVGL_SCREEN_MUSIC,
  LVGL_SCREEN_GAMES, LVGL_SCREEN_NOTES, LVGL_SCREEN_QUESTS,
  LVGL_SCREEN_SETTINGS, LVGL_SCREEN_PDF_READER, LVGL_SCREEN_FILE_BROWSER,
  LVGL_SCREEN_WEATHER, LVGL_SCREEN_COUNT
} lvgl_screen_t;

typedef struct {
  lv_color_t primary, secondary, accent, background, text, shadow;
} lvgl_theme_colors_t;

extern lvgl_screen_t current_lvgl_screen;
extern lv_obj_t* screen_objects[LVGL_SCREEN_COUNT];

bool initializeLVGLUI();
void lvgl_load_screen(lvgl_screen_t screen);
void lvgl_create_all_screens();
void lvgl_set_theme_colors(ThemeType theme);

// Watchface screens
lv_obj_t* lvgl_create_watchface_screen();
void lvgl_update_watchface();
void lvgl_create_luffy_watchface();
void lvgl_create_jinwoo_watchface();
void lvgl_create_yugo_watchface();

// App grid
lv_obj_t* lvgl_create_app_grid_screen();
void lvgl_update_app_grid();

// App screens
lv_obj_t* lvgl_create_music_screen();
lv_obj_t* lvgl_create_games_screen();
lv_obj_t* lvgl_create_notes_screen();
lv_obj_t* lvgl_create_quests_screen();
lv_obj_t* lvgl_create_settings_screen();
lv_obj_t* lvgl_create_weather_screen();

// Navigation
void lvgl_go_back();
void lvgl_go_home();
void lvgl_show_app_grid();

// Animations
void lvgl_slide_screen_left(lv_obj_t* screen);
void lvgl_slide_screen_right(lv_obj_t* screen);
void lvgl_fade_screen(lv_obj_t* screen);

// UI components
lv_obj_t* lvgl_create_app_button(lv_obj_t* parent, const char* text, lv_color_t color, lv_event_cb_t event_cb);
lv_obj_t* lvgl_create_title_bar(lv_obj_t* parent, const char* title, bool show_back_btn);
lv_obj_t* lvgl_create_progress_arc(lv_obj_t* parent, int32_t value, int32_t max_value);

// Theme management
void lvgl_apply_luffy_theme();
void lvgl_apply_jinwoo_theme();
void lvgl_apply_yugo_theme();

// Utility
lv_color_t hex_to_lv_color(uint32_t hex);

#endif // DISPLAY_H
