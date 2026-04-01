/*
 * display.cpp - Display Implementation
 * LVGL UI and display driver functions
 */

#include "display.h"
#include "config.h"

lvgl_screen_t current_lvgl_screen = LVGL_SCREEN_WATCHFACE;
lv_obj_t* screen_objects[LVGL_SCREEN_COUNT] = {nullptr};

// =============================================================================
// DISPLAY INITIALIZATION
// =============================================================================

bool initializeDisplay() {
  Serial.println("[Display] Initializing CO5300 AMOLED (2.06\" board)...");
  gfx->begin();
  gfx->setBrightness(255);
  delay(100);
  gfx->fillScreen(0x0000);
  Serial.println("[Display] Ready");
  return true;
}

// Alias function for initializeDisplay
void initDisplay() {
  initializeDisplay();
}

void clearDisplay() {
  gfx->fillScreen(RGB565_BLACK);
}

void updateDisplay() {
  // LVGL handles updates automatically
}

void setDisplayBrightness(int brightness) {
  brightness = constrain(brightness, 0, 255);
  gfx->setBrightness(brightness);
  system_state.brightness = brightness;
}

int getDisplayBrightness() {
  return system_state.brightness;
}

void enterDisplaySleep() {
  gfx->displayOff();
}

void wakeDisplay() {
  gfx->displayOn();
  gfx->setBrightness(system_state.brightness);
}

// =============================================================================
// DRAWING PRIMITIVES
// =============================================================================

void drawPixel(int x, int y, uint16_t color) {
  gfx->drawPixel(x, y, color);
}

void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
  gfx->drawLine(x0, y0, x1, y1, color);
}

void drawRect(int x, int y, int w, int h, uint16_t color) {
  gfx->drawRect(x, y, w, h, color);
}

void fillRect(int x, int y, int w, int h, uint16_t color) {
  gfx->fillRect(x, y, w, h, color);
}

void drawCircle(int x, int y, int radius, uint16_t color) {
  gfx->drawCircle(x, y, radius, color);
}

void fillCircle(int x, int y, int radius, uint16_t color) {
  gfx->fillCircle(x, y, radius, color);
}

void drawRoundRect(int x, int y, int w, int h, int radius, uint16_t color) {
  gfx->drawRoundRect(x, y, w, h, radius, color);
}

void fillRoundRect(int x, int y, int w, int h, int radius, uint16_t color) {
  gfx->fillRoundRect(x, y, w, h, radius, color);
}

void drawText(const char* text, int x, int y, uint16_t color, int size) {
  gfx->setTextColor(color);
  gfx->setTextSize(size);
  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawCenteredText(const char* text, int x, int y, uint16_t color, int size) {
  gfx->setTextColor(color);
  gfx->setTextSize(size);
  int width = strlen(text) * 6 * size;
  gfx->setCursor(x - width/2, y);
  gfx->print(text);
}

int getTextWidth(const char* text, int size) {
  return strlen(text) * 6 * size;
}

void drawBitmap(int x, int y, int w, int h, const uint16_t* bitmap) {
  gfx->draw16bitRGBBitmap(x, y, (uint16_t*)bitmap, w, h);
}

void drawGradient(int x, int y, int w, int h, uint16_t color1, uint16_t color2, bool vertical) {
  for (int i = 0; i < (vertical ? h : w); i++) {
    float ratio = (float)i / (vertical ? h : w);
    uint16_t r = ((color1 >> 11) & 0x1F) + ratio * (((color2 >> 11) & 0x1F) - ((color1 >> 11) & 0x1F));
    uint16_t g = ((color1 >> 5) & 0x3F) + ratio * (((color2 >> 5) & 0x3F) - ((color1 >> 5) & 0x3F));
    uint16_t b = (color1 & 0x1F) + ratio * ((color2 & 0x1F) - (color1 & 0x1F));
    uint16_t color = (r << 11) | (g << 5) | b;
    
    if (vertical) gfx->drawFastHLine(x, y + i, w, color);
    else gfx->drawFastVLine(x + i, y, h, color);
  }
}

void drawProgressRing(int centerX, int centerY, int radius, float progress, uint16_t color, int thickness) {
  progress = constrain(progress, 0.0f, 1.0f);
  float startAngle = -90;
  float endAngle = startAngle + (360 * progress);
  
  for (float angle = startAngle; angle < endAngle; angle += 2) {
    float rad = angle * PI / 180;
    for (int t = 0; t < thickness; t++) {
      int r = radius - t;
      int x = centerX + cos(rad) * r;
      int y = centerY + sin(rad) * r;
      gfx->drawPixel(x, y, color);
    }
  }
}

// drawActivityRing() is defined in themes.cpp to avoid duplicate definition

void fadeIn(int duration) {
  for (int i = 0; i <= 255; i += 5) {
    gfx->setBrightness(i);
    delay(duration / 51);
  }
}

void fadeOut(int duration) {
  for (int i = 255; i >= 0; i -= 5) {
    gfx->setBrightness(i);
    delay(duration / 51);
  }
}

void slideTransition(int direction, int duration) {
  // Simplified slide transition
  fadeOut(duration / 2);
  fadeIn(duration / 2);
}

// =============================================================================
// LVGL UI FUNCTIONS
// =============================================================================

bool initializeLVGLUI() {
  Serial.println("[LVGL UI] Initializing...");
  return true;
}

void lvgl_load_screen(lvgl_screen_t screen) {
  if (screen < LVGL_SCREEN_COUNT && screen_objects[screen] != nullptr) {
    lv_scr_load(screen_objects[screen]);
    current_lvgl_screen = screen;
  }
}

void lvgl_create_all_screens() {
  screen_objects[LVGL_SCREEN_WATCHFACE] = lvgl_create_watchface_screen();
  screen_objects[LVGL_SCREEN_APP_GRID] = lvgl_create_app_grid_screen();
}

void lvgl_set_theme_colors(ThemeType theme) {
  switch(theme) {
    case THEME_LUFFY_GEAR5: lvgl_apply_luffy_theme(); break;
    case THEME_SUNG_JINWOO: lvgl_apply_jinwoo_theme(); break;
    case THEME_YUGO_WAKFU: lvgl_apply_yugo_theme(); break;
    default: break;
  }
}

lv_obj_t* lvgl_create_watchface_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  
  lv_obj_t* time_label = lv_label_create(scr);
  lv_label_set_text(time_label, "12:00");
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(time_label);
  
  return scr;
}

void lvgl_update_watchface() {
  // Update time display
}

void lvgl_create_luffy_watchface() {
  // Luffy Gear 5 themed watchface
}

void lvgl_create_jinwoo_watchface() {
  // Sung Jin-Woo themed watchface
}

void lvgl_create_yugo_watchface() {
  // Yugo Wakfu themed watchface
}

lv_obj_t* lvgl_create_app_grid_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  return scr;
}

void lvgl_update_app_grid() {
  // Update app grid
}

lv_obj_t* lvgl_create_music_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

lv_obj_t* lvgl_create_games_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

lv_obj_t* lvgl_create_notes_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

lv_obj_t* lvgl_create_quests_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

lv_obj_t* lvgl_create_settings_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

lv_obj_t* lvgl_create_weather_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  return scr;
}

void lvgl_go_back() {
  lvgl_load_screen(LVGL_SCREEN_APP_GRID);
}

void lvgl_go_home() {
  lvgl_load_screen(LVGL_SCREEN_WATCHFACE);
}

void lvgl_show_app_grid() {
  lvgl_load_screen(LVGL_SCREEN_APP_GRID);
}

void lvgl_slide_screen_left(lv_obj_t* screen) {
  lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
}

void lvgl_slide_screen_right(lv_obj_t* screen) {
  lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
}

void lvgl_fade_screen(lv_obj_t* screen) {
  lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
}

lv_obj_t* lvgl_create_app_button(lv_obj_t* parent, const char* text, lv_color_t color, lv_event_cb_t event_cb) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, 80, 80);
  lv_obj_set_style_bg_color(btn, color, 0);
  lv_obj_set_style_radius(btn, 15, 0);
  
  if (event_cb) {
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, NULL);
  }
  
  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_center(label);
  
  return btn;
}

lv_obj_t* lvgl_create_title_bar(lv_obj_t* parent, const char* title, bool show_back_btn) {
  lv_obj_t* bar = lv_obj_create(parent);
  lv_obj_set_size(bar, LCD_WIDTH, 50);
  lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(0x1a1a1a), 0);
  
  lv_obj_t* label = lv_label_create(bar);
  lv_label_set_text(label, title);
  lv_obj_center(label);
  
  return bar;
}

lv_obj_t* lvgl_create_progress_arc(lv_obj_t* parent, int32_t value, int32_t max_value) {
  lv_obj_t* arc = lv_arc_create(parent);
  lv_arc_set_range(arc, 0, max_value);
  lv_arc_set_value(arc, value);
  lv_obj_set_size(arc, 100, 100);
  return arc;
}

void lvgl_apply_luffy_theme() {
  // Apply Luffy Gear 5 theme colors
}

void lvgl_apply_jinwoo_theme() {
  // Apply Sung Jin-Woo theme colors
}

void lvgl_apply_yugo_theme() {
  // Apply Yugo Wakfu theme colors
}

lv_color_t hex_to_lv_color(uint32_t hex) {
  return lv_color_hex(hex);
}
