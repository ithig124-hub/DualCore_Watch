/*
 * converter_app.cpp - Unit Converter Implementation
 * Quick Win: Essential unit conversions
 */

#include "converter_app.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// State
ConverterCategory current_category = CONVERT_LENGTH;
int from_unit = 0;
int to_unit = 1;
float input_value = 1.0;

// Length units (base: meter)
ConversionUnit length_units[] = {
  {"Meter", "m", 1.0},
  {"Kilometer", "km", 1000.0},
  {"Centimeter", "cm", 0.01},
  {"Millimeter", "mm", 0.001},
  {"Mile", "mi", 1609.34},
  {"Yard", "yd", 0.9144},
  {"Foot", "ft", 0.3048},
  {"Inch", "in", 0.0254}
};
int length_unit_count = 8;

// Weight units (base: kilogram)
ConversionUnit weight_units[] = {
  {"Kilogram", "kg", 1.0},
  {"Gram", "g", 0.001},
  {"Milligram", "mg", 0.000001},
  {"Pound", "lb", 0.453592},
  {"Ounce", "oz", 0.0283495},
  {"Ton", "t", 1000.0}
};
int weight_unit_count = 6;

// Temperature (special case - not multiplicative)
ConversionUnit temp_units[] = {
  {"Celsius", "°C", 0},
  {"Fahrenheit", "°F", 0},
  {"Kelvin", "K", 0}
};
int temp_unit_count = 3;

// Speed units (base: m/s)
ConversionUnit speed_units[] = {
  {"m/s", "m/s", 1.0},
  {"km/h", "km/h", 0.277778},
  {"mph", "mph", 0.44704},
  {"knot", "kt", 0.514444}
};
int speed_unit_count = 4;

void initConverterApp() {
  current_category = CONVERT_LENGTH;
  from_unit = 0;
  to_unit = 1;
  input_value = 1.0;
}

void drawConverterApp() {
  gfx->fillScreen(RGB565(8, 8, 12));
  
  ThemeColors* theme = getCurrentTheme();
  
  // Header
  gfx->fillRoundRect(0, 0, LCD_WIDTH, 55, 0, RGB565(16, 18, 24));
  gfx->drawFastHLine(0, 55, LCD_WIDTH, theme->primary);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(LCD_WIDTH/2 - 60, 18);
  gfx->print("Converter");
  
  // Category tabs
  const char* categories[] = {"Length", "Weight", "Temp", "Speed"};
  int tabW = LCD_WIDTH / 4;
  int tabY = 65;
  
  for (int i = 0; i < 4; i++) {
    uint16_t bg = (i == (int)current_category) ? theme->primary : RGB565(25, 27, 35);
    uint16_t txtColor = (i == (int)current_category) ? COLOR_WHITE : RGB565(120, 120, 130);
    
    gfx->fillRoundRect(i * tabW + 5, tabY, tabW - 10, 35, 8, bg);
    gfx->setTextColor(txtColor);
    gfx->setTextSize(1);
    int len = strlen(categories[i]) * 6;
    gfx->setCursor(i * tabW + (tabW - len) / 2, tabY + 12);
    gfx->print(categories[i]);
  }
  
  // Input value display
  int displayY = 120;
  gfx->fillRoundRect(30, displayY, 310, 60, 15, RGB565(25, 27, 35));
  gfx->drawRoundRect(30, displayY, 310, 60, 15, theme->accent);
  
  gfx->setTextSize(3);
  gfx->setTextColor(theme->primary);
  char valueStr[20];
  snprintf(valueStr, 20, "%.2f", input_value);
  gfx->setCursor(50, displayY + 18);
  gfx->print(valueStr);
  
  // From unit selector
  int fromY = 200;
  ConversionUnit* units = nullptr;
  int unit_count = 0;
  
  switch (current_category) {
    case CONVERT_LENGTH: units = length_units; unit_count = length_unit_count; break;
    case CONVERT_WEIGHT: units = weight_units; unit_count = weight_unit_count; break;
    case CONVERT_TEMPERATURE: units = temp_units; unit_count = temp_unit_count; break;
    case CONVERT_SPEED: units = speed_units; unit_count = speed_unit_count; break;
  }
  
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setCursor(30, fromY - 18);
  gfx->print("FROM:");
  
  gfx->fillRoundRect(30, fromY, 310, 40, 10, RGB565(30, 32, 40));
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(50, fromY + 12);
  gfx->print(units[from_unit].name);
  
  // To unit selector
  int toY = 260;
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setCursor(30, toY - 18);
  gfx->print("TO:");
  
  gfx->fillRoundRect(30, toY, 310, 40, 10, RGB565(30, 32, 40));
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(50, toY + 12);
  gfx->print(units[to_unit].name);
  
  // Result display
  int resultY = 320;
  float result = convertValue(input_value, from_unit, to_unit, current_category);
  
  gfx->fillRoundRect(30, resultY, 310, 70, 15, RGB565(20, 60, 20));
  gfx->drawRoundRect(30, resultY, 310, 70, 15, COLOR_GREEN);
  
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 200, 150));
  gfx->setCursor(50, resultY + 10);
  gfx->print("RESULT:");
  
  gfx->setTextSize(3);
  gfx->setTextColor(COLOR_GREEN);
  char resultStr[30];
  snprintf(resultStr, 30, "%.4f %s", result, units[to_unit].symbol);
  gfx->setCursor(50, resultY + 32);
  gfx->print(resultStr);
  
  // Quick value buttons
  int btnY = 410;
  const char* quickVals[] = {"1", "10", "100", "+1"};
  int btnW = 70;
  int spacing = 8;
  int startX = (LCD_WIDTH - (4 * btnW + 3 * spacing)) / 2;
  
  for (int i = 0; i < 4; i++) {
    gfx->fillRoundRect(startX + i * (btnW + spacing), btnY, btnW, 30, 8, RGB565(60, 60, 70));
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_WHITE);
    int lblLen = strlen(quickVals[i]) * 6;
    gfx->setCursor(startX + i * (btnW + spacing) + (btnW - lblLen) / 2, btnY + 11);
    gfx->print(quickVals[i]);
  }
  
  drawSwipeIndicator();
}

float convertValue(float value, int fromUnit, int toUnit, ConverterCategory category) {
  if (category == CONVERT_TEMPERATURE) {
    // Special case: temperature conversions
    float celsius = 0;
    
    // Convert to Celsius first
    switch (fromUnit) {
      case 0: celsius = value; break;  // Already Celsius
      case 1: celsius = (value - 32) * 5.0 / 9.0; break;  // Fahrenheit to Celsius
      case 2: celsius = value - 273.15; break;  // Kelvin to Celsius
    }
    
    // Convert from Celsius to target
    switch (toUnit) {
      case 0: return celsius;  // Celsius
      case 1: return celsius * 9.0 / 5.0 + 32;  // Fahrenheit
      case 2: return celsius + 273.15;  // Kelvin
    }
  } else {
    // Multiplicative conversions (length, weight, speed)
    ConversionUnit* units = nullptr;
    
    switch (category) {
      case CONVERT_LENGTH: units = length_units; break;
      case CONVERT_WEIGHT: units = weight_units; break;
      case CONVERT_SPEED: units = speed_units; break;
      default: return 0;
    }
    
    // Convert to base unit, then to target unit
    float base_value = value * units[fromUnit].to_base;
    return base_value / units[toUnit].to_base;
  }
  
  return 0;
}

void handleConverterTouch(TouchGesture& gesture) {
  // Swipe UP to exit
  if (gesture.event == TOUCH_SWIPE_UP) {
    returnToAppGrid();
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x;
  int y = gesture.y;
  
  // Category tabs
  int tabW = LCD_WIDTH / 4;
  int tabY = 65;
  if (y >= tabY && y < tabY + 35) {
    int tapped = x / tabW;
    if (tapped < 4) {
      current_category = (ConverterCategory)tapped;
      from_unit = 0;
      to_unit = 1;
      drawConverterApp();
    }
    return;
  }
  
  // Quick value buttons
  int btnY = 410;
  int btnW = 70;
  int spacing = 8;
  int startX = (LCD_WIDTH - (4 * btnW + 3 * spacing)) / 2;
  
  if (y >= btnY && y < btnY + 30) {
    for (int i = 0; i < 4; i++) {
      int btnX = startX + i * (btnW + spacing);
      if (x >= btnX && x < btnX + btnW) {
        if (i == 0) input_value = 1.0;
        else if (i == 1) input_value = 10.0;
        else if (i == 2) input_value = 100.0;
        else if (i == 3) input_value += 1.0;
        drawConverterApp();
        return;
      }
    }
  }
  
  // From unit selector (cycle through units)
  if (y >= 200 && y < 240) {
    int max_units = 0;
    switch (current_category) {
      case CONVERT_LENGTH: max_units = length_unit_count; break;
      case CONVERT_WEIGHT: max_units = weight_unit_count; break;
      case CONVERT_TEMPERATURE: max_units = temp_unit_count; break;
      case CONVERT_SPEED: max_units = speed_unit_count; break;
    }
    from_unit = (from_unit + 1) % max_units;
    drawConverterApp();
    return;
  }
  
  // To unit selector (cycle through units)
  if (y >= 260 && y < 300) {
    int max_units = 0;
    switch (current_category) {
      case CONVERT_LENGTH: max_units = length_unit_count; break;
      case CONVERT_WEIGHT: max_units = weight_unit_count; break;
      case CONVERT_TEMPERATURE: max_units = temp_unit_count; break;
      case CONVERT_SPEED: max_units = speed_unit_count; break;
    }
    to_unit = (to_unit + 1) % max_units;
    drawConverterApp();
    return;
  }
}
