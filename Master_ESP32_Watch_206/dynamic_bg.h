/*
 * dynamic_bg.h - Dynamic Background System
 * Day/Night themes based on time for all watch faces
 */

#ifndef DYNAMIC_BG_H
#define DYNAMIC_BG_H

#include "config.h"

// Time periods for dynamic backgrounds
enum TimePeriod {
  TIME_DAWN = 0,      // 5:00 - 7:59
  TIME_MORNING,       // 8:00 - 11:59
  TIME_AFTERNOON,     // 12:00 - 16:59
  TIME_EVENING,       // 17:00 - 19:59
  TIME_NIGHT,         // 20:00 - 4:59
  TIME_PERIOD_COUNT
};

// Background style
struct DynamicBackground {
  uint16_t skyTop;
  uint16_t skyBottom;
  uint16_t accentGlow;
  uint16_t starColor;
  uint8_t starCount;
  bool showSun;
  bool showMoon;
  bool showStars;
  uint8_t ambientBrightness;  // 0-100
};

// Get current time period
TimePeriod getCurrentTimePeriod();

// Get background colors for time period
DynamicBackground getBackgroundForTime(TimePeriod period, ThemeType theme);

// Draw dynamic background
void drawDynamicBackground(ThemeType theme);

// Draw sky gradient
void drawSkyGradient(uint16_t topColor, uint16_t bottomColor, int startY, int height);

// Draw celestial objects
void drawSun(int x, int y, int radius, uint16_t color);
void drawMoon(int x, int y, int radius, uint16_t color);
void drawStars(int count, uint16_t color, int startY, int height);
void drawClouds(int y, uint16_t color, int count);

// Character-specific ambient effects
void drawLuffyAmbient(TimePeriod period);
void drawJinwooAmbient(TimePeriod period);
void drawYugoAmbient(TimePeriod period);
void drawNarutoAmbient(TimePeriod period);
void drawGokuAmbient(TimePeriod period);
void drawTanjiroAmbient(TimePeriod period);
void drawGojoAmbient(TimePeriod period);
void drawLeviAmbient(TimePeriod period);
void drawSaitamaAmbient(TimePeriod period);
void drawDekuAmbient(TimePeriod period);
void drawBoboiboyAmbient(TimePeriod period);

#endif // DYNAMIC_BG_H
