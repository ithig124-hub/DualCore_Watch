/*
 * dynamic_bg.cpp - Dynamic Background Implementation
 * Day/Night themes with smooth transitions
 */

#include "dynamic_bg.h"
#include "config.h"
#include "display.h"
#include "themes.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// =============================================================================
// TIME PERIOD DETECTION
// =============================================================================

TimePeriod getCurrentTimePeriod() {
  WatchTime time = getCurrentTime();
  int hour = time.hour;
  
  if (hour >= 5 && hour < 8)   return TIME_DAWN;
  if (hour >= 8 && hour < 12)  return TIME_MORNING;
  if (hour >= 12 && hour < 17) return TIME_AFTERNOON;
  if (hour >= 17 && hour < 20) return TIME_EVENING;
  return TIME_NIGHT;  // 20:00 - 4:59
}

// =============================================================================
// BACKGROUND CONFIGURATIONS FOR EACH TIME PERIOD
// =============================================================================

DynamicBackground getBackgroundForTime(TimePeriod period, ThemeType theme) {
  DynamicBackground bg;
  
  switch (period) {
    case TIME_DAWN:
      bg.skyTop = RGB565(40, 30, 60);       // Purple-blue
      bg.skyBottom = RGB565(255, 150, 100); // Orange-pink
      bg.accentGlow = RGB565(255, 180, 120);
      bg.starColor = RGB565(200, 200, 220);
      bg.starCount = 5;
      bg.showSun = true;
      bg.showMoon = false;
      bg.showStars = true;
      bg.ambientBrightness = 60;
      break;
      
    case TIME_MORNING:
      bg.skyTop = RGB565(100, 180, 255);    // Light blue
      bg.skyBottom = RGB565(200, 230, 255); // Very light blue
      bg.accentGlow = RGB565(255, 240, 200);
      bg.starColor = 0;
      bg.starCount = 0;
      bg.showSun = true;
      bg.showMoon = false;
      bg.showStars = false;
      bg.ambientBrightness = 90;
      break;
      
    case TIME_AFTERNOON:
      bg.skyTop = RGB565(80, 160, 255);     // Bright blue
      bg.skyBottom = RGB565(150, 200, 255); // Lighter blue
      bg.accentGlow = RGB565(255, 220, 150);
      bg.starColor = 0;
      bg.starCount = 0;
      bg.showSun = true;
      bg.showMoon = false;
      bg.showStars = false;
      bg.ambientBrightness = 100;
      break;
      
    case TIME_EVENING:
      bg.skyTop = RGB565(60, 40, 100);      // Deep purple
      bg.skyBottom = RGB565(255, 100, 50);  // Orange-red
      bg.accentGlow = RGB565(255, 120, 80);
      bg.starColor = RGB565(255, 255, 200);
      bg.starCount = 10;
      bg.showSun = true;
      bg.showMoon = false;
      bg.showStars = true;
      bg.ambientBrightness = 70;
      break;
      
    case TIME_NIGHT:
    default:
      bg.skyTop = RGB565(5, 5, 20);         // Near black
      bg.skyBottom = RGB565(20, 20, 50);    // Dark blue
      bg.accentGlow = RGB565(100, 100, 150);
      bg.starColor = RGB565(255, 255, 255);
      bg.starCount = 30;
      bg.showSun = false;
      bg.showMoon = true;
      bg.showStars = true;
      bg.ambientBrightness = 40;
      break;
  }
  
  // Character-specific color adjustments
  switch (theme) {
    case THEME_LUFFY_GEAR5:
      // Luffy - Always has golden sun energy
      if (period == TIME_NIGHT) {
        bg.skyBottom = RGB565(40, 30, 15);  // Warm dark
        bg.accentGlow = LUFFY_SUN_GOLD;
      }
      break;
      
    case THEME_SUNG_JINWOO:
      // Jin-Woo - Always dark and shadowy
      bg.skyTop = RGB565(10, 5, 25);
      bg.skyBottom = RGB565(25, 15, 45);
      bg.accentGlow = JINWOO_ARISE_GLOW;
      bg.ambientBrightness = min(bg.ambientBrightness, (uint8_t)50);
      break;
      
    case THEME_GOJO_INFINITY:
      // Gojo - Infinity void
      if (period == TIME_NIGHT) {
        bg.skyTop = RGB565(0, 0, 15);
        bg.skyBottom = RGB565(20, 30, 60);
        bg.accentGlow = GOJO_INFINITY_BLUE;
      }
      break;
      
    case THEME_BOBOIBOY:
      // BoBoiBoy - Element-influenced sky
      bg.accentGlow = BBB_BAND_ORANGE;
      break;
      
    default:
      break;
  }
  
  return bg;
}

// =============================================================================
// DRAWING FUNCTIONS
// =============================================================================

void drawDynamicBackground(ThemeType theme) {
  TimePeriod period = getCurrentTimePeriod();
  DynamicBackground bg = getBackgroundForTime(period, theme);
  
  // Draw sky gradient
  drawSkyGradient(bg.skyTop, bg.skyBottom, 0, LCD_HEIGHT);
  
  // Draw stars (if night or twilight)
  if (bg.showStars && bg.starCount > 0) {
    drawStars(bg.starCount, bg.starColor, 0, LCD_HEIGHT / 2);
  }
  
  // Draw celestial objects
  if (bg.showMoon) {
    drawMoon(LCD_WIDTH - 60, 60, 25, RGB565(240, 240, 220));
  }
  
  if (bg.showSun) {
    int sunY = 50;
    if (period == TIME_DAWN) sunY = 120;
    if (period == TIME_EVENING) sunY = 100;
    drawSun(60, sunY, 20, bg.accentGlow);
  }
  
  // Draw ambient clouds for morning/afternoon
  if (period == TIME_MORNING || period == TIME_AFTERNOON) {
    drawClouds(80, RGB565(255, 255, 255), 3);
  }
  
  // Character-specific ambient effects
  switch (theme) {
    case THEME_LUFFY_GEAR5:    drawLuffyAmbient(period); break;
    case THEME_SUNG_JINWOO:    drawJinwooAmbient(period); break;
    case THEME_YUGO_WAKFU:     drawYugoAmbient(period); break;
    case THEME_NARUTO_SAGE:    drawNarutoAmbient(period); break;
    case THEME_GOKU_UI:        drawGokuAmbient(period); break;
    case THEME_TANJIRO_SUN:    drawTanjiroAmbient(period); break;
    case THEME_GOJO_INFINITY:  drawGojoAmbient(period); break;
    case THEME_LEVI_STRONGEST: drawLeviAmbient(period); break;
    case THEME_SAITAMA_OPM:    drawSaitamaAmbient(period); break;
    case THEME_DEKU_PLUSULTRA: drawDekuAmbient(period); break;
    case THEME_BOBOIBOY:       drawBoboiboyAmbient(period); break;
    default: break;
  }
}

void drawSkyGradient(uint16_t topColor, uint16_t bottomColor, int startY, int height) {
  // Extract RGB components
  uint8_t r1 = (topColor >> 11) & 0x1F;
  uint8_t g1 = (topColor >> 5) & 0x3F;
  uint8_t b1 = topColor & 0x1F;
  
  uint8_t r2 = (bottomColor >> 11) & 0x1F;
  uint8_t g2 = (bottomColor >> 5) & 0x3F;
  uint8_t b2 = bottomColor & 0x1F;
  
  for (int y = startY; y < startY + height; y++) {
    float ratio = (float)(y - startY) / height;
    
    uint8_t r = r1 + (r2 - r1) * ratio;
    uint8_t g = g1 + (g2 - g1) * ratio;
    uint8_t b = b1 + (b2 - b1) * ratio;
    
    uint16_t color = (r << 11) | (g << 5) | b;
    gfx->drawFastHLine(0, y, LCD_WIDTH, color);
  }
}

void drawSun(int x, int y, int radius, uint16_t color) {
  // Glow effect
  for (int r = radius + 15; r > radius; r -= 3) {
    uint8_t alpha = map(r, radius, radius + 15, 40, 5);
    uint16_t glowColor = RGB565(alpha * 2, alpha, alpha / 2);
    gfx->drawCircle(x, y, r, glowColor);
  }
  
  // Sun body
  gfx->fillCircle(x, y, radius, color);
  gfx->fillCircle(x, y, radius - 3, RGB565(255, 255, 200));
  
  // Sun rays
  for (int i = 0; i < 8; i++) {
    float angle = i * PI / 4;
    int x1 = x + cos(angle) * (radius + 5);
    int y1 = y + sin(angle) * (radius + 5);
    int x2 = x + cos(angle) * (radius + 12);
    int y2 = y + sin(angle) * (radius + 12);
    gfx->drawLine(x1, y1, x2, y2, color);
  }
}

void drawMoon(int x, int y, int radius, uint16_t color) {
  // Moon glow
  for (int r = radius + 10; r > radius; r -= 2) {
    uint8_t alpha = map(r, radius, radius + 10, 30, 5);
    gfx->drawCircle(x, y, r, RGB565(alpha, alpha, alpha + 10));
  }
  
  // Moon body
  gfx->fillCircle(x, y, radius, color);
  
  // Crescent shadow
  gfx->fillCircle(x + 8, y - 3, radius - 3, RGB565(20, 20, 50));
  
  // Craters
  gfx->fillCircle(x - 5, y + 5, 4, RGB565(200, 200, 180));
  gfx->fillCircle(x + 3, y - 8, 3, RGB565(210, 210, 190));
}

void drawStars(int count, uint16_t color, int startY, int height) {
  // Use time-based seed for consistent star positions
  WatchTime time = getCurrentTime();
  srand(time.day * 100 + time.month);
  
  for (int i = 0; i < count; i++) {
    int x = rand() % LCD_WIDTH;
    int y = startY + (rand() % height);
    int size = 1 + (rand() % 2);
    
    // Twinkling effect based on time
    float twinkle = 0.5 + 0.5 * sin((millis() + i * 200) / 500.0);
    uint8_t brightness = 150 + twinkle * 105;
    
    uint16_t starCol = RGB565(brightness, brightness, brightness);
    
    if (size == 1) {
      gfx->drawPixel(x, y, starCol);
    } else {
      gfx->fillCircle(x, y, size, starCol);
    }
  }
}

void drawClouds(int y, uint16_t color, int count) {
  for (int i = 0; i < count; i++) {
    int x = 30 + i * 120 + (millis() / 100 + i * 50) % 50;
    int cloudY = y + (i % 2) * 30;
    
    // Semi-transparent cloud
    uint16_t cloudColor = RGB565(240, 245, 255);
    gfx->fillCircle(x, cloudY, 15, cloudColor);
    gfx->fillCircle(x + 15, cloudY - 5, 12, cloudColor);
    gfx->fillCircle(x + 25, cloudY, 10, cloudColor);
    gfx->fillCircle(x - 10, cloudY + 3, 10, cloudColor);
  }
}

// =============================================================================
// CHARACTER-SPECIFIC AMBIENT EFFECTS
// =============================================================================

void drawLuffyAmbient(TimePeriod period) {
  // Golden sun energy particles
  int particleCount = (period == TIME_NIGHT) ? 5 : 10;
  for (int i = 0; i < particleCount; i++) {
    float t = (millis() / 1000.0 + i * 0.5);
    int x = 50 + (i * 30) % (LCD_WIDTH - 100);
    int y = 100 + sin(t + i) * 20;
    
    uint16_t color = (i % 2) ? LUFFY_SUN_GOLD : LUFFY_ENERGY_ORANGE;
    gfx->fillCircle(x, y, 2 + (i % 2), color);
  }
}

void drawJinwooAmbient(TimePeriod period) {
  // Shadow particles rising
  for (int i = 0; i < 12; i++) {
    int x = 20 + (i * 31) % (LCD_WIDTH - 40);
    int y = LCD_HEIGHT - 50 - ((millis() / 50 + i * 30) % 150);
    
    uint8_t alpha = 20 + (y * 15 / LCD_HEIGHT);
    gfx->fillCircle(x, y, 2, RGB565(alpha, alpha/3, alpha + 20));
  }
}

void drawYugoAmbient(TimePeriod period) {
  // Portal energy wisps
  int centerX = LCD_WIDTH / 2;
  for (int i = 0; i < 6; i++) {
    float angle = (millis() / 1000.0 + i * 1.05) * 2;
    int x = centerX + cos(angle) * (50 + i * 10);
    int y = 150 + sin(angle) * 20;
    
    gfx->fillCircle(x, y, 3, YUGO_PORTAL_CYAN);
  }
}

void drawNarutoAmbient(TimePeriod period) {
  // Chakra particles
  for (int i = 0; i < 8; i++) {
    float t = millis() / 800.0 + i * 0.8;
    int x = LCD_WIDTH / 2 + cos(t) * (60 + i * 8);
    int y = 180 + sin(t * 1.5) * 30;
    
    uint16_t color = (i % 2) ? NARUTO_CHAKRA_ORANGE : NARUTO_SAGE_GOLD;
    gfx->fillCircle(x, y, 2, color);
  }
}

void drawGokuAmbient(TimePeriod period) {
  // UI aura particles
  for (int i = 0; i < 10; i++) {
    float t = millis() / 600.0 + i * 0.6;
    int x = LCD_WIDTH / 2 + cos(t * 2) * (40 + i * 7);
    int y = 160 + sin(t) * 40 - (i * 3);
    
    gfx->fillCircle(x, y, 2, GOKU_UI_SILVER);
  }
}

void drawTanjiroAmbient(TimePeriod period) {
  // Flame/water wisps alternating
  for (int i = 0; i < 7; i++) {
    int x = 40 + i * 50;
    float t = millis() / 400.0 + i * 0.5;
    int y = 120 + sin(t) * 15 - abs(sin(t * 2)) * 10;
    
    uint16_t color = (i % 2) ? TANJIRO_FIRE_ORANGE : TANJIRO_WATER_BLUE;
    gfx->fillCircle(x, y, 3, color);
  }
}

void drawGojoAmbient(TimePeriod period) {
  // Infinity swirl
  int centerX = LCD_WIDTH / 2;
  int centerY = 140;
  
  for (int i = 0; i < 20; i++) {
    float t = millis() / 300.0 + i * 0.3;
    float r = 30 + i * 2;
    int x = centerX + cos(t) * r;
    int y = centerY + sin(t) * (r * 0.4);
    
    uint16_t color = (i % 3 == 0) ? GOJO_HOLLOW_PURPLE : GOJO_INFINITY_BLUE;
    gfx->drawPixel(x, y, color);
  }
}

void drawLeviAmbient(TimePeriod period) {
  // Speed lines
  if (period == TIME_NIGHT) return;  // More subtle at night
  
  for (int i = 0; i < 5; i++) {
    int x = random(50, LCD_WIDTH - 50);
    int y = 80 + i * 20;
    int len = 15 + random(20);
    
    gfx->drawLine(x, y, x + len, y + random(-5, 5), LEVI_SPEED_BLUR);
  }
}

void drawSaitamaAmbient(TimePeriod period) {
  // Simple - just occasional impact lines
  if (millis() % 3000 < 200) {
    int centerX = LCD_WIDTH / 2;
    int centerY = 150;
    
    for (int i = 0; i < 8; i++) {
      float angle = i * PI / 4;
      int x1 = centerX + cos(angle) * 30;
      int y1 = centerY + sin(angle) * 30;
      int x2 = centerX + cos(angle) * 60;
      int y2 = centerY + sin(angle) * 60;
      
      gfx->drawLine(x1, y1, x2, y2, SAITAMA_IMPACT_ORANGE);
    }
  }
}

void drawDekuAmbient(TimePeriod period) {
  // OFA lightning crackling
  for (int i = 0; i < 4; i++) {
    if (random(100) < 30) {
      int x1 = random(LCD_WIDTH);
      int y1 = random(80, 180);
      int x2 = x1 + random(-30, 30);
      int y2 = y1 + random(20, 40);
      
      gfx->drawLine(x1, y1, x2, y2, DEKU_OFA_LIGHTNING);
    }
  }
}

void drawBoboiboyAmbient(TimePeriod period) {
  // Element particles cycling through colors
  uint16_t colors[] = {
    BBB_LIGHTNING_YELLOW, BBB_WIND_BLUE, BBB_EARTH_BROWN,
    BBB_FIRE_RED, BBB_WATER_CYAN, BBB_LEAF_GREEN, BBB_LIGHT_GOLD
  };
  
  for (int i = 0; i < 7; i++) {
    float t = millis() / 700.0 + i * 0.9;
    int x = 50 + i * 45;
    int y = 130 + sin(t) * 25;
    
    int colorIdx = (i + (millis() / 3000)) % 7;
    gfx->fillCircle(x, y, 4, colors[colorIdx]);
  }
}
