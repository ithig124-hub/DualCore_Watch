/*
 * themes.cpp - IMPROVED Theme Implementation
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * IMPROVEMENTS:
 * - Modern glass morphism UI throughout
 * - Better visual hierarchy
 * - Animated effects for each character
 * - Improved color contrast
 * - Premium watch face designs
 * - Dynamic day/night backgrounds
 * - Tap-to-switch elements for BoBoiBoy
 */

#include "themes.h"
#include "config.h"
#include "display.h"
#include "hardware.h"
#include "dynamic_bg.h"
#include "ochobot.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// =============================================================================
// THEME COLOR DEFINITIONS - ALL 10 CHARACTERS (IMPROVED)
// =============================================================================

// Luffy Gear 5 - Sun God Nika
ThemeColors luffy_gear5_theme = {
  .primary = LUFFY_SUN_GOLD,
  .secondary = LUFFY_NIKA_WHITE,
  .accent = LUFFY_ENERGY_ORANGE,
  .background = LUFFY_DEEP_BLACK,
  .text = COLOR_WHITE,
  .shadow = LUFFY_CLOUD_WHITE,
  .effect1 = LUFFY_SUN_GLOW,
  .effect2 = LUFFY_JOY_YELLOW,
  .effect3 = LUFFY_FREEDOM_SKY,
  .effect4 = LUFFY_RUBBER_PINK,
  .corner_radius = 20,
  .glow_intensity = 200,
  .animation_speed = 8,
  .border_width = 3
};

// Jin-Woo - Shadow Monarch
ThemeColors sung_jinwoo_theme = {
  .primary = JINWOO_MONARCH_PURPLE,
  .secondary = JINWOO_VOID_BLACK,
  .accent = JINWOO_ARISE_GLOW,
  .background = JINWOO_ABSOLUTE_DARK,
  .text = COLOR_WHITE,
  .shadow = JINWOO_DEATH_BLACK,
  .effect1 = JINWOO_ARISE_GLOW,
  .effect2 = JINWOO_DAGGER_BLUE,
  .effect3 = JINWOO_MIST_PURPLE,
  .effect4 = JINWOO_SOUL_PURPLE,
  .corner_radius = 5,
  .glow_intensity = 255,
  .animation_speed = 10,
  .border_width = 2
};

// Yugo - Portal Master
ThemeColors yugo_wakfu_theme = {
  .primary = YUGO_PORTAL_CYAN,
  .secondary = YUGO_ELIATROPE_TEAL,
  .accent = YUGO_HAT_GOLD,
  .background = YUGO_SKY_BLUE_GREY,
  .text = COLOR_WHITE,
  .shadow = YUGO_MYSTERY_BLUE,
  .effect1 = YUGO_PORTAL_GLOW,
  .effect2 = YUGO_WAKFU_ENERGY,
  .effect3 = YUGO_PORTAL_RING,
  .effect4 = YUGO_ENERGY_BURST,
  .corner_radius = 15,
  .glow_intensity = 150,
  .animation_speed = 7,
  .border_width = 3
};

// Naruto - Sage Mode
ThemeColors naruto_sage_theme = {
  .primary = NARUTO_CHAKRA_ORANGE,
  .secondary = NARUTO_OUTFIT_BLACK,
  .accent = NARUTO_SAGE_GOLD,
  .background = NARUTO_SLATE_GREY,
  .text = COLOR_WHITE,
  .shadow = NARUTO_SHADOW_ORANGE,
  .effect1 = NARUTO_KURAMA_FLAME,
  .effect2 = NARUTO_RASENGAN_BLUE,
  .effect3 = NARUTO_SAGE_EYES,
  .effect4 = NARUTO_WILL_FIRE,
  .corner_radius = 10,
  .glow_intensity = 180,
  .animation_speed = 7,
  .border_width = 2
};

// Goku - Ultra Instinct
ThemeColors goku_ui_theme = {
  .primary = GOKU_UI_SILVER,
  .secondary = GOKU_GI_BLUE,
  .accent = GOKU_AURA_WHITE,
  .background = GOKU_VOID_BLACK,
  .text = COLOR_WHITE,
  .shadow = GOKU_SILVER_GLOW,
  .effect1 = GOKU_DIVINE_SILVER,
  .effect2 = GOKU_KI_BLAST_BLUE,
  .effect3 = GOKU_SPEED_LINES,
  .effect4 = GOKU_LIGHT_SILVER,
  .corner_radius = 10,
  .glow_intensity = 255,
  .animation_speed = 10,
  .border_width = 2
};

// Tanjiro - Sun Breathing
ThemeColors tanjiro_sun_theme = {
  .primary = TANJIRO_FIRE_ORANGE,
  .secondary = TANJIRO_CHECK_GREEN,
  .accent = TANJIRO_HANAFUDA_RED,
  .background = TANJIRO_DARK_CHARCOAL,
  .text = COLOR_WHITE,
  .shadow = TANJIRO_SCAR_BURGUNDY,
  .effect1 = TANJIRO_FLAME_GLOW,
  .effect2 = TANJIRO_WATER_BLUE,
  .effect3 = TANJIRO_SUN_FIRE,
  .effect4 = TANJIRO_SPIRIT_WHITE,
  .corner_radius = 8,
  .glow_intensity = 200,
  .animation_speed = 6,
  .border_width = 2
};

// Gojo - Infinity
ThemeColors gojo_infinity_theme = {
  .primary = GOJO_INFINITY_BLUE,
  .secondary = GOJO_SNOW_WHITE,
  .accent = GOJO_HOLLOW_PURPLE,
  .background = GOJO_VOID_BLACK,
  .text = COLOR_WHITE,
  .shadow = GOJO_DOMAIN_PURPLE,
  .effect1 = GOJO_LIGHT_BLUE_GLOW,
  .effect2 = GOJO_SIX_EYES_BLUE,
  .effect3 = GOJO_DEEP_INFINITY,
  .effect4 = GOJO_CURSED_PURPLE,
  .corner_radius = 10,
  .glow_intensity = 230,
  .animation_speed = 9,
  .border_width = 2
};

// Levi - Humanity's Strongest
ThemeColors levi_strongest_theme = {
  .primary = LEVI_SURVEY_GREEN,
  .secondary = LEVI_DARK_UNIFORM,
  .accent = LEVI_SILVER_BLADE,
  .background = LEVI_CHARCOAL_DARK,
  .text = COLOR_WHITE,
  .shadow = LEVI_MILITARY_GREY,
  .effect1 = LEVI_STEEL_METAL,
  .effect2 = LEVI_CAPE_GREEN,
  .effect3 = LEVI_SPEED_BLUR,
  .effect4 = LEVI_CLEAN_WHITE,
  .corner_radius = 5,
  .glow_intensity = 150,
  .animation_speed = 8,
  .border_width = 2
};

// Saitama - One Punch
ThemeColors saitama_opm_theme = {
  .primary = SAITAMA_HERO_YELLOW,
  .secondary = SAITAMA_CAPE_RED,
  .accent = SAITAMA_GOLDEN_PUNCH,
  .background = SAITAMA_SIMPLE_BLACK,
  .text = COLOR_WHITE,
  .shadow = SAITAMA_DEEP_RED_CAPE,
  .effect1 = SAITAMA_IMPACT_ORANGE,
  .effect2 = SAITAMA_GLOVE_RED,
  .effect3 = SAITAMA_BOOT_YELLOW,
  .effect4 = SAITAMA_BALD_WHITE,
  .corner_radius = 0,
  .glow_intensity = 100,
  .animation_speed = 1,
  .border_width = 3
};

// Deku - Plus Ultra
ThemeColors deku_plusultra_theme = {
  .primary = DEKU_HERO_GREEN,
  .secondary = DEKU_COSTUME_BLACK,
  .accent = DEKU_OFA_LIGHTNING,
  .background = DEKU_DARK_HERO,
  .text = COLOR_WHITE,
  .shadow = DEKU_BLACKWHIP_DARK,
  .effect1 = DEKU_FULL_COWL,
  .effect2 = DEKU_AIR_FORCE,
  .effect3 = DEKU_POWER_VEINS,
  .effect4 = DEKU_ALLMIGHT_GOLD,
  .corner_radius = 10,
  .glow_intensity = 220,
  .animation_speed = 9,
  .border_width = 2
};

ThemeColors* current_theme = &luffy_gear5_theme;

// =============================================================================
// BOBOIBOY ELEMENTAL THEME - Malaysian Anime Hero
// Power Band with 7 Element Transformations
// =============================================================================

ThemeColors boboiboy_elemental_theme = {
  .primary = BBB_BAND_ORANGE,           // Power Band Orange
  .secondary = BBB_LIGHTNING_YELLOW,    // Default Lightning
  .accent = BBB_OCHOBOT_WHITE,          // Ochobot helper
  .background = RGB565(15, 15, 20),     // Dark background
  .text = COLOR_WHITE,
  .shadow = BBB_THUNDERSTORM_BLACK,
  .effect1 = BBB_FIRE_RED,              // Element effect
  .effect2 = BBB_WIND_BLUE,             // Element effect
  .effect3 = BBB_EARTH_BROWN,           // Element effect
  .effect4 = BBB_LIGHT_GOLD,            // Solar effect
  .corner_radius = 12,
  .glow_intensity = 220,
  .animation_speed = 8,
  .border_width = 2
};

// =============================================================================
// CHARACTER PROFILES (11 characters including BoBoiBoy)
// =============================================================================

CharacterProfile character_profiles[11] = {
  // Luffy
  {"Monkey D. Luffy", "Sun God Nika", "One Piece", "Bajrang Gun", 
   "I'm gonna be King of the Pirates!", THEME_LUFFY_GEAR5,
   {"Haki", "All 3 types", 80, "Gear", "5th unlocked", 100, 
    "Freedom", "Unbound", 100, "Dream", "Pirate King", 85}},
  
  // Jin-Woo
  {"Sung Jin-Woo", "Shadow Monarch", "Solo Leveling", "Ruler's Authority", 
   "ARISE!", THEME_SUNG_JINWOO,
   {"Shadows", "10,000+", 85, "Power", "Monarch", 100,
    "Rank", "SSS", 87, "Mana", "Infinite", 70}},
  
  // Yugo
  {"Yugo", "Portal Master", "Wakfu", "Portal Slash", 
   "Adventure awaits!", THEME_YUGO_WAKFU,
   {"Wakfu", "Life force", 75, "Portals", "Dimensional", 80,
    "Speed", "Lightning", 95, "Heart", "Adventurer", 85}},
  
  // Naruto
  {"Naruto Uzumaki", "Sage Mode", "Naruto", "Rasengan", 
   "Believe it! Dattebayo!", THEME_NARUTO_SAGE,
   {"Chakra", "Nine-Tails", 100, "Sage", "Frog Kumite", 80,
    "Kurama", "Best friends", 95, "Bonds", "Never breaks", 100}},
  
  // Goku
  {"Son Goku", "Ultra Instinct", "Dragon Ball", "Kamehameha", 
   "I am the Saiyan who came from Earth!", THEME_GOKU_UI,
   {"Power", "Breaking limits", 100, "Speed", "Body moves", 100,
    "Ki", "Perfect", 95, "Forms", "SSJ to UI", 80}},
  
  // Tanjiro
  {"Tanjiro Kamado", "Sun Breathing", "Demon Slayer", "Hinokami Kagura", 
   "The bond between us can never be severed!", THEME_TANJIRO_SUN,
   {"Breathing", "13 forms", 80, "Forms", "All mastered", 95,
    "Smell", "Track demons", 100, "Will", "Never gives up", 100}},
  
  // Gojo
  {"Satoru Gojo", "Infinity", "Jujutsu Kaisen", "Hollow Purple", 
   "Throughout Heaven and Earth, I alone am the honored one.", THEME_GOJO_INFINITY,
   {"Infinity", "Perfect", 100, "Energy", "Unlimited", 100,
    "Six Eyes", "See all", 100, "Domain", "Unlimited Void", 100}},
  
  // Levi
  {"Levi Ackerman", "Humanity's Strongest", "Attack on Titan", "Spinning Slash", 
   "Give up on your dreams and die.", THEME_LEVI_STRONGEST,
   {"Kills", "58+ Titans", 100, "Speed", "Seconds/Titan", 100,
    "ODM Gear", "Unmatched", 100, "Clean", "OBSESSED", 100}},
  
  // Saitama
  {"Saitama", "One Punch", "One Punch Man", "Serious Punch", 
   "OK.", THEME_SAITAMA_OPM,
   {"Punch", "Infinite", 100, "Rank", "C-Class #7", 7,
    "Boredom", "No opponents", 100, "Bargains", "Expert", 100}},
  
  // Deku
  {"Izuku Midoriya", "Plus Ultra", "My Hero Academia", "United States of Smash", 
   "A hero is someone who saves people!", THEME_DEKU_PLUSULTRA,
   {"OFA", "45% control", 45, "Quirks", "7 of 9", 77,
    "Smash", "Building level", 80, "Spirit", "Saves all", 100}},
  
  // BoBoiBoy - NEW
  {"BoBoiBoy", "Elemental Hero", "BoBoiBoy Galaxy", "Elemental Fusion", 
   "BoBoiBoy Kuasa Tujuh!", THEME_BOBOIBOY,
   {"Elements", "7 Forms", 100, "Fusions", "6 Types", 85,
    "Power Band", "Ochobot", 90, "Teamwork", "TAPOPS", 95}}
};

// =============================================================================
// THEME MANAGEMENT
// =============================================================================

void initializeThemes() {
  Serial.println("[Themes] Initializing 10 anime character themes...");
  setTheme(system_state.current_theme);
}

void setTheme(ThemeType theme) {
  system_state.current_theme = theme;
  current_theme = getThemeColors(theme);
  Serial.printf("[Themes] Active: %s\n", getThemeName(theme));
}

// =============================================================================
// THEME TRANSITION ANIMATIONS - Character-specific cool effects
// =============================================================================

void playThemeTransition(ThemeType theme) {
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  switch(theme) {
    case THEME_SUNG_JINWOO: {
      // Jin-Woo: Black screen with "ARISE" text
      gfx->fillScreen(COLOR_BLACK);
      delay(200);
      
      // Purple mist effect rising from bottom
      for (int y = LCD_HEIGHT; y > centerY - 50; y -= 15) {
        for (int x = 0; x < LCD_WIDTH; x += 20) {
          int size = random(5, 15);
          gfx->fillCircle(x + random(-10, 10), y, size, JINWOO_MIST_PURPLE);
        }
        delay(30);
      }
      
      // "ARISE" text animation
      gfx->fillScreen(COLOR_BLACK);
      gfx->setTextColor(JINWOO_ARISE_GLOW);
      gfx->setTextSize(5);
      const char* arise = "ARISE";
      for (int i = 0; i < 5; i++) {
        gfx->setCursor(60 + i * 50, centerY - 30);
        gfx->print(arise[i]);
        delay(150);
      }
      
      // Flash effect
      delay(300);
      gfx->fillScreen(JINWOO_ARISE_GLOW);
      delay(100);
      break;
    }
    
    case THEME_LUFFY_GEAR5: {
      // Luffy: "Ha ha ha" laughing
      gfx->fillScreen(LUFFY_DEEP_BLACK);
      delay(200);
      
      // Sun rays effect
      for (int r = 0; r < 300; r += 20) {
        gfx->drawCircle(centerX, centerY, r, LUFFY_SUN_GOLD);
        delay(30);
      }
      
      // Laughing text
      gfx->fillScreen(LUFFY_SUN_GOLD);
      gfx->setTextColor(LUFFY_DEEP_BLACK);
      gfx->setTextSize(4);
      
      gfx->setCursor(centerX - 80, centerY - 60);
      gfx->print("HA");
      delay(200);
      gfx->setCursor(centerX - 40, centerY - 10);
      gfx->print("HA");
      delay(200);
      gfx->setCursor(centerX, centerY + 40);
      gfx->print("HA!");
      delay(400);
      
      gfx->fillScreen(COLOR_WHITE);
      delay(100);
      break;
    }
    
    case THEME_BOBOIBOY: {
      // BoBoiBoy: Flash with all element colors, "ELEMENTAL FUSION"
      uint16_t elementColors[] = {
        RGB565(255, 50, 0),    // Fire - Red/Orange
        RGB565(100, 200, 255), // Water - Blue
        RGB565(255, 220, 50),  // Lightning - Yellow
        RGB565(50, 255, 100),  // Wind - Green
        RGB565(139, 90, 43),   // Earth - Brown
        RGB565(255, 255, 255), // Light - White
        RGB565(80, 0, 120)     // Shadow - Purple
      };
      
      // Flash through all element colors
      for (int i = 0; i < 7; i++) {
        gfx->fillScreen(elementColors[i]);
        delay(80);
      }
      
      // Black with "ELEMENTAL FUSION" text
      gfx->fillScreen(COLOR_BLACK);
      delay(150);
      
      gfx->setTextColor(RGB565(255, 220, 50));
      gfx->setTextSize(2);
      gfx->setCursor(centerX - 60, centerY - 30);
      gfx->print("ELEMENTAL");
      delay(200);
      
      gfx->setTextColor(RGB565(255, 100, 50));
      gfx->setTextSize(3);
      gfx->setCursor(centerX - 60, centerY + 10);
      gfx->print("FUSION");
      delay(400);
      
      // Final flash
      gfx->fillScreen(RGB565(255, 220, 100));
      delay(100);
      break;
    }
    
    case THEME_NARUTO_SAGE: {
      // Naruto: Orange chakra swirl
      gfx->fillScreen(COLOR_BLACK);
      for (int r = 0; r < 250; r += 10) {
        float angle = r * 0.1;
        int sx = centerX + cos(angle) * r/2;
        int sy = centerY + sin(angle) * r/2;
        gfx->fillCircle(sx, sy, 20, NARUTO_CHAKRA_ORANGE);
        delay(20);
      }
      gfx->setTextColor(NARUTO_CHAKRA_ORANGE);
      gfx->setTextSize(3);
      gfx->setCursor(centerX - 80, centerY);
      gfx->print("DATTEBAYO!");
      delay(500);
      gfx->fillScreen(NARUTO_CHAKRA_ORANGE);
      delay(100);
      break;
    }
    
    case THEME_GOKU_UI: {
      // Goku: Silver aura pulse
      gfx->fillScreen(COLOR_BLACK);
      for (int pulse = 0; pulse < 3; pulse++) {
        for (int r = 50; r < 200; r += 30) {
          gfx->drawCircle(centerX, centerY, r, GOKU_UI_SILVER);
          gfx->drawCircle(centerX, centerY, r+1, GOKU_UI_SILVER);
          delay(30);
        }
        gfx->fillScreen(COLOR_BLACK);
        delay(100);
      }
      gfx->setTextColor(GOKU_UI_SILVER);
      gfx->setTextSize(2);
      gfx->setCursor(centerX - 80, centerY);
      gfx->print("ULTRA INSTINCT");
      delay(500);
      gfx->fillScreen(GOKU_UI_SILVER);
      delay(100);
      break;
    }
    
    case THEME_TANJIRO_SUN: {
      // Tanjiro: Fire breathing effect
      gfx->fillScreen(COLOR_BLACK);
      for (int i = 0; i < 15; i++) {
        int x = random(0, LCD_WIDTH);
        int y = LCD_HEIGHT - random(50, 200);
        int size = random(20, 60);
        gfx->fillCircle(x, y, size, TANJIRO_FIRE_ORANGE);
        gfx->fillCircle(x, y - size/2, size/2, TANJIRO_FLAME_GLOW);
        delay(50);
      }
      gfx->setTextColor(TANJIRO_FIRE_ORANGE);
      gfx->setTextSize(2);
      gfx->setCursor(centerX - 85, centerY);
      gfx->print("SUN BREATHING");
      delay(500);
      gfx->fillScreen(TANJIRO_FIRE_ORANGE);
      delay(100);
      break;
    }
    
    case THEME_GOJO_INFINITY: {
      // Gojo: DOMAIN EXPANSION - Unlimited Void
      gfx->fillScreen(COLOR_BLACK);
      delay(200);
      
      // "Domain Expansion" text first
      gfx->setTextColor(GOJO_INFINITY_BLUE);
      gfx->setTextSize(2);
      gfx->setCursor(centerX - 85, centerY - 40);
      gfx->print("DOMAIN EXPANSION");
      delay(600);
      
      // Screen goes to void blue-black
      gfx->fillScreen(RGB565(5, 10, 20));
      delay(200);
      
      // Domain EXPANDING outward from center - the void opening up
      for (int r = 10; r < 350; r += 15) {
        // Multiple expanding rings creating the infinite void effect
        gfx->drawCircle(centerX, centerY, r, GOJO_HOLLOW_PURPLE);
        gfx->drawCircle(centerX, centerY, r + 2, GOJO_INFINITY_BLUE);
        gfx->drawCircle(centerX, centerY, r + 4, RGB565(100, 150, 200));
        
        // Add some scattered infinity symbols/dots for the void particles
        for (int p = 0; p < 6; p++) {
          float angle = (r * 0.05) + (p * PI / 3);
          int px = centerX + cos(angle) * (r * 0.8);
          int py = centerY + sin(angle) * (r * 0.8);
          gfx->fillCircle(px, py, 2, GOJO_INFINITY_BLUE);
        }
        delay(25);
      }
      
      // Flash to white void
      gfx->fillScreen(RGB565(200, 220, 255));
      delay(100);
      gfx->fillScreen(COLOR_BLACK);
      delay(150);
      
      // "Limitless Void" - Gojo's domain name
      gfx->setTextColor(GOJO_INFINITY_BLUE);
      gfx->setTextSize(3);
      gfx->setCursor(centerX - 95, centerY - 15);
      gfx->print("LIMITLESS");
      gfx->setCursor(centerX - 50, centerY + 25);
      gfx->print("VOID");
      delay(600);
      
      // Final expansion flash
      for (int r = 0; r < 300; r += 50) {
        gfx->drawCircle(centerX, centerY, r, COLOR_WHITE);
        delay(20);
      }
      gfx->fillScreen(GOJO_INFINITY_BLUE);
      delay(100);
      break;
    }
    
    case THEME_LEVI_STRONGEST: {
      // Levi: Spinning blade slashes
      gfx->fillScreen(LEVI_CHARCOAL_DARK);
      for (int i = 0; i < 8; i++) {
        float angle = i * PI / 4;
        int x1 = centerX + cos(angle) * 20;
        int y1 = centerY + sin(angle) * 20;
        int x2 = centerX + cos(angle) * 180;
        int y2 = centerY + sin(angle) * 180;
        gfx->drawLine(x1, y1, x2, y2, LEVI_SILVER_BLADE);
        gfx->drawLine(x1+1, y1, x2+1, y2, LEVI_SILVER_BLADE);
        delay(60);
      }
      gfx->setTextColor(LEVI_SILVER_BLADE);
      gfx->setTextSize(2);
      gfx->setCursor(centerX - 90, centerY + 60);
      gfx->print("SHINZOU WO SASAGEYO");
      delay(500);
      gfx->fillScreen(LEVI_SILVER_BLADE);
      delay(100);
      break;
    }
    
    case THEME_SAITAMA_OPM: {
      // Saitama: Simple "OK" with punch effect
      gfx->fillScreen(SAITAMA_CAPE_RED);
      delay(200);
      gfx->fillScreen(COLOR_BLACK);
      gfx->setTextColor(SAITAMA_CAPE_RED);
      gfx->setTextSize(8);
      gfx->setCursor(centerX - 50, centerY - 40);
      gfx->print("OK");
      delay(600);
      // Punch impact
      for (int r = 10; r < 300; r += 40) {
        gfx->drawCircle(centerX, centerY, r, COLOR_WHITE);
        delay(30);
      }
      gfx->fillScreen(COLOR_WHITE);
      delay(100);
      break;
    }
    
    case THEME_DEKU_PLUSULTRA: {
      // Deku: Lightning crackling + PLUS ULTRA
      gfx->fillScreen(DEKU_DARK_HERO);
      for (int i = 0; i < 12; i++) {
        int x1 = random(50, LCD_WIDTH - 50);
        int y1 = random(50, LCD_HEIGHT - 50);
        for (int j = 0; j < 4; j++) {
          int x2 = x1 + random(-40, 40);
          int y2 = y1 + random(20, 60);
          gfx->drawLine(x1, y1, x2, y2, DEKU_OFA_LIGHTNING);
          gfx->drawLine(x1+1, y1, x2+1, y2, DEKU_FULL_COWL);
          x1 = x2;
          y1 = y2;
        }
        delay(50);
      }
      gfx->setTextColor(DEKU_FULL_COWL);
      gfx->setTextSize(3);
      gfx->setCursor(centerX - 95, centerY);
      gfx->print("PLUS ULTRA!");
      delay(500);
      gfx->fillScreen(DEKU_FULL_COWL);
      delay(100);
      break;
    }
    
    default: {
      // Default: Simple fade
      for (int b = 255; b > 0; b -= 25) {
        gfx->fillScreen(RGB565(b/10, b/10, b/8));
        delay(30);
      }
      break;
    }
  }
}

ThemeColors* getCurrentTheme() {
  return current_theme;
}

ThemeColors* getThemeColors(ThemeType theme) {
  switch(theme) {
    case THEME_LUFFY_GEAR5:    return &luffy_gear5_theme;
    case THEME_SUNG_JINWOO:    return &sung_jinwoo_theme;
    case THEME_YUGO_WAKFU:     return &yugo_wakfu_theme;
    case THEME_NARUTO_SAGE:    return &naruto_sage_theme;
    case THEME_GOKU_UI:        return &goku_ui_theme;
    case THEME_TANJIRO_SUN:    return &tanjiro_sun_theme;
    case THEME_GOJO_INFINITY:  return &gojo_infinity_theme;
    case THEME_LEVI_STRONGEST: return &levi_strongest_theme;
    case THEME_SAITAMA_OPM:    return &saitama_opm_theme;
    case THEME_DEKU_PLUSULTRA: return &deku_plusultra_theme;
    case THEME_BOBOIBOY:       return &boboiboy_elemental_theme;
    default: return &luffy_gear5_theme;
  }
}

CharacterProfile* getCharacterProfile(ThemeType theme) {
  if (theme < THEME_COUNT) {
    return &character_profiles[(int)theme];
  }
  return &character_profiles[0];
}

CharacterProfile* getCurrentCharacterProfile() {
  return getCharacterProfile(system_state.current_theme);
}

const char* getThemeName(ThemeType theme) {
  const char* names[] = {
    "Gear 5 Luffy", "Shadow Monarch", "Portal Master", "Sage Mode",
    "Ultra Instinct", "Sun Breathing", "Infinity", "Humanity's Strongest",
    "One Punch", "Plus Ultra", "BoBoiBoy", "Custom"
  };
  if (theme <= THEME_CUSTOM) return names[(int)theme];
  return "Unknown";
}

bool isMainCharacter(ThemeType theme) {
  return theme == THEME_LUFFY_GEAR5 || 
         theme == THEME_SUNG_JINWOO || 
         theme == THEME_YUGO_WAKFU;
}

ThemeType getDailyRotationTheme(int day_of_week) {
  ThemeType rotation[] = {
    THEME_NARUTO_SAGE, THEME_GOKU_UI, THEME_TANJIRO_SUN,
    THEME_GOJO_INFINITY, THEME_LEVI_STRONGEST, 
    THEME_SAITAMA_OPM, THEME_DEKU_PLUSULTRA
  };
  return rotation[day_of_week % 7];
}

void updateDailyCharacter() {
  WatchTime time = getCurrentTime();
  Serial.printf("[Themes] Daily: %s\n", getThemeName(getDailyRotationTheme(time.weekday)));
}

// =============================================================================
// WATCH FACE DISPATCHER
// =============================================================================

void drawWatchFace() {
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:    drawLuffyWatchFace(); break;
    case THEME_SUNG_JINWOO:    drawJinwooWatchFace(); break;
    case THEME_YUGO_WAKFU:     drawYugoWatchFace(); break;
    case THEME_NARUTO_SAGE:    drawNarutoWatchFace(); break;
    case THEME_GOKU_UI:        drawGokuWatchFace(); break;
    case THEME_TANJIRO_SUN:    drawTanjiroWatchFace(); break;
    case THEME_GOJO_INFINITY:  drawGojoWatchFace(); break;
    case THEME_LEVI_STRONGEST: drawLeviWatchFace(); break;
    case THEME_SAITAMA_OPM:    drawSaitamaWatchFace(); break;
    case THEME_DEKU_PLUSULTRA: drawDekuWatchFace(); break;
    case THEME_BOBOIBOY:       drawBoboiboyWatchFace(); break;
    default: drawLuffyWatchFace(); break;
  }
}

// =============================================================================
// PARTIAL WATCHFACE UPDATE - NO FLICKER
// Only redraws the time text and seconds indicator
// =============================================================================

void updateWatchFaceTime() {
  static int prev_hour = -1;
  static int prev_minute = -1;
  static int prev_second = -1;
  
  WatchTime time = getCurrentTime();
  
  // Nothing changed
  if (time.hour == prev_hour && time.minute == prev_minute && time.second == prev_second) {
    return;
  }
  
  prev_hour = time.hour;
  prev_minute = time.minute;
  prev_second = time.second;
  
  // FULL REDRAW to prevent ANY overlap artifacts
  // Each theme has unique layouts - partial updates cause the black bar issue
  drawWatchFace();
}

// =============================================================================
// LUFFY GEAR 5 WATCH FACE - MODERN SUN GOD (Optimized for 410x502)
// =============================================================================

void drawLuffyWatchFace() {
  // Pure AMOLED black
  gfx->fillScreen(0x0000);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;  // 205
  int centerY = 180;  // Adjusted for taller display
  
  // === AMBIENT SUN GLOW ===
  for (int r = 180; r > 0; r -= 4) {
    uint8_t alpha = map(r, 0, 180, 35, 0);
    gfx->drawCircle(centerX, centerY - 20, r, RGB565(alpha, alpha/3, 0));
  }
  
  // === TIME DISPLAY - Larger for bigger screen ===
  char hourStr[3], minStr[3];
  sprintf(hourStr, "%02d", time.hour);
  sprintf(minStr, "%02d", time.minute);
  
  int timeY = 110;
  
  // Glow effect
  gfx->setTextSize(10);  // Larger font
  gfx->setTextColor(RGB565(40, 30, 5));
  gfx->setCursor(15, timeY);
  gfx->print(hourStr);
  gfx->setCursor(215, timeY);
  gfx->print(minStr);
  
  // Main time
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(18, timeY);
  gfx->print(hourStr);
  gfx->setCursor(218, timeY);
  gfx->print(minStr);
  
  // Animated colon
  int colonX = 200;
  int colonY = timeY + 40;
  uint16_t colonColor = (time.second % 2) ? LUFFY_SUN_GOLD : RGB565(200, 150, 50);
  gfx->fillCircle(colonX, colonY - 22, 7, colonColor);
  gfx->fillCircle(colonX, colonY + 22, 7, colonColor);
  
  // Seconds arc - larger radius for bigger display
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 160;
    int sy = centerY - 10 + sin(a) * 70;
    gfx->fillCircle(sx, sy, 3, LUFFY_ENERGY_ORANGE);
  }
  
  // === DATE SECTION - Wider for bigger display ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  const char* months[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", 
                          "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
  
  int dateY = 265;
  int dateW = 260;
  int dateX = (LCD_WIDTH - dateW) / 2;
  gfx->fillRect(dateX, dateY, dateW, 35, RGB565(20, 15, 8));
  gfx->drawRect(dateX, dateY, dateW, 35, RGB565(80, 60, 25));
  gfx->fillRect(dateX, dateY, 5, 5, COLOR_GOLD);
  
  gfx->setTextSize(2);
  gfx->setTextColor(LUFFY_SUN_GOLD);
  gfx->setCursor(dateX + 15, dateY + 8);
  gfx->print(days[time.weekday % 7]);
  
  gfx->setTextSize(3);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(dateX + 100, dateY + 5);
  gfx->printf("%02d", time.day);
  
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(180, 140, 60));
  gfx->setCursor(dateX + 170, dateY + 8);
  gfx->print(months[(time.month - 1) % 12]);
  
  // === CHARACTER TITLE ===
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(100, 80, 30));
  gfx->setCursor(centerX - 40, 30);
  gfx->print("GEAR 5");
  
  // === STATS CARDS ===
  drawLuffyStatsCards();
  
  // === ACTIVITY RING - Positioned for taller display ===
  drawLuffyActivityRings(centerX, 440);
}

void drawLuffyStatsCards() {
  int cardY = 320;
  int cardH = 60;  // Taller cards
  int cardW = 120; // Wider cards
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(15, 20, 15));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, RGB565(50, 80, 50));
  gfx->fillRect(cardStartX, cardY, 5, 5, COLOR_GREEN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 160, 100));
  gfx->setCursor(cardStartX + 12, cardY + 10);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 230, 150));
  gfx->setCursor(cardStartX + 12, cardY + 30);
  gfx->printf("%d", system_state.steps_today);
  
  // Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = system_state.battery_percentage;
  uint16_t battColor = battPct > 20 ? RGB565(100, 180, 255) : RGB565(255, 100, 100);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 18, 22));
  gfx->drawRect(card2X, cardY, cardW, cardH, RGB565(50, 70, 90));
  gfx->fillRect(card2X, cardY, 5, 5, COLOR_BLUE);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 120, 160));
  gfx->setCursor(card2X + 12, cardY + 10);
  gfx->print("BATTERY");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 12, cardY + 30);
  gfx->printf("%d%%", battPct);
  
  // Gems card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(22, 18, 10));
  gfx->drawRect(card3X, cardY, cardW, cardH, RGB565(80, 65, 30));
  gfx->fillRect(card3X, cardY, 5, 5, COLOR_GOLD);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 110, 50));
  gfx->setCursor(card3X + 12, cardY + 10);
  gfx->print("GEMS");
  gfx->setTextSize(2);
  gfx->setTextColor(LUFFY_SUN_GOLD);
  gfx->setCursor(card3X + 12, cardY + 30);
  gfx->printf("%d", system_state.player_gems);
}

// =============================================================================
// JINWOO SHADOW MONARCH WATCH FACE - DARK POWER (Optimized for 410x502)
// =============================================================================

void drawJinwooWatchFace() {
  gfx->fillScreen(0x0000);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;  // 205
  int centerY = 180;  // Adjusted for taller display
  
  // === SHADOW PARTICLES ===
  static uint8_t particleY[15];
  static bool initialized = false;
  if (!initialized) {
    for (int i = 0; i < 15; i++) particleY[i] = random(0, LCD_HEIGHT);
    initialized = true;
  }
  
  for (int i = 0; i < 15; i++) {
    particleY[i] = (particleY[i] - 1 + LCD_HEIGHT) % LCD_HEIGHT;
    int px = 25 + (i * 25) % (LCD_WIDTH - 50);
    int size = 2 + (i % 3);
    uint8_t bright = 15 + (particleY[i] * 25) / LCD_HEIGHT;
    gfx->fillCircle(px, particleY[i], size, RGB565(bright, bright/3, bright + 15));
  }
  
  // === MONARCH HEXAGON - Larger ===
  for (int ring = 0; ring < 3; ring++) {
    int r = 120 + ring * 25;
    for (int i = 0; i < 6; i++) {
      float a1 = (i * 60) * PI / 180.0;
      float a2 = ((i + 1) * 60) * PI / 180.0;
      int x1 = centerX + cos(a1) * r;
      int y1 = centerY - 20 + sin(a1) * (r * 0.4);
      int x2 = centerX + cos(a2) * r;
      int y2 = centerY - 20 + sin(a2) * (r * 0.4);
      gfx->drawLine(x1, y1, x2, y2, RGB565(25 - ring*7, 12 - ring*3, 45 - ring*10));
    }
  }
  
  // === TIME - Larger ===
  char hourStr[3], minStr[3];
  sprintf(hourStr, "%02d", time.hour);
  sprintf(minStr, "%02d", time.minute);
  
  int timeY = 110;
  
  // Purple glow
  gfx->setTextSize(10);
  gfx->setTextColor(RGB565(30, 15, 50));
  gfx->setCursor(15, timeY);
  gfx->print(hourStr);
  gfx->setCursor(215, timeY);
  gfx->print(minStr);
  
  // Main time
  gfx->setTextColor(RGB565(230, 220, 255));
  gfx->setCursor(18, timeY);
  gfx->print(hourStr);
  gfx->setCursor(218, timeY);
  gfx->print(minStr);
  
  // Pulsing colon
  int colonX = 200;
  int colonY = timeY + 40;
  float pulse = 0.5 + 0.5 * sin(millis() / 300.0);
  uint8_t pulseVal = 100 + pulse * 155;
  gfx->fillCircle(colonX, colonY - 22, 7, RGB565(pulseVal/2, pulseVal/3, pulseVal));
  gfx->fillCircle(colonX, colonY + 22, 7, RGB565(pulseVal/2, pulseVal/3, pulseVal));
  
  // === ARISE BADGE - Wider ===
  int ariseY = 270;
  int badgeW = 140;
  for (int i = 2; i >= 0; i--) {
    gfx->fillRect(centerX - badgeW/2 - i*2, ariseY - i, badgeW + i*4, 35 + i,
                       RGB565(20 + i*8, 10 + i*4, 40 + i*12));
  }
  gfx->fillRect(centerX - badgeW/2, ariseY, badgeW, 32, RGB565(15, 8, 30));
  gfx->drawRect(centerX - badgeW/2, ariseY, badgeW, 32, JINWOO_ARISE_GLOW);
  
  gfx->setTextSize(3);
  gfx->setTextColor(JINWOO_ARISE_GLOW);
  gfx->setCursor(centerX - 50, ariseY + 4);
  gfx->print("ARISE!");
  
  // === DATE - Larger ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(100, 80, 140));
  char dateStr[15];
  sprintf(dateStr, "%s %02d.%02d", days[time.weekday % 7], time.day, time.month);
  gfx->setCursor(centerX - 60, 315);
  gfx->print(dateStr);
  
  // === SHADOW STATS - Larger cards ===
  int statsY = 350;
  int cardW = 120;
  int cardH = 58;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Shadows
  gfx->fillRect(cardStartX, statsY, cardW, cardH, RGB565(12, 8, 20));
  gfx->drawRect(cardStartX, statsY, cardW, cardH, RGB565(50, 35, 80));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 60, 120));
  gfx->setCursor(cardStartX + 12, statsY + 10);
  gfx->print("SHADOWS");
  gfx->setTextSize(2);
  gfx->setTextColor(JINWOO_MONARCH_PURPLE);
  gfx->setCursor(cardStartX + 12, statsY + 30);
  gfx->printf("%d", system_state.gacha_cards_collected);
  
  // Level
  int card2X = cardStartX + cardW + cardGap;
  gfx->fillRect(card2X, statsY, cardW, cardH, RGB565(12, 8, 20));
  gfx->drawRect(card2X, statsY, cardW, cardH, RGB565(50, 35, 80));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 60, 120));
  gfx->setCursor(card2X + 35, statsY + 10);
  gfx->print("LEVEL");
  gfx->setTextSize(3);
  gfx->setTextColor(JINWOO_ARISE_GLOW);
  gfx->setCursor(card2X + 35, statsY + 26);
  gfx->printf("%d", system_state.player_level);
  
  // Mana
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, statsY, cardW, cardH, RGB565(12, 8, 20));
  gfx->drawRect(card3X, statsY, cardW, cardH, RGB565(50, 35, 80));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 60, 120));
  gfx->setCursor(card3X + 35, statsY + 10);
  gfx->print("MANA");
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 120, 220));
  gfx->setCursor(card3X + 12, statsY + 30);
  gfx->printf("%d", system_state.player_gems);
  
  // Activity rings - positioned for taller display
  drawJinwooActivityRings(centerX, 445);
}

// =============================================================================
// REMAINING CHARACTER WATCH FACES (Optimized for 410x502)
// =============================================================================

void drawYugoWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(YUGO_SKY_BLUE_GREY);
  drawYugoPortals();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(YUGO_PORTAL_CYAN);
  gfx->setTextSize(3);
  gfx->setCursor(95, 35);
  gfx->print("PORTAL MASTER");
  
  gfx->setTextSize(7);
  gfx->setTextColor(YUGO_WAKFU_ENERGY);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextSize(3);
  gfx->setTextColor(YUGO_HAT_GOLD);
  gfx->setCursor(310, 210);
  gfx->printf("%02d", time.second);
  
  drawYugoActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawNarutoWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(NARUTO_SLATE_GREY);
  drawNarutoSageAura();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(NARUTO_CHAKRA_ORANGE);
  gfx->setTextSize(3);
  gfx->setCursor(120, 35);
  gfx->print("SAGE MODE");
  
  gfx->setTextSize(7);
  gfx->setTextColor(NARUTO_SAGE_GOLD);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextSize(3);
  gfx->setTextColor(NARUTO_KURAMA_FLAME);
  gfx->setCursor(310, 210);
  gfx->printf("%02d", time.second);
  
  gfx->setTextColor(NARUTO_CHAKRA_ORANGE);
  gfx->setTextSize(2);
  gfx->setCursor(130, 290);
  gfx->print("Believe it!");
  
  drawNarutoActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawGokuWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  drawGokuUIAura();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(GOKU_UI_SILVER);
  gfx->setTextSize(3);
  gfx->setCursor(75, 35);
  gfx->print("ULTRA INSTINCT");
  
  gfx->setTextSize(7);
  gfx->setTextColor(GOKU_DIVINE_SILVER);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  drawGokuSpeedLines();
  drawGokuActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawTanjiroWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(TANJIRO_DARK_CHARCOAL);
  drawTanjiroSunFlames();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(TANJIRO_FIRE_ORANGE);
  gfx->setTextSize(3);
  gfx->setCursor(85, 35);
  gfx->print("SUN BREATHING");
  
  gfx->setTextSize(7);
  gfx->setTextColor(TANJIRO_FLAME_GLOW);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextColor(TANJIRO_WATER_BLUE);
  gfx->setTextSize(2);
  gfx->setCursor(110, 290);
  gfx->print("Hinokami Kagura");
  
  drawTanjiroActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawGojoWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  drawGojoInfinityAura();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(GOJO_INFINITY_BLUE);
  gfx->setTextSize(3);
  gfx->setCursor(135, 35);
  gfx->print("INFINITY");
  
  gfx->setTextSize(7);
  gfx->setTextColor(GOJO_SIX_EYES_BLUE);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  drawGojoSixEyesGlow();
  
  gfx->setTextColor(GOJO_LIGHT_BLUE_GLOW);
  gfx->setTextSize(1);
  gfx->setCursor(70, 290);
  gfx->print("I alone am the honored one");
  
  drawGojoActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawLeviWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(LEVI_CHARCOAL_DARK);
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(LEVI_SURVEY_GREEN);
  gfx->setTextSize(2);
  gfx->setCursor(55, 35);
  gfx->print("HUMANITY'S STRONGEST");
  
  gfx->setTextSize(7);
  gfx->setTextColor(LEVI_CLEAN_WHITE);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextSize(3);
  gfx->setTextColor(LEVI_SILVER_BLADE);
  gfx->setCursor(310, 210);
  gfx->printf("%02d", time.second);
  
  drawLeviBladeShine();
  drawLeviActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawSaitamaWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(SAITAMA_HERO_YELLOW);
  gfx->setTextSize(3);
  gfx->setCursor(120, 35);
  gfx->print("ONE PUNCH");
  
  gfx->setTextSize(7);
  gfx->setTextColor(SAITAMA_BALD_WHITE);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextColor(SAITAMA_CAPE_RED);
  gfx->setTextSize(4);
  gfx->setCursor(170, 290);
  gfx->print("OK.");
  
  drawSaitamaActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawDekuWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(DEKU_DARK_HERO);
  drawDekuOFALightning();
  
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  gfx->setTextColor(DEKU_HERO_GREEN);
  gfx->setTextSize(3);
  gfx->setCursor(115, 35);
  gfx->print("PLUS ULTRA");
  
  gfx->setTextSize(7);
  gfx->setTextColor(DEKU_FULL_COWL);
  gfx->setCursor(50, 180);
  gfx->print(timeStr);
  
  gfx->setTextColor(DEKU_ALLMIGHT_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(130, 290);
  gfx->print("Full Cowl: 45%");
  
  drawDekuActivityRings(LCD_WIDTH/2, 390);
  drawBatteryIndicator();
  drawStepCounter();
}

void drawSleepWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  WatchTime time = getCurrentTime();
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  gfx->setTextSize(6);
  gfx->setTextColor(RGB565(50, 50, 50));
  gfx->setCursor(80, 220);
  gfx->print(timeStr);
}

// =============================================================================
// BOBOIBOY WATCH FACE - ELEMENTAL HERO (Optimized for 410x502)
// 7 Elements with Power Band UI
// =============================================================================

// Track current element for cycling animation
static int boboiboy_current_element = 0;
static unsigned long boboiboy_last_cycle = 0;

void drawBoboiboyWatchFace() {
  // CRITICAL FIX: Clear screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  
  // Use dynamic background based on time
  drawDynamicBackground(THEME_BOBOIBOY);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;  // 205
  int centerY = 180;  // Adjusted for taller display
  
  // Element colors array
  uint16_t elementColors[] = {
    BBB_LIGHTNING_YELLOW, BBB_WIND_BLUE, BBB_EARTH_BROWN,
    BBB_FIRE_RED, BBB_WATER_CYAN, BBB_LEAF_GREEN, BBB_LIGHT_GOLD
  };
  const char* elementNames[] = {
    "HALILINTAR", "TAUFAN", "GEMPA", "BLAZE", "ICE", "THORN", "SOLAR"
  };
  
  // Use manually selected element (tap-to-switch) instead of auto-cycle
  int currentElement = getCurrentBoboiboyElement();
  uint16_t currentColor = elementColors[currentElement];
  
  // === POWER BAND RING - Larger ===
  // Outer ring with element color
  for (int r = 155; r > 138; r -= 3) {
    gfx->drawCircle(centerX, centerY - 10, r, currentColor);
  }
  
  // Inner glow effect
  for (int r = 132; r > 115; r -= 5) {
    uint8_t alpha = map(r, 115, 132, 10, 40);
    gfx->drawCircle(centerX, centerY - 10, r, RGB565(alpha, alpha, alpha + 10));
  }
  
  // === ELEMENT INDICATOR - Larger dots ===
  // Draw 7 element dots around the ring
  for (int i = 0; i < 7; i++) {
    float angle = (i * 51.4 - 90) * PI / 180.0;  // 360/7 = ~51.4 degrees
    int dotX = centerX + cos(angle) * 150;
    int dotY = centerY - 10 + sin(angle) * 60;
    
    if (i == currentElement) {
      // Active element - filled and larger
      gfx->fillCircle(dotX, dotY, 10, elementColors[i]);
      gfx->drawCircle(dotX, dotY, 12, COLOR_WHITE);
    } else {
      // Inactive - smaller outline
      gfx->drawCircle(dotX, dotY, 6, elementColors[i]);
    }
  }
  
  // === TIME DISPLAY - Larger ===
  char hourStr[3], minStr[3];
  sprintf(hourStr, "%02d", time.hour);
  sprintf(minStr, "%02d", time.minute);
  
  int timeY = 110;
  
  // Glow effect with current element color
  gfx->setTextSize(10);
  gfx->setTextColor(RGB565(30, 25, 20));
  gfx->setCursor(15, timeY);
  gfx->print(hourStr);
  gfx->setCursor(215, timeY);
  gfx->print(minStr);
  
  // Main time
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(18, timeY);
  gfx->print(hourStr);
  gfx->setCursor(218, timeY);
  gfx->print(minStr);
  
  // Animated colon with element color - larger
  int colonX = 200;
  int colonY = timeY + 40;
  float pulse = 0.5 + 0.5 * sin(millis() / 250.0);
  uint8_t colAlpha = 150 + pulse * 105;
  gfx->fillCircle(colonX, colonY - 20, 7, currentColor);
  gfx->fillCircle(colonX, colonY + 20, 7, currentColor);
  
  // Seconds arc - larger radius
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 128;
    int sy = centerY - 10 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, currentColor);
  }
  
  // === ELEMENT NAME BADGE - Wider ===
  int badgeY = 265;
  int badgeW = 180;
  gfx->fillRect(centerX - badgeW/2, badgeY, badgeW, 35, RGB565(20, 22, 28));
  gfx->drawRect(centerX - badgeW/2, badgeY, badgeW, 35, currentColor);
  gfx->fillRect(centerX - badgeW/2, badgeY, 5, 5, currentColor);
  
  gfx->setTextColor(currentColor);
  gfx->setTextSize(3);
  int nameLen = strlen(elementNames[currentElement]) * 18;
  gfx->setCursor(centerX - nameLen/2, badgeY + 6);
  gfx->print(elementNames[currentElement]);
  
  // === DATE SECTION - Larger ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 315;
  
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 150, 160));
  char dateStr[20];
  sprintf(dateStr, "%s  %02d.%02d.%04d", days[time.weekday % 7], time.day, time.month, time.year);
  gfx->setCursor(centerX - 90, dateY);
  gfx->print(dateStr);
  
  // === STATS CARDS - Larger ===
  int cardY = 355;
  int cardH = 58;
  int cardW = 120;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Power card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(20, 15, 10));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, BBB_BAND_ORANGE);
  gfx->fillRect(cardStartX, cardY, 5, 5, BBB_BAND_ORANGE);
  gfx->setTextSize(1);
  gfx->setTextColor(BBB_BAND_ORANGE);
  gfx->setCursor(cardStartX + 12, cardY + 10);
  gfx->print("POWER");
  gfx->setTextSize(2);
  gfx->setTextColor(BBB_BAND_GLOW);
  gfx->setCursor(cardStartX + 12, cardY + 30);
  gfx->printf("%d%%", 100);
  
  // Steps card
  int card2X = cardStartX + cardW + cardGap;
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 20, 15));
  gfx->drawRect(card2X, cardY, cardW, cardH, BBB_LEAF_GREEN);
  gfx->fillRect(card2X, cardY, 5, 5, BBB_LEAF_GREEN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 160, 100));
  gfx->setCursor(card2X + 12, cardY + 10);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 230, 150));
  gfx->setCursor(card2X + 12, cardY + 30);
  gfx->printf("%d", system_state.steps_today);
  
  // Battery card - FIX: Use actual PMU reading
  int card3X = card2X + cardW + cardGap;
  int battPct = getBatteryPercentage();  // Read from AXP2101 PMU
  system_state.battery_percentage = battPct;  // Update cached value
  uint16_t battColor = battPct > 20 ? BBB_WIND_BLUE : BBB_FIRE_RED;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(15, 18, 22));
  gfx->drawRect(card3X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card3X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 120, 160));
  gfx->setCursor(card3X + 12, cardY + 10);
  gfx->print("BATTERY");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card3X + 12, cardY + 30);
  gfx->printf("%d%%", battPct);
  
  // === ACTIVITY RINGS - positioned for taller display ===
  drawBoboiboyActivityRings(centerX, 450);
  
  // === OCHOBOT ASSISTANT ===
  // Draw Ochobot in corner
  if (ochobot_pos.visible) {
    drawOchobot(LCD_WIDTH - 50, 75, OCHOBOT_IDLE, 0.9);
  }
  
  // === TAP HINT ===
  gfx->setTextColor(RGB565(80, 85, 95));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 65, 480);
  gfx->print("Tap center to switch!");
}

// =============================================================================
// CHARACTER EFFECTS (Optimized for 410x502)
// =============================================================================

void drawYugoPortals() {
  gfx->drawCircle(70, 110, 35, YUGO_PORTAL_CYAN);
  gfx->drawCircle(70, 110, 30, YUGO_WAKFU_ENERGY);
  gfx->drawCircle(LCD_WIDTH - 70, 110, 35, YUGO_PORTAL_CYAN);
  gfx->drawCircle(LCD_WIDTH - 70, 110, 30, YUGO_WAKFU_ENERGY);
}

void drawNarutoSageAura() {
  int cx = LCD_WIDTH / 2;
  int cy = LCD_HEIGHT / 2 - 60;
  for (int r = 70; r < 95; r += 5) {
    gfx->drawCircle(cx, cy, r, NARUTO_CHAKRA_ORANGE);
  }
}

void drawGokuUIAura() {
  int cx = LCD_WIDTH / 2;
  int cy = 140;
  for (int r = 60; r < 85; r += 4) {
    gfx->drawCircle(cx, cy, r, GOKU_SILVER_GLOW);
  }
}

void drawGokuSpeedLines() {
  for (int i = 0; i < 12; i++) {
    int x = random(0, LCD_WIDTH);
    int y1 = random(50, 160);
    gfx->drawLine(x, y1, x + random(-8, 8), y1 + random(25, 50), GOKU_SPEED_LINES);
  }
}

void drawTanjiroSunFlames() {
  for (int i = 0; i < 8; i++) {
    int x = 40 + i * 45;
    int y = 85;
    gfx->fillTriangle(x, y, x - 12, y + 30, x + 12, y + 30, TANJIRO_FLAME_GLOW);
  }
}

void drawGojoInfinityAura() {
  int cx = LCD_WIDTH / 2;
  int cy = 140;
  for (int r = 50; r < 80; r += 5) {
    gfx->drawCircle(cx, cy, r, GOJO_INFINITY_BLUE);
  }
}

void drawGojoSixEyesGlow() {
  gfx->fillCircle(LCD_WIDTH / 2 - 25, 60, 10, GOJO_SIX_EYES_BLUE);
  gfx->fillCircle(LCD_WIDTH / 2 + 25, 60, 10, GOJO_SIX_EYES_BLUE);
}

void drawLeviBladeShine() {
  gfx->drawLine(35, 85, 60, 150, LEVI_SILVER_BLADE);
  gfx->drawLine(LCD_WIDTH - 35, 85, LCD_WIDTH - 60, 150, LEVI_SILVER_BLADE);
}

void drawDekuOFALightning() {
  for (int i = 0; i < 7; i++) {
    int x1 = random(35, LCD_WIDTH - 35);
    int y1 = random(55, 110);
    gfx->drawLine(x1, y1, x1 + random(-25, 25), y1 + random(25, 50), DEKU_OFA_LIGHTNING);
  }
}

// =============================================================================
// ACTIVITY RINGS (Optimized for 410x502)
// =============================================================================

void drawActivityRing(int centerX, int centerY, int radius, float progress, uint16_t color, int thickness) {
  progress = constrain(progress, 0.0f, 1.0f);
  
  // Background ring
  for (int i = 0; i < 360; i += 3) {
    float a = i * PI / 180.0 - PI/2;
    int px = centerX + cos(a) * radius;
    int py = centerY + sin(a) * radius;
    gfx->drawPixel(px, py, RGB565(40, 40, 45));
  }
  
  // Progress arc
  int progressDeg = progress * 360;
  for (int i = 0; i < progressDeg; i += 3) {
    float a = i * PI / 180.0 - PI/2;
    int px = centerX + cos(a) * radius;
    int py = centerY + sin(a) * radius;
    gfx->fillCircle(px, py, thickness/2, color);
  }
}

void drawLuffyActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, LUFFY_SUN_GOLD, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, LUFFY_ENERGY_ORANGE, 6);
  
  // Center percentage
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 150));
  int pct = min(100, (int)(stepProgress * 100));
  gfx->setCursor(centerX - 10, centerY - 4);
  gfx->printf("%d%%", pct);
}

void drawJinwooActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, JINWOO_MONARCH_PURPLE, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, JINWOO_ARISE_GLOW, 6);
}

void drawYugoActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, YUGO_PORTAL_CYAN, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, YUGO_HAT_GOLD, 6);
}

void drawNarutoActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, NARUTO_CHAKRA_ORANGE, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, NARUTO_SAGE_GOLD, 6);
}

void drawGokuActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, GOKU_UI_SILVER, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, GOKU_KI_BLAST_BLUE, 6);
}

void drawTanjiroActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, TANJIRO_FIRE_ORANGE, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, TANJIRO_WATER_BLUE, 6);
}

void drawGojoActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, GOJO_INFINITY_BLUE, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, GOJO_HOLLOW_PURPLE, 6);
}

void drawLeviActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, LEVI_SURVEY_GREEN, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, LEVI_SILVER_BLADE, 6);
}

void drawSaitamaActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, SAITAMA_HERO_YELLOW, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, SAITAMA_CAPE_RED, 6);
}

void drawDekuActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  drawActivityRing(centerX, centerY, 40, stepProgress, DEKU_HERO_GREEN, 7);
  drawActivityRing(centerX, centerY, 28, 0.7, DEKU_OFA_LIGHTNING, 6);
}

void drawBoboiboyActivityRings(int centerX, int centerY) {
  float stepProgress = (float)system_state.steps_today / max(1, system_state.step_goal);
  
  // Outer ring - Power Band Orange (larger)
  drawActivityRing(centerX, centerY, 40, stepProgress, BBB_BAND_ORANGE, 7);
  
  // Inner ring - Cycles through element colors
  uint16_t elementColors[] = {
    BBB_LIGHTNING_YELLOW, BBB_WIND_BLUE, BBB_EARTH_BROWN,
    BBB_FIRE_RED, BBB_WATER_CYAN, BBB_LEAF_GREEN, BBB_LIGHT_GOLD
  };
  int elemIdx = (millis() / 1500) % 7;
  drawActivityRing(centerX, centerY, 28, 0.8, elementColors[elemIdx], 6);
  
  // Center - element icon (larger dot)
  gfx->fillCircle(centerX, centerY, 6, elementColors[elemIdx]);
}

// =============================================================================
// UI UTILITIES (Optimized for 410x502)
// =============================================================================

void drawStepCounter() {
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(12, 12);
  gfx->printf("%d steps", system_state.steps_today);
}

void drawBatteryIndicator() {
  // Read fresh battery data from AXP2101 PMU
  int battPct = getBatteryPercentage();
  if (battPct >= 0 && battPct <= 100) {
    system_state.battery_percentage = battPct;
  }
  
  int x = LCD_WIDTH - 55, y = 12;
  // Retro pixel battery - slightly larger
  gfx->drawRect(x, y, 45, 20, RGB565(80, 85, 100));
  gfx->fillRect(x + 45, y + 5, 5, 10, RGB565(80, 85, 100));
  
  // Clear inside first
  gfx->fillRect(x + 2, y + 2, 41, 16, RGB565(2, 2, 5));
  
  int fillWidth = (system_state.battery_percentage * 41) / 100;
  uint16_t color = system_state.battery_percentage > 20 ? RGB565(0, 200, 80) : COLOR_RED;
  if (system_state.is_charging) color = COLOR_GOLD;
  if (fillWidth > 0) {
    gfx->fillRect(x + 2, y + 2, fillWidth, 16, color);
  }
  
  // Battery percentage text
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setTextSize(1);
  gfx->setCursor(x - 28, y + 6);
  gfx->printf("%d%%", system_state.battery_percentage);
}

void drawThemeButton(int x, int y, int w, int h, const char* text, bool pressed) {
  uint16_t bg = pressed ? RGB565(25, 20, 35) : RGB565(15, 18, 25);
  uint16_t border = pressed ? current_theme->primary : RGB565(40, 45, 60);
  gfx->fillRect(x, y, w, h, bg);
  gfx->drawRect(x, y, w, h, border);
  gfx->fillRect(x, y, 4, 4, border);
  gfx->fillRect(x + w - 4, y, 4, 4, border);
  gfx->setTextColor(pressed ? current_theme->primary : RGB565(180, 185, 200));
  gfx->setTextSize(2);
  int textW = strlen(text) * 12;
  gfx->setCursor(x + (w - textW)/2, y + (h - 16)/2);
  gfx->print(text);
}

void drawGameButton(int x, int y, int w, int h, const char* text, bool pressed) {
  drawThemeButton(x, y, w, h, text, pressed);
}

void drawGlassPanel(int x, int y, int w, int h, uint8_t opacity) {
  gfx->fillRect(x, y, w, h, RGB565(12, 14, 20));
  gfx->drawRect(x, y, w, h, RGB565(40, 45, 60));
  gfx->fillRect(x, y, 5, 5, current_theme->primary);
  gfx->fillRect(x + w - 5, y, 5, 5, current_theme->primary);
}

void drawGlassButton(int x, int y, int w, int h, const char* text, bool pressed) {
  gfx->fillRect(x, y, w, h, pressed ? RGB565(18, 20, 30) : RGB565(15, 18, 25));
  gfx->drawRect(x, y, w, h, pressed ? current_theme->primary : RGB565(40, 45, 60));
  if (pressed) {
    gfx->fillRect(x, y, 4, 4, current_theme->primary);
    gfx->fillRect(x + w - 4, y, 4, 4, current_theme->primary);
  }
  gfx->setTextColor(pressed ? current_theme->primary : RGB565(180, 185, 200));
  gfx->setTextSize(2);
  int textW = strlen(text) * 12;
  gfx->setCursor(x + (w - textW)/2, y + (h - 16)/2);
  gfx->print(text);
}

void drawGlassStatBar(int x, int y, int w, int h, float progress, uint16_t color, const char* label) {
  gfx->fillRect(x, y, w, h, RGB565(8, 10, 14));
  gfx->drawRect(x, y, w, h, RGB565(30, 35, 50));
  int fillW = (int)(w * constrain(progress, 0.0f, 1.0f));
  if (fillW > 0) {
    gfx->fillRect(x + 1, y + 1, fillW - 2, h - 2, color);
  }
  // Pixel notches
  for (int nx = x; nx < x + w; nx += 20) {
    gfx->drawFastVLine(nx, y + 1, h - 2, RGB565(15, 18, 25));
  }
  if (label) {
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(x + 5, y + h/2 - 4);
    gfx->print(label);
  }
}

// =============================================================================
// CHARACTER STATS SCREEN
// =============================================================================

void drawCharacterStatsScreen() {
  // ========================================
  // RETRO ANIME CHARACTER STATS - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  CharacterProfile* profile = getCurrentCharacterProfile();
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(51, 14 + 1);
  gfx->print("CHARACTER STATS");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(50, 14);
  gfx->print("CHARACTER STATS");
  
  // Character info card - retro pixel
  gfx->fillRect(30, 55, 300, 90, RGB565(12, 14, 20));
  gfx->drawRect(30, 55, 300, 90, RGB565(40, 45, 60));
  gfx->fillRect(30, 55, 5, 5, theme->primary);
  gfx->fillRect(325, 55, 5, 5, theme->primary);
  gfx->fillRect(30, 140, 5, 5, theme->primary);
  gfx->fillRect(325, 140, 5, 5, theme->primary);
  
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(45, 65);
  gfx->print(profile->name);
  
  gfx->setTextColor(RGB565(180, 185, 200));
  gfx->setTextSize(1);
  gfx->setCursor(45, 90);
  gfx->print(profile->title);
  
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setCursor(45, 108);
  gfx->print(profile->series);
  
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(2);
  gfx->setCursor(45, 125);
  gfx->printf("Lv.%d", system_state.player_level);
  
  // Stat bars - retro pixel style
  int barY = 160;
  int barW = 270;
  int barH = 20;
  int spacing = 36;
  
  drawGlassStatBar(50, barY, barW, barH, profile->stats.stat1_value / 100.0f, theme->primary, profile->stats.stat1_name);
  drawGlassStatBar(50, barY + spacing, barW, barH, profile->stats.stat2_value / 100.0f, theme->accent, profile->stats.stat2_name);
  drawGlassStatBar(50, barY + spacing * 2, barW, barH, profile->stats.stat3_value / 100.0f, theme->effect1, profile->stats.stat3_name);
  drawGlassStatBar(50, barY + spacing * 3, barW, barH, profile->stats.stat4_value / 100.0f, theme->effect2, profile->stats.stat4_name);
  
  // Catchphrase - retro styled
  gfx->fillRect(40, barY + spacing * 4 + 10, 280, 24, RGB565(10, 12, 18));
  gfx->drawRect(40, barY + spacing * 4 + 10, 280, 24, RGB565(30, 35, 50));
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  gfx->setCursor(50, barY + spacing * 4 + 17);
  gfx->print(profile->catchphrase);
  
  // Swipe up indicator for progression
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 60, LCD_HEIGHT - 25);
  gfx->print("SWIPE UP: PROGRESSION");
  
  drawSwipeIndicator();
}

// =============================================================================
// CHARACTER PROGRESSION TREE SCREEN - Full Title List
// =============================================================================

static int progression_scroll = 0;

// Title definitions
struct Title {
  const char* name;
  const char* requirement;
  int level_req;
  bool unlocked;
};

// ==================== LUFFY TITLES ====================
Title LUFFY_TITLE_LIST[] = {
  {"Rubber Rookie", "Level 1 - Stretchy newcomer", 1, false},
  {"Straw Hat Fighter", "Level 5 - Determined crew member", 5, false},
  {"Gum-Gum Kid", "Level 10 - Young rubber pirate", 10, false},
  {"East Blue Champion", "Level 15 - Strongest in weakest sea", 15, false},
  {"Captain of Freedom", "Level 16 - Values freedom above all", 16, false},
  {"Sky Island Drifter", "Level 20 - Floating island adventurer", 20, false},
  {"Gear Second Surger", "Level 25 - Blood acceleration master", 25, false},
  {"Grand Line Veteran", "Level 30 - Pirate graveyard warrior", 30, false},
  {"Gear Third Brawler", "Level 31 - Giant bone balloon", 31, false},
  {"Gear Fourth Juggernaut", "Level 40 - Muscle balloon form", 40, false},
  {"Haki Awakener", "Level 45 - Power of will unlocked", 45, false},
  {"Conqueror's Vessel", "Level 50 - Kingly ambition wielder", 50, false},
  {"Emissary of Laughter", "Level 51 - Brings joy to oppressed", 51, false},
  {"Toon World Trickster", "Level 60 - Reality bends to will", 60, false},
  {"Nika's Wrath", "Level 65 - Sun God's fury vessel", 65, false},
  {"Joy Boy's Heir", "Level 70 - Inheritor of ancient will", 70, false},
  {"Toon God Warrior", "Level 71 - Cartoon power master", 71, false},
  {"Reality Cartoonist", "Level 80 - Draws reality with rubber", 80, false},
  {"Haki Harmonizer", "Level 85 - All Haki types fused", 85, false},
  {"Boundless Luffy", "Level 90 - Beyond physical limits", 90, false},
  {"Liberation Deity", "Level 95 - God of freedom and joy", 95, false},
  {"Sun God Nika", "Level 100 - Warrior of Liberation", 100, false}
};

// ==================== YUGO TITLES ====================
Title YUGO_TITLE_LIST[] = {
  {"Young Eliatrope", "Level 1 - Curious dragon child", 1, false},
  {"Curious Explorer", "Level 5 - Adventure seeker", 5, false},
  {"Brotherhood Member", "Level 10 - Tofu Brotherhood", 10, false},
  {"Wakfu Student", "Level 15 - Learning life energy", 15, false},
  {"Portal Initiate", "Level 16 - Dimensional magic start", 16, false},
  {"Warp Jumper", "Level 20 - Short teleportation", 20, false},
  {"Sadida Ally", "Level 25 - Tree people friend", 25, false},
  {"Dimensional Walker", "Level 30 - World traverser", 30, false},
  {"Shushu Whisperer", "Level 31 - Demon weapon speaker", 31, false},
  {"Wakfu Manipulator", "Level 40 - Life force control", 40, false},
  {"Rubilax Wielder", "Level 45 - Demon sword master", 45, false},
  {"Adamai's Brother", "Level 50 - Dragon twin bond", 50, false},
  {"Portal Architect", "Level 51 - Complex portals", 51, false},
  {"Temporal Traveler", "Level 60 - Time manipulation", 60, false},
  {"Prince of Eliatropes", "Level 65 - Royal dragon heir", 65, false},
  {"Inglorium Navigator", "Level 70 - Divine realm master", 70, false},
  {"Dimensional Weaver", "Level 71 - Reality shaper", 71, false},
  {"Keeper of Realities", "Level 80 - Dimension guardian", 80, false},
  {"Cosmic Warper", "Level 85 - Space-time bender", 85, false},
  {"Eliatrope King", "Level 90 - PATH CHOICE", 90, false},
  {"King of Dragons", "Level 95 - Dragon Path ruler", 95, false},
  {"Master of Portals", "Level 100 - Dimension lord", 100, false}
};

// ==================== JINWOO TITLES ====================
Title JINWOO_TITLE_LIST[] = {
  {"Weakest Hunter", "Level 1 - E-Rank at bottom", 1, false},
  {"E-Rank Grinder", "Level 5 - Persistent despite weak", 5, false},
  {"Dungeon Survivor", "Level 10 - Lived through death", 10, false},
  {"System User", "Level 15 - Chosen by system", 15, false},
  {"Rank D Ascender", "Level 16 - No longer weakest", 16, false},
  {"C-Rank Climber", "Level 25 - Recognized hunter", 25, false},
  {"Solo Dungeon Clearer", "Level 30 - Handles dungeons alone", 30, false},
  {"Grave Raiser", "Level 31 - First shadow soldier", 31, false},
  {"Shadow Initiate", "Level 35 - Commands the dead", 35, false},
  {"Shadow Commander", "Level 40 - Small shadow army", 40, false},
  {"B-Rank Hunter", "Level 45 - Elite hunter status", 45, false},
  {"Red Gate Conqueror", "Level 50 - Impossible dungeons", 50, false},
  {"Rank B Executioner", "Level 51 - Elite shadow power", 51, false},
  {"Dungeon Monarch", "Level 60 - Dungeon ruler", 60, false},
  {"Rank A Terminator", "Level 65 - National level threat", 65, false},
  {"Jeju Island Savior", "Level 70 - Saved from ants", 70, false},
  {"Shadow Monarch's Vessel", "Level 71 - Ancient power chosen", 71, false},
  {"Monarch of Shadows", "Level 80 - Shadow throne heir", 80, false},
  {"King of Death", "Level 85 - Life and death master", 85, false},
  {"Ashborn's Successor", "Level 90 - Full monarch power", 90, false},
  {"Darkness Embodied", "Level 95 - Shadow incarnation", 95, false},
  {"Shadow Monarch", "Level 100 - Supreme ruler of death", 100, false}
};

// ==================== GENERIC TITLES (for other characters) ====================
Title GENERIC_TITLE_LIST[] = {
  {"Newbie", "Level 1 - Start journey", 1, false},
  {"Apprentice", "Level 5", 5, false},
  {"Warrior", "Level 10", 10, false},
  {"Champion", "Level 15", 15, false},
  {"Elite", "Level 20", 20, false},
  {"Master", "Level 25", 25, false},
  {"Grandmaster", "Level 30", 30, false},
  {"Legend", "Level 40", 40, false},
  {"Mythic", "Level 50", 50, false},
  {"Immortal", "Level 75", 75, false},
  {"Transcendent", "Level 100", 100, false},
  {"Card Collector", "Collect 10 cards", 1, false},
  {"Gacha Addict", "Collect 50 cards", 1, false},
  {"Boss Slayer", "Defeat 5 bosses", 1, false},
  {"Boss Hunter", "Defeat 25 bosses", 1, false},
  {"Step Master", "Walk 10,000 steps", 1, false},
  {"Gem Hoarder", "Collect 10,000 gems", 1, false},
  {"Training Fanatic", "10 day streak", 1, false},
  {"Dedicated Player", "30 day login streak", 1, false}
};

void drawProgressionScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  CharacterProfile* profile = getCurrentCharacterProfile();
  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;
  
  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(centerX - 40, 14);
  gfx->print("TITLES");
  
  // Current level indicator
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH - 70, 18);
  gfx->printf("LVL %d", system_state.player_level);
  
  // Select title list based on current theme
  Title* titles;
  int numTitles;
  
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:
      titles = LUFFY_TITLE_LIST;
      numTitles = 22;
      break;
    case THEME_YUGO_WAKFU:
      titles = YUGO_TITLE_LIST;
      numTitles = 22;
      break;
    case THEME_SUNG_JINWOO:
      titles = JINWOO_TITLE_LIST;
      numTitles = 22;
      break;
    default:
      titles = GENERIC_TITLE_LIST;
      numTitles = 19;
      break;
  }
  
  // Update unlock status based on level
  for (int i = 0; i < numTitles; i++) {
    titles[i].unlocked = (system_state.player_level >= titles[i].level_req);
  }
  
  // For generic list, also check achievement-based titles
  if (system_state.current_theme != THEME_LUFFY_GEAR5 && 
      system_state.current_theme != THEME_YUGO_WAKFU &&
      system_state.current_theme != THEME_SUNG_JINWOO) {
    if (numTitles >= 12) titles[11].unlocked = (system_state.gacha_cards_collected >= 10);
    if (numTitles >= 13) titles[12].unlocked = (system_state.gacha_cards_collected >= 50);
    if (numTitles >= 14) titles[13].unlocked = (system_state.bosses_defeated >= 5);
    if (numTitles >= 15) titles[14].unlocked = (system_state.bosses_defeated >= 25);
    if (numTitles >= 16) titles[15].unlocked = (system_state.steps_today >= 10000);
    if (numTitles >= 17) titles[16].unlocked = (system_state.player_gems >= 10000);
    if (numTitles >= 18) titles[17].unlocked = (system_state.training_streak >= 10);
    if (numTitles >= 19) titles[18].unlocked = (system_state.daily_login_count >= 30);
  }
  
  // Count unlocked titles
  int unlockedCount = 0;
  for (int i = 0; i < numTitles; i++) {
    if (titles[i].unlocked) unlockedCount++;
  }
  
  // Show unlocked count
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(20, 18);
  gfx->printf("%d/%d", unlockedCount, numTitles);
  
  // Draw title cards (4 visible at a time)
  int startIdx = progression_scroll * 4;
  for (int i = 0; i < 4; i++) {
    int idx = startIdx + i;
    if (idx >= numTitles) break;
    
    int cardY = 58 + i * 90;
    bool unlocked = titles[idx].unlocked;
    
    // Card background
    gfx->fillRect(20, cardY, LCD_WIDTH - 40, 80, 
                  unlocked ? RGB565(15, 25, 15) : RGB565(12, 14, 20));
    gfx->drawRect(20, cardY, LCD_WIDTH - 40, 80, 
                  unlocked ? theme->primary : RGB565(40, 45, 60));
    
    if (unlocked) {
      // Corner accents for unlocked
      gfx->fillRect(20, cardY, 5, 5, theme->primary);
      gfx->fillRect(LCD_WIDTH - 25, cardY, 5, 5, theme->primary);
      gfx->fillRect(20, cardY + 75, 5, 5, theme->primary);
      gfx->fillRect(LCD_WIDTH - 25, cardY + 75, 5, 5, theme->primary);
    }
    
    // Trophy/lock icon
    int iconX = 45;
    int iconY = cardY + 40;
    if (unlocked) {
      // Trophy shape
      gfx->fillRect(iconX - 10, iconY - 15, 20, 18, COLOR_GOLD);
      gfx->fillRect(iconX - 6, iconY + 3, 12, 5, COLOR_GOLD);
      gfx->fillRect(iconX - 8, iconY + 8, 16, 4, RGB565(180, 140, 40));
    } else {
      // Lock shape
      gfx->drawRect(iconX - 8, iconY - 5, 16, 14, RGB565(80, 85, 100));
      gfx->drawCircle(iconX, iconY - 10, 6, RGB565(80, 85, 100));
    }
    
    // Title name
    gfx->setTextColor(unlocked ? theme->accent : RGB565(120, 125, 140));
    gfx->setTextSize(2);
    gfx->setCursor(75, cardY + 15);
    gfx->print(titles[idx].name);
    
    // Requirement
    gfx->setTextColor(unlocked ? RGB565(150, 180, 150) : RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(75, cardY + 42);
    gfx->print(titles[idx].requirement);
    
    // Status badge
    if (unlocked) {
      gfx->fillRect(LCD_WIDTH - 95, cardY + 52, 60, 18, RGB565(40, 80, 40));
      gfx->drawRect(LCD_WIDTH - 95, cardY + 52, 60, 18, RGB565(100, 200, 100));
      gfx->setTextColor(RGB565(150, 255, 150));
      gfx->setCursor(LCD_WIDTH - 88, cardY + 56);
      gfx->print("EARNED");
    } else {
      gfx->fillRect(LCD_WIDTH - 95, cardY + 52, 60, 18, RGB565(30, 30, 35));
      gfx->drawRect(LCD_WIDTH - 95, cardY + 52, 60, 18, RGB565(60, 65, 80));
      gfx->setTextColor(RGB565(100, 105, 120));
      gfx->setCursor(LCD_WIDTH - 88, cardY + 56);
      gfx->print("LOCKED");
    }
  }
  
  // Scroll indicators - LEFT/RIGHT navigation
  int maxScroll = (numTitles - 1) / 4;
  
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  
  // Left arrow
  gfx->setCursor(25, centerX);
  gfx->print("<");
  
  // Right arrow  
  gfx->setCursor(LCD_WIDTH - 35, centerX);
  gfx->print(">");
  
  // Page indicator at bottom
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setCursor(centerX - 25, LCD_HEIGHT - 25);
  gfx->printf("< %d/%d >", progression_scroll + 1, maxScroll + 1);
  
  // Swipe down hint
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setCursor(centerX - 45, LCD_HEIGHT - 12);
  gfx->print("SWIPE DOWN: BACK");
  
  drawSwipeIndicator();
}

void handleProgressionTouch(TouchGesture& gesture) {
  // Get max scroll based on current theme's title count
  int numTitles;
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:
    case THEME_YUGO_WAKFU:
    case THEME_SUNG_JINWOO:
      numTitles = 22;
      break;
    default:
      numTitles = 19;
      break;
  }
  int maxScroll = (numTitles - 1) / 4;
  
  if (gesture.event == TOUCH_SWIPE_LEFT) {
    // Swipe LEFT = next page (infinite loop)
    progression_scroll++;
    if (progression_scroll > maxScroll) {
      progression_scroll = 0;  // Loop back to start
    }
    drawProgressionScreen();
  } else if (gesture.event == TOUCH_SWIPE_RIGHT) {
    // Swipe RIGHT = previous page (infinite loop)
    progression_scroll--;
    if (progression_scroll < 0) {
      progression_scroll = maxScroll;  // Loop to end
    }
    drawProgressionScreen();
  } else if (gesture.event == TOUCH_SWIPE_DOWN) {
    // Swipe DOWN = go back to character stats
    progression_scroll = 0;
    system_state.current_screen = SCREEN_CHARACTER_STATS;
    drawCharacterStatsScreen();
  }
}

// Theme selector page state (shared between draw and touch handler)
static int themePageState = 0;

void drawThemeSelector() {
  // ========================================
  // RETRO ANIME THEME SELECTOR - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 44, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 42, 6, 3, theme->primary);
  }
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 42, 12);
  gfx->print("THEMES");
  
  // Page indicators - pixel squares
  gfx->fillRect(LCD_WIDTH - 28, 18, 8, 8, themePageState == 0 ? theme->primary : RGB565(40, 45, 60));
  gfx->fillRect(LCD_WIDTH - 14, 18, 8, 8, themePageState == 1 ? theme->primary : RGB565(40, 45, 60));
  
  if (themePageState == 0) {
    const char* names[] = {"Luffy", "Jin-Woo", "Yugo", "Naruto", "Goku", "Tanjiro"};
    ThemeColors* themes[] = {&luffy_gear5_theme, &sung_jinwoo_theme, &yugo_wakfu_theme,
                             &naruto_sage_theme, &goku_ui_theme, &tanjiro_sun_theme};
    
    for (int i = 0; i < 6; i++) {
      int x = (i % 2) * 170 + 15;
      int y = (i / 2) * 100 + 52;
      
      // Retro pixel card
      gfx->fillRect(x, y, 155, 85, RGB565(12, 14, 20));
      gfx->fillRect(x + 4, y + 4, 147, 28, themes[i]->primary);
      // CRT lines on color bar
      for (int sy = y + 5; sy < y + 31; sy += 3) {
        gfx->drawFastHLine(x + 5, sy, 145, RGB565(0, 0, 0));
      }
      
      if ((ThemeType)i == system_state.current_theme) {
        gfx->drawRect(x - 2, y - 2, 159, 89, COLOR_WHITE);
        gfx->drawRect(x - 1, y - 1, 157, 87, theme->accent);
      } else {
        gfx->drawRect(x, y, 155, 85, RGB565(40, 45, 60));
      }
      // Pixel corners
      gfx->fillRect(x, y, 5, 5, themes[i]->primary);
      gfx->fillRect(x + 150, y, 5, 5, themes[i]->primary);
      
      gfx->setTextColor(RGB565(200, 205, 220));
      gfx->setTextSize(2);
      int textLen = strlen(names[i]) * 12;
      gfx->setCursor(x + (155 - textLen) / 2, y + 50);
      gfx->print(names[i]);
    }
  } else {
    const char* names[] = {"Gojo", "Levi", "Saitama", "Deku", "BoBoiBoy"};
    ThemeColors* themes[] = {&gojo_infinity_theme, &levi_strongest_theme, 
                             &saitama_opm_theme, &deku_plusultra_theme, &boboiboy_elemental_theme};
    ThemeType types[] = {THEME_GOJO_INFINITY, THEME_LEVI_STRONGEST, 
                         THEME_SAITAMA_OPM, THEME_DEKU_PLUSULTRA, THEME_BOBOIBOY};
    
    for (int i = 0; i < 5; i++) {
      int col = i % 2;
      int row = i / 2;
      int x = col * 170 + 15;
      int y = row * 100 + 52;
      
      if (i == 4) {
        x = (LCD_WIDTH - 155) / 2;
        y = 2 * 100 + 52;
      }
      
      gfx->fillRect(x, y, 155, 85, RGB565(12, 14, 20));
      gfx->fillRect(x + 4, y + 4, 147, 28, themes[i]->primary);
      for (int sy = y + 5; sy < y + 31; sy += 3) {
        gfx->drawFastHLine(x + 5, sy, 145, RGB565(0, 0, 0));
      }
      
      if (types[i] == system_state.current_theme) {
        gfx->drawRect(x - 2, y - 2, 159, 89, COLOR_WHITE);
        gfx->drawRect(x - 1, y - 1, 157, 87, theme->accent);
      } else {
        gfx->drawRect(x, y, 155, 85, RGB565(40, 45, 60));
      }
      gfx->fillRect(x, y, 5, 5, themes[i]->primary);
      gfx->fillRect(x + 150, y, 5, 5, themes[i]->primary);
      
      gfx->setTextColor(RGB565(200, 205, 220));
      gfx->setTextSize(2);
      int textLen = strlen(names[i]) * 12;
      gfx->setCursor(x + (155 - textLen) / 2, y + 50);
      gfx->print(names[i]);
    }
  }
  
  // Swipe hint - retro
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT - 28);
  gfx->print("< SWIPE >");
  
  drawSwipeIndicator();
}

void handleThemeSelectorTouch(TouchGesture& gesture) {
  // Swipe LEFT/RIGHT to switch pages
  if (gesture.event == TOUCH_SWIPE_LEFT) {
    if (themePageState < 1) {
      themePageState++;
      drawThemeSelector();
    }
    return;
  }
  if (gesture.event == TOUCH_SWIPE_RIGHT) {
    if (themePageState > 0) {
      themePageState--;
      drawThemeSelector();
    }
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Forward declaration for save function
  extern void saveAllGameData();
  
  // Theme selection - tap on theme cards
  if (themePageState == 0) {
    for (int i = 0; i < 6; i++) {
      int tx = (i % 2) * 170 + 15;
      int ty = (i / 2) * 105 + 52;
      if (x >= tx && x < tx + 155 && y >= ty && y < ty + 90) {
        ThemeType newTheme = (ThemeType)i;
        playThemeTransition(newTheme);  // Play cool animation
        setTheme(newTheme);
        saveAllGameData();  // Save theme to NVS
        system_state.current_screen = SCREEN_WATCHFACE;  // Go to watch face
        drawWatchFace();
        return;
      }
    }
  } else {
    ThemeType types[] = {THEME_GOJO_INFINITY, THEME_LEVI_STRONGEST, 
                         THEME_SAITAMA_OPM, THEME_DEKU_PLUSULTRA, THEME_BOBOIBOY};
    for (int i = 0; i < 5; i++) {
      int col = i % 2;
      int row = i / 2;
      int tx = col * 170 + 15;
      int ty = row * 105 + 52;
      
      if (i == 4) {
        tx = (LCD_WIDTH - 155) / 2;
        ty = 2 * 105 + 52;
      }
      
      if (x >= tx && x < tx + 155 && y >= ty && y < ty + 90) {
        playThemeTransition(types[i]);  // Play cool animation
        setTheme(types[i]);
        saveAllGameData();  // Save theme to NVS
        system_state.current_screen = SCREEN_WATCHFACE;  // Go to watch face
        drawWatchFace();
        return;
      }
    }
  }
}
