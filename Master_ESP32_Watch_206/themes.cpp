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
#include "xp_system.h"  // FUSION OS: For XP progress and titles
#include "dynamic_bg.h"
#include "sd_preferences.h"
#include "ochobot.h"
#include "navigation.h"
#include "companion.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;
extern void forceAppGridRedraw();
// Force redraw flag for character stats - enables instant loading on navigation
bool g_force_char_stats_redraw = true;

// Forward declarations for visual effects
void drawYugoPortals();
void drawYugoPortalEffects();
void drawNarutoSageAura();
void drawNarutoSageAuraEnhanced();
void drawGokuUIAura();
void drawGokuUIAuraEnhanced();
void drawGokuSpeedLines();
void drawTanjiroSunFlames();
void drawTanjiroSunFlamesEnhanced();
void drawGojoInfinityAura();
void drawGojoInfinityAuraEnhanced();
void drawGojoSixEyesGlow();
void drawGojoSixEyesGlowEnhanced();
void drawLeviBladeShine();
void drawLeviWingsEffect();
void drawLeviBladeShineEnhanced();
void drawDekuOFALightning();
void drawDekuOFALightningEnhanced();
void drawSaitamaImpactLines();

// =============================================================================
// XP DATA LOADING FOR THEME CHANGES - Fixes title equip bug
// =============================================================================
void loadXPDataForTheme(ThemeType theme) {
  // This function ensures XP data is properly loaded for the selected character
  // when theme changes. This fixes the bug where Luffy's titles would be shown
  // instead of the new character's titles after a theme change.
  
  extern void loadXPData();  // Load XP from NVS
  extern CharacterXPData* getCurrentCharacterXP();
  
  // Load XP data from NVS (this reads character-specific data based on theme)
  loadXPData();
  
  // Ensure system state is updated with current character's level/XP
  CharacterXPData* char_xp = getCurrentCharacterXP();
  if (char_xp) {
    system_state.player_level = char_xp->level;
    system_state.player_xp = char_xp->xp;
    
    Serial.printf("[XP] Loaded XP data for theme %d: Level %d, XP %ld, Equipped Title: %d\n",
                  theme, char_xp->level, char_xp->xp, char_xp->equipped_title_index);
  }
}

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
  syncCompanionWithTheme();  // Companion follows theme
  Serial.printf("[Themes] Active: %s\n", getThemeName(theme));
}

// =============================================================================
// THEME TRANSITION ANIMATIONS - Character-specific cool effects
// =============================================================================

void playThemeTransition(ThemeType theme) {
  extern void feedWatchdog();
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  feedWatchdog();  // Feed before starting any animation
  
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();  // Feed mid-animation — Gojo's is the longest
      
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();
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
      feedWatchdog();
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
  // Force full redraw flag for this draw cycle
  extern void forceWatchfaceRedraw();
  forceWatchfaceRedraw();
  
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

// Dirty flag to track if full redraw is needed
static bool watchface_needs_full_redraw = true;

void forceWatchfaceRedraw() {
  watchface_needs_full_redraw = true;
}

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
  
  // Only full redraw when absolutely needed (on first draw or theme change)
  if (watchface_needs_full_redraw) {
    drawWatchFace();
    watchface_needs_full_redraw = false;
  }
  // For time updates, just redraw time area (no flickering)
  // Most watch faces have time in center, so we update that region only
  // Note: For production, each theme would have its own partial update region
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
  
  // Enhanced portal effects
  drawYugoPortalEffects();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with shadow glow
  gfx->setTextColor(RGB565(20, 40, 50));
  gfx->setTextSize(3);
  gfx->setCursor(97, 37);
  gfx->print("PORTAL MASTER");
  gfx->setTextColor(YUGO_PORTAL_CYAN);
  gfx->setCursor(95, 35);
  gfx->print("PORTAL MASTER");
  
  // Time with enhanced shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(10, 30, 40));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(YUGO_PORTAL_GLOW);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds with portal glow
  gfx->setTextSize(3);
  gfx->setTextColor(YUGO_WAKFU_ENERGY);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Seconds arc - portal energy ring
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, YUGO_PORTAL_CYAN);
  }
  
  // Character tagline
  gfx->setTextColor(YUGO_HAT_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(100, 270);
  gfx->print("Adventure awaits!");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 302;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(130, 160, 170));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 340;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(15, 25, 30));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, RGB565(50, 100, 100));
  gfx->fillRect(cardStartX, cardY, 5, 5, YUGO_PORTAL_CYAN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 140, 140));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(YUGO_PORTAL_GLOW);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Battery card with percentage
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? YUGO_WAKFU_ENERGY : RGB565(255, 100, 100);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 22, 28));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 120, 140));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("POWER");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Wakfu energy card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(20, 22, 18));
  gfx->drawRect(card3X, cardY, cardW, cardH, RGB565(80, 75, 40));
  gfx->fillRect(card3X, cardY, 5, 5, YUGO_HAT_GOLD);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(120, 110, 60));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("WAKFU");
  gfx->setTextSize(2);
  gfx->setTextColor(YUGO_HAT_GOLD);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("100%");
  
  drawYugoActivityRings(centerX, 435);
}

void drawNarutoWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(NARUTO_SLATE_GREY);
  
  // Enhanced sage aura
  drawNarutoSageAuraEnhanced();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with glow shadow
  gfx->setTextColor(RGB565(40, 25, 10));
  gfx->setTextSize(3);
  gfx->setCursor(122, 37);
  gfx->print("SAGE MODE");
  gfx->setTextColor(NARUTO_CHAKRA_ORANGE);
  gfx->setCursor(120, 35);
  gfx->print("SAGE MODE");
  
  // Time with shadow effect
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(40, 25, 10));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(NARUTO_SAGE_GOLD);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds with chakra glow
  gfx->setTextSize(3);
  gfx->setTextColor(NARUTO_KURAMA_FLAME);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Chakra arc - seconds progress
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, NARUTO_CHAKRA_ORANGE);
  }
  
  // Catchphrase
  gfx->setTextColor(NARUTO_CHAKRA_ORANGE);
  gfx->setTextSize(2);
  gfx->setCursor(100, 270);
  gfx->print("Believe it! Dattebayo!");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 302;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 130, 100));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 340;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(20, 20, 15));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, RGB565(80, 70, 50));
  gfx->fillRect(cardStartX, cardY, 5, 5, NARUTO_CHAKRA_ORANGE);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 90, 60));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(NARUTO_SAGE_GOLD);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Battery/Chakra card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? NARUTO_RASENGAN_BLUE : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(18, 20, 25));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(70, 100, 140));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("CHAKRA");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Kurama card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(25, 18, 12));
  gfx->drawRect(card3X, cardY, cardW, cardH, NARUTO_KURAMA_FLAME);
  gfx->fillRect(card3X, cardY, 5, 5, NARUTO_KURAMA_FLAME);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 90, 50));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("KURAMA");
  gfx->setTextSize(2);
  gfx->setTextColor(NARUTO_KURAMA_FLAME);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("100%");
  
  drawNarutoActivityRings(centerX, 435);
}

void drawGokuWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  drawGokuUIAuraEnhanced();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with silver glow
  gfx->setTextColor(RGB565(30, 35, 45));
  gfx->setTextSize(2);
  gfx->setCursor(77, 37);
  gfx->print("ULTRA INSTINCT");
  gfx->setTextColor(GOKU_UI_SILVER);
  gfx->setCursor(75, 35);
  gfx->print("ULTRA INSTINCT");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(30, 35, 45));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(GOKU_DIVINE_SILVER);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(GOKU_KI_BLAST_BLUE);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Speed lines and ki arc
  drawGokuSpeedLines();
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, GOKU_UI_SILVER);
  }
  
  // Tagline
  gfx->setTextColor(GOKU_AURA_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(90, 270);
  gfx->print("The body moves on its own");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 292;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(130, 140, 150));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 330;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(12, 15, 20));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, RGB565(60, 70, 85));
  gfx->fillRect(cardStartX, cardY, 5, 5, GOKU_UI_SILVER);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(90, 100, 115));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(GOKU_UI_SILVER);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Ki/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? GOKU_KI_BLAST_BLUE : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 18, 25));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(70, 100, 150));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("KI");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Power Level card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(18, 18, 22));
  gfx->drawRect(card3X, cardY, cardW, cardH, GOKU_DIVINE_SILVER);
  gfx->fillRect(card3X, cardY, 5, 5, GOKU_DIVINE_SILVER);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(120, 125, 135));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("POWER");
  gfx->setTextSize(2);
  gfx->setTextColor(GOKU_DIVINE_SILVER);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("9001+");
  
  drawGokuActivityRings(centerX, 425);
}

void drawTanjiroWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(TANJIRO_DARK_CHARCOAL);
  drawTanjiroSunFlamesEnhanced();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with flame glow
  gfx->setTextColor(RGB565(50, 25, 10));
  gfx->setTextSize(2);
  gfx->setCursor(87, 37);
  gfx->print("SUN BREATHING");
  gfx->setTextColor(TANJIRO_FIRE_ORANGE);
  gfx->setCursor(85, 35);
  gfx->print("SUN BREATHING");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(50, 25, 10));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(TANJIRO_FLAME_GLOW);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(TANJIRO_WATER_BLUE);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Flame arc
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, TANJIRO_FIRE_ORANGE);
  }
  
  // Tagline
  gfx->setTextColor(TANJIRO_WATER_BLUE);
  gfx->setTextSize(2);
  gfx->setCursor(95, 270);
  gfx->print("Hinokami Kagura");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 302;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(140, 120, 100));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 340;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(18, 22, 20));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, TANJIRO_CHECK_GREEN);
  gfx->fillRect(cardStartX, cardY, 5, 5, TANJIRO_CHECK_GREEN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 110, 90));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(TANJIRO_CHECK_GREEN);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Breathing/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? TANJIRO_WATER_BLUE : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 18, 25));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(70, 100, 140));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("BREATH");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Sun form card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(25, 18, 12));
  gfx->drawRect(card3X, cardY, cardW, cardH, TANJIRO_FIRE_ORANGE);
  gfx->fillRect(card3X, cardY, 5, 5, TANJIRO_FIRE_ORANGE);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 90, 50));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("FORMS");
  gfx->setTextSize(2);
  gfx->setTextColor(TANJIRO_FIRE_ORANGE);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("13");
  
  drawTanjiroActivityRings(centerX, 435);
}

void drawGojoWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  drawGojoInfinityAuraEnhanced();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with infinity glow
  gfx->setTextColor(RGB565(20, 30, 50));
  gfx->setTextSize(3);
  gfx->setCursor(137, 37);
  gfx->print("INFINITY");
  gfx->setTextColor(GOJO_INFINITY_BLUE);
  gfx->setCursor(135, 35);
  gfx->print("INFINITY");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(20, 30, 50));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(GOJO_SIX_EYES_BLUE);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(GOJO_HOLLOW_PURPLE);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Six Eyes glow
  drawGojoSixEyesGlowEnhanced();
  
  // Infinity arc
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, GOJO_INFINITY_BLUE);
  }
  
  // Tagline
  gfx->setTextColor(GOJO_LIGHT_BLUE_GLOW);
  gfx->setTextSize(1);
  gfx->setCursor(55, 270);
  gfx->print("Throughout Heaven and Earth...");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 292;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(120, 140, 170));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 330;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(12, 15, 25));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, GOJO_INFINITY_BLUE);
  gfx->fillRect(cardStartX, cardY, 5, 5, GOJO_INFINITY_BLUE);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(70, 100, 150));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(GOJO_INFINITY_BLUE);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Cursed Energy/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? GOJO_LIGHT_BLUE_GLOW : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 18, 28));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 120, 170));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("CURSED E");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Domain card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(18, 12, 25));
  gfx->drawRect(card3X, cardY, cardW, cardH, GOJO_HOLLOW_PURPLE);
  gfx->fillRect(card3X, cardY, 5, 5, GOJO_HOLLOW_PURPLE);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 70, 140));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("DOMAIN");
  gfx->setTextSize(2);
  gfx->setTextColor(GOJO_HOLLOW_PURPLE);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("VOID");
  
  drawGojoActivityRings(centerX, 425);
}

void drawLeviWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(LEVI_CHARCOAL_DARK);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Survey Corps wings effect
  drawLeviWingsEffect();
  
  // Title with military precision
  gfx->setTextColor(RGB565(20, 30, 25));
  gfx->setTextSize(2);
  gfx->setCursor(57, 37);
  gfx->print("HUMANITY'S STRONGEST");
  gfx->setTextColor(LEVI_SURVEY_GREEN);
  gfx->setCursor(55, 35);
  gfx->print("HUMANITY'S STRONGEST");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(25, 30, 35));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(LEVI_CLEAN_WHITE);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(LEVI_SILVER_BLADE);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Blade shine effect
  drawLeviBladeShineEnhanced();
  
  // Steel arc
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, LEVI_SILVER_BLADE);
  }
  
  // Tagline
  gfx->setTextColor(LEVI_MILITARY_GREY);
  gfx->setTextSize(1);
  gfx->setCursor(85, 270);
  gfx->print("Give up on your dreams and die.");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 292;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(120, 125, 130));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 330;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps/Distance card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(15, 18, 20));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, LEVI_SURVEY_GREEN);
  gfx->fillRect(cardStartX, cardY, 5, 5, LEVI_SURVEY_GREEN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 100, 90));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("ODM DIST");
  gfx->setTextSize(2);
  gfx->setTextColor(LEVI_SURVEY_GREEN);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Gas/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? LEVI_SILVER_BLADE : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(18, 20, 22));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 110, 120));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("GAS");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Titan Kills card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(20, 15, 15));
  gfx->drawRect(card3X, cardY, cardW, cardH, RGB565(150, 80, 80));
  gfx->fillRect(card3X, cardY, 5, 5, RGB565(150, 80, 80));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(130, 90, 90));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("KILLS");
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(180, 100, 100));
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("58+");
  
  drawLeviActivityRings(centerX, 425);
}

void drawSaitamaWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(COLOR_BLACK);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Impact lines background
  drawSaitamaImpactLines();
  
  // Title with punch impact
  gfx->setTextColor(RGB565(50, 40, 10));
  gfx->setTextSize(3);
  gfx->setCursor(122, 37);
  gfx->print("ONE PUNCH");
  gfx->setTextColor(SAITAMA_HERO_YELLOW);
  gfx->setCursor(120, 35);
  gfx->print("ONE PUNCH");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(40, 40, 45));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(SAITAMA_BALD_WHITE);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(SAITAMA_CAPE_RED);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Impact arc
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, SAITAMA_HERO_YELLOW);
  }
  
  // THE iconic "OK."
  gfx->setTextColor(SAITAMA_CAPE_RED);
  gfx->setTextSize(4);
  gfx->setCursor(168, 265);
  gfx->print("OK.");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 310;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(140, 140, 140));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 350;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(18, 18, 15));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, SAITAMA_HERO_YELLOW);
  gfx->fillRect(cardStartX, cardY, 5, 5, SAITAMA_HERO_YELLOW);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(120, 110, 60));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("10KM RUN");
  gfx->setTextSize(2);
  gfx->setTextColor(SAITAMA_HERO_YELLOW);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // Power/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? SAITAMA_BALD_WHITE : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(15, 15, 18));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(120, 120, 130));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("POWER");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Hero Rank card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(25, 15, 15));
  gfx->drawRect(card3X, cardY, cardW, cardH, SAITAMA_CAPE_RED);
  gfx->fillRect(card3X, cardY, 5, 5, SAITAMA_CAPE_RED);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 80, 80));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("RANK");
  gfx->setTextSize(2);
  gfx->setTextColor(SAITAMA_CAPE_RED);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("B #7");
  
  drawSaitamaActivityRings(centerX, 440);
}

void drawDekuWatchFace() {
  // CRITICAL FIX: Clear entire screen first to prevent overlap from other watchfaces
  gfx->fillScreen(RGB565(2, 2, 5));
  gfx->fillScreen(DEKU_DARK_HERO);
  drawDekuOFALightningEnhanced();
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;
  int centerY = 180;
  
  char timeStr[16];
  sprintf(timeStr, "%02d:%02d", time.hour, time.minute);
  
  // Title with lightning glow
  gfx->setTextColor(RGB565(20, 40, 30));
  gfx->setTextSize(3);
  gfx->setCursor(117, 37);
  gfx->print("PLUS ULTRA");
  gfx->setTextColor(DEKU_HERO_GREEN);
  gfx->setCursor(115, 35);
  gfx->print("PLUS ULTRA");
  
  // Time with shadow
  gfx->setTextSize(8);
  gfx->setTextColor(RGB565(20, 40, 30));
  gfx->setCursor(37, 142);
  gfx->print(timeStr);
  gfx->setTextColor(DEKU_FULL_COWL);
  gfx->setCursor(35, 140);
  gfx->print(timeStr);
  
  // Seconds
  gfx->setTextSize(3);
  gfx->setTextColor(DEKU_OFA_LIGHTNING);
  gfx->setCursor(310, 175);
  gfx->printf("%02d", time.second);
  
  // Lightning arc
  float secAngle = (time.second / 60.0) * 2 * PI - PI/2;
  for (float a = -PI/2; a < secAngle; a += 0.04) {
    int sx = centerX + cos(a) * 145;
    int sy = centerY - 20 + sin(a) * 55;
    gfx->fillCircle(sx, sy, 3, DEKU_OFA_LIGHTNING);
  }
  
  // Tagline with current OFA percentage
  gfx->setTextColor(DEKU_ALLMIGHT_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(100, 270);
  gfx->print("Full Cowl: 100%");
  
  // === DATE SECTION ===
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  int dateY = 302;
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(120, 150, 130));
  gfx->setCursor(centerX - 80, dateY);
  gfx->printf("%s  %02d.%02d", days[time.weekday % 7], time.day, time.month);
  
  // === STATS CARDS ===
  int cardY = 340;
  int cardH = 55;
  int cardW = 115;
  int cardGap = 12;
  int cardStartX = (LCD_WIDTH - (3 * cardW + 2 * cardGap)) / 2;
  
  // Steps card
  gfx->fillRect(cardStartX, cardY, cardW, cardH, RGB565(15, 22, 18));
  gfx->drawRect(cardStartX, cardY, cardW, cardH, DEKU_HERO_GREEN);
  gfx->fillRect(cardStartX, cardY, 5, 5, DEKU_HERO_GREEN);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(70, 120, 90));
  gfx->setCursor(cardStartX + 10, cardY + 8);
  gfx->print("STEPS");
  gfx->setTextSize(2);
  gfx->setTextColor(DEKU_HERO_GREEN);
  gfx->setCursor(cardStartX + 10, cardY + 28);
  gfx->printf("%d", system_state.steps_today);
  
  // OFA Power/Battery card
  int card2X = cardStartX + cardW + cardGap;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? DEKU_OFA_LIGHTNING : RGB565(255, 80, 80);
  gfx->fillRect(card2X, cardY, cardW, cardH, RGB565(18, 18, 25));
  gfx->drawRect(card2X, cardY, cardW, cardH, battColor);
  gfx->fillRect(card2X, cardY, 5, 5, battColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 100, 150));
  gfx->setCursor(card2X + 10, cardY + 8);
  gfx->print("OFA");
  gfx->setTextSize(2);
  gfx->setTextColor(battColor);
  gfx->setCursor(card2X + 10, cardY + 28);
  gfx->printf("%d%%", battPct);
  
  // Quirks card
  int card3X = card2X + cardW + cardGap;
  gfx->fillRect(card3X, cardY, cardW, cardH, RGB565(22, 20, 15));
  gfx->drawRect(card3X, cardY, cardW, cardH, DEKU_ALLMIGHT_GOLD);
  gfx->fillRect(card3X, cardY, 5, 5, DEKU_ALLMIGHT_GOLD);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 120, 70));
  gfx->setCursor(card3X + 10, cardY + 8);
  gfx->print("QUIRKS");
  gfx->setTextSize(2);
  gfx->setTextColor(DEKU_ALLMIGHT_GOLD);
  gfx->setCursor(card3X + 10, cardY + 28);
  gfx->print("9/9");
  
  drawDekuActivityRings(centerX, 435);
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
  // Full clear with dark tech background
  gfx->fillScreen(RGB565(5, 8, 15));
  
  // Use dynamic background
  drawDynamicBackground(THEME_BOBOIBOY);
  
  WatchTime time = getCurrentTime();
  int centerX = LCD_WIDTH / 2;  // 205
  
  // Element data
  uint16_t elementColors[] = {
    BBB_LIGHTNING_YELLOW,  // Halilintar - Yellow
    BBB_WIND_BLUE,         // Taufan - Cyan
    BBB_EARTH_BROWN,       // Gempa - Brown
    BBB_FIRE_RED,          // Blaze - Red
    BBB_WATER_CYAN,        // Ice - Light Blue
    BBB_LEAF_GREEN,        // Thorn - Green
    BBB_LIGHT_GOLD         // Solar - Gold
  };
  const char* elementNames[] = {
    "HALILINTAR", "TAUFAN", "GEMPA", "BLAZE", "ICE", "THORN", "SOLAR"
  };
  const char* elementShort[] = {
    "HAL", "TAU", "GEM", "BLZ", "ICE", "THN", "SOL"
  };
  
  int currentElement = getCurrentBoboiboyElement();
  uint16_t currentColor = elementColors[currentElement];
  uint16_t dimColor = RGB565((currentColor >> 11) / 2, ((currentColor >> 5) & 0x3F) / 2, (currentColor & 0x1F) / 2);
  
  // ============================================================================
  // TECH FRAME - Hexagonal outer border (not circular!)
  // ============================================================================
  
  int frameTop = 15;
  int frameInset = 40;
  gfx->drawLine(frameInset, frameTop, LCD_WIDTH - frameInset, frameTop, currentColor);
  gfx->drawLine(frameInset, frameTop, 10, frameTop + 30, currentColor);
  gfx->drawLine(LCD_WIDTH - frameInset, frameTop, LCD_WIDTH - 10, frameTop + 30, currentColor);
  
  gfx->drawLine(10, frameTop + 30, 10, 350, dimColor);
  gfx->drawLine(LCD_WIDTH - 10, frameTop + 30, LCD_WIDTH - 10, 350, dimColor);
  
  gfx->fillRect(5, frameTop + 25, 8, 8, currentColor);
  gfx->fillRect(LCD_WIDTH - 13, frameTop + 25, 8, 8, currentColor);
  
  // ============================================================================
  // SPLIT-FLAP CLOCK DISPLAY (Digital panels like departure boards)
  // ============================================================================
  
  int clockY = 50;
  int digitW = 75;
  int digitH = 100;
  int digitGap = 8;
  int colonW = 20;
  
  int totalW = (digitW * 4) + colonW + (digitGap * 3);
  int startX = (LCD_WIDTH - totalW) / 2;
  
  char digits[5];
  digits[0] = '0' + (time.hour / 10);
  digits[1] = '0' + (time.hour % 10);
  digits[2] = '0' + (time.minute / 10);
  digits[3] = '0' + (time.minute % 10);
  digits[4] = '\0';
  
  // Draw 4 digit panels
  for (int d = 0; d < 4; d++) {
    int x;
    if (d < 2) {
      x = startX + d * (digitW + digitGap);
    } else {
      x = startX + (digitW * 2) + (digitGap * 2) + colonW + digitGap + (d - 2) * (digitW + digitGap);
    }
    
    // Panel background
    gfx->fillRect(x, clockY, digitW, digitH, RGB565(15, 18, 25));
    
    // Split line (flip card effect)
    gfx->drawLine(x, clockY + digitH/2, x + digitW, clockY + digitH/2, RGB565(30, 35, 45));
    
    // Border
    gfx->drawRect(x, clockY, digitW, digitH, RGB565(40, 45, 55));
    
    // Corner accents in theme color
    gfx->fillRect(x, clockY, 4, 4, currentColor);
    gfx->fillRect(x + digitW - 4, clockY, 4, 4, currentColor);
    gfx->fillRect(x, clockY + digitH - 4, 4, 4, currentColor);
    gfx->fillRect(x + digitW - 4, clockY + digitH - 4, 4, 4, currentColor);
    
    // Digit
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(8);
    gfx->setCursor(x + (digitW - 48) / 2, clockY + (digitH - 64) / 2);
    gfx->print(digits[d]);
  }
  
  // Colon between hour and minute
  int colonX = startX + (digitW * 2) + (digitGap * 2);
  gfx->fillCircle(colonX + colonW/2, clockY + digitH/3, 6, currentColor);
  gfx->fillCircle(colonX + colonW/2, clockY + (digitH*2)/3, 6, currentColor);
  
  // Seconds display below colon
  gfx->setTextSize(2);
  gfx->setTextColor(dimColor);
  gfx->setCursor(colonX + 2, clockY + digitH + 5);
  gfx->printf(":%02d", time.second);
  
  // ============================================================================
  // ELEMENT NAME BANNER
  // ============================================================================
  
  int bannerY = clockY + digitH + 35;
  int bannerH = 50;
  
  // Banner background with gradient effect
  for (int i = 0; i < bannerH; i++) {
    uint8_t alpha = map(i, 0, bannerH, 40, 15);
    gfx->drawLine(20, bannerY + i, LCD_WIDTH - 20, bannerY + i, 
                  RGB565(alpha, alpha + 5, alpha + 10));
  }
  
  // Banner borders
  gfx->drawLine(20, bannerY, LCD_WIDTH - 20, bannerY, currentColor);
  gfx->drawLine(20, bannerY + bannerH, LCD_WIDTH - 20, bannerY + bannerH, currentColor);
  
  // Left accent bar
  gfx->fillRect(20, bannerY, 5, bannerH, currentColor);
  
  // "POWER:" label
  gfx->setTextColor(RGB565(120, 125, 135));
  gfx->setTextSize(2);
  gfx->setCursor(35, bannerY + 15);
  gfx->print("POWER:");
  
  // Element name in theme color
  gfx->setTextColor(currentColor);
  gfx->setTextSize(3);
  gfx->setCursor(130, bannerY + 10);
  gfx->print(elementNames[currentElement]);
  
  // ============================================================================
  // HORIZONTAL ELEMENT SELECTOR BAR (Not circular!)
  // ============================================================================
  
  int barY = bannerY + bannerH + 25;
  int barH = 45;
  int iconSize = 35;
  int iconGap = 8;
  int totalBarW = (iconSize * 7) + (iconGap * 6);
  int barStartX = (LCD_WIDTH - totalBarW) / 2;
  
  // Bar background
  gfx->fillRect(barStartX - 10, barY, totalBarW + 20, barH, RGB565(10, 12, 18));
  gfx->drawRect(barStartX - 10, barY, totalBarW + 20, barH, RGB565(40, 45, 55));
  
  // Draw 7 element icons horizontally
  for (int i = 0; i < 7; i++) {
    int iconX = barStartX + (i * (iconSize + iconGap));
    int iconY = barY + (barH - iconSize) / 2;
    
    if (i == currentElement) {
      // Active element - highlighted with border
      gfx->fillRect(iconX - 2, iconY - 2, iconSize + 4, iconSize + 4, currentColor);
      gfx->fillRect(iconX, iconY, iconSize, iconSize, RGB565(20, 25, 35));
      
      // Glow effect
      gfx->drawRect(iconX - 3, iconY - 3, iconSize + 6, iconSize + 6, currentColor);
      
      // Element short name
      gfx->setTextColor(currentColor);
      gfx->setTextSize(1);
      gfx->setCursor(iconX + 3, iconY + 13);
      gfx->print(elementShort[i]);
    } else {
      // Inactive element - muted
      gfx->fillRect(iconX, iconY, iconSize, iconSize, RGB565(25, 28, 35));
      gfx->drawRect(iconX, iconY, iconSize, iconSize, RGB565(50, 55, 65));
      
      // Small colored indicator at bottom
      gfx->fillRect(iconX + iconSize/2 - 4, iconY + iconSize - 8, 8, 5, elementColors[i]);
    }
  }
  
  // ============================================================================
  // STATS PANELS (3 at bottom)
  // ============================================================================
  
  int statsY = barY + barH + 20;
  int panelH = 70;
  int panelW = 115;
  int panelGap = 15;
  int panelStartX = (LCD_WIDTH - (panelW * 3 + panelGap * 2)) / 2;
  
  const char* days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  
  // Panel 1: Date
  gfx->fillRect(panelStartX, statsY, panelW, panelH, RGB565(12, 15, 22));
  gfx->drawRect(panelStartX, statsY, panelW, panelH, RGB565(35, 40, 50));
  gfx->fillRect(panelStartX, statsY, panelW, 3, BBB_WIND_BLUE);  // Top accent
  gfx->setTextColor(RGB565(100, 105, 115));
  gfx->setTextSize(1);
  gfx->setCursor(panelStartX + 8, statsY + 12);
  gfx->print("DATE");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(panelStartX + 8, statsY + 35);
  gfx->printf("%s %02d/%02d", days[time.weekday % 7], time.day, time.month);
  
  // Panel 2: Steps
  int panel2X = panelStartX + panelW + panelGap;
  gfx->fillRect(panel2X, statsY, panelW, panelH, RGB565(12, 15, 22));
  gfx->drawRect(panel2X, statsY, panelW, panelH, RGB565(35, 40, 50));
  gfx->fillRect(panel2X, statsY, panelW, 3, BBB_LEAF_GREEN);  // Top accent
  gfx->setTextColor(RGB565(100, 105, 115));
  gfx->setTextSize(1);
  gfx->setCursor(panel2X + 8, statsY + 12);
  gfx->print("STEPS");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(panel2X + 8, statsY + 35);
  gfx->printf("%d", system_state.steps_today);
  
  // Panel 3: Battery
  int panel3X = panelStartX + (panelW + panelGap) * 2;
  int battPct = getBatteryPercentage();
  system_state.battery_percentage = battPct;
  uint16_t battColor = battPct > 20 ? BBB_LIGHTNING_YELLOW : BBB_FIRE_RED;
  gfx->fillRect(panel3X, statsY, panelW, panelH, RGB565(12, 15, 22));
  gfx->drawRect(panel3X, statsY, panelW, panelH, RGB565(35, 40, 50));
  gfx->fillRect(panel3X, statsY, panelW, 3, battColor);  // Top accent
  gfx->setTextColor(RGB565(100, 105, 115));
  gfx->setTextSize(1);
  gfx->setCursor(panel3X + 8, statsY + 12);
  gfx->print("POWER");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(panel3X + 8, statsY + 35);
  gfx->printf("%d%%", battPct);
  
  // ============================================================================
  // BOTTOM HUD LINE
  // ============================================================================
  
  int hudY = statsY + panelH + 15;
  gfx->drawLine(30, hudY, LCD_WIDTH - 30, hudY, RGB565(40, 45, 55));
  
  // Tap hint
  gfx->setTextColor(RGB565(70, 75, 85));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 55, hudY + 8);
  gfx->print("[TAP TO SWITCH]");
  
  // ============================================================================
  // OCHOBOT (if visible)
  // ============================================================================
  
  if (ochobot_pos.visible) {
    drawOchobot(LCD_WIDTH - 45, 20, OCHOBOT_IDLE, 0.7);
  }
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

// Enhanced effect functions for improved watchfaces
void drawYugoPortalEffects() {
  // Left portal with multiple rings
  for (int r = 30; r <= 45; r += 5) {
    gfx->drawCircle(70, 100, r, YUGO_PORTAL_CYAN);
  }
  gfx->fillCircle(70, 100, 20, RGB565(20, 60, 70));
  gfx->fillCircle(70, 100, 12, YUGO_WAKFU_ENERGY);
  
  // Right portal
  for (int r = 30; r <= 45; r += 5) {
    gfx->drawCircle(LCD_WIDTH - 70, 100, r, YUGO_PORTAL_CYAN);
  }
  gfx->fillCircle(LCD_WIDTH - 70, 100, 20, RGB565(20, 60, 70));
  gfx->fillCircle(LCD_WIDTH - 70, 100, 12, YUGO_WAKFU_ENERGY);
}

void drawNarutoSageAura() {
  int cx = LCD_WIDTH / 2;
  int cy = LCD_HEIGHT / 2 - 60;
  for (int r = 70; r < 95; r += 5) {
    gfx->drawCircle(cx, cy, r, NARUTO_CHAKRA_ORANGE);
  }
}

void drawNarutoSageAuraEnhanced() {
  int cx = LCD_WIDTH / 2;
  int cy = 120;
  // Chakra aura with gradient effect
  for (int r = 50; r < 85; r += 4) {
    uint8_t alpha = map(r, 50, 85, 40, 15);
    gfx->drawCircle(cx, cy, r, RGB565(alpha * 3, alpha, 0));
  }
  // Sage eye marks
  gfx->fillRect(cx - 50, 75, 8, 20, NARUTO_SAGE_GOLD);
  gfx->fillRect(cx + 42, 75, 8, 20, NARUTO_SAGE_GOLD);
}

void drawGokuUIAura() {
  int cx = LCD_WIDTH / 2;
  int cy = 140;
  for (int r = 60; r < 85; r += 4) {
    gfx->drawCircle(cx, cy, r, GOKU_SILVER_GLOW);
  }
}

void drawGokuUIAuraEnhanced() {
  int cx = LCD_WIDTH / 2;
  int cy = 120;
  // Silver divine aura
  for (int r = 45; r < 80; r += 3) {
    uint8_t alpha = map(r, 45, 80, 50, 15);
    gfx->drawCircle(cx, cy, r, RGB565(alpha, alpha + 5, alpha + 10));
  }
  // Inner glow
  gfx->fillCircle(cx, cy, 35, RGB565(15, 18, 25));
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

void drawTanjiroSunFlamesEnhanced() {
  // Sun flames across the top
  for (int i = 0; i < 9; i++) {
    int x = 30 + i * 42;
    int y = 75;
    int h = 25 + (i % 2) * 10;
    // Outer flame (orange)
    gfx->fillTriangle(x, y, x - 10, y + h, x + 10, y + h, TANJIRO_FIRE_ORANGE);
    // Inner flame (bright)
    gfx->fillTriangle(x, y + 8, x - 5, y + h - 3, x + 5, y + h - 3, TANJIRO_FLAME_GLOW);
  }
}

void drawGojoInfinityAura() {
  int cx = LCD_WIDTH / 2;
  int cy = 140;
  for (int r = 50; r < 80; r += 5) {
    gfx->drawCircle(cx, cy, r, GOJO_INFINITY_BLUE);
  }
}

void drawGojoInfinityAuraEnhanced() {
  int cx = LCD_WIDTH / 2;
  int cy = 120;
  // Infinity void effect
  for (int r = 40; r < 75; r += 3) {
    uint8_t alpha = map(r, 40, 75, 60, 15);
    gfx->drawCircle(cx, cy, r, RGB565(alpha/3, alpha/2, alpha));
  }
  // Central void
  gfx->fillCircle(cx, cy, 30, RGB565(5, 8, 15));
}

void drawGojoSixEyesGlow() {
  gfx->fillCircle(LCD_WIDTH / 2 - 25, 60, 10, GOJO_SIX_EYES_BLUE);
  gfx->fillCircle(LCD_WIDTH / 2 + 25, 60, 10, GOJO_SIX_EYES_BLUE);
}

void drawGojoSixEyesGlowEnhanced() {
  // Six Eyes with glow effect
  int eyeY = 85;
  // Left eye
  gfx->fillCircle(LCD_WIDTH/2 - 30, eyeY, 14, RGB565(10, 20, 40));
  gfx->fillCircle(LCD_WIDTH/2 - 30, eyeY, 10, GOJO_SIX_EYES_BLUE);
  gfx->fillCircle(LCD_WIDTH/2 - 30, eyeY, 5, RGB565(200, 230, 255));
  // Right eye
  gfx->fillCircle(LCD_WIDTH/2 + 30, eyeY, 14, RGB565(10, 20, 40));
  gfx->fillCircle(LCD_WIDTH/2 + 30, eyeY, 10, GOJO_SIX_EYES_BLUE);
  gfx->fillCircle(LCD_WIDTH/2 + 30, eyeY, 5, RGB565(200, 230, 255));
}

void drawLeviBladeShine() {
  gfx->drawLine(35, 85, 60, 150, LEVI_SILVER_BLADE);
  gfx->drawLine(LCD_WIDTH - 35, 85, LCD_WIDTH - 60, 150, LEVI_SILVER_BLADE);
}

void drawLeviWingsEffect() {
  // Survey Corps Wings of Freedom
  int cx = LCD_WIDTH / 2;
  // Left wing (white)
  gfx->fillTriangle(cx - 15, 95, cx - 60, 70, cx - 45, 110, RGB565(200, 200, 210));
  gfx->fillTriangle(cx - 55, 75, cx - 85, 60, cx - 70, 100, RGB565(180, 180, 190));
  // Right wing (blue)
  gfx->fillTriangle(cx + 15, 95, cx + 60, 70, cx + 45, 110, RGB565(60, 100, 160));
  gfx->fillTriangle(cx + 55, 75, cx + 85, 60, cx + 70, 100, RGB565(50, 80, 140));
}

void drawLeviBladeShineEnhanced() {
  // Dual blades with shine effect
  // Left blade
  gfx->drawLine(30, 75, 55, 130, LEVI_SILVER_BLADE);
  gfx->drawLine(31, 75, 56, 130, RGB565(200, 200, 210));
  gfx->drawLine(32, 76, 55, 128, RGB565(255, 255, 255));
  // Right blade
  gfx->drawLine(LCD_WIDTH - 30, 75, LCD_WIDTH - 55, 130, LEVI_SILVER_BLADE);
  gfx->drawLine(LCD_WIDTH - 31, 75, LCD_WIDTH - 56, 130, RGB565(200, 200, 210));
  gfx->drawLine(LCD_WIDTH - 32, 76, LCD_WIDTH - 55, 128, RGB565(255, 255, 255));
}

void drawSaitamaImpactLines() {
  // Impact lines radiating from center
  int cx = LCD_WIDTH / 2;
  int cy = 160;
  for (int i = 0; i < 16; i++) {
    float angle = (i * 22.5) * PI / 180.0;
    int x1 = cx + cos(angle) * 60;
    int y1 = cy + sin(angle) * 30;
    int x2 = cx + cos(angle) * 180;
    int y2 = cy + sin(angle) * 90;
    gfx->drawLine(x1, y1, x2, y2, RGB565(40, 35, 30));
  }
}

void drawDekuOFALightning() {
  for (int i = 0; i < 7; i++) {
    int x1 = random(35, LCD_WIDTH - 35);
    int y1 = random(55, 110);
    gfx->drawLine(x1, y1, x1 + random(-25, 25), y1 + random(25, 50), DEKU_OFA_LIGHTNING);
  }
}

void drawDekuOFALightningEnhanced() {
  // Full Cowl lightning effect around the screen
  for (int i = 0; i < 10; i++) {
    int x1 = 30 + (i * 38);
    int y1 = 70 + (i % 3) * 15;
    // Main bolt
    gfx->drawLine(x1, y1, x1 + 15, y1 + 25, DEKU_OFA_LIGHTNING);
    gfx->drawLine(x1 + 15, y1 + 25, x1 + 5, y1 + 40, DEKU_OFA_LIGHTNING);
    // Glow
    gfx->drawLine(x1 + 1, y1, x1 + 16, y1 + 25, DEKU_FULL_COWL);
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
// CHARACTER STATS SCREEN - IMPROVED WITH PROMINENT TITLE
// =============================================================================

void forceCharacterStatsRedraw() {
  extern bool g_force_char_stats_redraw;
  g_force_char_stats_redraw = true;
}

void drawCharacterStatsScreen() {
  static int last_level = -1;
  static long last_xp = -1;
  static int last_equipped_title = -1;
  
  CharacterXPData* char_xp = getCurrentCharacterXP();
  
  // ==========================================================================
  // FIX: Check force flag FIRST - ensures redraw when navigating here
  // ==========================================================================
  extern bool g_force_char_stats_redraw;
  bool data_changed = g_force_char_stats_redraw ||
                      (system_state.player_level != last_level) || 
                      (char_xp && char_xp->xp != last_xp) ||
                      (char_xp && char_xp->equipped_title_index != last_equipped_title);
  
  // FIX: Reset force flag BEFORE skip check - this is CRITICAL
  if (g_force_char_stats_redraw) {
    Serial.println("[THEMES] Character stats: Force redraw flag was set");
    g_force_char_stats_redraw = false;
  }
  
  if (!data_changed) return;
  
  Serial.println("[THEMES] Drawing character stats screen");
  
  last_level = system_state.player_level;
  if (char_xp) {
    last_xp = char_xp->xp;
    last_equipped_title = char_xp->equipped_title_index;
  }
  
  CharacterProfile* profile = getCurrentCharacterProfile();
  ThemeColors* theme = getCurrentTheme();
  
  // ============================================================================
  // BACKGROUND
  // ============================================================================
  gfx->fillScreen(RGB565(5, 5, 12));
  
  // Subtle theme gradient at top
  for (int y = 0; y < 100; y++) {
    uint8_t r = (theme->primary >> 11) << 3;
    uint8_t g = ((theme->primary >> 5) & 0x3F) << 2;
    uint8_t b = (theme->primary & 0x1F) << 3;
    int fade = (100 - y) / 5;
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(r*fade/30, g*fade/30, b*fade/30));
  }
  
  // ============================================================================
  // HEADER - THEME NAME
  // ============================================================================
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(3);
  int nameLen = strlen(profile->name);
  gfx->setCursor(LCD_WIDTH/2 - nameLen * 9, 25);
  gfx->print(profile->name);
  
  // ============================================================================
  // EQUIPPED TITLE - BIG AND PROMINENT
  // ============================================================================
  int titleBoxY = 65;
  int titleBoxH = 60;
  
  // Title box with glow effect
  gfx->fillRect(30, titleBoxY, LCD_WIDTH - 60, titleBoxH, RGB565(15, 12, 5));
  gfx->drawRect(30, titleBoxY, LCD_WIDTH - 60, titleBoxH, RGB565(255, 200, 50));
  gfx->drawRect(31, titleBoxY + 1, LCD_WIDTH - 62, titleBoxH - 2, RGB565(180, 140, 30));
  
  // Corner decorations
  gfx->fillRect(30, titleBoxY, 12, 3, RGB565(255, 215, 0));
  gfx->fillRect(30, titleBoxY, 3, 12, RGB565(255, 215, 0));
  gfx->fillRect(LCD_WIDTH - 42, titleBoxY, 12, 3, RGB565(255, 215, 0));
  gfx->fillRect(LCD_WIDTH - 33, titleBoxY, 3, 12, RGB565(255, 215, 0));
  
  // "TITLE" label
  gfx->setTextColor(RGB565(180, 150, 80));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 18, titleBoxY + 8);
  gfx->print("~ TITLE ~");
  
  // THE EQUIPPED TITLE - BIG GOLD TEXT
  const char* equippedTitle = getEquippedTitle();
  gfx->setTextColor(RGB565(255, 215, 0));  // Gold
  gfx->setTextSize(2);
  int titleLen = strlen(equippedTitle);
  gfx->setCursor(LCD_WIDTH/2 - titleLen * 6, titleBoxY + 28);
  gfx->print(equippedTitle);
  
  // ============================================================================
  // LEVEL DISPLAY
  // ============================================================================
  int levelY = titleBoxY + titleBoxH + 15;
  
  gfx->fillRect(30, levelY, LCD_WIDTH - 60, 55, RGB565(10, 10, 18));
  gfx->drawRect(30, levelY, LCD_WIDTH - 60, 55, theme->primary);
  
  // Level number - left side (reflects current level including cheats)
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(4);
  gfx->setCursor(50, levelY + 8);
  gfx->printf("%d", system_state.player_level);
  
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setTextSize(1);
  gfx->setCursor(50, levelY + 42);
  gfx->print("LEVEL");
  
  // XP progress - right side
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(1);
  gfx->setCursor(140, levelY + 10);
  gfx->print("EXPERIENCE");
  
  // XP Bar
  int xpBarX = 140;
  int xpBarW = LCD_WIDTH - 190;
  gfx->fillRect(xpBarX, levelY + 24, xpBarW, 12, RGB565(20, 20, 30));
  
  int xp_percent = 0;
  if (char_xp && char_xp->xp_to_next_level > 0) {
    xp_percent = (char_xp->xp * 100) / char_xp->xp_to_next_level;
    int xp_fill = (xpBarW * xp_percent) / 100;
    gfx->fillRect(xpBarX, levelY + 24, xp_fill, 12, RGB565(138, 43, 226));
  }
  gfx->drawRect(xpBarX, levelY + 24, xpBarW, 12, RGB565(100, 60, 160));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(xpBarX + xpBarW + 8, levelY + 26);
  gfx->printf("%d%%", xp_percent);
  
  // ============================================================================
  // STAT BARS - HP, MP
  // ============================================================================
  int statsY = levelY + 70;
  int barH = 18;
  int barW = LCD_WIDTH - 130;
  
  // HP
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setTextSize(1);
  gfx->setCursor(40, statsY + 4);
  gfx->print("HP");
  
  gfx->fillRect(70, statsY, barW, barH, RGB565(25, 15, 15));
  gfx->fillRect(70, statsY, barW - 10, barH, RGB565(180, 50, 50));
  gfx->drawRect(70, statsY, barW, barH, RGB565(255, 80, 80));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(barW + 80, statsY + 4);
  gfx->print("100");
  
  // MP
  statsY += 28;
  gfx->setTextColor(RGB565(80, 150, 255));
  gfx->setCursor(40, statsY + 4);
  gfx->print("MP");
  
  gfx->fillRect(70, statsY, barW, barH, RGB565(15, 15, 25));
  gfx->fillRect(70, statsY, (barW * 75) / 100, barH, RGB565(50, 100, 200));
  gfx->drawRect(70, statsY, barW, barH, RGB565(80, 150, 255));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(barW + 80, statsY + 4);
  gfx->print("75");
  
  // ============================================================================
  // CHARACTER STATS - 2 COLUMN GRID (scales with level)
  // ============================================================================
  int gridY = statsY + 35;
  
  gfx->fillRect(30, gridY, LCD_WIDTH - 60, 105, RGB565(8, 8, 15));
  gfx->drawRect(30, gridY, LCD_WIDTH - 60, 105, theme->primary);
  
  // Stats header
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  gfx->setCursor(40, gridY + 8);
  gfx->print(">> ATTRIBUTES <<");
  
  // Left column
  int col1X = 45;
  int col2X = LCD_WIDTH/2 + 10;
  int rowY = gridY + 28;
  int rowH = 20;
  
  // Stats scale with current level (works with cheats too)
  int lvl = system_state.player_level;
  
  // STR
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col1X, rowY);
  gfx->print("STR:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 50 + lvl);
  
  // VIT
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col2X, rowY);
  gfx->print("VIT:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 40 + lvl);
  
  rowY += rowH;
  
  // AGI
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col1X, rowY);
  gfx->print("AGI:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 45 + lvl);
  
  // INT
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col2X, rowY);
  gfx->print("INT:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 35 + lvl);
  
  rowY += rowH;
  
  // PER
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col1X, rowY);
  gfx->print("PER:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 30 + lvl);
  
  // SEN
  gfx->setTextColor(theme->accent);
  gfx->setCursor(col2X, rowY);
  gfx->print("SEN:");
  gfx->setTextColor(COLOR_WHITE);
  gfx->printf(" %d", 25 + lvl);
  
  // ============================================================================
  // SIGNATURE MOVE
  // ============================================================================
  int moveY = gridY + 112;
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(1);
  gfx->setCursor(40, moveY);
  gfx->print("SIGNATURE: ");
  gfx->setTextColor(theme->accent);
  gfx->print(profile->signature_move);
  
  // ============================================================================
  // CHARACTER SPECIAL ABILITY BOX - Fills empty space
  // ============================================================================
  int abilityY = moveY + 25;
  int abilityH = 65;
  
  gfx->fillRect(30, abilityY, LCD_WIDTH - 60, abilityH, RGB565(12, 10, 18));
  gfx->drawRect(30, abilityY, LCD_WIDTH - 60, abilityH, theme->primary);
  gfx->fillRect(30, abilityY, 5, 5, theme->accent);
  gfx->fillRect(LCD_WIDTH - 35, abilityY, 5, 5, theme->accent);
  
  // Ability label
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  gfx->setCursor(40, abilityY + 8);
  gfx->print(">> SPECIAL ABILITY <<");
  
  // Ability description based on character
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(40, abilityY + 25);
  
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:
      gfx->print("Toon Force: Reality bends to joy");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(LUFFY_SUN_GOLD);
      gfx->print("Conqueror's Haki: King's ambition");
      break;
    case THEME_SUNG_JINWOO:
      gfx->print("Shadow Extraction: Arise the dead");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(JINWOO_ARISE_GLOW);
      gfx->print("Ruler's Authority: Telekinesis");
      break;
    case THEME_YUGO_WAKFU:
      gfx->print("Portal Creation: Dimension travel");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(YUGO_WAKFU_ENERGY);
      gfx->print("Wakfu Manipulation: Life energy");
      break;
    case THEME_NARUTO_SAGE:
      gfx->print("Sage Mode: Nature energy fusion");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(NARUTO_KURAMA_FLAME);
      gfx->print("Kurama Link: Nine-tails chakra");
      break;
    case THEME_GOKU_UI:
      gfx->print("Ultra Instinct: Body moves alone");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(GOKU_KI_BLAST_BLUE);
      gfx->print("Ki Mastery: Limitless energy");
      break;
    case THEME_TANJIRO_SUN:
      gfx->print("Sun Breathing: 13 forms mastered");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(TANJIRO_WATER_BLUE);
      gfx->print("Demon Slayer Mark: Power boost");
      break;
    case THEME_GOJO_INFINITY:
      gfx->print("Infinity: Untouchable barrier");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(GOJO_HOLLOW_PURPLE);
      gfx->print("Domain: Unlimited Void");
      break;
    case THEME_LEVI_STRONGEST:
      gfx->print("ODM Mastery: Unmatched mobility");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(LEVI_SILVER_BLADE);
      gfx->print("Ackerman Power: Superhuman");
      break;
    case THEME_SAITAMA_OPM:
      gfx->print("One Punch: Infinite strength");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(SAITAMA_CAPE_RED);
      gfx->print("Limiter Broken: No ceiling");
      break;
    case THEME_DEKU_PLUSULTRA:
      gfx->print("One For All: 9 quirks combined");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(DEKU_OFA_LIGHTNING);
      gfx->print("Full Cowl: Body reinforcement");
      break;
    case THEME_BOBOIBOY:
      gfx->print("Elemental Split: 7 forms active");
      gfx->setCursor(40, abilityY + 40);
      gfx->setTextColor(BBB_BAND_ORANGE);
      gfx->print("Fusion: Combine elements");
      break;
    default:
      gfx->print("Special powers unlocked!");
      break;
  }
  
  // ============================================================================
  // CATCHPHRASE BOX
  // ============================================================================
  int catchY = abilityY + abilityH + 10;
  gfx->fillRect(30, catchY, LCD_WIDTH - 60, 30, RGB565(15, 12, 8));
  gfx->drawRect(30, catchY, LCD_WIDTH - 60, 30, RGB565(100, 80, 40));
  
  gfx->setTextColor(RGB565(200, 180, 100));
  gfx->setTextSize(1);
  gfx->setCursor(40, catchY + 10);
  gfx->print("\"");
  gfx->print(profile->catchphrase);
  gfx->print("\"");
  
  // ============================================================================
  // NAVIGATION HINT
  // ============================================================================
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 70, LCD_HEIGHT - 30);
  gfx->print("SWIPE UP: PROGRESSION");
  
  drawSwipeIndicator();
}

// =============================================================================
// CHARACTER STATS TOUCH HANDLER
// =============================================================================
void handleCharacterStatsTouch(TouchGesture& gesture) {
  // Swipe DOWN = back to watchface  
  if (gesture.event == TOUCH_SWIPE_DOWN) {
    Serial.println("[CharStats] Swipe DOWN -> Watchface");
    system_state.current_screen = SCREEN_WATCHFACE;
    return;
  }
  
  // Swipe UP = go to app grid
  if (gesture.event == TOUCH_SWIPE_UP) {
    Serial.println("[CharStats] Swipe UP -> App Grid");
    system_state.current_screen = SCREEN_APP_GRID;
    forceAppGridRedraw();
    return;
  }
}
// =============================================================================
// CHARACTER PROGRESSION TREE SCREEN - Full Title List
// User can tap on any unlocked title to equip it
// =============================================================================

static int progression_scroll = 0;
static bool progression_needs_redraw = true;  // Global flag for force redraw

// Force progression screen redraw (called when switching to this screen)
void forceProgressionRedraw() {
  progression_needs_redraw = true;
}

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
// =============================================================================
// GOJO SATORU TITLES
// =============================================================================
Title GOJO_TITLE_LIST[] = {
  {"Six Eyes Prodigy", "Level 1 - Born with the power", 1, false},
  {"Limitless User", "Level 5 - Master of infinity", 5, false},
  {"Infinity's Wall", "Level 10 - Untouchable barrier", 10, false},
  {"Blue Energy", "Level 15 - Attraction technique", 15, false},
  {"Red Reversal", "Level 20 - Repulsion force", 20, false},
  {"Star Vessel Guard", "Level 25 - Protecting the stars", 25, false},
  {"High School Strongest", "Level 30 - Jujutsu High legend", 30, false},
  {"Toji's Nightmare", "Level 35 - Vengeance awakened", 35, false},
  {"Awakened Sorcerer", "Level 40 - True power unlocked", 40, false},
  {"Hollow Purple", "Level 45 - Imaginary technique", 45, false},
  {"Infinite Void Master", "Level 50 - Domain perfected", 50, false},
  {"Special Grade Anomaly", "Level 55 - Beyond classification", 55, false},
  {"Untouchable God", "Level 60 - Cannot be touched", 60, false},
  {"Shibuya's Ace", "Level 65 - Incident survivor", 65, false},
  {"Prison Realm Prisoner", "Level 70 - Sealed away", 70, false},
  {"The Returning Legend", "Level 75 - Back from the seal", 75, false},
  {"Shinjuku Challenger", "Level 80 - Final battle begins", 80, false},
  {"Sukuna's Rival", "Level 85 - King of Curses opponent", 85, false},
  {"Absolute Zenith", "Level 90 - Peak of sorcery", 90, false},
  {"Throughout Heaven and Earth...", "Level 93 - The famous quote", 93, false},
  {"The Honored One", "Level 96 - Self-proclaimed title", 96, false},
  {"The Strongest Sorcerer", "Level 100 - Undisputed master", 100, false}
};

// =============================================================================
// NARUTO UZUMAKI TITLES
// =============================================================================
Title NARUTO_TITLE_LIST[] = {
  {"Nine-Tails Jinchuriki", "Level 1 - Host of Kurama", 1, false},
  {"Prankster of the Leaf", "Level 5 - Village troublemaker", 5, false},
  {"Team 7 Genin", "Level 10 - Squad formation", 10, false},
  {"Worst Ninja", "Level 15 - Failed the academy", 15, false},
  {"Toad Sage Apprentice", "Level 20 - Learning from Jiraiya", 20, false},
  {"Rasenshuriken Innovator", "Level 25 - New jutsu created", 25, false},
  {"Sage of Mount Myoboku", "Level 30 - Sage Mode mastered", 30, false},
  {"Hero of the Leaf", "Level 35 - Pain defeated", 35, false},
  {"Kage-Level Contender", "Level 40 - Power recognized", 40, false},
  {"Kyubi Chakra Mode", "Level 45 - Nine-tails controlled", 45, false},
  {"Kurama Mode Link", "Level 50 - Full partnership", 50, false},
  {"Ninja Way", "Level 55 - Never give up", 55, false},
  {"Allied Force Leader", "Level 60 - War commander", 60, false},
  {"Six Paths Sage", "Level 65 - Hagoromo's gift", 65, false},
  {"Asura's Reincarnation", "Level 70 - Ancient soul", 70, false},
  {"Child of Prophecy", "Level 75 - Destined one", 75, false},
  {"Saviour of the World", "Level 80 - War hero", 80, false},
  {"Orange Hokage", "Level 85 - Dream achieved", 85, false},
  {"Baryon Mode", "Level 90 - Ultimate sacrifice", 90, false},
  {"Will Of Fire", "Level 93 - Konoha's spirit", 93, false},
  {"The 7th Hokage", "Level 96 - Village leader", 96, false},
  {"The Strongest Shinobi", "Level 100 - Legendary ninja", 100, false}
};

// =============================================================================
// SON GOKU TITLES
// =============================================================================
Title GOKU_TITLE_LIST[] = {
  {"Low-Class Warrior", "Level 1 - Saiyan beginnings", 1, false},
  {"Turtle School Student", "Level 5 - Roshi's training", 5, false},
  {"World Martial Artist", "Level 10 - Tournament fighter", 10, false},
  {"Kaio-ken Master", "Level 15 - King Kai technique", 15, false},
  {"Legendary Super Saiyan", "Level 20 - First transformation", 20, false},
  {"Ascended Saiyan", "Level 25 - Beyond limits", 25, false},
  {"Super Saiyan 2", "Level 30 - Lightning aura", 30, false},
  {"Super Saiyan 3", "Level 35 - Long hair power", 35, false},
  {"Fusion Component", "Level 40 - Gogeta/Vegito", 40, false},
  {"Super Saiyan God", "Level 45 - Divine ki awakened", 45, false},
  {"Super Saiyan Blue", "Level 50 - God ki mastered", 50, false},
  {"Blue Kaio-ken x20", "Level 55 - Ultimate combo", 55, false},
  {"Universe 7 Ace", "Level 60 - Tournament of Power", 60, false},
  {"Ultra Instinct Sign", "Level 65 - Dodging mastery", 65, false},
  {"Silver-Eyed Master", "Level 70 - UI appearance", 70, false},
  {"Mastered Ultra Instinct", "Level 75 - Complete form", 75, false},
  {"Giant Ki Avatar", "Level 80 - Massive power", 80, false},
  {"True Ultra Instinct", "Level 85 - Saiyan UI", 85, false},
  {"Beyond the Gods", "Level 90 - Surpassed deities", 90, false},
  {"Multiverse Apex", "Level 93 - Cross-universe power", 93, false},
  {"Kakarot", "Level 96 - True Saiyan name", 96, false},
  {"Mightiest in the Heavens", "Level 100 - Ultimate warrior", 100, false}
};

// =============================================================================
// SAITAMA TITLES
// =============================================================================
Title SAITAMA_TITLE_LIST[] = {
  {"Salaryman", "Level 1 - Before hero life", 1, false},
  {"100 Pushups Daily", "Level 5 - Training regimen", 5, false},
  {"Bald Aspirant", "Level 10 - Hair lost to training", 10, false},
  {"The Bald Cape", "Level 15 - Hero name given", 15, false},
  {"Class C Rookie", "Level 20 - Hero Association start", 20, false},
  {"Ghost of City Z", "Level 25 - Monster slayer", 25, false},
  {"Class B Leader", "Level 30 - Rising rank", 30, false},
  {"Normal Puncher", "Level 35 - Casual destruction", 35, false},
  {"Deep Sea King Slayer", "Level 40 - Rain battle victory", 40, false},
  {"Class A Hero", "Level 45 - Climbing ranks", 45, false},
  {"Serious Punch", "Level 50 - When he tries", 50, false},
  {"Serious Table Flip", "Level 55 - Garou battle", 55, false},
  {"Meteor Shatterer", "Level 60 - City Z saved", 60, false},
  {"Monster Nightmare", "Level 65 - Feared by all", 65, false},
  {"Serious Sneeze", "Level 70 - Jupiter destroyed", 70, false},
  {"Zero Punch Paradox", "Level 75 - Before contact KO", 75, false},
  {"Limiter Breaker", "Level 80 - No ceiling exists", 80, false},
  {"Absolute Strength", "Level 85 - Unmatched power", 85, false},
  {"The Bored God", "Level 90 - Seeking challenge", 90, false},
  {"S-Class Secret", "Level 93 - Hidden true power", 93, false},
  {"One Punch Man", "Level 96 - Defining title", 96, false},
  {"Strongest in Existence", "Level 100 - Absolute peak", 100, false}
};

// =============================================================================
// TANJIRO KAMADO TITLES
// =============================================================================
Title TANJIRO_TITLE_LIST[] = {
  {"Water Breathing Learner", "Level 1 - Urokodaki's student", 1, false},
  {"Final Selection Survivor", "Level 5 - Demon Slayer Corps", 5, false},
  {"Black Blade Wielder", "Level 10 - Rare sword color", 10, false},
  {"Drum House Victor", "Level 15 - Kyogai defeated", 15, false},
  {"Lower Moon Assassin", "Level 20 - Demon hunter rising", 20, false},
  {"Hinokami Kagura User", "Level 25 - Father's dance", 25, false},
  {"Sun Breath Successor", "Level 30 - Original style", 30, false},
  {"Constant Concentration", "Level 35 - Full focus breathing", 35, false},
  {"Upper Moon Executioner", "Level 40 - Elite demon slayer", 40, false},
  {"Slayer Mark Awakened", "Level 45 - Power unlocked", 45, false},
  {"Sun Halo Dragon", "Level 50 - Devastating technique", 50, false},
  {"Hashira Apprentice", "Level 55 - Pillar training", 55, false},
  {"See-Through World", "Level 60 - Perception mastered", 60, false},
  {"Selfless State Master", "Level 65 - No fighting spirit", 65, false},
  {"Hanafuda Inheritor", "Level 70 - Family legacy", 70, false},
  {"The 13th Form", "Level 75 - Complete sun breath", 75, false},
  {"Muzan's Nemesis", "Level 80 - Final battle", 80, false},
  {"Demon King (Shadow)", "Level 85 - Nearly turned", 85, false},
  {"Dawn Bringer", "Level 90 - Defeated Muzan", 90, false},
  {"Last Breath", "Level 93 - Final stand", 93, false},
  {"Sun God's Avatar", "Level 96 - Yoriichi's equal", 96, false},
  {"Demon Slayer", "Level 100 - Ultimate title", 100, false}
};

// =============================================================================
// LEVI ACKERMAN TITLES
// =============================================================================
Title LEVI_TITLE_LIST[] = {
  {"Underground Legend", "Level 1 - Survival expert", 1, false},
  {"Thieving Acrobat", "Level 5 - City thief days", 5, false},
  {"Scout Recruit", "Level 10 - Joined Survey Corps", 10, false},
  {"ODM Prodigy", "Level 15 - Gear mastery", 15, false},
  {"Blade Master", "Level 20 - Perfect cuts", 20, false},
  {"Ackerman Instinct", "Level 25 - Bloodline power", 25, false},
  {"Spinning Whirlwind", "Level 30 - Signature move", 30, false},
  {"Squad Leader", "Level 35 - Special Ops command", 35, false},
  {"Female Titan's Bane", "Level 40 - Forest battle", 40, false},
  {"Beast Titan Shredder", "Level 45 - Zeke encounter", 45, false},
  {"Wall Maria Hero", "Level 50 - Reclaim mission", 50, false},
  {"Zeke's Nightmare", "Level 55 - Hunted the beast", 55, false},
  {"Thunder Spear Expert", "Level 60 - New weapons", 60, false},
  {"Unkillable Captain", "Level 65 - Survived everything", 65, false},
  {"Last Ackerman", "Level 70 - Family legacy", 70, false},
  {"Crimson Blade", "Level 75 - Stained with titan blood", 75, false},
  {"Humanity's Vengeance", "Level 80 - For the fallen", 80, false},
  {"Fulfilled Promise", "Level 85 - Erwin's mission", 85, false},
  {"Fallen Wings Legacy", "Level 90 - Survey Corps honor", 90, false},
  {"Last Sentinel", "Level 93 - Final watch", 93, false},
  {"Humanity's Strongest", "Level 96 - Official title", 96, false},
  {"Greatest Soldier in History", "Level 100 - Legendary status", 100, false}
};

// =============================================================================
// DEKU (IZUKU MIDORIYA) TITLES
// =============================================================================
Title DEKU_TITLE_LIST[] = {
  {"Quirkless Kid", "Level 1 - Born without power", 1, false},
  {"All Might's Successor", "Level 5 - Chosen one", 5, false},
  {"One For All 5%", "Level 10 - Learning control", 10, false},
  {"Full Cowl Initiate", "Level 15 - Body coating", 15, false},
  {"Shoot Style User", "Level 20 - Legs over arms", 20, false},
  {"Stain's Equal", "Level 25 - Hero recognition", 25, false},
  {"One For All 20%", "Level 30 - Growing stronger", 30, false},
  {"Blackwhip Awakened", "Level 35 - First quirk unlocked", 35, false},
  {"Float Master", "Level 40 - Nana's quirk", 40, false},
  {"Danger Sense Instinct", "Level 45 - Threat detection", 45, false},
  {"Smokescreen Strategist", "Level 50 - Tactical advantage", 50, false},
  {"Fa Jin Kineticist", "Level 55 - Stored power", 55, false},
  {"One For All 45%", "Level 60 - Half power", 60, false},
  {"Dark Deku (Vigilante)", "Level 65 - Solo hero phase", 65, false},
  {"Gearshift Overdrive", "Level 70 - Speed manipulation", 70, false},
  {"Transmission Master", "Level 75 - All quirks synced", 75, false},
  {"Symbol of Peace", "Level 80 - Inheriting the will", 80, false},
  {"One For All 100%", "Level 85 - Full power", 85, false},
  {"Bane of All For One", "Level 90 - Villain's nemesis", 90, false},
  {"The World's Hero", "Level 93 - Global recognition", 93, false},
  {"The Symbol of Hope", "Level 96 - New era begins", 96, false},
  {"The Greatest Hero", "Level 100 - Surpassed All Might", 100, false}
};
// =============================================================================
// BOBOIBOY TITLES
// =============================================================================
Title BOBOIBOY_TITLE_LIST[] = {
  {"Elemental Novice", "Level 1 - Awakening potential", 1, false},
  {"Lightning Initiate", "Level 5 - Lightning base mastery", 5, false},
  {"Wind Initiate", "Level 10 - Wind agility unlocked", 10, false},
  {"Earth Initiate", "Level 15 - Grounded strength", 15, false},
  {"Fire Initiate", "Level 20 - Fire sparks fly", 20, false},
  {"Water Initiate", "Level 25 - Fluid adaptive power", 25, false},
  {"Thunderstorm Disciple", "Level 31 - Storm power refined", 31, false},
  {"Cyclone Disciple", "Level 36 - Chaotic wind force", 36, false},
  {"Quake Disciple", "Level 41 - Trembling earth force", 41, false},
  {"Blaze Disciple", "Level 46 - Hotter flames evolved", 46, false},
  {"Ice Disciple", "Level 51 - Cold precision mastery", 51, false},
  {"Solar Disciple", "Level 56 - Radiant core power", 56, false},
  {"FrostFire Adept", "Level 61 - Flame meets freeze", 61, false},
  {"Glacier Adept", "Level 66 - Earth crystallized with ice", 66, false},
  {"Supra Adept", "Level 71 - Solar + thunderstorm", 71, false},
  {"Sori Adept", "Level 76 - Solar + thorny roots", 76, false},
  {"Rumble Adept", "Level 81 - Earthquake + lightning", 81, false},
  {"Sopan Adept", "Level 86 - Solar + wind vortex", 86, false},
  {"Tempest Master", "Level 91 - Final storm form", 91, false},
  {"Master of Elements", "Level 96 - Supreme command", 96, false},
  {"Kuasa Tujuh", "Level 98 - Seven powers united", 98, false},
  {"Elemental Hero", "Level 100 - Ultimate BoBoiBoy", 100, false}
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
  // ANTI-FLICKER: Only redraw when data changes
  static int last_player_level = -1;
  static int last_progression_scroll = -1;
  static int last_equipped_title_idx = -1;
  
  CharacterXPData* char_xp = getCurrentCharacterXP();
  int current_equipped = char_xp ? char_xp->equipped_title_index : -1;
  
  // Check if anything changed or force redraw flag is set
  bool data_changed = (system_state.player_level != last_player_level) ||
                      (progression_scroll != last_progression_scroll) ||
                      (current_equipped != last_equipped_title_idx);
  
  if (!progression_needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  progression_needs_redraw = false;
  last_player_level = system_state.player_level;
  last_progression_scroll = progression_scroll;
  last_equipped_title_idx = current_equipped;
  
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
  
  
//Now you also need to update drawProgressionScreen() to use these new title lists based on the current theme. Find the part where it selects which title list to use and add:

///Here's the corrected switch statement for drawProgressionScreen():

Title* titles = GENERIC_TITLE_LIST;
int title_count = 22;

switch(system_state.current_theme) {
  case THEME_LUFFY_GEAR5:
    titles = LUFFY_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_SUNG_JINWOO:
    titles = JINWOO_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_YUGO_WAKFU:
    titles = YUGO_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_GOJO_INFINITY:
    titles = GOJO_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_NARUTO_SAGE:
    titles = NARUTO_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_GOKU_UI:
    titles = GOKU_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_SAITAMA_OPM:
    titles = SAITAMA_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_TANJIRO_SUN:
    titles = TANJIRO_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_LEVI_STRONGEST:
    titles = LEVI_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_DEKU_PLUSULTRA:
    titles = DEKU_TITLE_LIST;
    title_count = 22;
    break;
  case THEME_BOBOIBOY:
    titles = BOBOIBOY_TITLE_LIST;  // You'll need to create this too
    title_count = 22;
    break;
  default:
    titles = GENERIC_TITLE_LIST;
    title_count = 22;
    break;
}
  // Update unlock status based on level
  for (int i = 0; i < title_count; i++) {
    titles[i].unlocked = (system_state.player_level >= titles[i].level_req);
  }
  
  // For generic list, also check achievement-based titles
  if (system_state.current_theme != THEME_LUFFY_GEAR5 && 
      system_state.current_theme != THEME_YUGO_WAKFU &&
      system_state.current_theme != THEME_SUNG_JINWOO) {
    if (title_count >= 12) titles[11].unlocked = (system_state.gacha_cards_collected >= 10);
    if (title_count >= 13) titles[12].unlocked = (system_state.gacha_cards_collected >= 50);
    if (title_count >= 14) titles[13].unlocked = (system_state.bosses_defeated >= 5);
    if (title_count >= 15) titles[14].unlocked = (system_state.bosses_defeated >= 25);
    if (title_count >= 16) titles[15].unlocked = (system_state.steps_today >= 10000);
    if (title_count >= 17) titles[16].unlocked = (system_state.player_gems >= 10000);
    if (title_count >= 18) titles[17].unlocked = (system_state.training_streak >= 10);
    if (title_count >= 19) titles[18].unlocked = (system_state.daily_login_count >= 30);
  }
  
  // Count unlocked titles
  int unlockedCount = 0;
  for (int i = 0; i < title_count; i++) {
    if (titles[i].unlocked) unlockedCount++;
  }
  
  // Show unlocked count
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(20, 18);
  gfx->printf("%d/%d", unlockedCount, title_count);
  
  // Draw title cards (4 visible at a time)
  int startIdx = progression_scroll * 4;
  for (int i = 0; i < 4; i++) {
    int idx = startIdx + i;
    if (idx >= title_count) break;
    
    int cardY = 58 + i * 90;
    bool unlocked = titles[idx].unlocked;
    
    // Check if this title is currently equipped
    extern const char* getEquippedTitle();
    const char* equipped_title_name = getEquippedTitle();
    bool is_equipped = (strcmp(titles[idx].name, equipped_title_name) == 0);
    
    // Card background - gold border if equipped
    gfx->fillRect(20, cardY, LCD_WIDTH - 40, 80, 
                  unlocked ? RGB565(15, 25, 15) : RGB565(12, 14, 20));
    
    if (is_equipped) {
      // Gold border for equipped title
      gfx->drawRect(20, cardY, LCD_WIDTH - 40, 80, COLOR_GOLD);
      gfx->drawRect(21, cardY + 1, LCD_WIDTH - 42, 78, COLOR_GOLD);
    } else {
      gfx->drawRect(20, cardY, LCD_WIDTH - 40, 80, 
                    unlocked ? theme->primary : RGB565(40, 45, 60));
    }
    
    if (unlocked) {
      // Corner accents for unlocked
      uint16_t cornerColor = is_equipped ? COLOR_GOLD : theme->primary;
      gfx->fillRect(20, cardY, 5, 5, cornerColor);
      gfx->fillRect(LCD_WIDTH - 25, cardY, 5, 5, cornerColor);
      gfx->fillRect(20, cardY + 75, 5, 5, cornerColor);
      gfx->fillRect(LCD_WIDTH - 25, cardY + 75, 5, 5, cornerColor);
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
    if (is_equipped) {
      // EQUIPPED badge (gold)
      gfx->fillRect(LCD_WIDTH - 95, cardY + 52, 70, 18, RGB565(60, 50, 10));
      gfx->drawRect(LCD_WIDTH - 95, cardY + 52, 70, 18, COLOR_GOLD);
      gfx->setTextColor(COLOR_GOLD);
      gfx->setCursor(LCD_WIDTH - 93, cardY + 56);
      gfx->print("EQUIPPED");
    } else if (unlocked) {
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
  int maxScroll = (title_count - 1) / 4;
  
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
  int title_count;
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:
    case THEME_YUGO_WAKFU:
    case THEME_SUNG_JINWOO:
      title_count = 22;
      break;
    default:
      title_count = 19;
      break;
  }
  int maxScroll = (title_count - 1) / 4;
  
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
  } else if (gesture.event == TOUCH_TAP) {
    // FUSION OS: TAP to equip title
    int x = gesture.x;
    int y = gesture.y;
    
    // Title cards are at Y positions: 62, 162, 262, 362 (4 cards per page)
    // Each card is 80px tall
    int cardStartY = 62;
    int cardHeight = 80;
    int cardSpacing = 100;
    
    for (int i = 0; i < 4; i++) {
      int cardY = cardStartY + (i * cardSpacing);
      
      // Check if tap is within this card's Y bounds
      if (y >= cardY && y <= cardY + cardHeight) {
        int titleIdx = (progression_scroll * 4) + i;
        
        // Check if this title index is valid
        if (titleIdx >= title_count) break;
        
        // Get title list - FIXED: Handle ALL character themes
        Title* titles;
        switch(system_state.current_theme) {
          case THEME_LUFFY_GEAR5:
            titles = LUFFY_TITLE_LIST;
            break;
          case THEME_YUGO_WAKFU:
            titles = YUGO_TITLE_LIST;
            break;
          case THEME_SUNG_JINWOO:
            titles = JINWOO_TITLE_LIST;
            break;
          case THEME_GOJO_INFINITY:
            titles = GOJO_TITLE_LIST;
            break;
          case THEME_NARUTO_SAGE:
            titles = NARUTO_TITLE_LIST;
            break;
          case THEME_GOKU_UI:
            titles = GOKU_TITLE_LIST;
            break;
          case THEME_SAITAMA_OPM:
            titles = SAITAMA_TITLE_LIST;
            break;
          case THEME_TANJIRO_SUN:
            titles = TANJIRO_TITLE_LIST;
            break;
          case THEME_LEVI_STRONGEST:
            titles = LEVI_TITLE_LIST;
            break;
          case THEME_DEKU_PLUSULTRA:
            titles = DEKU_TITLE_LIST;
            break;
          case THEME_BOBOIBOY:
            titles = BOBOIBOY_TITLE_LIST;
            break;
          default:
            titles = GENERIC_TITLE_LIST;
            break;
        }
        
        // Check if title is unlocked
        bool unlocked = (system_state.player_level >= titles[titleIdx].level_req);
        
        if (unlocked) {
          // EQUIP THIS TITLE
          extern void equipTitle(int title_index);
          equipTitle(titleIdx);
          
          Serial.printf("[Progression] Equipped title #%d: %s\n", titleIdx, titles[titleIdx].name);
          
          // Visual feedback: redraw screen to show equipped badge
          drawProgressionScreen();
        } else {
          // Locked title - show message
          Serial.printf("[Progression] Locked title tapped: %s (Need Level %d)\n", 
                        titles[titleIdx].name, titles[titleIdx].level_req);
          // Could add a visual "LOCKED" flash here
        }
        break;
      }
    }
  }
}

// Theme selector page state (shared between draw and touch handler)
static int themePageState = 0;
static bool theme_selector_needs_redraw = true;  // Global flag for force redraw

// Force theme selector redraw (called when switching to this screen)
void forceThemeSelectorRedraw() {
  theme_selector_needs_redraw = true;
}

void drawThemeSelector() {
  // ANTI-FLICKER: Only redraw when page or theme changes
  static int last_theme_page = -1;
  static int last_current_theme = -1;
  
  // Check if anything changed or force redraw flag is set
  bool data_changed = (themePageState != last_theme_page) ||
                      (system_state.current_theme != last_current_theme);
  
  if (!theme_selector_needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  theme_selector_needs_redraw = false;
  last_theme_page = themePageState;
  last_current_theme = system_state.current_theme;
  
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

// =============================================================================
// DEDICATED THEME SAVE - Uses its own FusionData ("theme_cfg")
// Completely isolated from saveAllGameData() / "watchgame" namespace.
// Nothing else in the firmware reads or writes "theme_cfg".
// =============================================================================
void saveThemeToNVS(ThemeType theme) {
  extern void feedWatchdog();
  
  SDPreferences tp;
  
  // --- Write to dedicated namespace ---
  bool ok = tp.begin("theme_cfg", false);
  if (ok) {
    size_t written = tp.putInt("t", (int)theme);
    tp.end();
    Serial.printf("[THEME-SD] WRITE theme=%d to 'theme_cfg' (bytes=%u)\n", (int)theme, written);
  } else {
    Serial.println("[THEME-SD] ERROR: could not open 'theme_cfg' for write!");
  }
  
  feedWatchdog();
  
  // --- Read-back verification ---
  bool ok2 = tp.begin("theme_cfg", true);
  if (ok2) {
    int readback = tp.getInt("t", -999);
    tp.end();
    if (readback == (int)theme) {
      Serial.printf("[THEME-SD] VERIFY OK: read back %d == %d\n", readback, (int)theme);
    } else {
      Serial.printf("[THEME-SD] VERIFY FAIL: read back %d, expected %d\n", readback, (int)theme);
    }
  } else {
    Serial.println("[THEME-SD] ERROR: could not open 'theme_cfg' for verify read!");
  }
}

// =============================================================================
// LOAD THEME FROM NVS - Called from setup() before anything else.
// Returns the saved ThemeType, or THEME_LUFFY_GEAR5 if nothing saved yet.
// =============================================================================
ThemeType loadThemeFromNVS() {
  SDPreferences tp;
  ThemeType result = THEME_LUFFY_GEAR5;  // default
  
  bool ok = tp.begin("theme_cfg", true);  // read-only
  if (ok) {
    int val = tp.getInt("t", (int)THEME_LUFFY_GEAR5);
    tp.end();
    
    // Validate range
    if (val >= 0 && val < THEME_COUNT) {
      result = (ThemeType)val;
    }
    Serial.printf("[THEME-SD] LOAD theme=%d from 'theme_cfg'\n", (int)result);
  } else {
    // Namespace doesn't exist yet (first boot) — fall back to "watchgame"
    bool ok2 = tp.begin("watchgame", true);
    if (ok2) {
      int val = tp.getInt("theme", (int)THEME_LUFFY_GEAR5);
      tp.end();
      if (val >= 0 && val < THEME_COUNT) {
        result = (ThemeType)val;
      }
      Serial.printf("[THEME-SD] LOAD theme=%d from 'watchgame' (fallback)\n", (int)result);
    } else {
      Serial.println("[THEME-SD] No saved theme found — using default Luffy");
    }
  }
  
  return result;
}

// =============================================================================
// Save ONLY per-character data (gems, level, etc.) WITHOUT touching "theme" key.
// =============================================================================
static void savePerThemeDataOnly() {
  SDPreferences prefs;
  if (!prefs.begin("watchgame", false)) {
    Serial.println("[THEME] ERROR: Failed to open FusionData for per-theme save");
    return;
  }
  
  int t = (int)system_state.current_theme;
  char key[16];
  
  snprintf(key, sizeof(key), "t%d_gems", t);
  prefs.putInt(key, system_state.player_gems);
  
  snprintf(key, sizeof(key), "t%d_cards", t);
  prefs.putInt(key, system_state.gacha_cards_collected);
  
  snprintf(key, sizeof(key), "t%d_boss", t);
  prefs.putInt(key, system_state.bosses_defeated);
  
  snprintf(key, sizeof(key), "t%d_strk", t);
  prefs.putInt(key, system_state.training_streak);
  
  snprintf(key, sizeof(key), "t%d_pity", t);
  prefs.putInt(key, system_state.pity_counter);
  
  snprintf(key, sizeof(key), "t%d_pitl", t);
  prefs.putInt(key, system_state.pity_legendary_counter);
  
  snprintf(key, sizeof(key), "t%d_lvl", t);
  prefs.putInt(key, system_state.player_level);
  
  snprintf(key, sizeof(key), "t%d_xp", t);
  prefs.putInt(key, system_state.player_xp);
  
  prefs.end();
  
  Serial.printf("[THEME] Per-theme data saved for theme %d (theme key untouched)\n", t);
}

// =============================================================================
// Full theme switch + save + reboot (BULLETPROOF version)
//
// Strategy:
//   - Theme is saved to its OWN FusionData ("theme_cfg") that nothing
//     else in the firmware reads or writes.
//   - saveAllGameData() also writes the theme to "watchgame", but the boot
//     code reads "theme_cfg" FIRST, so even if "watchgame" has stale data,
//     the correct theme is loaded.
//   - feedWatchdog() called throughout to prevent WDT resets.
// =============================================================================
static void applyThemeAndReboot(ThemeType newTheme) {
  extern void saveAllGameData();
  extern void feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 1 — Save old character's per-theme data (no "theme" key).
  // ---------------------------------------------------------------
  savePerThemeDataOnly();
  feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 2 — Switch theme in RAM.
  // ---------------------------------------------------------------
  setTheme(newTheme);
  feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 3 — Persist new theme to dedicated FusionData.
  //          Includes write + read-back verification.
  //          After this line, the theme is crash-safe.
  // ---------------------------------------------------------------
  saveThemeToNVS(newTheme);
  feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 4 — Play the transition animation (purely visual).
  // ---------------------------------------------------------------
  playThemeTransition(newTheme);
  feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 5 — Load new character XP data.
  // ---------------------------------------------------------------
  loadXPDataForTheme(newTheme);
  
  CharacterXPData* char_xp = getCurrentCharacterXP();
  if (char_xp) {
    system_state.player_level = char_xp->level;
    system_state.player_xp = char_xp->xp;
  }
  feedWatchdog();
  
  // ---------------------------------------------------------------
  // STEP 6 — Full save (system_state has new theme now).
  // ---------------------------------------------------------------
  saveAllGameData();
  feedWatchdog();
  
  Serial.println("[THEME] Rebooting...");
  Serial.flush();
  delay(200);
  feedWatchdog();
  ESP.restart();
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
  
  // Theme selection - tap on theme cards
  if (themePageState == 0) {
    for (int i = 0; i < 6; i++) {
      int tx = (i % 2) * 170 + 15;
      int ty = (i / 2) * 105 + 52;
      if (x >= tx && x < tx + 155 && y >= ty && y < ty + 90) {
        applyThemeAndReboot((ThemeType)i);
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
        applyThemeAndReboot(types[i]);
        return;
      }
    }
  }
}



// =============================================================================
// TITLE SELECTION POPUP - FUSION OS (22 titles per character)
// =============================================================================

bool title_selection_active = false;  // Removed 'static' for extern access
static int title_scroll_offset = 0;   // Scroll position for titles
static const int TITLES_PER_PAGE = 7;  // Show 7 titles at once

void showTitleSelectionPopup() {
  title_selection_active = true;
  CharacterXPData* char_xp = getCurrentCharacterXP();
  if (!char_xp) return;
  
  // Count total unlocked titles
  int total_unlocked = 0;
  for (int i = 0; i < MAX_TITLES_PER_CHARACTER; i++) {
    if (char_xp->titles[i].unlocked) total_unlocked++;
  }
  
  // Clamp scroll offset
  int max_scroll = max(0, total_unlocked - TITLES_PER_PAGE);
  if (title_scroll_offset < 0) title_scroll_offset = 0;
  if (title_scroll_offset > max_scroll) title_scroll_offset = max_scroll;
  
  // Draw popup background
  gfx->fillRect(20, 60, 320, 400, RGB565(15, 17, 25));
  gfx->drawRect(20, 60, 320, 400, COLOR_WHITE);
  gfx->fillRect(20, 60, 5, 5, getCurrentTheme()->primary);
  gfx->fillRect(335, 60, 5, 5, getCurrentTheme()->primary);
  gfx->fillRect(20, 455, 5, 5, getCurrentTheme()->primary);
  gfx->fillRect(335, 455, 5, 5, getCurrentTheme()->primary);
  
  // Title header
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(80, 75);
  gfx->print("SELECT TITLE");
  
  // Subtitle with scroll hint
  gfx->setTextColor(RGB565(150, 150, 170));
  gfx->setTextSize(1);
  if (total_unlocked > 7) {
    gfx->setCursor(55, 95);
    gfx->print("Swipe up/down, tap to equip");
  } else {
    gfx->setCursor(75, 95);
    gfx->print("Tap a title to equip");
  }
  
  // List unlocked titles (with scrolling for 22 titles)
  int y = 120;
  int displayed_count = 0;
  int skip_count = 0;
  
  for (int i = 0; i < MAX_TITLES_PER_CHARACTER && displayed_count < TITLES_PER_PAGE; i++) {
    if (char_xp->titles[i].unlocked) {
      // Skip titles before scroll offset
      if (skip_count < title_scroll_offset) {
        skip_count++;
        continue;
      }
      
      bool is_equipped = (char_xp->equipped_title_index == i);
      
      // Title slot background
      uint16_t bg_color = is_equipped ? RGB565(80, 60, 150) : RGB565(30, 32, 40);
      gfx->fillRect(30, y, 300, 38, bg_color);
      gfx->drawRect(30, y, 300, 38, is_equipped ? COLOR_WHITE : RGB565(60, 60, 80));
      
      // Title name (WHITE text)
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(1);
      gfx->setCursor(40, y + 8);
      gfx->print(char_xp->titles[i].name);
      
      // Level requirement
      gfx->setTextColor(RGB565(150, 150, 170));
      gfx->setCursor(40, y + 22);
      gfx->printf("Lv.%d", char_xp->titles[i].level_required);
      
      // Equipped marker
      if (is_equipped) {
        gfx->fillCircle(310, y + 19, 8, RGB565(255, 215, 0));
        gfx->setTextColor(RGB565(80, 60, 150));
        gfx->setTextSize(1);
        gfx->setCursor(306, y + 14);
        gfx->print("E");
      }
      
      y += 42;
      displayed_count++;
    }
  }
  
  // Scroll indicators (only show if more than 7 titles)
  if (total_unlocked > 7) {
    if (title_scroll_offset > 0) {
      // Up arrow (more content above)
      gfx->setTextColor(getCurrentTheme()->primary);
      gfx->setTextSize(2);
      gfx->setCursor(180, 100);
      gfx->print("^");
    }
    
    if (title_scroll_offset < max_scroll) {
      // Down arrow (more content below)
      gfx->setTextColor(getCurrentTheme()->primary);
      gfx->setTextSize(2);
      gfx->setCursor(180, 390);
      gfx->print("v");
    }
    
    // Page indicator
    gfx->setTextColor(RGB565(100, 100, 120));
    gfx->setTextSize(1);
    gfx->setCursor(165, 400);
    gfx->printf("%d/%d", min(title_scroll_offset + 7, total_unlocked), total_unlocked);
  }
  
  // No titles message
  if (total_unlocked == 0) {
    gfx->setTextColor(RGB565(150, 150, 170));
    gfx->setTextSize(1);
    gfx->setCursor(70, 220);
    gfx->print("No titles unlocked yet!");
    gfx->setCursor(60, 240);
    gfx->print("Level up to unlock titles");
  }
  
  // Close button
  gfx->fillRect(110, 415, 140, 35, RGB565(100, 40, 40));
  gfx->drawRect(110, 415, 140, 35, COLOR_WHITE);
  gfx->fillRect(110, 415, 5, 5, RGB565(150, 50, 50));
  gfx->fillRect(245, 415, 5, 5, RGB565(150, 50, 50));
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(145, 423);
  gfx->print("CLOSE");
}

void equipTitleBySlot(int slot) {
  CharacterXPData* char_xp = getCurrentCharacterXP();
  if (!char_xp) return;
  
  // Find the nth unlocked title
  int unlocked_index = -1;
  int count = 0;
  
  for (int i = 0; i < MAX_TITLES_PER_CHARACTER; i++) {
    if (char_xp->titles[i].unlocked) {
      if (count == slot) {
        unlocked_index = i;
        break;
      }
      count++;
    }
  }
  
  if (unlocked_index >= 0) {
    char_xp->equipped_title_index = unlocked_index;
    
    // Save to NVS - using correct function from xp_system.h
    saveXPData();
    
    Serial.printf("[TITLES] Equipped: %s\n", char_xp->titles[unlocked_index].name);
  }
}

void handleTitleSelectionSwipe(int dy) {
  CharacterXPData* char_xp = getCurrentCharacterXP();
  if (!char_xp) return;
  
  // Count total unlocked titles
  int total_unlocked = 0;
  for (int i = 0; i < MAX_TITLES_PER_CHARACTER; i++) {
    if (char_xp->titles[i].unlocked) total_unlocked++;
  }
  
  int max_scroll = max(0, total_unlocked - TITLES_PER_PAGE);
  
  // Swipe up = scroll down (show lower titles)
  if (dy < 0 && title_scroll_offset < max_scroll) {
    title_scroll_offset++;
    showTitleSelectionPopup();
  }
  // Swipe down = scroll up (show higher titles)
  else if (dy > 0 && title_scroll_offset > 0) {
    title_scroll_offset--;
    showTitleSelectionPopup();
  }
}

void handleTitleSelectionTouch(int x, int y) {
  // Close button
  if (y >= 415 && y <= 450 && x >= 110 && x <= 250) {
    title_selection_active = false;
    title_scroll_offset = 0;  // Reset scroll
    drawCharacterStatsScreen();
    return;
  }
  
  // Title selection (7 visible slots, 42px each, starting at y=120)
  if (y >= 120 && y < 414) {
    int visible_slot = (y - 120) / 42;
    if (visible_slot >= 0 && visible_slot < TITLES_PER_PAGE) {
      // Convert visible slot to actual title index with scroll offset
      int actual_slot = visible_slot + title_scroll_offset;
      equipTitleBySlot(actual_slot);
      delay(100);
      showTitleSelectionPopup(); // Refresh to show new equipped title
    }
  }
}
