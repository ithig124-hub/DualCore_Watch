/*
 * themes.h - IMPROVED Theme System Header
 * Modern Anime Gaming Smartwatch - Enhanced Edition
 * 
 * All 10 Premium Anime Characters with modern visual effects
 */

#ifndef THEMES_H
#define THEMES_H

#include "config.h"

// Theme color schemes for all 11 characters (including BoBoiBoy)
extern ThemeColors luffy_gear5_theme;
extern ThemeColors sung_jinwoo_theme;
extern ThemeColors yugo_wakfu_theme;
extern ThemeColors naruto_sage_theme;
extern ThemeColors goku_ui_theme;
extern ThemeColors tanjiro_sun_theme;
extern ThemeColors gojo_infinity_theme;
extern ThemeColors levi_strongest_theme;
extern ThemeColors saitama_opm_theme;
extern ThemeColors deku_plusultra_theme;
extern ThemeColors boboiboy_elemental_theme;  // NEW: BoBoiBoy
extern ThemeColors* current_theme;

// Character profiles (11 characters)
extern CharacterProfile character_profiles[11];

// =============================================================================
// THEME MANAGEMENT
// =============================================================================

void initializeThemes();
void setTheme(ThemeType theme);
ThemeColors* getCurrentTheme();
ThemeColors* getThemeColors(ThemeType theme);
CharacterProfile* getCharacterProfile(ThemeType theme);
CharacterProfile* getCurrentCharacterProfile();
const char* getThemeName(ThemeType theme);

// NVS theme persistence (dedicated "theme_cfg" namespace)
void saveThemeToNVS(ThemeType theme);
ThemeType loadThemeFromNVS();


// Daily rotation
void updateDailyCharacter();
ThemeType getDailyRotationTheme(int day_of_week);
bool isMainCharacter(ThemeType theme);

// =============================================================================
// WATCH FACES - ALL 10 CHARACTERS (IMPROVED)
// =============================================================================

void drawWatchFace();
void updateWatchFaceTime();  // Partial update - no flicker
void drawLuffyWatchFace();
void drawJinwooWatchFace();
void drawYugoWatchFace();
void drawNarutoWatchFace();
void drawGokuWatchFace();
void drawTanjiroWatchFace();
void drawGojoWatchFace();
void drawLeviWatchFace();
void drawSaitamaWatchFace();
void drawDekuWatchFace();
void drawBoboiboyWatchFace();  // NEW: BoBoiBoy
void drawSleepWatchFace();

// Custom wallpaper
void drawCustomWallpaperWatchFace();
void drawWallpaperOverlayElements();
void drawMinimalTimeOverlay();

// =============================================================================
// CHARACTER EFFECTS - MODERN ANIME STYLE
// =============================================================================

// Luffy Gear 5 - Sun God effects
void drawLuffyGear5Effects();
void drawLuffySunRays();
void drawLuffyCloudWisps();
void drawLuffyRubberStretch();

// Jin-Woo Shadow Monarch - Dark power effects
void drawJinwooShadows();
void drawJinwooAriseEffect();
void drawJinwooVoidRifts();
void drawJinwooWeaponTrails();

// Yugo Portal Master - Portal energy
void drawYugoPortals();
void drawYugoWakfuParticles();
void drawYugoDimensionalRifts();
void drawYugoEnergyOrbs();

// Naruto Sage Mode - Chakra effects
void drawNarutoSageAura();
void drawNarutoKuramaFlames();
void drawNarutoRasengan();
void drawNarutoShadowClones();

// Goku Ultra Instinct - Divine silver aura
void drawGokuUIAura();
void drawGokuKiBlasts();
void drawGokuSpeedLines();
void drawGokuDivineGlow();

// Tanjiro Sun Breathing - Fire effects
void drawTanjiroSunFlames();
void drawTanjiroWaterTechniques();
void drawTanjiroHinokamiKagura();
void drawTanjiroNichirinBlade();

// Gojo Infinity - Cursed energy
void drawGojoInfinityAura();
void drawGojoSixEyesGlow();
void drawGojoHollowPurple();
void drawGojoDomainExpansion();

// Levi - ODM Gear effects
void drawLeviBladeShine();
void drawLeviODMGearMotion();
void drawLeviSpeedBlur();
void drawLeviCleanAesthetic();

// Saitama - Impact effects
void drawSaitamaPunchImpact();
void drawSaitamaCapeFlow();
void drawSaitamaDotEyes();
void drawSaitamaPowerEffect();

// Deku - OFA Lightning
void drawDekuOFALightning();
void drawDekuFullCowlAura();
void drawDekuAirForce();
void drawDekuPowerStrainMarks();

// =============================================================================
// ACTIVITY RINGS - CHARACTER THEMED
// =============================================================================

void drawActivityRing(int centerX, int centerY, int radius, float progress, uint16_t color, int thickness);
void drawLuffyActivityRings(int centerX, int centerY);
void drawJinwooActivityRings(int centerX, int centerY);
void drawYugoActivityRings(int centerX, int centerY);
void drawNarutoActivityRings(int centerX, int centerY);
void drawGokuActivityRings(int centerX, int centerY);
void drawTanjiroActivityRings(int centerX, int centerY);
void drawGojoActivityRings(int centerX, int centerY);
void drawLeviActivityRings(int centerX, int centerY);
void drawSaitamaActivityRings(int centerX, int centerY);
void drawDekuActivityRings(int centerX, int centerY);
void drawBoboiboyActivityRings(int centerX, int centerY);  // NEW

// =============================================================================
// ANIMATIONS
// =============================================================================

void playLuffyAnimation();
void playJinwooAnimation();
void playYugoAnimation();
void playNarutoAnimation();
void playGokuAnimation();
void playTanjiroAnimation();
void playGojoAnimation();
void playLeviAnimation();
void playSaitamaAnimation();
void playDekuAnimation();
void playBoboiboyAnimation();  // NEW: BoBoiBoy element cycling

// =============================================================================
// UI ELEMENTS - MODERN GLASS MORPHISM
// =============================================================================

void drawThemeButton(int x, int y, int w, int h, const char* text, bool pressed);
void drawGameButton(int x, int y, int w, int h, const char* text, bool pressed);
void drawThemeProgressBar(int x, int y, int w, int h, float progress, const char* label);
void drawThemeNotification(const char* title, const char* message);

// Glass Morphism Components
void drawGlassPanel(int x, int y, int w, int h, uint8_t opacity = 25);
void drawGlassButton(int x, int y, int w, int h, const char* text, bool pressed);
void drawGlassCard(int x, int y, int w, int h);
void drawGlassStatBar(int x, int y, int w, int h, float progress, uint16_t color, const char* label);

// Character effects
void drawCharacterRings(int centerX, int centerY);
void drawPulsingGlow(int x, int y, int radius, uint16_t color);

// Watch face utilities
void drawStepCounter();
void drawBatteryIndicator();
void forceWatchfaceRedraw();  // Force full watchface redraw (after theme change)
// Character-specific watch face elements
void drawLuffyStatsCards();
void drawLuffySunRays();

// =============================================================================
// CHARACTER STATS DISPLAY
// =============================================================================

void drawCharacterStatsScreen();
void drawStatBar(int x, int y, int w, int h, int value, int maxValue, uint16_t color, const char* label);
void drawAnimatedStatBar(int x, int y, int w, int h, float targetProgress, uint16_t color);

// Progression Screen
void drawProgressionScreen();
void handleProgressionTouch(TouchGesture& gesture);

// Theme Selector
void drawThemeSelector();
void handleThemeSelectorTouch(TouchGesture& gesture);
void forceThemeSelectorRedraw();  // Force full redraw on next drawThemeSelector()
void playThemeTransition(ThemeType theme);  // Character-specific transition animation

// =============================================================================
// TITLE EQUIPPING - FUSION OS
// =============================================================================

// Removed - using existing Progression screen instead (swipe up from Character Stats)
// void showTitleSelectionPopup();
// void equipTitleBySlot(int slot);
// void handleTitleSelectionTouch(int x, int y);
// void handleTitleSelectionSwipe(int dy);

// Title selection state (accessed from main loop)
// extern bool title_selection_active;

#endif // THEMES_H
