/*
 * companion.cpp - Virtual Pet System with NVS + Detailed Sprites
 * FUSION OS - 11 unique companions with evolution sprites
 * 
 * FIXED: 
 * - Watchdog-safe initialization (uses yield())
 * - Only loads/saves active companion from NVS (not all 11)
 * - Companion always matches current theme
 */

#include "companion.h"
#include "config.h"
#include "themes.h"
#include "xp_system.h"
#include "display.h"
#include "touch.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

CompanionSystemState companion_system;

CompanionProfile COMPANION_PROFILES[COMPANION_COUNT] = {
    {"Sunny", "One Piece", LUFFY_SUN_GOLD, LUFFY_NIKA_WHITE, "Meat", "Sailing", "Set sail!"},
    {"Igris", "Solo Leveling", RGB565(200, 50, 50), JINWOO_ARISE_GLOW, "Mana Crystals", "Training", "..."},
    {"Az", "Wakfu", YUGO_PORTAL_CYAN, YUGO_HAT_GOLD, "Dragon Treats", "Portal Chase", "Let's explore!"},
    {"Kurama", "Naruto", NARUTO_KURAMA_FLAME, NARUTO_CHAKRA_ORANGE, "Chakra Pills", "Rasengan", "Tch, brat..."},
    {"Puar", "Dragon Ball", GOKU_UI_SILVER, GOKU_AURA_WHITE, "Fish", "Shape Shift", "Yamcha!"},
    {"Nezuko", "Demon Slayer", TANJIRO_FIRE_ORANGE, TANJIRO_CHECK_GREEN, "Sleep", "Box Time", "Mmmph!"},
    {"Infinity", "Jujutsu Kaisen", GOJO_INFINITY_BLUE, GOJO_SNOW_WHITE, "Cursed Energy", "Barriers", "Honored one!"},
    {"Wings", "Attack on Titan", LEVI_SURVEY_GREEN, LEVI_SILVER_BLADE, "Tea", "Cleaning", "Keep it clean."},
    {"Genos Jr", "One Punch Man", SAITAMA_HERO_YELLOW, SAITAMA_CAPE_RED, "Parts", "Training", "Sensei!"},
    {"Might Jr", "My Hero Academia", DEKU_HERO_GREEN, DEKU_ALLMIGHT_GOLD, "Justice", "SMASH", "I AM HERE!"},
    {"Ochobot", "BoBoiBoy", BBB_BAND_ORANGE, BBB_OCHOBOT_WHITE, "Power Sphera", "Elements", "Let's go!"}
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void getCompKey(int i, const char* s, char* b, size_t n) {
    snprintf(b, n, "c%d_%s", i, s);
}

// Default stats for fresh companions
static void setDefaultStats(CompanionData* c, int index) {
    c->type = (CompanionType)index;
    c->profile = &COMPANION_PROFILES[index];
    c->stats = {80, 80, 80, 0, 1, MOOD_CONTENT, EVO_BABY, 0, 0};
    c->care = {0, 0, 0, millis(), 0, 0, 0, false, 0};
}

// Get theme index - uses system_state.current_theme from themes system
static int getThemeIndex() {
    // system_state.current_theme is ThemeType enum (0-10 for main characters)
    int idx = (int)system_state.current_theme;
    
    // Validate - only main character themes have companions
    if (idx < 0 || idx >= COMPANION_COUNT) {
        idx = 0;  // Default to Luffy/Sunny
    }
    return idx;
}

// ============================================================================
// NVS FUNCTIONS - OPTIMIZED: Only load/save specific companion
// ============================================================================

void initCompanionSystem() {
    // Reset system state
    companion_system.current_companion = nullptr;
    companion_system.current_companion_index = -1;
    companion_system.in_care_mode = false;
    companion_system.in_mini_game = false;
    companion_system.care_menu_selection = 0;
    companion_system.animation_frame = 0;
    companion_system.last_animation_time = 0;
    companion_system.nvs_initialized = false;
    companion_system.current_game = {"", 0, 0, false, 0, 0, 0};
    
    yield();  // Feed watchdog
    
    // Initialize all companions with DEFAULT values (fast, no NVS reads)
    for (int i = 0; i < COMPANION_COUNT; i++) {
        setDefaultStats(&companion_system.companions[i], i);
    }
    
    yield();  // Feed watchdog
    
    // Initialize NVS
    if (companion_system.prefs.begin(COMPANION_NVS_NAMESPACE, false)) {
        companion_system.nvs_initialized = true;
    }
    
    yield();  // Feed watchdog
    
    // Get companion index from current theme
    int themeIndex = getThemeIndex();
    
    // Set companion to match current theme
    companion_system.current_companion_index = themeIndex;
    companion_system.current_companion = &companion_system.companions[themeIndex];
    
    // Load ONLY this companion's saved data from NVS (if any)
    if (companion_system.nvs_initialized) {
        loadCompanionDataForIndex(themeIndex);
    }
    
    yield();  // Final watchdog feed
    
    Serial.printf("[Companion] Initialized: %s for theme %d\n", 
                  companion_system.current_companion->profile->name, themeIndex);
}

// Call this whenever theme changes - companion follows theme!
void syncCompanionWithTheme() {
    // Guard: Don't run if companion system hasn't been initialized yet
    if (!companion_system.nvs_initialized && companion_system.current_companion == nullptr) {
        return;  // Not ready yet - initCompanionSystem() hasn't been called
    }
    
    int themeIndex = getThemeIndex();
    
    // Only switch if different
    if (companion_system.current_companion_index != themeIndex) {
        // Save current companion's data before switching
        if (companion_system.current_companion_index >= 0 && companion_system.nvs_initialized) {
            saveCompanionDataForIndex(companion_system.current_companion_index);
        }
        
        yield();  // Feed watchdog
        
        // Switch to new companion (matching theme)
        companion_system.current_companion_index = themeIndex;
        companion_system.current_companion = &companion_system.companions[themeIndex];
        
        // Load new companion's data from NVS
        if (companion_system.nvs_initialized) {
            loadCompanionDataForIndex(themeIndex);
        }
        
        yield();  // Feed watchdog
        
        Serial.printf("[Companion] Switched to: %s\n", 
                      companion_system.current_companion->profile->name);
    }
}

// Load data for a SINGLE companion
void loadCompanionDataForIndex(int index) {
    if (index < 0 || index >= COMPANION_COUNT || !companion_system.nvs_initialized) return;
    
    char key[24];
    CompanionData* c = &companion_system.companions[index];
    
    // Check if this companion has saved data
    getCompKey(index, "init", key, sizeof(key));
    if (!companion_system.prefs.getBool(key, false)) {
        // No saved data, use defaults (already set)
        return;
    }
    
    yield();  // Feed watchdog
    
    // Load stats
    getCompKey(index, "hunger", key, sizeof(key));
    c->stats.hunger = companion_system.prefs.getInt(key, 80);
    
    getCompKey(index, "happy", key, sizeof(key));
    c->stats.happiness = companion_system.prefs.getInt(key, 80);
    
    getCompKey(index, "energy", key, sizeof(key));
    c->stats.energy = companion_system.prefs.getInt(key, 80);
    
    getCompKey(index, "bond", key, sizeof(key));
    c->stats.bond_level = companion_system.prefs.getInt(key, 0);
    
    getCompKey(index, "rank", key, sizeof(key));
    c->stats.bond_rank = companion_system.prefs.getInt(key, 1);
    
    getCompKey(index, "mood", key, sizeof(key));
    c->stats.mood = (CompanionMood)companion_system.prefs.getInt(key, MOOD_CONTENT);
    
    getCompKey(index, "evo", key, sizeof(key));
    c->stats.evolution = (CompanionEvolution)companion_system.prefs.getInt(key, EVO_BABY);
    
    getCompKey(index, "interact", key, sizeof(key));
    c->stats.total_interactions = companion_system.prefs.getInt(key, 0);
    
    getCompKey(index, "days", key, sizeof(key));
    c->stats.days_together = companion_system.prefs.getInt(key, 0);
    
    yield();  // Feed watchdog after load
}

// Save data for a SINGLE companion
void saveCompanionDataForIndex(int index) {
    if (index < 0 || index >= COMPANION_COUNT || !companion_system.nvs_initialized) return;
    
    char key[24];
    CompanionData* c = &companion_system.companions[index];
    
    // Mark this companion as having saved data
    getCompKey(index, "init", key, sizeof(key));
    companion_system.prefs.putBool(key, true);
    
    yield();  // Feed watchdog
    
    // Save stats
    getCompKey(index, "hunger", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.hunger);
    
    getCompKey(index, "happy", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.happiness);
    
    getCompKey(index, "energy", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.energy);
    
    getCompKey(index, "bond", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.bond_level);
    
    getCompKey(index, "rank", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.bond_rank);
    
    getCompKey(index, "mood", key, sizeof(key));
    companion_system.prefs.putInt(key, (int)c->stats.mood);
    
    getCompKey(index, "evo", key, sizeof(key));
    companion_system.prefs.putInt(key, (int)c->stats.evolution);
    
    getCompKey(index, "interact", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.total_interactions);
    
    getCompKey(index, "days", key, sizeof(key));
    companion_system.prefs.putInt(key, c->stats.days_together);
    
    yield();  // Feed watchdog after save
}

// Save ONLY current companion
void saveCompanionData() {
    if (!companion_system.nvs_initialized) return;
    if (companion_system.current_companion_index < 0) return;
    
    saveCompanionDataForIndex(companion_system.current_companion_index);
}

// Load ONLY current companion
void loadCompanionData() {
    if (!companion_system.nvs_initialized) return;
    if (companion_system.current_companion_index < 0) return;
    
    loadCompanionDataForIndex(companion_system.current_companion_index);
}

// Clear all companion data from NVS
void clearAllCompanionData() {
    if (!companion_system.nvs_initialized) return;
    
    companion_system.prefs.clear();
    
    // Reset all companions to defaults
    for (int i = 0; i < COMPANION_COUNT; i++) {
        setDefaultStats(&companion_system.companions[i], i);
        yield();  // Feed watchdog
    }
}

// ============================================================================
// COMPANION STATS UPDATE
// ============================================================================

void updateCompanionMood() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return;
    
    int avg = (c->stats.hunger + c->stats.happiness + c->stats.energy) / 3;
    
    if (avg >= 80) c->stats.mood = MOOD_ECSTATIC;
    else if (avg >= 60) c->stats.mood = MOOD_HAPPY;
    else if (avg >= 40) c->stats.mood = MOOD_CONTENT;
    else if (avg >= 20) c->stats.mood = MOOD_SAD;
    else c->stats.mood = MOOD_MISERABLE;
}

void updateCompanionEvolution() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return;
    
    int bond = c->stats.bond_level;
    int days = c->stats.days_together;
    
    if (bond >= 80 && days >= 30) c->stats.evolution = EVO_AWAKENED;
    else if (bond >= 50 && days >= 14) c->stats.evolution = EVO_ADULT;
    else if (bond >= 20 && days >= 7) c->stats.evolution = EVO_CHILD;
    else c->stats.evolution = EVO_BABY;
}

void updateCompanionStats() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return;
    
    unsigned long elapsed = millis() - c->care.last_stat_update;
    if (elapsed < 60000) return;  // Only update every minute
    
    float hrs = elapsed / 60000.0f;
    
    if (!c->care.is_sleeping) {
        c->stats.hunger = max(STAT_MIN, c->stats.hunger - (int)(HUNGER_DECAY_RATE * hrs));
        c->stats.happiness = max(STAT_MIN, c->stats.happiness - (int)(HAPPINESS_DECAY_RATE * hrs));
        c->stats.energy = max(STAT_MIN, c->stats.energy - (int)(ENERGY_DECAY_RATE * hrs));
    } else {
        c->stats.energy = min(STAT_MAX, c->stats.energy + (int)(5 * hrs));
    }
    
    c->care.last_stat_update = millis();
    
    updateCompanionMood();
    updateCompanionEvolution();
    
    // Save only the current companion
    saveCompanionDataForIndex(companion_system.current_companion_index);
}

void checkCompanionDailyReset() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return;
    
    // Simple daily reset check (could be improved with RTC)
    static unsigned long lastResetCheck = 0;
    if (millis() - lastResetCheck > 86400000) {  // 24 hours
        c->care.daily_feed_count = 0;
        c->care.daily_play_count = 0;
        c->care.daily_train_count = 0;
        c->stats.days_together++;
        lastResetCheck = millis();
        
        saveCompanionDataForIndex(companion_system.current_companion_index);
    }
}

// ============================================================================
// COMPANION CARE ACTIONS
// ============================================================================

bool feedCompanion() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return false;
    
    c->stats.hunger = min(STAT_MAX, c->stats.hunger + FEED_HUNGER_GAIN);
    c->stats.happiness = min(STAT_MAX, c->stats.happiness + FEED_HAPPINESS_GAIN);
    c->care.last_feed_time = millis();
    c->care.daily_feed_count++;
    c->stats.total_interactions++;
    
    updateCompanionMood();
    saveCompanionDataForIndex(companion_system.current_companion_index);
    
    return true;
}

bool playWithCompanion() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return false;
    if (c->stats.energy < PLAY_ENERGY_COST) return false;
    
    c->stats.energy -= PLAY_ENERGY_COST;
    c->stats.happiness = min(STAT_MAX, c->stats.happiness + PLAY_HAPPINESS_GAIN);
    c->stats.bond_level = min(BOND_MAX, c->stats.bond_level + PLAY_BOND_GAIN);
    c->care.last_play_time = millis();
    c->care.daily_play_count++;
    c->stats.total_interactions++;
    
    // Update bond rank
    if (c->stats.bond_level >= BOND_RANK_5) c->stats.bond_rank = 5;
    else if (c->stats.bond_level >= BOND_RANK_4) c->stats.bond_rank = 4;
    else if (c->stats.bond_level >= BOND_RANK_3) c->stats.bond_rank = 3;
    else if (c->stats.bond_level >= BOND_RANK_2) c->stats.bond_rank = 2;
    else if (c->stats.bond_level >= BOND_RANK_1) c->stats.bond_rank = 1;
    
    updateCompanionMood();
    updateCompanionEvolution();
    saveCompanionDataForIndex(companion_system.current_companion_index);
    
    return true;
}

bool trainCompanion() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return false;
    if (c->stats.energy < TRAIN_ENERGY_COST) return false;
    
    c->stats.energy -= TRAIN_ENERGY_COST;
    c->stats.bond_level = min(BOND_MAX, c->stats.bond_level + TRAIN_BOND_GAIN);
    c->care.last_train_time = millis();
    c->care.daily_train_count++;
    c->stats.total_interactions++;
    
    updateCompanionMood();
    updateCompanionEvolution();
    saveCompanionDataForIndex(companion_system.current_companion_index);
    
    return true;
}

void toggleCompanionSleep() {
    CompanionData* c = companion_system.current_companion;
    if (!c) return;
    
    c->care.is_sleeping = !c->care.is_sleeping;
    
    if (c->care.is_sleeping) {
        c->care.sleep_start_time = millis();
    } else {
        // Calculate energy gained during sleep
        unsigned long sleepTime = millis() - c->care.sleep_start_time;
        int energyGain = (sleepTime / 1000) * REST_ENERGY_GAIN / 30;
        c->stats.energy = min(STAT_MAX, c->stats.energy + energyGain);
    }
    
    saveCompanionDataForIndex(companion_system.current_companion_index);
}

void wakeCompanion() {
    CompanionData* c = companion_system.current_companion;
    if (!c || !c->care.is_sleeping) return;
    
    toggleCompanionSleep();
}

// ============================================================================
// MINI GAME
// ============================================================================

void startCompanionGame() {
    companion_system.in_mini_game = true;
    companion_system.current_game.active = true;
    companion_system.current_game.score = 0;
    companion_system.current_game.start_time = millis();
    companion_system.current_game.target_x = random(50, LCD_WIDTH - 50);
    companion_system.current_game.target_y = random(100, LCD_HEIGHT - 100);
}

void updateCompanionGame() {
    if (!companion_system.current_game.active) return;
    
    // 30 second game duration
    if (millis() - companion_system.current_game.start_time >= 30000) {
        endCompanionGame();
        return;
    }
    
    // Move target every 2 seconds
    if ((millis() - companion_system.current_game.start_time) % 2000 < 100) {
        companion_system.current_game.target_x = random(50, LCD_WIDTH - 50);
        companion_system.current_game.target_y = random(100, LCD_HEIGHT - 100);
    }
}

void endCompanionGame() {
    companion_system.current_game.active = false;
    companion_system.in_mini_game = false;
    
    // Update high score
    if (companion_system.current_game.score > companion_system.current_game.high_score) {
        companion_system.current_game.high_score = companion_system.current_game.score;
    }
    
    // Reward based on score
    if (companion_system.current_companion && companion_system.current_game.score > 0) {
        companion_system.current_companion->stats.happiness = 
            min(STAT_MAX, companion_system.current_companion->stats.happiness + companion_system.current_game.score / 2);
        companion_system.current_companion->stats.bond_level = 
            min(BOND_MAX, companion_system.current_companion->stats.bond_level + companion_system.current_game.score / 5);
        
        saveCompanionDataForIndex(companion_system.current_companion_index);
    }
}

bool handleGameTouch(int tx, int ty) {
    if (!companion_system.current_game.active) return false;
    
    int dx = tx - companion_system.current_game.target_x;
    int dy = ty - companion_system.current_game.target_y;
    int dist = sqrt(dx * dx + dy * dy);
    
    if (dist < 40) {  // Hit!
        companion_system.current_game.score += 10;
        companion_system.current_game.target_x = random(50, LCD_WIDTH - 50);
        companion_system.current_game.target_y = random(100, LCD_HEIGHT - 100);
        return true;
    }
    
    return false;
}

// ============================================================================
// ANIMATION
// ============================================================================

void updateCompanionAnimation() {
    if (millis() - companion_system.last_animation_time >= 500) {
        companion_system.animation_frame = (companion_system.animation_frame + 1) % 4;
        companion_system.last_animation_time = millis();
    }
}

// ============================================================================
// SPRITE DRAWING FUNCTIONS
// ============================================================================

void drawSunnySprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    // Sun-petal mane (7 orange petals radiating from head)
    uint16_t maneOrange = RGB565(255, 160, 30);
    uint16_t maneDark = RGB565(220, 120, 20);
    int maneR = size/2 + size/6;
    for (int p = 0; p < 7; p++) {
        float angle = (p * 51 - 153) * PI / 180.0;
        int px = x + cos(angle) * maneR;
        int py = y - size/4 + sin(angle) * maneR;
        int bx1 = x + cos(angle - 0.3) * (size/3);
        int by1 = y - size/4 + sin(angle - 0.3) * (size/3);
        int bx2 = x + cos(angle + 0.3) * (size/3);
        int by2 = y - size/4 + sin(angle + 0.3) * (size/3);
        gfx->fillTriangle(bx1, by1, bx2, by2, px, py, maneOrange);
        gfx->drawLine((bx1+bx2)/2, (by1+by2)/2, px, py, maneDark);
    }
    
    // Body
    uint16_t skinTone = RGB565(255, 220, 180);
    gfx->fillCircle(x, y, size/2, skinTone);
    
    // Eyes
    int eyeY = y - size/8;
    int eyeSpacing = size/4;
    gfx->fillCircle(x - eyeSpacing, eyeY, size/8, TFT_BLACK);
    gfx->fillCircle(x + eyeSpacing, eyeY, size/8, TFT_BLACK);
    gfx->fillCircle(x - eyeSpacing + 2, eyeY - 2, size/16, TFT_WHITE);
    gfx->fillCircle(x + eyeSpacing + 2, eyeY - 2, size/16, TFT_WHITE);
    
    // Smile
    int smileY = y + size/6;
    // Smile line
    for(int i=-size/5; i<size/5; i++) { gfx->drawPixel(x+i, smileY + abs(i)/3, TFT_BLACK); }
    
    // Scar under left eye
    gfx->drawLine(x - eyeSpacing - size/10, eyeY + size/6, x - eyeSpacing + size/10, eyeY + size/5, RGB565(180, 100, 100));
    
    // Straw hat
    uint16_t hatYellow = RGB565(255, 220, 80);
    uint16_t hatRed = RGB565(200, 50, 50);
    int hatY = y - size/2 - size/6;
    gfx->fillCircle(x, hatY, size/2, hatYellow);  // Hat brim
    gfx->fillCircle(x, hatY - size/10, size/3, hatYellow);  // Hat top
    gfx->fillRect(x - size/3, hatY - size/12, size*2/3, size/10, hatRed);
    
    // Animation: bouncing
    if (frame == 1 || frame == 3) {
        gfx->drawLine(x - size/2, y + size/2 + 5, x - size/4, y + size/2 + 10, skinTone);
        gfx->drawLine(x + size/2, y + size/2 + 5, x + size/4, y + size/2 + 10, skinTone);
    }
    
    // Evolution effects
    if (evo >= EVO_ADULT) {
        for (int i = 0; i < 8; i++) {
            float angle = (i * 45 + frame * 10) * PI / 180.0;
            int gx = x + cos(angle) * (size + 10);
            int gy = y + sin(angle) * (size + 10);
            gfx->fillCircle(gx, gy, 3, LUFFY_NIKA_WHITE);
        }
    }
    if (evo >= EVO_AWAKENED) {
        gfx->drawCircle(x, y, size + 15, LUFFY_SUN_GOLD);
        gfx->drawCircle(x, y, size + 17, LUFFY_SUN_GOLD);
    }
}

void drawShadowSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t armorDark = RGB565(40, 40, 50);
    uint16_t armorHighlight = RGB565(80, 80, 100);
    uint16_t redGlow = RGB565(200, 50, 50);
    
    gfx->fillCircle(x, y, size/2, armorDark);
    // Arc simplified for CO5300 compatibility
    // Arc simplified for CO5300 compatibility
    
    int helmetY = y - size/3;
    gfx->fillCircle(x, helmetY, size/2, armorDark);
    gfx->drawLine(x - size/3, helmetY, x + size/3, helmetY, armorHighlight);
    
    int eyeY = y - size/6;
    int eyeSize = size/10 + (frame % 2);
    gfx->fillCircle(x - size/5, eyeY, eyeSize, redGlow);
    gfx->fillCircle(x + size/5, eyeY, eyeSize, redGlow);
    
    gfx->fillTriangle(x - size/3, y + size/4, x + size/3, y + size/4, x, y + size, armorDark);
    
    if (evo >= EVO_ADULT) {
        for (int i = 0; i < 5; i++) {
            float angle = (i * 72 + frame * 15) * PI / 180.0;
            int wx = x + cos(angle) * (size/2 + 5);
            int wy = y + sin(angle) * (size/2 + 5);
            gfx->drawLine(x, y, wx, wy, RGB565(60, 60, 80));
        }
    }
    if (evo >= EVO_AWAKENED) {
        gfx->drawCircle(x, y, size + 10, JINWOO_ARISE_GLOW);
    }
}

void drawPortalSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinTone = RGB565(255, 220, 180);
    uint16_t hatBlue = RGB565(80, 140, 200);
    uint16_t portalCyan = YUGO_PORTAL_CYAN;
    
    gfx->fillCircle(x, y, size/2, skinTone);
    
    int hatY = y - size/2;
    gfx->fillCircle(x, hatY, size/2 + size/6, hatBlue);
    gfx->fillCircle(x, hatY - size/4, size/3, hatBlue);
    
    int eyeY = y - size/10;
    gfx->fillCircle(x - size/4, eyeY, size/6, TFT_WHITE);
    gfx->fillCircle(x + size/4, eyeY, size/6, TFT_WHITE);
    gfx->fillCircle(x - size/4, eyeY, size/10, RGB565(100, 80, 60));
    gfx->fillCircle(x + size/4, eyeY, size/10, RGB565(100, 80, 60));
    
    // Arc simplified for CO5300 compatibility
    
    int portalR = size/2 + 10 + (frame * 3);
    gfx->drawCircle(x, y, portalR, portalCyan);
    gfx->drawCircle(x, y, portalR + 2, portalCyan);
    
    if (evo >= EVO_ADULT) {
        gfx->drawCircle(x, y, size + 20, portalCyan);
        gfx->drawCircle(x, y, size + 25, YUGO_HAT_GOLD);
    }
    if (evo >= EVO_AWAKENED) {
        for (int i = 0; i < 6; i++) {
            float angle = (i * 60 + frame * 20) * PI / 180.0;
            int dx = x + cos(angle) * (size + 15);
            int dy = y + sin(angle) * (size + 15);
            gfx->fillCircle(dx, dy, 4, YUGO_HAT_GOLD);
        }
    }
}

void drawFoxSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t foxOrange = NARUTO_KURAMA_FLAME;
    uint16_t foxDark = RGB565(180, 80, 30);
    
    gfx->fillCircle(x, y, size/2, foxOrange);
    
    gfx->fillTriangle(x - size/2, y - size/4, x - size/4, y - size/2 - size/4, x - size/6, y - size/4, foxOrange);
    gfx->fillTriangle(x + size/2, y - size/4, x + size/4, y - size/2 - size/4, x + size/6, y - size/4, foxOrange);
    gfx->fillTriangle(x - size/3, y - size/4, x - size/4, y - size/2 - size/8, x - size/5, y - size/4, foxDark);
    gfx->fillTriangle(x + size/3, y - size/4, x + size/4, y - size/2 - size/8, x + size/5, y - size/4, foxDark);
    
    int eyeY = y - size/8;
    gfx->fillCircle(x - size/4, eyeY, size/8, TFT_WHITE);
    gfx->fillCircle(x + size/4, eyeY, size/8, TFT_WHITE);
    gfx->fillRect(x - size/4 - 1, eyeY - size/10, 2, size/5, TFT_BLACK);
    gfx->fillRect(x + size/4 - 1, eyeY - size/10, 2, size/5, TFT_BLACK);
    
    gfx->fillCircle(x, y + size/6, size/5, RGB565(255, 200, 180));
    gfx->fillCircle(x, y + size/8, size/12, TFT_BLACK);
    
    for (int i = 0; i < 3; i++) {
        int wy = y + (i - 1) * 5;
        gfx->drawLine(x - size/2 - 5, wy, x - size/4, wy, TFT_BLACK);
        gfx->drawLine(x + size/2 + 5, wy, x + size/4, wy, TFT_BLACK);
    }
    
    int tailCount = min(3 + (int)evo * 2, 9);
    for (int t = 0; t < tailCount; t++) {
        float angle = ((t - tailCount/2) * 20 + frame * 5) * PI / 180.0;
        int tx2 = x + sin(angle) * size/4;
        int ty2 = y + size/2;
        int tex = tx2 + sin(angle + 0.5) * size/2;
        int tey = ty2 + size/2 + t * 3;
        gfx->drawLine(tx2, ty2, tex, tey, foxOrange);
        gfx->drawLine(tx2+1, ty2, tex+1, tey, foxOrange);
    }
    
    if (evo >= EVO_AWAKENED) {
        for (int i = 0; i < 8; i++) {
            float angle = (i * 45 + frame * 10) * PI / 180.0;
            int fx = x + cos(angle) * (size + 10);
            int fy = y + sin(angle) * (size + 10);
            gfx->fillTriangle(fx, fy - 5, fx - 3, fy + 5, fx + 3, fy + 5, NARUTO_CHAKRA_ORANGE);
        }
    }
}

void drawCatSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t catBlue = RGB565(100, 150, 255);
    
    gfx->fillCircle(x, y, size/2, catBlue);
    
    gfx->fillTriangle(x - size/3, y - size/4, x - size/6, y - size/2, x, y - size/4, catBlue);
    gfx->fillTriangle(x + size/3, y - size/4, x + size/6, y - size/2, x, y - size/4, catBlue);
    
    int eyeY = y - size/8;
    gfx->fillCircle(x - size/5, eyeY, size/6, TFT_WHITE);
    gfx->fillCircle(x + size/5, eyeY, size/6, TFT_WHITE);
    gfx->fillCircle(x - size/5, eyeY, size/10, TFT_BLACK);
    gfx->fillCircle(x + size/5, eyeY, size/10, TFT_BLACK);
    
    gfx->fillTriangle(x, y + size/10, x - size/15, y + size/6, x + size/15, y + size/6, RGB565(255, 150, 150));
    gfx->drawLine(x, y + size/6, x, y + size/4, TFT_BLACK);
    // Arc simplified for CO5300 compatibility
    // Arc simplified for CO5300 compatibility
    
    int tailWave = sin(frame * PI / 2) * 5;
    gfx->drawLine(x + size/3, y + size/4, x + size/2 + tailWave, y + size/2, catBlue);
    gfx->drawLine(x + size/3 + 1, y + size/4, x + size/2 + 1 + tailWave, y + size/2, catBlue);
    
    if (evo >= EVO_CHILD) {
        for (int i = 0; i < 4; i++) {
            float angle = (i * 90 + frame * 30) * PI / 180.0;
            int sx = x + cos(angle) * (size/2 + 8);
            int sy = y + sin(angle) * (size/2 + 8);
            gfx->drawLine(sx - 2, sy, sx + 2, sy, TFT_WHITE);
            gfx->drawLine(sx, sy - 2, sx, sy + 2, TFT_WHITE);
        }
    }
}

void drawDemonSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinPale = RGB565(255, 230, 230);
    uint16_t hairBlack = RGB565(20, 20, 30);
    uint16_t pinkAccent = RGB565(255, 150, 180);
    
    gfx->fillCircle(x, y - size/4, size/2 + size/6, hairBlack);
    gfx->fillCircle(x, y, size/2, skinPale);
    
    int eyeY = y - size/10;
    gfx->fillCircle(x - size/4, eyeY, size/6, pinkAccent);
    gfx->fillCircle(x + size/4, eyeY, size/6, pinkAccent);
    gfx->fillCircle(x - size/4, eyeY, size/10, TFT_BLACK);
    gfx->fillCircle(x + size/4, eyeY, size/10, TFT_BLACK);
    
    uint16_t bambooGreen = RGB565(100, 180, 100);
    gfx->fillRect(x - size/3, y + size/8, size*2/3, size/6, bambooGreen);
    gfx->drawRect(x - size/3, y + size/8, size*2/3, size/6, RGB565(60, 120, 60));
    
    gfx->fillTriangle(x + size/4, y - size/2, x + size/3, y - size/2 - size/4, x + size/5, y - size/2, RGB565(255, 200, 200));
    gfx->fillRect(x - size/2, y - size/3, size/8, size/4, pinkAccent);
    
    if (evo >= EVO_ADULT) {
        gfx->drawLine(x - size/3, y - size/4, x - size/4, y - size/6, pinkAccent);
        gfx->drawLine(x + size/3, y - size/4, x + size/4, y - size/6, pinkAccent);
    }
    if (evo >= EVO_AWAKENED) {
        for (int i = 0; i < 6; i++) {
            float angle = (i * 60 + frame * 15) * PI / 180.0;
            int bx = x + cos(angle) * (size + 12);
            int by = y + sin(angle) * (size + 12);
            gfx->fillCircle(bx, by, 3, pinkAccent);
        }
    }
}

void drawInfinitySprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinTone = RGB565(255, 220, 200);
    uint16_t hairWhite = RGB565(240, 240, 255);
    uint16_t eyeBlue = GOJO_INFINITY_BLUE;
    
    gfx->fillCircle(x, y - size/4, size/2, hairWhite);
    gfx->fillRect(x - size/2, y - size/3, size, size/3, hairWhite);
    gfx->fillCircle(x, y, size/2, skinTone);
    
    if (evo < EVO_AWAKENED) {
        gfx->fillRect(x - size/2, y - size/6, size, size/5, TFT_BLACK);
    } else {
        int eyeY = y - size/10;
        gfx->fillCircle(x - size/4, eyeY, size/6, TFT_WHITE);
        gfx->fillCircle(x + size/4, eyeY, size/6, TFT_WHITE);
        gfx->fillCircle(x - size/4, eyeY, size/8, eyeBlue);
        gfx->fillCircle(x + size/4, eyeY, size/8, eyeBlue);
        gfx->drawCircle(x - size/4, eyeY, size/6 + 2, eyeBlue);
        gfx->drawCircle(x + size/4, eyeY, size/6 + 2, eyeBlue);
    }
    
    // Arc simplified for CO5300 compatibility
    
    int barrierR = size/2 + 5 + frame * 2;
    gfx->drawCircle(x, y, barrierR, eyeBlue);
    if (evo >= EVO_ADULT) {
        gfx->drawCircle(x, y, barrierR + 3, RGB565(200, 220, 255));
        gfx->drawCircle(x, y, barrierR + 6, RGB565(220, 230, 255));
    }
    
    gfx->fillRect(x - size/3, y + size/3, size*2/3, size/6, TFT_BLACK);
}

void drawBladesSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinTone = RGB565(255, 230, 210);
    uint16_t hairBlack = RGB565(30, 30, 35);
    uint16_t cloakGreen = LEVI_SURVEY_GREEN;
    uint16_t bladeGray = LEVI_SILVER_BLADE;
    
    gfx->fillTriangle(x - size/2, y, x + size/2, y, x, y + size/2 + size/4, cloakGreen);
    gfx->fillCircle(x, y - size/6, size/2, skinTone);
    
    gfx->fillRect(x - size/2, y - size/2, size, size/4, hairBlack);
    gfx->fillCircle(x, y - size/3, size/3, hairBlack);
    
    int eyeY = y - size/5;
    gfx->fillRect(x - size/3, eyeY, size/6, size/12, TFT_BLACK);
    gfx->fillRect(x + size/6, eyeY, size/6, size/12, TFT_BLACK);
    
    gfx->fillTriangle(x, y + size/6, x - size/8, y + size/3, x + size/8, y + size/3, TFT_WHITE);
    
    int bladeLen = size/2 + (frame % 2) * 3;
    gfx->fillRect(x - size/2 - bladeLen, y - size/8, bladeLen, size/10, bladeGray);
    gfx->fillRect(x + size/2, y - size/8, bladeLen, size/10, bladeGray);
    gfx->drawLine(x - size/2 - bladeLen + 2, y - size/8 + 1, x - size/2 - 5, y - size/8 + 1, TFT_WHITE);
    gfx->drawLine(x + size/2 + 5, y - size/8 + 1, x + size/2 + bladeLen - 2, y - size/8 + 1, TFT_WHITE);
    
    gfx->fillRect(x - size/3, y + size/4, size/10, size/6, RGB565(100, 100, 110));
    gfx->fillRect(x + size/4, y + size/4, size/10, size/6, RGB565(100, 100, 110));
    
    if (evo >= EVO_AWAKENED) {
        gfx->drawLine(x - size/2, y + size/3, x - size/2 - size/4, y + size/2, cloakGreen);
        gfx->drawLine(x + size/2, y + size/3, x + size/2 + size/4, y + size/2, cloakGreen);
    }
}

void drawHeroSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t metalGray = RGB565(180, 180, 190);
    uint16_t metalDark = RGB565(80, 80, 90);
    uint16_t eyeYellow = RGB565(255, 220, 50);
    
    gfx->fillCircle(x, y, size/2, metalGray);
    gfx->fillCircle(x, y, size/4, metalDark);
    gfx->fillCircle(x, y, size/6, eyeYellow);
    
    int headY = y - size/2;
    gfx->fillCircle(x, headY, size/3, metalGray);
    
    gfx->fillRect(x - size/4, headY - size/12, size/6, size/8, TFT_BLACK);
    gfx->fillRect(x + size/10, headY - size/12, size/6, size/8, TFT_BLACK);
    gfx->fillCircle(x - size/6, headY - size/16, size/16, eyeYellow);
    gfx->fillCircle(x + size/5, headY - size/16, size/16, eyeYellow);
    
    int armWave = sin(frame * PI / 2) * 3;
    gfx->fillRect(x - size/2 - size/4, y - size/8 + armWave, size/4, size/6, metalGray);
    gfx->fillRect(x + size/2, y - size/8 - armWave, size/4, size/6, metalGray);
    
    for (int i = 0; i < 3; i++) {
        gfx->drawLine(x - size/3 + i * size/6, y + size/4, x - size/3 + i * size/6, y + size/3, metalDark);
    }
    
    if (evo >= EVO_ADULT) {
        gfx->drawCircle(x, y, size/6 + 2, RGB565(255, 150, 50));
    }
    if (evo >= EVO_AWAKENED) {
        for (int i = 0; i < 8; i++) {
            float angle = (i * 45 + frame * 20) * PI / 180.0;
            int px2 = x + cos(angle) * (size + 8);
            int py2 = y + sin(angle) * (size + 8);
            gfx->fillCircle(px2, py2, 2, eyeYellow);
        }
    }
}

void drawMightSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinTone = RGB565(255, 220, 180);
    uint16_t hairGreen = DEKU_HERO_GREEN;
    uint16_t suitGreen = RGB565(60, 140, 80);
    
    gfx->fillCircle(x, y, size/2, suitGreen);
    gfx->fillCircle(x, y - size/8, size/3, skinTone);
    
    for (int i = 0; i < 7; i++) {
        float angle = (i * 25 - 75) * PI / 180.0;
        int hx = x + cos(angle) * (size/3);
        int hy = y - size/3 + sin(angle) * (size/4);
        gfx->fillCircle(hx, hy - size/6, size/8, hairGreen);
    }
    
    int eyeY = y - size/6;
    gfx->fillCircle(x - size/6, eyeY, size/8, TFT_WHITE);
    gfx->fillCircle(x + size/6, eyeY, size/8, TFT_WHITE);
    gfx->fillCircle(x - size/6, eyeY, size/12, hairGreen);
    gfx->fillCircle(x + size/6, eyeY, size/12, hairGreen);
    
    for (int i = 0; i < 4; i++) {
        gfx->fillCircle(x - size/5 + (i % 2) * size/10, y + (i / 2) * 4, 1, RGB565(200, 150, 130));
    }
    
    // Arc simplified for CO5300 compatibility
    
    if (evo >= EVO_CHILD) {
        int boltCount = 3 + (int)evo;
        for (int i = 0; i < boltCount; i++) {
            int bx = x + random(-size/2, size/2);
            int by = y + random(-size/2, size/2);
            gfx->drawLine(bx, by, bx + random(-5, 5), by + random(5, 10), DEKU_ALLMIGHT_GOLD);
            gfx->drawLine(bx + random(-5, 5), by + random(5, 10), bx + random(-5, 5), by + random(10, 15), DEKU_ALLMIGHT_GOLD);
        }
    }
    
    if (evo >= EVO_AWAKENED) {
        gfx->drawCircle(x, y, size/2 + 8, DEKU_HERO_GREEN);
        gfx->drawCircle(x, y, size/2 + 10, DEKU_ALLMIGHT_GOLD);
    }
}

void drawElementSprite(int x, int y, int size, CompanionEvolution evo) {
    int frame = companion_system.animation_frame;
    
    uint16_t skinTone = RGB565(200, 160, 120);
    uint16_t capOrange = BBB_BAND_ORANGE;
    uint16_t vestBlack = RGB565(30, 30, 40);
    
    gfx->fillCircle(x, y, size/2, vestBlack);
    gfx->fillCircle(x, y - size/8, size/3, skinTone);
    
    gfx->fillCircle(x, y - size/3, size/3, capOrange);
    gfx->fillRect(x - size/3, y - size/3 - size/10, size*2/3, size/8, capOrange);
    
    int eyeY = y - size/6;
    gfx->fillCircle(x - size/6, eyeY, size/8, TFT_WHITE);
    gfx->fillCircle(x + size/6, eyeY, size/8, TFT_WHITE);
    gfx->fillCircle(x - size/6, eyeY, size/12, RGB565(80, 60, 40));
    gfx->fillCircle(x + size/6, eyeY, size/12, RGB565(80, 60, 40));
    
    // Arc simplified for CO5300 compatibility
    gfx->drawCircle(x, y, size/2 + 5, capOrange);
    
    int elements = 1 + (int)evo;
    uint16_t elemColors[] = {
        RGB565(255, 100, 50),   // Fire
        RGB565(50, 150, 255),   // Water
        RGB565(100, 200, 100),  // Earth
        RGB565(200, 200, 50),   // Lightning
        RGB565(150, 150, 150),  // Wind
        RGB565(100, 50, 150),   // Shadow
        RGB565(255, 255, 200)   // Light
    };
    
    for (int e = 0; e < min(elements, 7); e++) {
        float angle = (e * (360 / elements) + frame * 10) * PI / 180.0;
        int ex = x + cos(angle) * (size/2 + 12);
        int ey = y + sin(angle) * (size/2 + 12);
        gfx->fillCircle(ex, ey, 5, elemColors[e]);
    }
    
    if (evo >= EVO_CHILD) {
        int ox = x + size/2 + 10;
        int oy = y - size/4;
        gfx->fillCircle(ox, oy, size/6, BBB_OCHOBOT_WHITE);
        gfx->fillCircle(ox - 3, oy - 2, 2, TFT_BLACK);
        gfx->fillCircle(ox + 3, oy - 2, 2, TFT_BLACK);
        // Arc simplified for CO5300 compatibility
    }
}

// Main sprite dispatcher
void drawCompanionSprite(int x, int y, CompanionType type, CompanionEvolution evo) {
    if (!gfx) return;  // Safety check
    int size = 60;
    
    yield();  // Feed watchdog before sprite draw
    
    switch (type) {
        case COMP_SUNNY:    drawSunnySprite(x, y, size, evo); break;
        case COMP_IGRIS:    drawShadowSprite(x, y, size, evo); break;
        case COMP_AZ:       drawPortalSprite(x, y, size, evo); break;
        case COMP_KURAMA:   drawFoxSprite(x, y, size, evo); break;
        case COMP_PUAR:     drawCatSprite(x, y, size, evo); break;
        case COMP_NEZUKO:   drawDemonSprite(x, y, size, evo); break;
        case COMP_SPIRIT:   drawInfinitySprite(x, y, size, evo); break;
        case COMP_BLADES:   drawBladesSprite(x, y, size, evo); break;
        case COMP_GENOS:    drawHeroSprite(x, y, size, evo); break;
        case COMP_ALLMIGHT: drawMightSprite(x, y, size, evo); break;
        case COMP_OCHOBOT:  drawElementSprite(x, y, size, evo); break;
        default: break;
    }
}

// ============================================================================
// CARE MODE UI
// ============================================================================

void enterCompanionCareMode() {
    companion_system.in_care_mode = true;
    companion_system.care_menu_selection = 0;
}

void exitCompanionCareMode() {
    companion_system.in_care_mode = false;
    companion_system.in_mini_game = false;
    companion_system.current_game.active = false;
}

void drawCompanionCareScreen() {
    if (!companion_system.current_companion) return;
    if (!gfx) return;
    
    CompanionData* c = companion_system.current_companion;
    ThemeColors* theme = getCurrentTheme();
    int centerX = LCD_WIDTH / 2;
    
    gfx->fillScreen(RGB565(2, 2, 5));
    for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
        gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(4, 4, 7));
    }
    
    yield();
    
    // === HEADER BAR ===
    int headerH = 48;
    gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
    for (int hx = 0; hx < LCD_WIDTH; hx += 8) {
        gfx->fillRect(hx, headerH - 3, 6, 3, c->profile->primary_color);
    }
    gfx->setTextColor(c->profile->primary_color);
    gfx->setTextSize(2);
    gfx->setCursor(15, 8);
    gfx->print(c->profile->name);
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(120, 125, 140));
    gfx->setCursor(15, 30);
    gfx->print(c->profile->series);
    
    // Mood + Evolution badges (top-right)
    const char* moodStr[] = {"ECSTATIC", "HAPPY", "CONTENT", "SAD", "MISERABLE"};
    uint16_t moodColors[] = {RGB565(255, 220, 50), RGB565(100, 255, 100), RGB565(150, 180, 200), RGB565(100, 120, 180), RGB565(200, 80, 80)};
    gfx->setTextSize(1);
    gfx->setTextColor(moodColors[c->stats.mood]);
    gfx->setCursor(LCD_WIDTH - 80, 8);
    gfx->print(moodStr[c->stats.mood]);
    
    const char* evoStr[] = {"BABY", "CHILD", "ADULT", "AWAKENED"};
    uint16_t evoColors[] = {RGB565(180, 180, 190), RGB565(100, 200, 255), RGB565(200, 150, 255), RGB565(255, 200, 50)};
    gfx->setTextColor(evoColors[c->stats.evolution]);
    gfx->setCursor(LCD_WIDTH - 80, 22);
    gfx->print(evoStr[c->stats.evolution]);
    
    // === SPRITE with themed glow ===
    int spriteY = 110;
    for (int r = 50; r > 30; r -= 3) {
        gfx->drawCircle(centerX, spriteY, r, RGB565(8, 5, 12));
    }
    drawCompanionSprite(centerX, spriteY, c->type, c->stats.evolution);
    
    yield();
    
    // === CATCHPHRASE ===
    gfx->setTextColor(c->profile->secondary_color);
    gfx->setTextSize(1);
    int cpLen = strlen(c->profile->catchphrase);
    gfx->setCursor(centerX - (cpLen * 3), 160);
    gfx->print(c->profile->catchphrase);
    
    // === STATS PANEL ===
    int panelY = 178;
    int panelH = 135;
    gfx->fillRect(15, panelY, LCD_WIDTH - 30, panelH, RGB565(10, 12, 18));
    gfx->drawRect(15, panelY, LCD_WIDTH - 30, panelH, RGB565(30, 35, 50));
    gfx->fillRect(15, panelY, 5, 5, c->profile->primary_color);
    gfx->fillRect(LCD_WIDTH - 20, panelY, 5, 5, c->profile->primary_color);
    
    // Stat bars
    const char* statLabels[] = {"HUNGER", "HAPPY", "ENERGY", "BOND"};
    int statValues[] = {c->stats.hunger, c->stats.happiness, c->stats.energy, c->stats.bond_level};
    uint16_t statColors[] = {RGB565(255, 150, 50), RGB565(255, 220, 50), RGB565(80, 220, 130), RGB565(200, 120, 255)};
    
    int barX = 80;
    int barW = LCD_WIDTH - 135;
    int barH = 14;
    
    for (int i = 0; i < 4; i++) {
        int sy = panelY + 10 + i * 28;
        
        gfx->setTextColor(statColors[i]);
        gfx->setTextSize(1);
        gfx->setCursor(25, sy + 3);
        gfx->print(statLabels[i]);
        
        gfx->fillRect(barX, sy, barW, barH, RGB565(8, 10, 14));
        gfx->drawRect(barX, sy, barW, barH, RGB565(25, 30, 40));
        
        int fillW = max(0, (barW - 2) * statValues[i] / 100);
        if (fillW > 0) {
            gfx->fillRect(barX + 1, sy + 1, fillW, barH - 2, statColors[i]);
            gfx->drawFastHLine(barX + 1, sy + 1, fillW, COLOR_WHITE);
        }
        
        gfx->setTextColor(COLOR_WHITE);
        gfx->setCursor(barX + barW + 6, sy + 3);
        gfx->printf("%d%%", statValues[i]);
    }
    
    // Bond rank stars
    int starsY = panelY + panelH - 20;
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(25, starsY);
    gfx->print("RANK:");
    for (int s = 0; s < 5; s++) {
        int sx = 70 + s * 18;
        uint16_t starColor = (s < c->stats.bond_rank) ? RGB565(255, 200, 50) : RGB565(40, 45, 55);
        gfx->fillRect(sx, starsY - 2, 12, 12, starColor);
    }
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(175, starsY);
    gfx->printf("Days:%d Plays:%d", c->stats.days_together, c->stats.total_interactions);
    
    yield();
    
    // === INFO ROW ===
    int infoY = panelY + panelH + 6;
    int halfW = (LCD_WIDTH - 40) / 2;
    gfx->fillRect(15, infoY, halfW, 26, RGB565(12, 14, 20));
    gfx->drawRect(15, infoY, halfW, 26, RGB565(30, 35, 50));
    gfx->setTextColor(RGB565(255, 180, 80));
    gfx->setTextSize(1);
    gfx->setCursor(22, infoY + 4);
    gfx->printf("Fav: %s", c->profile->favorite_food);
    gfx->setTextColor(RGB565(100, 200, 255));
    gfx->setCursor(22, infoY + 15);
    gfx->printf("Likes: %s", c->profile->favorite_game);
    
    int sleepX = 15 + halfW + 10;
    gfx->fillRect(sleepX, infoY, halfW, 26, RGB565(12, 14, 20));
    gfx->drawRect(sleepX, infoY, halfW, 26, RGB565(30, 35, 50));
    gfx->setTextColor(c->care.is_sleeping ? RGB565(100, 150, 255) : RGB565(80, 200, 80));
    gfx->setTextSize(1);
    gfx->setCursor(sleepX + 8, infoY + 9);
    gfx->print(c->care.is_sleeping ? "STATUS: SLEEPING" : "STATUS: AWAKE");
    
    // === ACTION BUTTONS ===
    int btnY = infoY + 36;
    int btnW = 85;
    int btnH = 48;
    int btnGap = 8;
    int btnStartX = (LCD_WIDTH - (4 * btnW + 3 * btnGap)) / 2;
    
    const char* btnLabels[] = {"FEED", "PLAY", "TRAIN", "REST"};
    uint16_t btnColors[] = {RGB565(255, 150, 50), RGB565(255, 220, 50), RGB565(80, 220, 130), RGB565(100, 150, 255)};
    
    for (int i = 0; i < 4; i++) {
        int bx = btnStartX + i * (btnW + btnGap);
        bool selected = (companion_system.care_menu_selection == i);
        uint16_t bg = selected ? btnColors[i] : RGB565(15, 18, 25);
        uint16_t border = selected ? COLOR_WHITE : btnColors[i];
        uint16_t textColor = selected ? RGB565(2, 2, 5) : btnColors[i];
        
        gfx->fillRect(bx, btnY, btnW, btnH, bg);
        gfx->drawRect(bx, btnY, btnW, btnH, border);
        gfx->fillRect(bx, btnY, 4, 4, btnColors[i]);
        gfx->fillRect(bx + btnW - 4, btnY, 4, 4, btnColors[i]);
        
        gfx->setTextColor(textColor);
        gfx->setTextSize(2);
        int labelW = strlen(btnLabels[i]) * 12;
        gfx->setCursor(bx + (btnW - labelW) / 2, btnY + 16);
        gfx->print(btnLabels[i]);
    }
    
    // === FOOTER ===
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 55, LCD_HEIGHT - 25);
    gfx->print("SWIPE LEFT TO EXIT");
    
    drawSwipeIndicator();
    yield();
}

void handleCareModeTouch(int tx, int ty) {
    // Updated button positions to match new care screen layout
    // Buttons start after info row: panelY(178) + panelH(135) + infoRow(6+26) + gap(36) = ~381
    int panelY = 178;
    int panelH = 135;
    int infoY = panelY + panelH + 6;
    int btnY = infoY + 36;
    int btnH = 48;
    int btnW = 85;
    int btnGap = 8;
    int btnStartX = (LCD_WIDTH - (4 * btnW + 3 * btnGap)) / 2;
    
    if (ty >= btnY && ty <= btnY + btnH) {
        for (int i = 0; i < 4; i++) {
            int bx = btnStartX + i * (btnW + btnGap);
            if (tx >= bx && tx <= bx + btnW) {
                companion_system.care_menu_selection = i;
                
                switch (i) {
                    case 0: feedCompanion(); break;
                    case 1: playWithCompanion(); break;
                    case 2: trainCompanion(); break;
                    case 3: toggleCompanionSleep(); break;
                }
                break;
            }
        }
    }
}

// ============================================================================
// GETTERS
// ============================================================================

CompanionData* getCurrentCompanion() {
    return companion_system.current_companion;
}

bool isCompanionCareMode() {
    return companion_system.in_care_mode;
}

bool isCompanionGameActive() {
    return companion_system.in_mini_game && companion_system.current_game.active;
}
