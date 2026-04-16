/*
 * boss_rush.cpp - Boss Rush Challenge Implementation (IMPROVED)
 * Complete boss battle system with 20 anime bosses
 *
 * FIXES:
 * - Improved UI with retro pixel style
 * - Tier selection actually works
 * - Better touch handling
 * - Returns to app grid properly
 *
 * FUSION OS:
 * - XP rewards based on boss tier
 */

#include "boss_rush.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "gacha.h"
#include "navigation.h"
#include "xp_system.h"  // FUSION OS: XP rewards
#include <Arduino.h>
#include "sd_preferences.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Number of boss tiers
const int BOSS_TIERS = 7;

// Boss database
BossData boss_database[TOTAL_BOSSES];
bool bosses_defeated[TOTAL_BOSSES];

// Combat state
int current_boss_index = 0;
int current_tier = 0;  // Which tier is selected
int player_hp, player_max_hp;
int player_attack, player_defense;
int player_energy;
int potions_remaining;
int combo_count;
int turn_count;
bool player_defending;
bool battle_active;

// Battle log
String battle_log[5];
int log_index = 0;

// Screen state
static bool show_boss_selection = false;

// =============================================================================
// BOSS DATABASE
// =============================================================================

// FUSION OS: Boss Move Names (4 moves per boss)
const char* BOSS_MOVES[][4] = {
    // Tier 1
    {"Bara Bara Cannon", "Split Attack", "Buggy Ball", "Chop Rush"},          // Buggy
    {"Water Dragon", "Hidden Mist", "Silent Kill", "Blade Slash"},            // Zabuza
    {"Double Sunday", "Saturday Crush", "Power Blast", "Tail Whip"},          // Raditz
    {"Blood Feast", "Hand Crusher", "Terror Scream", "Demon Grip"},           // Hand Demon
    {"Cursed Strike", "Shadow Bind", "Dark Pulse", "Spirit Drain"},           // Cursed Spirit

    // Tier 2
    {"Desert Spada", "Ground Death", "Sables Storm", "Poison Hook"},          // Crocodile
    {"Kusanagi Slash", "Body Replacement", "Snake Summon", "Cursed Seal"},    // Orochimaru
    {"Death Beam", "Supernova", "Death Ball", "Tail Slam"},                   // Frieza
    {"Blood Thread", "Spider Web", "Demon Art", "Moon Slice"},                // Rui
    {"Idle Transfiguration", "Domain Expansion", "Black Flash", "Soul Crush"},// Mahito

    // Tier 3
    {"Overheat Whip", "Parasite String", "Bird Cage", "God Thread"},          // Doflamingo
    {"Almighty Push", "Planetary Devastation", "Soul Rip", "Chibaku Tensei"}, // Pain
    {"Perfect Kamehameha", "Solar Flare", "Regeneration", "Death Beam"},      // Cell
    {"Air Type", "Compass Needle", "Leg Technique", "Demon Blood"},           // Akaza
    {"Cleave", "Dismantle", "Domain Expansion", "Malevolent Shrine"},         // Sukuna 4F

    // Tier 4
    {"Thunder Bagua", "Flame Clouds", "Dragon Twister", "Blast Breath"},      // Kaido
    {"Perfect Susanoo", "Infinite Tsukuyomi", "Limbo Clone", "God Tree"},     // Madara
    {"Power Impact", "Overheat Magnetron", "Ultimate Strike", "Omega Blast"}, // Jiren
    {"Shock Absorption", "Blood Demon Art", "Flesh Manipulation", "Decay"},   // Muzan
    {"World Slash", "Fire Arrow", "Domain Expansion", "Fuga"},                // Sukuna Full

    // Tier 5 - Godlike Bosses
    {"Ikoku Sovereignty", "Divine Split", "Conqueror's Infusion", "Bajrang"}, // Shanks
    {"Ash Bone", "Truth-Seeking Orb", "Rinne Rebirth", "Infinite Void"},      // Kaguya
    {"Hakai", "Ultra Instinct", "Sphere of Destruction", "Erasure"},          // Beerus
    {"Final Moon", "Sun Absorption", "Demon King Form", "Eternity"},          // Muzan Final
    {"Malevolent Shrine", "Cleave Barrage", "Domain Expansion", "Fuga Max"},  // Sukuna 20F

    // Tier 6 - Multiverse Bosses
    {"Five Elders Power", "World Government", "Ancient Weapon", "Void Century"}, // Imu
    {"Baryon Mode Counter", "Otsutsuki God Tree", "Infinite Tsukuyomi", "Code"}, // Shibai
    {"Ultra Ego", "Hakai Bomb", "Granolah Eye", "GoD Aura"},                     // Vegeta/GoD
    {"Cosmic Fear Mode", "Gravity Knuckle", "Gamma Burst", "Awakened Garou"},   // Garou
    {"Infinite Void Max", "Six Eyes Limit", "Purple Hollow", "Domain Collapse"},// Gojo Unsealed

    // Tier 7 - Final Bosses (Ultimate)
    {"One Piece Awakening", "Dream Crusher", "World Ender", "Final Will"},    // Joy Boy
    {"Sage of Six Paths", "Creation of All", "10 Tails Bomb", "Divine Gate"}, // Hagoromo
    {"Grand Priest Ki", "Angel Arts", "Erase", "Ultra Complete"},             // Grand Priest
    {"Breathing: 13th Form", "Transparent World", "Sun Halo", "Final Breath"},// Yoriichi
    {"Megumi Vessel", "King of Curses", "True Form", "Dismantle All"}         // True Sukuna
};

void initBossDatabase() {
    // Tier 1 Bosses (Level 1-5)
    boss_database[0]  = {"Buggy", "One Piece", BOSS_TIER_1, 5000, 5000, 300, 100, 1, false, 50};
    boss_database[1]  = {"Zabuza", "Naruto", BOSS_TIER_1, 6000, 6000, 400, 150, 2, false, 60};
    boss_database[2]  = {"Raditz", "Dragon Ball", BOSS_TIER_1, 7000, 7000, 500, 120, 3, false, 70};
    boss_database[3]  = {"Hand Demon", "Demon Slayer", BOSS_TIER_1, 5500, 5500, 350, 80, 4, false, 55};
    boss_database[4]  = {"Cursed Spirit", "JJK", BOSS_TIER_1, 6500, 6500, 380, 130, 5, false, 65};

    // Tier 2 Bosses (Level 6-10)
    boss_database[5]  = {"Crocodile", "One Piece", BOSS_TIER_2, 15000, 15000, 800, 300, 6, false, 150};
    boss_database[6]  = {"Orochimaru", "Naruto", BOSS_TIER_2, 18000, 18000, 900, 350, 7, false, 180};
    boss_database[7]  = {"Frieza", "Dragon Ball", BOSS_TIER_2, 20000, 20000, 1000, 400, 8, false, 200};
    boss_database[8]  = {"Rui", "Demon Slayer", BOSS_TIER_2, 16000, 16000, 850, 280, 9, false, 160};
    boss_database[9]  = {"Mahito", "JJK", BOSS_TIER_2, 19000, 19000, 950, 320, 10, false, 190};

    // Tier 3 Bosses (Level 11-15)
    boss_database[10] = {"Doflamingo", "One Piece", BOSS_TIER_3, 40000, 40000, 1500, 600, 11, false, 300};
    boss_database[11] = {"Pain", "Naruto", BOSS_TIER_3, 45000, 45000, 1800, 700, 12, false, 350};
    boss_database[12] = {"Cell", "Dragon Ball", BOSS_TIER_3, 50000, 50000, 2000, 800, 13, false, 400};
    boss_database[13] = {"Akaza", "Demon Slayer", BOSS_TIER_3, 42000, 42000, 1600, 550, 14, false, 320};
    boss_database[14] = {"Sukuna 4F", "JJK", BOSS_TIER_3, 48000, 48000, 1900, 750, 15, false, 380};

    // Tier 4 Bosses (Level 16-20)
    boss_database[15] = {"Kaido", "One Piece", BOSS_TIER_4, 100000, 100000, 3000, 1200, 16, false, 600};
    boss_database[16] = {"Madara", "Naruto", BOSS_TIER_4, 110000, 110000, 3500, 1400, 17, false, 700};
    boss_database[17] = {"Jiren", "Dragon Ball", BOSS_TIER_4, 120000, 120000, 4000, 1600, 18, false, 800};
    boss_database[18] = {"Muzan", "Demon Slayer", BOSS_TIER_4, 105000, 105000, 3200, 1100, 19, false, 650};
    boss_database[19] = {"Sukuna Full", "JJK", BOSS_TIER_4, 150000, 150000, 5000, 2000, 20, false, 1000};

    // Tier 5 Bosses (Level 21-25) - GODLIKE
    boss_database[20] = {"Shanks", "One Piece", BOSS_TIER_5, 250000, 250000, 7000, 3000, 21, false, 1500};
    boss_database[21] = {"Kaguya", "Naruto", BOSS_TIER_5, 280000, 280000, 8000, 3500, 22, false, 1800};
    boss_database[22] = {"Beerus", "Dragon Ball", BOSS_TIER_5, 300000, 300000, 9000, 4000, 23, false, 2000};
    boss_database[23] = {"Muzan Final", "Demon Slayer", BOSS_TIER_5, 260000, 260000, 7500, 2800, 24, false, 1600};
    boss_database[24] = {"Sukuna 20F", "JJK", BOSS_TIER_5, 320000, 320000, 10000, 4500, 25, false, 2500};

    // Tier 6 Bosses (Level 26-30) - MULTIVERSE
    boss_database[25] = {"Imu", "One Piece", BOSS_TIER_6, 500000, 500000, 12000, 5000, 26, false, 3000};
    boss_database[26] = {"Shibai", "Naruto", BOSS_TIER_6, 550000, 550000, 14000, 6000, 27, false, 3500};
    boss_database[27] = {"GoD Vegeta", "Dragon Ball", BOSS_TIER_6, 600000, 600000, 15000, 7000, 28, false, 4000};
    boss_database[28] = {"Cosmic Garou", "OPM", BOSS_TIER_6, 520000, 520000, 13000, 5500, 29, false, 3200};
    boss_database[29] = {"Gojo Unsealed", "JJK", BOSS_TIER_6, 580000, 580000, 14500, 6500, 30, false, 3800};

    // Tier 7 Bosses (Level 31-35) - ULTIMATE FINAL
    boss_database[30] = {"Joy Boy", "One Piece", BOSS_TIER_7, 1000000, 1000000, 20000, 10000, 31, false, 5000};
    boss_database[31] = {"Hagoromo", "Naruto", BOSS_TIER_7, 1100000, 1100000, 22000, 11000, 32, false, 6000};
    boss_database[32] = {"Grand Priest", "Dragon Ball", BOSS_TIER_7, 1500000, 1500000, 30000, 15000, 33, false, 8000};
    boss_database[33] = {"Yoriichi", "Demon Slayer", BOSS_TIER_7, 1200000, 1200000, 25000, 12000, 34, false, 7000};
    boss_database[34] = {"True Sukuna", "JJK", BOSS_TIER_7, 2000000, 2000000, 35000, 18000, 35, false, 10000};

    Serial.println("[BossRush] Database initialized (35 bosses, 7 tiers)");
}

// =============================================================================
// SYSTEM FUNCTIONS
// =============================================================================

void initBossRush() {
    Serial.println("[BossRush] Initializing...");
    initBossDatabase();

    if (!loadBossProgress()) {
        for (int i = 0; i < TOTAL_BOSSES; i++) {
            bosses_defeated[i] = false;
        }
        system_state.bosses_defeated = 0;
    }

    show_boss_selection = false;
}

void saveBossProgress() {
    // Save boss defeat status to NVS
    SDPreferences bossPrefs;
    bossPrefs.begin("bossrush", false);
    bossPrefs.putInt("defeated", getBossesDefeated());
    
    // Pack boss defeat flags into bytes (35 bosses = 5 bytes)
    for (int i = 0; i < TOTAL_BOSSES; i++) {
        char key[12];
        snprintf(key, sizeof(key), "b%d", i);
        bossPrefs.putBool(key, bosses_defeated[i]);
    }
    bossPrefs.end();
    
    // Also save to SD card if available
    extern bool sdCardInitialized;
    if (sdCardInitialized) {
        extern bool saveBossDataToSD();
        saveBossDataToSD();
    }
    
    Serial.println("[BossRush] Progress saved (NVS + SD)");
}

bool loadBossProgress() {
    SDPreferences bossPrefs;
    bossPrefs.begin("bossrush", true);  // read-only
    
    int saved_defeated = bossPrefs.getInt("defeated", -1);
    if (saved_defeated < 0) {
        bossPrefs.end();
        return false;  // No save data found
    }
    
    for (int i = 0; i < TOTAL_BOSSES; i++) {
        char key[12];
        snprintf(key, sizeof(key), "b%d", i);
        bosses_defeated[i] = bossPrefs.getBool(key, false);
    }
    
    system_state.bosses_defeated = getBossesDefeated();
    bossPrefs.end();
    
    Serial.printf("[BossRush] Progress loaded: %d bosses defeated\n", system_state.bosses_defeated);
    return true;
}

BossData* getBoss(int index) {
    if (index < 0 || index >= TOTAL_BOSSES) return nullptr;
    return &boss_database[index];
}

BossData* getCurrentBoss() {
    return getBoss(current_boss_index);
}

int getBossesDefeated() {
    int count = 0;
    for (int i = 0; i < TOTAL_BOSSES; i++) {
        if (bosses_defeated[i]) count++;
    }
    return count;
}

// =============================================================================
// COMBAT SYSTEM
// =============================================================================

void startBossFight(int boss_index) {
    if (boss_index < 0 || boss_index >= TOTAL_BOSSES) return;

    current_boss_index = boss_index;
    battle_active = true;
    show_boss_selection = false;

    player_max_hp = 1000 + system_state.player_level * 100 + getDeckBonusHP();
    player_hp = player_max_hp;
    player_attack = 500 + system_state.player_level * 50 + getDeckBonusATK();
    player_defense = 300 + system_state.player_level * 30 + getDeckBonusDEF();
    player_energy = 0;
    potions_remaining = MAX_POTIONS;
    combo_count = 0;
    turn_count = 0;
    player_defending = false;

    boss_database[boss_index].hp = boss_database[boss_index].max_hp;

    log_index = 0;
    for (int i = 0; i < 5; i++) battle_log[i] = "";

    addBattleLogEntry("Battle Start!");

    Serial.printf("[BossRush] Fighting: %s\n", boss_database[boss_index].name);
    drawBossBattle();
}

void addBattleLogEntry(const char* message) {
    if (log_index < 5) {
        battle_log[log_index++] = String(message);
    } else {
        for (int i = 0; i < 4; i++) battle_log[i] = battle_log[i + 1];
        battle_log[4] = String(message);
    }
}

void playerAttack() {
    if (!battle_active) return;

    BossData* boss = getCurrentBoss();
    int base_damage = player_attack - boss->defense / 2;
    int multiplier = 10 + getComboMultiplier();  // Returns 10-20

    bool critical = random(0, 100) < CRITICAL_CHANCE;
    if (critical) multiplier = (int)(multiplier * CRITICAL_MULTIPLIER);

    int damage = max(1, (base_damage * multiplier) / 10);
    boss->hp -= damage;

    incrementCombo();
    player_energy = min(100, player_energy + 25);

    char log[64];
    sprintf(log, "%sATK! %d dmg", critical ? "CRIT " : "", damage);
    addBattleLogEntry(log);

    processCombatRound();
}

void playerDefend() {
    if (!battle_active) return;
    player_defending = true;
    player_energy = min(100, player_energy + 25);
    addBattleLogEntry("Defending!");
    processCombatRound();
}

void playerSpecial() {
    if (!battle_active || player_energy < SPECIAL_ATTACK_COST) return;

    BossData* boss = getCurrentBoss();
    player_energy = 0;

    int damage = (player_attack * 3) - boss->defense / 4;
    damage = max(100, damage);
    boss->hp -= damage;

    incrementCombo();

    char log[64];
    sprintf(log, "SPECIAL! %d dmg!", damage);
    addBattleLogEntry(log);

    processCombatRound();
}

void playerUseItem() {
    if (!battle_active || potions_remaining <= 0) return;

    potions_remaining--;
    int heal = player_max_hp * POTION_HEAL_PERCENT / 100;
    player_hp = min(player_max_hp, player_hp + heal);

    char log[64];
    sprintf(log, "Potion! +%d HP", heal);
    addBattleLogEntry(log);

    processCombatRound();
}

void bossAction() {
    if (!battle_active) return;

    BossData* boss = getCurrentBoss();
    int hp_percent = (boss->hp * 100) / boss->max_hp;

    // FUSION OS: Select random move and show animation
    int move_id = random(0, 4);
    drawBossAttackAnimation(current_boss_index, move_id);

    int damage = boss->attack - player_defense / 2;

    if (hp_percent < 25) {
        damage = (int)(damage * 1.5);
        addBattleLogEntry("BERSERK!");
    }

    if (player_defending) {
        damage = damage / 2;
        player_defending = false;
    }

    damage = max(1, damage);
    player_hp -= damage;
    resetCombo();

    char log[64];
    sprintf(log, "%s: %d dmg", boss->name, damage);
    addBattleLogEntry(log);
}

// FUSION OS: Draw boss attack animation with move name
void drawBossAttackAnimation(int boss_id, int move_id) {
    if (boss_id < 0 || boss_id >= TOTAL_BOSSES) return;
    if (move_id < 0 || move_id >= 4) return;

    const char* move_name = BOSS_MOVES[boss_id][move_id];

    // Flash red background at top
    gfx->fillRect(0, 40, LCD_WIDTH, 60, RGB565(120, 0, 0));

    // Move name (large text)
    gfx->setTextSize(3);
    gfx->setTextColor(RGB565(255, 50, 50));
    int text_w = strlen(move_name) * 18;
    int text_x = max(10, (LCD_WIDTH - text_w) / 2);
    gfx->setCursor(text_x, 55);
    gfx->print(move_name);

    // Ripple animation circles
    for (int r = 10; r < 100; r += 15) {
        gfx->drawCircle(LCD_WIDTH/2, 130, r, RGB565(255, 100 - r/2, 100 - r/2));
    }

    // Impact lines
    for (int i = 0; i < 8; i++) {
        int angle = i * 45;
        int x1 = LCD_WIDTH/2;
        int y1 = 130;
        int x2 = x1 + cos(angle * 0.0174533) * 80;
        int y2 = y1 + sin(angle * 0.0174533) * 80;
        gfx->drawLine(x1, y1, x2, y2, RGB565(255, 150, 150));
    }

    // Show for 600ms
    delay(600);
}

void processCombatRound() {
    turn_count++;
    if (checkBattleEnd()) return;
    bossAction();
    checkBattleEnd();
    drawBossBattle();
}

bool checkBattleEnd() {
    BossData* boss = getCurrentBoss();

    if (boss->hp <= 0) {
        battle_active = false;
        bosses_defeated[current_boss_index] = true;
        system_state.bosses_defeated++;

        // Award gems
        system_state.player_gems += boss->gem_reward;

        // FUSION OS: Award XP based on boss tier
        int xp_reward = 0;
        switch (boss->tier) {
            case BOSS_TIER_1: xp_reward = XP_BOSS_TIER_1; break;
            case BOSS_TIER_2: xp_reward = XP_BOSS_TIER_2; break;
            case BOSS_TIER_3: xp_reward = XP_BOSS_TIER_3; break;
            case BOSS_TIER_4: xp_reward = XP_BOSS_TIER_4; break;
            case BOSS_TIER_5: xp_reward = XP_BOSS_TIER_5; break;
            case BOSS_TIER_6: xp_reward = XP_BOSS_TIER_6; break;
            case BOSS_TIER_7: xp_reward = XP_BOSS_TIER_7; break;
            default: xp_reward = boss->level * 10; break;
        }

        char xp_msg[32];
        sprintf(xp_msg, "Boss Defeated (Tier %d)", boss->tier);
        gainExperience(xp_reward, xp_msg);

        Serial.printf("[BossRush] Victory! +%d gems, +%d XP\n", boss->gem_reward, xp_reward);

        // Save gems and XP to NVS immediately
        extern void saveAllGameData();
        saveAllGameData();

        saveBossProgress();
        drawBossVictory(*boss);
        return true;
    }

    if (player_hp <= 0) {
        battle_active = false;
        drawBossDefeat();
        return true;
    }

    return false;
}

void incrementCombo() { combo_count++; }
void resetCombo() { combo_count = 0; }
int getComboMultiplier() { return min(20, 10 + combo_count); }

// =============================================================================
// UI - IMPROVED RETRO STYLE
// =============================================================================

// Scroll state for tier menu and boss selection
static int tier_scroll_offset = 0;
static int boss_scroll_offset = 0;

void drawBossRushMenu() {
    gfx->fillScreen(RGB565(2, 2, 5));
    show_boss_selection = false;

    // CRT scanlines
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();

    // Header
    gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, 46, 6, 3, COLOR_RED);
    }

    gfx->setTextColor(COLOR_RED);
    gfx->setTextSize(2);
    gfx->setCursor(90, 14);
    gfx->print("BOSS RUSH");

    // Progress bar
    int defeated = getBossesDefeated();
    gfx->setTextColor(RGB565(150, 155, 170));
    gfx->setTextSize(1);
    gfx->setCursor(20, 58);
    gfx->printf("Defeated: %d/%d", defeated, TOTAL_BOSSES);

    int barW = 200;
    gfx->fillRect(120, 55, barW, 12, RGB565(20, 20, 30));
    int fillW = (defeated * barW) / TOTAL_BOSSES;
    if (fillW > 0) gfx->fillRect(120, 55, fillW, 12, COLOR_RED);
    gfx->drawRect(120, 55, barW, 12, RGB565(60, 60, 80));

    // Player stats with deck bonuses
    int deckATK = getDeckBonusATK();
    int deckDEF = getDeckBonusDEF();
    int deckHP = getDeckBonusHP();
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setCursor(20, 78);
    gfx->printf("Lv.%d  ATK:%d  DEF:%d",
                system_state.player_level,
                500 + system_state.player_level * 50 + deckATK,
                300 + system_state.player_level * 30 + deckDEF);
    if (deckATK > 0 || deckDEF > 0 || deckHP > 0) {
        gfx->setTextColor(COLOR_CYAN);
        gfx->setTextSize(1);
        gfx->setCursor(20, 92);
        gfx->printf("DECK BONUS: +%dATK +%dHP +%dDEF", deckATK, deckHP, deckDEF);
    }

    // 7 Tier buttons - SCROLLABLE (show 4 at a time)
    const char* tiers[] = {"TIER 1", "TIER 2", "TIER 3", "TIER 4", "TIER 5", "TIER 6", "TIER 7"};
    const char* descs[] = {"Rookie Lv.1-5", "Rising Lv.6-10", "Elite Lv.11-15", "Boss Lv.16-20", "Godlike Lv.21-25", "Multiverse Lv.26-30", "ULTIMATE Lv.31-35"};
    uint16_t colors[] = {COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_RED, RGB565(255,200,50), RGB565(255,100,255), RGB565(255,255,255)};

    int visible_tiers = 4;
    int tier_item_h = 68;
    int tier_gap = 7;
    int startY = 105;

    // Clamp scroll
    int max_scroll = max(0, BOSS_TIERS - visible_tiers);
    if (tier_scroll_offset > max_scroll) tier_scroll_offset = max_scroll;
    if (tier_scroll_offset < 0) tier_scroll_offset = 0;

    for (int v = 0; v < visible_tiers && (v + tier_scroll_offset) < BOSS_TIERS; v++) {
        int i = v + tier_scroll_offset;
        int y = startY + v * (tier_item_h + tier_gap);
        int start = i * 5;
        int tier_defeated = 0;
        for (int j = start; j < start + 5 && j < TOTAL_BOSSES; j++) {
            if (bosses_defeated[j]) tier_defeated++;
        }

        // Card
        gfx->fillRect(25, y, 310, tier_item_h, RGB565(12, 14, 20));
        gfx->drawRect(25, y, 310, tier_item_h, colors[i]);
        gfx->fillRect(25, y, 5, 5, colors[i]);
        gfx->fillRect(330, y, 5, 5, colors[i]);

        // Colored bar on left
        gfx->fillRect(25, y, 8, tier_item_h, colors[i]);

        // Tier name
        gfx->setTextColor(RGB565(220, 225, 240));
        gfx->setTextSize(2);
        gfx->setCursor(45, y + 10);
        gfx->print(tiers[i]);

        // Description
        gfx->setTextColor(RGB565(100, 105, 120));
        gfx->setTextSize(1);
        gfx->setCursor(45, y + 38);
        gfx->print(descs[i]);

        // Completion
        gfx->setTextColor(tier_defeated == 5 ? COLOR_GREEN : RGB565(150, 155, 170));
        gfx->setCursor(240, y + 22);
        gfx->printf("%d/5", tier_defeated);

        if (tier_defeated == 5) {
            gfx->setTextColor(COLOR_GREEN);
            gfx->setCursor(280, y + 22);
            gfx->print("OK");
        }

        // Gem reward range
        gfx->setTextColor(COLOR_GOLD);
        gfx->setTextSize(1);
        gfx->setCursor(240, y + 42);
        int minGem = boss_database[start].gem_reward;
        int maxGem = boss_database[min(start + 4, TOTAL_BOSSES - 1)].gem_reward;
        gfx->printf("%d-%dg", minGem, maxGem);
    }

    // Scroll indicator
    if (BOSS_TIERS > visible_tiers) {
        int scrollBarH = 200;
        int scrollBarY = startY;
        int thumbH = max(20, scrollBarH * visible_tiers / BOSS_TIERS);
        int thumbY = scrollBarY + (tier_scroll_offset * (scrollBarH - thumbH)) / max(1, max_scroll);

        gfx->fillRect(LCD_WIDTH - 8, scrollBarY, 5, scrollBarH, RGB565(15, 17, 25));
        gfx->fillRect(LCD_WIDTH - 8, thumbY, 5, thumbH, RGB565(80, 85, 100));

        // Arrow hints
        if (tier_scroll_offset > 0) {
            gfx->setTextColor(RGB565(100, 105, 120));
            gfx->setTextSize(1);
            gfx->setCursor(LCD_WIDTH - 15, startY - 12);
            gfx->print("^");
        }
        if (tier_scroll_offset < max_scroll) {
            gfx->setTextColor(RGB565(100, 105, 120));
            gfx->setTextSize(1);
            gfx->setCursor(LCD_WIDTH - 15, startY + visible_tiers * (tier_item_h + tier_gap) + 2);
            gfx->print("v");
        }
    }

    // Bottom info
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 50, LCD_HEIGHT - 15);
    gfx->print("Swipe to scroll");

    drawSwipeIndicator();
}

void drawBossSelection() {
    gfx->fillScreen(RGB565(2, 2, 5));
    show_boss_selection = true;

    for (int y = 0; y < LCD_HEIGHT; y += 4) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    ThemeColors* theme = getCurrentTheme();
    uint16_t tierColor = getBossTierColor((BossTier)(current_tier + 1));

    // Header
    gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
    for (int x = 0; x < LCD_WIDTH; x += 8) {
        gfx->fillRect(x, 46, 6, 3, tierColor);
    }

    gfx->setTextColor(tierColor);
    gfx->setTextSize(2);
    gfx->setCursor(80, 14);
    gfx->printf("TIER %d BOSSES", current_tier + 1);

    // Show 5 bosses (scrollable within tier if needed, but 5 fits)
    int tier_start = current_tier * 5;

    for (int i = 0; i < 5; i++) {
        int boss_idx = tier_start + i;
        if (boss_idx >= TOTAL_BOSSES) break;
        BossData* boss = getBoss(boss_idx);
        int y = 60 + i * 78;

        uint16_t bg = bosses_defeated[boss_idx] ? RGB565(10, 30, 10) : RGB565(12, 14, 20);

        gfx->fillRect(20, y, 320, 68, bg);
        gfx->drawRect(20, y, 320, 68, tierColor);
        gfx->fillRect(20, y, 5, 5, tierColor);

        // Boss name
        gfx->setTextColor(RGB565(220, 225, 240));
        gfx->setTextSize(2);
        gfx->setCursor(35, y + 8);
        gfx->print(boss->name);

        // Series and level
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(100, 105, 120));
        gfx->setCursor(35, y + 32);
        gfx->printf("%s | Lv.%d", boss->series, boss->level);

        // HP and stats
        gfx->setTextColor(RGB565(150, 100, 100));
        gfx->setCursor(35, y + 48);
        if (boss->max_hp >= 1000000) {
            gfx->printf("HP:%dM ATK:%dK", boss->max_hp/1000000, boss->attack/1000);
        } else if (boss->max_hp >= 10000) {
            gfx->printf("HP:%dK ATK:%d", boss->max_hp/1000, boss->attack);
        } else {
            gfx->printf("HP:%d ATK:%d", boss->max_hp, boss->attack);
        }

        // Reward
        gfx->setTextColor(COLOR_GOLD);
        gfx->setCursor(250, y + 15);
        gfx->printf("+%d", boss->gem_reward);

        // Status
        if (bosses_defeated[boss_idx]) {
            gfx->setTextColor(COLOR_GREEN);
            gfx->setCursor(250, y + 42);
            gfx->print("DEFEATED");
        } else {
            gfx->setTextColor(RGB565(255, 80, 80));
            gfx->setCursor(250, y + 42);
            gfx->print("FIGHT!");
        }
    }

    // Back hint
    gfx->setTextColor(RGB565(50, 55, 70));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 45, LCD_HEIGHT - 15);
    gfx->print("Swipe down = back");

    drawSwipeIndicator();
}

void drawBossBattle() {
    gfx->fillScreen(RGB565(2, 2, 5));

    BossData* boss = getCurrentBoss();

    // Boss name and tier
    gfx->setTextColor(getBossTierColor(boss->tier));
    gfx->setTextSize(2);
    gfx->setCursor(30, 15);
    gfx->print(boss->name);

    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setCursor(30, 40);
    gfx->printf("Lv.%d - %s", boss->level, boss->series);

    // Boss HP
    drawBossHealthBar(20, 60, 320, 30, *boss);

    // Divider
    gfx->drawFastHLine(0, 100, LCD_WIDTH, RGB565(40, 40, 60));

    // Battle log
    gfx->fillRect(20, 110, 320, 80, RGB565(10, 12, 18));
    gfx->drawRect(20, 110, 320, 80, RGB565(30, 35, 50));
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setTextSize(1);
    for (int i = 0; i < min(4, log_index); i++) {
        gfx->setCursor(30, 118 + i * 18);
        gfx->print(battle_log[max(0, log_index - 4 + i)]);
    }

    // Player section
    gfx->drawFastHLine(0, 200, LCD_WIDTH, RGB565(40, 40, 60));

    CharacterProfile* profile = getCharacterProfile(system_state.current_theme);
    gfx->setTextColor(getCurrentTheme()->primary);
    gfx->setTextSize(2);
    gfx->setCursor(30, 210);
    gfx->print(profile ? profile->name : "Hero");

    // Player HP
    drawPlayerHealthBar(20, 240, 240, 25);

    // Energy
    drawEnergyBar(20, 275, 160, 18);

    // Combo
    if (combo_count > 0) {
        gfx->setTextColor(COLOR_GOLD);
        gfx->setTextSize(1);
        gfx->setCursor(200, 278);
        gfx->printf("x%d COMBO", combo_count);
    }

    // Potions
    gfx->setTextColor(COLOR_GREEN);
    gfx->setCursor(280, 250);
    gfx->printf("Potions: %d", potions_remaining);

    // Action buttons
    drawCombatActions();
}

void drawBossHealthBar(int x, int y, int w, int h, BossData& boss) {
    float progress = (float)boss.hp / boss.max_hp;

    gfx->fillRect(x, y, w, h, RGB565(40, 20, 20));
    int fillW = (int)(w * constrain(progress, 0.0f, 1.0f));
    if (fillW > 0) gfx->fillRect(x, y, fillW, h, COLOR_RED);
    gfx->drawRect(x, y, w, h, RGB565(80, 40, 40));

    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(x + w/2 - 40, y + h/2 - 4);
    gfx->printf("%d / %d", max(0, boss.hp), boss.max_hp);
}

void drawPlayerHealthBar(int x, int y, int w, int h) {
    float progress = (float)player_hp / player_max_hp;

    gfx->fillRect(x, y, w, h, RGB565(20, 40, 20));
    int fillW = (int)(w * constrain(progress, 0.0f, 1.0f));
    if (fillW > 0) {
        uint16_t color = progress > 0.3 ? COLOR_GREEN : COLOR_RED;
        gfx->fillRect(x, y, fillW, h, color);
    }
    gfx->drawRect(x, y, w, h, RGB565(40, 80, 40));

    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(x + 10, y + h/2 - 4);
    gfx->printf("HP: %d/%d", max(0, player_hp), player_max_hp);
}

void drawEnergyBar(int x, int y, int w, int h) {
    float progress = player_energy / 100.0f;

    gfx->fillRect(x, y, w, h, RGB565(20, 20, 40));
    int fillW = (int)(w * progress);
    if (fillW > 0) gfx->fillRect(x, y, fillW, h, COLOR_BLUE);
    gfx->drawRect(x, y, w, h, RGB565(40, 40, 80));

    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(x + 5, y + 3);
    gfx->printf("SP: %d%%", player_energy);
}

void drawCombatActions() {
    int y = 320;
    int btnW = 75, btnH = 50, gap = 10;
    int startX = (LCD_WIDTH - (4 * btnW + 3 * gap)) / 2;

    // Attack
    gfx->fillRect(startX, y, btnW, btnH, COLOR_RED);
    gfx->drawRect(startX, y, btnW, btnH, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(startX + 15, y + 20);
    gfx->print("ATTACK");

    // Defend
    gfx->fillRect(startX + btnW + gap, y, btnW, btnH, COLOR_BLUE);
    gfx->drawRect(startX + btnW + gap, y, btnW, btnH, COLOR_WHITE);
    gfx->setCursor(startX + btnW + gap + 12, y + 20);
    gfx->print("DEFEND");

    // Special
    uint16_t spColor = player_energy >= 100 ? COLOR_PURPLE : RGB565(60, 60, 80);
    gfx->fillRect(startX + 2 * (btnW + gap), y, btnW, btnH, spColor);
    gfx->drawRect(startX + 2 * (btnW + gap), y, btnW, btnH, COLOR_WHITE);
    gfx->setCursor(startX + 2 * (btnW + gap) + 8, y + 20);
    gfx->print("SPECIAL");

    // Item
    uint16_t itemColor = potions_remaining > 0 ? COLOR_GREEN : RGB565(60, 60, 80);
    gfx->fillRect(startX + 3 * (btnW + gap), y, btnW, btnH, itemColor);
    gfx->drawRect(startX + 3 * (btnW + gap), y, btnW, btnH, COLOR_WHITE);
    gfx->setCursor(startX + 3 * (btnW + gap) + 18, y + 20);
    gfx->print("ITEM");
}

void drawBattleLog() {
    // Integrated into drawBossBattle
}

void drawBossVictory(BossData& boss) {
    gfx->fillScreen(RGB565(2, 2, 5));

    gfx->setTextColor(COLOR_GOLD);
    gfx->setTextSize(4);
    gfx->setCursor(70, 80);
    gfx->print("VICTORY!");

    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(50, 160);
    gfx->printf("%s defeated!", boss.name);

    gfx->setTextColor(COLOR_GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(100, 220);
    gfx->printf("+%d Gems", boss.gem_reward);

    gfx->setTextColor(COLOR_CYAN);
    gfx->setCursor(100, 260);
    gfx->printf("+%d XP", boss.level * 100);

    // Continue button
    gfx->fillRect(100, 340, 160, 50, getCurrentTheme()->primary);
    gfx->drawRect(100, 340, 160, 50, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(130, 355);
    gfx->print("Continue");
}

void drawBossDefeat() {
    gfx->fillScreen(RGB565(2, 2, 5));

    gfx->setTextColor(COLOR_RED);
    gfx->setTextSize(4);
    gfx->setCursor(90, 120);
    gfx->print("DEFEAT");

    gfx->setTextColor(RGB565(150, 155, 170));
    gfx->setTextSize(1);
    gfx->setCursor(80, 220);
    gfx->print("Train harder and try again!");

    gfx->fillRect(100, 340, 160, 50, RGB565(60, 60, 80));
    gfx->drawRect(100, 340, 160, 50, COLOR_WHITE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(130, 355);
    gfx->print("Continue");
}

// =============================================================================
// TOUCH HANDLING - FIXED
// =============================================================================

void handleBossRushMenuTouch(TouchGesture& gesture) {
    if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_DOWN) {
        if (show_boss_selection) {
            // Go back to tier menu from boss selection
            show_boss_selection = false;
            drawBossRushMenu();
            return;
        }
        returnToAppGrid();
        return;
    }

    // Scroll the tier list with swipe up/down on the tier menu
    if (!show_boss_selection && (gesture.event == TOUCH_SWIPE_UP)) {
        int max_scroll = max(0, BOSS_TIERS - 4);
        if (tier_scroll_offset < max_scroll) {
            tier_scroll_offset++;
            drawBossRushMenu();
        }
        return;
    }
    // Note: TOUCH_SWIPE_DOWN handled above to go back

    if (gesture.event != TOUCH_TAP) return;

    int x = gesture.x;
    int y = gesture.y;

    if (show_boss_selection) {
        // Boss selection screen - tap to start fight
        int tier_start = current_tier * 5;
        for (int i = 0; i < 5; i++) {
            int boss_idx = tier_start + i;
            if (boss_idx >= TOTAL_BOSSES) break;
            int by = 60 + i * 78;
            if (y >= by && y < by + 68) {
                startBossFight(boss_idx);
                return;
            }
        }
    } else {
        // Tier selection screen (scrollable)
        int startY = 105;
        int tier_item_h = 68;
        int tier_gap = 7;

        // Scroll buttons (top/bottom areas)
        if (y < startY && tier_scroll_offset > 0) {
            tier_scroll_offset--;
            drawBossRushMenu();
            return;
        }

        int bottomArea = startY + 4 * (tier_item_h + tier_gap);
        if (y > bottomArea && tier_scroll_offset < BOSS_TIERS - 4) {
            tier_scroll_offset++;
            drawBossRushMenu();
            return;
        }

        for (int v = 0; v < 4; v++) {
            int i = v + tier_scroll_offset;
            if (i >= BOSS_TIERS) break;
            int by = startY + v * (tier_item_h + tier_gap);
            if (y >= by && y < by + tier_item_h) {
                current_tier = i;
                drawBossSelection();
                return;
            }
        }
    }
}

void handleBossRushTouch(TouchGesture& gesture) {
    if (battle_active) {
        handleBossBattleTouch(gesture);
    } else if (!battle_active && system_state.current_screen == SCREEN_BOSS_RUSH) {
        // Victory/Defeat screen or menu
        if (gesture.event == TOUCH_TAP && gesture.y >= 340) {
            if (show_boss_selection) {
                drawBossSelection();
            } else {
                drawBossRushMenu();
            }
        } else {
            handleBossRushMenuTouch(gesture);
        }
    }
}

void handleBossSelectionTouch(TouchGesture& gesture) {
    handleBossRushMenuTouch(gesture);
}

void handleBossBattleTouch(TouchGesture& gesture) {
    if (gesture.event == TOUCH_SWIPE_UP && !battle_active) {
        drawBossRushMenu();
        return;
    }

    if (gesture.event != TOUCH_TAP) return;

    if (!battle_active) {
        if (gesture.y >= 340) {
            drawBossRushMenu();
        }
        return;
    }

    int x = gesture.x, y = gesture.y;

    // Action buttons at y = 320
    if (y >= 320 && y < 370) {
        int btnW = 75, gap = 10;
        int startX = (LCD_WIDTH - (4 * btnW + 3 * gap)) / 2;

        if (x >= startX && x < startX + btnW) {
            playerAttack();
        } else if (x >= startX + btnW + gap && x < startX + 2 * btnW + gap) {
            playerDefend();
        } else if (x >= startX + 2 * (btnW + gap) && x < startX + 3 * btnW + 2 * gap) {
            playerSpecial();
        } else if (x >= startX + 3 * (btnW + gap)) {
            playerUseItem();
        }
    }
}

const char* getBossTierName(BossTier tier) {
    const char* names[] = {"Tier 1", "Tier 2", "Tier 3", "Tier 4", "Tier 5", "Tier 6", "Tier 7"};
    int idx = (int)tier - 1;
    if (idx < 0 || idx >= 7) return "???";
    return names[idx];
}

uint16_t getBossTierColor(BossTier tier) {
    uint16_t colors[] = {COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_RED, RGB565(255,200,50), RGB565(255,100,255), RGB565(255,255,255)};
    int idx = (int)tier - 1;
    if (idx < 0 || idx >= 7) return COLOR_WHITE;
    return colors[idx];
}

