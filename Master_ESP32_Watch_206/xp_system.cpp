/*
 * xp_system.cpp - XP & Leveling System Implementation
 * FUSION OS RPG Integration
 *
 * UPDATED: Compact SD Save, Auto-Migration, Rebirth System,
 *          Rebirth Counter, Multi-Slot SD Card Backup
 *
 * Fixes NVS fragmentation: 280+ keys → 1 key (~85 bytes)
 */

#include "xp_system.h"
#include "display.h"
#include "themes.h"
#include "sd_preferences.h"
#include <nvs_flash.h>
#include <SD_MMC.h>
#include <FS.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

SDPreferences xp_prefs;

// =============================================================================
// REBIRTH & BACKUP GLOBAL STATE
// =============================================================================
RebirthState rebirth_state = REBIRTH_IDLE;
unsigned long rebirth_timeout = 0;
int selected_backup_slot = -1;
bool in_backup_list_view = false;

// =============================================================================
// XP SYSTEM STATE
// =============================================================================
XPSystemState xp_system = {
  .current_character = nullptr,
  .total_gems = 0,
  .last_login_day = -1,
  .last_hourly_claim_hour = -1,
  .daily_step_goal_claimed = false,
  .login_streak = 0,
  .last_streak_day = -1,
  .last_streak_month = -1,
  .longest_streak = 0
};

// =============================================================================
// CHARACTER TITLE DEFINITIONS
// =============================================================================

const char* luffy_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Rubber Rookie", "Straw Hat Fighter", "Gum-Gum Kid",
  "East Blue Champion", "Captain of Freedom", "Sky Island Drifter",
  "Gear Second Surger", "Grand Line Veteran", "Gear Third Brawler",
  "Gear Fourth Juggernaut", "Haki Awakener", "Conqueror's Vessel",
  "Emissary of Laughter", "Toon World Trickster", "Nika's Wrath",
  "Joy Boy's Heir", "Toon God Warrior", "Reality Cartoonist",
  "Haki Harmonizer", "Boundless Luffy", "Liberation Deity",
  "Sun God Nika"
};

const char* jinwoo_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Weakest Hunter", "E-Rank Grinder", "Dungeon Survivor",
  "System User", "Rank D Ascender", "C-Rank Climber",
  "Solo Dungeon Clearer", "Grave Raiser", "Shadow Initiate",
  "Shadow Commander", "B-Rank Hunter", "Red Gate Conqueror",
  "Rank B Executioner", "Dungeon Monarch", "Rank A Terminator",
  "Jeju Island Savior", "Shadow Monarch's Vessel", "Monarch of Shadows",
  "King of Death", "Ashborn's Successor", "Darkness Embodied",
  "Shadow Monarch"
};

const char* yugo_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Young Eliatrope", "Curious Explorer", "Brotherhood Member",
  "Wakfu Student", "Portal Initiate", "Warp Jumper",
  "Sadida Ally", "Dimensional Walker", "Shushu Whisperer",
  "Wakfu Manipulator", "Rubilax Wielder", "Adamai's Brother",
  "Portal Architect", "Temporal Traveler", "Prince of Eliatropes",
  "Inglorium Navigator", "Dimensional Weaver", "Keeper of Realities",
  "Cosmic Warper", "Eliatrope King", "King of Dragons",
  "Master of Portals"
};

const char* boboiboy_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Element Rookie", "Power Sphere Bearer", "Lightning Wielder",
  "Dual Element User", "Triple Split Master", "Earth Protector",
  "Wind Rider", "Fire Blazer", "Water Bender",
  "Leaf Guardian", "Light Emitter", "Tier 2 Awakened",
  "Thunderstorm Rager", "Cyclone Tornado", "Quake Shaker",
  "Blaze Inferno", "Ice Freezer", "Thorn Piercer",
  "Solar Eclipse", "Fusion Master", "Elemental God",
  "Kuasa Tujuh!"
};

const char* gojo_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Six Eyes Prodigy", "Limitless User", "Infinity's Wall",
  "Blue Energy", "Red Reversal", "Star Vessel Guard",
  "High School Strongest", "Toji's Nightmare", "Awakened Sorcerer",
  "Hollow Purple", "Infinite Void Master", "Special Grade Anomaly",
  "Untouchable God", "Shibuya's Ace", "Prison Realm Prisoner",
  "The Returning Legend", "Shinjuku Challenger", "Sukuna's Rival",
  "Absolute Zenith", "Throughout Heaven and Earth...", "The Honored One",
  "The Strongest Sorcerer"
};

const char* naruto_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Nine-Tails Jinchuriki", "Prankster of the Leaf", "Team 7 Genin",
  "Worst Ninja", "Toad Sage Apprentice", "Rasenshuriken Innovator",
  "Sage of Mount Myoboku", "Hero of the Leaf", "Kage-Level Contender",
  "Kyubi Chakra Mode", "Kurama Mode Link", "Ninja Way",
  "Allied Force Leader", "Six Paths Sage", "Asura's Reincarnation",
  "Child of Prophecy", "Saviour of the World", "Orange Hokage",
  "Baryon Mode", "Will Of Fire", "The 7th Hokage",
  "The Strongest Shinobi"
};

const char* goku_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Low-Class Warrior", "Turtle School Student", "World Martial Artist",
  "Kaio-ken Master", "Legendary Super Saiyan", "Ascended Saiyan",
  "Super Saiyan 2", "Super Saiyan 3", "Fusion Component",
  "Super Saiyan God", "Super Saiyan Blue", "Blue Kaio-ken x20",
  "Universe 7 Ace", "Ultra Instinct Sign", "Silver-Eyed Master",
  "Mastered Ultra Instinct", "Giant Ki Avatar", "True Ultra Instinct",
  "Beyond the Gods", "Multiverse Apex", "Kakarot",
  "Mightiest in the Heavens"
};

const char* saitama_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Salaryman", "100 Pushups Daily", "Bald Aspirant",
  "The Bald Cape", "Class C Rookie", "Ghost of City Z",
  "Class B Leader", "Normal Puncher", "Deep Sea King Slayer",
  "Class A Hero", "Serious Punch", "Serious Table Flip",
  "Meteor Shatterer", "Monster Nightmare", "Serious Sneeze",
  "Zero Punch Paradox", "Limiter Breaker", "Absolute Strength",
  "The Bored God", "S-Class Secret", "One Punch Man",
  "Strongest in Existence"
};

const char* tanjiro_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Water Breathing Learner", "Final Selection Survivor", "Black Blade Wielder",
  "Drum House Victor", "Lower Moon Assassin", "Hinokami Kagura User",
  "Sun Breath Successor", "Constant Concentration", "Upper Moon Executioner",
  "Slayer Mark Awakened", "Sun Halo Dragon", "Hashira Apprentice",
  "See-Through World", "Selfless State Master", "Hanafuda Inheritor",
  "The 13th Form", "Muzan's Nemesis", "Demon King (Shadow)",
  "Dawn Bringer", "Last Breath", "Sun God's Avatar",
  "Demon Slayer"
};

const char* levi_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Underground Legend", "Thieving Acrobat", "Scout Recruit",
  "ODM Prodigy", "Blade Master", "Ackerman Instinct",
  "Spinning Whirlwind", "Squad Leader", "Female Titan's Bane",
  "Beast Titan Shredder", "Wall Maria Hero", "Zeke's Nightmare",
  "Thunder Spear Expert", "Unkillable Captain", "Last Ackerman",
  "Crimson Blade", "Humanity's Vengeance", "Fulfilled Promise",
  "Fallen Wings Legacy", "Last Sentinel", "Humanity's Strongest",
  "Greatest Soldier in History"
};

const char* deku_title_names[MAX_TITLES_PER_CHARACTER] = {
  "Quirkless Kid", "All Might's Successor", "One For All 5%",
  "Full Cowl Initiate", "Shoot Style User", "Stain's Equal",
  "One For All 20%", "Blackwhip Awakened", "Float Master",
  "Danger Sense Instinct", "Smokescreen Strategist", "Fa Jin Kineticist",
  "One For All 45%", "Dark Deku (Vigilante)", "Gearshift Overdrive",
  "Transmission Master", "Symbol of Peace", "One For All 100%",
  "Bane of All For One", "The World's Hero", "The Symbol of Hope",
  "The Greatest Hero"
};

// Character XP data structures
CharacterXPData luffy_xp = {THEME_LUFFY_GEAR5, 1, 0, 100, 0};
CharacterXPData jinwoo_xp = {THEME_SUNG_JINWOO, 1, 0, 100, 0};
CharacterXPData yugo_xp = {THEME_YUGO_WAKFU, 1, 0, 100, 0};
CharacterXPData boboiboy_xp = {THEME_BOBOIBOY, 1, 0, 100, 0};
CharacterXPData gojo_xp = {THEME_GOJO_INFINITY, 1, 0, 100, 0};
CharacterXPData naruto_xp = {THEME_NARUTO_SAGE, 1, 0, 100, 0};
CharacterXPData goku_xp = {THEME_GOKU_UI, 1, 0, 100, 0};
CharacterXPData saitama_xp = {THEME_SAITAMA_OPM, 1, 0, 100, 0};
CharacterXPData tanjiro_xp = {THEME_TANJIRO_SUN, 1, 0, 100, 0};
CharacterXPData levi_xp = {THEME_LEVI_STRONGEST, 1, 0, 100, 0};
CharacterXPData deku_xp = {THEME_DEKU_PLUSULTRA, 1, 0, 100, 0};

// Character array for indexed access
CharacterXPData* all_characters[11] = {
  &luffy_xp, &jinwoo_xp, &yugo_xp, &boboiboy_xp,
  &gojo_xp, &naruto_xp, &goku_xp, &saitama_xp,
  &tanjiro_xp, &levi_xp, &deku_xp
};

// =============================================================================
// INITIALIZE XP SYSTEM
// =============================================================================
void initXPSystem() {
  const char** title_arrays[11] = {
    luffy_title_names, jinwoo_title_names, yugo_title_names, boboiboy_title_names,
    gojo_title_names, naruto_title_names, goku_title_names,
    saitama_title_names, tanjiro_title_names, levi_title_names, deku_title_names
  };

  int level_reqs[MAX_TITLES_PER_CHARACTER] = {
    1, 5, 10, 15, 16, 20, 25, 30, 31, 35, 40, 45, 50, 51,
    60, 65, 70, 71, 80, 85, 90, 95
  };

  for (int char_idx = 0; char_idx < 11; char_idx++) {
    CharacterXPData* char_data = all_characters[char_idx];
    for (int i = 0; i < MAX_TITLES_PER_CHARACTER; i++) {
      char_data->titles[i].name = title_arrays[char_idx][i];
      char_data->titles[i].level_required = level_reqs[i];
      char_data->titles[i].unlocked = false;
    }
  }

  loadXPData();
  switchCharacter(system_state.current_theme);

  Serial.println("[XP] XP System initialized (Compact SD)");
  if (xp_system.current_character) {
    Serial.printf("     Current: Lv%d, %ld/%ld XP, Title: %s\n",
                  xp_system.current_character->level,
                  xp_system.current_character->xp,
                  xp_system.current_character->xp_to_next_level,
                  getEquippedTitle());
  }
  Serial.printf("     Rebirth count: %d\n", getRebirthCount());
}

// =============================================================================
// CALCULATE XP FOR LEVEL
// =============================================================================
long calculateXPForLevel(int level) {
  if (level <= 1) return 0;
  return (long)(BASE_XP_PER_LEVEL * pow(XP_SCALING_FACTOR, level - 2));
}

// =============================================================================
// GET CURRENT CHARACTER XP
// =============================================================================
CharacterXPData* getCurrentCharacterXP() {
  return xp_system.current_character;
}

// =============================================================================
// SWITCH CHARACTER
// =============================================================================
void switchCharacter(ThemeType theme) {
  switch (theme) {
    case THEME_LUFFY_GEAR5:       xp_system.current_character = &luffy_xp; break;
    case THEME_SUNG_JINWOO:       xp_system.current_character = &jinwoo_xp; break;
    case THEME_YUGO_WAKFU:        xp_system.current_character = &yugo_xp; break;
    case THEME_BOBOIBOY:          xp_system.current_character = &boboiboy_xp; break;
    case THEME_GOJO_INFINITY:     xp_system.current_character = &gojo_xp; break;
    case THEME_NARUTO_SAGE:       xp_system.current_character = &naruto_xp; break;
    case THEME_GOKU_UI:           xp_system.current_character = &goku_xp; break;
    case THEME_SAITAMA_OPM:       xp_system.current_character = &saitama_xp; break;
    case THEME_TANJIRO_SUN:       xp_system.current_character = &tanjiro_xp; break;
    case THEME_LEVI_STRONGEST:    xp_system.current_character = &levi_xp; break;
    case THEME_DEKU_PLUSULTRA:    xp_system.current_character = &deku_xp; break;
    default:                      xp_system.current_character = &luffy_xp; break;
  }
  Serial.printf("[XP] Switched to character (theme %d)\n", theme);
}

// =============================================================================
// GAIN EXPERIENCE
// =============================================================================
void gainExperience(int amount, const char* source) {
  if (!xp_system.current_character) return;

  CharacterXPData* char_data = xp_system.current_character;

  char_data->xp += amount;
  Serial.printf("[XP] +%d XP from %s (Total: %ld/%ld)\n",
                amount, source, char_data->xp, char_data->xp_to_next_level);

  while (char_data->xp >= char_data->xp_to_next_level) {
    char_data->xp -= char_data->xp_to_next_level;
    char_data->level++;
    char_data->xp_to_next_level = calculateXPForLevel(char_data->level + 1);
    handleLevelUp(char_data->level);
  }

  system_state.player_level = char_data->level;
  system_state.player_xp = char_data->xp;

  saveXPData();

  extern SystemState system_state;
  if (system_state.current_screen == SCREEN_CHARACTER_STATS) {
    extern void drawCharacterStatsScreen();
    drawCharacterStatsScreen();
  }
}

// =============================================================================
// HANDLE LEVEL UP
// =============================================================================
void handleLevelUp(int new_level) {
  Serial.printf("[XP] LEVEL UP! -> Level %d\n", new_level);

  CharacterXPData* char_data = xp_system.current_character;

  if (new_level <= MAX_TITLE_LEVEL) {
    for (int i = 0; i < MAX_TITLES_PER_CHARACTER; i++) {
      if (!char_data->titles[i].unlocked &&
          new_level >= char_data->titles[i].level_required) {
        char_data->titles[i].unlocked = true;
        Serial.printf("[XP] Title unlocked: \"%s\" (Lv%d)\n",
                      char_data->titles[i].name,
                      char_data->titles[i].level_required);
      }
    }
  }
}

// =============================================================================
// GET EQUIPPED TITLE
// =============================================================================
const char* getEquippedTitle() {
  if (!xp_system.current_character) return "None";

  CharacterXPData* char_data = xp_system.current_character;
  int title_idx = char_data->equipped_title_index;

  if (title_idx >= 0 && title_idx < MAX_TITLES_PER_CHARACTER &&
      char_data->titles[title_idx].unlocked) {
    return char_data->titles[title_idx].name;
  }
  return "No Title";
}

// =============================================================================
// EQUIP TITLE
// =============================================================================
void equipTitle(int title_index) {
  if (!xp_system.current_character) return;

  CharacterXPData* char_data = xp_system.current_character;

  if (title_index >= 0 && title_index < MAX_TITLES_PER_CHARACTER &&
      char_data->titles[title_index].unlocked) {
    char_data->equipped_title_index = title_index;
    Serial.printf("[XP] Equipped title: \"%s\"\n", char_data->titles[title_index].name);
    saveXPData();
  }
}

// =============================================================================
// CHECK DAILY LOGIN BONUS WITH STREAK SYSTEM
// =============================================================================
void checkDailyLoginBonus() {
  WatchTime current = getCurrentTime();
  int current_day = current.day;
  int current_month = current.month;

  if (current_day != xp_system.last_login_day) {
    xp_system.last_login_day = current_day;

    bool is_consecutive = false;
    if (xp_system.last_streak_day >= 0) {
      int expected_yesterday = xp_system.last_streak_day + 1;
      if (current_month == xp_system.last_streak_month) {
        is_consecutive = (current_day == expected_yesterday);
      } else if (current_day == 1) {
        is_consecutive = (xp_system.last_streak_day >= 28);
      }
    }

    if (is_consecutive) {
      xp_system.login_streak++;
    } else if (xp_system.last_streak_day < 0) {
      xp_system.login_streak = 1;
    } else {
      xp_system.login_streak = 1;
      Serial.println("[STREAK] Streak broken! Restarting at Day 1");
    }

    if (xp_system.login_streak > xp_system.longest_streak) {
      xp_system.longest_streak = xp_system.login_streak;
    }

    xp_system.last_streak_day = current_day;
    xp_system.last_streak_month = current_month;

    int streak = xp_system.login_streak;
    int gem_reward = 0;
    int xp_reward = XP_DAILY_LOGIN;

    if (streak >= 30 && (streak % 30 == 0)) {
      gem_reward = 2000;
      xp_reward = 1000;
    } else if (streak >= 14 && (streak % 14 == 0)) {
      gem_reward = 1000;
      xp_reward = 500;
    } else if (streak >= 7 && (streak % 7 == 0)) {
      gem_reward = 500;
      xp_reward = 200;
    } else {
      gem_reward = min(50 + (streak - 1) * 10, 100);
    }

    if (gem_reward > 0) {
      system_state.player_gems += gem_reward;
    }
    gainExperience(xp_reward, "Daily Login");
    system_state.daily_login_count++;

    saveXPData();
  }
}

// =============================================================================
// CHECK HOURLY SHOP CLAIM
// =============================================================================
void checkHourlyShopClaim() {
  WatchTime current = getCurrentTime();
  int current_hour = current.hour;

  if (current_hour != xp_system.last_hourly_claim_hour) {
    xp_system.last_hourly_claim_hour = current_hour;
    gainExperience(XP_HOURLY_SHOP, "Hourly Shop");
  }
}

// =============================================================================
// GET XP PROGRESS (0.0 to 1.0)
// =============================================================================
float getXPProgress() {
  if (!xp_system.current_character) return 0.0f;

  CharacterXPData* char_data = xp_system.current_character;
  if (char_data->xp_to_next_level == 0) return 1.0f;

  return (float)char_data->xp / (float)char_data->xp_to_next_level;
}

// =============================================================================
// SAVE XP DATA - NEW COMPACT FORMAT (Single NVS key, ~85 bytes)
// =============================================================================
void saveXPData() {
  SDPreferences prefs;
  prefs.begin("xp", false);

  CompactXPSave save;
  memset(&save, 0, sizeof(save));
  save.version = 1;

  // Pack all character data into compact struct
  for (int i = 0; i < 11; i++) {
    save.levels[i] = (uint16_t)all_characters[i]->level;
    save.xp[i] = (uint32_t)all_characters[i]->xp;
    save.titles[i] = (int8_t)all_characters[i]->equipped_title_index;
  }

  save.gems = (uint16_t)min((int)system_state.player_gems, 65535);
  save.streak = (uint8_t)min(xp_system.login_streak, 255);
  save.last_day = (uint8_t)((xp_system.last_login_day < 0) ? 0 : xp_system.last_login_day);
  save.streak_day = (uint8_t)((xp_system.last_streak_day < 0) ? 0 : xp_system.last_streak_day);
  save.streak_month = (uint8_t)((xp_system.last_streak_month < 0) ? 0 : xp_system.last_streak_month);
  save.longest_streak = (uint8_t)min(xp_system.longest_streak, 255);

  // Single write operation - 85 bytes!
  prefs.putBytes("d", &save, sizeof(save));

  // Also save hourly claim hour (not in compact struct as it's volatile)
  prefs.putInt("last_hr", xp_system.last_hourly_claim_hour);

  prefs.end();

  // Sync gems with system_state
  xp_system.total_gems = system_state.player_gems;

  Serial.printf("[XP] Saved compact data (%d bytes)\n", sizeof(save));
}

// =============================================================================
// LOAD XP DATA - WITH AUTO-MIGRATION FROM OLD FORMAT
// =============================================================================
void loadXPData() {
  SDPreferences prefs;
  prefs.begin("xp", true);  // read-only

  // Try to load new compact format first
  CompactXPSave save;
  size_t loaded = prefs.getBytes("d", &save, sizeof(save));

  if (loaded == sizeof(save) && save.version == 1) {
    // NEW FORMAT - Unpack compact data
    Serial.println("[XP] Loading compact SD format");

    for (int i = 0; i < 11; i++) {
      all_characters[i]->level = save.levels[i];
      all_characters[i]->xp = save.xp[i];
      all_characters[i]->equipped_title_index = save.titles[i];

      if (all_characters[i]->equipped_title_index < 0 ||
          all_characters[i]->equipped_title_index >= MAX_TITLES_PER_CHARACTER) {
        all_characters[i]->equipped_title_index = 0;
      }

      all_characters[i]->xp_to_next_level = calculateXPForLevel(all_characters[i]->level + 1);

      // Unlock titles based on level
      for (int j = 0; j < MAX_TITLES_PER_CHARACTER; j++) {
        if (all_characters[i]->level >= all_characters[i]->titles[j].level_required) {
          all_characters[i]->titles[j].unlocked = true;
        }
      }
    }

    system_state.player_gems = save.gems;
    xp_system.total_gems = save.gems;
    xp_system.login_streak = save.streak;
    xp_system.last_login_day = (save.last_day == 0) ? -1 : save.last_day;
    xp_system.last_streak_day = (save.streak_day == 0) ? -1 : save.streak_day;
    xp_system.last_streak_month = (save.streak_month == 0) ? -1 : save.streak_month;
    xp_system.longest_streak = save.longest_streak;
    xp_system.last_hourly_claim_hour = prefs.getInt("last_hr", -1);

    prefs.end();
    Serial.println("[XP] Compact data loaded successfully");

  } else {
    // OLD FORMAT - Auto-migrate from individual keys
    Serial.println("[XP] Old format detected - auto-migrating...");

    const char* prefixes[11] = {"lf", "jw", "yg", "bb", "gj", "nr", "gk", "st", "tj", "lv", "dk"};

    for (int i = 0; i < 11; i++) {
      char key[16];
      sprintf(key, "%s_lv", prefixes[i]);
      all_characters[i]->level = prefs.getInt(key, 1);

      sprintf(key, "%s_xp", prefixes[i]);
      all_characters[i]->xp = prefs.getLong(key, 0);

      sprintf(key, "%s_ttl", prefixes[i]);
      all_characters[i]->equipped_title_index = prefs.getInt(key, 0);

      if (all_characters[i]->equipped_title_index < 0 ||
          all_characters[i]->equipped_title_index >= MAX_TITLES_PER_CHARACTER) {
        all_characters[i]->equipped_title_index = 0;
      }

      all_characters[i]->xp_to_next_level = calculateXPForLevel(all_characters[i]->level + 1);

      for (int j = 0; j < MAX_TITLES_PER_CHARACTER; j++) {
        if (all_characters[i]->level >= all_characters[i]->titles[j].level_required) {
          all_characters[i]->titles[j].unlocked = true;
        }
      }
    }

    xp_system.total_gems = prefs.getInt("gems", system_state.player_gems);
    xp_system.last_login_day = prefs.getInt("last_day", -1);
    xp_system.last_hourly_claim_hour = prefs.getInt("last_hr", -1);
    xp_system.login_streak = prefs.getInt("streak", 0);
    xp_system.last_streak_day = prefs.getInt("strk_day", -1);
    xp_system.last_streak_month = prefs.getInt("strk_mon", -1);
    xp_system.longest_streak = prefs.getInt("strk_max", 0);

    system_state.player_gems = xp_system.total_gems;

    prefs.end();

    // Now save in new compact format (migration!)
    saveXPData();
    Serial.println("[XP] Migration complete! Old data converted to compact format.");
  }

  system_state.player_level = xp_system.current_character ? xp_system.current_character->level : 1;
  system_state.player_xp = xp_system.current_character ? xp_system.current_character->xp : 0;
}

// =============================================================================
// REBIRTH COUNTER - Stored in separate "rebirth" namespace (survives rebirth!)
// =============================================================================

int getRebirthCount() {
  SDPreferences rebirthPrefs;
  rebirthPrefs.begin("rebirth", true);  // read-only
  int count = rebirthPrefs.getInt("count", 0);
  rebirthPrefs.end();
  return count;
}

void incrementRebirthCount() {
  SDPreferences rebirthPrefs;
  rebirthPrefs.begin("rebirth", false);  // read-write
  int count = rebirthPrefs.getInt("count", 0);
  rebirthPrefs.putInt("count", count + 1);
  rebirthPrefs.end();
  Serial.printf("[REBIRTH] Counter incremented to %d\n", count + 1);
}

// =============================================================================
// REBIRTH SYSTEM - UI FUNCTIONS
// =============================================================================

void showRebirthButton() {
  ThemeColors* theme = getCurrentTheme();
  int rebirth_count = getRebirthCount();

  int btnY = LCD_HEIGHT - 70;
  int btnX = 30;
  int btnW = LCD_WIDTH - 60;
  int btnH = 42;

  // Dark red background
  gfx->fillRect(btnX, btnY, btnW, btnH, RGB565(40, 10, 10));
  gfx->drawRect(btnX, btnY, btnW, btnH, RGB565(200, 60, 60));

  // Corner accents
  gfx->fillRect(btnX, btnY, 5, 5, RGB565(200, 60, 60));
  gfx->fillRect(btnX + btnW - 5, btnY, 5, 5, RGB565(200, 60, 60));
  gfx->fillRect(btnX, btnY + btnH - 5, 5, 5, RGB565(200, 60, 60));
  gfx->fillRect(btnX + btnW - 5, btnY + btnH - 5, 5, 5, RGB565(200, 60, 60));

  // Left side accent bar
  gfx->fillRect(btnX + 2, btnY + 2, 4, btnH - 4, RGB565(200, 60, 60));

  // Text
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setTextSize(2);
  gfx->setCursor(btnX + 20, btnY + 5);
  gfx->print("REBIRTH");

  // Rebirth count
  if (rebirth_count > 0) {
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(180, 80, 80));
    gfx->setCursor(btnX + 20, btnY + 27);
    gfx->printf("Rebirths: #%d", rebirth_count);
  } else {
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(120, 60, 60));
    gfx->setCursor(btnX + 20, btnY + 27);
    gfx->print("Reset all progress");
  }

  // Arrow on right side
  gfx->setTextColor(RGB565(150, 50, 50));
  gfx->setTextSize(2);
  gfx->setCursor(btnX + btnW - 30, btnY + 12);
  gfx->print(">");
}

void showRebirthConfirm1() {
  rebirth_state = REBIRTH_CONFIRM_1;
  rebirth_timeout = millis() + 10000;  // 10 second auto-cancel

  gfx->fillScreen(RGB565(5, 0, 0));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(10, 2, 2));
  }

  int centerX = LCD_WIDTH / 2;

  // Warning header
  gfx->fillRect(0, 0, LCD_WIDTH, 55, RGB565(60, 10, 10));
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setCursor(centerX - 100, 14);
  gfx->print("ARE YOU SURE?");

  // Warning content
  int contentY = 75;
  gfx->fillRect(20, contentY, LCD_WIDTH - 40, 200, RGB565(15, 5, 5));
  gfx->drawRect(20, contentY, LCD_WIDTH - 40, 200, RGB565(200, 60, 60));

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(255, 100, 100));
  gfx->setCursor(40, contentY + 15);
  gfx->print("This will DELETE:");

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(200, 150, 150));
  gfx->setCursor(40, contentY + 50);
  gfx->print("All character levels");
  gfx->setCursor(40, contentY + 75);
  gfx->print("All XP progress");
  gfx->setCursor(40, contentY + 100);
  gfx->print("Gems and items");
  gfx->setCursor(40, contentY + 125);
  gfx->print("Login streaks");

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 100, 100));
  gfx->setCursor(40, contentY + 160);
  gfx->print("TIP: Save to SD card first!");

  // Buttons
  int btnY = contentY + 220;
  int btnW = 160;
  int btnH = 50;

  // YES button
  gfx->fillRect(30, btnY, btnW, btnH, RGB565(80, 15, 15));
  gfx->drawRect(30, btnY, btnW, btnH, RGB565(255, 80, 80));
  gfx->fillRect(30, btnY, 5, 5, RGB565(255, 80, 80));
  gfx->setTextColor(RGB565(255, 100, 100));
  gfx->setTextSize(2);
  gfx->setCursor(55, btnY + 16);
  gfx->print("YES, DELETE");

  // NO button
  int noBtnX = LCD_WIDTH - 30 - btnW;
  gfx->fillRect(noBtnX, btnY, btnW, btnH, RGB565(15, 30, 15));
  gfx->drawRect(noBtnX, btnY, btnW, btnH, RGB565(80, 200, 80));
  gfx->fillRect(noBtnX + btnW - 5, btnY, 5, 5, RGB565(80, 200, 80));
  gfx->setTextColor(RGB565(80, 200, 80));
  gfx->setCursor(noBtnX + 25, btnY + 16);
  gfx->print("NO, CANCEL");

  // Auto-cancel timer
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 60, 60));
  gfx->setCursor(centerX - 60, btnY + 65);
  gfx->print("Auto-cancel in 10s");
}

void showRebirthConfirm2() {
  rebirth_state = REBIRTH_CONFIRM_2;

  gfx->fillScreen(RGB565(10, 0, 0));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(15, 3, 3));
  }

  int centerX = LCD_WIDTH / 2;

  // FINAL WARNING header
  gfx->fillRect(0, 0, LCD_WIDTH, 55, RGB565(80, 10, 10));
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(255, 50, 50));
  gfx->setCursor(centerX - 120, 14);
  gfx->print("FINAL WARNING!");

  // Stats display
  int statsY = 70;
  gfx->fillRect(20, statsY, LCD_WIDTH - 40, 180, RGB565(20, 5, 5));
  gfx->drawRect(20, statsY, LCD_WIDTH - 40, 180, RGB565(255, 60, 60));

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(255, 200, 200));
  gfx->setCursor(40, statsY + 10);
  gfx->print("REALLY?!");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(200, 100, 100));
  gfx->setCursor(40, statsY + 35);
  gfx->print("There is NO going back!");

  // Show current stats
  int total_levels = 0;
  for (int i = 0; i < 11; i++) {
    total_levels += all_characters[i]->level;
  }

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(200, 180, 180));
  gfx->setCursor(40, statsY + 60);
  gfx->printf("Total Levels: %d", total_levels);
  gfx->setCursor(40, statsY + 85);
  gfx->printf("Total Gems: %d", system_state.player_gems);
  gfx->setCursor(40, statsY + 110);
  gfx->printf("Login Streak: %d days", xp_system.login_streak);

  int rebirth_count = getRebirthCount();
  gfx->setCursor(40, statsY + 135);
  gfx->printf("Rebirths: %d", rebirth_count);

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(200, 200, 100));
  gfx->setCursor(LCD_WIDTH - 160, statsY + 155);
  gfx->print("TIP: Save to SD first!");

  // Buttons
  int btnY = statsY + 200;
  int btnW = 160;
  int btnH = 50;

  // CONFIRM DELETE
  gfx->fillRect(30, btnY, btnW, btnH, RGB565(120, 15, 15));
  gfx->drawRect(30, btnY, btnW, btnH, RGB565(255, 50, 50));
  gfx->fillRect(30, btnY, 5, 5, RGB565(255, 50, 50));
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setTextSize(2);
  gfx->setCursor(40, btnY + 16);
  gfx->print("CONFIRM DEL");

  // CANCEL
  int cancelX = LCD_WIDTH - 30 - btnW;
  gfx->fillRect(cancelX, btnY, btnW, btnH, RGB565(15, 40, 15));
  gfx->drawRect(cancelX, btnY, btnW, btnH, RGB565(80, 255, 80));
  gfx->fillRect(cancelX + btnW - 5, btnY, 5, 5, RGB565(80, 255, 80));
  gfx->setTextColor(RGB565(80, 255, 80));
  gfx->setCursor(cancelX + 35, btnY + 16);
  gfx->print("CANCEL");
}

void executeRebirth() {
  rebirth_state = REBIRTH_COMPLETE;
  int centerX = LCD_WIDTH / 2;

  // Increment rebirth counter BEFORE clearing (survives!)
  incrementRebirthCount();
  int rebirth_num = getRebirthCount();

  // Show rebirth animation
  gfx->fillScreen(RGB565(0, 0, 0));
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setCursor(centerX - 90, 80);
  gfx->printf("REBIRTH #%d", rebirth_num);

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(200, 150, 150));
  gfx->setCursor(centerX - 70, 130);
  gfx->print("Resetting...");

  // Progress bar animation
  int barX = 40;
  int barY = 200;
  int barW = LCD_WIDTH - 80;
  int barH = 30;

  gfx->drawRect(barX, barY, barW, barH, RGB565(200, 60, 60));

  for (int progress = 0; progress <= 100; progress += 2) {
    int fillW = (barW - 4) * progress / 100;
    gfx->fillRect(barX + 2, barY + 2, fillW, barH - 4, RGB565(200, 60, 60));

    gfx->fillRect(centerX - 30, barY + 40, 60, 25, RGB565(0, 0, 0));
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(200, 150, 150));
    gfx->setCursor(centerX - 25, barY + 45);
    gfx->printf("%d%%", progress);

    delay(20);
  }

  // Clear main data namespaces (NOT "rebirth" namespace!)
  const char* namespaces_to_clear[] = {"xp", "game", "config", "stats", "achievements"};
  for (int i = 0; i < 5; i++) {
    SDPreferences p;
    p.begin(namespaces_to_clear[i], false);
    p.clear();
    p.end();
    Serial.printf("[REBIRTH] Cleared namespace: %s\n", namespaces_to_clear[i]);
  }

  // Show completion
  gfx->fillScreen(RGB565(0, 0, 0));
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(255, 80, 80));
  gfx->setCursor(centerX - 120, LCD_HEIGHT / 2 - 40);
  gfx->printf("REBIRTH #%d", rebirth_num);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(200, 200, 200));
  gfx->setCursor(centerX - 70, LCD_HEIGHT / 2 + 10);
  gfx->print("COMPLETE!");

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 150));
  gfx->setCursor(centerX - 55, LCD_HEIGHT / 2 + 50);
  gfx->print("Restarting...");

  delay(2000);

  Serial.printf("[REBIRTH] Rebirth #%d complete. Restarting...\n", rebirth_num);
  ESP.restart();
}

void handleRebirthTouch(int x, int y) {
  int centerX = LCD_WIDTH / 2;

  switch (rebirth_state) {
    case REBIRTH_IDLE: {
      // Rebirth button was tapped - show first confirmation
      showRebirthConfirm1();
      break;
    }

    case REBIRTH_CONFIRM_1: {
      // Check for auto-cancel timeout
      if (millis() > rebirth_timeout) {
        cancelRebirth();
        return;
      }

      int btnY = 295;  // contentY(75) + 220
      int btnW = 160;
      int btnH = 50;

      // YES button
      if (x >= 30 && x < 30 + btnW && y >= btnY && y < btnY + btnH) {
        showRebirthConfirm2();
        return;
      }

      // NO button
      int noBtnX = LCD_WIDTH - 30 - btnW;
      if (x >= noBtnX && x < noBtnX + btnW && y >= btnY && y < btnY + btnH) {
        cancelRebirth();
        return;
      }
      break;
    }

    case REBIRTH_CONFIRM_2: {
      int btnY = 270;  // statsY(70) + 200
      int btnW = 160;
      int btnH = 50;

      // CONFIRM DELETE
      if (x >= 30 && x < 30 + btnW && y >= btnY && y < btnY + btnH) {
        executeRebirth();
        return;
      }

      // CANCEL
      int cancelX = LCD_WIDTH - 30 - btnW;
      if (x >= cancelX && x < cancelX + btnW && y >= btnY && y < btnY + btnH) {
        cancelRebirth();
        return;
      }
      break;
    }

    case REBIRTH_COMPLETE:
      // Do nothing, reboot in progress
      break;
  }
}

void cancelRebirth() {
  rebirth_state = REBIRTH_IDLE;
  // Redraw settings screen
  extern void drawSettingsApp();
  drawSettingsApp();
}

// =============================================================================
// SD CARD BACKUP - Calculate checksum
// =============================================================================
static uint32_t calculateChecksum(const SDBackupData* data) {
  uint32_t sum = 0;
  const uint8_t* bytes = (const uint8_t*)data;
  // Checksum everything except the checksum field itself (last 4 bytes)
  size_t len = sizeof(SDBackupData) - sizeof(uint32_t);
  for (size_t i = 0; i < len; i++) {
    sum += bytes[i];
    sum = (sum << 3) | (sum >> 29);  // Rotate
  }
  return sum;
}

// =============================================================================
// SD CARD BACKUP - Save to slot
// =============================================================================
bool saveStatsToSD(int slot) {
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    Serial.println("[BACKUP] SD card not available!");
    return false;
  }

  if (slot < 1 || slot > MAX_BACKUPS) {
    Serial.printf("[BACKUP] Invalid slot: %d\n", slot);
    return false;
  }

  // Ensure backup folder exists
  if (!SD_MMC.exists(BACKUP_FOLDER)) {
    SD_MMC.mkdir(BACKUP_FOLDER);
  }

  // Build file path
  char filepath[48];
  sprintf(filepath, "%s/%s%d%s", BACKUP_FOLDER, BACKUP_PREFIX, slot, BACKUP_EXT);

  // Build backup data
  SDBackupData backup;
  memset(&backup, 0, sizeof(backup));
  backup.magic = BACKUP_MAGIC;
  backup.version = 1;
  backup.timestamp = millis();  // Simple timestamp (could use RTC)

  // Pack current XP data
  CompactXPSave* xp = &backup.xp_data;
  xp->version = 1;
  for (int i = 0; i < 11; i++) {
    xp->levels[i] = (uint16_t)all_characters[i]->level;
    xp->xp[i] = (uint32_t)all_characters[i]->xp;
    xp->titles[i] = (int8_t)all_characters[i]->equipped_title_index;
  }
  xp->gems = (uint16_t)min((int)system_state.player_gems, 65535);
  xp->streak = (uint8_t)min(xp_system.login_streak, 255);
  xp->last_day = (uint8_t)((xp_system.last_login_day < 0) ? 0 : xp_system.last_login_day);
  xp->streak_day = (uint8_t)((xp_system.last_streak_day < 0) ? 0 : xp_system.last_streak_day);
  xp->streak_month = (uint8_t)((xp_system.last_streak_month < 0) ? 0 : xp_system.last_streak_month);
  xp->longest_streak = (uint8_t)min(xp_system.longest_streak, 255);

  // Extra stats
  backup.total_steps = system_state.steps_today;
  backup.bosses_defeated = system_state.bosses_defeated;
  backup.cards_collected = system_state.gacha_cards_collected;
  backup.rebirth_count = (uint16_t)getRebirthCount();

  // Calculate total levels for preview
  uint16_t total = 0;
  for (int i = 0; i < 11; i++) total += xp->levels[i];
  backup.total_levels = total;

  // Calculate checksum
  backup.checksum = calculateChecksum(&backup);

  // Write to SD
  File file = SD_MMC.open(filepath, FILE_WRITE);
  if (!file) {
    Serial.printf("[BACKUP] Failed to open %s for writing!\n", filepath);
    return false;
  }

  size_t written = file.write((uint8_t*)&backup, sizeof(backup));
  file.close();

  if (written == sizeof(backup)) {
    Serial.printf("[BACKUP] Saved to Backup #%d (%s) - %d bytes\n", slot, filepath, written);
    return true;
  } else {
    Serial.printf("[BACKUP] Write failed! Expected %d, wrote %d\n", sizeof(backup), written);
    return false;
  }
}

// =============================================================================
// SD CARD BACKUP - Load from slot
// =============================================================================
bool loadStatsFromSD(int slot) {
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    Serial.println("[BACKUP] SD card not available!");
    return false;
  }

  char filepath[48];
  sprintf(filepath, "%s/%s%d%s", BACKUP_FOLDER, BACKUP_PREFIX, slot, BACKUP_EXT);

  File file = SD_MMC.open(filepath, FILE_READ);
  if (!file) {
    Serial.printf("[BACKUP] Failed to open %s!\n", filepath);
    return false;
  }

  SDBackupData backup;
  size_t read_bytes = file.read((uint8_t*)&backup, sizeof(backup));
  file.close();

  if (read_bytes != sizeof(backup)) {
    Serial.printf("[BACKUP] Read failed! Expected %d, got %d\n", sizeof(backup), read_bytes);
    return false;
  }

  // Validate magic number
  if (backup.magic != BACKUP_MAGIC) {
    Serial.println("[BACKUP] Invalid file format (bad magic)!");
    return false;
  }

  // Validate checksum
  uint32_t expected = calculateChecksum(&backup);
  if (backup.checksum != expected) {
    Serial.println("[BACKUP] Checksum mismatch - file may be corrupted!");
    return false;
  }

  // Unpack data into characters
  CompactXPSave* xp = &backup.xp_data;
  for (int i = 0; i < 11; i++) {
    all_characters[i]->level = xp->levels[i];
    all_characters[i]->xp = xp->xp[i];
    all_characters[i]->equipped_title_index = xp->titles[i];
    all_characters[i]->xp_to_next_level = calculateXPForLevel(all_characters[i]->level + 1);

    for (int j = 0; j < MAX_TITLES_PER_CHARACTER; j++) {
      all_characters[i]->titles[j].unlocked =
        (all_characters[i]->level >= all_characters[i]->titles[j].level_required);
    }
  }

  system_state.player_gems = xp->gems;
  xp_system.total_gems = xp->gems;
  xp_system.login_streak = xp->streak;
  xp_system.last_login_day = (xp->last_day == 0) ? -1 : xp->last_day;
  xp_system.last_streak_day = (xp->streak_day == 0) ? -1 : xp->streak_day;
  xp_system.last_streak_month = (xp->streak_month == 0) ? -1 : xp->streak_month;
  xp_system.longest_streak = xp->longest_streak;

  // Save to NVS so it persists
  saveXPData();

  Serial.printf("[BACKUP] Loaded Backup #%d successfully! Total levels: %d\n",
                slot, backup.total_levels);
  return true;
}

// =============================================================================
// SD CARD BACKUP - Get next available slot
// =============================================================================
int getNextBackupSlot() {
  extern bool sdCardInitialized;
  if (!sdCardInitialized) return 1;

  for (int slot = 1; slot <= MAX_BACKUPS; slot++) {
    char filepath[48];
    sprintf(filepath, "%s/%s%d%s", BACKUP_FOLDER, BACKUP_PREFIX, slot, BACKUP_EXT);
    if (!SD_MMC.exists(filepath)) {
      return slot;
    }
  }
  return MAX_BACKUPS;  // Overwrite last slot if all full
}

// =============================================================================
// SD CARD BACKUP - Get backup info for preview
// =============================================================================
void getBackupInfo(int slot, BackupInfo* info) {
  info->exists = false;
  info->slot = slot;
  info->total_levels = 0;
  info->gems = 0;
  info->rebirth_count = 0;
  info->timestamp = 0;

  extern bool sdCardInitialized;
  if (!sdCardInitialized) return;

  char filepath[48];
  sprintf(filepath, "%s/%s%d%s", BACKUP_FOLDER, BACKUP_PREFIX, slot, BACKUP_EXT);

  File file = SD_MMC.open(filepath, FILE_READ);
  if (!file) return;

  SDBackupData backup;
  size_t read_bytes = file.read((uint8_t*)&backup, sizeof(backup));
  file.close();

  if (read_bytes != sizeof(backup)) return;
  if (backup.magic != BACKUP_MAGIC) return;

  info->exists = true;
  info->total_levels = backup.total_levels;
  info->gems = backup.xp_data.gems;
  info->rebirth_count = backup.rebirth_count;
  info->timestamp = backup.timestamp;
}

// =============================================================================
// SD CARD BACKUP - Main Menu UI
// =============================================================================
void showSDBackupMenu() {
  in_backup_list_view = false;

  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;

  // Header
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(80, 200, 255));
  }
  gfx->setTextSize(3);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(centerX - 54 + 2, 14);
  gfx->print("BACKUP");
  gfx->setTextColor(RGB565(80, 200, 255));
  gfx->setCursor(centerX - 54, 12);
  gfx->print("BACKUP");

  // SD Card status
  int statusY = headerH + 10;
  extern bool sdCardInitialized;
  gfx->fillRect(20, statusY, LCD_WIDTH - 40, 40, RGB565(12, 14, 20));
  gfx->drawRect(20, statusY, LCD_WIDTH - 40, 40, RGB565(40, 45, 60));
  gfx->setTextSize(2);
  gfx->setTextColor(sdCardInitialized ? RGB565(0, 200, 80) : RGB565(200, 60, 60));
  gfx->setCursor(40, statusY + 10);
  gfx->printf("SD Card: %s", sdCardInitialized ? "READY" : "NOT FOUND");

  // Current stats
  int statsY = statusY + 50;
  gfx->fillRect(20, statsY, LCD_WIDTH - 40, 80, RGB565(12, 14, 20));
  gfx->drawRect(20, statsY, LCD_WIDTH - 40, 80, RGB565(40, 45, 60));
  gfx->fillRect(20, statsY, 5, 5, RGB565(80, 200, 255));

  uint16_t total_levels = 0;
  for (int i = 0; i < 11; i++) total_levels += all_characters[i]->level;

  int num_saves = 0;
  for (int s = 1; s <= MAX_BACKUPS; s++) {
    BackupInfo bi;
    getBackupInfo(s, &bi);
    if (bi.exists) num_saves++;
  }

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setCursor(35, statsY + 8);
  gfx->print("CURRENT STATS:");

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setCursor(35, statsY + 25);
  gfx->printf("Lvls: %d   Gems: %d", total_levels, system_state.player_gems);
  gfx->setCursor(35, statsY + 50);
  gfx->printf("Reb: %d   Saves: %d/%d", getRebirthCount(), num_saves, MAX_BACKUPS);

  // NEW SAVE button
  int btnY = statsY + 100;
  int btnH = 60;
  gfx->fillRect(20, btnY, LCD_WIDTH - 40, btnH, RGB565(10, 25, 15));
  gfx->drawRect(20, btnY, LCD_WIDTH - 40, btnH, RGB565(80, 255, 80));
  gfx->fillRect(20, btnY, 5, 5, RGB565(80, 255, 80));
  gfx->fillRect(LCD_WIDTH - 25, btnY, 5, 5, RGB565(80, 255, 80));
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(80, 255, 80));
  gfx->setCursor(40, btnY + 10);
  gfx->print("NEW SAVE");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(60, 180, 60));
  int nextSlot = getNextBackupSlot();
  gfx->setCursor(40, btnY + 35);
  gfx->printf("Creates Backup #%d", nextSlot);

  // LOAD BACKUP button
  int loadY = btnY + btnH + 10;
  gfx->fillRect(20, loadY, LCD_WIDTH - 40, btnH, RGB565(10, 15, 25));
  gfx->drawRect(20, loadY, LCD_WIDTH - 40, btnH, RGB565(80, 180, 255));
  gfx->fillRect(20, loadY, 5, 5, RGB565(80, 180, 255));
  gfx->fillRect(LCD_WIDTH - 25, loadY, 5, 5, RGB565(80, 180, 255));
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(80, 180, 255));
  gfx->setCursor(40, loadY + 10);
  gfx->print("LOAD BACKUP");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(60, 130, 180));
  gfx->setCursor(40, loadY + 35);
  gfx->printf("Choose from %d saves", num_saves);

  // BACK button
  int backY = loadY + btnH + 15;
  gfx->fillRect(centerX - 50, backY, 100, 40, RGB565(15, 18, 25));
  gfx->drawRect(centerX - 50, backY, 100, 40, RGB565(60, 65, 80));
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(centerX - 25, backY + 12);
  gfx->print("BACK");

  extern void drawSwipeIndicator();
  drawSwipeIndicator();
}

// =============================================================================
// SD CARD BACKUP - List Menu UI
// =============================================================================
void showBackupListMenu() {
  in_backup_list_view = true;

  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  int centerX = LCD_WIDTH / 2;

  // Header
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, RGB565(80, 180, 255));
  }
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(80, 180, 255));
  gfx->setCursor(centerX - 80, 18);
  gfx->print("SELECT BACKUP");

  // List backups
  int cardY = headerH + 10;
  int cardH = 55;
  int cardGap = 6;
  int count = 0;

  for (int slot = 1; slot <= MAX_BACKUPS && count < 7; slot++) {
    BackupInfo info;
    getBackupInfo(slot, &info);
    if (!info.exists) continue;

    int y = cardY + count * (cardH + cardGap);

    gfx->fillRect(20, y, LCD_WIDTH - 40, cardH, RGB565(12, 14, 20));
    gfx->drawRect(20, y, LCD_WIDTH - 40, cardH, RGB565(40, 45, 60));
    gfx->fillRect(20, y, 5, cardH, RGB565(80, 180, 255));

    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setCursor(35, y + 8);
    gfx->printf("Backup #%d", slot);

    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(130, 135, 150));
    gfx->setCursor(35, y + 32);
    gfx->printf("Lvls: %d  Gems: %d  Reb: %d",
                info.total_levels, info.gems, info.rebirth_count);

    // TAP indicator
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 180, 255));
    gfx->setCursor(LCD_WIDTH - 55, y + 22);
    gfx->print("[TAP]");

    count++;
  }

  if (count == 0) {
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(centerX - 70, LCD_HEIGHT / 2 - 10);
    gfx->print("No backups found");
  }

  // BACK button
  int backY = LCD_HEIGHT - 60;
  gfx->fillRect(centerX - 50, backY, 100, 40, RGB565(15, 18, 25));
  gfx->drawRect(centerX - 50, backY, 100, 40, RGB565(60, 65, 80));
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(centerX - 25, backY + 12);
  gfx->print("BACK");

  extern void drawSwipeIndicator();
  drawSwipeIndicator();
}

// =============================================================================
// SD CARD BACKUP - Touch Handlers
// =============================================================================

void handleSDBackupTouch(int x, int y) {
  int centerX = LCD_WIDTH / 2;
  int headerH = 55;
  int statusY = headerH + 10;
  int statsY = statusY + 50;
  int btnY = statsY + 100;
  int btnH = 60;

  // NEW SAVE button
  if (x >= 20 && x < LCD_WIDTH - 20 && y >= btnY && y < btnY + btnH) {
    int slot = getNextBackupSlot();

    // Show saving animation
    gfx->fillRect(30, btnY + 5, LCD_WIDTH - 60, btnH - 10, RGB565(80, 255, 80));
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(0, 50, 0));
    gfx->setCursor(centerX - 55, btnY + 22);
    gfx->printf("SAVING #%d...", slot);

    if (saveStatsToSD(slot)) {
      delay(500);
      gfx->fillRect(30, btnY + 5, LCD_WIDTH - 60, btnH - 10, RGB565(0, 180, 80));
      gfx->setTextColor(COLOR_WHITE);
      gfx->setCursor(centerX - 65, btnY + 22);
      gfx->printf("BACKUP #%d SAVED!", slot);
      delay(1000);
    } else {
      delay(500);
      gfx->fillRect(30, btnY + 5, LCD_WIDTH - 60, btnH - 10, RGB565(200, 60, 60));
      gfx->setTextColor(COLOR_WHITE);
      gfx->setCursor(centerX - 55, btnY + 22);
      gfx->print("SAVE FAILED!");
      delay(1000);
    }

    showSDBackupMenu();
    return;
  }

  // LOAD BACKUP button
  int loadY = btnY + btnH + 10;
  if (x >= 20 && x < LCD_WIDTH - 20 && y >= loadY && y < loadY + btnH) {
    showBackupListMenu();
    return;
  }

  // BACK button
  int backY = loadY + btnH + 15;
  if (x >= centerX - 50 && x < centerX + 50 && y >= backY && y < backY + 40) {
    system_state.current_screen = SCREEN_SETTINGS;
    extern void drawSettingsApp();
    drawSettingsApp();
    return;
  }
}

void handleBackupListTouch(int x, int y) {
  int centerX = LCD_WIDTH / 2;
  int headerH = 55;
  int cardY = headerH + 10;
  int cardH = 55;
  int cardGap = 6;

  // Check backup slot taps
  int count = 0;
  for (int slot = 1; slot <= MAX_BACKUPS && count < 7; slot++) {
    BackupInfo info;
    getBackupInfo(slot, &info);
    if (!info.exists) continue;

    int slotY = cardY + count * (cardH + cardGap);

    if (x >= 20 && x < LCD_WIDTH - 20 && y >= slotY && y < slotY + cardH) {
      // Show loading animation
      gfx->fillRect(25, slotY + 2, LCD_WIDTH - 50, cardH - 4, RGB565(80, 180, 255));
      gfx->setTextSize(2);
      gfx->setTextColor(RGB565(0, 0, 50));
      gfx->setCursor(centerX - 50, slotY + 18);
      gfx->print("Loading...");

      if (loadStatsFromSD(slot)) {
        delay(500);
        gfx->fillRect(25, slotY + 2, LCD_WIDTH - 50, cardH - 4, RGB565(0, 180, 80));
        gfx->setTextColor(COLOR_WHITE);
        gfx->setCursor(centerX - 50, slotY + 18);
        gfx->print("SUCCESS!");
        delay(1000);

        // Show reboot message
        gfx->fillScreen(RGB565(0, 0, 0));
        gfx->setTextSize(3);
        gfx->setTextColor(RGB565(80, 200, 255));
        gfx->setCursor(centerX - 90, LCD_HEIGHT / 2 - 30);
        gfx->printf("Backup #%d", slot);
        gfx->setTextSize(2);
        gfx->setTextColor(RGB565(200, 200, 200));
        gfx->setCursor(centerX - 55, LCD_HEIGHT / 2 + 10);
        gfx->print("Loaded!");
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(150, 150, 150));
        gfx->setCursor(centerX - 55, LCD_HEIGHT / 2 + 40);
        gfx->print("Restarting...");
        delay(2000);
        ESP.restart();
      } else {
        delay(500);
        gfx->fillRect(25, slotY + 2, LCD_WIDTH - 50, cardH - 4, RGB565(200, 60, 60));
        gfx->setTextColor(COLOR_WHITE);
        gfx->setCursor(centerX - 60, slotY + 18);
        gfx->print("LOAD FAILED!");
        delay(1500);
        showBackupListMenu();
      }
      return;
    }

    count++;
  }

  // BACK button
  int backY = LCD_HEIGHT - 60;
  if (x >= centerX - 50 && x < centerX + 50 && y >= backY && y < backY + 40) {
    showSDBackupMenu();
    return;
  }
}
