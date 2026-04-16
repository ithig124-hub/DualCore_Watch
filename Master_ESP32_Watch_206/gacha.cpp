/*
 * gacha.cpp - Gacha Collection System Implementation
 * Complete collectible card game with gem economy
 *
 * FUSION OS:
 * - XP rewards for pulls (5 XP per pull, 20 XP for legendary)
 * - Sell button in collection UI
 * - Sell All duplicates feature
 */

#include "gacha.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"
#include "games.h"
#include "xp_system.h" // FUSION OS: XP rewards
#include <SD_MMC.h>
#include <Arduino.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Forward declarations
bool removeCardFromDeck(int slotIndex);

// Card database - 100 cards (10 series x 10 cards each)
GachaCard gacha_cards[GACHA_TOTAL_CARDS];
bool cards_owned[GACHA_TOTAL_CARDS];
int cards_duplicates[GACHA_TOTAL_CARDS];

// =============================================================================
// COLLECTION BROWSER STATE - Swipe-based card viewing
// =============================================================================
static int collection_view_index = 0; // Current card being viewed
static int collection_filter = 0; // 0=ALL, 1-5=rarity filter (by star count)
static int collection_sort = 0; // 0=Rarity, 1=Power, 2=Name, 3=Series
static int filtered_cards[GACHA_TOTAL_CARDS]; // Filtered card indices
static int filtered_count = 0; // Number of cards after filter
static bool collection_needs_rebuild = true; // Flag to rebuild filtered list

// Series names
const char* GACHA_SERIES_NAMES[10] = {
  "One Piece", "Solo Leveling", "Wakfu", "Naruto", "Dragon Ball",
  "Demon Slayer", "Jujutsu Kaisen", "Attack on Titan", "One Punch Man", "My Hero Academia"
};

// Character names per series (10 per series)
const char* SERIES_CHARACTERS[10][10] = {
  // One Piece
  {"Luffy", "Zoro", "Sanji", "Nami", "Robin", "Chopper", "Franky", "Brook", "Jinbe", "Shanks"},
  // Solo Leveling
  {"Jin-Woo", "Igris", "Beru", "Tusk", "Iron", "Tank", "Kaisel", "Bellion", "Go Gun-Hee", "Cha Hae-In"},
  // Wakfu
  {"Yugo", "Adamai", "Ruel", "Amalia", "Evangelyne", "Percedal", "Grougaloragran", "Qilby", "Nox", "Ogrest"},
  // Naruto
  {"Naruto", "Sasuke", "Sakura", "Kakashi", "Itachi", "Madara", "Minato", "Jiraiya", "Tsunade", "Gaara"},
  // Dragon Ball
  {"Goku", "Vegeta", "Gohan", "Piccolo", "Frieza", "Cell", "Buu", "Broly", "Beerus", "Whis"},
  // Demon Slayer
  {"Tanjiro", "Nezuko", "Zenitsu", "Inosuke", "Giyu", "Rengoku", "Shinobu", "Muzan", "Akaza", "Kokushibo"},
  // Jujutsu Kaisen
  {"Gojo", "Yuji", "Megumi", "Nobara", "Sukuna", "Todo", "Maki", "Toge", "Nanami", "Yuta"},
  // Attack on Titan
  {"Levi", "Eren", "Mikasa", "Armin", "Erwin", "Hange", "Annie", "Reiner", "Bertholdt", "Zeke"},
  // One Punch Man
  {"Saitama", "Genos", "Tatsumaki", "Fubuki", "King", "Bang", "Garou", "Boros", "Sonic", "Atomic"},
  // My Hero Academia
  {"Deku", "Bakugo", "Todoroki", "All Might", "Endeavor", "Hawks", "Aizawa", "Shigaraki", "Dabi", "Toga"}
};

// Character catchphrases
const char* SERIES_CATCHPHRASES[10][10] = {
  // One Piece
  {"I'll be King of Pirates!", "Nothing happened.", "I found it... All Blue!", "Money!", "I want to live!",
   "DOCTOR!!", "SUPER!", "Yohohoho!", "Leave it to me!", "Let's drink!"},
  // Solo Leveling
  {"ARISE!", "My liege...", "SCREECH!", "...", "Ready to serve!",
   "Protecting master!", "Fly!", "At your command!", "Fight well!", "You're strong..."},
  // Wakfu
  {"Adventure!", "Brother...", "Gold!", "Nature's call!", "On target!",
   "IIIIIOP!", "Wisdom of ages", "Eliatrope secrets", "Time is Wakfu", "RAAAAWR!"},
  // Naruto
  {"Believe it!", "I'll kill Itachi!", "CHA!", "Yo.", "Forgive me, Sasuke.",
   "I am Madara Uchiha!", "Yellow Flash!", "Research time!", "GAMBLING!", "Sand coffin!"},
  // Dragon Ball
  {"Kamehameha!", "Prince of Saiyans!", "I'm the Great Saiyaman!", "DODGE!", "I'll destroy you!",
   "Perfect!", "BUU!", "KAKAROT!", "Before creation...", "That was fun!"},
  // Demon Slayer
  {"Total Concentration!", "Brother...", "ZENITSU!", "PIG ASSAULT!", "The water is calm.",
   "SET YOUR HEART ABLAZE!", "Ara ara...", "I am... inevitable.", "Die weak!", "Moon Breathing..."},
  // Jujutsu Kaisen
  {"I'm the strongest.", "Give me a sec!", "Divine Dogs!", "Nails!", "Know your place.",
   "My best friend!", "Physical strength!", "Salmon!", "Overtime again...", "Pure love!"},
  // Attack on Titan
  {"Tch.", "I'll destroy them all!", "Eren!", "Let's think...", "ADVANCE!",
   "Science!", "Female Titan!", "Armored Titan!", "Colossal kick!", "No regrets!"},
  // One Punch Man
  {"OK.", "Incinerate!", "Don't underestimate me!", "Blizzard of Hell!", "King Engine!",
   "Water Stream!", "Hunt!", "Meteoric Burst!", "Too slow!", "Atomic Slash!"},
  // My Hero Academia
  {"SMASH!", "DIE!", "Half-cold Half-hot!", "PLUS ULTRA!", "PROMINENCE BURN!",
   "Hawks!", "Don't make me use it.", "Decay!", "Cremation!", "I love you!"}
};

// =============================================================================
// GACHA SYSTEM
// =============================================================================

void initGachaSystem() {
  Serial.println("[Gacha] Initializing gacha system...");
  initCardDatabase();

  if (!loadGachaProgress()) {
    // Initialize fresh gacha-specific data only
    // NOTE: Do NOT overwrite system_state.player_gems here!
    // Gems are already loaded from NVS by loadAllGameData() in setup().
    // Overwriting here was causing gems to reset to 500 after every reboot
    // when the SD card gacha save file didn't exist.
    system_state.gacha_cards_collected = 0;
    system_state.pity_counter = 0;
    system_state.pity_legendary_counter = 0;
    system_state.deck_size = 0;
    for (int i = 0; i < MAX_DECK_SIZE; i++) system_state.battle_deck[i] = -1;

    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
      cards_owned[i] = false;
      cards_duplicates[i] = 0;
      gacha_cards[i].evolution_level = 0;
    }
  }

  Serial.printf("[Gacha] Init complete. Gems: %d\n", system_state.player_gems);
}

void initCardDatabase() {
  int card_id = 0;

  for (int series = 0; series < 10; series++) {
    for (int c = 0; c < 10; c++) {
      // Assign rarity based on character importance (first few are rarer)
      GachaRarity rarity;
      if (c == 0) rarity = RARITY_LEGENDARY; // Main character
      else if (c < 3) rarity = RARITY_EPIC; // Major characters
      else if (c < 6) rarity = RARITY_RARE; // Supporting
      else rarity = RARITY_COMMON; // Others

      // Some mythic cards for special characters
      if ((series == 0 && c == 0) || // Luffy
          (series == 1 && c == 0) || // Jin-Woo
          (series == 4 && c == 0) || // Goku
          (series == 6 && c == 0)) { // Gojo
        // These have a mythic variant too
      }

      gacha_cards[card_id] = generateCard(series, c, rarity);
      card_id++;
    }
  }

  Serial.printf("[Gacha] Initialized %d cards\n", card_id);
}

GachaCard generateCard(int series_index, int card_index, GachaRarity rarity) {
  GachaCard card;
  card.character_name = String(SERIES_CHARACTERS[series_index][card_index]);
  card.series = String(GACHA_SERIES_NAMES[series_index]);
  card.catchphrase = String(SERIES_CATCHPHRASES[series_index][card_index]);
  card.rarity = rarity;

  // Power based on rarity
  int min_power = getPowerRange(rarity, false);
  int max_power = getPowerRange(rarity, true);
  card.power_rating = random(min_power, max_power + 1);

  // Color based on series
  uint16_t series_colors[] = {
    LUFFY_SUN_GOLD, JINWOO_MONARCH_PURPLE, YUGO_PORTAL_CYAN, NARUTO_CHAKRA_ORANGE, GOKU_UI_SILVER,
    TANJIRO_FIRE_ORANGE, GOJO_INFINITY_BLUE, LEVI_SURVEY_GREEN, SAITAMA_HERO_YELLOW, DEKU_HERO_GREEN
  };
  card.card_color = series_colors[series_index];

  card.owned = false;
  card.duplicate_count = 0;
  card.duplicates = 0;
  card.evolution_level = 0;
  card.power = card.power_rating; // Store base power

  // Stats based on rarity + power
  card.hp = 50 + card.power_rating / 2;
  card.attack = 10 + card.power_rating / 5;
  card.defense = 5 + card.power_rating / 8;

  return card;
}

void saveGachaProgress() {
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    Serial.println("[Gacha] SD Card not available");
    return;
  }

  File dataFile = SD_MMC.open("/WATCH/gacha/cards.dat", FILE_WRITE);
  if (!dataFile) {
    Serial.println("[Gacha] Cannot save progress");
    return;
  }

  dataFile.printf("VERSION=2\n");
  dataFile.printf("TOTAL_COLLECTED=%d\n", system_state.gacha_cards_collected);
  dataFile.printf("PITY_EPIC=%d\n", system_state.pity_counter);
  dataFile.printf("PITY_LEGEND=%d\n", system_state.pity_legendary_counter);
  dataFile.printf("DECK_SIZE=%d\n", system_state.deck_size);

  // Save deck
  for (int i = 0; i < system_state.deck_size; i++) {
    dataFile.printf("DECK_%d=%d\n", i, system_state.battle_deck[i]);
  }

  // Save each card's owned status, duplicate count, and evolution level
  for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
    if (cards_owned[i]) {
      dataFile.printf("CARD_%d=%d\n", i, cards_duplicates[i]);
      if (gacha_cards[i].evolution_level > 0) {
        dataFile.printf("EVO_%d=%d\n", i, gacha_cards[i].evolution_level);
      }
    }
  }

  dataFile.close();
  Serial.println("[Gacha] Progress saved to SD card");
}

bool loadGachaProgress() {
  extern bool sdCardInitialized;
  if (!sdCardInitialized) {
    return false;
  }

  File dataFile = SD_MMC.open("/WATCH/gacha/cards.dat", FILE_READ);
  if (!dataFile) {
    Serial.println("[Gacha] No saved progress found");
    return false;
  }

  // Reset all cards first
  for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
    cards_owned[i] = false;
    cards_duplicates[i] = 0;
    gacha_cards[i].evolution_level = 0;
  }
  system_state.pity_counter = 0;
  system_state.pity_legendary_counter = 0;
  system_state.deck_size = 0;
  for (int i = 0; i < MAX_DECK_SIZE; i++) system_state.battle_deck[i] = -1;

  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();

    int eqPos = line.indexOf('=');
    if (eqPos < 0) continue;

    String key = line.substring(0, eqPos);
    int value = line.substring(eqPos + 1).toInt();

    if (key == "TOTAL_COLLECTED") {
      system_state.gacha_cards_collected = value;
    } else if (key == "PITY_EPIC") {
      system_state.pity_counter = value;
    } else if (key == "PITY_LEGEND") {
      system_state.pity_legendary_counter = value;
    } else if (key == "DECK_SIZE") {
      system_state.deck_size = min(value, MAX_DECK_SIZE);
    } else if (key.startsWith("DECK_")) {
      int slot = key.substring(5).toInt();
      if (slot >= 0 && slot < MAX_DECK_SIZE) {
        system_state.battle_deck[slot] = value;
      }
    } else if (key.startsWith("EVO_")) {
      int cardIndex = key.substring(4).toInt();
      if (cardIndex >= 0 && cardIndex < GACHA_TOTAL_CARDS) {
        gacha_cards[cardIndex].evolution_level = value;
        // Recalculate power with evolution multiplier
        float mult = getEvolvePowerMult(value);
        gacha_cards[cardIndex].power_rating = (int)(gacha_cards[cardIndex].power * mult);
      }
    } else if (key.startsWith("CARD_")) {
      int cardIndex = key.substring(5).toInt();
      if (cardIndex >= 0 && cardIndex < GACHA_TOTAL_CARDS) {
        cards_owned[cardIndex] = true;
        cards_duplicates[cardIndex] = value;
      }
    }
  }

  dataFile.close();
  Serial.printf("[Gacha] Progress loaded: %d cards collected\n", system_state.gacha_cards_collected);
  return true;
}

// Save a pulled card to collection
void saveGachaCard(GachaCard& card) {
  int cardId = card.id;

  if (!cards_owned[cardId]) {
    // New card!
    cards_owned[cardId] = true;
    cards_duplicates[cardId] = 1;
    system_state.gacha_cards_collected++;
    Serial.printf("[Gacha] NEW CARD: %s (%s) - %d/%d collected\n",
                  card.character_name, card.series,
                  system_state.gacha_cards_collected, GACHA_TOTAL_CARDS);
  } else {
    // Duplicate
    cards_duplicates[cardId]++;
    Serial.printf("[Gacha] Duplicate: %s (x%d)\n", card.character_name, cards_duplicates[cardId]);
  }

  // Auto-save to SD card
  saveGachaProgress();
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Draw a 5-pointed star
void drawStar(int x, int y, int size, uint16_t color) {
  int points = 5;
  float angleStep = 360.0 / points;

  for (int i = 0; i < points; i++) {
    float angle1 = (i * angleStep - 90) * PI / 180.0;
    float angle2 = ((i + 0.5) * angleStep - 90) * PI / 180.0;
    float angle3 = ((i + 1) * angleStep - 90) * PI / 180.0;

    int x1 = x + cos(angle1) * size;
    int y1 = y + sin(angle1) * size;
    int x2 = x + cos(angle2) * (size * 0.4);
    int y2 = y + sin(angle2) * (size * 0.4);
    int x3 = x + cos(angle3) * size;
    int y3 = y + sin(angle3) * size;

    gfx->fillTriangle(x, y, x1, y1, x2, y2, color);
    gfx->fillTriangle(x, y, x2, y2, x3, y3, color);
  }
}

// =============================================================================
// GEM MANAGEMENT
// =============================================================================

int getPlayerGems() {
  return system_state.player_gems;
}

void addGems(int amount, const char* source) {
  system_state.player_gems += amount;
  Serial.printf("[Gacha] +%d gems from %s (Total: %d)\n", amount, source, system_state.player_gems);
  // Persist gems to NVS immediately so they survive reboot
  extern void saveAllGameData();
  saveAllGameData();
}

bool spendGems(int amount) {
  if (system_state.player_gems >= amount) {
    system_state.player_gems -= amount;
    // Persist gems to NVS immediately so they survive reboot
    extern void saveAllGameData();
    saveAllGameData();
    return true;
  }
  return false;
}

// =============================================================================
// CARD SELLING SYSTEM
// =============================================================================

// XP/Gems rewards for selling cards by rarity
#define XP_SELL_MYTHIC     10000
#define XP_SELL_LEGENDARY  5000
#define XP_SELL_EPIC       1000
#define XP_SELL_OTHER      500
#define GEMS_SELL_MYTHIC   500
#define GEMS_SELL_LEGENDARY 200
#define GEMS_SELL_EPIC     100
#define GEMS_SELL_OTHER    50

int getCardSellXP(GachaRarity rarity) {
    switch(rarity) {
        case RARITY_MYTHIC:    return XP_SELL_MYTHIC;     // 10000 XP
        case RARITY_LEGENDARY: return XP_SELL_LEGENDARY;  // 5000 XP
        case RARITY_EPIC:      return XP_SELL_EPIC;       // 1000 XP
        default:               return XP_SELL_OTHER;      // 500 XP
    }
}

int getCardSellGems(GachaRarity rarity) {
    switch(rarity) {
        case RARITY_MYTHIC:    return GEMS_SELL_MYTHIC;    // 500 gems
        case RARITY_LEGENDARY: return GEMS_SELL_LEGENDARY; // 200 gems
        case RARITY_EPIC:      return GEMS_SELL_EPIC;      // 100 gems
        default:               return GEMS_SELL_OTHER;     // 50 gems
    }
}

bool canSellCard(int card_id) {
    if (card_id < 0 || card_id >= GACHA_TOTAL_CARDS) return false;
    return cards_owned[card_id];
}

int sellCard(int card_id) {
    if (!canSellCard(card_id)) return 0;
    
    GachaCard& card = gacha_cards[card_id];
    int xp_reward = getCardSellXP(card.rarity);
    int gem_reward = getCardSellGems(card.rarity);
    
    // Remove card from collection
    if (cards_duplicates[card_id] > 0) {
        // Has duplicates, just remove one duplicate
        cards_duplicates[card_id]--;
        Serial.printf("[Gacha] Sold duplicate %s for %d XP + %d Gems\n", 
                     card.character_name.c_str(), xp_reward, gem_reward);
    } else {
        // Selling last copy
        cards_owned[card_id] = false;
        system_state.gacha_cards_collected--;
        Serial.printf("[Gacha] Sold %s (last copy) for %d XP + %d Gems\n", 
                     card.character_name.c_str(), xp_reward, gem_reward);
        
        // Remove from deck if present
        for (int i = 0; i < system_state.deck_size; i++) {
            if (system_state.battle_deck[i] == card_id) {
                removeCardFromDeck(i);
                break;
            }
        }
    }
    
    // Award XP and Gems
    gainExperience(xp_reward, "Card Sold");
    addGems(gem_reward, "Card Sold");
    
    collection_needs_rebuild = true;
    saveGachaProgress();
    
    return xp_reward;
}

int sellCardDuplicates(int card_id) {
    if (card_id < 0 || card_id >= GACHA_TOTAL_CARDS) return 0;
    if (!cards_owned[card_id] || cards_duplicates[card_id] == 0) return 0;
    
    GachaCard& card = gacha_cards[card_id];
    int dupes = cards_duplicates[card_id];
    int xp_per_card = getCardSellXP(card.rarity);
    int gems_per_card = getCardSellGems(card.rarity);
    int total_xp = dupes * xp_per_card;
    int total_gems = dupes * gems_per_card;
    
    cards_duplicates[card_id] = 0;
    
    gainExperience(total_xp, "Sold Duplicates");
    addGems(total_gems, "Sold Duplicates");
    
    Serial.printf("[Gacha] Sold %d duplicates of %s for %d XP + %d Gems\n", 
                 dupes, card.character_name.c_str(), total_xp, total_gems);
    
    collection_needs_rebuild = true;
    saveGachaProgress();
    
    return total_xp;
}

int sellAllDuplicates() {
    int total_xp = 0;
    int total_gems = 0;
    int cards_sold = 0;
    
    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
        if (cards_owned[i] && cards_duplicates[i] > 0) {
            int dupes = cards_duplicates[i];
            int xp = dupes * getCardSellXP(gacha_cards[i].rarity);
            int gems = dupes * getCardSellGems(gacha_cards[i].rarity);
            total_xp += xp;
            total_gems += gems;
            cards_sold += dupes;
            cards_duplicates[i] = 0;
        }
    }
    
    if (cards_sold > 0) {
        gainExperience(total_xp, "Sold All Duplicates");
        addGems(total_gems, "Sold All Duplicates");
        Serial.printf("[Gacha] Sold ALL %d duplicates for %d XP + %d Gems\n", 
                     cards_sold, total_xp, total_gems);
        collection_needs_rebuild = true;
        saveGachaProgress();
    }
    
    return total_xp;
}

int getSellableCardCount(bool duplicatesOnly) {
    int count = 0;
    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
        if (cards_owned[i]) {
            if (duplicatesOnly) {
                count += cards_duplicates[i];
            } else {
                count += 1 + cards_duplicates[i];
            }
        }
    }
    return count;
}

// Get count of cards owned by rarity
void getCardCountByRarity(int* common, int* rare, int* epic, int* legendary, int* mythic) {
    *common = *rare = *epic = *legendary = *mythic = 0;
    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
        if (cards_owned[i]) {
            int totalCopies = 1 + cards_duplicates[i];
            switch(gacha_cards[i].rarity) {
                case RARITY_COMMON:    *common += totalCopies; break;
                case RARITY_RARE:      *rare += totalCopies; break;
                case RARITY_EPIC:      *epic += totalCopies; break;
                case RARITY_LEGENDARY: *legendary += totalCopies; break;
                case RARITY_MYTHIC:    *mythic += totalCopies; break;
            }
        }
    }
}

void getCollectionValue(int* outXP, int* outGems) {
    *outXP = 0;
    *outGems = 0;
    
    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
        if (cards_owned[i]) {
            int totalCopies = 1 + cards_duplicates[i];
            *outXP += totalCopies * getCardSellXP(gacha_cards[i].rarity);
            *outGems += totalCopies * getCardSellGems(gacha_cards[i].rarity);
        }
    }
}

int sellEntireCollection() {
    int total_xp = 0;
    int total_gems = 0;
    int cards_sold = 0;
    
    for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
        if (cards_owned[i]) {
            int totalCopies = 1 + cards_duplicates[i];
            int xp = totalCopies * getCardSellXP(gacha_cards[i].rarity);
            int gems = totalCopies * getCardSellGems(gacha_cards[i].rarity);
            total_xp += xp;
            total_gems += gems;
            cards_sold += totalCopies;
            
            // Remove the card completely
            cards_owned[i] = false;
            cards_duplicates[i] = 0;
            gacha_cards[i].evolution_level = 0;
        }
    }
    
    // Clear the deck since all cards are sold
    for (int i = 0; i < MAX_DECK_SIZE; i++) {
        system_state.battle_deck[i] = -1;
    }
    system_state.deck_size = 0;
    system_state.gacha_cards_collected = 0;
    
    if (cards_sold > 0) {
        gainExperience(total_xp, "Sold Entire Collection!");
        addGems(total_gems, "Sold Entire Collection!");
        Serial.printf("[Gacha] SOLD ENTIRE COLLECTION: %d cards for %d XP + %d Gems\n", 
                     cards_sold, total_xp, total_gems);
        collection_needs_rebuild = true;
        saveGachaProgress();
    }
    
    return total_xp;
}

// =============================================================================
// PULLING
// =============================================================================

bool canPullSingle() {
  return system_state.player_gems >= GACHA_SINGLE_PULL_COST;
}

bool canPullTen() {
  return system_state.player_gems >= GACHA_TEN_PULL_COST;
}

GachaRarity rollRarity() {
  int roll = random(0, 100);

  GachaRarity result;
  if (roll < 1) result = RARITY_MYTHIC; // 1%
  else if (roll < 5) result = RARITY_LEGENDARY; // 4%
  else if (roll < 20) result = RARITY_EPIC; // 15%
  else if (roll < 50) result = RARITY_RARE; // 30%
  else result = RARITY_COMMON; // 50%

  // Apply pity system
  applyPitySystem(result);
  return result;
}

void applyPitySystem(GachaRarity& rarity) {
  system_state.pity_counter++;
  system_state.pity_legendary_counter++;

  // Hard pity: guaranteed Legendary at 90 pulls
  if (system_state.pity_legendary_counter >= PITY_LEGENDARY_GUARANTEE) {
    if (rarity < RARITY_LEGENDARY) {
      rarity = RARITY_LEGENDARY;
      Serial.println("[Pity] LEGENDARY PITY triggered!");
    }
  }
  // Soft pity: guaranteed Epic at 30 pulls
  else if (system_state.pity_counter >= PITY_EPIC_GUARANTEE) {
    if (rarity < RARITY_EPIC) {
      rarity = RARITY_EPIC;
      Serial.println("[Pity] EPIC PITY triggered!");
    }
  }

  // Reset counters on high rarity pull
  if (rarity >= RARITY_EPIC) {
    system_state.pity_counter = 0;
  }
  if (rarity >= RARITY_LEGENDARY) {
    system_state.pity_legendary_counter = 0;
  }
}

GachaCard performSinglePull() {
  if (!spendGems(GACHA_SINGLE_PULL_COST)) {
    GachaCard empty;
    empty.character_name = "ERROR";
    return empty;
  }

  // FUSION OS: Award XP for gacha pull
  gainExperience(XP_GACHA_PULL, "Gacha Pull");

  GachaRarity rarity = rollRarity();

  // Find a card of this rarity
  int attempts = 0;
  while (attempts < 100) {
    int card_id = random(0, GACHA_TOTAL_CARDS);
    if (gacha_cards[card_id].rarity == rarity ||
        (rarity == RARITY_MYTHIC && gacha_cards[card_id].rarity == RARITY_LEGENDARY)) {

      GachaCard pulled = gacha_cards[card_id];
      pulled.rarity = rarity; // Override with rolled rarity for mythic upgrade

      // FUSION OS: Extra XP for legendary pulls
      if (rarity >= RARITY_LEGENDARY) {
        gainExperience(XP_GACHA_LEGENDARY, "Legendary Pull!");
      }

      addCardToCollection(pulled);
      return pulled;
    }
    attempts++;
  }

  // Fallback - return random card with rolled rarity
  int card_id = random(0, GACHA_TOTAL_CARDS);
  GachaCard pulled = gacha_cards[card_id];
  pulled.rarity = rarity;

  // FUSION OS: Extra XP for legendary pulls
  if (rarity >= RARITY_LEGENDARY) {
    gainExperience(XP_GACHA_LEGENDARY, "Legendary Pull!");
  }

  addCardToCollection(pulled);
  return pulled;
}

void performTenPull(GachaCard* results) {
  if (!spendGems(GACHA_TEN_PULL_COST)) return;

  // FUSION OS: Award XP for 10-pull (10 pulls = 50 XP base)
  gainExperience(XP_GACHA_PULL * 10, "10x Gacha Pull");

  // Guaranteed at least one Epic or higher in 10-pull
  bool hasHighRarity = false;
  int legendary_count = 0;

  for (int i = 0; i < 10; i++) {
    GachaRarity rarity = rollRarity();

    // On last card, guarantee Epic+ if none yet
    if (i == 9 && !hasHighRarity && rarity < RARITY_EPIC) {
      rarity = RARITY_EPIC;
    }

    if (rarity >= RARITY_EPIC) hasHighRarity = true;
    if (rarity >= RARITY_LEGENDARY) legendary_count++;

    // Find card of this rarity
    int card_id = random(0, GACHA_TOTAL_CARDS);
    results[i] = gacha_cards[card_id];
    results[i].rarity = rarity;

    addCardToCollection(results[i]);
  }

  // FUSION OS: Bonus XP for legendary pulls in 10-pull
  if (legendary_count > 0) {
    gainExperience(XP_GACHA_LEGENDARY * legendary_count, "Legendary in 10-Pull!");
  }
}

// =============================================================================
// COLLECTION
// =============================================================================

int getCardsOwned() {
  int count = 0;
  for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
    if (cards_owned[i]) count++;
  }
  return count;
}

int getTotalCards() {
  return GACHA_TOTAL_CARDS;
}

bool ownsCard(int card_id) {
  if (card_id < 0 || card_id >= GACHA_TOTAL_CARDS) return false;
  return cards_owned[card_id];
}

void addCardToCollection(GachaCard& card) {
  // Find this card in the database
  for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
    if (gacha_cards[i].character_name == card.character_name &&
        gacha_cards[i].series == card.series) {

      if (!cards_owned[i]) {
        cards_owned[i] = true;
        system_state.gacha_cards_collected++;
        Serial.printf("[Gacha] NEW CARD: %s (%s)\n", card.character_name.c_str(), getRarityName(card.rarity));
      } else {
        cards_duplicates[i]++;
        Serial.printf("[Gacha] Duplicate: %s (x%d)\n", card.character_name.c_str(), cards_duplicates[i]);
      }

      collection_needs_rebuild = true; // Force collection to refresh with new card

      checkCollectionRewards();
      return;
    }
  }
}

GachaCard* getCard(int card_id) {
  if (card_id < 0 || card_id >= GACHA_TOTAL_CARDS) return nullptr;
  return &gacha_cards[card_id];
}

int getDuplicateCount(int card_id) {
  if (card_id < 0 || card_id >= GACHA_TOTAL_CARDS) return 0;
  return cards_duplicates[card_id];
}

void checkCollectionRewards() {
  int owned = getCardsOwned();

  if (owned == 25) giveCollectionReward(25);
  else if (owned == 50) giveCollectionReward(50);
  else if (owned == 75) giveCollectionReward(75);
  else if (owned == 100) giveCollectionReward(100);
}

void giveCollectionReward(int milestone) {
  switch(milestone) {
    case 25: addGems(500, "25 Card Milestone"); break;
    case 50: addGems(1000, "50 Card Milestone"); break;
    case 75: addGems(2000, "75 Card Milestone"); break;
    case 100: addGems(5000, "100 Card Completion!"); break;
  }
}

// =============================================================================
// RARITY HELPERS
// =============================================================================

uint16_t getRarityBorderColor(GachaRarity rarity) {
  switch(rarity) {
    case RARITY_COMMON: return COLOR_WHITE;
    case RARITY_RARE: return COLOR_BLUE;
    case RARITY_EPIC: return COLOR_PURPLE;
    case RARITY_LEGENDARY: return COLOR_GOLD;
    case RARITY_MYTHIC: return COLOR_PINK; // Will animate rainbow
    default: return COLOR_WHITE;
  }
}

uint16_t getRarityGlowColor(GachaRarity rarity) {
  switch(rarity) {
    case RARITY_COMMON: return RGB565(50, 50, 50);
    case RARITY_RARE: return RGB565(0, 100, 200);
    case RARITY_EPIC: return RGB565(150, 50, 200);
    case RARITY_LEGENDARY: return RGB565(255, 200, 50);
    case RARITY_MYTHIC: return RGB565(255, 100, 200);
    default: return COLOR_GRAY;
  }
}

const char* getRarityName(GachaRarity rarity) {
  const char* names[] = {"Common", "Rare", "Epic", "Legendary", "MYTHIC"};
  return names[(int)rarity];
}

int getRarityStars(GachaRarity rarity) {
  return (int)rarity + 1; // 1-5 stars
}

int getPowerRange(GachaRarity rarity, bool max) {
  int ranges[][2] = {
    {100, 500},   // Common
    {501, 1500},  // Rare
    {1501, 3000}, // Epic
    {3001, 7000}, // Legendary
    {7001, 9999}  // Mythic
  };
  return ranges[(int)rarity][max ? 1 : 0];
}

// =============================================================================
// COLLECTION BROWSER - Build filtered card list for swipe navigation
// =============================================================================

void buildFilteredCardList() {
  filtered_count = 0;

  for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
    // Only show owned cards
    if (!cards_owned[i]) continue;

    // Apply rarity filter (0 = ALL, 1-5 = specific star count)
    if (collection_filter > 0) {
      int card_stars = getRarityStars(gacha_cards[i].rarity);
      if (card_stars != collection_filter) continue;
    }

    filtered_cards[filtered_count] = i;
    filtered_count++;
  }

  // Sort the filtered list
  for (int i = 0; i < filtered_count - 1; i++) {
    for (int j = i + 1; j < filtered_count; j++) {
      bool swap = false;
      int a = filtered_cards[i];
      int b = filtered_cards[j];

      switch (collection_sort) {
        case 0: // Sort by Rarity (highest first)
          swap = gacha_cards[a].rarity < gacha_cards[b].rarity;
          break;
        case 1: // Sort by Power (highest first)
          swap = gacha_cards[a].power_rating < gacha_cards[b].power_rating;
          break;
        case 2: // Sort by Name (A-Z)
          swap = gacha_cards[a].character_name > gacha_cards[b].character_name;
          break;
        case 3: // Sort by Series (A-Z)
          swap = gacha_cards[a].series > gacha_cards[b].series;
          break;
      }

      if (swap) {
        int temp = filtered_cards[i];
        filtered_cards[i] = filtered_cards[j];
        filtered_cards[j] = temp;
      }
    }
  }

  // Reset view index if out of bounds
  if (collection_view_index >= filtered_count) {
    collection_view_index = max(0, filtered_count - 1);
  }

  collection_needs_rebuild = false;
}

// =============================================================================
// GACHA UI
// =============================================================================

void drawGachaScreen() {
  // ========================================
  // RETRO ANIME GACHA SCREEN - CRT Style
  // ANTI-FLICKER: Only redraw when gems or collection changes
  // ========================================

  // ANTI-FLICKER: Track changes
  static bool needs_redraw = true;
  static int last_gems = -1;
  static int last_cards_owned = -1;

  // Check if data changed
  int current_owned = getCardsOwned();
  bool data_changed = (system_state.player_gems != last_gems) ||
                      (current_owned != last_cards_owned);

  if (!needs_redraw && !data_changed) {
    return; // Skip redraw - NO FLICKER!
  }

  // Update tracking
  needs_redraw = false;
  last_gems = system_state.player_gems;
  last_cards_owned = current_owned;

  gfx->fillScreen(RGB565(2, 2, 5));

  // CRT scan lines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();

  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }

  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(LCD_WIDTH/2 - 30 + 1, 14 + 1);
  gfx->print("GACHA");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH/2 - 30, 14);
  gfx->print("GACHA");

  // Gems display - retro card
  gfx->fillRect(20, 56, 150, 28, RGB565(12, 14, 20));
  gfx->fillRect(20, 56, 4, 4, COLOR_GOLD);
  gfx->fillRect(166, 56, 4, 4, COLOR_GOLD);
  gfx->drawRect(20, 56, 150, 28, RGB565(40, 45, 60));
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(1);
  gfx->setCursor(30, 65);
  gfx->printf("GEMS: %d", system_state.player_gems);

  // Collection progress - retro card
  gfx->fillRect(190, 56, 160, 28, RGB565(12, 14, 20));
  gfx->fillRect(190, 56, 4, 4, theme->accent);
  gfx->fillRect(346, 56, 4, 4, theme->accent);
  gfx->drawRect(190, 56, 160, 28, RGB565(40, 45, 60));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(200, 65);
  gfx->printf("CARDS: %d/%d", getCardsOwned(), getTotalCards());

  // Pull panel - retro framed
  gfx->fillRect(30, 95, LCD_WIDTH - 60, 120, RGB565(12, 14, 20));
  gfx->drawRect(30, 95, LCD_WIDTH - 60, 120, RGB565(40, 45, 60));
  // Pixel corner accents
  gfx->fillRect(30, 95, 5, 5, theme->primary);
  gfx->fillRect(LCD_WIDTH - 35, 95, 5, 5, theme->primary);
  gfx->fillRect(30, 210, 5, 5, theme->primary);
  gfx->fillRect(LCD_WIDTH - 35, 210, 5, 5, theme->primary);

  // Single Pull button
  bool canSingle = canPullSingle();
  uint16_t pullColor = canSingle ? theme->primary : RGB565(40, 42, 55);
  gfx->fillRect(50, 115, LCD_WIDTH - 100, 35, pullColor);
  gfx->drawRect(50, 115, LCD_WIDTH - 100, 35, RGB565(60, 65, 80));
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(80, 122);
  gfx->printf("1x Pull - %d", GACHA_SINGLE_PULL_COST);

  // 10x Pull button
  bool canTen = canPullTen();
  uint16_t pull10Color = canTen ? theme->accent : RGB565(40, 42, 55);
  gfx->fillRect(50, 165, LCD_WIDTH - 100, 35, pull10Color);
  gfx->drawRect(50, 165, LCD_WIDTH - 100, 35, RGB565(60, 65, 80));
  gfx->setCursor(70, 172);
  gfx->printf("10x Pull - %d", GACHA_TEN_PULL_COST);

  // Recent pull display
  gfx->setTextColor(RGB565(100, 110, 130));
  gfx->setTextSize(1);
  gfx->setCursor(40, 230);
  gfx->print("RECENT PULLS:");

  gfx->fillRect(30, 245, LCD_WIDTH - 60, 100, RGB565(10, 12, 18));
  gfx->drawRect(30, 245, LCD_WIDTH - 60, 100, RGB565(35, 40, 55));

  // Scan lines inside pull area
  for (int sy = 248; sy < 342; sy += 4) {
    gfx->drawFastHLine(32, sy, LCD_WIDTH - 64, RGB565(6, 6, 10));
  }

  // === NAVIGATION BUTTONS - LARGER, MORE VISIBLE ===
  int btnY = 365;
  int btnH = 40;
  
  // Collection button
  gfx->fillRoundRect(10, btnY, 92, btnH, 5, RGB565(15, 20, 30));
  gfx->drawRoundRect(10, btnY, 92, btnH, 5, theme->primary);
  gfx->drawFastHLine(14, btnY + 1, 84, theme->primary);
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(14, btnY + 4);
  gfx->print("CARDS");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(140, 145, 165));
  gfx->setCursor(14, btnY + 24);
  gfx->print("View & Sell");

  // Evolve button
  gfx->fillRoundRect(110, btnY, 80, btnH, 5, RGB565(22, 20, 10));
  gfx->drawRoundRect(110, btnY, 80, btnH, 5, COLOR_GOLD);
  gfx->drawFastHLine(114, btnY + 1, 72, COLOR_GOLD);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(114, btnY + 4);
  gfx->print("EVOLVE");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(160, 140, 60));
  gfx->setCursor(114, btnY + 24);
  gfx->print("Power Up");

  // Deck button
  gfx->fillRoundRect(198, btnY, 72, btnH, 5, RGB565(10, 20, 22));
  gfx->drawRoundRect(198, btnY, 72, btnH, 5, COLOR_CYAN);
  gfx->drawFastHLine(202, btnY + 1, 64, COLOR_CYAN);
  gfx->setTextColor(COLOR_CYAN);
  gfx->setTextSize(2);
  gfx->setCursor(206, btnY + 4);
  gfx->print("DECK");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(60, 130, 140));
  gfx->setCursor(206, btnY + 24);
  gfx->print("Battle");

  // Back button
  gfx->fillRoundRect(278, btnY, 78, btnH, 5, RGB565(18, 14, 20));
  gfx->drawRoundRect(278, btnY, 78, btnH, 5, RGB565(140, 100, 160));
  gfx->drawFastHLine(282, btnY + 1, 70, RGB565(120, 80, 140));
  gfx->setTextColor(RGB565(200, 160, 220));
  gfx->setTextSize(2);
  gfx->setCursor(286, btnY + 4);
  gfx->print("BACK");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(130, 100, 140));
  gfx->setCursor(282, btnY + 24);
  gfx->print("Games Menu");

  // Pity counter at bottom
  drawPityCounter(20, 415);

  drawSwipeIndicator();
}

// =============================================================================
// WATCHDOG-SAFE ANIMATION FUNCTIONS
// These use shorter delays and fewer frames to prevent watchdog timeouts
// =============================================================================

void drawGachaPullAnimation(GachaCard& card) {
  // Retro CRT-style pull animation - WATCHDOG SAFE
  gfx->fillScreen(RGB565(2, 2, 5));

  uint16_t glowColor = getRarityGlowColor(card.rarity);

  // Expanding pixel squares instead of circles - fewer iterations
  for (int r = 10; r < 100; r += 10) {
    gfx->drawRect(LCD_WIDTH/2 - r, LCD_HEIGHT/2 - r, r*2, r*2, glowColor);
    delay(50);
  }

  // Flash based on rarity - CRT flicker - shorter delays
  for (int i = 0; i < (int)card.rarity + 1; i++) {
    gfx->fillScreen(glowColor);
    delay(50);
    gfx->fillScreen(RGB565(2, 2, 5));
    delay(50);
  }
}

void drawGachaReveal(GachaCard& card) {
  // Retro CRT reveal
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  // Draw large card in center
  int cardW = 200, cardH = 280;
  int cardX = (LCD_WIDTH - cardW) / 2;
  int cardY = 60;

  drawGachaCard(cardX, cardY, cardW, cardH, card);

  // Retro tap prompt
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 50, 400);
  gfx->print("> TAP TO CONTINUE <");
}

// IMPROVED GACHA REVEAL with retro CRT animations - WATCHDOG SAFE
void drawGachaRevealImproved(GachaCard& card) {
  // Animated pixel reveal - reduced frames to prevent watchdog
  for (int frame = 0; frame < 15; frame++) {
    gfx->fillScreen(RGB565(2, 2, 5));

    // CRT scan lines
    for (int y = 0; y < LCD_HEIGHT; y += 4) {
      gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
    }

    // Particle effects - pixel style based on rarity
    for (int i = 0; i < 25; i++) {
      int px = random(0, LCD_WIDTH);
      int py = random(0, LCD_HEIGHT);
      uint16_t pcolor;

      switch(card.rarity) {
        case RARITY_MYTHIC: pcolor = (random(0, 3) == 0) ? COLOR_GOLD : COLOR_PINK; break;
        case RARITY_LEGENDARY: pcolor = COLOR_GOLD; break;
        case RARITY_EPIC: pcolor = COLOR_PURPLE; break;
        case RARITY_RARE: pcolor = COLOR_BLUE; break;
        default: pcolor = RGB565(100, 105, 120); break;
      }
      gfx->fillRect(px, py, 3, 3, pcolor);
    }

    // Card with retro pixel glow
    int cardW = 240, cardH = 320;
    int cardX = (LCD_WIDTH - cardW) / 2;
    int cardY = 80;

    // Animated glow - pixel rectangles
    uint16_t glowColor = getRarityBorderColor(card.rarity);
    for (int r = 0; r < 4; r++) {
      int offset = r + (frame % 3);
      gfx->drawRect(cardX - offset, cardY - offset,
                    cardW + offset*2, cardH + offset*2, glowColor);
    }

    // Card background - retro dark
    gfx->fillRect(cardX, cardY, cardW, cardH, RGB565(12, 14, 20));
    gfx->drawRect(cardX, cardY, cardW, cardH, glowColor);
    // Pixel corners
    gfx->fillRect(cardX, cardY, 8, 8, glowColor);
    gfx->fillRect(cardX + cardW - 8, cardY, 8, 8, glowColor);
    gfx->fillRect(cardX, cardY + cardH - 8, 8, 8, glowColor);
    gfx->fillRect(cardX + cardW - 8, cardY + cardH - 8, 8, 8, glowColor);

    // CRT scan lines on card
    for (int sy = cardY + 2; sy < cardY + cardH - 2; sy += 4) {
      gfx->drawFastHLine(cardX + 2, sy, cardW - 4, RGB565(8, 10, 16));
    }

    // Character name with glow effect
    gfx->setTextSize(2);
    gfx->setTextColor(glowColor);
    int nameLen = strlen(card.character_name.c_str()) * 12;
    gfx->setCursor(cardX + (cardW - nameLen)/2, cardY + 30);
    gfx->print(card.character_name);

    // Series
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(120, 125, 140));
    int seriesLen = strlen(card.series.c_str()) * 6;
    gfx->setCursor(cardX + (cardW - seriesLen)/2, cardY + 60);
    gfx->print(card.series);

    // Rarity stars (animated - pixel style)
    int starY = cardY + 100;
    int starCount = getRarityStars(card.rarity);
    int starSpacing = 30;
    int startX = cardX + (cardW - (starCount * starSpacing))/2;

    for (int s = 0; s < starCount; s++) {
      if (frame > s * 2) {
        drawStar(startX + s * starSpacing, starY, 12, COLOR_GOLD);
      }
    }

    // Power level
    gfx->setTextSize(4);
    gfx->setTextColor(glowColor);
    char powerStr[10];
    sprintf(powerStr, "%d", card.power_rating);
    int powerLen = strlen(powerStr) * 24;
    gfx->setCursor(cardX + (cardW - powerLen)/2, cardY + 160);
    gfx->print(powerStr);

    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(cardX + cardW/2 - 18, cardY + 200);
    gfx->print("POWER");

    // Rarity name
    const char* rarityName = getRarityName(card.rarity);
    gfx->setTextSize(2);
    gfx->setTextColor(glowColor);
    int rarityLen = strlen(rarityName) * 12;
    gfx->setCursor(cardX + (cardW - rarityLen)/2, cardY + 230);
    gfx->print(rarityName);

    // Catchphrase
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(150, 155, 170));
    String phrase = card.catchphrase;
    if (phrase.length() > 30) phrase = phrase.substring(0, 27) + "...";
    int phraseLen = phrase.length() * 6;
    gfx->setCursor(cardX + (cardW - phraseLen)/2, cardY + 270);
    gfx->print(phrase);

    delay(40);
  }

  // Final state - retro blinking prompt
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(LCD_WIDTH/2 - 96, 430);
  gfx->print("> CONTINUE <");

  for (int i = 0; i < 3; i++) {
    delay(300);
    gfx->fillRect(0, 425, LCD_WIDTH, 30, RGB565(2, 2, 5));
    delay(300);
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(LCD_WIDTH/2 - 96, 430);
    gfx->print("> CONTINUE <");
  }
}

// =============================================================================
// 10-CARD GRID REVEAL for 10x Pull
// Shows all 10 pulled cards in a 5x2 grid layout
// =============================================================================
void drawTenPullGrid(GachaCard* results) {
  // Brief pull animation
  gfx->fillScreen(RGB565(2, 2, 5));
  uint16_t flashColor = getRarityGlowColor(results[9].rarity);
  for (int r = 10; r < 80; r += 15) {
    gfx->drawRect(LCD_WIDTH/2 - r, LCD_HEIGHT/2 - r, r*2, r*2, flashColor);
    delay(30);
  }
  gfx->fillScreen(flashColor);
  delay(60);

  // Draw the grid screen
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();

  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 38, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 36, 6, 3, theme->accent);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH/2 - 60, 10);
  gfx->print("10x PULL!");

  // Grid layout: 5 columns x 2 rows
  int cols = 5;
  int rows = 2;
  int cardW = 72;
  int cardH = 170;
  int gapX = 5;
  int gapY = 8;
  int gridW = cols * cardW + (cols - 1) * gapX;
  int startX = (LCD_WIDTH - gridW) / 2;
  int startY = 46;

  // Count rarities for summary
  int rarCount[5] = {0, 0, 0, 0, 0}; // Common, Rare, Epic, Legendary, Mythic

  for (int i = 0; i < 10; i++) {
    int col = i % cols;
    int row = i / cols;
    int cx = startX + col * (cardW + gapX);
    int cy = startY + row * (cardH + gapY);

    GachaCard& card = results[i];
    uint16_t borderColor = getRarityBorderColor(card.rarity);
    uint16_t glowColor = getRarityGlowColor(card.rarity);
    int rarIdx = min((int)card.rarity, 4);
    rarCount[rarIdx]++;

    // Card glow (subtle)
    for (int g = 2; g > 0; g--) {
      gfx->drawRoundRect(cx - g, cy - g, cardW + g*2, cardH + g*2, 4, glowColor);
    }

    // Card background
    gfx->fillRoundRect(cx, cy, cardW, cardH, 4, RGB565(12, 14, 22));
    gfx->drawRoundRect(cx, cy, cardW, cardH, 4, borderColor);

    // Rarity banner (compact)
    const char* rarName = getRarityName(card.rarity);
    int bannerW = cardW - 8;
    gfx->fillRect(cx + 4, cy + 4, bannerW, 14, borderColor);
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(0, 0, 0));
    int rnLen = strlen(rarName) * 6;
    gfx->setCursor(cx + 4 + (bannerW - rnLen) / 2, cy + 7);
    gfx->print(rarName);

    // Portrait area
    gfx->fillRect(cx + 6, cy + 22, cardW - 12, 50, card.card_color);
    for (int sy = cy + 23; sy < cy + 71; sy += 3) {
      gfx->drawFastHLine(cx + 7, sy, cardW - 14, RGB565(0, 0, 0));
    }

    // Character initial
    gfx->setTextSize(3);
    gfx->setTextColor(COLOR_WHITE);
    char init[2] = {card.character_name[0], 0};
    gfx->setCursor(cx + cardW/2 - 9, cy + 32);
    gfx->print(init);

    // Character name (truncated)
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_WHITE);
    String name = card.character_name;
    if (name.length() > 10) name = name.substring(0, 9) + ".";
    int nLen = name.length() * 6;
    gfx->setCursor(cx + (cardW - nLen) / 2, cy + 78);
    gfx->print(name);

    // Series (small)
    gfx->setTextColor(RGB565(100, 105, 120));
    String ser = card.series;
    if (ser.length() > 10) ser = ser.substring(0, 9) + ".";
    int sLen = ser.length() * 6;
    gfx->setCursor(cx + (cardW - sLen) / 2, cy + 92);
    gfx->print(ser);

    // Stars
    int starCount = getRarityStars(card.rarity);
    int starW = starCount * 8;
    int starSX = cx + (cardW - starW) / 2;
    gfx->setTextColor(COLOR_GOLD);
    for (int s = 0; s < starCount; s++) {
      gfx->setCursor(starSX + s * 8, cy + 106);
      gfx->print("*");
    }

    // Power
    gfx->setTextSize(2);
    gfx->setTextColor(borderColor);
    char pwrStr[8];
    sprintf(pwrStr, "%d", card.power_rating);
    int pLen = strlen(pwrStr) * 12;
    gfx->setCursor(cx + (cardW - pLen) / 2, cy + 122);
    gfx->print(pwrStr);

    // Power label
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(cx + cardW/2 - 12, cy + 142);
    gfx->print("PWR");

    // New badge indicator
    gfx->setTextColor(COLOR_GREEN);
    gfx->setCursor(cx + cardW/2 - 6, cy + cardH - 14);
    gfx->print("NEW");
  }

  // Bottom summary bar
  int sumY = startY + rows * (cardH + gapY) + 4;
  gfx->fillRoundRect(10, sumY, LCD_WIDTH - 20, 24, 4, RGB565(10, 14, 22));
  gfx->drawRoundRect(10, sumY, LCD_WIDTH - 20, 24, 4, RGB565(40, 45, 60));

  gfx->setTextSize(1);
  int sx2 = 18;
  const char* rNames[] = {"C", "R", "E", "L", "M"};
  uint16_t rColors[] = {
    RGB565(150, 150, 160), RGB565(30, 144, 255), RGB565(160, 32, 240),
    RGB565(255, 215, 0), RGB565(255, 100, 200)
  };
  for (int r = 0; r < 5; r++) {
    if (rarCount[r] > 0) {
      gfx->setTextColor(rColors[r]);
      gfx->setCursor(sx2, sumY + 8);
      gfx->printf("%s:%d", rNames[r], rarCount[r]);
      sx2 += 40 + (rarCount[r] > 9 ? 6 : 0);
    }
  }

  // Tap to continue
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(LCD_WIDTH/2 - 96, sumY + 30);
  gfx->print("> CONTINUE <");

  // Blink prompt
  for (int i = 0; i < 2; i++) {
    delay(400);
    gfx->fillRect(0, sumY + 26, LCD_WIDTH, 22, RGB565(2, 2, 5));
    delay(400);
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(LCD_WIDTH/2 - 96, sumY + 30);
    gfx->print("> CONTINUE <");
  }
}


void drawGachaCard(int x, int y, int w, int h, GachaCard& card) {
  // FUSION OS: FANCY GACHA CARD VISUALS

  // Rarity-based gradient background
  uint16_t rarity_gradients[5][3] = {
    // Common: Gray gradient
    {RGB565(60, 60, 70), RGB565(80, 80, 90), RGB565(100, 100, 110)},
    // Rare: Blue gradient
    {RGB565(30, 80, 150), RGB565(50, 100, 180), RGB565(70, 120, 210)},
    // Epic: Purple gradient
    {RGB565(120, 40, 180), RGB565(150, 60, 210), RGB565(180, 80, 240)},
    // Legendary: Gold gradient
    {RGB565(180, 120, 0), RGB565(220, 160, 30), RGB565(255, 200, 60)},
    // Mythic: Rainbow/Prismatic
    {RGB565(255, 50, 150), RGB565(100, 200, 255), RGB565(255, 255, 100)}
  };

  int rarity_idx = min((int)card.rarity, 4);

  // Draw gradient background
  for (int i = 0; i < h; i++) {
    float progress = (float)i / h;
    uint16_t color;
    if (progress < 0.33) {
      color = rarity_gradients[rarity_idx][0];
    } else if (progress < 0.66) {
      color = rarity_gradients[rarity_idx][1];
    } else {
      color = rarity_gradients[rarity_idx][2];
    }
    gfx->drawFastHLine(x, y + i, w, color);
  }

  // Shine effect (diagonal lines)
  for (int i = 0; i < 8; i++) {
    int shine_x = x + (i * 20) - 20;
    gfx->drawLine(shine_x, y, shine_x + 30, y + h, RGB565(255, 255, 255));
  }

  // Fancy border with glow
  uint16_t borderColor = getRarityBorderColor(card.rarity);
  gfx->drawRoundRect(x - 2, y - 2, w + 4, h + 4, 8, rarity_gradients[rarity_idx][2]);
  gfx->drawRoundRect(x, y, w, h, 8, COLOR_WHITE);
  gfx->drawRoundRect(x + 1, y + 1, w - 2, h - 2, 8, borderColor);

  // Pixel corner accents (larger)
  gfx->fillRect(x, y, 10, 10, borderColor);
  gfx->fillRect(x + w - 10, y, 10, 10, borderColor);
  gfx->fillRect(x, y + h - 10, 10, 10, borderColor);
  gfx->fillRect(x + w - 10, y + h - 10, 10, 10, borderColor);

  // Character portrait (circular area)
  int portrait_cx = x + w/2;
  int portrait_cy = y + h/3;
  int portrait_r = min(w, h) / 5;
  gfx->fillCircle(portrait_cx, portrait_cy, portrait_r, RGB565(20, 20, 30));
  gfx->drawCircle(portrait_cx, portrait_cy, portrait_r, COLOR_WHITE);
  gfx->drawCircle(portrait_cx, portrait_cy, portrait_r - 2, borderColor);

  // Character name with shadow (fancy)
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(0, 0, 0));
  gfx->setCursor(x + 11, y + h/2 + 11);
  gfx->print(card.character_name);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(x + 10, y + h/2 + 10);
  gfx->print(card.character_name);

  // Series name
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(x + 10, y + h/2 + 35);
  gfx->print(card.series);

  // Rarity stars (golden)
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(255, 220, 0));
  gfx->setCursor(x + 10, y + h/2 + 55);
  for (int i = 0; i < getRarityStars(card.rarity); i++) {
    gfx->print("★");
  }

  // Stat boxes (fancy mini boxes)
  int stat_y = y + h - 50;
  int box_w = (w - 40) / 3;
  int box_h = 35;

  // Power stat box
  gfx->fillRect(x + 10, stat_y, box_w, box_h, RGB565(20, 20, 30));
  gfx->drawRect(x + 10, stat_y, box_w, box_h, RGB565(100, 100, 120));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 170));
  gfx->setCursor(x + 12, stat_y + 5);
  gfx->print("PWR");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(x + 15, stat_y + 18);
  gfx->printf("%d", card.power);

  // HP stat box
  gfx->fillRect(x + 15 + box_w, stat_y, box_w, box_h, RGB565(20, 20, 30));
  gfx->drawRect(x + 15 + box_w, stat_y, box_w, box_h, RGB565(100, 100, 120));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 170));
  gfx->setCursor(x + 17 + box_w, stat_y + 5);
  gfx->print("HP");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(x + 17 + box_w, stat_y + 18);
  gfx->printf("%d", card.hp);

  // ATK stat box
  gfx->fillRect(x + 20 + box_w * 2, stat_y, box_w, box_h, RGB565(20, 20, 30));
  gfx->drawRect(x + 20 + box_w * 2, stat_y, box_w, box_h, RGB565(100, 100, 120));
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 150, 170));
  gfx->setCursor(x + 22 + box_w * 2, stat_y + 5);
  gfx->print("ATK");
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(x + 22 + box_w * 2, stat_y + 18);
  gfx->printf("%d", card.attack);

  // Rarity name badge at bottom
  gfx->fillRoundRect(x + w/2 - 30, y + h - 15, 60, 12, 3, borderColor);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(0, 0, 0));
  gfx->setCursor(x + w/2 - 20, y + h - 12);
  gfx->print(getRarityName(card.rarity));
}

void drawCardRarityBorder(int x, int y, int w, int h, GachaRarity rarity) {
  uint16_t color = getRarityBorderColor(rarity);
  int thickness = (rarity >= RARITY_EPIC) ? 4 : 2;

  // Retro pixel borders
  for (int i = 0; i < thickness; i++) {
    gfx->drawRect(x - i, y - i, w + i*2, h + i*2, color);
  }

  // Mythic pixel effect
  if (rarity == RARITY_MYTHIC) {
    gfx->drawRect(x - 4, y - 4, w + 8, h + 8, COLOR_PINK);
    gfx->drawRect(x - 5, y - 5, w + 10, h + 10, COLOR_CYAN);
    // Pixel corners
    gfx->fillRect(x - 5, y - 5, 8, 8, COLOR_GOLD);
    gfx->fillRect(x + w - 3, y - 5, 8, 8, COLOR_GOLD);
    gfx->fillRect(x - 5, y + h - 3, 8, 8, COLOR_GOLD);
    gfx->fillRect(x + w - 3, y + h - 3, 8, 8, COLOR_GOLD);
  }
}

void drawCardGlow(int x, int y, int w, int h, GachaRarity rarity) {
  if (rarity < RARITY_RARE) return;

  uint16_t glowColor = getRarityGlowColor(rarity);

  // Retro pixel glow
  for (int i = 0; i < 3; i++) {
    gfx->drawRect(x - 6 - i, y - 6 - i, w + 12 + i*2, h + 12 + i*2, glowColor);
  }
}

void drawGachaCollection() {
  // ========================================
  // IMPROVED SWIPE-BASED CARD COLLECTION UI
  // Features: Large card view, swipe navigation, filter by rarity, sort options
  // ========================================

  // Rebuild filtered list if needed
  if (collection_needs_rebuild) {
    buildFilteredCardList();
  }

  gfx->fillScreen(RGB565(2, 2, 5));

  // CRT scan lines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();

  // === HEADER ===
  gfx->fillRect(0, 0, LCD_WIDTH, 50, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 48, 6, 3, theme->accent);
  }

  // Title with swipe indicators
  gfx->setTextSize(2);
  gfx->setTextColor(theme->primary);
  gfx->setCursor(LCD_WIDTH/2 - 60, 8);
  gfx->print("COLLECTION");

  // Progress counter
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(150, 155, 170));
  gfx->setCursor(LCD_WIDTH/2 - 45, 32);
  gfx->printf("%d / %d cards owned", getCardsOwned(), getTotalCards());

  // === FILTER TABS ===
  int filterY = 55;
  int tabW = 58;
  int tabSpacing = 5;
  int filterStartX = (LCD_WIDTH - (6 * tabW + 5 * tabSpacing)) / 2;

  const char* filterLabels[] = {"ALL", "*", "**", "***", "****", "*****"};
  uint16_t filterColors[] = {
    RGB565(100, 105, 120), // ALL - Gray
    RGB565(150, 150, 160), // Common - Light gray
    RGB565(30, 144, 255),  // Rare - Blue
    RGB565(160, 32, 240),  // Epic - Purple
    RGB565(255, 215, 0),   // Legendary - Gold
    RGB565(255, 100, 200)  // Mythic - Pink
  };

  for (int f = 0; f < 6; f++) {
    int fx = filterStartX + f * (tabW + tabSpacing);
    bool selected = (collection_filter == f);

    if (selected) {
      gfx->fillRect(fx, filterY, tabW, 22, RGB565(25, 28, 35));
      gfx->drawRect(fx, filterY, tabW, 22, filterColors[f]);
      gfx->fillRect(fx, filterY, 4, 4, filterColors[f]);
      gfx->fillRect(fx + tabW - 4, filterY, 4, 4, filterColors[f]);
    } else {
      gfx->fillRect(fx, filterY, tabW, 22, RGB565(12, 14, 20));
      gfx->drawRect(fx, filterY, tabW, 22, RGB565(40, 45, 60));
    }

    gfx->setTextSize(1);
    gfx->setTextColor(selected ? filterColors[f] : RGB565(80, 85, 100));
    int labelLen = strlen(filterLabels[f]) * 6;
    gfx->setCursor(fx + (tabW - labelLen) / 2, filterY + 7);
    gfx->print(filterLabels[f]);
  }

  // === MAIN CARD DISPLAY AREA ===
  if (filtered_count == 0) {
    // No cards message
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 - 55, 200);
    gfx->print("NO CARDS");

    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 70, 230);
    if (collection_filter == 0) {
      gfx->print("Pull cards from Gacha!");
    } else {
      gfx->print("No cards of this rarity");
    }
  } else {
    // Get current card
    int cardIdx = filtered_cards[collection_view_index];
    GachaCard& card = gacha_cards[cardIdx];

    // === SWIPE INDICATORS - Enhanced ===
    // Left arrow (if not first card)
    if (collection_view_index > 0) {
      gfx->setTextSize(3);
      gfx->setTextColor(RGB565(60, 65, 85));
      gfx->setCursor(8, 210);
      gfx->print("<");
      // Small dot indicator
      gfx->fillCircle(18, 240, 3, theme->accent);
    }

    // Right arrow (if not last card)
    if (collection_view_index < filtered_count - 1) {
      gfx->setTextSize(3);
      gfx->setTextColor(RGB565(60, 65, 85));
      gfx->setCursor(LCD_WIDTH - 28, 210);
      gfx->print(">");
      gfx->fillCircle(LCD_WIDTH - 18, 240, 3, theme->accent);
    }

    // === LARGE CARD DISPLAY ===
    int cardW = 280;
    int cardH = 270;
    int cardX = (LCD_WIDTH - cardW) / 2;
    int cardY = 88;

    // Card glow based on rarity
    uint16_t glowColor = getRarityGlowColor(card.rarity);
    uint16_t borderColor = getRarityBorderColor(card.rarity);

    // Enhanced outer glow - wider, softer
    for (int g = 8; g > 0; g--) {
      uint16_t fadeGlow = RGB565(
        ((glowColor >> 11) & 0x1F) * g / 10,
        ((glowColor >> 5) & 0x3F) * g / 12,
        (glowColor & 0x1F) * g / 10
      );
      gfx->drawRoundRect(cardX - g, cardY - g, cardW + g*2, cardH + g*2, 14, fadeGlow);
    }

    // Card background with subtle gradient effect
    gfx->fillRoundRect(cardX, cardY, cardW, cardH, 12, RGB565(12, 14, 22));
    // Inner lighter area for depth
    gfx->fillRoundRect(cardX + 3, cardY + 3, cardW - 6, cardH - 6, 10, RGB565(18, 20, 30));

    // Double border for premium feel
    gfx->drawRoundRect(cardX, cardY, cardW, cardH, 12, borderColor);
    gfx->drawRoundRect(cardX + 2, cardY + 2, cardW - 4, cardH - 4, 10, RGB565(
      ((borderColor >> 11) & 0x1F) * 12 / 31,
      ((borderColor >> 5) & 0x3F) * 12 / 63,
      (borderColor & 0x1F) * 12 / 31
    ));

    // CRT scan lines on card (subtler)
    for (int sy = cardY + 4; sy < cardY + cardH - 4; sy += 5) {
      gfx->drawFastHLine(cardX + 4, sy, cardW - 8, RGB565(8, 10, 16));
    }

    // Corner accents - diamond style
    for (int c = 0; c < 3; c++) {
      gfx->fillRect(cardX + c, cardY + c, 10 - c*2, 10 - c*2, borderColor);
      gfx->fillRect(cardX + cardW - 10 + c, cardY + c, 10 - c*2, 10 - c*2, borderColor);
      gfx->fillRect(cardX + c, cardY + cardH - 10 + c, 10 - c*2, 10 - c*2, borderColor);
      gfx->fillRect(cardX + cardW - 10 + c, cardY + cardH - 10 + c, 10 - c*2, 10 - c*2, borderColor);
    }

    // === RARITY BANNER - Enhanced ===
    const char* rarityName = getRarityName(card.rarity);
    int bannerW = strlen(rarityName) * 12 + 28;
    int bannerX = cardX + (cardW - bannerW) / 2;
    // Banner shadow
    gfx->fillRoundRect(bannerX + 2, cardY + 10, bannerW, 26, 6, RGB565(0, 0, 0));
    // Banner fill
    gfx->fillRoundRect(bannerX, cardY + 8, bannerW, 26, 6, borderColor);
    // Banner highlight line
    gfx->drawFastHLine(bannerX + 4, cardY + 9, bannerW - 8, COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(0, 0, 0));
    gfx->setCursor(bannerX + 14, cardY + 13);
    gfx->print(rarityName);

    // === CHARACTER PORTRAIT AREA - Enhanced ===
    int portraitY = cardY + 42;
    int portraitH = 85;
    int portraitX = cardX + 18;
    int portraitW = cardW - 36;
    
    // Portrait border glow
    gfx->drawRect(portraitX - 1, portraitY - 1, portraitW + 2, portraitH + 2, borderColor);
    // Portrait fill with card color
    gfx->fillRect(portraitX, portraitY, portraitW, portraitH, card.card_color);
    // Vignette corners (darken edges)
    for (int v = 0; v < 15; v++) {
      uint8_t alpha = 15 - v;
      gfx->drawFastHLine(portraitX, portraitY + v, portraitW, RGB565(alpha, alpha, alpha + 2));
      gfx->drawFastHLine(portraitX, portraitY + portraitH - 1 - v, portraitW, RGB565(alpha, alpha, alpha + 2));
    }
    // Subtle scanlines on portrait
    for (int py = portraitY + 2; py < portraitY + portraitH - 2; py += 4) {
      gfx->drawFastHLine(portraitX + 1, py, portraitW - 2, RGB565(0, 0, 0));
    }

    // Character initial - larger with shadow
    gfx->setTextSize(6);
    gfx->setTextColor(RGB565(0, 0, 0));
    char initial[2] = {card.character_name[0], 0};
    gfx->setCursor(cardX + cardW/2 - 16, portraitY + 15);
    gfx->print(initial);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(cardX + cardW/2 - 18, portraitY + 13);
    gfx->print(initial);

    // === CHARACTER NAME - Enhanced ===
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_WHITE);
    int nameLen = card.character_name.length() * 12;
    gfx->setCursor(cardX + (cardW - nameLen) / 2, cardY + 135);
    gfx->print(card.character_name);

    // === SERIES NAME with decorative line ===
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(140, 145, 165));
    int seriesLen = card.series.length() * 6;
    int seriesX = cardX + (cardW - seriesLen) / 2;
    gfx->setCursor(seriesX, cardY + 157);
    gfx->print(card.series);
    // Decorative lines beside series
    gfx->drawFastHLine(cardX + 20, cardY + 161, seriesX - cardX - 26, RGB565(40, 45, 60));
    gfx->drawFastHLine(seriesX + seriesLen + 6, cardY + 161, cardX + cardW - 20 - seriesX - seriesLen - 6, RGB565(40, 45, 60));

    // === RARITY STARS - Enhanced ===
    int starCount = getRarityStars(card.rarity);
    int starSpacing = 22;
    int starStartX = cardX + (cardW - (starCount * starSpacing)) / 2;
    for (int s = 0; s < starCount; s++) {
      // Star shadow
      drawStar(starStartX + s * starSpacing + 11, cardY + 178, 9, RGB565(80, 60, 0));
      drawStar(starStartX + s * starSpacing + 10, cardY + 177, 9, COLOR_GOLD);
    }

    // === POWER RATING - Enhanced with label ===
    // Power background plate
    int pwrPlateW = 120;
    int pwrPlateX = cardX + (cardW - pwrPlateW) / 2;
    gfx->fillRoundRect(pwrPlateX, cardY + 192, pwrPlateW, 42, 6, RGB565(8, 10, 16));
    gfx->drawRoundRect(pwrPlateX, cardY + 192, pwrPlateW, 42, 6, RGB565(40, 45, 60));
    
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(120, 125, 140));
    gfx->setCursor(pwrPlateX + pwrPlateW/2 - 18, cardY + 195);
    gfx->print("POWER");
    
    gfx->setTextSize(3);
    gfx->setTextColor(borderColor);
    char powerStr[10];
    sprintf(powerStr, "%d", card.power_rating);
    int powerLen = strlen(powerStr) * 18;
    gfx->setCursor(pwrPlateX + (pwrPlateW - powerLen) / 2, cardY + 208);
    gfx->print(powerStr);

    // === OWNED COUNT BADGE - Enhanced ===
    int ownedCount = cards_duplicates[cardIdx] + 1;
    char ownedStr[15];
    sprintf(ownedStr, "x%d", ownedCount);
    int ownedBadgeW = strlen(ownedStr) * 6 + 20;
    int ownedBadgeX = cardX + cardW - ownedBadgeW - 8;
    gfx->fillRoundRect(ownedBadgeX, cardY + cardH - 26, ownedBadgeW, 20, 4, RGB565(0, 40, 50));
    gfx->drawRoundRect(ownedBadgeX, cardY + cardH - 26, ownedBadgeW, 20, 4, COLOR_CYAN);
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_CYAN);
    gfx->setCursor(ownedBadgeX + 10, cardY + cardH - 21);
    gfx->print(ownedStr);

    // === EVOLUTION BADGE (if evolved) - Enhanced ===
    if (card.evolution_level > 0) {
      const char* evoName = getEvolutionName(card.evolution_level);
      uint16_t evoColor = getEvolutionColor(card.evolution_level);
      int evoBadgeW = strlen(evoName) * 6 + 16;
      gfx->fillRoundRect(cardX + 8, cardY + cardH - 26, evoBadgeW, 20, 4, evoColor);
      gfx->drawRoundRect(cardX + 8, cardY + cardH - 26, evoBadgeW, 20, 4, COLOR_WHITE);
      gfx->setTextSize(1);
      gfx->setTextColor(RGB565(0, 0, 0));
      gfx->setCursor(cardX + 16, cardY + cardH - 21);
      gfx->print(evoName);
    }
    
    // =============================================================================
    // SELL INFO & BUTTONS - Enhanced with rarity-aware values
    // =============================================================================
    int infoY = cardY + cardH + 6;
    int sellXP = getCardSellXP(card.rarity);
    int sellGems = getCardSellGems(card.rarity);
    
    // Sell value banner with background
    int sellInfoW = cardW;
    int sellInfoX = cardX;
    gfx->fillRoundRect(sellInfoX, infoY, sellInfoW, 20, 4, RGB565(10, 15, 10));
    gfx->drawRoundRect(sellInfoX, infoY, sellInfoW, 20, 4, RGB565(40, 60, 40));
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GREEN);
    gfx->setCursor(sellInfoX + 8, infoY + 6);
    gfx->printf("SELL VALUE: +%d XP", sellXP);
    gfx->setTextColor(COLOR_GOLD);
    // Right-align gems
    char gemsStr[16];
    sprintf(gemsStr, "+%d GEMS", sellGems);
    int gemsLen = strlen(gemsStr) * 6;
    gfx->setCursor(sellInfoX + sellInfoW - gemsLen - 8, infoY + 6);
    gfx->print(gemsStr);
    
    // SELL CARD button - Enhanced with gradient feel
    int sellBtnY = infoY + 24;
    int sellBtnW = 120;
    int sellBtnH = 32;
    int sellBtnX = cardX;
    
    // Button shadow
    gfx->fillRoundRect(sellBtnX + 2, sellBtnY + 2, sellBtnW, sellBtnH, 5, RGB565(30, 0, 0));
    // Button fill
    gfx->fillRoundRect(sellBtnX, sellBtnY, sellBtnW, sellBtnH, 5, RGB565(60, 15, 15));
    // Button border
    gfx->drawRoundRect(sellBtnX, sellBtnY, sellBtnW, sellBtnH, 5, RGB565(220, 60, 60));
    // Highlight top edge
    gfx->drawFastHLine(sellBtnX + 4, sellBtnY + 1, sellBtnW - 8, RGB565(180, 40, 40));
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(255, 100, 100));
    gfx->setCursor(sellBtnX + 12, sellBtnY + 6);
    gfx->print("SELL CARD");
    gfx->setTextColor(RGB565(180, 80, 80));
    gfx->setCursor(sellBtnX + 12, sellBtnY + 18);
    gfx->printf("%dXP +%dG", sellXP, sellGems);
    
    // SELL ALL DUPES button - Enhanced
    int totalDupes = getSellableCardCount(true);
    if (totalDupes > 0) {
        int sellAllBtnW2 = cardW - sellBtnW - 8;
        int sellAllBtnX = cardX + sellBtnW + 8;
        // Button shadow
        gfx->fillRoundRect(sellAllBtnX + 2, sellBtnY + 2, sellAllBtnW2, sellBtnH, 5, RGB565(30, 15, 0));
        // Button fill
        gfx->fillRoundRect(sellAllBtnX, sellBtnY, sellAllBtnW2, sellBtnH, 5, RGB565(55, 30, 10));
        // Button border
        gfx->drawRoundRect(sellAllBtnX, sellBtnY, sellAllBtnW2, sellBtnH, 5, RGB565(255, 160, 50));
        // Highlight
        gfx->drawFastHLine(sellAllBtnX + 4, sellBtnY + 1, sellAllBtnW2 - 8, RGB565(200, 120, 30));
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(255, 180, 80));
        gfx->setCursor(sellAllBtnX + 8, sellBtnY + 6);
        gfx->printf("SELL DUPES (%d)", totalDupes);
        gfx->setTextColor(RGB565(200, 140, 60));
        gfx->setCursor(sellAllBtnX + 8, sellBtnY + 18);
        gfx->print("ALL DUPLICATES");
    }
  }

  // === CARD POSITION INDICATOR - Enhanced ===
  if (filtered_count > 0) {
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(120, 125, 145));
    char posStr[15];
    sprintf(posStr, "%d / %d", collection_view_index + 1, filtered_count);
    int posLen = strlen(posStr) * 6;
    int posY = 388;
    // Arrow indicators
    if (collection_view_index > 0) {
      gfx->setTextColor(theme->accent);
      gfx->setCursor(LCD_WIDTH/2 - posLen/2 - 18, posY);
      gfx->print("<");
    }
    gfx->setTextColor(RGB565(150, 155, 175));
    gfx->setCursor(LCD_WIDTH/2 - posLen/2, posY);
    gfx->print(posStr);
    if (collection_view_index < filtered_count - 1) {
      gfx->setTextColor(theme->accent);
      gfx->setCursor(LCD_WIDTH/2 + posLen/2 + 6, posY);
      gfx->print(">");
    }
  }

  // === SORT BUTTON - Enhanced ===
  const char* sortLabels[] = {"RARITY", "POWER", "NAME", "SERIES"};
  char sortStr[20];
  sprintf(sortStr, "Sort: %s", sortLabels[collection_sort]);
  int sortBtnW = strlen(sortStr) * 6 + 20;

  gfx->fillRoundRect(LCD_WIDTH/2 - sortBtnW/2, 400, sortBtnW, 24, 5, RGB565(15, 18, 28));
  gfx->drawRoundRect(LCD_WIDTH/2 - sortBtnW/2, 400, sortBtnW, 24, 5, RGB565(60, 65, 80));
  gfx->drawFastHLine(LCD_WIDTH/2 - sortBtnW/2 + 3, 401, sortBtnW - 6, theme->accent);
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(170, 175, 195));
  gfx->setCursor(LCD_WIDTH/2 - (strlen(sortStr) * 6) / 2, 408);
  gfx->print(sortStr);

  // === STATS BAR - Enhanced ===
  int statsY = 430;
  gfx->fillRoundRect(10, statsY, LCD_WIDTH - 20, 28, 5, RGB565(10, 12, 18));
  gfx->drawRoundRect(10, statsY, LCD_WIDTH - 20, 28, 5, RGB565(35, 40, 55));

  // Show rarity breakdown with colored dots
  int cCommon, cRare, cEpic, cLegendary, cMythic;
  getCardCountByRarity(&cCommon, &cRare, &cEpic, &cLegendary, &cMythic);
  
  gfx->setTextSize(1);
  int statX = 18;
  
  // Common
  gfx->fillCircle(statX, statsY + 14, 3, RGB565(150, 150, 160));
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setCursor(statX + 6, statsY + 10);
  gfx->printf("%d", cCommon);
  
  // Rare
  statX = 60;
  gfx->fillCircle(statX, statsY + 14, 3, RGB565(30, 144, 255));
  gfx->setTextColor(RGB565(30, 144, 255));
  gfx->setCursor(statX + 6, statsY + 10);
  gfx->printf("%d", cRare);
  
  // Epic
  statX = 100;
  gfx->fillCircle(statX, statsY + 14, 3, RGB565(160, 32, 240));
  gfx->setTextColor(RGB565(160, 32, 240));
  gfx->setCursor(statX + 6, statsY + 10);
  gfx->printf("%d", cEpic);
  
  // Legendary
  statX = 145;
  gfx->fillCircle(statX, statsY + 14, 3, RGB565(255, 215, 0));
  gfx->setTextColor(RGB565(255, 215, 0));
  gfx->setCursor(statX + 6, statsY + 10);
  gfx->printf("%d", cLegendary);
  
  // Mythic
  statX = 195;
  gfx->fillCircle(statX, statsY + 14, 3, RGB565(255, 100, 200));
  gfx->setTextColor(RGB565(255, 100, 200));
  gfx->setCursor(statX + 6, statsY + 10);
  gfx->printf("%d", cMythic);
  
  // Total
  int totalCards = cCommon + cRare + cEpic + cLegendary + cMythic;
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(245, statsY + 10);
  gfx->printf("TOTAL:%d", totalCards);

  // === BOTTOM ROW: SELL ALL + BACK (side by side) ===
  int bottomY = 464;
  
  // SELL ALL CARDS button - Enhanced
  if (totalCards > 0) {
    int sellAllW = LCD_WIDTH/2 - 20;
    int sellAllH = 30;
    int sellAllX = 12;
    
    // Get total value
    int totalXP, totalGems;
    getCollectionValue(&totalXP, &totalGems);
    
    // Button shadow
    gfx->fillRoundRect(sellAllX + 2, bottomY + 2, sellAllW, sellAllH, 5, RGB565(30, 0, 0));
    // Button fill
    gfx->fillRoundRect(sellAllX, bottomY, sellAllW, sellAllH, 5, RGB565(55, 10, 10));
    // Button border
    gfx->drawRoundRect(sellAllX, bottomY, sellAllW, sellAllH, 5, RGB565(200, 50, 50));
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565(220, 70, 70));
    gfx->setCursor(sellAllX + 8, bottomY + 4);
    gfx->print("SELL ALL CARDS");
    gfx->setTextColor(RGB565(160, 80, 80));
    gfx->setCursor(sellAllX + 8, bottomY + 16);
    gfx->printf("%dXP +%dG", totalXP, totalGems);
  }

  // === BACK BUTTON - Enhanced ===
  int backBtnW = LCD_WIDTH/2 - 20;
  int backBtnH = 30;
  int backBtnX = LCD_WIDTH/2 + 8;
  
  gfx->fillRoundRect(backBtnX + 2, bottomY + 2, backBtnW, backBtnH, 5, RGB565(8, 8, 12));
  gfx->fillRoundRect(backBtnX, bottomY, backBtnW, backBtnH, 5, RGB565(18, 22, 32));
  gfx->drawRoundRect(backBtnX, bottomY, backBtnW, backBtnH, 5, RGB565(60, 65, 85));
  gfx->drawFastHLine(backBtnX + 4, bottomY + 1, backBtnW - 8, RGB565(45, 50, 65));
  gfx->setTextColor(RGB565(200, 205, 225));
  gfx->setTextSize(2);
  gfx->setCursor(backBtnX + backBtnW/2 - 24, bottomY + 8);
  gfx->print("BACK");

  // === BOTTOM SWIPE HINT (removed - replaced by BACK button) ===
}

// Global state for gacha reveal
static bool showing_gacha_result = false;
static GachaCard last_revealed_card;

void handleGachaTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;

  int x = gesture.x, y = gesture.y;

  // TAP TO CONTINUE - Handle result screen
  if (showing_gacha_result) {
    // Save the card to collection
    saveGachaCard(last_revealed_card);
    showing_gacha_result = false;
    drawGachaScreen();
    return;
  }

  // Single pull button
  if (y >= 120 && y < 155 && x >= 60 && x < 300) {
    if (canPullSingle()) {
      GachaCard card = performSinglePull();
      last_revealed_card = card;
      drawGachaPullAnimation(card);
      drawGachaRevealImproved(card);
      showing_gacha_result = true;
    }
    return;
  }

  // 10x pull button
  if (y >= 165 && y < 200 && x >= 60 && x < 300) {
    if (canPullTen()) {
      GachaCard results[10];
      performTenPull(results);
      // Save all cards to collection
      for (int i = 0; i < 10; i++) {
        saveGachaCard(results[i]);
      }
      // Show 10-card grid reveal
      drawTenPullGrid(results);
      last_revealed_card = results[9];
      showing_gacha_result = true;
    }
    return;
  }

  // Collection button
  if (y >= 365 && y < 405 && x >= 10 && x < 102) {
    system_state.current_screen = SCREEN_COLLECTION;
    collection_view_index = 0;
    collection_filter = 0;
    collection_sort = 0;
    collection_needs_rebuild = true;
    drawGachaCollection();
    return;
  }

  // Evolve button
  if (y >= 365 && y < 405 && x >= 110 && x < 190) {
    system_state.current_screen = SCREEN_CARD_EVOLUTION;
    drawCardEvolutionScreen();
    return;
  }

  // Deck button
  if (y >= 365 && y < 405 && x >= 198 && x < 270) {
    system_state.current_screen = SCREEN_DECK_BUILDER;
    drawDeckBuilderScreen();
    return;
  }

  // Back button
  if (y >= 365 && y < 405 && x >= 278 && x < 356) {
    system_state.current_screen = SCREEN_GAMES;
    drawGameMenu();
    return;
  }
}

void handleCollectionTouch(TouchGesture& gesture) {
  int x = gesture.x;
  int y = gesture.y;

  // === HANDLE SWIPE GESTURES ===
  if (gesture.event == TOUCH_SWIPE_LEFT) {
    // Next card
    if (filtered_count > 0 && collection_view_index < filtered_count - 1) {
      collection_view_index++;
      drawGachaCollection();
    }
    return;
  }

  if (gesture.event == TOUCH_SWIPE_RIGHT) {
    // Previous card
    if (filtered_count > 0 && collection_view_index > 0) {
      collection_view_index--;
      drawGachaCollection();
    }
    return;
  }

  // === HANDLE TAP GESTURES ===
  if (gesture.event != TOUCH_TAP) return;

  // Filter tabs (y: 55-77)
  if (y >= 55 && y < 77) {
    int tabW = 58;
    int tabSpacing = 5;
    int filterStartX = (LCD_WIDTH - (6 * tabW + 5 * tabSpacing)) / 2;

    for (int f = 0; f < 6; f++) {
      int fx = filterStartX + f * (tabW + tabSpacing);
      if (x >= fx && x < fx + tabW) {
        if (collection_filter != f) {
          collection_filter = f;
          collection_view_index = 0;
          collection_needs_rebuild = true;
          drawGachaCollection();
        }
        return;
      }
    }
  }

  // Sort button (y: 400-424)
  int sortBtnW = 100;
  if (y >= 400 && y < 424 && x >= LCD_WIDTH/2 - sortBtnW/2 && x < LCD_WIDTH/2 + sortBtnW/2) {
    collection_sort = (collection_sort + 1) % 4; // Cycle through sort options
    collection_needs_rebuild = true;
    drawGachaCollection();
    return;
  }

  // Left arrow tap (quick navigation)
  if (x < 60 && y >= 150 && y < 300) {
    if (filtered_count > 0 && collection_view_index > 0) {
      collection_view_index--;
      drawGachaCollection();
    }
    return;
  }

  // Right arrow tap (quick navigation)
  if (x > LCD_WIDTH - 60 && y >= 150 && y < 300) {
    if (filtered_count > 0 && collection_view_index < filtered_count - 1) {
      collection_view_index++;
      drawGachaCollection();
    }
    return;
  }

  // Card tap - visual feedback flash
  int cardW = 280;
  int cardH = 270;
  int cardX = (LCD_WIDTH - cardW) / 2;
  int cardY = 88;
  if (x >= cardX && x < cardX + cardW && y >= cardY && y < cardY + cardH) {
    if (filtered_count > 0) {
      int cardIdx = filtered_cards[collection_view_index];
      // Flash the card
      gfx->fillRoundRect(cardX, cardY, cardW, cardH, 12, getRarityBorderColor(gacha_cards[cardIdx].rarity));
      delay(50);
      drawGachaCollection();
    }
    return;
  }

  // =============================================================================
  // SELL CARD button - Updated coordinates
  // =============================================================================
  int cardX2 = (LCD_WIDTH - 280) / 2;
  int cardH2 = 270;
  int cardY2 = 88;
  int infoY = cardY2 + cardH2 + 6;
  int sellBtnY = infoY + 24;
  int sellBtnH = 32;
  int sellBtnW2 = 120;
  int sellBtnX = cardX2;
  
  if (y >= sellBtnY && y < sellBtnY + sellBtnH && x >= sellBtnX && x < sellBtnX + sellBtnW2) {
      if (filtered_count > 0) {
          int cardIdx = filtered_cards[collection_view_index];
          GachaCard& sellCard_ = gacha_cards[cardIdx];
          int sellXP = getCardSellXP(sellCard_.rarity);
          int sellGems_ = getCardSellGems(sellCard_.rarity);
          
          // Enhanced confirmation flash
          gfx->fillScreen(RGB565(40, 5, 5));
          delay(50);
          
          // Sell the card
          int xpGained = sellCard(cardIdx);
          
          if (xpGained > 0) {
              // Enhanced sell result screen
              gfx->fillScreen(RGB565(5, 20, 8));
              
              // Title
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_WHITE);
              gfx->setCursor(LCD_WIDTH/2 - 48, LCD_HEIGHT/2 - 60);
              gfx->print("CARD SOLD!");
              
              // Card name
              gfx->setTextSize(1);
              gfx->setTextColor(getRarityBorderColor(sellCard_.rarity));
              int nameLen2 = sellCard_.character_name.length() * 6;
              gfx->setCursor(LCD_WIDTH/2 - nameLen2/2, LCD_HEIGHT/2 - 35);
              gfx->print(sellCard_.character_name);
              
              // XP reward
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_GREEN);
              gfx->setCursor(LCD_WIDTH/2 - 55, LCD_HEIGHT/2 - 10);
              gfx->printf("+%d XP", sellXP);
              
              // Gems reward
              gfx->setTextColor(COLOR_GOLD);
              gfx->setCursor(LCD_WIDTH/2 - 60, LCD_HEIGHT/2 + 15);
              gfx->printf("+%d GEMS", sellGems_);
              
              // Rarity label
              gfx->setTextSize(1);
              gfx->setTextColor(RGB565(100, 105, 120));
              const char* rarityStr = getRarityName(sellCard_.rarity);
              int rarLen = strlen(rarityStr) * 6;
              gfx->setCursor(LCD_WIDTH/2 - rarLen/2, LCD_HEIGHT/2 + 45);
              gfx->print(rarityStr);
              
              delay(600);
              
              // Rebuild and redraw
              collection_needs_rebuild = true;
              drawGachaCollection();
          }
      }
      return;
  }
  
  // =============================================================================
  // SELL ALL DUPLICATES button - Updated coordinates
  // =============================================================================
  int cardW2 = 280;
  int sellAllBtnW3 = cardW2 - 120 - 8;
  int sellAllBtnX = cardX2 + 120 + 8;
  
  if (y >= sellBtnY && y < sellBtnY + sellBtnH && x >= sellAllBtnX && x < sellAllBtnX + sellAllBtnW3) {
      int totalDupes = getSellableCardCount(true);
      
      if (totalDupes > 0) {
          // Calculate per-rarity breakdown BEFORE selling
          int dupesByRarity[5] = {0, 0, 0, 0, 0};
          int xpByRarity[5] = {0, 0, 0, 0, 0};
          int gemsByRarity[5] = {0, 0, 0, 0, 0};
          int previewXP = 0;
          int previewGems = 0;
          
          for (int i = 0; i < GACHA_TOTAL_CARDS; i++) {
              if (cards_owned[i] && cards_duplicates[i] > 0) {
                  int d = cards_duplicates[i];
                  int r = min((int)gacha_cards[i].rarity, 4);
                  int xp = d * getCardSellXP(gacha_cards[i].rarity);
                  int gems = d * getCardSellGems(gacha_cards[i].rarity);
                  dupesByRarity[r] += d;
                  xpByRarity[r] += xp;
                  gemsByRarity[r] += gems;
                  previewXP += xp;
                  previewGems += gems;
              }
          }
          
          // Flash
          gfx->fillScreen(RGB565(40, 25, 5));
          delay(50);
          
          // Sell all duplicates
          int xpGained = sellAllDuplicates();
          
          if (xpGained > 0) {
              // Enhanced rewards screen with RARITY BREAKDOWN
              gfx->fillScreen(RGB565(5, 18, 8));
              
              // Border
              gfx->drawRoundRect(5, 5, LCD_WIDTH - 10, LCD_HEIGHT - 10, 8, COLOR_GREEN);
              
              // Title
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_WHITE);
              gfx->setCursor(LCD_WIDTH/2 - 72, 30);
              gfx->print("DUPES SOLD!");
              
              gfx->setTextSize(1);
              gfx->setTextColor(RGB565(160, 165, 180));
              gfx->setCursor(LCD_WIDTH/2 - 55, 55);
              gfx->printf("%d duplicates sold", totalDupes);
              
              // === RARITY BREAKDOWN TABLE ===
              int tableY = 80;
              const char* rarNames[] = {"Common", "Rare", "Epic", "Legendary", "Mythic"};
              uint16_t rarColors[] = {
                RGB565(150, 150, 160), RGB565(30, 144, 255), RGB565(160, 32, 240),
                RGB565(255, 215, 0), RGB565(255, 100, 200)
              };
              
              // Table header
              gfx->fillRect(20, tableY, LCD_WIDTH - 40, 16, RGB565(10, 14, 10));
              gfx->setTextColor(RGB565(100, 105, 120));
              gfx->setCursor(28, tableY + 4);
              gfx->print("RARITY      QTY    XP       GEMS");
              tableY += 20;
              
              for (int r = 4; r >= 0; r--) {  // Mythic first
                if (dupesByRarity[r] > 0) {
                  gfx->fillRect(20, tableY, LCD_WIDTH - 40, 18, RGB565(8, 12, 8));
                  gfx->drawFastHLine(20, tableY + 17, LCD_WIDTH - 40, RGB565(15, 25, 15));
                  
                  gfx->setTextColor(rarColors[r]);
                  gfx->setCursor(28, tableY + 5);
                  gfx->printf("%-10s", rarNames[r]);
                  
                  gfx->setTextColor(COLOR_WHITE);
                  gfx->setCursor(100, tableY + 5);
                  gfx->printf("x%-3d", dupesByRarity[r]);
                  
                  gfx->setTextColor(COLOR_GREEN);
                  gfx->setCursor(145, tableY + 5);
                  gfx->printf("+%d", xpByRarity[r]);
                  
                  gfx->setTextColor(COLOR_GOLD);
                  gfx->setCursor(230, tableY + 5);
                  gfx->printf("+%d", gemsByRarity[r]);
                  
                  tableY += 20;
                }
              }
              
              // Separator
              tableY += 4;
              gfx->drawFastHLine(20, tableY, LCD_WIDTH - 40, COLOR_GREEN);
              tableY += 8;
              
              // TOTALS
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_GREEN);
              gfx->setCursor(LCD_WIDTH/2 - 55, tableY);
              gfx->printf("+%d XP", xpGained);
              
              gfx->setTextColor(COLOR_GOLD);
              gfx->setCursor(LCD_WIDTH/2 - 60, tableY + 28);
              gfx->printf("+%d GEMS", previewGems);
              
              delay(1200);
              
              collection_needs_rebuild = true;
              drawGachaCollection();
          }
      }
      return;
  }

  // =============================================================================
  // SELL ALL CARDS button (DANGER!) - Updated coordinates
  // =============================================================================
  int sellAllCardsX = 12;
  int sellAllCardsY = 464;
  int sellAllCardsW = LCD_WIDTH/2 - 20;
  int sellAllCardsH = 30;
  
  if (y >= sellAllCardsY && y < sellAllCardsY + sellAllCardsH && 
      x >= sellAllCardsX && x < sellAllCardsX + sellAllCardsW) {
      int totalCards = getSellableCardCount(false);
      
      if (totalCards > 0) {
          // Get total value before selling
          int totalXP, totalGems;
          getCollectionValue(&totalXP, &totalGems);
          
          // Enhanced warning screen
          gfx->fillScreen(RGB565(50, 0, 0));
          
          // Warning border
          for (int b = 0; b < 4; b++) {
            gfx->drawRect(b, b, LCD_WIDTH - b*2, LCD_HEIGHT - b*2, COLOR_RED);
          }
          
          gfx->setTextSize(2);
          gfx->setTextColor(COLOR_WHITE);
          gfx->setCursor(LCD_WIDTH/2 - 72, LCD_HEIGHT/2 - 55);
          gfx->print("SELLING ALL!");
          
          gfx->setTextSize(1);
          gfx->setTextColor(COLOR_RED);
          gfx->setCursor(LCD_WIDTH/2 - 42, LCD_HEIGHT/2 - 30);
          gfx->printf("%d CARDS TOTAL", totalCards);
          
          gfx->setTextColor(RGB565(150, 150, 160));
          gfx->setCursor(LCD_WIDTH/2 - 60, LCD_HEIGHT/2 - 15);
          gfx->print("This cannot be undone!");
          
          delay(400);
          
          // Sell entire collection
          int xpGained = sellEntireCollection();
          
          if (xpGained > 0) {
              // Enhanced massive rewards screen
              gfx->fillScreen(RGB565(5, 25, 5));
              
              // Gold border
              for (int b = 0; b < 3; b++) {
                gfx->drawRect(b, b, LCD_WIDTH - b*2, LCD_HEIGHT - b*2, COLOR_GOLD);
              }
              
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_GOLD);
              gfx->setCursor(LCD_WIDTH/2 - 96, LCD_HEIGHT/2 - 50);
              gfx->print("COLLECTION SOLD!");
              
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_GREEN);
              gfx->setCursor(LCD_WIDTH/2 - 55, LCD_HEIGHT/2 - 15);
              gfx->printf("+%d XP", xpGained);
              
              gfx->setTextSize(2);
              gfx->setTextColor(COLOR_GOLD);
              gfx->setCursor(LCD_WIDTH/2 - 60, LCD_HEIGHT/2 + 15);
              gfx->printf("+%d GEMS", totalGems);
              
              delay(1200);
              
              collection_needs_rebuild = true;
              drawGachaCollection();
          }
      }
      return;
  }

  // Back button - Updated coordinates
  int backBtnX = LCD_WIDTH/2 + 8;
  int backBtnW = LCD_WIDTH/2 - 20;
  int backBtnY = 464;
  int backBtnH = 30;
  if (y >= backBtnY && y < backBtnY + backBtnH && x >= backBtnX && x < backBtnX + backBtnW) {
    // Reset view state before leaving
    collection_view_index = 0;
    collection_filter = 0;
    collection_sort = 0;
    collection_needs_rebuild = true;

    system_state.current_screen = SCREEN_GACHA;
    drawGachaScreen();
    return;
  }
}

// =============================================================================
// PITY SYSTEM
// =============================================================================

void drawPityCounter(int x, int y) {
  ThemeColors* theme = getCurrentTheme();

  // Pity counter panel - retro terminal style
  gfx->fillRect(x, y, LCD_WIDTH - 40, 50, RGB565(8, 10, 14));
  gfx->drawRect(x, y, LCD_WIDTH - 40, 50, RGB565(30, 35, 50));
  gfx->fillRect(x, y, 4, 4, theme->accent);

  gfx->setTextSize(1);

  // Epic pity bar
  int epicPulls = system_state.pity_counter;
  float epicProgress = (float)epicPulls / PITY_EPIC_GUARANTEE;
  if (epicProgress > 1.0f) epicProgress = 1.0f;

  gfx->setTextColor(COLOR_PURPLE);
  gfx->setCursor(x + 6, y + 6);
  gfx->printf("EPIC PITY: %d/%d", epicPulls, PITY_EPIC_GUARANTEE);

  // Epic bar
  int barW = LCD_WIDTH - 60;
  gfx->fillRect(x + 6, y + 18, barW, 6, RGB565(15, 12, 22));
  gfx->drawRect(x + 6, y + 18, barW, 6, RGB565(30, 25, 45));
  int epicFill = (int)(barW * epicProgress);
  if (epicFill > 0) gfx->fillRect(x + 7, y + 19, epicFill - 2, 4, COLOR_PURPLE);

  // Legendary pity bar
  int legPulls = system_state.pity_legendary_counter;
  float legProgress = (float)legPulls / PITY_LEGENDARY_GUARANTEE;
  if (legProgress > 1.0f) legProgress = 1.0f;

  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(x + 6, y + 28);
  gfx->printf("LEGEND PITY: %d/%d", legPulls, PITY_LEGENDARY_GUARANTEE);

  // Legendary bar
  gfx->fillRect(x + 6, y + 40, barW, 6, RGB565(20, 18, 8));
  gfx->drawRect(x + 6, y + 40, barW, 6, RGB565(40, 35, 15));
  int legFill = (int)(barW * legProgress);
  if (legFill > 0) gfx->fillRect(x + 7, y + 41, legFill - 2, 4, COLOR_GOLD);
}

// =============================================================================
// CARD EVOLUTION
// =============================================================================

const char* getEvolutionName(int level) {
  switch (level) {
    case 0: return "BASE";
    case 1: return "EVOLVED";
    case 2: return "AWAKENED";
    case 3: return "TRANSCENDED";
    default: return "BASE";
  }
}

uint16_t getEvolutionColor(int level) {
  switch (level) {
    case 0: return RGB565(130, 135, 150);
    case 1: return RGB565(80, 200, 255);
    case 2: return COLOR_GOLD;
    case 3: return COLOR_PINK;
    default: return RGB565(130, 135, 150);
  }
}

int getEvolveCost(int currentLevel) {
  switch (currentLevel) {
    case 0: return EVOLVE_COST_LV1;
    case 1: return EVOLVE_COST_LV2;
    case 2: return EVOLVE_COST_LV3;
    default: return 999;
  }
}

float getEvolvePowerMult(int level) {
  switch (level) {
    case 1: return EVOLVE_POWER_MULT_LV1;
    case 2: return EVOLVE_POWER_MULT_LV2;
    case 3: return EVOLVE_POWER_MULT_LV3;
    default: return 1.0f;
  }
}

bool evolveCard(int cardIndex) {
  if (cardIndex < 0 || cardIndex >= GACHA_TOTAL_CARDS) return false;
  if (!cards_owned[cardIndex]) return false;
  if (gacha_cards[cardIndex].evolution_level >= 3) return false;

  int cost = getEvolveCost(gacha_cards[cardIndex].evolution_level);
  if (cards_duplicates[cardIndex] < cost) return false;

  // Consume duplicates
  cards_duplicates[cardIndex] -= cost;
  gacha_cards[cardIndex].evolution_level++;

  // Apply power boost
  float mult = getEvolvePowerMult(gacha_cards[cardIndex].evolution_level);
  gacha_cards[cardIndex].power_rating = (int)(gacha_cards[cardIndex].power * mult);
  gacha_cards[cardIndex].hp = (int)(gacha_cards[cardIndex].hp * 1.2f);
  gacha_cards[cardIndex].attack = (int)(gacha_cards[cardIndex].attack * 1.2f);
  gacha_cards[cardIndex].defense = (int)(gacha_cards[cardIndex].defense * 1.2f);

  Serial.printf("[Evolve] %s -> %s (PWR: %d)\n",
                gacha_cards[cardIndex].character_name.c_str(),
                getEvolutionName(gacha_cards[cardIndex].evolution_level),
                gacha_cards[cardIndex].power_rating);

  saveGachaProgress();
  return true;
}

// Evolution selection state
static int evo_scroll_offset = 0;
static int evo_selected_card = -1;

void drawCardEvolutionScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;

  // Header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_GOLD);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(centerX - 54, 8);
  gfx->print("EVOLUTION");
  
  // Evolution path legend - color coded
  gfx->setTextSize(1);
  int legendY = 30;
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setCursor(15, legendY);
  gfx->print("BASE");
  gfx->fillRect(45, legendY + 3, 12, 4, RGB565(80, 85, 100));
  gfx->setTextColor(getEvolutionColor(1));
  gfx->setCursor(62, legendY);
  gfx->print("EVO");
  gfx->fillRect(85, legendY + 3, 12, 4, getEvolutionColor(1));
  gfx->setTextColor(getEvolutionColor(2));
  gfx->setCursor(102, legendY);
  gfx->print("AWAKE");
  gfx->fillRect(135, legendY + 3, 12, 4, getEvolutionColor(2));
  gfx->setTextColor(getEvolutionColor(3));
  gfx->setCursor(152, legendY);
  gfx->print("MAX");

  // Card list - show owned cards
  int cardY = 55;
  int shown = 0;
  int rowH = 94;

  for (int i = 0; i < GACHA_TOTAL_CARDS && shown < 4; i++) {
    if (!cards_owned[i]) continue;
    if (shown < evo_scroll_offset) { shown++; continue; }

    int cy = cardY + (shown - evo_scroll_offset) * rowH;
    bool selected = (i == evo_selected_card);
    int evoLvl = gacha_cards[i].evolution_level;
    int dupes = cards_duplicates[i];
    int cost = getEvolveCost(evoLvl);
    bool canEvo = (evoLvl < 3 && dupes >= cost);
    float powerMult = getEvolvePowerMult(evoLvl + 1);

    // Card row background
    uint16_t bg = selected ? RGB565(18, 22, 32) : RGB565(12, 14, 20);
    uint16_t border = selected ? getEvolutionColor(evoLvl) : RGB565(40, 45, 60);
    gfx->fillRect(8, cy, LCD_WIDTH - 16, rowH - 4, bg);
    gfx->drawRect(8, cy, LCD_WIDTH - 16, rowH - 4, border);

    // Left evolution color bar
    gfx->fillRect(8, cy, 5, rowH - 4, getEvolutionColor(evoLvl));

    // Card portrait swatch
    gfx->fillRect(18, cy + 8, 52, 68, gacha_cards[i].card_color);
    gfx->drawRect(18, cy + 8, 52, 68, border);
    for (int sy = cy + 10; sy < cy + 74; sy += 3) {
      gfx->drawFastHLine(19, sy, 50, RGB565(0, 0, 0));
    }
    // Character initial
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(3);
    char init[2] = {gacha_cards[i].character_name[0], 0};
    gfx->setCursor(31, cy + 18);
    gfx->print(init);

    // Rarity stars under portrait
    int starCount = getRarityStars(gacha_cards[i].rarity);
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GOLD);
    for (int s = 0; s < starCount; s++) {
      gfx->setCursor(20 + s * 10, cy + 62);
      gfx->print("*");
    }

    // Character name
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(78, cy + 8);
    gfx->print(gacha_cards[i].character_name);

    // Evolution level badge + power
    gfx->setTextSize(1);
    gfx->setTextColor(getEvolutionColor(evoLvl));
    gfx->setCursor(78, cy + 28);
    gfx->printf("[%s]", getEvolutionName(evoLvl));

    gfx->setTextColor(theme->accent);
    gfx->setCursor(180, cy + 28);
    gfx->printf("PWR: %d", gacha_cards[i].power_rating);

    // Dupe progress bar
    if (evoLvl < 3) {
      int barX = 78;
      int barY2 = cy + 44;
      int barW = 140;
      int barH = 12;
      float progress = (float)min(dupes, cost) / (float)cost;
      
      gfx->fillRect(barX, barY2, barW, barH, RGB565(15, 18, 25));
      gfx->drawRect(barX, barY2, barW, barH, RGB565(40, 45, 60));
      int fillW = (int)(barW * progress);
      if (fillW > 0) {
        uint16_t fillColor = canEvo ? RGB565(0, 200, 80) : RGB565(200, 150, 30);
        gfx->fillRect(barX + 1, barY2 + 1, fillW - 2, barH - 2, fillColor);
        gfx->drawFastHLine(barX + 1, barY2 + 1, fillW - 2, COLOR_WHITE); // Highlight
      }
      
      gfx->setTextColor(canEvo ? COLOR_GREEN : RGB565(160, 130, 50));
      gfx->setCursor(barX + barW + 6, barY2 + 2);
      gfx->printf("%d/%d", min(dupes, cost), cost);
      
      // Next level info
      gfx->setTextColor(RGB565(100, 105, 120));
      gfx->setCursor(78, cy + 62);
      gfx->printf("Next: %s (x%.1f PWR)", getEvolutionName(evoLvl + 1), powerMult);
    } else {
      gfx->setTextColor(RGB565(255, 100, 200));
      gfx->setTextSize(1);
      gfx->setCursor(78, cy + 44);
      gfx->print("MAXIMUM EVOLUTION!");
      gfx->setTextColor(RGB565(100, 105, 120));
      gfx->setCursor(78, cy + 58);
      gfx->print("This card is fully powered up");
    }

    // EVOLVE BUTTON
    if (canEvo) {
      int btnX = LCD_WIDTH - 88;
      int btnY2 = cy + 12;
      int btnW = 75;
      int btnH2 = 60;
      gfx->fillRect(btnX, btnY2, btnW, btnH2, RGB565(15, 40, 12));
      gfx->drawRect(btnX, btnY2, btnW, btnH2, RGB565(0, 220, 80));
      gfx->fillRect(btnX, btnY2, 4, 4, RGB565(0, 220, 80));
      gfx->fillRect(btnX + btnW - 4, btnY2, 4, 4, RGB565(0, 220, 80));
      gfx->setTextColor(RGB565(0, 255, 100));
      gfx->setTextSize(2);
      gfx->setCursor(btnX + 6, btnY2 + 8);
      gfx->print("EVO!");
      gfx->setTextSize(1);
      gfx->setTextColor(RGB565(0, 180, 60));
      gfx->setCursor(btnX + 8, btnY2 + 30);
      gfx->printf("-%d dupe", cost);
      gfx->setCursor(btnX + 8, btnY2 + 42);
      gfx->printf("x%.1f PWR", powerMult);
    } else if (evoLvl >= 3) {
      int btnX = LCD_WIDTH - 80;
      gfx->fillRect(btnX, cy + 22, 68, 40, RGB565(25, 15, 30));
      gfx->drawRect(btnX, cy + 22, 68, 40, RGB565(255, 100, 200));
      gfx->fillRect(btnX, cy + 22, 4, 4, RGB565(255, 100, 200));
      gfx->setTextColor(RGB565(255, 100, 200));
      gfx->setTextSize(2);
      gfx->setCursor(btnX + 14, cy + 32);
      gfx->print("MAX");
    }

    shown++;
  }

  if (shown == 0) {
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(2);
    gfx->setCursor(centerX - 72, 180);
    gfx->print("NO CARDS");
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 60, 210);
    gfx->print("Pull cards from Gacha!");
  }

  // Scroll hint
  if (getCardsOwned() > 4) {
    gfx->setTextColor(RGB565(60, 65, 80));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 45, 425);
    gfx->print("SWIPE FOR MORE");
  }

  // Back button
  int backX = centerX - 50;
  gfx->fillRect(backX, 445, 100, 32, RGB565(18, 18, 25));
  gfx->drawRect(backX, 445, 100, 32, RGB565(60, 65, 80));
  gfx->fillRect(backX, 445, 5, 5, RGB565(60, 65, 80));
  gfx->setTextColor(RGB565(200, 205, 225));
  gfx->setTextSize(2);
  gfx->setCursor(backX + 22, 451);
  gfx->print("BACK");
}

void handleCardEvolutionTap(int x, int y) {
  // Check evolve buttons
  int cardY = 55;
  int shown = 0;
  int rowH = 94;

  for (int i = 0; i < GACHA_TOTAL_CARDS && shown < 4; i++) {
    if (!cards_owned[i]) continue;
    if (shown < evo_scroll_offset) { shown++; continue; }

    int cy = cardY + (shown - evo_scroll_offset) * rowH;

    // Evolve button hit - larger area
    if (x >= LCD_WIDTH - 88 && x < LCD_WIDTH - 8 && y >= cy + 14 && y < cy + 70) {
      if (gacha_cards[i].evolution_level < 3) {
        if (evolveCard(i)) {
          // Enhanced evolution flash
          uint16_t evoColor = getEvolutionColor(gacha_cards[i].evolution_level);
          gfx->fillScreen(evoColor);
          delay(100);
          gfx->fillScreen(RGB565(2, 2, 5));
          
          // Show result briefly
          gfx->setTextSize(2);
          gfx->setTextColor(evoColor);
          gfx->setCursor(LCD_WIDTH/2 - 54, LCD_HEIGHT/2 - 20);
          gfx->print("EVOLVED!");
          gfx->setTextSize(1);
          gfx->setTextColor(COLOR_WHITE);
          gfx->setCursor(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 + 10);
          gfx->printf("Now: %s", getEvolutionName(gacha_cards[i].evolution_level));
          delay(400);
        }
      }
      drawCardEvolutionScreen();
      return;
    }

    shown++;
  }

  // Back button - updated position
  if (y >= 445 && y < 477 && x >= LCD_WIDTH/2 - 50 && x < LCD_WIDTH/2 + 50) {
    system_state.current_screen = SCREEN_GACHA;
    drawGachaScreen();
    return;
  }
}

// =============================================================================
// BATTLE DECK BUILDER
// =============================================================================

static int deck_scroll_offset = 0;

bool addCardToDeck(int cardIndex) {
  if (cardIndex < 0 || cardIndex >= GACHA_TOTAL_CARDS) return false;
  if (!cards_owned[cardIndex]) return false;
  if (system_state.deck_size >= MAX_DECK_SIZE) return false;

  // Check if already in deck
  for (int i = 0; i < system_state.deck_size; i++) {
    if (system_state.battle_deck[i] == cardIndex) return false;
  }

  system_state.battle_deck[system_state.deck_size] = cardIndex;
  system_state.deck_size++;
  Serial.printf("[Deck] Added %s to deck (slot %d)\n",
                gacha_cards[cardIndex].character_name.c_str(), system_state.deck_size);
  return true;
}

bool removeCardFromDeck(int slotIndex) {
  if (slotIndex < 0 || slotIndex >= system_state.deck_size) return false;

  // Shift cards down
  for (int i = slotIndex; i < system_state.deck_size - 1; i++) {
    system_state.battle_deck[i] = system_state.battle_deck[i + 1];
  }
  system_state.deck_size--;
  system_state.battle_deck[system_state.deck_size] = -1;
  return true;
}

int getDeckTotalPower() {
  int total = 0;
  for (int i = 0; i < system_state.deck_size; i++) {
    int idx = system_state.battle_deck[i];
    if (idx >= 0 && idx < GACHA_TOTAL_CARDS && cards_owned[idx]) {
      total += gacha_cards[idx].power_rating;
    }
  }
  return total;
}

int getDeckBonusATK() {
  int bonus = 0;
  for (int i = 0; i < system_state.deck_size; i++) {
    int idx = system_state.battle_deck[i];
    if (idx >= 0 && idx < GACHA_TOTAL_CARDS && cards_owned[idx]) {
      bonus += DECK_ATK_BONUS_PER_CARD + gacha_cards[idx].attack / 10;
    }
  }
  return bonus;
}

int getDeckBonusHP() {
  int bonus = 0;
  for (int i = 0; i < system_state.deck_size; i++) {
    int idx = system_state.battle_deck[i];
    if (idx >= 0 && idx < GACHA_TOTAL_CARDS && cards_owned[idx]) {
      bonus += DECK_HP_BONUS_PER_CARD + gacha_cards[idx].hp / 10;
    }
  }
  return bonus;
}

int getDeckBonusDEF() {
  int bonus = 0;
  for (int i = 0; i < system_state.deck_size; i++) {
    int idx = system_state.battle_deck[i];
    if (idx >= 0 && idx < GACHA_TOTAL_CARDS && cards_owned[idx]) {
      bonus += DECK_DEF_BONUS_PER_CARD + gacha_cards[idx].defense / 10;
    }
  }
  return bonus;
}

void drawDeckBuilderScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }

  ThemeColors* theme = getCurrentTheme();
  int centerX = LCD_WIDTH / 2;

  // === HEADER ===
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_CYAN);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(centerX - 66, 8);
  gfx->print("BATTLE DECK");
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(100, 110, 130));
  gfx->setCursor(centerX - 80, 30);
  gfx->print("Build your team for Boss Rush!");

  // === DECK SLOTS - YOUR TEAM ===
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(15, 54);
  gfx->printf("YOUR TEAM  %d / %d", system_state.deck_size, MAX_DECK_SIZE);

  int slotW = 72, slotH = 95;
  int slotSpacing = 5;
  int slotStartX = (LCD_WIDTH - (MAX_DECK_SIZE * slotW + (MAX_DECK_SIZE - 1) * slotSpacing)) / 2;
  int slotY = 68;

  for (int i = 0; i < MAX_DECK_SIZE; i++) {
    int sx = slotStartX + i * (slotW + slotSpacing);

    if (i < system_state.deck_size && system_state.battle_deck[i] >= 0) {
      int cardIdx = system_state.battle_deck[i];
      uint16_t evoColor = getEvolutionColor(gacha_cards[cardIdx].evolution_level);
      uint16_t rarColor = getRarityBorderColor(gacha_cards[cardIdx].rarity);
      
      // Card background
      gfx->fillRect(sx, slotY, slotW, slotH, gacha_cards[cardIdx].card_color);
      gfx->drawRect(sx, slotY, slotW, slotH, rarColor);
      gfx->drawRect(sx + 1, slotY + 1, slotW - 2, slotH - 2, rarColor);
      
      // Scanlines
      for (int sy = slotY + 2; sy < slotY + slotH - 2; sy += 3) {
        gfx->drawFastHLine(sx + 2, sy, slotW - 4, RGB565(0, 0, 0));
      }
      
      // Character initial - big
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(3);
      char init[2] = {gacha_cards[cardIdx].character_name[0], 0};
      gfx->setCursor(sx + slotW/2 - 9, slotY + 8);
      gfx->print(init);
      
      // Rarity stars
      int starCount = getRarityStars(gacha_cards[cardIdx].rarity);
      gfx->setTextSize(1);
      gfx->setTextColor(COLOR_GOLD);
      for (int s = 0; s < starCount; s++) {
        gfx->setCursor(sx + (slotW - starCount * 8) / 2 + s * 8, slotY + 34);
        gfx->print("*");
      }
      
      // Name (truncated)
      gfx->setTextSize(1);
      gfx->setTextColor(COLOR_WHITE);
      String nm = gacha_cards[cardIdx].character_name;
      if (nm.length() > 9) nm = nm.substring(0, 8) + ".";
      int nmLen = nm.length() * 6;
      gfx->setCursor(sx + (slotW - nmLen) / 2, slotY + 48);
      gfx->print(nm);
      
      // Power rating
      gfx->setTextColor(COLOR_GOLD);
      gfx->setTextSize(1);
      char pwrBuf[10];
      sprintf(pwrBuf, "PWR:%d", gacha_cards[cardIdx].power_rating);
      int pLen = strlen(pwrBuf) * 6;
      gfx->setCursor(sx + (slotW - pLen) / 2, slotY + 62);
      gfx->print(pwrBuf);
      
      // Evolution badge
      gfx->setTextColor(evoColor);
      const char* eName = getEvolutionName(gacha_cards[cardIdx].evolution_level);
      int eLen = strlen(eName) * 6;
      gfx->setCursor(sx + (slotW - eLen) / 2, slotY + 76);
      gfx->print(eName);
      
      // Remove X button (top-right corner)
      gfx->fillRect(sx + slotW - 16, slotY + 2, 14, 14, RGB565(120, 20, 20));
      gfx->drawRect(sx + slotW - 16, slotY + 2, 14, 14, COLOR_RED);
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(1);
      gfx->setCursor(sx + slotW - 13, slotY + 5);
      gfx->print("X");
    } else {
      // Empty slot
      gfx->fillRect(sx, slotY, slotW, slotH, RGB565(8, 10, 14));
      gfx->drawRect(sx, slotY, slotW, slotH, RGB565(30, 35, 50));
      // Dashed center
      for (int d = slotY + 12; d < slotY + slotH - 12; d += 8) {
        gfx->drawFastHLine(sx + slotW/2 - 1, d, 3, RGB565(40, 45, 60));
      }
      gfx->setTextColor(RGB565(50, 55, 70));
      gfx->setTextSize(3);
      gfx->setCursor(sx + slotW/2 - 9, slotY + slotH/2 - 12);
      gfx->print("+");
    }
  }

  // === STATS PANEL ===
  int statsY = slotY + slotH + 8;
  gfx->fillRect(10, statsY, LCD_WIDTH - 20, 52, RGB565(10, 12, 20));
  gfx->drawRect(10, statsY, LCD_WIDTH - 20, 52, RGB565(35, 40, 55));
  gfx->fillRect(10, statsY, 5, 5, COLOR_CYAN);

  int totalPwr = getDeckTotalPower();
  int bonusATK = getDeckBonusATK();
  int bonusHP = getDeckBonusHP();
  int bonusDEF = getDeckBonusDEF();

  // Total power
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setCursor(20, statsY + 6);
  gfx->printf("POWER: %d", totalPwr);

  // Combat bonuses
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setCursor(20, statsY + 28);
  gfx->print("BOSS BONUSES:");
  
  gfx->setTextColor(RGB565(255, 100, 100));
  gfx->setCursor(110, statsY + 28);
  gfx->printf("+%d ATK", bonusATK);
  
  gfx->setTextColor(RGB565(100, 255, 100));
  gfx->setCursor(190, statsY + 28);
  gfx->printf("+%d HP", bonusHP);
  
  gfx->setTextColor(RGB565(100, 150, 255));
  gfx->setCursor(270, statsY + 28);
  gfx->printf("+%d DEF", bonusDEF);

  if (system_state.deck_size >= MAX_DECK_SIZE) {
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(20, statsY + 40);
    gfx->print("FULL TEAM BONUS ACTIVE!");
  }

  // === AVAILABLE CARDS ===
  int listY = statsY + 60;
  gfx->setTextColor(RGB565(140, 145, 165));
  gfx->setTextSize(1);
  gfx->setCursor(15, listY);
  gfx->print("AVAILABLE CARDS:");
  listY += 16;

  int shown = 0;
  for (int i = 0; i < GACHA_TOTAL_CARDS && shown < 3; i++) {
    if (!cards_owned[i]) continue;

    bool inDeck = false;
    for (int d = 0; d < system_state.deck_size; d++) {
      if (system_state.battle_deck[d] == i) { inDeck = true; break; }
    }
    if (inDeck) continue;

    if (shown < deck_scroll_offset) { shown++; continue; }

    int cy = listY + (shown - deck_scroll_offset) * 50;

    // Card row
    gfx->fillRect(12, cy, LCD_WIDTH - 24, 44, RGB565(12, 14, 22));
    gfx->drawRect(12, cy, LCD_WIDTH - 24, 44, getRarityBorderColor(gacha_cards[i].rarity));
    
    // Left rarity stripe
    gfx->fillRect(12, cy, 5, 44, getRarityBorderColor(gacha_cards[i].rarity));

    // Mini card swatch
    gfx->fillRect(22, cy + 4, 36, 36, gacha_cards[i].card_color);
    for (int sy = cy + 5; sy < cy + 39; sy += 3) {
      gfx->drawFastHLine(23, sy, 34, RGB565(0, 0, 0));
    }
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    char init2[2] = {gacha_cards[i].character_name[0], 0};
    gfx->setCursor(30, cy + 12);
    gfx->print(init2);

    // Info
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(1);
    gfx->setCursor(65, cy + 8);
    gfx->print(gacha_cards[i].character_name);

    gfx->setTextColor(getRarityBorderColor(gacha_cards[i].rarity));
    gfx->setCursor(65, cy + 20);
    gfx->printf("%s | PWR:%d", getRarityName(gacha_cards[i].rarity), gacha_cards[i].power_rating);
    
    // Evo level
    gfx->setTextColor(getEvolutionColor(gacha_cards[i].evolution_level));
    gfx->setCursor(65, cy + 32);
    gfx->printf("[%s]", getEvolutionName(gacha_cards[i].evolution_level));

    // ADD button
    if (system_state.deck_size < MAX_DECK_SIZE) {
      int addX = LCD_WIDTH - 65;
      gfx->fillRect(addX, cy + 8, 48, 28, RGB565(8, 25, 22));
      gfx->drawRect(addX, cy + 8, 48, 28, COLOR_CYAN);
      gfx->fillRect(addX, cy + 8, 4, 4, COLOR_CYAN);
      gfx->setTextColor(COLOR_CYAN);
      gfx->setTextSize(2);
      gfx->setCursor(addX + 6, cy + 14);
      gfx->print("ADD");
    }

    shown++;
  }

  if (shown == 0) {
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(centerX - 65, listY + 20);
    gfx->print("No more cards available");
  }

  // Back button
  int backX = centerX - 50;
  gfx->fillRect(backX, 455, 100, 32, RGB565(18, 18, 25));
  gfx->drawRect(backX, 455, 100, 32, RGB565(60, 65, 80));
  gfx->fillRect(backX, 455, 5, 5, RGB565(60, 65, 80));
  gfx->setTextColor(RGB565(200, 205, 225));
  gfx->setTextSize(2);
  gfx->setCursor(backX + 22, 461);
  gfx->print("BACK");
}

void handleDeckBuilderTap(int x, int y) {
  // Check deck slot remove buttons (X buttons)
  int slotW = 72, slotH = 95;
  int slotSpacing = 5;
  int slotStartX = (LCD_WIDTH - (MAX_DECK_SIZE * slotW + (MAX_DECK_SIZE - 1) * slotSpacing)) / 2;
  int slotY = 68;

  for (int i = 0; i < system_state.deck_size; i++) {
    int sx = slotStartX + i * (slotW + slotSpacing);
    // X button: top-right of slot
    if (x >= sx + slotW - 16 && x < sx + slotW && y >= slotY && y < slotY + 18) {
      removeCardFromDeck(i);
      drawDeckBuilderScreen();
      return;
    }
  }

  // Check add buttons in available cards list
  int statsY = slotY + slotH + 8 + 52 + 60;
  int listY = statsY + 16;

  int shown = 0;
  for (int i = 0; i < GACHA_TOTAL_CARDS && shown < 3; i++) {
    if (!cards_owned[i]) continue;

    bool inDeck = false;
    for (int d = 0; d < system_state.deck_size; d++) {
      if (system_state.battle_deck[d] == i) { inDeck = true; break; }
    }
    if (inDeck) continue;

    if (shown < deck_scroll_offset) { shown++; continue; }

    int cy = listY + (shown - deck_scroll_offset) * 50;

    // Add button hit
    if (x >= LCD_WIDTH - 65 && x < LCD_WIDTH - 17 && y >= cy + 8 && y < cy + 36) {
      if (addCardToDeck(i)) {
        drawDeckBuilderScreen();
        return;
      }
    }

    shown++;
  }

  // Back button
  if (y >= 455 && y < 487 && x >= LCD_WIDTH/2 - 50 && x < LCD_WIDTH/2 + 50) {
    system_state.current_screen = SCREEN_GACHA;
    drawGachaScreen();
    return;
  }
}
