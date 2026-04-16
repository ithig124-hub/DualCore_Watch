/*
 * gacha.h - Gacha Collection System
 * Collectible cards with rarity tiers, gem economy
 * 100 unique cards across 10 anime series
 * 
 * UPDATED: Added card selling functions for XP and Gems
 */

#ifndef GACHA_H
#define GACHA_H

#include "types.h"    // Must include types.h for GachaCard, GachaRarity
#include "config.h"

// =============================================================================
// GACHA SYSTEM
// =============================================================================
void initGachaSystem();
void saveGachaProgress();
bool loadGachaProgress();

// Gem Management
int getPlayerGems();
void addGems(int amount, const char* source);
bool spendGems(int amount);

// Pulling
bool canPullSingle();
bool canPullTen();
GachaCard performSinglePull();
void performTenPull(GachaCard* results);
GachaRarity rollRarity();

// Collection
int getCardsOwned();
int getTotalCards();
bool ownsCard(int card_id);
void addCardToCollection(GachaCard& card);
GachaCard* getCard(int card_id);
int getDuplicateCount(int card_id);

// Rewards
void checkCollectionRewards();
void giveCollectionReward(int milestone);

// =============================================================================
// NEW: CARD SELLING SYSTEM
// =============================================================================

// Sell a single card - returns XP gained
int sellCard(int card_id);

// Sell all duplicates of a specific card - returns total XP gained
int sellCardDuplicates(int card_id);

// Sell ALL duplicate cards across collection - returns total XP gained
int sellAllDuplicates();

// Sell ENTIRE collection (all cards) - returns total XP gained
int sellEntireCollection();

// Get total value of entire collection (for preview)
void getCollectionValue(int* outXP, int* outGems);

// Get XP value for selling a card based on rarity
int getCardSellXP(GachaRarity rarity);

// Get Gem value for selling a card based on rarity
int getCardSellGems(GachaRarity rarity);

// Check if a card can be sold
bool canSellCard(int card_id);

// Get number of sellable cards (duplicates only or all)
int getSellableCardCount(bool duplicatesOnly);

// =============================================================================
// GACHA UI
// =============================================================================
void drawGachaScreen();
void drawGachaShop();
void drawGachaPullAnimation(GachaCard& card);
void drawGachaReveal(GachaCard& card);
void drawGachaRevealImproved(GachaCard& card);
void drawTenPullGrid(GachaCard* results);
void drawGachaCollection();
void drawGachaCard(int x, int y, int w, int h, GachaCard& card);
void drawCardRarityBorder(int x, int y, int w, int h, GachaRarity rarity);
void drawCardGlow(int x, int y, int w, int h, GachaRarity rarity);
void handleGachaTouch(TouchGesture& gesture);
void handleCollectionTouch(TouchGesture& gesture);

// Sell All Confirmation Screen
void drawSellAllConfirmation();
void handleSellAllConfirmationTouch(TouchGesture& gesture);

// =============================================================================
// CARD DATABASE
// =============================================================================
extern GachaCard gacha_cards[GACHA_TOTAL_CARDS];
extern bool cards_owned[GACHA_TOTAL_CARDS];
extern int cards_duplicates[GACHA_TOTAL_CARDS];

void initCardDatabase();
GachaCard generateCard(int series_index, int card_index, GachaRarity rarity);

// Series names
extern const char* GACHA_SERIES_NAMES[10];

// Rarity colors and names
uint16_t getRarityBorderColor(GachaRarity rarity);
uint16_t getRarityGlowColor(GachaRarity rarity);
const char* getRarityName(GachaRarity rarity);
int getRarityStars(GachaRarity rarity);
int getPowerRange(GachaRarity rarity, bool max);

// Evolution & Deck features
void drawPityCounter(int x, int y);
void drawCardEvolutionScreen();
void drawDeckBuilderScreen();
void handleCardEvolutionTap(int x, int y);
void handleDeckBuilderTap(int x, int y);
bool evolveCard(int cardIndex);
int getEvolveCost(int currentLevel);
float getEvolvePowerMult(int level);
int getDeckTotalPower();
int getDeckBonusATK();
int getDeckBonusHP();
int getDeckBonusDEF();
bool addCardToDeck(int cardIndex);
bool removeCardFromDeck(int slotIndex);
void applyPitySystem(GachaRarity& rarity);
const char* getEvolutionName(int level);
uint16_t getEvolutionColor(int level);

// Helper function
void saveGachaCard(GachaCard& card);

#endif // GACHA_H
