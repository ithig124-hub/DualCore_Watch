/*
 * games.h - Complete Games System
 * Battle Arena, Snake, Memory, Gacha, Training, Boss Rush
 */

#ifndef GAMES_H
#define GAMES_H

#include "types.h"  // Must include types.h for GameType, GameState, BattleCreature, etc.
#include "config.h"

// Include sub-game systems
#include "gacha.h"
#include "training.h"
#include "boss_rush.h"

// =============================================================================
// GAME SYSTEM
// =============================================================================

void initializeGames();
void launchGame(GameType game);
void drawGameMenu();
void handleGameMenuTouch(TouchGesture& gesture);

// Battle Arena (Pokemon Showdown style)
void initBattleArena();
void drawBattleArena();
void handleBattleTouch(TouchGesture& gesture);
void startBattle();
void selectMove(int move_index);
void executeBattleTurn();
void dealDamage(BattleCreature& attacker, BattleCreature& defender, int move_index);
void addBattleLog(String message);
// void checkBattleEnd(); // Conflicts with boss_rush.h
void generateRandomEnemy();

// Shadow Dungeon (Jin-Woo themed)
void initShadowDungeon();
void drawShadowDungeon();
void handleDungeonTouch(TouchGesture& gesture);
void movePlayer(int dx, int dy);
void fightShadowMonster();
void levelUpPlayer();

// Pirate Adventure (Luffy themed)
void initPirateAdventure();
void drawPirateAdventure();
void handlePirateTouch(TouchGesture& gesture);
void exploreIsland();
void findTreasure();

// Wakfu Quest (Yugo themed)
void initWakfuQuest();
void drawWakfuQuest();
void handleWakfuTouch(TouchGesture& gesture);
void solvePuzzle();
void openPortal();

// Snake Game - ENHANCED
void initSnakeGame();
void drawSnakeGame();
void handleSnakeTouch(TouchGesture& gesture);
void updateSnake();
void checkSnakeCollision();
void increaseSnakeSpeed();
int getSnakeDelay();

// Memory Match
void initMemoryGame();
void drawMemoryGame();
void handleMemoryTouch(TouchGesture& gesture);
void flipCard(int x, int y);
void checkMatch();

// Utility
void drawHealthBar(int x, int y, int w, int h, int current_hp, int max_hp, uint16_t color);
void drawProgressBar(int x, int y, int w, int h, float progress, uint16_t color);
void showGameNotification(String title, String message);
int calculateDamage(BattleCreature& attacker, BattleCreature& defender, int move_power);
String getTypeEffectiveness(String attack_type, String defend_type);

void saveGameProgress();
void loadGameProgress();

// Game session
extern GameSession current_game_session;
extern BattleCreature battle_creatures[];
extern int num_battle_creatures;

// Advanced Game Manager
class AdvancedGameManager {
public:
  static void initGame(GameType type);
  static void updateGame();
  static void drawGame();
  static void handleGameInput(TouchGesture& gesture);
  static GameState getCurrentState();
  static void setCurrentState(GameState state);
private:
  static GameType current_game;
  static GameState current_state;
};

#endif // GAMES_H
