/*
 * character_games.h - Character-Specific Minigames
 * Unique games for each anime character
 */

#ifndef CHARACTER_GAMES_H
#define CHARACTER_GAMES_H

#include "config.h"

// Game types for each character
enum CharacterGameType {
  GAME_LUFFY_PUNCH = 0,    // Rubber punch timing game
  GAME_GOJO_DODGE,         // Domain expansion dodge
  GAME_LEVI_SLASH,         // Titan slash combo
  GAME_NARUTO_JUTSU,       // Handseal sequence
  GAME_GOKU_KAMEHAMEHA,    // Power charge minigame
  GAME_TANJIRO_BREATH,     // Breathing rhythm game
  GAME_JINWOO_ARISE,       // Shadow summoning
  GAME_DEKU_SMASH,         // Power control game
  GAME_SAITAMA_PUNCH,      // One punch timing (instant win joke)
  GAME_BOBOIBOY_ELEMENT,   // Element battle
  CHARACTER_GAME_COUNT
};

// Game state
struct CharacterGameState {
  CharacterGameType game_type;
  bool is_active;
  int score;
  int combo;
  int lives;
  int level;
  unsigned long game_start_time;
  unsigned long last_update;
};

extern CharacterGameState char_game_state;

// =============================================================================
// GAME FUNCTIONS
// =============================================================================

// Initialize character games
void initCharacterGames();

// Get game for current theme
CharacterGameType getGameForTheme(ThemeType theme);

// Start a character game
void startCharacterGame(CharacterGameType game);

// Update game (called in loop)
void updateCharacterGame();

// Handle touch in game
void handleCharacterGameTouch(TouchGesture& gesture);

// Draw current game
void drawCharacterGame();

// End game and show results
void endCharacterGame();

// =============================================================================
// LUFFY RUBBER PUNCH GAME
// =============================================================================
void drawLuffyPunchGame();
void updateLuffyPunchGame();
void handleLuffyPunchTouch(TouchGesture& gesture);

// =============================================================================
// GOJO DOMAIN DODGE GAME
// =============================================================================
void drawGojoDodgeGame();
void updateGojoDodgeGame();
void handleGojoDodgeTouch(TouchGesture& gesture);

// =============================================================================
// LEVI TITAN SLASH GAME
// =============================================================================
void drawLeviSlashGame();
void updateLeviSlashGame();
void handleLeviSlashTouch(TouchGesture& gesture);

// =============================================================================
// BOBOIBOY ELEMENT BATTLE
// =============================================================================

struct ElementBattleState {
  int player_element;      // Current element (0-6)
  int enemy_element;       // Enemy element
  int player_hp;
  int enemy_hp;
  int player_power;        // Charge level
  bool is_attacking;
  bool enemy_attacking;
  int enemy_attack_timer;
  int battle_phase;        // 0=select, 1=fight, 2=win, 3=lose
};

extern ElementBattleState element_battle;

void initElementBattle();
void drawElementBattle();
void updateElementBattle();
void handleElementBattleTouch(TouchGesture& gesture);
void elementPlayerAttack();
void enemyAttack();
int getElementAdvantage(int attacker, int defender);

#endif // CHARACTER_GAMES_H
