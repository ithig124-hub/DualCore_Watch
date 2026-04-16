/*
 * games.cpp - Complete Games Implementation
 * Battle Arena, Snake (Enhanced), Memory, + Gacha, Training, Boss Rush integration
 * UPDATED: Added Story Mode integration
 */

#include "games.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "xp_system.h"  // FUSION OS: For gainExperience() and XP rewards
#include "gacha.h"
#include "training.h"
#include "boss_rush.h"
#include "storyline.h"   // FUSION OS: Enhanced Story Mode system

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

GameSession current_game_session;

// Battle creatures database
BattleCreature battle_creatures[] = {
  {"Luffy", "Fighter", 50, 100, 100, 80, 60, 70, {"Gum Gum Pistol", "Gear Second", "Red Hawk", "King Kong Gun"}, {30, 45, 60, 80}, true, COLOR_ORANGE},
  {"Jinwoo", "Shadow", 50, 100, 100, 90, 70, 85, {"Dagger Strike", "Shadow Exchange", "Ruler's Authority", "Dragon's Fear"}, {35, 50, 70, 90}, true, JINWOO_MONARCH_PURPLE},
  {"Yugo", "Portal", 50, 100, 100, 70, 80, 75, {"Portal Slash", "Wakfu Beam", "Stasis", "Eliacube Power"}, {25, 40, 55, 75}, true, YUGO_PORTAL_CYAN},
  {"Naruto", "Ninja", 50, 100, 100, 75, 65, 80, {"Rasengan", "Shadow Clone", "Sage Mode", "Kurama Mode"}, {35, 40, 60, 85}, true, NARUTO_CHAKRA_ORANGE},
  {"Goku", "Saiyan", 50, 100, 100, 95, 55, 90, {"Kamehameha", "Spirit Bomb", "UI Dodge", "Ultra Instinct"}, {40, 70, 50, 95}, true, GOKU_UI_SILVER},
  {"Tanjiro", "Slayer", 50, 100, 100, 72, 75, 70, {"Water Breathing", "Fire Breathing", "Sun Breathing", "Hinokami Kagura"}, {30, 45, 65, 80}, true, TANJIRO_FIRE_ORANGE},
  {"Gojo", "Sorcerer", 50, 100, 100, 100, 100, 95, {"Red", "Blue", "Hollow Purple", "Infinite Void"}, {40, 45, 90, 100}, true, GOJO_INFINITY_BLUE},
  {"Levi", "Soldier", 50, 100, 100, 85, 60, 100, {"Spin Attack", "ODM Slice", "Blade Barrage", "Spinning Slash"}, {35, 50, 70, 85}, true, LEVI_SURVEY_GREEN},
  {"Saitama", "Hero", 50, 100, 100, 100, 50, 80, {"Normal Punch", "Consecutive Punches", "Serious Side Hops", "Serious Punch"}, {100, 100, 50, 999}, true, SAITAMA_HERO_YELLOW},
  {"Deku", "Hero", 50, 100, 100, 75, 70, 75, {"Detroit Smash", "Delaware Smash", "Full Cowl", "United States Smash"}, {35, 40, 55, 85}, true, DEKU_HERO_GREEN},
  {"Shadow Soldier", "Dark", 20, 60, 60, 40, 30, 35, {"Slash", "Dark Bite", "", ""}, {20, 30, 0, 0}, false, RGB565(50, 0, 80)},
  {"Pirate", "Normal", 15, 50, 50, 35, 25, 30, {"Cutlass", "Cannonball", "", ""}, {15, 25, 0, 0}, false, COLOR_RED},
  {"Portal Beast", "Magic", 25, 70, 70, 50, 40, 45, {"Teleport", "Energy Blast", "Confusion", ""}, {25, 35, 20, 0}, false, YUGO_WAKFU_ENERGY}
};
int num_battle_creatures = 13;

// AdvancedGameManager static members
GameType AdvancedGameManager::current_game = GAME_BATTLE_ARENA;
GameState AdvancedGameManager::current_state = GAME_MENU;

// Enhanced Snake state
static int snake_speed_level = 0;  // 0=Normal, 1=Fast, 2=Faster, 3=Extreme
static int snake_extra_lives = 2;

// =============================================================================
// GAME INITIALIZATION
// =============================================================================

void initializeGames() {
  Serial.println("[Games] Initializing complete game system...");
  current_game_session.state = GAME_MENU;
  current_game_session.score = 0;
  current_game_session.level = 1;
  current_game_session.lives = 3;
  current_game_session.sound_enabled = true;
  current_game_session.snake_speed_level = 0;
  current_game_session.snake_extra_lives = 2;
  loadGameProgress();
}

void launchGame(GameType game) {
  current_game_session.current_game = game;
  current_game_session.start_time = millis();
  
  switch(game) {
    case GAME_BATTLE_ARENA:    initBattleArena(); break;
    case GAME_SHADOW_DUNGEON:  initShadowDungeon(); break;
    case GAME_PIRATE_ADVENTURE: initPirateAdventure(); break;
    case GAME_WAKFU_QUEST:     initWakfuQuest(); break;
    case GAME_MINI_SNAKE:      initSnakeGame(); break;
    case GAME_MEMORY_MATCH:    initMemoryGame(); break;
    case GAME_GACHA:           
      system_state.current_screen = SCREEN_GACHA;
      drawGachaScreen();
      break;
    case GAME_TRAINING_REFLEX:
    case GAME_TRAINING_TARGET:
    case GAME_TRAINING_SPEED:
    case GAME_TRAINING_MEMORY: 
      system_state.current_screen = SCREEN_TRAINING;
      drawTrainingMenu();  // Fixed: use drawTrainingMenu not drawTrainingScreen
      break;
    case GAME_BOSS_RUSH:       
      system_state.current_screen = SCREEN_BOSS_RUSH;
      drawBossRushMenu();
      break;
    default: break;
  }
}

void drawGameMenu() {
  // ========================================
  // RETRO ANIME GAME MENU - CRT Style (Optimized for 410x502)
  // ANTI-FLICKER: Only redraw when player stats change
  // ========================================
  
  // ANTI-FLICKER: Track changes
  static bool needs_redraw = true;
  static int last_gems = -1;
  static int last_level = -1;
  
  // Check if data changed
  bool data_changed = (system_state.player_gems != last_gems) ||
                      (system_state.player_level != last_level);
  
  if (!needs_redraw && !data_changed) {
    return;  // Skip redraw - NO FLICKER!
  }
  
  // Update tracking
  needs_redraw = false;
  last_gems = system_state.player_gems;
  last_level = system_state.player_level;
  
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header - taller
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, theme->primary);
  }
  
  gfx->setTextSize(4);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(131, 12);
  gfx->print("GAMES");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(130, 10);
  gfx->print("GAMES");
  
  // Game buttons - larger for 410x502
  // UPDATED: Added Story Mode (7 games in grid)
  const char* games[] = {"Story", "Battle", "Snake", "Memory", "Gacha", "Training", "Boss Rush"};
  uint16_t colors[] = {COLOR_GOLD, COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_PURPLE, COLOR_ORANGE, COLOR_RED};
  
  int cardW = 115;  // Slightly smaller to fit 7
  int cardH = 80;
  int gapX = 12;
  int gapY = 8;
  int startX = (LCD_WIDTH - (3 * cardW + 2 * gapX)) / 2;
  int startY = headerH + 10;
  
  // Draw 7 game cards (3 + 3 + 1 layout)
  for (int i = 0; i < 7; i++) {
    int col, row, x, y;
    
    if (i < 6) {
      col = i % 3;
      row = i / 3;
      x = startX + col * (cardW + gapX);
      y = startY + row * (cardH + gapY);
    } else {
      // 7th card centered on third row
      x = LCD_WIDTH / 2 - cardW / 2;
      y = startY + 2 * (cardH + gapY);
    }
    
    // Special styling for Story Mode (first card)
    bool isStory = (i == 0);
    uint16_t bgColor = isStory ? RGB565(20, 18, 10) : RGB565(12, 14, 20);
    
    gfx->fillRect(x, y, cardW, cardH, bgColor);
    gfx->drawRect(x, y, cardW, cardH, isStory ? COLOR_GOLD : RGB565(40, 45, 60));
    
    // Pixel corners with game color
    gfx->fillRect(x, y, 6, 6, colors[i]);
    gfx->fillRect(x + cardW - 6, y, 6, 6, colors[i]);
    gfx->fillRect(x, y + cardH - 6, 6, 6, colors[i]);
    gfx->fillRect(x + cardW - 6, y + cardH - 6, 6, 6, colors[i]);
    
    // Color stripe at top
    gfx->fillRect(x + 2, y + 2, cardW - 4, 5, colors[i]);
    
    // Special "NEW" badge for Story Mode
    if (isStory) {
      gfx->fillRect(x + cardW - 35, y + 10, 30, 14, COLOR_RED);
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(1);
      gfx->setCursor(x + cardW - 32, y + 13);
      gfx->print("NEW");
    }
    
    gfx->setTextColor(isStory ? COLOR_GOLD : RGB565(200, 205, 220));
    gfx->setTextSize(2);
    int textW = strlen(games[i]) * 12;
    gfx->setCursor(x + (cardW - textW)/2, y + 35);
    gfx->print(games[i]);
  }
  
  // Player stats - retro panel (positioned for new layout)
  int statsY = startY + 3 * (cardH + gapY) + 5;
  int statsW = LCD_WIDTH - 30;
  gfx->fillRect(15, statsY, statsW, 32, RGB565(10, 12, 18));
  gfx->drawRect(15, statsY, statsW, 32, RGB565(35, 40, 55));
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(25, statsY + 8);
  gfx->printf("GEMS: %d | LVL: %d", system_state.player_gems, system_state.player_level);
  
  gfx->fillRect(15, statsY + 37, statsW, 22, RGB565(10, 12, 18));
  gfx->drawRect(15, statsY + 37, statsW, 22, RGB565(35, 40, 55));
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(25, statsY + 44);
  gfx->printf("Cards: %d/100 | Bosses: %d/20", 
              system_state.gacha_cards_collected, system_state.bosses_defeated);
  
  // Adventures section - retro header (compact)
  int advY = statsY + 65;
  gfx->setTextColor(theme->accent);
  gfx->setTextSize(2);
  gfx->setCursor(45, advY);
  gfx->print("ADVENTURES");
  // Pixel underline
  for (int x = 45; x < 215; x += 6) {
    gfx->fillRect(x, advY + 20, 4, 2, theme->accent);
  }
  
  const char* adv[] = {"Shadow", "Pirate", "Wakfu"};
  int advCardW = 110;
  int advStartX = (LCD_WIDTH - (3 * advCardW + 2 * 12)) / 2;
  for (int i = 0; i < 3; i++) {
    int x = advStartX + i * (advCardW + 12);
    int y = advY + 26;
    gfx->fillRect(x, y, advCardW, 35, RGB565(15, 18, 25));
    gfx->drawRect(x, y, advCardW, 35, RGB565(40, 45, 60));
    gfx->fillRect(x, y, 5, 5, theme->accent);
    gfx->fillRect(x + advCardW - 5, y, 5, 5, theme->accent);
    gfx->setTextColor(RGB565(180, 185, 200));
    gfx->setTextSize(2);
    int tw = strlen(adv[i]) * 12;
    gfx->setCursor(x + (advCardW - tw)/2, y + 10);
    gfx->print(adv[i]);
  }
  
  // Back button - positioned for taller display
  gfx->fillRect(330, LCD_HEIGHT - 35, 65, 28, RGB565(15, 18, 25));
  gfx->drawRect(330, LCD_HEIGHT - 35, 65, 28, RGB565(40, 45, 60));
  gfx->setTextColor(RGB565(120, 125, 140));
  gfx->setTextSize(1);
  gfx->setCursor(347, LCD_HEIGHT - 26);
  gfx->print("Back");
}

void handleGameMenuTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Match the UPDATED layout for 410x502 (7 games)
  int cardW = 115;
  int cardH = 80;
  int gapX = 12;
  int gapY = 8;
  int headerH = 55;
  int startX = (LCD_WIDTH - (3 * cardW + 2 * gapX)) / 2;
  int startY = headerH + 10;
  
  // Check main game buttons (7 games: Story, Battle, Snake, Memory, Gacha, Training, Boss Rush)
  for (int i = 0; i < 7; i++) {
    int col, row, bx, by;
    
    if (i < 6) {
      col = i % 3;
      row = i / 3;
      bx = startX + col * (cardW + gapX);
      by = startY + row * (cardH + gapY);
    } else {
      // 7th card centered on third row
      bx = LCD_WIDTH / 2 - cardW / 2;
      by = startY + 2 * (cardH + gapY);
    }
    
    if (x >= bx && x < bx + cardW && y >= by && y < by + cardH) {
      // Map game buttons to actions
      if (i == 0) {
        // Story Mode - uses storyline system
        setCurrentStory(system_state.current_theme);
        system_state.current_screen = SCREEN_STORY_MENU;
        drawStoryMenu();
        return;
      } else {
        // Other games (offset by 1 due to Story being first)
        GameType gameMap[] = {GAME_BATTLE_ARENA, GAME_MINI_SNAKE, GAME_MEMORY_MATCH,
                              GAME_GACHA, GAME_TRAINING_REFLEX, GAME_BOSS_RUSH};
        launchGame(gameMap[i - 1]);
        return;
      }
    }
  }
  
  // Adventure buttons (adjusted Y positions)
  int statsY = startY + 3 * (cardH + gapY) + 5;
  int advY = statsY + 65 + 26;
  int advCardW = 110;
  int advStartX = (LCD_WIDTH - (3 * advCardW + 2 * 12)) / 2;
  
  if (y >= advY && y < advY + 35) {
    if (x >= advStartX && x < advStartX + advCardW) {
      launchGame(GAME_SHADOW_DUNGEON);
      return;
    }
    else if (x >= advStartX + advCardW + 12 && x < advStartX + 2 * advCardW + 12) {
      launchGame(GAME_PIRATE_ADVENTURE);
      return;
    }
    else if (x >= advStartX + 2 * (advCardW + 12) && x < advStartX + 3 * advCardW + 24) {
      launchGame(GAME_WAKFU_QUEST);
      return;
    }
  }
  
  // Back button
  if (y >= 420 && x >= 290) {
    system_state.current_screen = SCREEN_APP_GRID;
  }
}

// =============================================================================
// BATTLE ARENA
// =============================================================================

void initBattleArena() {
  current_game_session.state = GAME_BATTLE_SELECT;
  
  // Set player creature based on theme
  switch(system_state.current_theme) {
    case THEME_LUFFY_GEAR5:  current_game_session.player_creature = battle_creatures[0]; break;
    case THEME_SUNG_JINWOO:  current_game_session.player_creature = battle_creatures[1]; break;
    case THEME_YUGO_WAKFU:   current_game_session.player_creature = battle_creatures[2]; break;
    default: current_game_session.player_creature = battle_creatures[0]; break;
  }
  
  generateRandomEnemy();
  current_game_session.log_count = 0;
}

void drawBattleArena() {
  // ========================================
  // RETRO ANIME BATTLE ARENA - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 40, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 38, 6, 3, COLOR_RED);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_RED);
  gfx->setCursor(80, 10);
  gfx->print("BATTLE ARENA");
  
  // Enemy panel - retro card
  BattleCreature& enemy = current_game_session.enemy_creature;
  gfx->fillRect(15, 48, LCD_WIDTH - 30, 65, RGB565(12, 14, 20));
  gfx->drawRect(15, 48, LCD_WIDTH - 30, 65, RGB565(40, 45, 60));
  gfx->fillRect(15, 48, 5, 5, enemy.color);
  gfx->fillRect(LCD_WIDTH - 20, 48, 5, 5, enemy.color);
  
  gfx->setTextColor(enemy.color);
  gfx->setTextSize(2);
  gfx->setCursor(25, 55);
  gfx->print(enemy.name);
  gfx->setTextSize(1);
  gfx->setCursor(25, 75);
  gfx->printf("Lv.%d", enemy.level);
  drawHealthBar(25, 90, 200, 16, enemy.hp, enemy.max_hp, enemy.color);
  
  // Battle log - retro terminal
  gfx->fillRect(15, 120, LCD_WIDTH - 30, 50, RGB565(8, 10, 14));
  gfx->drawRect(15, 120, LCD_WIDTH - 30, 50, RGB565(30, 35, 50));
  gfx->setTextColor(RGB565(0, 200, 80));
  gfx->setTextSize(1);
  for (int i = 0; i < min(3, current_game_session.log_count); i++) {
    gfx->setCursor(22, 128 + i * 14);
    gfx->print(current_game_session.battle_log[i]);
  }
  
  // Player panel - retro card
  BattleCreature& player = current_game_session.player_creature;
  gfx->fillRect(15, 180, LCD_WIDTH - 30, 65, RGB565(12, 14, 20));
  gfx->drawRect(15, 180, LCD_WIDTH - 30, 65, RGB565(40, 45, 60));
  gfx->fillRect(15, 180, 5, 5, player.color);
  gfx->fillRect(LCD_WIDTH - 20, 180, 5, 5, player.color);
  
  gfx->setTextColor(player.color);
  gfx->setTextSize(2);
  gfx->setCursor(25, 188);
  gfx->print(player.name);
  gfx->setTextSize(1);
  gfx->setCursor(25, 208);
  gfx->printf("Lv.%d", player.level);
  drawHealthBar(25, 225, 200, 16, player.hp, player.max_hp, player.color);
  
  // Move buttons - retro pixel style
  for (int i = 0; i < 4; i++) {
    if (strlen(player.moves[i]) > 0) {
      int x = (i % 2) * 180 + 10;
      int y = (i / 2) * 55 + 260;
      bool selected = (current_game_session.selected_move == i);
      uint16_t btnBg = selected ? theme->primary : RGB565(15, 18, 25);
      gfx->fillRect(x, y, 170, 48, btnBg);
      gfx->drawRect(x, y, 170, 48, selected ? theme->accent : RGB565(40, 45, 60));
      gfx->fillRect(x, y, 4, 4, theme->primary);
      gfx->fillRect(x + 166, y, 4, 4, theme->primary);
      
      gfx->setTextColor(selected ? COLOR_WHITE : RGB565(180, 185, 200));
      gfx->setTextSize(1);
      int tw = strlen(player.moves[i]) * 6;
      gfx->setCursor(x + (170 - tw)/2, y + 15);
      gfx->print(player.moves[i]);
      
      gfx->setTextColor(RGB565(80, 85, 100));
      gfx->setCursor(x + 60, y + 32);
      gfx->printf("PWR:%d", player.move_power[i]);
    }
  }
}

void handleBattleTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Check move buttons
  for (int i = 0; i < 4; i++) {
    int bx = (i % 2) * 180 + 10;
    int by = (i / 2) * 60 + 300;
    if (x >= bx && x < bx + 170 && y >= by && y < by + 55) {
      selectMove(i);
      return;
    }
  }
}

void startBattle() {
  current_game_session.state = GAME_BATTLE_FIGHT;
  addBattleLog("Battle Start!");
}

void selectMove(int move_index) {
  current_game_session.selected_move = move_index;
  executeBattleTurn();
}

void executeBattleTurn() {
  BattleCreature& player = current_game_session.player_creature;
  BattleCreature& enemy = current_game_session.enemy_creature;
  
  // Player attacks
  int move_idx = current_game_session.selected_move;
  int damage = calculateDamage(player, enemy, player.move_power[move_idx]);
  enemy.hp -= damage;
  addBattleLog(String(player.name) + " used " + String(player.moves[move_idx]) + "!");
  addBattleLog("Dealt " + String(damage) + " damage!");
  
  if (enemy.hp <= 0) {
    enemy.hp = 0;
    addBattleLog("Enemy defeated!");
    current_game_session.state = GAME_WIN;
    current_game_session.score += enemy.level * 10;
    return;
  }
  
  // Enemy attacks
  int enemy_move = random(0, 2);
  damage = calculateDamage(enemy, player, enemy.move_power[enemy_move]);
  player.hp -= damage;
  addBattleLog(String(enemy.name) + " used " + String(enemy.moves[enemy_move]) + "!");
  
  if (player.hp <= 0) {
    player.hp = 0;
    addBattleLog("You were defeated!");
    current_game_session.state = GAME_OVER;
  }
}

void dealDamage(BattleCreature& attacker, BattleCreature& defender, int move_index) {
  int damage = calculateDamage(attacker, defender, attacker.move_power[move_index]);
  defender.hp -= damage;
  if (defender.hp < 0) defender.hp = 0;
}

void addBattleLog(String message) {
  if (current_game_session.log_count < 5) {
    current_game_session.battle_log[current_game_session.log_count++] = message;
  } else {
    for (int i = 0; i < 4; i++) {
      current_game_session.battle_log[i] = current_game_session.battle_log[i + 1];
    }
    current_game_session.battle_log[4] = message;
  }
}

// checkBattleEnd is declared in boss_rush.h as bool - use that version
// This local version is renamed to avoid conflict
void checkGameBattleEnd() {
  if (current_game_session.enemy_creature.hp <= 0) {
    current_game_session.state = GAME_WIN;
  }
  if (current_game_session.player_creature.hp <= 0) {
    current_game_session.state = GAME_OVER;
  }
}

void generateRandomEnemy() {
  int idx = random(3, num_battle_creatures);
  current_game_session.enemy_creature = battle_creatures[idx];
  current_game_session.enemy_creature.level = current_game_session.level * 5 + random(1, 10);
  current_game_session.enemy_creature.max_hp = 50 + current_game_session.enemy_creature.level * 2;
  current_game_session.enemy_creature.hp = current_game_session.enemy_creature.max_hp;
}

// =============================================================================
// SHADOW DUNGEON
// =============================================================================

void initShadowDungeon() {
  current_game_session.state = GAME_PLAYING;
  current_game_session.player_x = 5;
  current_game_session.player_y = 5;
  current_game_session.map_level = 1;
  current_game_session.items_collected = 0;
}

void drawShadowDungeon() {
  // ========================================
  // RETRO ANIME SHADOW DUNGEON - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, JINWOO_PURPLE);
  }
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setTextSize(2);
  gfx->setCursor(61, 14 + 1);
  gfx->print("SHADOW DUNGEON");
  gfx->setTextColor(JINWOO_PURPLE);
  gfx->setCursor(60, 14);
  gfx->print("SHADOW DUNGEON");
  
  // Draw map grid - retro pixel style
  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      int px = x * 35 + 20;
      int py = y * 35 + 60;
      gfx->fillRect(px, py, 32, 32, RGB565(8, 10, 14));
      gfx->drawRect(px, py, 32, 32, RGB565(25, 20, 40));
      
      if (x == current_game_session.player_x && y == current_game_session.player_y) {
        // Player - pixel square with glow
        gfx->fillRect(px + 4, py + 4, 24, 24, JINWOO_PURPLE);
        gfx->drawRect(px + 2, py + 2, 28, 28, JINWOO_MONARCH_PURPLE);
      }
    }
  }
  
  // Stats panel - retro
  gfx->fillRect(15, 415, LCD_WIDTH - 30, 28, RGB565(10, 12, 18));
  gfx->drawRect(15, 415, LCD_WIDTH - 30, 28, RGB565(35, 40, 55));
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(25, 424);
  gfx->printf("Floor: %d  Items: %d", current_game_session.map_level, current_game_session.items_collected);
}

void handleDungeonTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_UP) movePlayer(0, -1);
  else if (gesture.event == TOUCH_SWIPE_DOWN) movePlayer(0, 1);
  else if (gesture.event == TOUCH_SWIPE_LEFT) movePlayer(-1, 0);
  else if (gesture.event == TOUCH_SWIPE_RIGHT) movePlayer(1, 0);
}

void movePlayer(int dx, int dy) {
  int newX = current_game_session.player_x + dx;
  int newY = current_game_session.player_y + dy;
  
  if (newX >= 0 && newX < 10 && newY >= 0 && newY < 10) {
    current_game_session.player_x = newX;
    current_game_session.player_y = newY;
  }
}

void fightShadowMonster() {
  current_game_session.score += 10;
}

void levelUpPlayer() {
  current_game_session.level++;
}

// =============================================================================
// PIRATE ADVENTURE
// =============================================================================

void initPirateAdventure() {
  current_game_session.state = GAME_PLAYING;
  current_game_session.items_collected = 0;
}

void drawPirateAdventure() {
  // ========================================
  // RETRO ANIME PIRATE ADVENTURE - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, COLOR_ORANGE);
  }
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setTextSize(2);
  gfx->setCursor(41, 14 + 1);
  gfx->print("PIRATE ADVENTURE");
  gfx->setTextColor(COLOR_ORANGE);
  gfx->setCursor(40, 14);
  gfx->print("PIRATE ADVENTURE");
  
  // Treasure counter - retro card
  gfx->fillRect(60, 180, 240, 50, RGB565(12, 14, 20));
  gfx->drawRect(60, 180, 240, 50, RGB565(40, 45, 60));
  gfx->fillRect(60, 180, 5, 5, COLOR_GOLD);
  gfx->fillRect(295, 180, 5, 5, COLOR_GOLD);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(1);
  gfx->setCursor(80, 200);
  gfx->printf("TREASURES FOUND: %d", current_game_session.items_collected);
  
  // Explore button - retro
  gfx->fillRect(100, 280, 160, 50, RGB565(15, 18, 25));
  gfx->drawRect(100, 280, 160, 50, COLOR_ORANGE);
  gfx->fillRect(100, 280, 5, 5, COLOR_ORANGE);
  gfx->fillRect(255, 280, 5, 5, COLOR_ORANGE);
  gfx->fillRect(100, 325, 5, 5, COLOR_ORANGE);
  gfx->fillRect(255, 325, 5, 5, COLOR_ORANGE);
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(2);
  gfx->setCursor(130, 296);
  gfx->print("Explore");
}

void handlePirateTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_TAP && gesture.y >= 300 && gesture.y < 350) {
    exploreIsland();
  }
}

void exploreIsland() {
  if (random(0, 100) < 30) {
    findTreasure();
  }
}

void findTreasure() {
  current_game_session.items_collected++;
  current_game_session.score += 20;
}

// =============================================================================
// WAKFU QUEST
// =============================================================================

void initWakfuQuest() {
  current_game_session.state = GAME_PLAYING;
}

void drawWakfuQuest() {
  // ========================================
  // RETRO ANIME WAKFU QUEST - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, YUGO_TEAL);
  }
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setTextSize(2);
  gfx->setCursor(81, 14 + 1);
  gfx->print("WAKFU QUEST");
  gfx->setTextColor(YUGO_TEAL);
  gfx->setCursor(80, 14);
  gfx->print("WAKFU QUEST");
  
  // Placeholder content - retro styled
  gfx->fillRect(60, 180, 240, 60, RGB565(12, 14, 20));
  gfx->drawRect(60, 180, 240, 60, RGB565(40, 45, 60));
  gfx->fillRect(60, 180, 5, 5, YUGO_TEAL);
  gfx->fillRect(295, 180, 5, 5, YUGO_TEAL);
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(100, 205);
  gfx->print("PORTAL AWAITS...");
}

void handleWakfuTouch(TouchGesture& gesture) {
  // Handle puzzle interactions
}

void solvePuzzle() {
  current_game_session.score += 15;
}

void openPortal() {
  current_game_session.map_level++;
}

// =============================================================================
// SNAKE GAME - ENHANCED
// =============================================================================

void initSnakeGame() {
  current_game_session.state = GAME_PLAYING;
  current_game_session.snake_length = 3;
  current_game_session.direction = 1;  // 0=up, 1=right, 2=down, 3=left
  current_game_session.snake_speed_level = 0;  // Start at normal speed
  current_game_session.snake_extra_lives = 2;  // 2 extra lives
  current_game_session.lives = 3;
  current_game_session.score = 0;
  
  for (int i = 0; i < current_game_session.snake_length; i++) {
    current_game_session.snake_x[i] = 5 - i;
    current_game_session.snake_y[i] = 5;
  }
  
  current_game_session.food_x = random(1, 18);
  current_game_session.food_y = random(1, 18);
}

void drawSnakeGame() {
  // ========================================
  // RETRO ANIME SNAKE - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  
  ThemeColors* theme = getCurrentTheme();
  
  // Retro header bar
  gfx->fillRect(0, 0, LCD_WIDTH, 40, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 38, 6, 2, theme->primary);
  }
  
  // Title - shadow text
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(131, 9);
  gfx->print("SNAKE");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(130, 8);
  gfx->print("SNAKE");
  
  // Speed indicator - retro
  const char* speeds[] = {"NORMAL", "FAST", "FASTER", "EXTREME"};
  gfx->setTextSize(1);
  uint16_t speedColor = current_game_session.snake_speed_level >= 2 ? COLOR_RED : RGB565(130, 135, 150);
  gfx->setTextColor(speedColor);
  gfx->setCursor(280, 15);
  gfx->print(speeds[current_game_session.snake_speed_level]);
  
  // Lives display - pixel hearts
  gfx->setTextColor(COLOR_RED);
  gfx->setCursor(20, 15);
  for (int i = 0; i < current_game_session.lives; i++) {
    gfx->print("* ");
  }
  
  // Game grid - retro pixel border
  gfx->fillRect(20, 48, 320, 320, RGB565(4, 5, 8));
  gfx->drawRect(20, 48, 320, 320, theme->primary);
  gfx->drawRect(21, 49, 318, 318, RGB565(30, 35, 50));
  
  // Subtle grid lines
  for (int gx = 20; gx < 340; gx += 16) {
    gfx->drawFastVLine(gx, 48, 320, RGB565(6, 7, 10));
  }
  for (int gy = 48; gy < 48 + 320; gy += 16) {
    gfx->drawFastHLine(20, gy, 320, RGB565(6, 7, 10));
  }
  
  // Draw snake - retro pixel blocks
  uint16_t headColor = theme->primary;
  uint16_t bodyColor = theme->accent;
  
  for (int i = 0; i < current_game_session.snake_length; i++) {
    int x = current_game_session.snake_x[i] * 16 + 20;
    int y = current_game_session.snake_y[i] * 16 + 48;
    
    if (i == 0) {
      // Head - solid pixel block with eye dots
      gfx->fillRect(x + 1, y + 1, 14, 14, headColor);
      gfx->drawRect(x, y, 16, 16, RGB565(50, 55, 70));
      // Eyes
      int ex1, ey1, ex2, ey2;
      switch (current_game_session.direction) {
        case 0: ex1 = x+3; ey1 = y+3; ex2 = x+10; ey2 = y+3; break;
        case 1: ex1 = x+10; ey1 = y+3; ex2 = x+10; ey2 = y+10; break;
        case 2: ex1 = x+3; ey1 = y+10; ex2 = x+10; ey2 = y+10; break;
        case 3: ex1 = x+3; ey1 = y+3; ex2 = x+3; ey2 = y+10; break;
      }
      gfx->fillRect(ex1, ey1, 3, 3, COLOR_WHITE);
      gfx->fillRect(ex2, ey2, 3, 3, COLOR_WHITE);
    } else {
      // Body - pixel blocks
      gfx->fillRect(x + 1, y + 1, 13, 13, bodyColor);
    }
  }
  
  // Draw food - pixel style pulsing square
  int fx = current_game_session.food_x * 16 + 20;
  int fy = current_game_session.food_y * 16 + 48;
  gfx->fillRect(fx + 2, fy + 2, 12, 12, COLOR_RED);
  gfx->drawRect(fx + 1, fy + 1, 14, 14, COLOR_ORANGE);
  
  // Score panel - retro
  gfx->fillRect(15, 375, LCD_WIDTH - 30, 28, RGB565(10, 12, 18));
  gfx->drawRect(15, 375, LCD_WIDTH - 30, 28, RGB565(35, 40, 55));
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(25, 384);
  gfx->printf("Score: %d", current_game_session.score);
  gfx->setCursor(130, 384);
  gfx->printf("Len: %d", current_game_session.snake_length);
  gfx->setCursor(230, 384);
  gfx->printf("Lvl: %d", current_game_session.snake_speed_level + 1);
  
  // Controls hint - retro
  gfx->setTextColor(RGB565(50, 55, 70));
  gfx->setCursor(LCD_WIDTH/2 - 50, 415);
  gfx->print("> SWIPE TO MOVE <");
}

void handleSnakeTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_UP && current_game_session.direction != 2) 
    current_game_session.direction = 0;
  else if (gesture.event == TOUCH_SWIPE_RIGHT && current_game_session.direction != 3) 
    current_game_session.direction = 1;
  else if (gesture.event == TOUCH_SWIPE_DOWN && current_game_session.direction != 0) 
    current_game_session.direction = 2;
  else if (gesture.event == TOUCH_SWIPE_LEFT && current_game_session.direction != 1) 
    current_game_session.direction = 3;
}

int getSnakeDelay() {
  // Speed based on level: Normal=200ms, Fast=150ms, Faster=100ms, Extreme=60ms
  int delays[] = {200, 150, 100, 60};
  return delays[current_game_session.snake_speed_level];
}

void increaseSnakeSpeed() {
  if (current_game_session.snake_speed_level < 3) {
    current_game_session.snake_speed_level++;
    Serial.printf("[Snake] Speed increased to level %d\n", current_game_session.snake_speed_level);
  }
}

void updateSnake() {
  static unsigned long lastMove = 0;
  if (millis() - lastMove < getSnakeDelay()) return;
  lastMove = millis();
  
  // Move body
  for (int i = current_game_session.snake_length - 1; i > 0; i--) {
    current_game_session.snake_x[i] = current_game_session.snake_x[i - 1];
    current_game_session.snake_y[i] = current_game_session.snake_y[i - 1];
  }
  
  // Move head
  switch (current_game_session.direction) {
    case 0: current_game_session.snake_y[0]--; break;
    case 1: current_game_session.snake_x[0]++; break;
    case 2: current_game_session.snake_y[0]++; break;
    case 3: current_game_session.snake_x[0]--; break;
  }
  
  // Check food collision
  if (current_game_session.snake_x[0] == current_game_session.food_x &&
      current_game_session.snake_y[0] == current_game_session.food_y) {
    current_game_session.snake_length++;
    current_game_session.score += 10 * (current_game_session.snake_speed_level + 1);
    current_game_session.food_x = random(1, 18);
    current_game_session.food_y = random(1, 18);
    
    // Increase speed every 5 foods
    if (current_game_session.snake_length % 5 == 0) {
      increaseSnakeSpeed();
    }
    
    // Give gems for high scores
    if (current_game_session.score % 100 == 0) {
      addGems(GEMS_GAME_WIN_MIN, "Snake Score");
    }
  }
  
  checkSnakeCollision();
}

void checkSnakeCollision() {
  int x = current_game_session.snake_x[0];
  int y = current_game_session.snake_y[0];
  
  bool collision = false;
  
  // Wall collision
  if (x < 0 || x >= 20 || y < 0 || y >= 20) {
    collision = true;
  }
  
  // Self collision
  for (int i = 1; i < current_game_session.snake_length; i++) {
    if (x == current_game_session.snake_x[i] && y == current_game_session.snake_y[i]) {
      collision = true;
      break;
    }
  }
  
  if (collision) {
    current_game_session.lives--;
    
    if (current_game_session.lives <= 0) {
      current_game_session.state = GAME_OVER;
      
      // Give XP based on score
      gainExperience(current_game_session.score / 2, "Snake Game");
    } else {
      // Respawn with lives remaining
      current_game_session.snake_length = 3;
      current_game_session.direction = 1;
      
      for (int i = 0; i < current_game_session.snake_length; i++) {
        current_game_session.snake_x[i] = 5 - i;
        current_game_session.snake_y[i] = 5;
      }
      
      current_game_session.food_x = random(1, 18);
      current_game_session.food_y = random(1, 18);
    }
  }
}

// =============================================================================
// MEMORY MATCH
// =============================================================================

static int memory_cards[16];
static bool memory_revealed[16];
static int first_card = -1, second_card = -1;
static int matches_found = 0;

void initMemoryGame() {
  current_game_session.state = GAME_PLAYING;
  matches_found = 0;
  first_card = second_card = -1;
  
  // Setup card pairs
  for (int i = 0; i < 8; i++) {
    memory_cards[i * 2] = i;
    memory_cards[i * 2 + 1] = i;
  }
  
  // Shuffle
  for (int i = 15; i > 0; i--) {
    int j = random(0, i + 1);
    int temp = memory_cards[i];
    memory_cards[i] = memory_cards[j];
    memory_cards[j] = temp;
  }
  
  for (int i = 0; i < 16; i++) memory_revealed[i] = false;
}

void drawMemoryGame() {
  // ========================================
  // RETRO ANIME MEMORY MATCH - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
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
  gfx->setCursor(71, 14 + 1);
  gfx->print("MEMORY MATCH");
  gfx->setTextColor(theme->primary);
  gfx->setCursor(70, 14);
  gfx->print("MEMORY MATCH");
  
  uint16_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, 
                       COLOR_ORANGE, COLOR_CYAN, COLOR_PINK, JINWOO_PURPLE};
  
  for (int i = 0; i < 16; i++) {
    int x = (i % 4) * 85 + 25;
    int y = (i / 4) * 85 + 60;
    
    if (memory_revealed[i]) {
      // Revealed card - retro pixel with color
      gfx->fillRect(x, y, 75, 75, RGB565(12, 14, 20));
      gfx->fillRect(x + 5, y + 5, 65, 65, colors[memory_cards[i]]);
      gfx->drawRect(x, y, 75, 75, colors[memory_cards[i]]);
      // Pixel corners
      gfx->fillRect(x, y, 5, 5, colors[memory_cards[i]]);
      gfx->fillRect(x + 70, y, 5, 5, colors[memory_cards[i]]);
      gfx->fillRect(x, y + 70, 5, 5, colors[memory_cards[i]]);
      gfx->fillRect(x + 70, y + 70, 5, 5, colors[memory_cards[i]]);
      // CRT lines on card
      for (int sy = y + 6; sy < y + 69; sy += 3) {
        gfx->drawFastHLine(x + 6, sy, 63, RGB565(0, 0, 0));
      }
    } else {
      // Hidden card - retro dark
      gfx->fillRect(x, y, 75, 75, RGB565(15, 18, 25));
      gfx->drawRect(x, y, 75, 75, RGB565(40, 45, 60));
      gfx->fillRect(x, y, 4, 4, theme->primary);
      gfx->fillRect(x + 71, y, 4, 4, theme->primary);
      // Question mark
      gfx->setTextColor(RGB565(50, 55, 70));
      gfx->setTextSize(2);
      gfx->setCursor(x + 28, y + 28);
      gfx->print("?");
    }
  }
  
  // Score panel - retro
  gfx->fillRect(15, 410, LCD_WIDTH - 30, 28, RGB565(10, 12, 18));
  gfx->drawRect(15, 410, LCD_WIDTH - 30, 28, RGB565(35, 40, 55));
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(25, 419);
  gfx->printf("Matches: %d/8  Score: %d", matches_found, current_game_session.score);
}

void handleMemoryTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int cardX = (gesture.x - 25) / 85;
  int cardY = (gesture.y - 60) / 85;
  
  if (cardX >= 0 && cardX < 4 && cardY >= 0 && cardY < 4) {
    flipCard(cardX, cardY);
  }
}

void flipCard(int x, int y) {
  int idx = y * 4 + x;
  if (memory_revealed[idx]) return;
  
  memory_revealed[idx] = true;
  
  if (first_card < 0) {
    first_card = idx;
  } else if (second_card < 0) {
    second_card = idx;
    checkMatch();
  }
}

void checkMatch() {
  if (memory_cards[first_card] == memory_cards[second_card]) {
    matches_found++;
    current_game_session.score += 20;
    
    if (matches_found >= 8) {
      current_game_session.state = GAME_WIN;
    }
  } else {
    delay(500);
    memory_revealed[first_card] = false;
    memory_revealed[second_card] = false;
  }
  
  first_card = second_card = -1;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void drawHealthBar(int x, int y, int w, int h, int current_hp, int max_hp, uint16_t color) {
  gfx->fillRect(x, y, w, h, RGB565(8, 10, 14));
  gfx->drawRect(x, y, w, h, RGB565(40, 45, 60));
  int fillW = (current_hp * (w - 4)) / max_hp;
  if (fillW > 0) gfx->fillRect(x + 2, y + 2, fillW, h - 4, color);
  // Pixel notches
  for (int nx = x + 2; nx < x + w - 2; nx += 20) {
    gfx->drawFastVLine(nx, y + 1, h - 2, RGB565(20, 22, 30));
  }
}

void drawProgressBar(int x, int y, int w, int h, float progress, uint16_t color) {
  progress = constrain(progress, 0.0f, 1.0f);
  gfx->fillRect(x, y, w, h, RGB565(8, 10, 14));
  gfx->drawRect(x, y, w, h, RGB565(40, 45, 60));
  int fillW = (int)(progress * (w - 4));
  if (fillW > 0) gfx->fillRect(x + 2, y + 2, fillW, h - 4, color);
}

void showGameNotification(String title, String message) {
  drawThemeNotification(title.c_str(), message.c_str());
}

int calculateDamage(BattleCreature& attacker, BattleCreature& defender, int move_power) {
  float base = (attacker.attack * move_power) / (float)(defender.defense + 50);
  return max(1, (int)(base * (0.85 + random(0, 30) / 100.0)));
}

String getTypeEffectiveness(String attack_type, String defend_type) {
  // Type effectiveness chart (simplified)
  return "Normal";
}

void saveGameProgress() {
  // Save to EEPROM or file
}

void loadGameProgress() {
  // Load from EEPROM or file
}

// AdvancedGameManager implementation
void AdvancedGameManager::initGame(GameType type) {
  current_game = type;
  current_state = GAME_MENU;
  launchGame(type);
}

void AdvancedGameManager::updateGame() {
  if (current_game == GAME_MINI_SNAKE && current_state == GAME_PLAYING) {
    updateSnake();
  }
}

void AdvancedGameManager::drawGame() {
  switch(current_game) {
    case GAME_BATTLE_ARENA: drawBattleArena(); break;
    case GAME_SHADOW_DUNGEON: drawShadowDungeon(); break;
    case GAME_PIRATE_ADVENTURE: drawPirateAdventure(); break;
    case GAME_WAKFU_QUEST: drawWakfuQuest(); break;
    case GAME_MINI_SNAKE: drawSnakeGame(); break;
    case GAME_MEMORY_MATCH: drawMemoryGame(); break;
  }
}

void AdvancedGameManager::handleGameInput(TouchGesture& gesture) {
  switch(current_game) {
    case GAME_BATTLE_ARENA: handleBattleTouch(gesture); break;
    case GAME_SHADOW_DUNGEON: handleDungeonTouch(gesture); break;
    case GAME_PIRATE_ADVENTURE: handlePirateTouch(gesture); break;
    case GAME_WAKFU_QUEST: handleWakfuTouch(gesture); break;
    case GAME_MINI_SNAKE: handleSnakeTouch(gesture); break;
    case GAME_MEMORY_MATCH: handleMemoryTouch(gesture); break;
  }
}

GameState AdvancedGameManager::getCurrentState() { return current_state; }
void AdvancedGameManager::setCurrentState(GameState state) { current_state = state; }
