/*
 * training.h - Training Mini-Games System
 * Reflex Test, Target Shoot, Speed Tap, Memory Match
 * XP rewards, streak bonuses, level progression
 */

#ifndef TRAINING_H
#define TRAINING_H

#include "types.h"  // Must include types.h for TrainingType, TrainingScore
#include "config.h"

// =============================================================================
// TRAINING SYSTEM
// =============================================================================

void initTrainingSystem();
void saveTrainingProgress();
bool loadTrainingProgress();

// Training session management
void startTrainingGame(TrainingType type);
void updateTrainingGame();
void endTrainingGame();
bool isTrainingActive();

// XP and rewards
int calculateTrainingXP(TrainingType type, int score, bool perfect);
void applyStreakBonus(int& xp);
int getTrainingStreak();
void updateTrainingStreak();

// =============================================================================
// REFLEX TEST
// =============================================================================

void initReflexTest();
void updateReflexTest();
void drawReflexTest();
void handleReflexTouch(TouchGesture& gesture);
void flashReflexButton(int button_index);
void recordReflexTime(int time_ms);
const char* getReflexRating(int avg_ms);

// =============================================================================
// TARGET SHOOT
// =============================================================================

void initTargetShoot();
void updateTargetShoot();
void drawTargetShoot();
void handleTargetTouch(TouchGesture& gesture);
void spawnTarget();
void hitTarget(int target_index);
void missTarget();

// =============================================================================
// SPEED TAP
// =============================================================================

void initSpeedTap();
void updateSpeedTap();
void drawSpeedTap();
void handleSpeedTapTouch(TouchGesture& gesture);
void recordTap();
const char* getSpeedTapRanking(int taps);

// =============================================================================
// MEMORY MATCH (Training Version)
// =============================================================================

void initTrainingMemory();
void updateTrainingMemory();
void drawTrainingMemory();
void handleTrainingMemoryTouch(TouchGesture& gesture);
void showMemoryPattern();
bool checkMemoryInput(int button);
void nextMemoryLevel();

// =============================================================================
// TRAINING UI
// =============================================================================

void drawTrainingMenu();
void drawTrainingResults(TrainingScore& score);
void handleTrainingMenuTouch(TouchGesture& gesture);
void drawTrainingButton(int x, int y, int size, int button_id, bool lit, bool pressed);
void drawTrainingTimer(int seconds_remaining);
void drawTrainingScore(int score);

// Training state
extern TrainingScore current_training_score;
extern TrainingType current_training_type;
extern bool training_active;
extern int training_round;
extern unsigned long training_start_time;

#endif // TRAINING_H
