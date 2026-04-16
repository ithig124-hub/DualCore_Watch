/*
 * fusion_game.h - BoBoiBoy Fusion Minigame
 * Drag and combine two elements to unlock fusions
 */

#ifndef FUSION_GAME_H
#define FUSION_GAME_H

#include "config.h"

// Fusion game state
enum FusionGameState {
  FUSION_IDLE = 0,
  FUSION_DRAGGING,
  FUSION_COMBINING,
  FUSION_SUCCESS,
  FUSION_FAIL,
  FUSION_COMPLETE
};

// Draggable element
struct DraggableElement {
  int baseX;
  int baseY;
  int currentX;
  int currentY;
  int elementIndex;
  bool isDragging;
  bool isInDropZone;
};

// Fusion combination
struct FusionCombo {
  int element1;
  int element2;
  int resultIndex;  // Index in boboiboy_elements
  const char* fusionName;
  bool unlocked;
};

extern FusionGameState fusion_state;
extern DraggableElement dragged_element;

// Valid fusion combinations
extern FusionCombo fusion_combos[6];

// =============================================================================
// FUSION GAME FUNCTIONS
// =============================================================================

// Initialize fusion game
void initFusionGame();

// Draw fusion game screen
void drawFusionGame();

// Handle touch in fusion game
void handleFusionGameTouch(TouchGesture& gesture);

// Start dragging an element
void startDraggingElement(int elementIndex, int x, int y);

// Update drag position
void updateDragPosition(int x, int y);

// Drop element and check for fusion
void dropElement();

// Check if two elements can fuse
int checkFusion(int elem1, int elem2);

// Play fusion animation
void playFusionAnimation(int fusionIndex);

// Draw element orb (draggable)
void drawElementOrb(int x, int y, int elementIndex, bool highlight, float scale);

// Draw drop zone
void drawDropZone(int x, int y, bool active);

// Draw fusion zone
void drawFusionZone(int x, int y);

// Draw fusion button
void drawFusionButton(int x, int y);

// Draw fusion result
void drawFusionResult(int fusionIndex);

// Animation functions
void drawFusionCombiningAnim(int x, int y);
void drawFusionSuccessAnim(int x, int y);
void drawFusionFailAnim(int x, int y);

#endif // FUSION_GAME_H
