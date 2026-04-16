/*
 * ochobot.h - Ochobot Assistant System
 * BoBoiBoy's Power Sphere helper with tips and notifications
 */

#ifndef OCHOBOT_H
#define OCHOBOT_H

#include "config.h"

// Ochobot states
enum OchobotState {
  OCHOBOT_IDLE = 0,
  OCHOBOT_TALKING,
  OCHOBOT_HAPPY,
  OCHOBOT_ALERT,
  OCHOBOT_SLEEPING
};

// Ochobot tip categories
enum OchobotTipType {
  TIP_ELEMENT = 0,
  TIP_FUSION,
  TIP_TRAINING,
  TIP_BATTLE,
  TIP_GENERAL,
  TIP_MOTIVATION
};

// Ochobot position on screen
struct OchobotPosition {
  int x;
  int y;
  bool visible;
  bool bouncing;
};

extern OchobotPosition ochobot_pos;
extern OchobotState ochobot_state;
extern int current_boboiboy_element;  // 0-6 for manual switching

// =============================================================================
// OCHOBOT FUNCTIONS
// =============================================================================

// Initialize Ochobot
void initOchobot();

// Draw Ochobot on screen
void drawOchobot(int x, int y, OchobotState state, float scale);

// Draw Ochobot with speech bubble
void drawOchobotWithMessage(const char* message, int duration_ms);

// Get random tip for current context
const char* getOchobotTip(OchobotTipType type);

// Get element-specific tip
const char* getElementTip(int elementIndex);

// Show notification through Ochobot
void ochobotNotify(const char* title, const char* message);

// Update Ochobot animation
void updateOchobot();

// =============================================================================
// TAP-TO-SWITCH ELEMENT SYSTEM
// =============================================================================

// Get current manually selected element
int getCurrentBoboiboyElement();

// Switch to next element (tap action)
void switchToNextElement();

// Switch to specific element
void switchToElement(int elementIndex);

// Check if tap is on element switch zone
bool isElementSwitchZoneTap(int x, int y);

// Draw element indicator (shows current element)
void drawElementIndicator(int x, int y, int currentElement);

#endif // OCHOBOT_H
