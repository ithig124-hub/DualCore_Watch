/*
 * navigation.h - IMPROVED Navigation Header
 * Modern Anime Gaming Smartwatch
 * 
 * Better swipe detection, clearer touch targets, smooth transitions
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"

// =============================================================================
// NAVIGATION SYSTEM - IMPROVED
// =============================================================================

// Initialize navigation
void initNavigation();

// Check if navigation is allowed
bool canNavigate();

// Handle swipe gestures - IMPROVED with better detection
void handleSwipeNavigation(int dx, int dy);

// Navigate between main screens
void navigateLeft();
void navigateRight();
void navigateUp();
void navigateDown();
void navigateToScreen(MainScreen screen);

// Get current state
MainScreen getCurrentMainScreen();
int getCurrentAppGridPage();

// =============================================================================
// SCREEN DRAWING
// =============================================================================

// Draw the current screen
void drawCurrentScreen();

// Draw navigation indicators (dots at bottom)
void drawNavigationIndicators();

// Watch face screen
void drawStatsBar();

// App Grid screens - IMPROVED LAYOUT
void drawAppGrid1();
void drawAppGrid2();
void drawAppGrid3();  // NEW: Third page

// Character stats screen
void drawStatBar(int x, int y, const char* name, int value, uint16_t color);
int getXPForNextLevel();

// =============================================================================
// TOUCH HANDLING - IMPROVED
// =============================================================================

// Handle touch events for current screen
void handleCurrentScreenTouch(TouchGesture& gesture);

// Handle taps on app grid - IMPROVED with better hit detection
void handleAppGridTap(int x, int y);

// Open an app
void openApp(const char* appName);

// Return to app grid from any app
void returnToAppGrid();

// =============================================================================
// APP ICON HELPERS
// =============================================================================

// Get color for app icon based on name
uint16_t getAppColor(const char* appName);

// Draw app icon with modern styling
void drawAppIcon(int x, int y, int w, int h, const char* name, uint16_t color, bool selected);

// Draw back button (standardized)
void drawBackButton(int x, int y);
void drawSwipeIndicator();

// =============================================================================
// TRANSITION ANIMATIONS
// =============================================================================

// Slide transition between screens
void slideTransition(MainScreen from, MainScreen to, bool leftToRight);

// Fade transition
void fadeTransition();

// =============================================================================
// TOUCH INPUT PROCESSING - IMPROVED
// =============================================================================

// Process raw touch input and return gesture
TouchGesture handleTouchInput();

// Improved touch state tracking
struct TouchState {
  bool isPressed;
  int16_t startX;
  int16_t startY;
  int16_t lastX;
  int16_t lastY;
  uint32_t pressTime;
  uint32_t lastEventTime;
};

extern TouchState touchState;

#endif // NAVIGATION_H
