/*
 * ui.h - User Interface Framework
 * Consolidates: ui.h/cpp, ui_navigation.h/cpp
 */

#ifndef UI_H
#define UI_H

#include "config.h"

// UI Component types
enum UIComponentType {
  UI_BUTTON, UI_LABEL, UI_SLIDER, UI_TOGGLE, UI_LIST, UI_PROGRESS_BAR, UI_ACTIVITY_RING
};

struct UIComponent {
  UIComponentType type;
  int x, y, width, height;
  String text;
  uint16_t color;
  bool visible, enabled, pressed;
  void (*callback)(void);
};

// =============================================================================
// UI SYSTEM
// =============================================================================

void initializeUI();
void ui_init();  // LVGL compatibility
void drawSplashScreen();
void drawAboutScreen();

// Screen management
void setCurrentScreen(ScreenType screen);
ScreenType getCurrentScreen();
void drawCurrentScreen();

// Component drawing
void drawButton(UIComponent& button);
void drawLabel(UIComponent& label);
void drawSlider(UIComponent& slider);
void drawToggle(UIComponent& toggle);

// Touch handling
void handleUITouch(TouchGesture& gesture);
bool isTouchInComponent(TouchGesture& gesture, UIComponent& component);
void handleButtonPress(UIComponent& button);

// Animation
void animateScreenTransition(ScreenType from, ScreenType to);
void animateButtonPress(UIComponent& button);
void animateSlideIn(int direction);

// UI elements
void drawDigitalCrown(int x, int y, int value);
void drawNavigationBar(const char* title, bool back_button = true);
void drawStatusBar();
void drawNotificationDot(int x, int y, uint16_t color);

// List view
void drawListItem(int x, int y, int width, const char* title, const char* subtitle, uint16_t color);
void handleListScroll(TouchGesture& gesture, int& scroll_offset);

// Modal dialogs
void showAlert(const char* title, const char* message);
void showConfirm(const char* title, const char* message, void (*yes_callback)(void));

// Loading indicators
void showLoadingSpinner(const char* message);
void hideLoadingSpinner();
void updateLoadingProgress(int percentage);

// Complications
void drawComplicationSlot(int x, int y, int w, int h, const char* data, uint16_t color);
void updateComplications();

// =============================================================================
// NAVIGATION
// =============================================================================

void initializeNavigation();
void handleNavigation(TouchGesture& gesture);
void navigateBack();
void navigateHome();
void navigateToScreen(ScreenType screen);

// Screen history
void pushScreen(ScreenType screen);
ScreenType popScreen();
void clearScreenHistory();

// Swipe navigation
void handleSwipeNavigation(TouchGesture& gesture);
void showNextScreen();
void showPreviousScreen();

#endif // UI_H
