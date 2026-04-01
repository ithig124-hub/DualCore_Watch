/*
 * ochobot.cpp - Ochobot Assistant Implementation
 * Cute Power Sphere helper for BoBoiBoy mode
 */

#include "ochobot.h"
#include "config.h"
#include "display.h"
#include "themes.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Global state
OchobotPosition ochobot_pos = {LCD_WIDTH - 50, LCD_HEIGHT - 80, false, false};
OchobotState ochobot_state = OCHOBOT_IDLE;
int current_boboiboy_element = 0;  // Start with Halilintar

// Message display state
static bool showing_message = false;
static unsigned long message_start_time = 0;
static int message_duration = 0;
static char current_message[64] = "";

// Animation state
static float ochobot_bounce_offset = 0;
static unsigned long last_blink = 0;
static bool ochobot_eyes_closed = false;

// =============================================================================
// ELEMENT TIPS
// =============================================================================

const char* element_tips[] = {
  "Halilintar: Speed is key!",
  "Taufan: Ride the wind!",
  "Gempa: Stand your ground!",
  "Blaze: Feel the burn!",
  "Ice: Stay cool, stay calm!",
  "Thorn: Nature is power!",
  "Solar: Shine bright!"
};

const char* fusion_tips[] = {
  "FrostFire: Hot and cold!",
  "Combine elements wisely!",
  "Fusions need practice!",
  "Try Blaze + Ice combo!",
  "Supra is the strongest!"
};

const char* training_tips[] = {
  "Train daily for gems!",
  "Complete all exercises!",
  "Streak bonus is huge!",
  "Don't skip leg day!",
  "Rest is important too!"
};

const char* battle_tips[] = {
  "Match element to enemy!",
  "Dodge then attack!",
  "Save power for boss!",
  "Use terrain advantage!",
  "Timing is everything!"
};

const char* motivation_tips[] = {
  "You can do it!",
  "BoBoiBoy believes in you!",
  "Power up, hero!",
  "Kuasa Tujuh awaits!",
  "Never give up!",
  "TAPOPS needs you!",
  "Protect the galaxy!"
};

// =============================================================================
// INITIALIZATION
// =============================================================================

void initOchobot() {
  Serial.println("[OCHOBOT] Initializing Ochobot assistant...");
  ochobot_pos.x = LCD_WIDTH - 50;
  ochobot_pos.y = LCD_HEIGHT - 80;
  ochobot_pos.visible = (system_state.current_theme == THEME_BOBOIBOY);
  ochobot_pos.bouncing = true;
  ochobot_state = OCHOBOT_IDLE;
  current_boboiboy_element = 0;
}

// =============================================================================
// DRAWING
// =============================================================================

void drawOchobot(int x, int y, OchobotState state, float scale) {
  int size = 30 * scale;
  
  // Bounce animation
  if (ochobot_pos.bouncing) {
    ochobot_bounce_offset = sin(millis() / 300.0) * 3;
  }
  y += ochobot_bounce_offset;
  
  // Body (sphere shape)
  gfx->fillCircle(x, y, size, BBB_OCHOBOT_WHITE);
  gfx->drawCircle(x, y, size, RGB565(180, 185, 200));
  
  // Inner ring detail
  gfx->drawCircle(x, y, size - 5, RGB565(220, 225, 240));
  
  // Eyes
  int eyeY = y - 3 * scale;
  int eyeSpacing = 10 * scale;
  
  // Blink animation
  if (millis() - last_blink > 3000) {
    ochobot_eyes_closed = true;
    if (millis() - last_blink > 3200) {
      ochobot_eyes_closed = false;
      last_blink = millis();
    }
  }
  
  if (ochobot_eyes_closed || state == OCHOBOT_SLEEPING) {
    // Closed eyes (lines)
    gfx->drawLine(x - eyeSpacing - 4, eyeY, x - eyeSpacing + 4, eyeY, COLOR_BLACK);
    gfx->drawLine(x + eyeSpacing - 4, eyeY, x + eyeSpacing + 4, eyeY, COLOR_BLACK);
  } else {
    // Open eyes
    uint16_t eyeColor = COLOR_BLACK;
    if (state == OCHOBOT_HAPPY) eyeColor = RGB565(50, 50, 60);
    if (state == OCHOBOT_ALERT) eyeColor = BBB_FIRE_RED;
    
    gfx->fillCircle(x - eyeSpacing, eyeY, 5 * scale, eyeColor);
    gfx->fillCircle(x + eyeSpacing, eyeY, 5 * scale, eyeColor);
    
    // Eye shine
    gfx->fillCircle(x - eyeSpacing + 2, eyeY - 2, 2 * scale, COLOR_WHITE);
    gfx->fillCircle(x + eyeSpacing + 2, eyeY - 2, 2 * scale, COLOR_WHITE);
  }
  
  // Mouth based on state
  int mouthY = y + 8 * scale;
  
  switch (state) {
    case OCHOBOT_HAPPY:
      // Big smile
      gfx->drawLine(x - 8, mouthY, x, mouthY + 4, COLOR_BLACK);
      gfx->drawLine(x, mouthY + 4, x + 8, mouthY, COLOR_BLACK);
      break;
      
    case OCHOBOT_TALKING:
      // Open mouth (ellipse)
      gfx->fillEllipse(x, mouthY + 2, 6 * scale, 4 * scale, RGB565(100, 50, 50));
      break;
      
    case OCHOBOT_ALERT:
      // O shape
      gfx->drawCircle(x, mouthY + 2, 4 * scale, COLOR_BLACK);
      break;
      
    case OCHOBOT_SLEEPING:
      // Zzz
      gfx->setTextColor(RGB565(100, 100, 120));
      gfx->setTextSize(1);
      gfx->setCursor(x + 15, y - 20);
      gfx->print("z");
      gfx->setCursor(x + 22, y - 28);
      gfx->print("z");
      break;
      
    default:
      // Neutral line
      gfx->drawLine(x - 6, mouthY, x + 6, mouthY, COLOR_BLACK);
      break;
  }
  
  // Power Band symbol on forehead
  gfx->fillCircle(x, y - size + 8, 6 * scale, BBB_BAND_ORANGE);
  gfx->drawCircle(x, y - size + 8, 6 * scale, RGB565(200, 100, 20));
  
  // Antenna
  gfx->drawLine(x, y - size, x, y - size - 10 * scale, RGB565(180, 185, 200));
  gfx->fillCircle(x, y - size - 10 * scale, 3 * scale, BBB_BAND_ORANGE);
}

void drawOchobotWithMessage(const char* message, int duration_ms) {
  showing_message = true;
  message_start_time = millis();
  message_duration = duration_ms;
  strncpy(current_message, message, 63);
  current_message[63] = '\0';
  
  ochobot_state = OCHOBOT_TALKING;
}

void drawOchobotMessageBubble() {
  if (!showing_message) return;
  
  // Check if message should still be shown
  if (millis() - message_start_time > message_duration) {
    showing_message = false;
    ochobot_state = OCHOBOT_IDLE;
    return;
  }
  
  int bubbleX = ochobot_pos.x - 150;
  int bubbleY = ochobot_pos.y - 50;
  int bubbleW = 140;
  int bubbleH = 40;
  
  // Speech bubble
  gfx->fillRect(bubbleX, bubbleY, bubbleW, bubbleH, COLOR_WHITE);
  gfx->drawRect(bubbleX, bubbleY, bubbleW, bubbleH, BBB_BAND_ORANGE);
  
  // Bubble tail
  gfx->fillTriangle(
    bubbleX + bubbleW - 20, bubbleY + bubbleH,
    bubbleX + bubbleW - 5, bubbleY + bubbleH,
    ochobot_pos.x - 20, ochobot_pos.y - 15,
    COLOR_WHITE
  );
  
  // Message text
  gfx->setTextColor(COLOR_BLACK);
  gfx->setTextSize(1);
  
  // Word wrap simple
  int textX = bubbleX + 8;
  int textY = bubbleY + 10;
  int maxWidth = bubbleW - 16;
  
  gfx->setCursor(textX, textY);
  
  int lineLen = 0;
  for (int i = 0; current_message[i] != '\0'; i++) {
    if (lineLen >= 22 || (lineLen > 15 && current_message[i] == ' ')) {
      textY += 12;
      gfx->setCursor(textX, textY);
      lineLen = 0;
      if (current_message[i] == ' ') continue;
    }
    gfx->print(current_message[i]);
    lineLen++;
  }
}

// =============================================================================
// TIPS SYSTEM
// =============================================================================

const char* getOchobotTip(OchobotTipType type) {
  const char** tips;
  int count;
  
  switch (type) {
    case TIP_ELEMENT:
      tips = element_tips;
      count = 7;
      break;
    case TIP_FUSION:
      tips = fusion_tips;
      count = 5;
      break;
    case TIP_TRAINING:
      tips = training_tips;
      count = 5;
      break;
    case TIP_BATTLE:
      tips = battle_tips;
      count = 5;
      break;
    case TIP_MOTIVATION:
    default:
      tips = motivation_tips;
      count = 7;
      break;
  }
  
  return tips[random(count)];
}

const char* getElementTip(int elementIndex) {
  if (elementIndex >= 0 && elementIndex < 7) {
    return element_tips[elementIndex];
  }
  return "Power up!";
}

void ochobotNotify(const char* title, const char* message) {
  // Show notification with Ochobot
  char fullMsg[64];
  snprintf(fullMsg, 63, "%s: %s", title, message);
  drawOchobotWithMessage(fullMsg, 3000);
}

// =============================================================================
// UPDATE
// =============================================================================

void updateOchobot() {
  if (!ochobot_pos.visible) return;
  
  // Update message display
  if (showing_message) {
    drawOchobotMessageBubble();
  }
  
  // Random tip every 30 seconds
  static unsigned long lastTip = 0;
  if (millis() - lastTip > 30000 && !showing_message) {
    lastTip = millis();
    
    // 30% chance to show a tip
    if (random(100) < 30) {
      const char* tip = getOchobotTip((OchobotTipType)random(5));
      drawOchobotWithMessage(tip, 4000);
    }
  }
}

// =============================================================================
// TAP-TO-SWITCH ELEMENT SYSTEM
// =============================================================================

int getCurrentBoboiboyElement() {
  return current_boboiboy_element;
}

void switchToNextElement() {
  current_boboiboy_element = (current_boboiboy_element + 1) % 7;
  
  const char* elementNames[] = {
    "Halilintar!", "Taufan!", "Gempa!", 
    "Blaze!", "Ice!", "Thorn!", "Solar!"
  };
  
  // Show Ochobot notification
  char msg[32];
  snprintf(msg, 31, "BoBoiBoy %s", elementNames[current_boboiboy_element]);
  drawOchobotWithMessage(msg, 1500);
  
  Serial.printf("[BBB] Switched to element: %d (%s)\n", 
                current_boboiboy_element, elementNames[current_boboiboy_element]);
}

void switchToElement(int elementIndex) {
  if (elementIndex >= 0 && elementIndex < 7) {
    current_boboiboy_element = elementIndex;
    
    const char* elementNames[] = {
      "Halilintar!", "Taufan!", "Gempa!", 
      "Blaze!", "Ice!", "Thorn!", "Solar!"
    };
    
    char msg[32];
    snprintf(msg, 31, "BoBoiBoy %s", elementNames[elementIndex]);
    drawOchobotWithMessage(msg, 1500);
  }
}

bool isElementSwitchZoneTap(int x, int y) {
  // Element switch zone is the center of the watch face
  int centerX = LCD_WIDTH / 2;
  int centerY = 160;
  int zoneRadius = 80;
  
  int dist = sqrt((x - centerX) * (x - centerX) + (y - centerY) * (y - centerY));
  return dist < zoneRadius;
}

void drawElementIndicator(int x, int y, int currentElement) {
  uint16_t colors[] = {
    BBB_LIGHTNING_YELLOW, BBB_WIND_BLUE, BBB_EARTH_BROWN,
    BBB_FIRE_RED, BBB_WATER_CYAN, BBB_LEAF_GREEN, BBB_LIGHT_GOLD
  };
  
  // Draw small element dots
  int dotRadius = 6;
  int spacing = 20;
  int totalWidth = 7 * spacing;
  int startX = x - totalWidth / 2 + spacing / 2;
  
  for (int i = 0; i < 7; i++) {
    int dotX = startX + i * spacing;
    
    if (i == currentElement) {
      // Current element - larger and glowing
      gfx->fillCircle(dotX, y, dotRadius + 2, colors[i]);
      gfx->drawCircle(dotX, y, dotRadius + 4, COLOR_WHITE);
    } else {
      // Other elements - smaller
      gfx->fillCircle(dotX, y, dotRadius - 2, colors[i]);
    }
  }
}
