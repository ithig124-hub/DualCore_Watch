/*
 * fusion_game.cpp - BoBoiBoy Fusion Minigame
 * Drag two elements to the fusion zone to combine them
 */

#include "fusion_game.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "boboiboy_elements.h"
#include "ochobot.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Game state
FusionGameState fusion_state = FUSION_IDLE;
DraggableElement dragged_element = {0, 0, 0, 0, -1, false, false};

// Drop zone elements (two slots)
static int dropZone1Element = -1;
static int dropZone2Element = -1;
static unsigned long lastAnimTime = 0;
static int animFrame = 0;

// Valid fusion combinations: element1, element2, resultIndex, name, unlocked
// OFFICIAL BOBOIBOY FUSIONS:
FusionCombo fusion_combos[6] = {
  {3, 4, 14, "FrostFire", false},      // Blaze + Ice
  {9, 4, 15, "Glacier", false},        // Quake (evolved Gempa) + Ice
  {7, 13, 16, "Supra", false},         // Thunderstorm + Solar (evolved)
  {12, 13, 17, "Sori", false},         // Thorn (Darkwood) + Solar (Supernova)
  {9, 7, 18, "Rumble", false},         // Quake + Thunderstorm
  {13, 8, 19, "Sopan", false}          // Solar (Supernova) + Cyclone
};

// Element colors for fusion (14 elements: 7 base + 7 evolved)
uint16_t fusionElementColors[] = {
  // Base (0-6)
  BBB_LIGHTNING_YELLOW, BBB_WIND_BLUE, BBB_EARTH_BROWN,
  BBB_FIRE_RED, BBB_WATER_CYAN, BBB_LEAF_GREEN, BBB_LIGHT_GOLD,
  // Evolved (7-13)
  BBB_THUNDERSTORM_RED, BBB_CYCLONE_DARK_BLUE, BBB_QUAKE_DARK,
  BBB_BLAZE_CRIMSON, BBB_ICE_LIGHT, BBB_THORN_DARK, BBB_SOLAR_ORANGE
};

const char* fusionElementNames[] = {
  // Base
  "Lightning", "Wind", "Earth", "Fire", "Ice", "Leaf", "Light",
  // Evolved
  "Thunderstorm", "Cyclone", "Quake", "Inferno", "Glacier", "Darkwood", "Supernova"
};

// =============================================================================
// INITIALIZATION
// =============================================================================

void initFusionGame() {
  Serial.println("[FUSION] Initializing fusion minigame...");
  fusion_state = FUSION_IDLE;
  dragged_element.isDragging = false;
  dragged_element.elementIndex = -1;
  dropZone1Element = -1;
  dropZone2Element = -1;
  animFrame = 0;
}

// =============================================================================
// MAIN DRAW FUNCTION
// =============================================================================

void drawFusionGame() {
  // ========================================
  // RETRO ANIME FUSION GAME - CRT Style
  // ========================================
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  // Retro header
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, BBB_BAND_ORANGE);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565(30, 35, 50));
  gfx->setCursor(61, 14 + 1);
  gfx->print("ELEMENT FUSION");
  gfx->setTextColor(BBB_BAND_ORANGE);
  gfx->setCursor(60, 14);
  gfx->print("ELEMENT FUSION");
  
  // Instructions
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(1);
  gfx->setCursor(60, 55);
  gfx->print("Drag elements to fusion zone!");
  
  // Draw the 14 element orbs (7 base + 7 evolved) - Two rows
  int orbY1 = 360;  // Base elements row
  int orbY2 = 410;  // Evolved elements row
  int orbSpacing = 50;
  int startX = (LCD_WIDTH - 7 * orbSpacing) / 2 + orbSpacing / 2;
  
  // Draw Base elements (top row)
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setTextSize(1);
  gfx->setCursor(20, orbY1 - 20);
  gfx->print("BASE:");
  
  for (int i = 0; i < 7; i++) {
    int orbX = startX + i * orbSpacing;
    
    // Don't draw if this element is in drop zone
    if (i == dropZone1Element || i == dropZone2Element) continue;
    
    // Don't draw if being dragged
    if (dragged_element.isDragging && dragged_element.elementIndex == i) continue;
    
    drawElementOrb(orbX, orbY1, i, false, 0.7);
  }
  
  // Draw Evolved elements (bottom row)
  gfx->setCursor(20, orbY2 - 20);
  gfx->print("EVOLVED:");
  
  for (int i = 7; i < 14; i++) {
    int orbX = startX + (i - 7) * orbSpacing;
    
    // Don't draw if this element is in drop zone
    if (i == dropZone1Element || i == dropZone2Element) continue;
    
    // Don't draw if being dragged
    if (dragged_element.isDragging && dragged_element.elementIndex == i) continue;
    
    drawElementOrb(orbX, orbY2, i, false, 0.7);
  }
  
  // Draw fusion zone in center
  int zoneX = LCD_WIDTH / 2;
  int zoneY = 200;
  
  drawFusionZone(zoneX, zoneY);
  
  // Draw elements in drop zones
  if (dropZone1Element >= 0) {
    drawElementOrb(zoneX - 60, zoneY, dropZone1Element, true, 1.2);
  }
  if (dropZone2Element >= 0) {
    drawElementOrb(zoneX + 60, zoneY, dropZone2Element, true, 1.2);
  }
  
  // Draw dragged element
  if (dragged_element.isDragging && dragged_element.elementIndex >= 0) {
    drawElementOrb(dragged_element.currentX, dragged_element.currentY, 
                   dragged_element.elementIndex, true, 1.3);
  }
  
  // Fusion button (if two elements selected)
  if (dropZone1Element >= 0 && dropZone2Element >= 0) {
    drawFusionButton(zoneX, zoneY + 80);
  }
  
  // Draw fusion state animations
  switch (fusion_state) {
    case FUSION_COMBINING:
      drawFusionCombiningAnim(zoneX, zoneY);
      break;
    case FUSION_SUCCESS:
      drawFusionSuccessAnim(zoneX, zoneY);
      break;
    case FUSION_FAIL:
      drawFusionFailAnim(zoneX, zoneY);
      break;
    default:
      break;
  }
  
  // Back button - retro
  gfx->fillRect(10, 410, 70, 30, RGB565(15, 18, 25));
  gfx->drawRect(10, 410, 70, 30, BBB_BAND_ORANGE);
  gfx->fillRect(10, 410, 4, 4, BBB_BAND_ORANGE);
  gfx->setTextColor(RGB565(200, 205, 220));
  gfx->setTextSize(1);
  gfx->setCursor(28, 421);
  gfx->print("Back");
  
  // Reset button - retro
  gfx->fillRect(290, 410, 70, 30, RGB565(15, 18, 25));
  gfx->drawRect(290, 410, 70, 30, RGB565(50, 55, 70));
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setCursor(305, 421);
  gfx->print("Reset");
}

void drawElementOrb(int x, int y, int elementIndex, bool highlight, float scale) {
  if (elementIndex < 0 || elementIndex >= 14) return;
  
  int radius = 25 * scale;
  uint16_t color = fusionElementColors[elementIndex];
  
  // Retro pixel style - square orbs with pixel corners
  int size = radius * 2;
  int ox = x - radius;
  int oy = y - radius;
  
  if (highlight) {
    // Glow - pixel outline
    gfx->drawRect(ox - 4, oy - 4, size + 8, size + 8, color);
    gfx->drawRect(ox - 2, oy - 2, size + 4, size + 4, RGB565(40, 42, 50));
  }
  
  // Main orb - pixel square
  gfx->fillRect(ox, oy, size, size, color);
  gfx->drawRect(ox, oy, size, size, COLOR_WHITE);
  // Pixel corners
  gfx->fillRect(ox, oy, 4, 4, COLOR_WHITE);
  gfx->fillRect(ox + size - 4, oy, 4, 4, COLOR_WHITE);
  gfx->fillRect(ox, oy + size - 4, 4, 4, COLOR_WHITE);
  gfx->fillRect(ox + size - 4, oy + size - 4, 4, 4, COLOR_WHITE);
  
  // CRT scan lines on orb
  for (int sy = oy + 2; sy < oy + size - 2; sy += 3) {
    gfx->drawFastHLine(ox + 2, sy, size - 4, RGB565(0, 0, 0));
  }
  
  // Element initial
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(scale > 0.9 ? 2 : 1);
  char initial[2] = {fusionElementNames[elementIndex][0], '\0'};
  int offset = scale > 0.9 ? 6 : 3;
  gfx->setCursor(x - offset, y - offset);
  gfx->print(initial);
  
  // Evolved indicator (pixel star)
  if (elementIndex >= 7) {
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(x + radius - 5, y - radius);
    gfx->print("*");
  }
}

void drawFusionZone(int x, int y) {
  // Retro pixel fusion zone - square instead of circle
  gfx->drawRect(x - 100, y - 80, 200, 160, RGB565(60, 65, 80));
  gfx->drawRect(x - 98, y - 78, 196, 156, RGB565(40, 42, 50));
  
  // Inner area
  gfx->fillRect(x - 90, y - 70, 180, 140, RGB565(8, 10, 14));
  // CRT lines inside zone
  for (int sy = y - 68; sy < y + 68; sy += 4) {
    gfx->drawFastHLine(x - 88, sy, 176, RGB565(6, 6, 10));
  }
  
  // Drop zone indicators - pixel squares
  drawDropZone(x - 60, y, dropZone1Element >= 0);
  drawDropZone(x + 60, y, dropZone2Element >= 0);
  
  // Plus sign in center
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(3);
  gfx->setCursor(x - 9, y - 12);
  gfx->print("+");
  
  // "FUSION" label
  gfx->setTextColor(RGB565(60, 65, 80));
  gfx->setTextSize(1);
  gfx->setCursor(x - 20, y + 50);
  gfx->print("FUSION");
}

void drawDropZone(int x, int y, bool active) {
  uint16_t color = active ? BBB_BAND_ORANGE : RGB565(40, 42, 50);
  // Pixel square drop zone
  gfx->drawRect(x - 35, y - 35, 70, 70, color);
  if (!active) {
    // Dashed pixel effect
    for (int i = 0; i < 70; i += 8) {
      gfx->fillRect(x - 35 + i, y - 35, 4, 2, RGB565(50, 52, 60));
      gfx->fillRect(x - 35 + i, y + 33, 4, 2, RGB565(50, 52, 60));
    }
  }
}

void drawFusionButton(int x, int y) {
  int fusionResult = checkFusion(dropZone1Element, dropZone2Element);
  bool validFusion = (fusionResult >= 0);
  
  uint16_t btnColor = validFusion ? BBB_BAND_ORANGE : RGB565(40, 42, 55);
  
  // Retro pixel button
  gfx->fillRect(x - 50, y, 100, 40, btnColor);
  gfx->drawRect(x - 50, y, 100, 40, COLOR_WHITE);
  gfx->fillRect(x - 50, y, 5, 5, COLOR_WHITE);
  gfx->fillRect(x + 45, y, 5, 5, COLOR_WHITE);
  gfx->fillRect(x - 50, y + 35, 5, 5, COLOR_WHITE);
  gfx->fillRect(x + 45, y + 35, 5, 5, COLOR_WHITE);
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(x - 30, y + 10);
  gfx->print("FUSE!");
}

// =============================================================================
// ANIMATIONS
// =============================================================================

void drawFusionCombiningAnim(int x, int y) {
  // Retro pixel combining effect
  animFrame = (animFrame + 1) % 60;
  
  for (int i = 0; i < 12; i++) {
    int offset = (animFrame + i * 5) % 60;
    int px = x - 60 + (offset * 2);
    int py = y - 30 + (i * 5);
    
    uint16_t color = (i % 2) ? fusionElementColors[dropZone1Element] 
                             : fusionElementColors[dropZone2Element];
    gfx->fillRect(px, py, 6, 6, color);
  }
  
  // Center glow - pixel square growing
  int glowSize = animFrame / 2;
  gfx->fillRect(x - glowSize, y - glowSize, glowSize * 2, glowSize * 2, RGB565(255, 200, 100));
}

void drawFusionSuccessAnim(int x, int y) {
  // Retro pixel explosion
  for (int i = 0; i < 16; i++) {
    int len = 30 + (millis() / 10) % 30;
    int dx = (i % 4 - 2) * len / 2;
    int dy = (i / 4 - 2) * len / 2;
    gfx->fillRect(x + dx - 2, y + dy - 2, 5, 5, BBB_BAND_GLOW);
  }
  
  // Success text - retro
  gfx->fillRect(x - 55, y - 15, 110, 30, RGB565(10, 12, 18));
  gfx->drawRect(x - 55, y - 15, 110, 30, COLOR_GOLD);
  gfx->setTextColor(COLOR_GOLD);
  gfx->setTextSize(2);
  gfx->setCursor(x - 45, y - 8);
  gfx->print("SUCCESS!");
}

void drawFusionFailAnim(int x, int y) {
  // Red pixel X
  for (int i = 0; i < 30; i++) {
    gfx->fillRect(x - 30 + i * 2, y - 30 + i * 2, 4, 4, COLOR_RED);
    gfx->fillRect(x + 28 - i * 2, y - 30 + i * 2, 4, 4, COLOR_RED);
  }
  
  gfx->fillRect(x - 50, y + 35, 100, 20, RGB565(10, 12, 18));
  gfx->drawRect(x - 50, y + 35, 100, 20, COLOR_RED);
  gfx->setTextColor(COLOR_RED);
  gfx->setTextSize(1);
  gfx->setCursor(x - 35, y + 41);
  gfx->print("Cannot fuse!");
}

// =============================================================================
// TOUCH HANDLING
// =============================================================================

void handleFusionGameTouch(TouchGesture& gesture) {
  int x = gesture.x;
  int y = gesture.y;
  
  // Back button
  if (gesture.event == TOUCH_TAP && x < 80 && y > 400) {
    returnToAppGrid();
    return;
  }
  
  // Reset button
  if (gesture.event == TOUCH_TAP && x > 280 && y > 400) {
    dropZone1Element = -1;
    dropZone2Element = -1;
    fusion_state = FUSION_IDLE;
    drawFusionGame();
    return;
  }
  
  // Handle based on current state
  if (fusion_state == FUSION_SUCCESS || fusion_state == FUSION_FAIL) {
    // Tap anywhere to reset
    if (gesture.event == TOUCH_TAP) {
      fusion_state = FUSION_IDLE;
      dropZone1Element = -1;
      dropZone2Element = -1;
      drawFusionGame();
    }
    return;
  }
  
  // Fusion button tap
  if (gesture.event == TOUCH_TAP && dropZone1Element >= 0 && dropZone2Element >= 0) {
    int zoneX = LCD_WIDTH / 2;
    int zoneY = 200;
    
    if (y >= zoneY + 80 && y < zoneY + 120 && x >= zoneX - 50 && x < zoneX + 50) {
      // Try fusion
      int fusionResult = checkFusion(dropZone1Element, dropZone2Element);
      
      if (fusionResult >= 0) {
        fusion_state = FUSION_COMBINING;
        
        // Animate then show success
        delay(500);  // Simple delay for now
        
        fusion_state = FUSION_SUCCESS;
        
        // Unlock the fusion
        for (int i = 0; i < 6; i++) {
          if (fusion_combos[i].resultIndex == fusionResult) {
            fusion_combos[i].unlocked = true;
            unlockElement((BoBoiBoyForm)fusionResult);
            ochobotNotify("Fusion", fusion_combos[i].fusionName);
            break;
          }
        }
        
        drawFusionGame();
      } else {
        fusion_state = FUSION_FAIL;
        drawFusionGame();
      }
      return;
    }
  }
  
  // Check element orb touches (Two rows: base at 360, evolved at 410)
  int orbY1 = 360;  // Base elements row
  int orbY2 = 410;  // Evolved elements row
  int orbSpacing = 50;
  int startX = (LCD_WIDTH - 7 * orbSpacing) / 2 + orbSpacing / 2;
  
  if (gesture.event == TOUCH_PRESS || gesture.event == TOUCH_TAP) {
    // Check base elements (0-6)
    for (int i = 0; i < 7; i++) {
      if (i == dropZone1Element || i == dropZone2Element) continue;
      
      int orbX = startX + i * orbSpacing;
      int dist = sqrt((x - orbX) * (x - orbX) + (y - orbY1) * (y - orbY1));
      
      if (dist < 25) {
        if (gesture.event == TOUCH_TAP) {
          if (dropZone1Element < 0) {
            dropZone1Element = i;
          } else if (dropZone2Element < 0) {
            dropZone2Element = i;
          }
          drawFusionGame();
        } else {
          startDraggingElement(i, x, y);
        }
        return;
      }
    }
    
    // Check evolved elements (7-13)
    for (int i = 7; i < 14; i++) {
      if (i == dropZone1Element || i == dropZone2Element) continue;
      
      int orbX = startX + (i - 7) * orbSpacing;
      int dist = sqrt((x - orbX) * (x - orbX) + (y - orbY2) * (y - orbY2));
      
      if (dist < 25) {
        if (gesture.event == TOUCH_TAP) {
          if (dropZone1Element < 0) {
            dropZone1Element = i;
          } else if (dropZone2Element < 0) {
            dropZone2Element = i;
          }
          drawFusionGame();
        } else {
          startDraggingElement(i, x, y);
        }
        return;
      }
    }
    
    // Check if tapping drop zones to remove element
    int zoneX = LCD_WIDTH / 2;
    int zoneY = 200;
    
    if (gesture.event == TOUCH_TAP) {
      int dist1 = sqrt((x - (zoneX - 60)) * (x - (zoneX - 60)) + (y - zoneY) * (y - zoneY));
      int dist2 = sqrt((x - (zoneX + 60)) * (x - (zoneX + 60)) + (y - zoneY) * (y - zoneY));
      
      if (dist1 < 40 && dropZone1Element >= 0) {
        dropZone1Element = -1;
        drawFusionGame();
        return;
      }
      if (dist2 < 40 && dropZone2Element >= 0) {
        dropZone2Element = -1;
        drawFusionGame();
        return;
      }
    }
  }
  
  // Update drag
  if (dragged_element.isDragging) {
    if (gesture.event == TOUCH_RELEASE || gesture.event == TOUCH_TAP) {
      dropElement();
      drawFusionGame();
    } else {
      updateDragPosition(x, y);
      drawFusionGame();
    }
  }
}

void startDraggingElement(int elementIndex, int x, int y) {
  dragged_element.elementIndex = elementIndex;
  dragged_element.currentX = x;
  dragged_element.currentY = y;
  dragged_element.isDragging = true;
  
  Serial.printf("[FUSION] Started dragging element %d\n", elementIndex);
}

void updateDragPosition(int x, int y) {
  dragged_element.currentX = x;
  dragged_element.currentY = y;
  
  // Check if in drop zone
  int zoneX = LCD_WIDTH / 2;
  int zoneY = 200;
  int dist = sqrt((x - zoneX) * (x - zoneX) + (y - zoneY) * (y - zoneY));
  dragged_element.isInDropZone = (dist < 100);
}

void dropElement() {
  if (!dragged_element.isDragging) return;
  
  int x = dragged_element.currentX;
  int y = dragged_element.currentY;
  int elem = dragged_element.elementIndex;
  
  // Check if dropped in fusion zone
  int zoneX = LCD_WIDTH / 2;
  int zoneY = 200;
  
  int dist1 = sqrt((x - (zoneX - 60)) * (x - (zoneX - 60)) + (y - zoneY) * (y - zoneY));
  int dist2 = sqrt((x - (zoneX + 60)) * (x - (zoneX + 60)) + (y - zoneY) * (y - zoneY));
  
  if (dist1 < 50 && dropZone1Element < 0) {
    dropZone1Element = elem;
    Serial.printf("[FUSION] Dropped element %d in zone 1\n", elem);
  } else if (dist2 < 50 && dropZone2Element < 0) {
    dropZone2Element = elem;
    Serial.printf("[FUSION] Dropped element %d in zone 2\n", elem);
  }
  
  dragged_element.isDragging = false;
  dragged_element.elementIndex = -1;
}

int checkFusion(int elem1, int elem2) {
  if (elem1 < 0 || elem2 < 0) return -1;
  
  for (int i = 0; i < 6; i++) {
    if ((fusion_combos[i].element1 == elem1 && fusion_combos[i].element2 == elem2) ||
        (fusion_combos[i].element1 == elem2 && fusion_combos[i].element2 == elem1)) {
      return fusion_combos[i].resultIndex;
    }
  }
  
  return -1;  // No valid fusion
}

void playFusionAnimation(int fusionIndex) {
  // Animation handled in draw function based on state
}
