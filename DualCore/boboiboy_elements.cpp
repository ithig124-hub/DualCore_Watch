/*
 * boboiboy_elements.cpp - BoBoiBoy Element Tree Implementation
 * Visual element tree showing all forms, evolutions, and fusions
 * Only accessible when BoBoiBoy theme is active
 */

#include "boboiboy_elements.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// Current state
int selected_element = 0;
int element_tree_page = 0;  // 0 = Base, 1 = Evolved, 2 = Fusions

// Element unlock status (for game progression)
static bool elements_unlocked[FORM_COUNT] = {
  true, true, true, true, true, true, true,  // Base elements always unlocked
  false, false, false, false, false, false, false,  // Tier 2 locked initially
  false, false, false, false, false, false   // Fusions locked initially
};

// =============================================================================
// ELEMENT DATA - All 20 BoBoiBoy Forms
// =============================================================================

ElementData boboiboy_elements[FORM_COUNT] = {
  // Tier 1 - Base Elements (7)
  {"Halilintar", "Lightning", BBB_LIGHTNING_YELLOW, BBB_LIGHTNING_BLACK, 
   TIER_BASE, "Lightning Speed", "First elemental power. Super speed and lightning attacks.", -1, -1},
  
  {"Taufan", "Wind", BBB_WIND_BLUE, BBB_WIND_CYAN,
   TIER_BASE, "Wind Control", "Controls wind and air currents. Can fly with hover board.", -1, -1},
  
  {"Gempa", "Earth", BBB_EARTH_BROWN, BBB_EARTH_ORANGE,
   TIER_BASE, "Earth Power", "Controls earth and rock. Creates golems and barriers.", -1, -1},
  
  {"Blaze", "Fire", BBB_FIRE_RED, BBB_FIRE_ORANGE,
   TIER_BASE, "Fire Control", "Controls fire with intense heat. Aggressive fighter.", -1, -1},
  
  {"Ice", "Ice", BBB_WATER_CYAN, BBB_WATER_WHITE,
   TIER_BASE, "Ice Creation", "Creates and controls ice. Calm and strategic.", -1, -1},
  
  {"Thorn", "Leaf", BBB_LEAF_GREEN, BBB_LEAF_DARK,
   TIER_BASE, "Plant Control", "Controls plants and vines. Nature powers.", -1, -1},
  
  {"Solar", "Light", BBB_LIGHT_GOLD, BBB_LIGHT_WHITE,
   TIER_BASE, "Light Energy", "Harnesses solar energy. Powerful light attacks.", -1, -1},
  
  // Tier 2 - Evolved Forms (7)
  {"Thunderstorm", "Guruh", BBB_THUNDERSTORM_BLACK, BBB_THUNDERSTORM_RED,
   TIER_EVOLVED, "Thunder Fury", "Evolved Halilintar. Dark lightning with red eyes.", FORM_HALILINTAR, -1},
  
  {"Cyclone", "Taufan+", BBB_CYCLONE_DARK_BLUE, BBB_WIND_CYAN,
   TIER_EVOLVED, "Storm Master", "Evolved Taufan. Creates powerful cyclones.", FORM_TAUFAN, -1},
  
  {"Quake", "Gempa+", BBB_QUAKE_DARK, BBB_EARTH_ORANGE,
   TIER_EVOLVED, "Earthquake", "Evolved Gempa. Massive earth manipulation.", FORM_GEMPA, -1},
  
  {"Inferno", "Blaze+", BBB_BLAZE_CRIMSON, BBB_FIRE_ORANGE,
   TIER_EVOLVED, "Hellfire", "Evolved Blaze. Blue-core flames.", FORM_BLAZE, -1},
  
  {"Glacier", "Ice+", BBB_ICE_LIGHT, BBB_WATER_CYAN,
   TIER_EVOLVED, "Absolute Zero", "Evolved Ice. Flash freeze abilities.", FORM_ICE, -1},
  
  {"Darkwood", "Thorn+", BBB_THORN_DARK, BBB_LEAF_GREEN,
   TIER_EVOLVED, "Forest Wrath", "Evolved Thorn. Dark nature powers.", FORM_THORN, -1},
  
  {"Supernova", "Solar+", BBB_SOLAR_ORANGE, BBB_LIGHT_GOLD,
   TIER_EVOLVED, "Star Power", "Evolved Solar. Stellar energy control.", FORM_SOLAR, -1},
  
  // Fusions (6) - OFFICIAL BOBOIBOY FUSIONS
  {"FrostFire", "Api Ais", BBB_FROSTFIRE_PINK, BBB_FIRE_RED,
   TIER_FUSION, "Hot & Cold", "Blaze + Ice fusion. Fire and ice combined.", FORM_BLAZE, FORM_ICE},
  
  {"Glacier", "Glasier", BBB_GLACIER_BLUE, BBB_EARTH_BROWN,
   TIER_FUSION, "Frozen Earth", "Quake + Ice fusion. Icy rock powers.", FORM_QUAKE, FORM_ICE},
  
  {"Supra", "Supra", BBB_SUPRA_GOLD, BBB_THUNDERSTORM_BLACK,
   TIER_FUSION, "Ultimate Power", "Thunderstorm + Solar. Strongest fusion.", FORM_THUNDERSTORM, FORM_SUPERNOVA},
  
  {"Sori", "Sori", BBB_LIGHT_GOLD, BBB_LEAF_GREEN,
   TIER_FUSION, "Light Thorns", "Thorn + Solar fusion. Nature and light.", FORM_DARKWOOD, FORM_SUPERNOVA},
  
  {"Rumble", "Rumble", BBB_EARTH_ORANGE, BBB_THUNDERSTORM_RED,
   TIER_FUSION, "Thunder Quake", "Quake + Thunderstorm. Seismic thunder.", FORM_QUAKE, FORM_THUNDERSTORM},
  
  {"Sopan", "Sopan", BBB_LIGHT_GOLD, BBB_WIND_CYAN,
   TIER_FUSION, "Solar Wind", "Solar + Cyclone. Light and storm.", FORM_SUPERNOVA, FORM_CYCLONE}
};

// =============================================================================
// INITIALIZATION
// =============================================================================

void initBoboiboyElements() {
  Serial.println("[BBB] Initializing element tree system...");
  selected_element = 0;
  element_tree_page = 0;
  
  // Base elements always unlocked
  for (int i = 0; i < 7; i++) {
    elements_unlocked[i] = true;
  }
}

// =============================================================================
// ELEMENT TREE DRAWING - MAIN SCREEN
// =============================================================================

void drawElementTree() {
  // ======================================================
  // BOBOIBOY POWER BAND STYLE - Like the actual show watch!
  // ======================================================
  
  // Dark background like the power band interface
  gfx->fillScreen(RGB565(5, 5, 10));
  
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  // Draw power band outer ring (like Ochobot's power band)
  for (int r = 170; r >= 155; r--) {
    gfx->drawCircle(centerX, centerY, r, RGB565(50, 30, 10));
  }
  gfx->drawCircle(centerX, centerY, 175, BBB_BAND_ORANGE);
  gfx->drawCircle(centerX, centerY, 176, BBB_BAND_GLOW);
  
  // Inner glowing ring
  for (int r = 150; r >= 140; r--) {
    uint8_t glow = 20 + (150 - r) * 3;
    gfx->drawCircle(centerX, centerY, r, RGB565(glow, glow/2, 0));
  }
  
  // Power band center - OCHOBOT style
  gfx->fillCircle(centerX, centerY, 55, RGB565(20, 15, 10));
  gfx->drawCircle(centerX, centerY, 55, BBB_BAND_ORANGE);
  gfx->drawCircle(centerX, centerY, 53, RGB565(80, 50, 20));
  
  // Ochobot eye style in center
  gfx->fillCircle(centerX, centerY, 25, RGB565(0, 150, 200));
  gfx->fillCircle(centerX, centerY, 18, RGB565(100, 200, 255));
  gfx->fillCircle(centerX - 5, centerY - 5, 6, COLOR_WHITE);
  
  // Page indicator at top
  gfx->fillRect(0, 0, LCD_WIDTH, 35, RGB565(15, 10, 5));
  gfx->drawFastHLine(0, 34, LCD_WIDTH, BBB_BAND_ORANGE);
  
  const char* pageNames[] = {"KUASA 7", "EVOLVED", "FUSION"};
  gfx->setTextColor(BBB_BAND_GLOW);
  gfx->setTextSize(2);
  int textW = strlen(pageNames[element_tree_page]) * 12;
  gfx->setCursor(centerX - textW/2, 8);
  gfx->print(pageNames[element_tree_page]);
  
  // Page dots
  for (int i = 0; i < 3; i++) {
    int dotX = centerX - 20 + i * 20;
    if (i == element_tree_page) {
      gfx->fillCircle(dotX, 28, 4, BBB_BAND_ORANGE);
    } else {
      gfx->drawCircle(dotX, 28, 3, RGB565(80, 50, 20));
    }
  }
  
  // Draw elements based on current page
  if (element_tree_page == 0) {
    drawBaseElements();
  } else if (element_tree_page == 1) {
    drawEvolvedElements();
  } else {
    drawFusionElements();
  }
  
  // Bottom instruction bar
  gfx->fillRect(0, LCD_HEIGHT - 35, LCD_WIDTH, 35, RGB565(15, 10, 5));
  gfx->drawFastHLine(0, LCD_HEIGHT - 35, LCD_WIDTH, BBB_BAND_ORANGE);
  gfx->setTextColor(RGB565(120, 90, 50));
  gfx->setTextSize(1);
  gfx->setCursor(centerX - 75, LCD_HEIGHT - 23);
  gfx->print("< SWIPE L/R FOR PAGES >");
  
  drawSwipeIndicator();
}

void drawBackButtonElement(int x, int y) {
  // Removed - now using swipe navigation
}

// =============================================================================
// BASE ELEMENTS (7) - Power Band Circular Layout like the show
// =============================================================================

void drawBaseElements() {
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  int radius = 115;
  
  // Draw 7 elements in a circle around the power band
  // Element colors from the show
  uint16_t elemColors[] = {
    BBB_LIGHTNING_YELLOW,  // Halilintar
    BBB_WIND_CYAN,         // Taufan
    BBB_EARTH_BROWN,       // Gempa
    BBB_FIRE_RED,          // Blaze
    BBB_WATER_CYAN,        // Ice
    BBB_LEAF_GREEN,        // Thorn
    BBB_LIGHT_GOLD         // Solar
  };
  
  const char* elemIcons[] = {"H", "T", "G", "B", "I", "Th", "S"};
  
  for (int i = 0; i < 7; i++) {
    float angle = (i * 51.43 - 90) * PI / 180.0;  // 360/7 = 51.43, start at top
    int nodeX = centerX + cos(angle) * radius;
    int nodeY = centerY + sin(angle) * radius;
    
    // Connection line to center (energy beam style)
    for (int w = -1; w <= 1; w++) {
      gfx->drawLine(centerX, centerY, nodeX, nodeY, RGB565(40, 30, 15));
    }
    
    // Outer glow
    gfx->fillCircle(nodeX, nodeY, 28, RGB565(30, 25, 15));
    
    // Element circle with color
    gfx->fillCircle(nodeX, nodeY, 22, elemColors[i]);
    gfx->drawCircle(nodeX, nodeY, 22, COLOR_WHITE);
    gfx->drawCircle(nodeX, nodeY, 24, RGB565(60, 45, 20));
    
    // Highlight
    gfx->fillCircle(nodeX - 6, nodeY - 6, 5, COLOR_WHITE);
    
    // Element initial
    gfx->setTextColor(RGB565(30, 20, 10));
    gfx->setTextSize(2);
    int txtW = strlen(elemIcons[i]) * 12;
    gfx->setCursor(nodeX - txtW/2, nodeY - 7);
    gfx->print(elemIcons[i]);
    
    // Element name below (small)
    gfx->setTextColor(RGB565(150, 120, 80));
    gfx->setTextSize(1);
    gfx->setCursor(nodeX - strlen(boboiboy_elements[i].name) * 3, nodeY + 30);
    gfx->print(boboiboy_elements[i].name);
  }
}

// =============================================================================
// EVOLVED ELEMENTS (7) - Power Band Style
// =============================================================================

void drawEvolvedElements() {
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  int radius = 115;
  
  // Evolved element colors (darker/more intense versions)
  uint16_t evolvedColors[] = {
    BBB_THUNDERSTORM_BLACK,  // Thunderstorm (evolved Halilintar)
    BBB_CYCLONE_DARK_BLUE,   // Cyclone (evolved Taufan)
    BBB_QUAKE_DARK,          // Quake (evolved Gempa)
    BBB_BLAZE_CRIMSON,       // Inferno (evolved Blaze)
    BBB_ICE_LIGHT,           // Glacier (evolved Ice)
    BBB_THORN_DARK,          // Darkwood (evolved Thorn)
    BBB_SOLAR_ORANGE         // Supernova (evolved Solar)
  };
  
  const char* evolvedIcons[] = {"Ts", "Cy", "Qu", "In", "Gl", "Dw", "Sn"};
  
  for (int i = 0; i < 7; i++) {
    float angle = (i * 51.43 - 90) * PI / 180.0;
    int nodeX = centerX + cos(angle) * radius;
    int nodeY = centerY + sin(angle) * radius;
    
    int elemIdx = i + 7;  // Evolved forms start at index 7
    bool unlocked = elements_unlocked[elemIdx];
    
    // Connection line
    for (int w = -1; w <= 1; w++) {
      gfx->drawLine(centerX, centerY, nodeX, nodeY, RGB565(50, 20, 20));
    }
    
    // Outer glow (red for evolved)
    gfx->fillCircle(nodeX, nodeY, 28, RGB565(40, 15, 15));
    
    if (unlocked) {
      // Element circle with color
      gfx->fillCircle(nodeX, nodeY, 22, evolvedColors[i]);
      gfx->drawCircle(nodeX, nodeY, 22, BBB_THUNDERSTORM_RED);
      gfx->drawCircle(nodeX, nodeY, 24, RGB565(80, 30, 30));
      
      // Highlight
      gfx->fillCircle(nodeX - 6, nodeY - 6, 4, COLOR_WHITE);
      
      // Element initial
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(2);
      int txtW = strlen(evolvedIcons[i]) * 12;
      gfx->setCursor(nodeX - txtW/2, nodeY - 7);
      gfx->print(evolvedIcons[i]);
    } else {
      // Locked - dark circle with lock
      gfx->fillCircle(nodeX, nodeY, 22, RGB565(25, 20, 20));
      gfx->drawCircle(nodeX, nodeY, 22, RGB565(60, 40, 40));
      gfx->setTextColor(RGB565(80, 60, 60));
      gfx->setTextSize(2);
      gfx->setCursor(nodeX - 5, nodeY - 7);
      gfx->print("?");
    }
    
    // Element name below
    gfx->setTextColor(unlocked ? RGB565(180, 100, 80) : RGB565(80, 60, 60));
    gfx->setTextSize(1);
    gfx->setCursor(nodeX - strlen(boboiboy_elements[elemIdx].name) * 3, nodeY + 30);
    gfx->print(boboiboy_elements[elemIdx].name);
  }
}

// =============================================================================
// FUSION ELEMENTS (6) - Power Band Style Hexagonal Layout
// =============================================================================

void drawFusionElements() {
  int centerX = LCD_WIDTH / 2;
  int centerY = LCD_HEIGHT / 2;
  
  // Fusion element data
  uint16_t fusionColors[] = {
    BBB_FROSTFIRE_PINK,   // FrostFire
    BBB_GLACIER_BLUE,     // Glacier  
    BBB_SUPRA_GOLD,       // Supra
    BBB_LIGHT_GOLD,       // Sori
    BBB_EARTH_ORANGE,     // Rumble
    BBB_WIND_CYAN         // Sopan
  };
  
  const char* fusionNames[] = {"FrostFire", "Glacier", "Supra", "Sori", "Rumble", "Sopan"};
  
  // Draw 6 fusions in hexagonal pattern
  int positions[][2] = {
    {centerX - 80, centerY - 60},   // Top left
    {centerX + 80, centerY - 60},   // Top right
    {centerX - 120, centerY + 30},  // Middle left
    {centerX + 120, centerY + 30},  // Middle right
    {centerX - 80, centerY + 120},  // Bottom left
    {centerX + 80, centerY + 120}   // Bottom right
  };
  
  for (int i = 0; i < 6; i++) {
    int fusionIdx = 14 + i;  // Fusions start at index 14
    int nodeX = positions[i][0];
    int nodeY = positions[i][1];
    bool unlocked = elements_unlocked[fusionIdx];
    
    // Energy lines connecting to center
    gfx->drawLine(centerX, centerY, nodeX, nodeY, RGB565(60, 30, 50));
    
    // Fusion glow (pink/purple for fusions)
    gfx->fillCircle(nodeX, nodeY, 32, RGB565(40, 20, 35));
    
    if (unlocked) {
      // Dual-color gradient effect for fusion
      gfx->fillCircle(nodeX, nodeY, 26, fusionColors[i]);
      gfx->drawCircle(nodeX, nodeY, 26, BBB_FROSTFIRE_PINK);
      gfx->drawCircle(nodeX, nodeY, 28, RGB565(100, 50, 80));
      
      // Inner highlight
      gfx->fillCircle(nodeX - 7, nodeY - 7, 5, COLOR_WHITE);
      
      // Fusion symbol (two halves)
      gfx->drawLine(nodeX - 8, nodeY - 10, nodeX + 8, nodeY + 10, COLOR_WHITE);
      gfx->fillCircle(nodeX - 8, nodeY - 10, 3, COLOR_WHITE);
      gfx->fillCircle(nodeX + 8, nodeY + 10, 3, COLOR_WHITE);
    } else {
      gfx->fillCircle(nodeX, nodeY, 26, RGB565(30, 20, 25));
      gfx->drawCircle(nodeX, nodeY, 26, RGB565(60, 40, 50));
      gfx->setTextColor(RGB565(80, 50, 60));
      gfx->setTextSize(2);
      gfx->setCursor(nodeX - 5, nodeY - 7);
      gfx->print("?");
    }
    
    // Fusion name
    gfx->setTextColor(unlocked ? BBB_FROSTFIRE_PINK : RGB565(80, 50, 60));
    gfx->setTextSize(1);
    int nameW = strlen(fusionNames[i]) * 6;
    gfx->setCursor(nodeX - nameW/2, nodeY + 35);
    gfx->print(fusionNames[i]);
  }
}

// =============================================================================
// ELEMENT NODE DRAWING
// =============================================================================

void drawElementNode(int x, int y, BoBoiBoyForm form, bool selected, bool unlocked) {
  ElementData& elem = boboiboy_elements[(int)form];
  int radius = 28;
  
  if (unlocked) {
    // Filled circle with element color
    gfx->fillCircle(x, y, radius, elem.primary_color);
    
    // Selection highlight
    if (selected) {
      gfx->drawCircle(x, y, radius + 2, COLOR_WHITE);
      gfx->drawCircle(x, y, radius + 4, elem.secondary_color);
    } else {
      gfx->drawCircle(x, y, radius, RGB565(80, 80, 90));
    }
    
    // Element initial
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    char initial[2] = {elem.name[0], '\0'};
    gfx->setCursor(x - 6, y - 8);
    gfx->print(initial);
    
  } else {
    // Locked - gray circle with question mark
    gfx->fillCircle(x, y, radius, RGB565(30, 30, 35));
    gfx->drawCircle(x, y, radius, RGB565(50, 50, 55));
    gfx->setTextColor(RGB565(60, 60, 65));
    gfx->setTextSize(2);
    gfx->setCursor(x - 6, y - 8);
    gfx->print("?");
  }
}

// =============================================================================
// ELEMENT INFO CARD
// =============================================================================

void drawElementInfoCard(ElementData& elem, int y) {
  int cardX = 20;
  int cardW = LCD_WIDTH - 40;
  int cardH = 70;
  
  // Card background
  gfx->fillRect(cardX, y, cardW, cardH, RGB565(25, 27, 35));
  gfx->drawRect(cardX, y, cardW, cardH, elem.primary_color);
  
  // Color stripe
  gfx->fillRect(cardX, y, 8, cardH, elem.primary_color);
  gfx->fillRect(cardX + 4, y, 4, cardH, elem.primary_color);
  
  // Element name
  gfx->setTextColor(elem.primary_color);
  gfx->setTextSize(2);
  gfx->setCursor(cardX + 20, y + 8);
  gfx->print(elem.name);
  
  // Malay name
  gfx->setTextColor(RGB565(120, 120, 130));
  gfx->setTextSize(1);
  gfx->setCursor(cardX + 20 + strlen(elem.name) * 12 + 10, y + 12);
  gfx->print(elem.malay_name);
  
  // Power
  gfx->setTextColor(elem.secondary_color);
  gfx->setCursor(cardX + 20, y + 32);
  gfx->print(elem.power);
  
  // Description (truncated)
  gfx->setTextColor(RGB565(150, 150, 160));
  gfx->setCursor(cardX + 20, y + 48);
  char desc[45];
  strncpy(desc, elem.description, 44);
  desc[44] = '\0';
  gfx->print(desc);
}

// =============================================================================
// TOUCH HANDLING
// =============================================================================

void handleElementTreeTouch(TouchGesture& gesture) {
  if (gesture.event == TOUCH_SWIPE_LEFT || gesture.event == TOUCH_SWIPE_RIGHT) {
    // Change page
    if (gesture.event == TOUCH_SWIPE_LEFT && element_tree_page < 2) {
      element_tree_page++;
      selected_element = -1;
      drawElementTree();
    } else if (gesture.event == TOUCH_SWIPE_RIGHT && element_tree_page > 0) {
      element_tree_page--;
      selected_element = -1;
      drawElementTree();
    }
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x;
  int y = gesture.y;
  
  // Back button
  if (x >= 10 && x < 80 && y >= 405 && y < 435) {
    returnToAppGrid();
    return;
  }
  
  // Tab selection
  if (y >= 50 && y < 78) {
    for (int i = 0; i < 3; i++) {
      int tabX = 20 + i * 115;
      if (x >= tabX && x < tabX + 105) {
        element_tree_page = i;
        selected_element = -1;
        drawElementTree();
        return;
      }
    }
  }
  
  // Element selection based on page
  if (element_tree_page == 0) {
    // Base elements - circular layout
    int centerX = LCD_WIDTH / 2;
    int centerY = 230;
    int radius = 100;
    
    for (int i = 0; i < 7; i++) {
      float angle = (i * 51.43 - 90) * PI / 180.0;
      int nodeX = centerX + cos(angle) * radius;
      int nodeY = centerY + sin(angle) * radius;
      
      int dist = sqrt((x - nodeX) * (x - nodeX) + (y - nodeY) * (y - nodeY));
      if (dist < 35) {
        selected_element = i;
        drawElementTree();
        return;
      }
    }
  }
  else if (element_tree_page == 1) {
    // Evolved elements
    int startY = 95;
    int spacing = 85;
    
    for (int i = 0; i < 7; i++) {
      int row = i / 2;
      int col = i % 2;
      int evolvedX = 50 + col * 180 + 90;
      int elemY = startY + row * spacing;
      
      if (i == 6) {
        evolvedX = LCD_WIDTH / 2 + 45;
        elemY = startY + 3 * spacing;
      }
      
      int dist = sqrt((x - evolvedX) * (x - evolvedX) + (y - elemY) * (y - elemY));
      if (dist < 30) {
        selected_element = 7 + i;
        drawElementTree();
        return;
      }
    }
  }
  else {
    // Fusion elements
    int startY = 105;
    int spacing = 95;
    
    for (int i = 0; i < 6; i++) {
      int row = i / 2;
      int col = i % 2;
      int fusionX = 90 + col * 190;
      int fusionY = startY + row * spacing + 35;
      
      int dist = sqrt((x - fusionX) * (x - fusionX) + (y - fusionY) * (y - fusionY));
      if (dist < 35) {
        selected_element = 14 + i;
        drawElementTree();
        return;
      }
    }
  }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

bool isElementUnlocked(BoBoiBoyForm form) {
  return elements_unlocked[(int)form];
}

void unlockElement(BoBoiBoyForm form) {
  elements_unlocked[(int)form] = true;
  Serial.printf("[BBB] Unlocked element: %s\n", boboiboy_elements[(int)form].name);
}

uint16_t getElementColor(BoBoiBoyForm form) {
  return boboiboy_elements[(int)form].primary_color;
}

const char* getElementName(BoBoiBoyForm form) {
  return boboiboy_elements[(int)form].name;
}
