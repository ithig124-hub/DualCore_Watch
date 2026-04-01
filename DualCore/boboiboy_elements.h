/*
 * boboiboy_elements.h - BoBoiBoy Element Tree System
 * Shows all 7 elements, Tier 2 evolutions, and Fusions
 * Only accessible when BoBoiBoy theme is active
 */

#ifndef BOBOIBOY_ELEMENTS_H
#define BOBOIBOY_ELEMENTS_H

#include "config.h"

// =============================================================================
// ELEMENT DEFINITIONS
// =============================================================================

// Element Tiers
enum ElementTier {
  TIER_BASE = 1,      // Original 7 elements
  TIER_EVOLVED = 2,   // Tier 2 evolutions
  TIER_FUSION = 3     // Element fusions
};

// All BoBoiBoy Forms
enum BoBoiBoyForm {
  // Tier 1 - Base Elements (7)
  FORM_HALILINTAR = 0,  // Lightning
  FORM_TAUFAN,          // Wind/Cyclone
  FORM_GEMPA,           // Earth/Quake
  FORM_BLAZE,           // Fire
  FORM_ICE,             // Water/Ice
  FORM_THORN,           // Leaf/Plant
  FORM_SOLAR,           // Light/Sun
  
  // Tier 2 - Evolved Forms (7)
  FORM_THUNDERSTORM,    // Halilintar Tier 2
  FORM_CYCLONE,         // Taufan Tier 2
  FORM_QUAKE,           // Gempa Tier 2
  FORM_INFERNO,         // Blaze Tier 2
  FORM_GLACIER,         // Ice Tier 2
  FORM_DARKWOOD,        // Thorn Tier 2
  FORM_SUPERNOVA,       // Solar Tier 2
  
  // Fusions (6)
  FORM_FROSTFIRE,       // Blaze + Ice
  FORM_VOLCANICE,       // Gempa + Ice
  FORM_THUNDERBLAZE,    // Halilintar + Blaze
  FORM_SUPRA,           // Thunderstorm + Solar
  FORM_GENTAR,          // Gempa + Halilintar
  FORM_BELIUNG,         // Taufan + Halilintar
  
  FORM_COUNT = 20
};

// Element data structure
struct ElementData {
  const char* name;
  const char* malay_name;
  uint16_t primary_color;
  uint16_t secondary_color;
  ElementTier tier;
  const char* power;
  const char* description;
  int parent1;  // -1 for base elements
  int parent2;  // -1 for non-fusions
};

// External element data array
extern ElementData boboiboy_elements[FORM_COUNT];

// Current selected element in tree view
extern int selected_element;
extern int element_tree_page;

// =============================================================================
// ELEMENT TREE FUNCTIONS
// =============================================================================

// Initialize element system
void initBoboiboyElements();

// Draw the element tree screen (main view)
void drawElementTree();

// Draw element detail view
void drawElementDetail(BoBoiBoyForm form);

// Draw element node in tree
void drawElementNode(int x, int y, BoBoiBoyForm form, bool selected, bool unlocked);

// Draw connection lines between elements
void drawElementConnections();

// Draw fusion diagram
void drawFusionDiagram(BoBoiBoyForm fusion);

// Handle touch on element tree
void handleElementTreeTouch(TouchGesture& gesture);

// Check if element is unlocked (game progression)
bool isElementUnlocked(BoBoiBoyForm form);

// Unlock an element
void unlockElement(BoBoiBoyForm form);

// Get element color
uint16_t getElementColor(BoBoiBoyForm form);

// Get element name
const char* getElementName(BoBoiBoyForm form);

// Internal drawing functions
void drawBaseElements();
void drawEvolvedElements();
void drawFusionElements();
void drawBackButtonElement(int x, int y);
void drawElementInfoCard(ElementData& element, int y);

#endif // BOBOIBOY_ELEMENTS_H
