# ESP32 Watch - Story Mode & Companion System with NVS Persistence

## 📁 Modified Files

```
ESP32_Watch_206/
├── storyline.h       # Story Mode header (NVS persistence)
├── storyline.cpp     # Story Mode implementation (NVS persistence)
├── companion.h       # Virtual Pet header (NVS persistence)
├── companion.cpp     # Virtual Pet implementation (NVS persistence)
└── NVS_INTEGRATION_GUIDE.md  # This file
```

---

## 🔧 What Changed - NVS Persistence

The original implementation used SPIFFS/LittleFS for data persistence. This update replaces file-based storage with **ESP32 NVS (Non-Volatile Storage)** using the `Preferences` library.

### Benefits of NVS:
- **Wear-leveling**: NVS handles flash wear automatically
- **Atomic writes**: Less risk of data corruption
- **No file system overhead**: Faster read/write operations
- **Built-in ESP32 support**: Part of the ESP-IDF framework

---

## 📦 Integration Steps

### Step 1: Add Includes to ESP32_Watch_206.ino

Add these includes after your existing includes:

```cpp
// Add to ESP32_Watch_206.ino after other includes
#include <Preferences.h>  // Required for NVS
#include "storyline.h"
#include "companion.h"
```

### Step 2: Initialize Systems in setup()

Add to your `setup()` function:

```cpp
void setup() {
    // ... existing setup code ...
    
    // Initialize new systems (NVS is initialized internally)
    initStorySystem();      // Loads story progress from NVS
    initCompanionSystem();  // Loads companion data from NVS
    
    // Set companion for current character
    setCurrentCompanion(system_state.current_theme);
    setCurrentStory(system_state.current_theme);
}
```

### Step 3: Add to Main Loop

Add to your `loop()` function:

```cpp
void loop() {
    // ... existing loop code ...
    
    // Update companion stats periodically (handles NVS save internally)
    updateCompanionStats();
    updateCompanionAnimation();
    
    // Check for story events (handles NVS save on day change)
    checkDailyStoryEvents();
}
```

### Step 4: Add Screen Types to config.h

Add these to the `ScreenType` enum in config.h:

```cpp
enum ScreenType {
    // ... existing screens ...
    SCREEN_STORY_MENU,      // Story mode main menu
    SCREEN_CHAPTER_SELECT,  // Chapter selection
    SCREEN_STORY_DIALOGUE,  // Story dialogue
    SCREEN_STORY_BOSS,      // Story boss battle
    SCREEN_COMPANION,       // Companion care screen
    SCREEN_COMPANION_GAME,  // Companion mini-game
};
```

### Step 5: Add to Navigation (navigation.cpp)

Add to `handleAppGridTap()`:

```cpp
void handleAppGridTap(int x, int y) {
    // ... existing app handling ...
    
    if (strcmp(appName, "Story") == 0) {
        setCurrentStory(system_state.current_theme);
        system_state.current_screen = SCREEN_STORY_MENU;
    }
    else if (strcmp(appName, "Care") == 0) {
        setCurrentCompanion(system_state.current_theme);
        system_state.current_screen = SCREEN_COMPANION;
    }
}
```

### Step 6: Add to Screen Drawing

Add to your screen drawing switch statement:

```cpp
void drawCurrentScreen() {
    switch (system_state.current_screen) {
        // ... existing cases ...
        
        case SCREEN_STORY_MENU:
            drawStoryMenu();
            break;
        case SCREEN_CHAPTER_SELECT:
            drawChapterSelect();
            break;
        case SCREEN_STORY_DIALOGUE:
            if (isYugoDecisionPoint()) {
                drawYugoPathChoice();
            } else {
                drawDialogueScreen();
            }
            break;
        case SCREEN_STORY_BOSS:
            drawStoryBossScreen();
            break;
        case SCREEN_COMPANION:
            if (companion_system.in_care_mode) {
                drawCareMenu();
            } else if (companion_system.in_mini_game) {
                drawCompanionGame();
            } else {
                drawCompanionScreen();
            }
            break;
        case SCREEN_COMPANION_GAME:
            drawCompanionGame();
            updateCompanionGame();
            break;
    }
}
```

### Step 7: Add to Touch Handling

Add to your touch handling:

```cpp
void handleCurrentScreenTouch(TouchGesture& gesture) {
    switch (system_state.current_screen) {
        // ... existing cases ...
        
        case SCREEN_STORY_MENU:
            handleStoryMenuTouch(gesture);
            break;
        case SCREEN_CHAPTER_SELECT:
            handleChapterSelectTouch(gesture);
            break;
        case SCREEN_STORY_DIALOGUE:
            handleDialogueTouch(gesture);
            break;
        case SCREEN_STORY_BOSS:
            handleStoryBossTouch(gesture);
            break;
        case SCREEN_COMPANION:
            if (companion_system.in_care_mode) {
                handleCareMenuTouch(gesture);
            } else if (companion_system.in_mini_game) {
                handleCompanionGameTouch(gesture);
            } else {
                handleCompanionScreenTouch(gesture);
            }
            break;
        case SCREEN_COMPANION_GAME:
            handleCompanionGameTouch(gesture);
            break;
    }
}
```

### Step 8: Character Switching Integration

When player changes character theme:

```cpp
void switchCharacter(ThemeType newTheme) {
    system_state.current_theme = newTheme;
    
    // Update companion (saves current to NVS, loads new)
    setCurrentCompanion(newTheme);
    
    // Update story
    setCurrentStory(newTheme);
    
    // ... existing character switch code ...
}
```

---

## 💾 NVS Storage Structure

### Story Data (Namespace: "story_data")
```
Keys:
├── initialized        # bool - Data exists flag
├── last_event_day     # int - Last event check day
├── event_X_done       # bool - Event X completed today
├── yugo_path          # int - Yugo's path choice
│
├── sX_chapter         # int - Character X current chapter
├── sX_completed       # int - Character X chapters completed
├── sX_done            # bool - Character X story complete
│
├── sX_cY_done         # bool - Character X, Chapter Y completed
├── sX_cY_boss         # bool - Character X, Chapter Y boss defeated
└── sX_cY_reward       # bool - Character X, Chapter Y rewards claimed
```

### Companion Data (Namespace: "companion_data")
```
Keys:
├── initialized        # bool - Data exists flag
├── current_idx        # int - Current companion index
│
├── cX_hunger          # int - Companion X hunger
├── cX_happy           # int - Companion X happiness
├── cX_energy          # int - Companion X energy
├── cX_bond            # int - Companion X bond level
├── cX_rank            # int - Companion X bond rank
├── cX_mood            # int - Companion X mood
├── cX_evo             # int - Companion X evolution
├── cX_interact        # int - Companion X total interactions
├── cX_days            # int - Companion X days together
│
├── cX_feed_t          # ulong - Last feed time
├── cX_play_t          # ulong - Last play time
├── cX_train_t         # ulong - Last train time
├── cX_sleep           # bool - Is sleeping
├── cX_d_feed          # int - Daily feed count
├── cX_d_play          # int - Daily play count
└── cX_d_train         # int - Daily train count
```

---

## 📊 NVS Space Usage

### Estimated NVS Usage:
- Story System: ~2KB (all characters + events)
- Companion System: ~3KB (all 11 companions)
- **Total**: ~5KB

ESP32 default NVS partition is typically 20KB, so there's plenty of room.

---

## 🔄 Auto-Save Behavior

### Story System saves to NVS when:
- Chapter is started
- Chapter is completed
- Boss is defeated
- Rewards are claimed
- Yugo path is chosen
- Daily events change

### Companion System saves to NVS when:
- Care action performed (feed, play, train, sleep)
- Companion switched
- Stats updated (every minute during decay)
- Mini-game ends

---

## 🔧 Factory Reset Functions

To clear all saved data:

```cpp
// Clear story progress
clearAllStoryProgress();

// Clear companion data
clearAllCompanionData();
```

---

## 🎮 Feature Summary

### Story Mode Features
- ✅ 10 chapters per character (unlocks via XP levels)
- ✅ Yugo-exclusive branching path (Dragons vs Portals)
- ✅ Story boss battles at chapter ends
- ✅ XP, Gems, and exclusive card rewards
- ✅ Time-based story events (Morning/Afternoon/Evening/Midnight)
- ✅ **NVS persistent storage** (survives power cycles)

### Companion Features
- ✅ 11 unique companions (one per character)
- ✅ Care actions: Feed, Play, Train, Rest
- ✅ Stat decay over time (Hunger, Happiness, Energy)
- ✅ Bond level system (1-5 stars)
- ✅ Mood system with visual indicators
- ✅ Companion evolution (Baby → Child → Adult → Awakened)
- ✅ XP bonus based on bond level
- ✅ Mini-game for Play action
- ✅ **NVS persistent storage** (survives power cycles)

---

## 🐛 Dependencies

The code references these external functions - ensure they exist:

```cpp
// From config.h/themes.cpp
ThemeColors getThemeColors(ThemeType theme);
WatchTime getCurrentTime();

// From xp_system.cpp
void gainExperience(int amount, const char* source);
CharacterXPData* getCurrentCharacterXP();

// From navigation.cpp
void returnToAppGrid();

// From display.cpp
extern Arduino_GFX* gfx;

// Built-in (ESP32)
#include <Preferences.h>
```

---

## 📱 Memory Considerations

### RAM Usage Estimate
- Story System State: ~500 bytes
- Companion System State: ~2.5KB (all 11 companions)
- Total Additional RAM: ~3KB

### Flash/NVS Usage Estimate
- storyline.cpp: ~20KB code
- companion.cpp: ~15KB code
- NVS data: ~5KB
- Total Additional Flash: ~40KB

These should fit comfortably on ESP32-S3 with 16MB flash.

---

## 🔄 Testing Checklist

### Story Mode Tests
- [ ] Story menu opens correctly
- [ ] Chapter unlocks based on XP level
- [ ] Dialogue advances on tap
- [ ] Yugo shows branching choice at Chapter 3
- [ ] Story boss battle works
- [ ] Rewards are claimed correctly
- [ ] **Progress saves after power cycle**
- [ ] **Yugo path persists after reset**

### Companion Tests
- [ ] Companion screen shows correct companion for theme
- [ ] Feed costs 5 gems and works
- [ ] Play mini-game starts
- [ ] Train gives XP
- [ ] Rest/Wake toggles sleep
- [ ] Stats decay over time
- [ ] Bond level increases with care
- [ ] **All companion data persists after power cycle**
- [ ] **Daily counts reset at midnight**

---

## 📝 Customization Notes

### Adding More Story Dialogue

Edit `storyline.cpp` to add actual dialogue content:

```cpp
// In storyline.cpp, modify chapter initialization
stories[THEME_LUFFY_GEAR5].chapters[0].dialogues[0] = {
    "Luffy",
    "I'm gonna be the King of the Pirates!",
    LUFFY_SUN_GOLD,
    false, "", "", 0, 0
};
stories[THEME_LUFFY_GEAR5].chapters[0].dialogue_count = 3;
```

### Customizing Companion Sprites

The `drawCompanionSprite()` function uses simple shapes. Replace with actual sprites:

```cpp
void drawCompanionSprite(int x, int y, CompanionType type, CompanionEvolution evo) {
    // Replace with bitmap/sprite drawing
    // gfx->drawBitmap(x, y, companion_sprites[type][evo], width, height);
}
```

---

## 💡 Future Enhancement Ideas

1. **Cloud Sync**: Backup NVS data to a server via WiFi
2. **Multiple Save Slots**: Allow different save profiles
3. **Data Export**: Export save data to SD card
4. **Companion Trading**: WiFi-based companion interaction
5. **Story Achievements**: Badges stored in NVS
6. **Seasonal Events**: Time-limited story chapters

---

Happy coding! 🚀
