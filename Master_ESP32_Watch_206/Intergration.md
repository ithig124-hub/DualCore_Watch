"# ESP32 Watch - Story Mode & Companion System Integration Guide

## 📁 New Files Created

```
ESP32_Watch_206/
├── storyline.h       # Story Mode header
├── storyline.cpp     # Story Mode implementation
├── companion.h       # Virtual Pet header
├── companion.cpp     # Virtual Pet implementation
└── INTEGRATION_GUIDE.md  # This file
```

---

## 🔧 Integration Steps

### Step 1: Add Includes to ESP32_Watch_206.ino

Add these includes after your existing includes:

```cpp
// Add to ESP32_Watch_206.ino after other includes
#include \"storyline.h\"
#include \"companion.h\"
```

### Step 2: Initialize Systems in setup()

Add to your `setup()` function:

```cpp
void setup() {
    // ... existing setup code ...
    
    // Initialize new systems
    initStorySystem();
    initCompanionSystem();
    
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
    
    // Update companion stats periodically
    updateCompanionStats();
    updateCompanionAnimation();
    
    // Check for story events
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
    
    if (strcmp(appName, \"Story\") == 0) {
        setCurrentStory(system_state.current_theme);
        system_state.current_screen = SCREEN_STORY_MENU;
    }
    else if (strcmp(appName, \"Care\") == 0) {
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
            } else {
                drawCompanionScreen();
            }
            break;
        case SCREEN_COMPANION_GAME:
            drawCompanionGame();
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

### Step 8: Add App Icons to App Grid

Add to your app grid (apps.h or navigation.cpp):

```cpp
// Add these to your apps array
{\"Story\", STORY_COLOR},    // Story Mode app
{\"Care\", CARE_COLOR},      // Companion Care app

// Color definitions (add to config.h)
#define STORY_COLOR RGB565(200, 100, 50)  // Bronze/adventure
#define CARE_COLOR RGB565(255, 150, 200)  // Pink/caring
```

### Step 9: Character Switching Integration

When player changes character theme:

```cpp
void switchCharacter(ThemeType newTheme) {
    system_state.current_theme = newTheme;
    
    // Update companion
    setCurrentCompanion(newTheme);
    
    // Update story
    setCurrentStory(newTheme);
    
    // ... existing character switch code ...
}
```

---

## 📊 Data Persistence

### Save Data to SPIFFS/LittleFS

Add to your save system:

```cpp
void saveGameData() {
    // ... existing save code ...
    
    saveStoryProgress();
    saveCompanionData();
}

void loadGameData() {
    // ... existing load code ...
    
    loadStoryProgress();
    loadCompanionData();
}
```

### Storage Structure

```
/save/
├── story_luffy.dat      # Luffy's story progress
├── story_jinwoo.dat     # Jinwoo's story progress
├── story_yugo.dat       # Yugo's story + path choice
├── ... (other characters)
├── companion_data.dat   # All companion stats
└── events.dat           # Daily event tracking
```

---

## 🎮 Feature Summary

### Story Mode Features
- ✅ 10 chapters per character (unlocks via XP levels)
- ✅ Yugo-exclusive branching path (Dragons vs Portals)
- ✅ Story boss battles at chapter ends
- ✅ XP, Gems, and exclusive card rewards
- ✅ Time-based story events (Morning/Afternoon/Evening/Midnight)
- ✅ Dialogue system with choices

### Companion Features
- ✅ 11 unique companions (one per character)
- ✅ Care actions: Feed, Play, Train, Rest
- ✅ Stat decay over time (Hunger, Happiness, Energy)
- ✅ Bond level system (1-5 stars)
- ✅ Mood system with visual indicators
- ✅ Companion evolution (Baby → Child → Adult → Awakened)
- ✅ XP bonus based on bond level
- ✅ Mini-game for Play action

---

## 🐛 Known Dependencies

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
void drawBackButton(int x, int y);

// From display.cpp
extern Arduino_GFX* gfx;
```

---

## 📱 Memory Considerations

### RAM Usage Estimate
- Story System State: ~500 bytes
- Companion System State: ~2KB (all 11 companions)
- Total Additional: ~2.5KB

### Flash Usage Estimate
- storyline.cpp: ~15KB
- companion.cpp: ~12KB
- Total Additional: ~27KB

These should fit comfortably on ESP32-S3 with 16MB flash.

---

## 🔄 Testing Checklist

- [ ] Story menu opens correctly
- [ ] Chapter unlocks based on XP level
- [ ] Dialogue advances on tap
- [ ] Yugo shows branching choice at Chapter 3
- [ ] Story boss battle works
- [ ] Rewards are claimed correctly
- [ ] Companion screen shows correct companion for theme
- [ ] Feed costs 5 gems and works
- [ ] Play mini-game starts
- [ ] Train gives XP
- [ ] Rest/Wake toggles sleep
- [ ] Stats decay over time
- [ ] Bond level increases with care
- [ ] Daily events appear at correct times

---

## 📝 Customization Notes

### Adding More Story Dialogue

Edit `storyline.cpp` to add actual dialogue content:

```cpp
// In storyline.cpp, modify chapter initialization
stories[THEME_LUFFY_GEAR5].chapters[0].dialogues[0] = {
    \"Luffy\",
    \"I'm gonna be the King of the Pirates!\",
    LUFFY_SUN_GOLD,
    false, \"\", \"\", 0, 0
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

1. **Story Events**: Add more varied daily events with mini-quests
2. **Companion Interactions**: Let companions from different characters interact
3. **Story Multiplayer**: WiFi-based story co-op
4. **Companion Training Games**: Multiple mini-games for different stats
5. **Story Achievements**: Badges for completing story milestones
6. **Seasonal Events**: Holiday-themed story chapters and companion accessories

---

Happy coding! 🚀
"