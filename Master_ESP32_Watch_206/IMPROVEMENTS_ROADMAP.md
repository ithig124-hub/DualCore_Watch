# ESP32 Watch - Feature Improvements Roadmap

## Overview
This document outlines improvements to make the RPG system more functional throughout the watch, improve quest visibility, and remove unnecessary features.

---

## 1. REMOVED: Notes App
The Notes app has been removed from the app grid since it's not needed on a smartwatch.

**In navigation.cpp:**
- Removed "NOTES" from app grid page 1
- Replaced with "TIMER" or another useful app

---

## 2. IMPROVED: Daily Quests (Scrollable & Visible)

### Changes Made:
- **Larger quest cards** (90px height vs 85px)
- **Bigger claim buttons** (75x25px) with better touch targets
- **Scroll indicators** showing which quest you're viewing
- **RPG level display** in header showing current level
- **XP rewards shown prominently** for RPG integration

### Touch Behavior:
- **TAP on CLAIM button** → Claims reward + gives RPG XP
- **SWIPE UP** → Exit to app grid
- **SWIPE DOWN** → Reserved for scrolling (if more quests added)

---

## 3. RPG INTEGRATION THROUGHOUT THE WATCH

### How RPG Connects to Everything:

```
┌─────────────────────────────────────────────────────────────────┐
│                        RPG CHARACTER                            │
│                     (Luffy/Jinwoo/Yugo)                         │
│                                                                 │
│  Level 1-100  │  XP System  │  Stats  │  Titles  │  Abilities  │
└───────┬───────────────┬─────────────┬───────────────┬───────────┘
        │               │             │               │
        ▼               ▼             ▼               ▼
┌───────────┐   ┌───────────┐   ┌───────────┐   ┌───────────┐
│  QUESTS   │   │  TRAINING │   │ BOSS RUSH │   │   STEPS   │
│           │   │           │   │           │   │           │
│ Complete  │   │ Complete  │   │ Defeat    │   │ Walk more │
│ = XP      │   │ = XP      │   │ = XP      │   │ = XP      │
│ + Gems    │   │ + Stats   │   │ + Gems    │   │ + Quest   │
└───────────┘   └───────────┘   └───────────┘   └───────────┘
```

### XP Sources (Currently Implemented):
1. **Daily Quests** → 50-200 XP per quest
2. **Quest Streak Bonus** → streak_days × 10 XP
3. **Boss Rush** → XP based on boss difficulty
4. **Training** → XP for completed workouts

### XP Sources (To Add):
1. **Steps** → 1 XP per 100 steps
2. **Active Minutes** → 2 XP per minute
3. **Gacha Pulls** → 5 XP per pull
4. **Daily Login** → 25 XP bonus

---

## 4. SUGGESTED IMPROVEMENTS

### A. Steps → XP Integration
```cpp
// In steps_tracker.cpp, add to updateStepCount():
static uint32_t last_xp_steps = 0;
if (steps_data.steps_today - last_xp_steps >= 100) {
    gainExperience(1, "Walking");
    last_xp_steps = steps_data.steps_today;
}
```

### B. Character Stats Screen Improvements
Show on Character Stats screen:
- Current RPG character (based on theme)
- Level + Title
- XP bar with progress
- Today's XP earned
- Stats breakdown

### C. Watchface RPG Widget
Add small RPG status to watchface:
- Level badge in corner
- XP progress arc around time
- Daily quest completion indicator

### D. Training → RPG Stats
When completing training:
- STR workouts → +1 Strength (per session)
- Cardio → +1 Speed
- Focus training → +1 Intelligence

### E. Boss Rush → RPG Integration
- Boss difficulty scales with RPG level
- Defeating bosses = large XP rewards
- Unlocking new bosses at level milestones

---

## 5. APP GRID CHANGES

### Page 1 (Main Apps):
| Before | After |
|--------|-------|
| GACHA | GACHA |
| TRAINING | TRAINING |
| BOSS | BOSS |
| QUESTS → | **ELEMENTS** (for BoBoiBoy) / **TIMER** (others) |
| MUSIC | MUSIC |
| WEATHER | WEATHER |
| WIFI | WIFI |
| **NOTES** | **REMOVED** → **COMPASS** |
| SETTINGS | SETTINGS |

### Page 2 (Utilities):
| App | Purpose |
|-----|---------|
| THEMES | Change watch theme |
| COLLECT | Gacha collection |
| FILES | File browser |
| CALC | Calculator |
| TORCH | Flashlight |
| OTA | Firmware update |
| BACKUP | Save data |
| FUSION | BoBoiBoy fusion game |
| ABOUT | Watch info |

### Page 3 (New Features):
| App | Purpose |
|-----|---------|
| GAMES | Game menu |
| STEPS | Step tracker |
| ACHIEVE | Achievements |
| SHOP | In-game shop |
| SOCIAL | Coming soon |
| GALLERY | Photo viewer |
| TIMER | Stopwatch/Timer |
| COMPASS | Magnetometer compass |
| CONVERT | Unit converter |

---

## 6. SENSOR UTILIZATION

### Currently Used:
- **QMI8658** → Steps (accelerometer)
- **AXP2101** → Battery percentage
- **PCF85063** → Time/Date
- **FT3168** → Touch input

### To Improve:
- **QMI8658 Gyroscope** → Tilt detection for games, raise-to-wake
- **QMI8658 Compass** → Magnetometer for compass app
- **QMI8658 Pedometer** → Hardware step counter (more accurate)
- **Temperature sensor** → If available, show on watchface

---

## 7. IMPLEMENTATION PRIORITY

### Phase 1 (Quick Wins):
1. ✅ Remove Notes app
2. ✅ Improve quest visibility (larger cards)
3. ✅ Add RPG XP to quest rewards
4. Add steps → XP integration

### Phase 2 (RPG Enhancement):
1. Add XP widget to watchface
2. Training → stat bonuses
3. Boss difficulty scaling
4. Level-up notifications

### Phase 3 (Full Integration):
1. Achievement system
2. Daily login bonus
3. Character abilities unlock at levels
4. Social features (if WiFi connected)

---

## 8. CODE CHANGES SUMMARY

### Files Modified:
1. `daily_quests.cpp` - Scrollable, RPG integration
2. `navigation.cpp` - Remove Notes, add Compass/Timer
3. `steps_tracker.cpp` - Add XP for walking
4. `training.cpp` - Add stat bonuses
5. `boss_rush.cpp` - Level scaling

### New Connections:
```cpp
// In any activity that should give XP:
#include "rpg.h"
gainExperience(amount, "Source Name");

// To check player level:
int level = getPlayerLevel();

// To get current character:
RPGCharacterData* char = getCurrentCharacter();
```

---

## 9. USER EXPERIENCE FLOW

```
User wakes watch (touch/button)
         │
         ▼
    ┌─────────────┐
    │  WATCHFACE  │ ← Shows time, battery, steps, RPG level badge
    └──────┬──────┘
           │ Swipe LEFT
           ▼
    ┌─────────────┐
    │   STEPS     │ ← Walking gives XP, shows distance/calories
    └──────┬──────┘
           │ Swipe LEFT
           ▼
    ┌─────────────┐
    │  APP GRID   │ ← 3 pages of apps (swipe up/down)
    └──────┬──────┘
           │ Tap QUESTS
           ▼
    ┌─────────────┐
    │   QUESTS    │ ← Complete quests → earn XP → level up!
    └──────┬──────┘
           │ Complete all 3
           ▼
    ┌─────────────┐
    │  LEVEL UP!  │ ← New title, stat boosts, unlock abilities
    └─────────────┘
```

This creates a gameplay loop that encourages:
- Daily use (quests reset daily)
- Physical activity (steps = XP)
- Feature exploration (training, boss rush = XP)
- Long-term engagement (level 1-100 progression)
