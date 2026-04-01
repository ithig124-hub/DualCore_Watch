# Quick Win Features - Integration Guide

## New Apps Created

### 1. ⏱️ Timer & Stopwatch (`timer_app.cpp/h`)
**Features:**
- Stopwatch with lap tracking (up to 10 laps)
- Countdown timer with progress ring
- Multiple timer support (placeholder)
- Large, easy-to-read time display

**Controls:**
- START/PAUSE button
- RESET button
- LAP button (when running)
- Swipe UP to exit

### 2. 🧭 Compass (`compass_app.cpp/h`)
**Features:**
- Real-time heading using QMI8658 magnetometer
- 360° compass rose with cardinal directions
- Animated needle pointing North
- Auto-calibration (rotate watch to calibrate)

**Controls:**
- Tap to recalibrate
- Swipe UP to exit

### 3. 🔦 Enhanced Flashlight (`flashlight_app.cpp/h`)
**Features:**
- **White Light** mode with adjustable brightness
- **SOS Mode** - International distress signal (... --- ...)
- **Strobe Mode** - Fast blinking for attention
- **Red Light** mode - Night vision preservation

**Controls:**
- Mode buttons at bottom
- Brightness slider (White/Red modes only)
- Swipe UP to exit

### 4. 📏 Unit Converter (`converter_app.cpp/h`)
**Features:**
- **Length**: m, km, cm, mm, mi, yd, ft, in
- **Weight**: kg, g, mg, lb, oz, ton
- **Temperature**: °C, °F, K
- **Speed**: m/s, km/h, mph, knot

**Controls:**
- Category tabs at top
- Tap FROM/TO units to cycle through options
- Quick value buttons: 1, 10, 100, +1
- Swipe UP to exit

---

## Integration Steps

### Step 1: Add includes to ESP32_Watch.ino

Add these lines after the existing includes (around line 40):

```cpp
#include "timer_app.h"
#include "compass_app.h"
#include "flashlight_app.h"
#include "converter_app.h"
```

### Step 2: Initialize apps in setup()

Add these lines in `setup()` function after `initWifiApps()` (around line 150):

```cpp
// Initialize Quick Win apps
initTimerApp();
initCompassApp();
initFlashlightApp();
initConverterApp();
```

### Step 3: Add screen types to config.h

In `config.h`, add these to the `ScreenType` enum (around line 350):

```cpp
SCREEN_TIMER,
SCREEN_COMPASS,
SCREEN_UNIT_CONVERTER,
```

### Step 4: Update navigation.cpp - openApp() function

In `/app/fix/navigation.cpp`, find the `openApp()` function (around line 504) and add these cases:

```cpp
else if (strcmp(appName, "TIMER") == 0) {
  system_state.current_screen = SCREEN_TIMER;
  drawTimerApp();
}
else if (strcmp(appName, "COMPASS") == 0) {
  system_state.current_screen = SCREEN_COMPASS;
  drawCompassApp();
}
else if (strcmp(appName, "TORCH") == 0) {
  system_state.current_screen = SCREEN_FLASHLIGHT;
  drawFlashlightApp();
}
else if (strcmp(appName, "CONVERT") == 0) {
  system_state.current_screen = SCREEN_UNIT_CONVERTER;
  drawConverterApp();
}
```

### Step 5: Update handleTouchGesture() in ESP32_Watch.ino

Add these cases to the `switch` statement in `handleTouchGesture()` (around line 280):

```cpp
case SCREEN_TIMER:
  handleTimerTouch(gesture);
  break;

case SCREEN_COMPASS:
  handleCompassTouch(gesture);
  break;

case SCREEN_FLASHLIGHT:
  handleFlashlightTouch(gesture);
  break;

case SCREEN_UNIT_CONVERTER:
  handleConverterTouch(gesture);
  break;
```

### Step 6: Update app grid in navigation.cpp

**Option A: Replace existing placeholder apps**

In `drawAppGrid2()` function (around line 399), replace the `apps2[]` array:

```cpp
const char* apps2[] = {"THEMES", "COLLECT", "FILES", "TIMER", "COMPASS", "CONVERT", "BACKUP", "FUSION", "ABOUT"};
```

**Option B: Add to existing grid**

Keep current apps and add new ones to a third page, or replace less-used apps like "OTA", "BACKUP" with Quick Win apps.

### Step 7: Update app colors in navigation.cpp

In `getAppColor()` function (around line 267), add:

```cpp
if (strcmp(appName, "TIMER") == 0)     return RGB565(100, 180, 255);
if (strcmp(appName, "COMPASS") == 0)   return RGB565(255, 100, 150);
if (strcmp(appName, "CONVERT") == 0)   return RGB565(180, 120, 255);
```

---

## Compilation

### Add to your Arduino sketch folder:
```
ESP32_Watch/
├── ESP32_Watch.ino
├── timer_app.h
├── timer_app.cpp
├── compass_app.h
├── compass_app.cpp
├── flashlight_app.h
├── flashlight_app.cpp
├── converter_app.h
├── converter_app.cpp
└── (all other existing files)
```

### Compile & Upload
1. Open `ESP32_Watch.ino` in Arduino IDE
2. Select board: ESP32S3 Dev Module
3. Upload to your watch

---

## Testing

### Timer:
1. Open app grid → Tap "TIMER"
2. Tap START to begin stopwatch
3. Tap LAP to record lap times
4. Switch to "Timer" tab
5. Tap START for 5-minute countdown
6. Swipe UP to exit

### Compass:
1. Open app grid → Tap "COMPASS"
2. Rotate watch horizontally to calibrate
3. Red arrow points to magnetic North
4. Heading in degrees shown at top
5. Swipe UP to exit

### Flashlight:
1. Open app grid → Tap "TORCH"
2. Tap mode buttons: White, SOS, Strobe, Red
3. Drag brightness slider (White/Red modes)
4. Watch SOS blink pattern: ... --- ...
5. Swipe UP to exit

### Converter:
1. Open app grid → Tap "CONVERT"
2. Tap category tabs: Length, Weight, Temp, Speed
3. Tap "FROM" unit to cycle through options
4. Tap "TO" unit to select target
5. Tap quick value buttons
6. Result updates automatically
7. Swipe UP to exit

---

## Notes

- **Compass**: Requires QMI8658 IMU with magnetometer. May need calibration on first use.
- **Timer**: Stopwatch continues running in background. Use RESET before exiting if you want to clear it.
- **Flashlight SOS**: Uses standard Morse code pattern (3 short, 3 long, 3 short).
- **Converter**: Temperature conversion uses proper formulas (not multiplicative).

---

## Optional Enhancements

### Add vibration to Timer countdown finish:
If your watch has a vibration motor, add this to `getCountdownRemaining()` in `timer_app.cpp`:

```cpp
if (remaining <= 0 && !countdown.alarm_triggered) {
  countdown.running = false;
  countdown.alarm_triggered = true;
  // Add vibration here
  // digitalWrite(VIBRATION_PIN, HIGH);
  // delay(500);
  // digitalWrite(VIBRATION_PIN, LOW);
  return 0;
}
```

### Compass magnetic declination adjustment:
Add your location's magnetic declination in `calculateHeading()`:

```cpp
// Adjust for magnetic declination
// Example: +14° for New York, -12° for Los Angeles
heading += MAGNETIC_DECLINATION;
```

Enjoy your Quick Win features! 🚀
