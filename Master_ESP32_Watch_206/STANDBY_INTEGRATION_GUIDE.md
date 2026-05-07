# STANDBY MODE + BATTERY APP — INTEGRATION GUIDE
DualCore_Watch / Master_ESP32_Watch_206 (ESP32-S3-Touch-AMOLED-2.06")

## Files to drop in

```
Master_ESP32_Watch_206/
 ├── standby_mode.h
 ├── standby_mode.cpp
 ├── aod_watchface.h
 ├── aod_watchface.cpp      ← reuses your existing character watchface
 ├── battery_app.h
 ├── battery_app.cpp        ← now with runtime estimator
 └── STANDBY_INTEGRATION_GUIDE.md
```

---

## 1. State machine

```
ACTIVE  ─(15s no touch)→  IDLE  ─(45s)→  AOD  ─(3min)→  DEEP-SLEEP
  ↑_________________________________________________________|
           (touch / button / motion → ACTIVE)
```

- **ACTIVE** – your normal UI
- **IDLE** – dim to 50 %, lower FPS (UI still live → premium feel)
- **AOD** – your **actual character watchface**, drawn once per minute, animations off, ~15 % brightness, light-sleep between ticks (instant wake)
- **DEEP** – display off, `esp_deep_sleep_start()`; wake on touch / BOOT / motion

**Two profiles** switched from the Battery app:

| Profile                 | Behaviour                                       |
|-------------------------|-------------------------------------------------|
| Balanced (default)      | IDLE → AOD → DEEP. Premium instant-wake feel.   |
| Max Battery (aggressive)| IDLE → **DEEP directly**. Skips AOD for max mAh.|

---

## 2. `Master_ESP32_Watch_206.ino`

### 2a. Includes (top)
```cpp
#include "standby_mode.h"
#include "battery_app.h"
```

### 2b. `setup()` — after display / touch / `initPowerManager()`
```cpp
initPowerManager();
// ... display / touch / lvgl init ...
standbyInit();

// Restore step count across deep sleep
system_state.steps_today = (int)standby_rtc_steps;
```

### 2c. `loop()` — at the end
```cpp
void loop() {
    // ... your existing UI + touch + LVGL work ...
    standbyTick();              // drives state machine; may light-sleep
    delay(getPowerLoopDelay());
}
```

### 2d. Every real user input (in `touch.cpp` / `navigation.cpp`)
```cpp
standbyRecordInteraction();
```

---

## 3. Hook the Battery app into the **main loop navigation**

The Battery app is **not** added to the app grid. It lives only in the infinite
main-screen loop, so the user reaches it by swiping through:

```
Watchface → Steps → AppGrid → Character Stats → Battery → Watchface → …
```

### 3a. `config.h` — add a ScreenType
```cpp
SCREEN_BATTERY,     // add anywhere in ScreenType enum
```

### 3b. `config.h` — extend the MainScreen enum + count
Replace the existing MainScreen block:
```cpp
enum MainScreen {
    MAIN_WATCHFACE = 0,
    MAIN_STEPS_TRACKER,
    MAIN_APP_GRID_1,
    MAIN_CHARACTER_STATS,
    MAIN_BATTERY            // ← NEW
};

#define MAIN_SCREEN_COUNT 5   // ← bumped from 4
```

### 3c. `navigation.cpp` — in the main-loop → ScreenType mapper
Find the switch/if chain that translates `navState.currentMain` into a
`ScreenType` (the one that already handles `MAIN_WATCHFACE`,
`MAIN_STEPS_TRACKER`, `MAIN_APP_GRID_1`, `MAIN_CHARACTER_STATS`) and add:
```cpp
case MAIN_BATTERY:
    setCurrentScreen(SCREEN_BATTERY);
    break;
```

### 3d. `ui.cpp` — in `drawCurrentScreen()`
```cpp
case SCREEN_BATTERY:
    batteryAppDraw();
    break;
```

### 3e. Touch dispatcher (wherever per-screen touch is routed)
```cpp
case SCREEN_BATTERY:
    batteryAppHandleTouch(gesture);
    break;
```

That’s it — swipe from Character Stats one more time and you land on the
Battery app. One more swipe loops back to the Watchface.

---

## 4. Battery app features

- Battery gauge with color-coded % (green / yellow / orange / red)
- Charging indicator when plugged in
- **Runtime estimator card**:
  ```
  ~9h Balanced      ~26h Max Batt
  Full-on screen time: ~2.4h
  ```
  Active profile is highlighted green, inactive in grey. Numbers scale with
  current charge %.
- **Standby mode** row → tap to toggle ON/OFF
- **Profile** row → tap to switch **Balanced ↔ Max Battery** (estimator
  highlight + behaviour follow instantly)
- **Sleep now** button → immediate deep sleep
- Footer: `State: AOD   Last wake: touch`

Tune the runtime constants to your measured numbers (define BEFORE including
`battery_app.h`):
```cpp
#define BATTERY_HOURS_BALANCED_AT_100   10.0f
#define BATTERY_HOURS_MAXBATT_AT_100    30.0f
#define BATTERY_HOURS_ACTIVE_AT_100      3.0f
```

### Self-learning runtime predictor

After ~20 minutes of real use (off charger), the estimate becomes **measured**
instead of guessed. The card shows a green **"LEARNED"** badge; before that
it shows a yellow **"learning..."** badge and uses the compile-time constants.

How it works:
- A 5-minute sampler inside `standbyTick()` watches battery % drop.
- Two EMAs (α = 0.25) track drain rate per bucket:
  - `active`   — while ACTIVE state (user interacting)
  - `balanced` — while IDLE / AOD
- Max-battery estimate = balanced / `STANDBY_MAXBATT_SCALE` (default 3.0).
- All state lives in `RTC_DATA_ATTR` (~18 bytes), survives deep sleep.
- **Zero NVS writes. Zero flash wear. No measurable battery cost.**
- Charging time is automatically excluded (pct rising = anchor reset).
- Each learned rate is clamped to 0.1–50 %/hr so one bad ADC read can't
  wreck the EMA.

Override if you want (before `#include "standby_mode.h"`):
```cpp
#define STANDBY_LEARN_INTERVAL_MS   (5UL * 60UL * 1000UL)  // 5 min
#define STANDBY_LEARN_ALPHA         0.25f                  // EMA weight
#define STANDBY_MAXBATT_SCALE       3.0f                   // deep-sleep ratio
```

---

## 5. AOD = your real character watchface

`aod_watchface.cpp` does **not** render anything custom. It temporarily:
- forces `system_state.current_screen = SCREEN_WATCHFACE`
- sets `power_manager.animations_active = false`
- calls your existing `drawCurrentScreen()`
- restores state

So in AOD you see your Gojo / Luffy / Jinwoo / BoBoiBoy watchface, just dim
and frozen (no CPU burn on animation, minute-boundary refresh).

Brightness handled in `standby_mode.cpp`:
- IDLE → 50 % of user-set
- AOD  → 15 % of user-set
- Wake → restored to user-set

---

## 6. Wake sources (already wired for 2.06″ board)

| Source               | Pin                   |
|----------------------|-----------------------|
| FT3168 touch INT     | `TP_INT` (GPIO 38)    |
| BOOT button          | `BOOT_BUTTON` (GPIO 0)|
| QMI8658 motion (opt.)| `STANDBY_IMU_INT_PIN` (wire + `#define`) |

Active-low → EXT1 `ANY_LOW` for deep sleep, `GPIO_INTR_LOW_LEVEL` for AOD light sleep.

---

## 7. Step count across deep sleep

`standby_mode.cpp` exposes:
```cpp
extern RTC_DATA_ATTR uint32_t standby_rtc_steps;
extern RTC_DATA_ATTR uint32_t standby_rtc_boot_count;
```
In `steps_tracker.cpp`, whenever you update `system_state.steps_today`:
```cpp
standby_rtc_steps = (uint32_t)system_state.steps_today;
```
Restore in `setup()` after `standbyInit()` (see 2b).

---

## 8. Timing knobs (before `#include "standby_mode.h"`)
```cpp
#define STANDBY_IDLE_AFTER_MS         10000UL
#define STANDBY_AOD_AFTER_MS          30000UL
#define STANDBY_DEEP_AFTER_MS         120000UL
#define STANDBY_AOD_TICK_MS           60000UL
#define STANDBY_IDLE_BRIGHTNESS_PCT   50
#define STANDBY_AOD_BRIGHTNESS_PCT    15
```

---

## 9. Weak hooks
```cpp
void standbyOnEnterAOD()  { /* e.g. 300ms breathing fade */ }
void standbyOnEnterDeep() { /* save NVS/SD */ }
void standbyOnExitToActive(StandbyWakeReason r) {
    /* 300ms character-themed wake animation */
}
```

---

## 10. Push to GitHub
I can’t push for you. Use **"Save to GitHub"** in the Emergent chat to commit these 7 files into
`ithig124-hub/DualCore_Watch/Master_ESP32_Watch_206/`, or copy them manually from `/app/Master_ESP32_Watch_206/`.
