/*
 * standby_mode.h - 4-Level Power Standby System + Self-Learning Estimator
 * DualCore_Watch / Master_ESP32_Watch_206 (ESP32-S3-Touch-AMOLED-2.06")
 *
 * Layered "mostly-off brain-awake" architecture:
 *   ACTIVE (full UI) -> IDLE (dim + low FPS) -> AOD -> DEEP_STANDBY (sleep)
 *
 * Now also includes a self-learning runtime estimator. All learning state
 * lives in RTC_DATA_ATTR (~18 bytes). Zero NVS writes, zero flash wear.
 */

#ifndef STANDBY_MODE_H
#define STANDBY_MODE_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// STANDBY STATES
// =============================================================================
enum StandbyState {
    STANDBY_OFF = 0,     // ACTIVE - normal operation
    STANDBY_IDLE,        // UI fade: dim + reduced FPS
    STANDBY_AOD,         // Always-On Display
    STANDBY_DEEP         // Screen OFF, CPU deep-sleep
};

// =============================================================================
// TIMING (ms)
// =============================================================================
#ifndef STANDBY_IDLE_AFTER_MS
#define STANDBY_IDLE_AFTER_MS       15000UL
#endif
#ifndef STANDBY_AOD_AFTER_MS
#define STANDBY_AOD_AFTER_MS        45000UL
#endif
#ifndef STANDBY_DEEP_AFTER_MS
#define STANDBY_DEEP_AFTER_MS       180000UL
#endif
#ifndef STANDBY_AOD_TICK_MS
#define STANDBY_AOD_TICK_MS         30000UL
#endif
#ifndef STANDBY_IDLE_BRIGHTNESS_PCT
#define STANDBY_IDLE_BRIGHTNESS_PCT 50
#endif
#ifndef STANDBY_AOD_BRIGHTNESS_PCT
#define STANDBY_AOD_BRIGHTNESS_PCT  15
#endif

// Self-learning estimator
#ifndef STANDBY_LEARN_INTERVAL_MS
#define STANDBY_LEARN_INTERVAL_MS   (5UL * 60UL * 1000UL)  // 5 min
#endif
#ifndef STANDBY_LEARN_ALPHA
#define STANDBY_LEARN_ALPHA         0.25f                  // EMA weight
#endif
#ifndef STANDBY_MAXBATT_SCALE
#define STANDBY_MAXBATT_SCALE       3.0f                   // deep-sleep ~3x longer
#endif

// Wake sources
#ifndef STANDBY_WAKE_TOUCH_PIN
#define STANDBY_WAKE_TOUCH_PIN      TP_INT
#endif
#ifndef STANDBY_WAKE_BUTTON_PIN
#define STANDBY_WAKE_BUTTON_PIN     BOOT_BUTTON
#endif
// Optional: #define STANDBY_IMU_INT_PIN to enable motion wake.

// =============================================================================
// WAKE REASON
// =============================================================================
enum StandbyWakeReason {
    WAKE_NONE = 0, WAKE_TOUCH, WAKE_BUTTON, WAKE_MOTION, WAKE_TIMER, WAKE_OTHER
};

// =============================================================================
// PUBLIC STATE
// =============================================================================
struct StandbyInfo {
    StandbyState       state;
    StandbyWakeReason  last_wake;
    uint32_t           last_interaction_ms;
    uint32_t           entered_state_ms;
    bool               enabled;
    bool               aggressive_mode;
};

extern StandbyInfo standby;

// RTC-persisted (survives deep sleep, lost on cold boot)
extern RTC_DATA_ATTR uint32_t standby_rtc_steps;
extern RTC_DATA_ATTR uint32_t standby_rtc_boot_count;
extern RTC_DATA_ATTR uint8_t  standby_rtc_last_state;

// =============================================================================
// PUBLIC API
// =============================================================================
void standbyInit();
void standbyTick();
void standbyRecordInteraction();
void standbyEnable(bool on);
bool standbyIsEnabled();
void standbySetAggressive(bool on);
void standbyEnterDeepSleepNow();
StandbyState      standbyGetState();
const char*       standbyGetStateName(StandbyState s);
StandbyWakeReason standbyGetLastWake();

// --- Self-learning runtime estimator ----------------------------------------
// Returns measured %/hour drain for each bucket. 0.0f = not learned yet.
// Total RTC memory cost: ~30 bytes. Zero NVS writes.
float standbyGetLearnedBalancedRate();  // %/hr, or 0 if unlearned
float standbyGetLearnedActiveRate();    // %/hr, or 0 if unlearned
float standbyGetLearnedDeepRate();      // %/hr in deep sleep, or 0 if unlearned
float standbyGetLearnedScale();         // balanced / deep ratio (auto-calibrated)
bool  standbyEstimatorIsLearned();      // true once at least one bucket learned

// Weak hooks
void standbyOnEnterIdle();
void standbyOnEnterAOD();
void standbyOnEnterDeep();
void standbyOnExitToActive(StandbyWakeReason reason);

#endif // STANDBY_MODE_H