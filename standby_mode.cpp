/*
 * standby_mode.cpp - 4-Level Power Standby System (Implementation)
 * DualCore_Watch / Master_ESP32_Watch_206 (ESP32-S3-Touch-AMOLED-2.06")
 *
 * State machine:
 *   ACTIVE ─(no touch 15s)→ IDLE ─(45s)→ AOD ─(3min)→ DEEP
 *     ↑_________________________________________________|
 *              (any wake source → ACTIVE)
 *
 * Two flavours selectable at runtime via standbySetAggressive(true):
 *   - balanced  (default): AOD uses light-sleep, DEEP uses deep-sleep
 *   - aggressive         : skip AOD path, go DEEP quickly (max battery)
 */

#include "standby_mode.h"
#include "power_manager.h"
#include "display.h"     // Arduino_CO5300 *gfx
#include "aod_watchface.h"

#include <esp_sleep.h>
#include <esp_pm.h>
#include <driver/rtc_io.h>

extern Arduino_CO5300 *gfx;
extern SystemState     system_state;
extern bool            screenOn;

// =============================================================================
// RTC-persisted data (survives deep sleep, lost on full reset)
// =============================================================================
RTC_DATA_ATTR uint32_t standby_rtc_steps       = 0;
RTC_DATA_ATTR uint32_t standby_rtc_boot_count  = 0;
RTC_DATA_ATTR uint8_t  standby_rtc_last_state  = STANDBY_OFF;

// --- Self-learning runtime estimator (RTC-only, zero NVS) -------------------
// 3 floats (12B) + 1 uint32 (4B) + 1 int8 + 1 uint8 = 18 bytes of RTC RAM.
// Zero NVS writes. Zero flash wear. One millis() check per standbyTick().
RTC_DATA_ATTR float    rtc_ema_drain_balanced_pph = 0.0f; // 0 = not learned
RTC_DATA_ATTR float    rtc_ema_drain_active_pph   = 0.0f;
RTC_DATA_ATTR uint32_t rtc_learn_last_ms          = 0;
RTC_DATA_ATTR int8_t   rtc_learn_last_pct         = -1;   // -1 = no anchor
RTC_DATA_ATTR uint8_t  rtc_learn_last_bucket      = 0xFF; // 0=active, 1=balanced

// Deep-sleep auto-calibration (snapshot before sleep, compare on wake).
// Lets STANDBY_MAXBATT_SCALE self-tune from real measurements instead of the
// hard-coded default. +12 bytes RTC RAM, still zero NVS.
RTC_DATA_ATTR float    rtc_ema_drain_deep_pph     = 0.0f; // 0 = not learned
RTC_DATA_ATTR int8_t   rtc_deep_snap_pct          = -1;   // -1 = no snapshot
RTC_DATA_ATTR int16_t  rtc_deep_snap_minute       = -1;   // minute-of-day at sleep

// =============================================================================
// Global singleton
// =============================================================================
StandbyInfo standby = {
    .state                = STANDBY_OFF,
    .last_wake            = WAKE_NONE,
    .last_interaction_ms  = 0,
    .entered_state_ms     = 0,
    .enabled              = true,
    .aggressive_mode      = false
};

// Cached user brightness (so we can restore on wake without "Settings only" rule)
static uint8_t s_user_brightness_cache = 200;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static inline uint8_t scaleBrightness(uint8_t base, uint8_t pct) {
    uint32_t v = ((uint32_t)base * pct) / 100u;
    if (v < 2)   v = 2;     // never fully off on IDLE/AOD — it's AOD
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static void setPanelBrightness(uint8_t b) {
    if (!gfx) return;
    gfx->setBrightness(b);
}

// -----------------------------------------------------------------------------
// Self-learning estimator (lightweight, RTC-only)
// Sampled every STANDBY_LEARN_INTERVAL_MS (5 min). Skips while charging.
// -----------------------------------------------------------------------------
static uint8_t currentLearnBucket() {
    // ACTIVE state → active bucket. IDLE/AOD → balanced bucket. DEEP is
    // unreachable here (CPU is asleep). Anything else defaults to balanced.
    return (standby.state == STANDBY_OFF) ? 0 : 1;
}

static void standbyLearningSample() {
    // Don't learn while charging (pct goes UP, poisons EMA) → reset anchor.
    if (system_state.is_charging) {
        rtc_learn_last_pct = -1;
        return;
    }

    uint32_t now = millis();

    // First sample (or post-charge re-anchor): just record and return.
    if (rtc_learn_last_pct < 0) {
        rtc_learn_last_ms     = now;
        rtc_learn_last_pct    = (int8_t)system_state.battery_percentage;
        rtc_learn_last_bucket = currentLearnBucket();
        return;
    }

    // Only sample every N ms — makes this effectively free in the hot loop.
    if ((uint32_t)(now - rtc_learn_last_ms) < STANDBY_LEARN_INTERVAL_MS) return;

    int      cur_pct = system_state.battery_percentage;
    int      dpct    = (int)rtc_learn_last_pct - cur_pct;  // positive = drain
    uint32_t dt_ms   = now - rtc_learn_last_ms;
    float    hours   = (float)dt_ms / 3600000.0f;

    uint8_t cur_bucket = currentLearnBucket();

    // Only learn when: real drain observed, bucket didn't change mid-interval,
    // and the reading is physically plausible.
    if (dpct > 0 && hours > 0.01f && cur_bucket == rtc_learn_last_bucket) {
        float rate = (float)dpct / hours;    // %/hr
        if (rate < 0.1f)  rate = 0.1f;       // clamp — one bad ADC read can't
        if (rate > 50.0f) rate = 50.0f;      // wreck the EMA

        float* target = (cur_bucket == 0) ? &rtc_ema_drain_active_pph
                                          : &rtc_ema_drain_balanced_pph;

        if (*target <= 0.01f) {
            *target = rate;                  // first learned sample initializes
        } else {
            *target = (*target) * (1.0f - STANDBY_LEARN_ALPHA)
                    + rate * STANDBY_LEARN_ALPHA;
        }
    }

    // Re-anchor for next interval
    rtc_learn_last_ms     = now;
    rtc_learn_last_pct    = (int8_t)cur_pct;
    rtc_learn_last_bucket = cur_bucket;
}

float standbyGetLearnedBalancedRate() { return rtc_ema_drain_balanced_pph; }
float standbyGetLearnedActiveRate()   { return rtc_ema_drain_active_pph;   }
float standbyGetLearnedDeepRate()     { return rtc_ema_drain_deep_pph;     }

// Auto-calibrated deep-sleep efficiency multiplier.
// Returns balanced_rate / deep_rate when both learned, clamped to a sane band
// so a single noisy reading can't break the estimate.
float standbyGetLearnedScale() {
    if (rtc_ema_drain_balanced_pph > 0.01f && rtc_ema_drain_deep_pph > 0.01f) {
        float s = rtc_ema_drain_balanced_pph / rtc_ema_drain_deep_pph;
        if (s < 1.5f)  s = 1.5f;
        if (s > 30.0f) s = 30.0f;
        return s;
    }
    return STANDBY_MAXBATT_SCALE;  // fall back to the compile-time default
}

bool standbyEstimatorIsLearned() {
    return (rtc_ema_drain_balanced_pph > 0.01f)
        || (rtc_ema_drain_active_pph   > 0.01f)
        || (rtc_ema_drain_deep_pph     > 0.01f);
}

// -----------------------------------------------------------------------------
// Weak default hooks (override in your app if you want custom behaviour)
// -----------------------------------------------------------------------------
__attribute__((weak)) void standbyOnEnterIdle()  {}
__attribute__((weak)) void standbyOnEnterAOD()   {}
__attribute__((weak)) void standbyOnEnterDeep()  {}
__attribute__((weak)) void standbyOnExitToActive(StandbyWakeReason) {}

// -----------------------------------------------------------------------------
// State transitions
// -----------------------------------------------------------------------------
static void enterActive(StandbyWakeReason reason) {
    if (standby.state == STANDBY_OFF) return;

    Serial.printf("[STANDBY] %s -> ACTIVE (wake=%d)\n",
                  standbyGetStateName(standby.state), reason);

    // Restore display
    if (!screenOn) {
        // Deep-sleep path already rebooted us; display gets re-initialized
        // by the normal setup() flow. For light-sleep path just turn on.
        if (gfx) {
            gfx->displayOn();
        }
        screenOn = true;
    }
    setPanelBrightness(s_user_brightness_cache);

    standby.state               = STANDBY_OFF;
    standby.last_wake           = reason;
    standby.last_interaction_ms = millis();
    standby.entered_state_ms    = millis();

    // Kick power_manager back to ACTIVE + animation burst for premium feel
    forceActiveState();
    triggerAnimationBurst(350);

    standbyOnExitToActive(reason);
}

static void enterIdle() {
    Serial.println("[STANDBY] ACTIVE -> IDLE (dim)");
    standby.state            = STANDBY_IDLE;
    standby.entered_state_ms = millis();

    s_user_brightness_cache = system_state.brightness;
    setPanelBrightness(scaleBrightness(s_user_brightness_cache,
                                       STANDBY_IDLE_BRIGHTNESS_PCT));

    // Lower FPS + CPU via power_manager
    setCpuFrequencyMhz(CPU_FREQ_IDLE);

    standbyOnEnterIdle();
}

static void enterAOD() {
    Serial.println("[STANDBY] IDLE -> AOD");
    standby.state            = STANDBY_AOD;
    standby.entered_state_ms = millis();

    // Extremely dim, black background, theme-aware minimal glow
    setPanelBrightness(scaleBrightness(s_user_brightness_cache,
                                       STANDBY_AOD_BRIGHTNESS_PCT));

    setCpuFrequencyMhz(CPU_FREQ_MINIMAL);  // 40 MHz

    aodRender(/*full_redraw=*/true);        // first paint

    standbyOnEnterAOD();
}

static void enterDeep() {
    Serial.println("[STANDBY] -> DEEP SLEEP (screen off)");
    standby.state            = STANDBY_DEEP;
    standby.entered_state_ms = millis();
    standby_rtc_last_state   = STANDBY_DEEP;

    // Auto-calibration snapshot: pct + minute-of-day, only if not charging.
    // On wake we'll diff vs current readings to learn deep-sleep drain rate.
    if (!system_state.is_charging) {
        WatchTime t = getCurrentTime();
        rtc_deep_snap_pct    = (int8_t)system_state.battery_percentage;
        rtc_deep_snap_minute = (int16_t)(t.hour * 60 + t.minute);
    } else {
        rtc_deep_snap_pct    = -1;
    }

    standbyOnEnterDeep();

    // Screen OFF
    if (gfx) {
        gfx->displayOff();
    }
    screenOn = false;

    // Configure wake sources for deep sleep. The FT3168 and BOOT_BUTTON are
    // active-low -> wake on level 0.
    //
    // We use EXT1 with ANY_LOW so multiple GPIOs can wake us.
    uint64_t wake_mask = 0;
    wake_mask |= (1ULL << STANDBY_WAKE_TOUCH_PIN);
    wake_mask |= (1ULL << STANDBY_WAKE_BUTTON_PIN);
#ifdef STANDBY_IMU_INT_PIN
    wake_mask |= (1ULL << STANDBY_IMU_INT_PIN);
#endif
    esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW);

    // Also a 60s safety timer in case a driver didn't re-arm its INT
    esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL);

    Serial.flush();
    esp_deep_sleep_start();
    // No return — chip resets on wake and re-enters setup().
}

// -----------------------------------------------------------------------------
// Light-sleep tick used during AOD. Wakes on touch / button / IMU / timer.
// Returns the wake reason.
// -----------------------------------------------------------------------------
static StandbyWakeReason aodLightSleep(uint32_t duration_ms) {
    // Arm GPIO wakes (active-low)
    gpio_wakeup_enable((gpio_num_t)STANDBY_WAKE_TOUCH_PIN,  GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)STANDBY_WAKE_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
#ifdef STANDBY_IMU_INT_PIN
    gpio_wakeup_enable((gpio_num_t)STANDBY_IMU_INT_PIN,     GPIO_INTR_LOW_LEVEL);
#endif
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup((uint64_t)duration_ms * 1000ULL);

    esp_light_sleep_start();   // ~1-5 ms to wake, RAM preserved

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_GPIO: {
            // Figure out which line
            if (digitalRead(STANDBY_WAKE_TOUCH_PIN)  == LOW) return WAKE_TOUCH;
            if (digitalRead(STANDBY_WAKE_BUTTON_PIN) == LOW) return WAKE_BUTTON;
#ifdef STANDBY_IMU_INT_PIN
            if (digitalRead(STANDBY_IMU_INT_PIN)     == LOW) return WAKE_MOTION;
#endif
            return WAKE_OTHER;
        }
        case ESP_SLEEP_WAKEUP_TIMER: return WAKE_TIMER;
        default:                     return WAKE_OTHER;
    }
}

// =============================================================================
// PUBLIC API
// =============================================================================
void standbyInit() {
    standby_rtc_boot_count++;

    standby.state               = STANDBY_OFF;
    standby.last_interaction_ms = millis();
    standby.entered_state_ms    = millis();
    s_user_brightness_cache     = system_state.brightness;

    // Configure wake pins as inputs with pull-ups (active-low devices)
    pinMode(STANDBY_WAKE_TOUCH_PIN,  INPUT_PULLUP);
    pinMode(STANDBY_WAKE_BUTTON_PIN, INPUT_PULLUP);
#ifdef STANDBY_IMU_INT_PIN
    pinMode(STANDBY_IMU_INT_PIN,     INPUT_PULLUP);
#endif

    // If we just woke from deep sleep, record why
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT1:
        case ESP_SLEEP_WAKEUP_EXT0:
        case ESP_SLEEP_WAKEUP_GPIO:
            standby.last_wake = WAKE_TOUCH;  // best-guess; refined below
            if (digitalRead(STANDBY_WAKE_BUTTON_PIN) == LOW) standby.last_wake = WAKE_BUTTON;
#ifdef STANDBY_IMU_INT_PIN
            if (digitalRead(STANDBY_IMU_INT_PIN)     == LOW) standby.last_wake = WAKE_MOTION;
#endif
            Serial.printf("[STANDBY] Woke from DEEP sleep (reason=%d)\n", standby.last_wake);
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            standby.last_wake = WAKE_TIMER;
            Serial.println("[STANDBY] Woke from DEEP sleep (timer)");
            break;
        default:
            standby.last_wake = WAKE_NONE;
            Serial.println("[STANDBY] Cold boot");
            break;
    }

    Serial.printf("[STANDBY] init ok. boot_count=%lu rtc_steps=%lu\n",
                  (unsigned long)standby_rtc_boot_count,
                  (unsigned long)standby_rtc_steps);

    // -------------------------------------------------------------------------
    // Auto-calibrate deep-sleep drain (Max-Battery profile).
    // If we have a valid pre-sleep snapshot AND we just woke from deep sleep,
    // compare battery + minute-of-day delta and update the EMA.
    // -------------------------------------------------------------------------
    bool woke_from_deep = (cause == ESP_SLEEP_WAKEUP_EXT1
                        || cause == ESP_SLEEP_WAKEUP_EXT0
                        || cause == ESP_SLEEP_WAKEUP_GPIO
                        || cause == ESP_SLEEP_WAKEUP_TIMER);
    if (woke_from_deep && rtc_deep_snap_pct >= 0
        && !system_state.is_charging) {
        WatchTime t  = getCurrentTime();
        int now_min  = t.hour * 60 + t.minute;
        int dmin     = now_min - rtc_deep_snap_minute;
        if (dmin < 0) dmin += 24 * 60;          // midnight wrap
        int dpct     = (int)rtc_deep_snap_pct - system_state.battery_percentage;

        // Only learn when: meaningful elapsed time, real drain, sane bounds.
        if (dmin >= 5 && dpct > 0 && dmin <= 24 * 60) {
            float hours = (float)dmin / 60.0f;
            float rate  = (float)dpct / hours;   // %/hr
            if (rate < 0.05f) rate = 0.05f;
            if (rate > 20.0f) rate = 20.0f;      // deep sleep can't drain faster

            if (rtc_ema_drain_deep_pph <= 0.01f) {
                rtc_ema_drain_deep_pph = rate;
            } else {
                rtc_ema_drain_deep_pph =
                    rtc_ema_drain_deep_pph * (1.0f - STANDBY_LEARN_ALPHA)
                    + rate * STANDBY_LEARN_ALPHA;
            }
            Serial.printf("[STANDBY] deep-sleep auto-calibrated: %.2f %%/hr "
                          "(%.2fh window, %d%% drop)\n",
                          rtc_ema_drain_deep_pph, hours, dpct);
        }
    }
  rtc_deep_snap_pct = -1;   // consume snapshot
}

void standbyTick() {
    if (!standby.enabled) return;

    // Cheap 5-min sampler for the self-learning runtime estimator.
    standbyLearningSample();

    const uint32_t now  = millis();
    const uint32_t idle = now - standby.last_interaction_ms;

    switch (standby.state) {

        case STANDBY_OFF: {
            if (idle >= STANDBY_IDLE_AFTER_MS) {
                if (standby.aggressive_mode) {
                    // Max-battery path: skip IDLE/AOD and dive
                    enterDeep();   // no return
                } else {
                    enterIdle();
                }
            }
            break;
        }

        case STANDBY_IDLE: {
            if (idle >= STANDBY_AOD_AFTER_MS) enterAOD();
            break;
        }

        case STANDBY_AOD: {
            if (idle >= STANDBY_DEEP_AFTER_MS) {
                enterDeep();       // no return
                return;
            }

            // Light-sleep until the next minute boundary OR a wake event.
            // This is where the real battery saving during AOD happens.
            StandbyWakeReason r = aodLightSleep(STANDBY_AOD_TICK_MS);
            if (r == WAKE_TOUCH || r == WAKE_BUTTON || r == WAKE_MOTION) {
                enterActive(r);
            } else {
                // Just tick refresh
                aodRender(/*full_redraw=*/false);
            }
            break;
        }

        case STANDBY_DEEP:
            // We never actually loop in this state — esp_deep_sleep_start()
            // never returns. Reaching here is a bug.
            break;
    }
}

void standbyRecordInteraction() {
    standby.last_interaction_ms = millis();
    if (standby.state != STANDBY_OFF) {
        enterActive(WAKE_TOUCH);
    }
}

void standbyEnable(bool on) {
    standby.enabled = on;
    if (!on && standby.state != STANDBY_OFF) {
        enterActive(WAKE_OTHER);
    }
    Serial.printf("[STANDBY] enabled=%d\n", on);
}

bool standbyIsEnabled()               { return standby.enabled; }
void standbySetAggressive(bool on)    { standby.aggressive_mode = on; }
void standbyEnterDeepSleepNow()       { enterDeep(); }
StandbyState      standbyGetState()   { return standby.state; }
StandbyWakeReason standbyGetLastWake(){ return standby.last_wake; }

const char* standbyGetStateName(StandbyState s) {
    switch (s) {
        case STANDBY_OFF:  return "ACTIVE";
        case STANDBY_IDLE: return "IDLE";
        case STANDBY_AOD:  return "AOD";
        case STANDBY_DEEP: return "DEEP";
        default:           return "?";
    }
}