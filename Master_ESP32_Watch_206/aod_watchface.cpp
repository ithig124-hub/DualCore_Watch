/*
 * aod_watchface.cpp - AOD renderer (uses the active character watchface)
 *
 * Strategy:
 *   - Turn off animations through power_manager so the watchface draws a
 *     single static frame (no auras pulsing, no particles, etc.).
 *   - Call drawWatchFace() — the same per-theme dispatcher the rest of the
 *     OS uses, so the look per character is IDENTICAL to active mode, just dim.
 *   - Only redraw when the minute changes (or the caller requests full redraw).
 */

#include "aod_watchface.h"
#include "config.h"
#include "power_manager.h"
#include "themes.h"          // drawWatchFace()

extern SystemState system_state;

// Cache so we only redraw on minute boundary / theme change
static int       s_last_minute = -1;
static int       s_last_hour   = -1;
static ThemeType s_last_theme  = (ThemeType)-1;

void aodInvalidate() {
    s_last_minute = -1;
    s_last_hour   = -1;
    s_last_theme  = (ThemeType)-1;
}

void aodRender(bool full_redraw) {
    WatchTime t = getCurrentTime();

    bool theme_changed = (system_state.current_theme != s_last_theme);
    bool time_changed  = (t.minute != s_last_minute) || (t.hour != s_last_hour);

    if (!full_redraw && !time_changed && !theme_changed) return;

    // --- Save state we're about to override -----------------------------------
    bool saved_animations = power_manager.animations_active;

    // --- Force the watchface to draw as a still frame -------------------------
    power_manager.animations_active = false;  // single static frame, no FX

    drawWatchFace();   // uses the user's selected character theme

    // --- Restore state --------------------------------------------------------
    power_manager.animations_active = saved_animations;

    s_last_minute = t.minute;
    s_last_hour   = t.hour;
    s_last_theme  = system_state.current_theme;
}