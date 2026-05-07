/*
 * battery_app.h - "Battery" app
 * DualCore_Watch / Master_ESP32_Watch_206
 *
 * Reachable ONLY via the main-screen loop:
 *   Watchface -> Steps -> AppGrid -> Character Stats -> Battery -> Watchface
 *
 * Shows battery state, runtime estimate for both power profiles, and lets
 * the user toggle standby + switch profile + sleep immediately.
 *
 * Integration (minimal):
 *   1. Add SCREEN_BATTERY to ScreenType enum in config.h.
 *   2. Add MAIN_BATTERY to MainScreen enum and bump MAIN_SCREEN_COUNT.
 *   3. In drawCurrentScreen(): case SCREEN_BATTERY: batteryAppDraw();
 *   4. In the main-loop navigation mapper: MAIN_BATTERY -> SCREEN_BATTERY.
 *   5. In touch dispatcher for SCREEN_BATTERY: batteryAppHandleTouch(g).
 *
 * See STANDBY_INTEGRATION_GUIDE.md for exact snippets.
 */

#ifndef BATTERY_APP_H
#define BATTERY_APP_H

#include "config.h"   // TouchGesture

// =============================================================================
// Runtime-estimate constants (tune to your measured values)
// Expressed as projected hours of runtime at 100% charge for each profile.
// Override BEFORE #include if you measure the real numbers on your hardware.
// =============================================================================
#ifndef BATTERY_HOURS_BALANCED_AT_100
#define BATTERY_HOURS_BALANCED_AT_100     10.0f   // mostly AOD + light sleep
#endif
#ifndef BATTERY_HOURS_MAXBATT_AT_100
#define BATTERY_HOURS_MAXBATT_AT_100      30.0f   // mostly deep sleep
#endif
#ifndef BATTERY_HOURS_ACTIVE_AT_100
#define BATTERY_HOURS_ACTIVE_AT_100        3.0f   // screen fully on, animated
#endif

void batteryAppDraw();
void batteryAppHandleTouch(TouchGesture& g);

#endif // BATTERY_APP_H