/*
 * aod_watchface.h - AOD renderer
 * DualCore_Watch / Master_ESP32_Watch_206
 *
 * Reuses the currently-selected character watchface (drawn via the project's
 * existing drawCurrentScreen()). In AOD we just:
 *   - force SCREEN_WATCHFACE
 *   - kill animations
 *   - drop brightness
 *   - redraw only on minute change (or first entry / theme change)
 *
 * No custom minimal render any more — your anime watchface IS the AOD.
 */

#ifndef AOD_WATCHFACE_H
#define AOD_WATCHFACE_H

#include <Arduino.h>

// Paint the AOD. full_redraw=true on entry to AOD so the frame is clean.
void aodRender(bool full_redraw);

// Drop the internal cache (call if the user changed theme while in AOD).
void aodInvalidate();

#endif // AOD_WATCHFACE_H