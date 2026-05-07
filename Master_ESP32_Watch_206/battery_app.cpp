/*
 * battery_app.cpp - "Battery" app implementation with runtime estimator
 *
 * Layout (410 x 502):
 *   Header bar
 *   Big battery gauge + %
 *   Runtime estimate card:  "~9h Balanced   ~26h Max Battery"
 *   Standby ON/OFF toggle row
 *   Profile row (tap: Balanced <-> Max Battery)
 *   Sleep-now button
 *   Status footer (state, last wake)
 */

#include "battery_app.h"
#include "standby_mode.h"
#include "display.h"   // Arduino_CO5300 *gfx

extern Arduino_CO5300 *gfx;
extern SystemState     system_state;

// Inline minimal header (avoids cross-module dependency on ui.h's variant)
static void drawHeader(const char* title) {
    gfx->fillRect(0, 0, LCD_WIDTH, 50, COLOR_BLACK);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(3);
    int tw = (int)strlen(title) * 6 * 3;
    gfx->setCursor((LCD_WIDTH - tw) / 2, 14);
    gfx->print(title);
    gfx->drawFastHLine(20, 48, LCD_WIDTH - 40, COLOR_GRAY);
}

// =============================================================================
// Layout
// =============================================================================
static const int SCR_W = LCD_WIDTH;

static const int BAT_X = 40;
static const int BAT_Y = 70;
static const int BAT_W = SCR_W - 80;
static const int BAT_H = 80;

static const int EST_Y = BAT_Y + BAT_H + 14;     // runtime estimate card
static const int EST_H = 56;

static const int ROW_H         = 58;
static const int ROW_PAD_X     = 30;
static const int ROW_STANDBY_Y = EST_Y + EST_H + 14;
static const int ROW_MODE_Y    = ROW_STANDBY_Y + ROW_H + 10;
static const int ROW_SLEEP_Y   = ROW_MODE_Y    + ROW_H + 10;
static const int ROW_INFO_Y    = ROW_SLEEP_Y   + ROW_H + 10;

// =============================================================================
// Helpers
// =============================================================================
static uint16_t batteryColor(int pct) {
    if (pct <= BATTERY_CRITICAL)      return COLOR_RED;
    if (pct <= BATTERY_LOW_THRESHOLD) return COLOR_ORANGE;
    if (pct >= 80)                    return COLOR_GREEN;
    return COLOR_YELLOW;
}

static void formatHours(float h, char* out, size_t outsz) {
    if (h < 1.0f) {
        int mins = (int)(h * 60.0f + 0.5f);
        if (mins < 1) mins = 1;
        snprintf(out, outsz, "~%dm", mins);
    } else if (h < 10.0f) {
        snprintf(out, outsz, "~%.1fh", h);
    } else {
        snprintf(out, outsz, "~%dh", (int)(h + 0.5f));
    }
}

static void drawRow(int y, const char* label, const char* value, uint16_t value_col) {
    gfx->drawRoundRect(ROW_PAD_X, y, SCR_W - 2 * ROW_PAD_X, ROW_H, 12, COLOR_GRAY);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(ROW_PAD_X + 16, y + (ROW_H / 2) - 8);
    gfx->print(label);

    int vw = (int)strlen(value) * 6 * 2;
    gfx->setTextColor(value_col);
    gfx->setCursor(SCR_W - ROW_PAD_X - 16 - vw, y + (ROW_H / 2) - 8);
    gfx->print(value);
}

static bool hit(TouchGesture& g, int x, int y, int w, int h) {
    return (g.event == TOUCH_TAP)
        && g.x >= x && g.x < x + w
        && g.y >= y && g.y < y + h;
}

// =============================================================================
// DRAW
// =============================================================================
void batteryAppDraw() {
    if (!gfx) return;

    gfx->fillScreen(COLOR_BLACK);

    drawHeader("Battery");

    // -------- Battery gauge --------------------------------------------------
    int pct = system_state.battery_percentage;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    uint16_t bc = batteryColor(pct);

    gfx->drawRoundRect(BAT_X,     BAT_Y,     BAT_W,     BAT_H,     14, COLOR_WHITE);
    gfx->drawRoundRect(BAT_X + 1, BAT_Y + 1, BAT_W - 2, BAT_H - 2, 13, COLOR_WHITE);
    gfx->fillRect(BAT_X + BAT_W, BAT_Y + BAT_H / 3, 10, BAT_H / 3, COLOR_WHITE);
    int fill_w = ((BAT_W - 10) * pct) / 100;
    if (fill_w > 0) {
        gfx->fillRoundRect(BAT_X + 5, BAT_Y + 5, fill_w, BAT_H - 10, 10, bc);
    }
    char pbuf[12];
    snprintf(pbuf, sizeof(pbuf), "%d%%", pct);
    gfx->setTextSize(4);
    gfx->setTextColor(COLOR_WHITE);
    int pw = (int)strlen(pbuf) * 6 * 4;
    gfx->setCursor(BAT_X + (BAT_W - pw) / 2, BAT_Y + (BAT_H - 32) / 2);
    gfx->print(pbuf);

    if (system_state.is_charging) {
        gfx->setTextSize(2);
        gfx->setTextColor(COLOR_GREEN);
        gfx->setCursor(BAT_X, BAT_Y + BAT_H + 2);
        gfx->print("Charging");
    }

    // -------- Runtime estimator card -----------------------------------------
    // Use learned %-per-hour when available; fall back to compile-time
    // constants during the first ~20 min before the EMA has data.
    float frac        = (float)pct / 100.0f;
    float learned_bal = standbyGetLearnedBalancedRate();  // %/hr, 0 if unlearned
    float learned_act = standbyGetLearnedActiveRate();

    float h_balanced, h_maxbatt, h_active;
    float scale = standbyGetLearnedScale();   // auto-calibrated, falls back to const
    if (learned_bal > 0.01f) {
        h_balanced = (float)pct / learned_bal;
        h_maxbatt  = (float)pct / (learned_bal / scale);
    } else {
        h_balanced = BATTERY_HOURS_BALANCED_AT_100 * frac;
        h_maxbatt  = BATTERY_HOURS_MAXBATT_AT_100  * frac;
    }
    h_active = (learned_act > 0.01f) ? ((float)pct / learned_act)
                                     : (BATTERY_HOURS_ACTIVE_AT_100 * frac);

    bool is_learned = standbyEstimatorIsLearned();

    char sb[12], sm[12], sa[12];
    formatHours(h_balanced, sb, sizeof(sb));
    formatHours(h_maxbatt,  sm, sizeof(sm));
    formatHours(h_active,   sa, sizeof(sa));

    gfx->drawRoundRect(ROW_PAD_X, EST_Y, SCR_W - 2 * ROW_PAD_X, EST_H, 12, COLOR_CYAN);

    // Header + learning badge
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GRAY);
    gfx->setCursor(ROW_PAD_X + 14, EST_Y + 8);
    gfx->print("ESTIMATED RUNTIME");

    {
        const char* badge = is_learned ? "LEARNED" : "learning...";
        uint16_t bc       = is_learned ? COLOR_GREEN : COLOR_YELLOW;
        int bw = (int)strlen(badge) * 6;
        gfx->setTextColor(bc);
        gfx->setCursor(SCR_W - ROW_PAD_X - 14 - bw, EST_Y + 8);
        gfx->print(badge);
    }

    // Highlight the active profile
    uint16_t bal_col = standby.aggressive_mode ? COLOR_GRAY  : COLOR_GREEN;
    uint16_t max_col = standby.aggressive_mode ? COLOR_GREEN : COLOR_GRAY;

    gfx->setTextSize(2);
    char line1[48], line2[48];
    snprintf(line1, sizeof(line1), "%s Balanced", sb);
    snprintf(line2, sizeof(line2), "%s Max Batt", sm);

    gfx->setTextColor(bal_col);
    gfx->setCursor(ROW_PAD_X + 14, EST_Y + 22);
    gfx->print(line1);

    gfx->setTextColor(max_col);
    int l2w = (int)strlen(line2) * 6 * 2;
    gfx->setCursor(SCR_W - ROW_PAD_X - 14 - l2w, EST_Y + 22);
    gfx->print(line2);

    // Active-use hint (always visible, small)
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GRAY);
    char line3[64];
    snprintf(line3, sizeof(line3), "Full-on screen time: %s", sa);
    gfx->setCursor(ROW_PAD_X + 14, EST_Y + EST_H - 12);
    gfx->print(line3);

    // -------- Rows -----------------------------------------------------------
    drawRow(ROW_STANDBY_Y, "Standby mode",
            standbyIsEnabled() ? "ON" : "OFF",
            standbyIsEnabled() ? COLOR_GREEN : COLOR_GRAY);

    drawRow(ROW_MODE_Y, "Profile",
            standby.aggressive_mode ? "Max Battery" : "Balanced",
            standby.aggressive_mode ? COLOR_ORANGE  : COLOR_CYAN);

    // Sleep-now button
    gfx->fillRoundRect(ROW_PAD_X, ROW_SLEEP_Y, SCR_W - 2 * ROW_PAD_X, ROW_H, 12, COLOR_PURPLE);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    {
        const char* t = "Sleep now";
        int tw = (int)strlen(t) * 6 * 2;
        gfx->setCursor((SCR_W - tw) / 2, ROW_SLEEP_Y + (ROW_H / 2) - 8);
        gfx->print(t);
    }

    // Footer: current state + last wake
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_GRAY);
    const char* wake;
    switch (standbyGetLastWake()) {
        case WAKE_TOUCH:  wake = "touch";  break;
        case WAKE_BUTTON: wake = "button"; break;
        case WAKE_MOTION: wake = "motion"; break;
        case WAKE_TIMER:  wake = "timer";  break;
        case WAKE_OTHER:  wake = "other";  break;
        default:          wake = "cold";   break;
    }
    char ibuf[64];
    snprintf(ibuf, sizeof(ibuf), "State: %s   Last wake: %s",
             standbyGetStateName(standbyGetState()), wake);
    gfx->setCursor(ROW_PAD_X, ROW_INFO_Y);
    gfx->print(ibuf);
}

// =============================================================================
// TOUCH
// =============================================================================
void batteryAppHandleTouch(TouchGesture& g) {
    if (hit(g, ROW_PAD_X, ROW_STANDBY_Y, SCR_W - 2 * ROW_PAD_X, ROW_H)) {
        standbyEnable(!standbyIsEnabled());
        batteryAppDraw();
        return;
    }
    if (hit(g, ROW_PAD_X, ROW_MODE_Y, SCR_W - 2 * ROW_PAD_X, ROW_H)) {
        standbySetAggressive(!standby.aggressive_mode);
        batteryAppDraw();
        return;
    }
    if (hit(g, ROW_PAD_X, ROW_SLEEP_Y, SCR_W - 2 * ROW_PAD_X, ROW_H)) {
        standbyEnterDeepSleepNow();   // no return
        return;
    }
}