# DualCore_Watch — STANDBY + AOD + BATTERY APP — MEGA BUNDLE

> Single-file dump containing:
>   1. Conversation summary
>   2. Architecture overview
>   3. Every NEW file (full source)
>   4. Every EDIT to existing files (unified diffs + the exact snippets)
>   5. Integration guide
>   6. Verification checklist
>
> Target board: ESP32-S3-Touch-AMOLED-2.06" (CO5300 AMOLED 410x502, FT3168 touch,
> QMI8658 IMU, PCF85063 RTC, AXP2101 PMU).

---

## 1. CONVERSATION SUMMARY

**Goal**: add a real layered standby system to the existing
`ithig124-hub/DualCore_Watch/Master_ESP32_Watch_206` Arduino project, with:

- 4-level power state machine: **ACTIVE → IDLE → AOD → DEEP-SLEEP**
- AOD that **reuses the existing per-character watchface** (just dimmed +
  animations off), not a custom minimal render
- A new **"Battery" app** reachable only via the infinite main-screen loop
  (Watchface → Steps → AppGrid → Character Stats → Battery → Watchface), NOT
  in the app grid
- Battery app shows %, charging state, runtime estimator for both profiles,
  and lets the user toggle Standby ON/OFF and switch Balanced ↔ Max-Battery
- **Self-learning** runtime predictor (EMA over real %/hour drain) so the
  estimator becomes accurate after ~20 minutes of use
- **Auto-calibrated** deep-sleep efficiency multiplier (no fixed constant)
- All learning state in `RTC_DATA_ATTR` — **zero NVS writes, zero flash wear,
  ~30 bytes RTC RAM, no measurable battery cost**

**Hardware wake sources wired**:
- FT3168 touch INT (GPIO 38)
- BOOT button (GPIO 0)
- QMI8658 motion INT (optional, define `STANDBY_IMU_INT_PIN`)

**Profiles** (selectable from Battery app):
| Profile      | Behaviour                                               |
|--------------|---------------------------------------------------------|
| Balanced     | IDLE → AOD → DEEP. Premium instant-wake, AMOLED-friendly|
| Max Battery  | IDLE → DEEP directly. Skips AOD for maximum mAh         |

---

## 2. ARCHITECTURE

```
ACTIVE  ─(15s no touch)→  IDLE  ─(45s)→  AOD  ─(3min)→  DEEP-SLEEP
  ↑_________________________________________________________|
           (touch / button / motion → ACTIVE)
```

| State  | CPU       | Display       | Mechanism                             |
|--------|-----------|---------------|---------------------------------------|
| ACTIVE | 240 MHz   | 100 %         | normal loop                           |
| IDLE   | ~160 MHz  | 50 %          | dim + lower FPS                       |
| AOD    | 40 MHz    | 15 %, frozen  | char watchface, light-sleep ticks     |
| DEEP   | OFF       | OFF           | esp_deep_sleep_start, EXT1 wake       |

Self-learning estimator (3 EMAs, RTC RAM):
- `active`   — sampled every 5 min while interacting
- `balanced` — sampled every 5 min while in IDLE/AOD
- `deep`     — measured per deep-sleep session (snapshot pct+min, diff on wake)

`STANDBY_MAXBATT_SCALE` = `balanced / deep` once both learned (auto-calibrated),
else compile-time default 3.0.
