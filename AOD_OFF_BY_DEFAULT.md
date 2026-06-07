# AOD OFF By Default — Patch Notes

This patch makes **standby (and therefore AOD) OFF by default** in `Master_ESP32_Watch_206`.

## What changed

Files modified:

1. `standby_mode.h`
   - Added `bool aod_enabled` field to `StandbyInfo`.
   - Added public API:
     - `void standbySetAODEnabled(bool on);`
     - `bool standbyIsAODEnabled();`

2. `standby_mode.cpp`
   - Global `standby` initializer:
     - `.enabled              = false`   ← whole standby state machine OFF on boot
     - `.aod_enabled          = false`   ← AOD step OFF even if standby is later enabled
   - `standbyTick()` IDLE-state branch now skips AOD when `aod_enabled == false`
     and falls straight through to DEEP sleep when the DEEP timer elapses.
   - Implemented `standbySetAODEnabled()` / `standbyIsAODEnabled()`.
     Turning AOD off while currently displaying AOD bounces back to ACTIVE.

## Behavior

| State of `standby.enabled` | State of `standby.aod_enabled` | Idle path                          |
|---------------------------:|:------------------------------:|:-----------------------------------|
| `false` (default)          | (n/a)                          | Stays ACTIVE forever — no IDLE/AOD/DEEP |
| `true`                     | `false` (default)              | ACTIVE → IDLE → DEEP (AOD skipped) |
| `true`                     | `true`                         | ACTIVE → IDLE → AOD → DEEP         |

## Re-enabling at runtime (if desired)

```cpp
#include "standby_mode.h"

standbyEnable(true);          // turn the standby state machine on
standbySetAODEnabled(true);   // turn the AOD step on
```

No other files need changing — `standbyInit()` and `standbyTick()` are already wired
into `Master_ESP32_Watch_206.ino`.
