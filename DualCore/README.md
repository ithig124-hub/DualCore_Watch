# DualCore - Complete ESP32 Dual-OS Firmware

## 🎮 FULL Fusion OS + FULL Widget OS in ONE Folder

This folder contains the **COMPLETE** code for both operating systems with Dual Core Mode switching.

---

## 📁 What's Inside

```
DualCore/
├── DualCore.ino           ← MAIN ENTRY (open this in Arduino IDE)
├── fusion_os_full.cpp     ← COMPLETE Fusion OS (952 lines)
├── widget_os_full.cpp     ← COMPLETE Widget OS (15,057 lines)
├── config.h               ← Hardware config + SCREEN_DUAL_CORE
├── apps.cpp/h             ← Fusion OS apps
├── games.cpp/h            ← Fusion OS games
├── gacha.cpp/h            ← Fusion OS gacha system
├── ... (65+ more files)   ← All Fusion OS modules
├── pin_config.h           ← Widget OS pins
├── es8311.c/h             ← Widget OS audio
└── README.md              ← This file
```

**Total: 72 files, ONE .ino**

---

## ⚡ How It Works

```
                    POWER ON
                       │
                       ▼
            ┌──────────────────┐
            │ Check NVS Flag:  │
            │ OS_FUSION or     │
            │ OS_WIDGET?       │
            └────────┬─────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
        ▼                         ▼
┌───────────────┐        ┌───────────────┐
│   FUSION OS   │        │  WIDGET OS    │
│ ───────────── │        │ ───────────── │
│ • 952 lines   │        │ • 15,057 lines│
│ • Anime watch │        │ • LVGL UI     │
│ • Games/Gacha │        │ • Full widgets│
│ • Arduino_GFX │        │ • Battery mgmt│
│               │        │               │
│  DUAL CORE:   │        │  DUAL CORE:   │
│  ┌─────────┐  │        │  ┌──────────┐ │
│  │ACTIVATE │  │        │  │ACTIVATED │ │
│  │(golden) │  │        │  │ (golden) │ │
│  └────┬────┘  │        │  └────┬─────┘ │
│       │       │        │       │       │
│       ▼       │        │       ▼       │
│ 4-Phase Anime │        │   Immediate   │
│  Animation    │        │    Switch     │
│  (3 seconds)  │        │               │
└───────┬───────┘        └───────┬───────┘
        │                        │
        └──────────┬─────────────┘
                   │
                   ▼
            ESP.restart()
                   │
                   ▼
           OTHER OS BOOTS
```

---

## 🔋 Battery Impact

**ZERO drain from inactive OS!**

| When Running | Fusion OS RAM | Widget OS RAM |
|--------------|---------------|---------------|
| Fusion OS    | ~50KB active  | 0 (not loaded)|
| Widget OS    | 0 (not loaded)| ~200KB active |

The `ESP.restart()` completely clears RAM - no hidden processes.

---

## 🚀 How To Flash

### 1. Open in Arduino IDE
```
File → Open → DualCore/DualCore.ino
```
(All other files will load automatically)

### 2. Board Settings
```
Board:         ESP32S3 Dev Module
USB Mode:      USB-OTG (TinyUSB)
PSRAM:         OPI PSRAM
Flash Mode:    QIO 80MHz  
Flash Size:    16MB (or your board size)
Partition:     Default 4MB or Huge APP
```

### 3. Upload
Click Upload button. Done!

---

## 🔄 How To Switch OS

### Fusion OS → Widget OS

1. From watchface, swipe to **App Grid**
2. Navigate to find **Dual Core** app
3. Tap golden **"ACTIVATE"** button
4. **Watch anime animation** (3 seconds):
   - ⚡ Energy gathering
   - 💥 Aura expansion
   - 🔀 Screen fracture
   - 🔵 Fade to Widget
5. Watch restarts into Widget OS

### Widget OS → Fusion OS

1. Swipe to **CAT_ABOUT** category
2. Find **Dual Core Mode** card (second subcard)
3. Tap golden **"ACTIVATED"** button
4. **Immediate restart** (no animation)
5. Watch boots into Fusion OS

---

## 📝 Technical Details

### Boot Flag
```
Location:  ESP32 NVS (flash memory)
Namespace: "dualcore"
Key:       "activeOS"
Values:    0 = Fusion, 1 = Widget
```

### Why Can't Both Run Together?
- Fusion OS: Uses Arduino_GFX (direct pixel rendering)
- Widget OS: Uses LVGL (buffered UI framework)
- Different display drivers, different memory models
- Clean restart ensures proper initialization

### Code Size
| Component | Lines | Size |
|-----------|-------|------|
| DualCore.ino | 128 | 6.8KB |
| fusion_os_full.cpp | 952 | 29KB |
| widget_os_full.cpp | 15,057 | 674KB |
| Support files | ~10,000 | ~400KB |
| **Total** | **~26,000** | **~1.1MB** |

---

## ❓ Troubleshooting

### "Can't compile" errors
1. Ensure all files are in DualCore folder
2. Check: Tools → Board → ESP32S3 Dev Module
3. Check: Tools → PSRAM → OPI PSRAM

### "Stuck in one OS"
Add this to setup() temporarily to reset:
```cpp
setActiveOS(OS_FUSION);  // or OS_WIDGET
```

### Animation stutters
Normal on first boot. If persistent, reduce animation frames in `fusion_os_full.cpp`.

### LVGL errors
Widget OS needs PSRAM. Verify PSRAM is enabled in board settings.

---

## 🙏 Credits

- **Fusion OS**: Anime Gaming Watch by @ithig124-hub
- **Widget OS**: S3 MiniOS v7.4 by @ithig124-hub
- **Hardware**: ESP32-S3-Touch-AMOLED-2.06" (Waveshare)

---

## 📜 License

MIT License - Free to use and modify!
