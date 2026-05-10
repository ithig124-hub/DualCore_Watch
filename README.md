# ESP32 Watch - Ported for ESP32-S3-Touch-AMOLED-2.06"

This is a ported version of the ESP32_Watch project for the **Waveshare ESP32-S3-Touch-AMOLED-2.06"** board.

## Hardware Changes from 1.8" to 2.06" Board

| Feature | Original (1.8") | New (2.06") |
|---------|-----------------|-------------|
| Display Driver | SH8601 | CO5300 |
| Resolution | 368x448 | 410x502 |
| Touch INT Pin | GPIO 21 | GPIO 38 |
| Touch RST Pin | N/A (-1) | GPIO 9 |
| LCD RST Pin | Not defined | GPIO 8 |
| Column Offset | 0 | 22 |
| SD Card CS | N/A | GPIO 17 |

## Files Modified for Port

1. **pin_config.h** - Updated all pin definitions
2. **config.h** - Updated resolution, touch pins, and display settings
3. **lv_conf.h** - Updated LVGL resolution settings
4. **ESP32_Watch.ino** - Changed display driver from Arduino_SH8601 to Arduino_CO5300
5. **display.h** - Updated display driver type declaration
6. **display.cpp** - Updated initialization message
7. **touch.cpp** - Added touch reset pin support, updated initialization

## Key Code Changes

### Display Initialization (ESP32_Watch.ino)
```cpp
// OLD (1.8" board - SH8601)
Arduino_SH8601 *gfx = new Arduino_SH8601(
    bus, GFX_NOT_DEFINED, 0, LCD_WIDTH, LCD_HEIGHT);

// NEW (2.06" board - CO5300)
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0 /* rotation */, LCD_WIDTH, LCD_HEIGHT,
    22 /* col_offset1 */, 0 /* row_offset1 */,
    0 /* col_offset2 */, 0 /* row_offset2 */);
```

### Touch Initialization (touch.cpp)
```cpp
// NEW - Touch reset support for 2.06" board
bool initializeTouch() {
  #if defined(TP_RST) && TP_RST >= 0
    pinMode(TP_RST, OUTPUT);
    digitalWrite(TP_RST, LOW);
    delay(10);
    digitalWrite(TP_RST, HIGH);
    delay(50);
  #endif
  // ... rest of initialization
}
```

## Required Libraries

Install these libraries in Arduino IDE:
- **Arduino_GFX_Library** (with CO5300 support) - Included in `libraries/` folder
- **Arduino_DriveBus** (for touch/power chips) - Included in `libraries/` folder
- **lvgl** (v8.x or v9.x)
- **XPowersLib** (for AXP2101 PMU)

**Note:** The `libraries/` folder contains the Waveshare-specific versions of Arduino_GFX and Arduino_DriveBus that include CO5300 and FT3168 support. Copy these to your Arduino libraries folder or add them to your project.

## Arduino IDE Board Settings

1. Board: **ESP32S3 Dev Module**
2. PSRAM: **OPI PSRAM**
3. Flash Mode: **QIO 80MHz**
4. Flash Size: **16MB (128Mb)**
5. Partition Scheme: **16M Flash (3MB APP/9.9MB FATFS)**
6. USB CDC On Boot: **Enabled** (for serial output)

## Notes

- The larger display (410x502 vs 368x448) provides **~30% more screen real estate**
- All UI elements use LCD_WIDTH/LCD_HEIGHT constants for automatic scaling
- Touch coordinates are returned directly in screen pixels by the FT3168 controller
- The CO5300 requires a column offset of 22 pixels

## UI Optimizations for 2.06" Display

The following UI improvements were made to take advantage of the larger display:

### App Grid (all 3 pages)
- Larger app icons (115x100px vs 100x88px)
- Better centered layout with proper gaps (15px)
- Taller headers (55px vs 48px)
- Larger title text (size 3 vs size 2)
- Larger page indicators

### All Watchfaces Optimized
- **Luffy**: Larger time (font 10), wider stat cards (120x60), activity rings at Y=440
- **Jinwoo**: Larger hexagons, wider ARISE badge (140px), cards at Y=350
- **Yugo**: Font size 7 time, activity rings at Y=390
- **Naruto**: Font size 7, larger Sage Aura, rings at Y=390
- **Goku**: Larger Ultra Instinct aura, rings at Y=390
- **Tanjiro**: Bigger sun flames, font size 7, rings at Y=390
- **Gojo**: Larger infinity aura, Six Eyes glow, rings at Y=390
- **Levi**: Bigger blade shine effects, rings at Y=390
- **Saitama**: Font size 7, larger OK text (size 4), rings at Y=390
- **Deku**: More OFA lightning bolts, rings at Y=390
- **BoBoiBoy**: Larger power band ring (155px radius), bigger element dots, wider name badge (180px), cards at Y=355, rings at Y=450

### Music Player
- Taller header (55px)
- Larger album art area (180x180px vs 150x150px)
- Bigger vinyl disc visualization (70px)
- Larger track info text (size 3)
- Wider progress bar

### Games Menu
- Larger game cards (120x95px vs 105x85px)
- Bigger adventure cards (115x40px)
- Larger stats panels
- Better positioned back button

### Character Effects
- Larger portal circles (35px radius)
- Bigger aura rings
- More/larger speed lines
- Bigger flame triangles
- Larger Six Eyes glow (10px)
- Extended blade shine effects
- More lightning bolts

### Activity Rings (All Characters)
- Larger outer ring (40px radius, 7px thickness)
- Larger inner ring (28px radius, 6px thickness)
- Bigger center elements

### UI Utilities
- Larger battery indicator (45x20px)
- Better positioned step counter

## Original Project

This port is based on: https://github.com/ithig124-hub/ESP32_Watch-BETTER

Reference for 2.06" board: https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.06
