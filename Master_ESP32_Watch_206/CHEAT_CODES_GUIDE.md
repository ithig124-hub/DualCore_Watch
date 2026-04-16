# 🎮 FUSION OS - Developer Cheat Codes

## Overview
Two secret passcodes in the Shop app allow you to test max-level gameplay and undo changes.

---

## 🔓 Code "1001" - Instant Max Level

### What it does:
1. **Creates automatic backup** of your current progress:
   - Current level
   - Current XP
   - All title unlock states (22 titles)
   
2. **Instantly levels you up to:**
   - Level 100
   - XP reset to 0
   - ALL 22 titles unlocked for current character
   
3. **Saves to NVS** (persistent storage)

### How to use:
1. Open the **Shop** app
2. Tap the **GEMS** button (with passcode icon)
3. Enter: `1001`
4. Press **OK**
5. See green "SUCCESS!" screen

### Result:
✅ Level 100  
✅ All Titles Unlocked  
✅ Backup Created (for undo)

---

## ↩️ Code "2002" - Undo Restore

### What it does:
1. **Checks if backup exists** (from using code "1001")
2. **Restores your character to:**
   - Previous level (before "1001")
   - Previous XP (before "1001")
   - Previous title unlock states
   
3. **Saves restored state to NVS**
4. **Clears the backup** (can only undo once per cheat)

### How to use:
1. Open the **Shop** app  
2. Tap the **GEMS** button
3. Enter: `2002`
4. Press **OK**
5. See cyan "RESTORED!" screen

### Result (if backup exists):
✅ Level restored  
✅ Titles restored  
✅ XP restored  

### Result (if NO backup):
⚠️ Orange "NO BACKUP!" warning  
(You must use "1001" first to create a backup)

---

## 🔒 Backup System Details

### Backup Variables (in `apps.cpp`):
```cpp
static int backup_level = 0;           // Saved level
static long backup_xp = 0;             // Saved XP
static bool backup_titles[22] = {false}; // Saved title unlock states
static bool backup_exists = false;     // Flag: is backup valid?
```

### Backup Creation:
- Triggered automatically when you enter "1001"
- Stores current state BEFORE applying the cheat
- Per-character (only backs up active character)

### Backup Restore:
- Only works if `backup_exists == true`
- Automatically clears backup after restore (single-use)
- To restore again, you must re-enter "1001" then "2002"

---

## 📝 Notes

### Character-Specific
- Backup is per current character
- If you use "1001" on Luffy, then switch to Gojo, the backup still belongs to Luffy
- Switching characters does NOT clear the backup

### Persistent Storage
- Both codes write to ESP32 NVS (Non-Volatile Storage)
- Changes survive watch reboots
- **Important:** Backup variables are in RAM only (not saved to NVS)
- If you reboot the watch after "1001", you CANNOT undo with "2002"

### Testing Workflow
1. Check your current level (e.g., Level 5)
2. Enter "1001" → Level 100 (backup of Level 5 created)
3. Play around with max level
4. Enter "2002" → Back to Level 5
5. Enter "1001" again → Level 100 (new backup of Level 5 created)
6. Reboot watch → "2002" will show "NO BACKUP" (RAM cleared)

---

## 🐛 Troubleshooting

### "DENIED!" message
- You entered the wrong code
- Try again (code is case-sensitive: must be exactly "1001" or "2002")

### "NO BACKUP!" message  
- You tried "2002" without using "1001" first
- OR you already used "2002" once (backup cleared)
- OR you rebooted the watch (RAM cleared)

### Titles not restoring correctly
- Check that all 22 title states are being backed up
- View Serial Monitor output for debug logs
- Look for: `[SHOP] Backup created: Level X, XP Y`

---

## 🔧 Implementation Details

### File Modified: `apps.cpp`

**Lines 1652-1656:** Backup variable declarations  
**Lines 1686-1739:** "1001" cheat handler (with backup creation)  
**Lines 1740-1797:** "2002" undo handler (with restore logic)

### Key Functions Called:
- `getCurrentCharacterXP()` - Get active character's XP data
- `calculateXPForLevel(int level)` - Calculate XP needed for level
- `saveXPData()` - Save to NVS (from `xp_system.cpp`)

### Serial Monitor Debug Output:
```
[SHOP] Backup created: Level 5, XP 120
[SHOP] DEV MODE: Level 100 + All Titles Unlocked!
[SHOP] UNDO: Restored to Level 5, XP 120
```

---

## ✅ Feature Status

| Feature | Status | Notes |
|---------|--------|-------|
| "1001" Instant Level 100 | ✅ Working | Creates backup automatically |
| "2002" Undo Restore | ✅ Working | Restores from backup |
| Backup Creation | ✅ Working | Saves level, XP, 22 titles |
| Backup Persistence | ⚠️ RAM Only | Cleared on reboot |
| Per-Character Backup | ✅ Working | Backup follows active character |
| Visual Feedback | ✅ Working | Green/Cyan success screens |

---

**Last Updated:** December 2025  
**Part of:** Fusion OS v2.0 
