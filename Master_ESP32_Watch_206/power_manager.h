/*
 * power_manager.h - Adaptive Power Management System
 * FUSION OS Battery Optimization
 * 
 * Target: 2.5h → 10h+ battery life
 * 
 * Features:
 * - Smart Idle Engine (5s/10s/15s thresholds)
 * - Adaptive FPS (60/30/15/1 FPS based on activity)
 * - CPU Frequency Scaling (240/160/80/40 MHz)
 * - Brightness Auto-Dim
 * - Moment-Based Animations
 * - Smart Sensor Polling
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// =============================================================================
// POWER STATES
// =============================================================================
enum PowerState {
  POWER_ACTIVE = 0,    // Full power - user interacting (60 FPS, 240 MHz)
  POWER_IDLE,          // No interaction for 5s (30 FPS, 160 MHz)
  POWER_DIMMED,        // No interaction for 10s (15 FPS, 80 MHz, brightness 50%)
  POWER_MINIMAL,       // No interaction for 15s (1 FPS, 40 MHz, brightness 30%)
  POWER_SCREEN_OFF     // Screen timeout (0 FPS, 40 MHz, screen off)
};

// =============================================================================
// IDLE THRESHOLDS - Quick timeout, resets on touch
// =============================================================================
#define IDLE_THRESHOLD_1      5000    // 5s → POWER_IDLE
#define IDLE_THRESHOLD_2      5000    // 5s → POWER_DIMMED
#define IDLE_THRESHOLD_3      5000    // 5s → POWER_MINIMAL
#define SCREEN_OFF_TIMEOUT    5000    // 5 seconds → Screen off (resets on touch!)
#define GAME_SCREEN_TIMEOUT   300000  // 5 min for games

// =============================================================================
// FPS DELAYS (milliseconds)
// =============================================================================
#define FPS_ACTIVE    16      // 60 FPS
#define FPS_IDLE      33      // 30 FPS
#define FPS_DIMMED    66      // 15 FPS
#define FPS_MINIMAL   1000    // 1 FPS

// =============================================================================
// CPU FREQUENCIES (MHz)
// =============================================================================
#define CPU_FREQ_ACTIVE   240
#define CPU_FREQ_IDLE     160
#define CPU_FREQ_DIMMED   80
#define CPU_FREQ_MINIMAL  40

// =============================================================================
// SENSOR POLLING INTERVALS (milliseconds)
// =============================================================================
#define SENSOR_POLL_ACTIVE   250    // 4 Hz
#define SENSOR_POLL_IDLE     1000   // 1 Hz
#define SENSOR_POLL_DIMMED   2000   // 0.5 Hz
#define SENSOR_POLL_MINIMAL  5000   // 0.2 Hz

// =============================================================================
// ANIMATION CONTROL
// =============================================================================
struct AnimationState {
  bool enabled;
  bool constant_animation;     // If true, animation runs constantly (battery drain)
  unsigned long last_burst;    // Last moment-based animation burst
  unsigned long burst_duration; // How long the burst lasts (ms)
};

// =============================================================================
// POWER MANAGER STATE
// =============================================================================
struct PowerManagerState {
  PowerState current_state;
  unsigned long last_interaction;
  unsigned long state_change_time;
  int original_brightness;
  int current_delay;
  int sensor_poll_interval;
  bool animations_active;
  bool needs_redraw;
};

// =============================================================================
// GLOBAL POWER MANAGER
// =============================================================================
extern PowerManagerState power_manager;

// =============================================================================
// FUNCTIONS
// =============================================================================

// Initialize power management system
void initPowerManager();

// Update power state based on activity
void updatePowerState();

// Record user interaction (resets idle timer)
void recordInteraction();

// Get appropriate loop delay for current power state
int getPowerLoopDelay();

// Apply CPU frequency scaling
void applyCPUFrequency();

// Apply brightness dimming
void applyBrightnessDimming();

// Check if animations should be active
bool shouldAnimationsRun();

// Get sensor polling interval for current state
int getSensorPollInterval();

// Trigger moment-based animation burst
void triggerAnimationBurst(unsigned long duration_ms);

// Force active state (for games, important interactions)
void forceActiveState();

// Check if screen should turn off
bool shouldScreenTurnOff();

// Get power state name (for debugging)
const char* getPowerStateName(PowerState state);

// Power Saver mode toggle and query
void togglePowerSaver();
bool isPowerSaverEnabled();

#endif // POWER_MANAGER_H
