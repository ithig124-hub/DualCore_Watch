/*
 * power_manager.cpp - Adaptive Power Management Implementation
 * FUSION OS Battery Optimization
 */

#include "power_manager.h"
#include "display.h"  // For Arduino_CO5300 type definition
#include "config.h"
#include <esp32-hal-cpu.h>

extern Arduino_CO5300 *gfx;
extern SystemState system_state;
extern bool screenOn;

PowerManagerState power_manager = {
  .current_state = POWER_ACTIVE,
  .last_interaction = 0,
  .state_change_time = 0,
  .original_brightness = 200,
  .current_delay = FPS_ACTIVE,
  .sensor_poll_interval = SENSOR_POLL_ACTIVE,
  .animations_active = true,
  .needs_redraw = false
};

static AnimationState animation_state = {
  .enabled = true,
  .constant_animation = false,
  .last_burst = 0,
  .burst_duration = 0
};

// =============================================================================
// INITIALIZE POWER MANAGER
// =============================================================================
void initPowerManager() {
  power_manager.last_interaction = millis();
  power_manager.state_change_time = millis();
  power_manager.original_brightness = system_state.brightness;
  power_manager.current_state = POWER_ACTIVE;
  power_manager.current_delay = FPS_ACTIVE;
  power_manager.sensor_poll_interval = SENSOR_POLL_ACTIVE;
  power_manager.animations_active = true;
  
  // KEEP ANIMATIONS - User wants them! No battery drain issue
  animation_state.constant_animation = true;   // Keep animations running!
  animation_state.enabled = true;
  
  // Start at full power
  setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
  
  Serial.println("[POWER] Power manager initialized - SIMPLE MODE");
  Serial.println("       Screen timeout: 5 SECONDS (resets on touch)");
  Serial.println("       Animations: ALWAYS ON");
  Serial.println("       Touch screen → timer resets → stays on!");
}

// =============================================================================
// RECORD USER INTERACTION
// =============================================================================
void recordInteraction() {
  unsigned long now = millis();
  power_manager.last_interaction = now;
  
  // If we were in a low-power state, transition back to ACTIVE
  if (power_manager.current_state != POWER_ACTIVE) {
    PowerState old_state = power_manager.current_state;
    power_manager.current_state = POWER_ACTIVE;
    power_manager.state_change_time = now;
    power_manager.current_delay = FPS_ACTIVE;
    power_manager.animations_active = true;
    
    // Restore brightness
    if (screenOn) {
      gfx->setBrightness(power_manager.original_brightness);
    }
    
    // Restore CPU frequency
    setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
    
    // Trigger wake animation burst
    triggerAnimationBurst(300);
    
    Serial.printf("[POWER] %s → ACTIVE (user interaction)\n", getPowerStateName(old_state));
  }
}

// =============================================================================
// UPDATE POWER STATE (CALL IN MAIN LOOP)
// =============================================================================
void updatePowerState() {
  // Don't manage power if screen is off
  if (!screenOn) {
    power_manager.current_state = POWER_SCREEN_OFF;
    return;
  }
  
  // === POWER SAVER MODE ===
  // Auto-enable power saver at 20% battery
  if (system_state.battery_percentage <= 20 && !system_state.power_saver_enabled) {
    system_state.power_saver_enabled = true;
    Serial.println("[POWER] Auto power saver: battery <= 20%");
  }
  
  if (system_state.power_saver_enabled) {
    // POWER SAVER: reduce brightness, CPU, animation rate
    power_manager.current_state = POWER_IDLE;
    power_manager.current_delay = FPS_IDLE;  // 30 FPS instead of 60
    power_manager.sensor_poll_interval = SENSOR_POLL_IDLE;
    power_manager.animations_active = false;  // Disable watchface animations
    
    // Dim brightness to 50% of user setting
    if (screenOn) {
      int dimBrightness = power_manager.original_brightness / 2;
      if (dimBrightness < 30) dimBrightness = 30;  // Minimum visible
      gfx->setBrightness(dimBrightness);
    }
    
    // Lower CPU frequency
    setCpuFrequencyMhz(CPU_FREQ_IDLE);  // 160 MHz instead of 240
    return;
  }
  
  // NORMAL: Stay ACTIVE all the time
  power_manager.current_state = POWER_ACTIVE;
  power_manager.current_delay = FPS_ACTIVE;
  power_manager.sensor_poll_interval = SENSOR_POLL_ACTIVE;
  power_manager.animations_active = true;
}

// =============================================================================
// GET LOOP DELAY FOR CURRENT STATE
// =============================================================================
int getPowerLoopDelay() {
  return power_manager.current_delay;
}

// =============================================================================
// APPLY CPU FREQUENCY SCALING
// =============================================================================
void applyCPUFrequency() {
  int target_freq;
  
  switch (power_manager.current_state) {
    case POWER_ACTIVE:
      target_freq = CPU_FREQ_ACTIVE;
      break;
    case POWER_IDLE:
      target_freq = CPU_FREQ_IDLE;
      break;
    case POWER_DIMMED:
      target_freq = CPU_FREQ_DIMMED;
      break;
    case POWER_MINIMAL:
    case POWER_SCREEN_OFF:
      target_freq = CPU_FREQ_MINIMAL;
      break;
    default:
      target_freq = CPU_FREQ_ACTIVE;
  }
  
  setCpuFrequencyMhz(target_freq);
}

// =============================================================================
// APPLY BRIGHTNESS DIMMING
// =============================================================================
void applyBrightnessDimming() {
  if (!screenOn) return;
  
  int target_brightness;
  
  switch (power_manager.current_state) {
    case POWER_ACTIVE:
    case POWER_IDLE:
      target_brightness = power_manager.original_brightness;
      break;
    
    case POWER_DIMMED:
      target_brightness = power_manager.original_brightness / 2;  // 50%
      break;
    
    case POWER_MINIMAL:
      target_brightness = (power_manager.original_brightness * 3) / 10;  // 30%
      break;
    
    default:
      target_brightness = power_manager.original_brightness;
  }
  
  gfx->setBrightness(target_brightness);
}

// =============================================================================
// CHECK IF ANIMATIONS SHOULD RUN
// =============================================================================
bool shouldAnimationsRun() {
  // Always allow moment-based animation bursts
  unsigned long now = millis();
  if (animation_state.enabled && 
      animation_state.last_burst > 0 &&
      (now - animation_state.last_burst) < animation_state.burst_duration) {
    return true;  // Animation burst active
  }
  
  // Otherwise, only run animations in ACTIVE state
  return (power_manager.current_state == POWER_ACTIVE && power_manager.animations_active);
}

// =============================================================================
// GET SENSOR POLLING INTERVAL
// =============================================================================
int getSensorPollInterval() {
  return power_manager.sensor_poll_interval;
}

// =============================================================================
// TRIGGER MOMENT-BASED ANIMATION BURST
// =============================================================================
void triggerAnimationBurst(unsigned long duration_ms) {
  animation_state.last_burst = millis();
  animation_state.burst_duration = duration_ms;
  Serial.printf("[POWER] Animation burst triggered (%lu ms)\n", duration_ms);
}

// =============================================================================
// FORCE ACTIVE STATE (FOR GAMES)
// =============================================================================
void forceActiveState() {
  if (power_manager.current_state != POWER_ACTIVE) {
    power_manager.current_state = POWER_ACTIVE;
    power_manager.last_interaction = millis();
    power_manager.state_change_time = millis();
    power_manager.current_delay = FPS_ACTIVE;
    power_manager.animations_active = true;
    
    setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
    if (screenOn) {
      gfx->setBrightness(power_manager.original_brightness);
    }
  }
}

// =============================================================================
// CHECK IF SCREEN SHOULD TURN OFF
// =============================================================================
bool shouldScreenTurnOff() {
  if (!screenOn) return false;
  
  unsigned long idle_time = millis() - power_manager.last_interaction;
  
  // Extended timeout for games
  if (system_state.current_screen == SCREEN_GAMES ||
      system_state.current_screen == SCREEN_BOSS_RUSH ||
      system_state.current_screen == SCREEN_TRAINING ||
      system_state.current_screen == SCREEN_CHARACTER_GAME) {
    return (idle_time >= GAME_SCREEN_TIMEOUT);
  }
  
  // Normal timeout
  return (idle_time >= SCREEN_OFF_TIMEOUT);
}

// =============================================================================
// GET POWER STATE NAME
// =============================================================================
const char* getPowerStateName(PowerState state) {
  switch (state) {
    case POWER_ACTIVE:     return "ACTIVE";
    case POWER_IDLE:       return "IDLE";
    case POWER_DIMMED:     return "DIMMED";
    case POWER_MINIMAL:    return "MINIMAL";
    case POWER_SCREEN_OFF: return "SCREEN_OFF";
    default:               return "UNKNOWN";
  }
}

// =============================================================================
// POWER SAVER MODE TOGGLE
// =============================================================================
void togglePowerSaver() {
  system_state.power_saver_enabled = !system_state.power_saver_enabled;
  
  if (system_state.power_saver_enabled) {
    Serial.println("[POWER] Power Saver: ON");
  } else {
    // Restore full power
    Serial.println("[POWER] Power Saver: OFF");
    power_manager.current_state = POWER_ACTIVE;
    power_manager.current_delay = FPS_ACTIVE;
    power_manager.animations_active = true;
    setCpuFrequencyMhz(CPU_FREQ_ACTIVE);
    if (screenOn) {
      gfx->setBrightness(power_manager.original_brightness);
    }
  }
}

bool isPowerSaverEnabled() {
  return system_state.power_saver_enabled;
}
