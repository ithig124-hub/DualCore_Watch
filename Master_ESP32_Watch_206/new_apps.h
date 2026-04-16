/*
 * new_apps.h - New Apps Header
 * Pomodoro Timer, Habit Tracker, Daily Dungeon, Streak Viewer, Card Crafting
 */

#ifndef NEW_APPS_H
#define NEW_APPS_H

// Pomodoro Timer
void initPomodoroApp();
void drawPomodoroApp();
void updatePomodoro();
void handlePomodoroTouch(int x, int y);

// Habit Tracker
void initHabitsApp();
void drawHabitsApp();
void handleHabitsTouch(int x, int y);

// Daily Dungeon
void initDungeonApp();
void drawDungeonApp();
void handleDungeonTouch(int x, int y);

// Login Streak Viewer
void drawStreakApp();

// Card Crafting
void initCraftApp();
void drawCraftApp();
void handleCraftTouch(int x, int y);

#endif // NEW_APPS_H
