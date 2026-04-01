/*
 * filesystem.h - File System Management
 * SD Card, Music, PDF, Wallpaper file handling
 */

#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include "types.h"  // Must include types.h for FileType, FileInfo, MusicFile, etc.
#include "config.h"
#include <SD.h>
#include <FS.h>

// =============================================================================
// FILE SYSTEM
// =============================================================================

bool initializeFileSystem();
void initFilesystem();  // Alias for initializeFileSystem()
bool checkSDCardAvailable();
void handleSDCardRemoval();

// File operations
bool fileExists(const char* path);
size_t getFileSize(const char* path);
String readTextFile(const char* path);
bool writeTextFile(const char* path, const String& content);
bool deleteFile(const char* path);

// Directory operations
bool listDirectory(const char* path, FileInfo files[], int max_files, int& count);
bool createDirectory(const char* path);
bool removeDirectory(const char* path);

// Music files
int scanMusicFiles(MusicFile music_files[], int max_files);
bool loadMusicMetadata(const char* filepath, MusicFile& music);
String extractMP3Title(const char* filepath);
String extractMP3Artist(const char* filepath);

// PDF files
int scanPDFFiles(FileInfo pdf_files[], int max_files);

// Wallpaper files
int scanWallpaperFiles(WallpaperFile wallpaper_files[], int max_files);
int scanWallpapersByTheme(ThemeType theme, WallpaperFile wallpaper_files[], int max_files);
bool createWallpaperDirectories();
bool isValidWallpaperFormat(const String& filename);
String getWallpaperPath(ThemeType theme);
// String getThemeName(ThemeType theme); // Declared in themes.h

// Settings
void saveSettingsToFile();
void loadSettingsFromFile();
void saveUserPreferences();
void loadUserPreferences();

// File type detection
FileType getFileType(const String& filename);
bool isAudioFile(const String& filename);
bool isPDFFile(const String& filename);
bool isImageFile(const String& filename);

// Utility
String getFileExtension(const String& filename);
String formatFileSize(size_t bytes);

// File browser
void drawFileBrowser(const char* current_path);
void handleFileBrowserNavigation(TouchGesture& gesture, const char* current_path);

// Global arrays
extern MusicFile* music_files;
extern int total_music_files;
extern int total_pdf_files;
extern int total_wallpaper_files;

#endif // FILESYSTEM_H
