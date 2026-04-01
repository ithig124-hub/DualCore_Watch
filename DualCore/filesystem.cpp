/*
 * filesystem.cpp - File System Implementation
 */

#include "filesystem.h"
#include "config.h"
#include "display.h"
#include <SD_MMC.h>
#include <FS.h>

// Forward declarations
int scanMusicCount();
int scanPDFCount();

extern SystemState system_state;

MusicFile* music_files = nullptr;
int total_music_files = 0;
int total_pdf_files = 0;
int total_wallpaper_files = 0;

static bool sd_available = false;

// =============================================================================
// INITIALIZATION
// =============================================================================

bool initializeFileSystem() {
  Serial.println("[FS] Initializing file system...");
  
  // Try SDMMC first
  if (SD_MMC.begin("/sdcard", true)) {
    Serial.println("[FS] SD card mounted (SDMMC)");
    sd_available = true;
    system_state.filesystem_available = true;
    
    // Create directories
    createWallpaperDirectories();
    
    // Scan files
    total_music_files = scanMusicCount();
    total_pdf_files = scanPDFCount();
    system_state.total_mp3_files = total_music_files;
    system_state.total_pdf_files = total_pdf_files;
    
    Serial.printf("[FS] Found %d MP3, %d PDF files\n", total_music_files, total_pdf_files);
    return true;
  }
  
  Serial.println("[FS] No SD card found");
  system_state.filesystem_available = false;
  return false;
}

// Alias function for initializeFileSystem
void initFilesystem() {
  initializeFileSystem();
}

bool checkSDCardAvailable() {
  return sd_available;
}

void handleSDCardRemoval() {
  sd_available = false;
  system_state.filesystem_available = false;
  total_music_files = 0;
  total_pdf_files = 0;
}

// =============================================================================
// FILE OPERATIONS
// =============================================================================

bool fileExists(const char* path) {
  if (!sd_available) return false;
  return SD_MMC.exists(path);
}

size_t getFileSize(const char* path) {
  if (!sd_available) return 0;
  File file = SD_MMC.open(path);
  if (!file) return 0;
  size_t size = file.size();
  file.close();
  return size;
}

String readTextFile(const char* path) {
  if (!sd_available) return "";
  File file = SD_MMC.open(path);
  if (!file) return "";
  String content = file.readString();
  file.close();
  return content;
}

bool writeTextFile(const char* path, const String& content) {
  if (!sd_available) return false;
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) return false;
  file.print(content);
  file.close();
  return true;
}

bool deleteFile(const char* path) {
  if (!sd_available) return false;
  return SD_MMC.remove(path);
}

// =============================================================================
// DIRECTORY OPERATIONS
// =============================================================================

bool listDirectory(const char* path, FileInfo files[], int max_files, int& count) {
  if (!sd_available) { count = 0; return false; }
  
  File root = SD_MMC.open(path);
  if (!root || !root.isDirectory()) { count = 0; return false; }
  
  count = 0;
  File file = root.openNextFile();
  while (file && count < max_files) {
    files[count].filename = file.name();
    files[count].filepath = String(path) + "/" + file.name();
    files[count].size = file.size();
    files[count].is_directory = file.isDirectory();
    files[count].type = getFileType(file.name());
    count++;
    file = root.openNextFile();
  }
  return true;
}

bool createDirectory(const char* path) {
  if (!sd_available) return false;
  return SD_MMC.mkdir(path);
}

bool removeDirectory(const char* path) {
  if (!sd_available) return false;
  return SD_MMC.rmdir(path);
}

// =============================================================================
// MUSIC FILES
// =============================================================================

int scanMusicCount() {
  if (!sd_available) return 0;
  
  int count = 0;
  File music_dir = SD_MMC.open("/Music");
  if (!music_dir) return 0;
  
  File file = music_dir.openNextFile();
  while (file) {
    if (isAudioFile(file.name())) count++;
    file = music_dir.openNextFile();
  }
  return count;
}

int scanMusicFiles(MusicFile music_files[], int max_files) {
  if (!sd_available) return 0;
  
  int count = 0;
  File music_dir = SD_MMC.open("/Music");
  if (!music_dir) return 0;
  
  File file = music_dir.openNextFile();
  while (file && count < max_files) {
    if (isAudioFile(file.name())) {
      music_files[count].filename = file.name();
      music_files[count].filepath = String("/Music/") + file.name();
      music_files[count].title = extractMP3Title(file.path());
      music_files[count].artist = extractMP3Artist(file.path());
      count++;
    }
    file = music_dir.openNextFile();
  }
  return count;
}

bool loadMusicMetadata(const char* filepath, MusicFile& music) {
  music.title = extractMP3Title(filepath);
  music.artist = extractMP3Artist(filepath);
  return true;
}

String extractMP3Title(const char* filepath) {
  // Simple extraction - use filename if no ID3 tag
  String path = String(filepath);
  int lastSlash = path.lastIndexOf('/');
  int lastDot = path.lastIndexOf('.');
  if (lastSlash >= 0 && lastDot > lastSlash) {
    return path.substring(lastSlash + 1, lastDot);
  }
  return "Unknown";
}

String extractMP3Artist(const char* filepath) {
  return "Unknown Artist";
}

// =============================================================================
// PDF FILES
// =============================================================================

int scanPDFCount() {
  if (!sd_available) return 0;
  
  int count = 0;
  File docs_dir = SD_MMC.open("/Documents");
  if (!docs_dir) return 0;
  
  File file = docs_dir.openNextFile();
  while (file) {
    if (isPDFFile(file.name())) count++;
    file = docs_dir.openNextFile();
  }
  return count;
}

int scanPDFFiles(FileInfo pdf_files[], int max_files) {
  if (!sd_available) return 0;
  
  int count = 0;
  File docs_dir = SD_MMC.open("/Documents");
  if (!docs_dir) return 0;
  
  File file = docs_dir.openNextFile();
  while (file && count < max_files) {
    if (isPDFFile(file.name())) {
      pdf_files[count].filename = file.name();
      pdf_files[count].filepath = String("/Documents/") + file.name();
      pdf_files[count].size = file.size();
      pdf_files[count].type = FILE_PDF;
      count++;
    }
    file = docs_dir.openNextFile();
  }
  return count;
}

// =============================================================================
// WALLPAPER FILES
// =============================================================================

int scanWallpaperFiles(WallpaperFile wallpaper_files[], int max_files) {
  int count = 0;
  count += scanWallpapersByTheme(THEME_LUFFY_GEAR5, wallpaper_files + count, max_files - count);
  count += scanWallpapersByTheme(THEME_SUNG_JINWOO, wallpaper_files + count, max_files - count);
  count += scanWallpapersByTheme(THEME_YUGO_WAKFU, wallpaper_files + count, max_files - count);
  total_wallpaper_files = count;
  return count;
}

int scanWallpapersByTheme(ThemeType theme, WallpaperFile wallpaper_files[], int max_files) {
  if (!sd_available) return 0;
  
  String path = getWallpaperPath(theme);
  File dir = SD_MMC.open(path.c_str());
  if (!dir) return 0;
  
  int count = 0;
  File file = dir.openNextFile();
  while (file && count < max_files) {
    if (isValidWallpaperFormat(file.name())) {
      wallpaper_files[count].filename = file.name();
      wallpaper_files[count].filepath = path + "/" + file.name();
      wallpaper_files[count].theme = theme;
      wallpaper_files[count].file_size = file.size();
      wallpaper_files[count].is_valid = true;
      count++;
    }
    file = dir.openNextFile();
  }
  return count;
}

bool createWallpaperDirectories() {
  if (!sd_available) return false;
  
  createDirectory("/Wallpapers");
  createDirectory("/Wallpapers/Luffy");
  createDirectory("/Wallpapers/JinWoo");
  createDirectory("/Wallpapers/Yugo");
  createDirectory("/Music");
  createDirectory("/Documents");
  
  return true;
}

bool isValidWallpaperFormat(const String& filename) {
  String lower = filename;
  lower.toLowerCase();
  return lower.endsWith(".jpg") || lower.endsWith(".jpeg") || 
         lower.endsWith(".png") || lower.endsWith(".bmp");
}

String getWallpaperPath(ThemeType theme) {
  switch(theme) {
    case THEME_LUFFY_GEAR5: return "/Wallpapers/Luffy";
    case THEME_SUNG_JINWOO: return "/Wallpapers/JinWoo";
    case THEME_YUGO_WAKFU:  return "/Wallpapers/Yugo";
    default: return "/Wallpapers";
  }
}

// getThemeName is defined in themes.cpp - removed duplicate here

// =============================================================================
// SETTINGS
// =============================================================================

void saveSettingsToFile() {
  if (!sd_available) return;
  
  String settings = "{\n";
  settings += "  \"theme\": " + String((int)system_state.current_theme) + ",\n";
  settings += "  \"brightness\": " + String(system_state.brightness) + ",\n";
  settings += "  \"step_goal\": " + String(system_state.step_goal) + "\n";
  settings += "}";
  
  writeTextFile("/settings.json", settings);
}

void loadSettingsFromFile() {
  if (!fileExists("/settings.json")) return;
  String content = readTextFile("/settings.json");
  // Parse JSON and apply settings
}

void saveUserPreferences() {
  saveSettingsToFile();
}

void loadUserPreferences() {
  loadSettingsFromFile();
}

// =============================================================================
// FILE TYPE DETECTION
// =============================================================================

FileType getFileType(const String& filename) {
  String lower = filename;
  lower.toLowerCase();
  
  if (lower.endsWith(".mp3") || lower.endsWith(".wav") || lower.endsWith(".flac")) return FILE_MP3;
  if (lower.endsWith(".pdf")) return FILE_PDF;
  if (lower.endsWith(".txt")) return FILE_TXT;
  if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) return FILE_JPG;
  if (lower.endsWith(".png")) return FILE_PNG;
  if (lower.endsWith(".bmp")) return FILE_BMP;
  if (lower.endsWith(".json") || lower.endsWith(".cfg")) return FILE_CONFIG;
  
  return FILE_UNKNOWN;
}

bool isAudioFile(const String& filename) {
  FileType type = getFileType(filename);
  return type == FILE_MP3;
}

bool isPDFFile(const String& filename) {
  return getFileType(filename) == FILE_PDF;
}

bool isImageFile(const String& filename) {
  FileType type = getFileType(filename);
  return type == FILE_JPG || type == FILE_PNG || type == FILE_BMP;
}

String getFileExtension(const String& filename) {
  int lastDot = filename.lastIndexOf('.');
  if (lastDot >= 0) return filename.substring(lastDot);
  return "";
}

String formatFileSize(size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  if (bytes < 1024 * 1024) return String(bytes / 1024) + " KB";
  return String(bytes / (1024 * 1024)) + " MB";
}

void drawFileBrowser(const char* current_path) {
  // Draw file browser UI
}

void handleFileBrowserNavigation(TouchGesture& gesture, const char* current_path) {
  // Handle navigation
}
