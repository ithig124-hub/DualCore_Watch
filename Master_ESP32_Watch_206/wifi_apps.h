/*
 * wifi_apps.h - WiFi & Internet Applications
 * Consolidates: wifi_manager.h/cpp, wifi_screens.cpp, internet_apps.h/cpp
 */

#ifndef WIFI_APPS_H
#define WIFI_APPS_H

#include "types.h"  // Must include types.h for WiFiState, WeatherData, NewsArticle
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>

// Network info
struct NetworkInfo {
  String ssid;
  int rssi;
  bool encrypted;
};

// WiFi config
struct WiFiConfig {
  String ssid;
  String password;
  bool auto_connect;
  int connection_timeout;
};

// =============================================================================
// WIFI MANAGER
// =============================================================================

void initWiFiManager();
void initWifiApps();  // Alias for initWiFiManager()
void updateWiFiManager();
void updateWiFiStatus();

bool connectWiFi(const String& ssid, const String& password);
bool connectSavedWiFi();
void disconnectWiFi();
bool isWiFiConnected();
WiFiState getWiFiState();

void startWiFiScan();
bool isScanComplete();
int getAvailableNetworks(NetworkInfo* networks, int max_count);

int getSignalStrength();
String getSignalQuality();
String getLocalIP();
String getMacAddress();

// =============================================================================
// WIFI SCREENS
// =============================================================================

void initWiFiSetupScreen();
void showWiFiSetupScreen();
void showNetworkScanScreen();
void showNetworkStatusScreen();
void handleWiFiSetupTouch(TouchGesture& gesture);
void handleWifiManagerTouch(TouchGesture& gesture);  // Alias for WiFi setup touch

void drawNetworkListScreen();
void drawPasswordEntryScreen();
void handleNetworkListTouch(TouchGesture& gesture);
void handlePasswordEntryTouch(TouchGesture& gesture);

// =============================================================================
// MANUAL WIFI CONNECT - Try all networks with on-screen status
// =============================================================================
void runManualWiFiConnect();
void drawManualConnectScreen();
void handleManualConnectTouch(TouchGesture& gesture);

// =============================================================================
// INTERNET APPS
// =============================================================================

// Weather
void initWeatherApp();
void drawWeatherApp();
void handleWeatherTouch(TouchGesture& gesture);
void refreshWeatherData();
WeatherData fetchWeatherData(const String& location = "");

// News
void initNewsApp();
void drawNewsApp();
void handleNewsTouch(TouchGesture& gesture);
void refreshNewsData();
int fetchNewsHeadlines(NewsArticle* articles, int max_count);

// Utilities
bool syncTimeWithNTP();
bool checkInternetConnectivity();
bool pingHost(const String& host);
void updateInternetApps();

// Network diagnostics
void showNetworkTools();
void drawNetworkDiagnostics();
void runSpeedTest();

#endif // WIFI_APPS_H
