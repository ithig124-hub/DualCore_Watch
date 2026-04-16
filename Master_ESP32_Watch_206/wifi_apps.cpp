/*
 * wifi_apps.cpp - WiFi & Internet Implementation (FIXED)
 */

#include "wifi_apps.h"
#include "wifi_sync.h"
#include "config.h"
#include "display.h"
#include "themes.h"
#include "hardware.h"
#include "navigation.h"

extern Arduino_CO5300 *gfx;
extern SystemState system_state;

// =============================================================================
// HARDCODED WIFI CREDENTIALS - CONFIGURED FOR YOUR NETWORK
// =============================================================================
const char* AUTO_WIFI_SSID = "Optus_9D2E3D";
const char* AUTO_WIFI_PASSWORD = "snucktemptGLeQU";
bool wifi_auto_connected = false;

// WiFi state
 WiFiState wifi_state = WIFI_DISCONNECTED;
static WiFiConfig wifi_config;
static NetworkInfo available_networks[20];
static int network_count = 0;
static int selected_network = -1;
static String password_input = "";

// =============================================================================
// MANUAL WIFI CONNECT - Status tracking
// =============================================================================
#define MANUAL_MAX_NETWORKS 10

enum ManualConnectStatus {
  MWIFI_WAITING = 0,
  MWIFI_TRYING,
  MWIFI_CONNECTED,
  MWIFI_FAILED
};

struct ManualNetworkEntry {
  char ssid[64];
  char password[64];
  bool isFromSD;
  bool valid;
  ManualConnectStatus status;
};

static ManualNetworkEntry manualNetworks[MANUAL_MAX_NETWORKS];
static int manualNetworkCount = 0;
static int manualScrollOffset = 0;
static bool manualConnectDone = false;
static bool manualConnectRunning = false;
static int manualConnectedIndex = -1;  // which network connected
static bool manualTimeSynced = false;
static bool manualWeatherFetched = false;
static bool manualTimezoneFetched = false;

// Weather data - struct order: location, description, icon, last_update, temperature, humidity, wind_speed, pressure, valid
static WeatherData cached_weather = {"Unknown", "Clear", "01d", "", 20.0f, 50.0f, 5.0f, 1013, false};

// News data
static NewsArticle cached_news[5];
static int news_count = 0;

// =============================================================================
// AUTO WIFI CONNECTION
// =============================================================================

void autoConnectWiFi() {
  if (wifi_auto_connected || WiFi.status() == WL_CONNECTED) return;
  
  Serial.println("[WiFi] Auto-connecting to configured network...");
  Serial.printf("[WiFi] SSID: %s\\n", AUTO_WIFI_SSID);
  
  WiFi.begin(AUTO_WIFI_SSID, AUTO_WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_auto_connected = true;
    wifi_state = WIFI_CONNECTED;
    system_state.wifi_connected = true;
    system_state.wifi_ssid = WiFi.SSID();
    system_state.wifi_signal_strength = WiFi.RSSI();
    
    Serial.println("\\n[WiFi] Auto-connected successfully!");
    Serial.printf("[WiFi] IP: %s\\n", WiFi.localIP().toString().c_str());
    
    // Auto-fetch weather on successful connection
    fetchWeatherData();
  } else {
    Serial.println("\\n[WiFi] Auto-connect failed");
  }
}

// =============================================================================
// WIFI MANAGER
// =============================================================================

void initWiFiManager() {
  Serial.println("[WiFi] Initializing WiFi manager...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  // Try auto-connect
  autoConnectWiFi();
}

// Alias function for initWiFiManager
void initWifiApps() {
  initWiFiManager();
}

void updateWiFiManager() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    updateWiFiStatus();
  }
}

void updateWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    wifi_state = WIFI_CONNECTED;
    system_state.wifi_connected = true;
    system_state.wifi_ssid = WiFi.SSID();
    system_state.wifi_signal_strength = WiFi.RSSI();
  } else {
    wifi_state = WIFI_DISCONNECTED;
    system_state.wifi_connected = false;
  }
}

bool connectWiFi(const String& ssid, const String& password) {
  Serial.printf("[WiFi] Connecting to %s...\n", ssid.c_str());
  wifi_state = WIFI_CONNECTING;
  
  WiFi.begin(ssid.c_str(), password.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected!");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    wifi_state = WIFI_CONNECTED;
    system_state.wifi_connected = true;
    system_state.wifi_ssid = ssid;
    return true;
  }
  
  Serial.println("[WiFi] Connection failed");
  wifi_state = WIFI_FAILED;
  return false;
}

bool connectSavedWiFi() {
  if (wifi_config.ssid.length() > 0) {
    return connectWiFi(wifi_config.ssid, wifi_config.password);
  }
  return false;
}


bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

WiFiState getWiFiState() {
  return wifi_state;
}

void startWiFiScan() {
  Serial.println("[WiFi] Starting scan...");
  wifi_state = WIFI_SCANNING;
  WiFi.scanNetworks(true);  // Async scan
}

bool isScanComplete() {
  int result = WiFi.scanComplete();
  return result >= 0;
}

int getAvailableNetworks(NetworkInfo* networks, int max_count) {
  int count = WiFi.scanComplete();
  if (count < 0) return 0;
  
  int n = min(count, max_count);
  for (int i = 0; i < n; i++) {
    networks[i].ssid = WiFi.SSID(i);
    networks[i].rssi = WiFi.RSSI(i);
    networks[i].encrypted = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }
  
  WiFi.scanDelete();
  return n;
}

int getSignalStrength() {
  return WiFi.RSSI();
}

String getSignalQuality() {
  int rssi = WiFi.RSSI();
  if (rssi > -50) return "Excellent";
  if (rssi > -60) return "Good";
  if (rssi > -70) return "Fair";
  return "Weak";
}

String getLocalIP() {
  return WiFi.localIP().toString();
}

String getMacAddress() {
  return WiFi.macAddress();
}

// =============================================================================
// WIFI SCREENS
// =============================================================================

void initWiFiSetupScreen() {
  system_state.current_screen = SCREEN_WIFI_SETUP;
  startWiFiScan();
}

void showWiFiSetupScreen() {
  drawNetworkListScreen();
}

void showNetworkScanScreen() {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(80, 200);
  gfx->print("Scanning...");
}

void showNetworkStatusScreen() {
  gfx->fillScreen(COLOR_BLACK);
  
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(60, 20);
  gfx->print("NETWORK STATUS");
  
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(1);
  
  gfx->setCursor(20, 80);
  gfx->printf("Status: %s", isWiFiConnected() ? "Connected" : "Disconnected");
  
  if (isWiFiConnected()) {
    gfx->setCursor(20, 110);
    gfx->printf("SSID: %s", system_state.wifi_ssid.c_str());
    gfx->setCursor(20, 140);
    gfx->printf("IP: %s", getLocalIP().c_str());
    gfx->setCursor(20, 170);
    gfx->printf("Signal: %s (%d dBm)", getSignalQuality().c_str(), getSignalStrength());
    gfx->setCursor(20, 200);
    gfx->printf("MAC: %s", getMacAddress().c_str());
  }
  
  drawSwipeIndicator();
}

void handleWiFiSetupTouch(TouchGesture& gesture) {
  // Swipe up to exit handled by main gesture handler
  // Taps handled by handleNetworkListTouch
}

// Alias function for handleWiFiSetupTouch
void handleWifiManagerTouch(TouchGesture& gesture) {
  // If manual connect results screen is active, handle its touch events
  if (manualConnectDone && !manualConnectRunning) {
    handleManualConnectTouch(gesture);
    return;
  }
  
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  // Network selection
  for (int i = 0; i < min(network_count, 6); i++) {
    int ny = 70 + i * 50;
    if (y >= ny && y < ny + 45) {
      selected_network = i;
      if (!available_networks[i].encrypted) {
        connectWiFi(available_networks[i].ssid, "");
      }
      return;
    }
  }
  
  // NEW: Connect button - launches Manual WiFi Connect
  int connectBtnY = LCD_HEIGHT - 130;
  if (y >= connectBtnY && y < connectBtnY + 50) {
    Serial.println("[WiFi] Connect button tapped - starting Manual WiFi Connect...");
    runManualWiFiConnect();
    return;
  }
  
  // Scan button at bottom
  if (y >= 380 && y < 420) {
    startWiFiScan();
    drawNetworkListScreen();
  }
}

void drawNetworkListScreen() {
  // Reset manual connect state when entering normal WiFi screen
  manualConnectDone = false;
  manualConnectRunning = false;
  
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  uint16_t wifiBlue = RGB565(80, 180, 255);
  
  // Header - taller for 410x502
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, wifiBlue);
  }
  gfx->setTextColor(wifiBlue);
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 36, 14);
  gfx->print("WiFi");
  
  if (wifi_state == WIFI_SCANNING) {
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 60, 220);
    gfx->print("Scanning...");
  } else {
    network_count = getAvailableNetworks(available_networks, 20);
    
    // Larger network items for 410x502
    int itemH = 65;
    int itemGap = 10;
    int startY = headerH + 12;
    
    for (int i = 0; i < min(network_count, 5); i++) {
      int y = startY + i * (itemH + itemGap);
      gfx->fillRect(20, y, LCD_WIDTH - 40, itemH, RGB565(12, 14, 20));
      gfx->drawRect(20, y, LCD_WIDTH - 40, itemH, RGB565(40, 45, 60));
      gfx->fillRect(20, y, 5, 5, wifiBlue);
      gfx->setTextColor(RGB565(200, 205, 220));
      gfx->setTextSize(2);
      gfx->setCursor(40, y + 12);
      gfx->print(available_networks[i].ssid);
      gfx->setTextColor(RGB565(80, 85, 100));
      gfx->setTextSize(1);
      gfx->setCursor(40, y + 40);
      gfx->printf("%d dBm %s", available_networks[i].rssi, 
                 available_networks[i].encrypted ? "Secured" : "Open");
    }
    
    if (network_count == 0) {
      gfx->setTextColor(RGB565(80, 85, 100));
      gfx->setTextSize(2);
      gfx->setCursor(LCD_WIDTH/2 - 90, 220);
      gfx->print("No networks found");
    }
  }
  
  // Connect button - NEW! Try all WiFi networks
  int connectBtnY = LCD_HEIGHT - 130;
  gfx->fillRect(LCD_WIDTH/2 - 85, connectBtnY, 170, 50, RGB565(50, 150, 50));  // Green button
  gfx->drawRect(LCD_WIDTH/2 - 85, connectBtnY, 170, 50, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 72, connectBtnY + 8);
  gfx->print("Connect All");
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 55, connectBtnY + 30);
  gfx->print("(HC + SD Card)");
  
  // Scan button - larger and positioned for taller display
  int btnY = LCD_HEIGHT - 65;
  gfx->fillRect(LCD_WIDTH/2 - 70, btnY, 140, 50, theme->primary);
  gfx->drawRect(LCD_WIDTH/2 - 70, btnY, 140, 50, COLOR_WHITE);
  gfx->fillRect(LCD_WIDTH/2 - 70, btnY, 5, 5, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 36, btnY + 12);
  gfx->print("Scan");
  
  drawSwipeIndicator();
}

void drawPasswordEntryScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 48, 14);
  gfx->print("PASSWORD");
  
  gfx->setTextColor(RGB565(130, 135, 150));
  gfx->setTextSize(1);
  gfx->setCursor(30, 65);
  gfx->printf("Network: %s", available_networks[selected_network].ssid.c_str());
  
  // Password field - retro terminal
  gfx->fillRect(20, 95, LCD_WIDTH - 40, 40, RGB565(8, 10, 14));
  gfx->drawRect(20, 95, LCD_WIDTH - 40, 40, RGB565(40, 45, 60));
  gfx->fillRect(20, 95, 4, 4, theme->primary);
  gfx->setTextColor(RGB565(0, 200, 80));
  gfx->setTextSize(1);
  gfx->setCursor(35, 112);
  String masked = "";
  for (int i = 0; i < password_input.length(); i++) masked += "*";
  gfx->print(masked.length() > 0 ? masked : "> Tap to enter");
  
  // Connect button - retro
  gfx->fillRect(LCD_WIDTH/2 - 55, 380, 110, 40, theme->primary);
  gfx->drawRect(LCD_WIDTH/2 - 55, 380, 110, 40, COLOR_WHITE);
  gfx->fillRect(LCD_WIDTH/2 - 55, 380, 4, 4, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 36, 391);
  gfx->print("Connect");
  
  drawSwipeIndicator();
}

void handleNetworkListTouch(TouchGesture& gesture) {
  // Handled by handleWifiManagerTouch now
}

void handlePasswordEntryTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  if (gesture.y >= 380 && gesture.y < 420) {
    if (gesture.x < LCD_WIDTH/2) {
      // Connect
      if (selected_network >= 0) {
        connectWiFi(available_networks[selected_network].ssid, password_input);
      }
    }
  }
}

// =============================================================================
// WEATHER APP
// =============================================================================

void initWeatherApp() {
  system_state.current_screen = SCREEN_WEATHER_APP;
  if (isWiFiConnected()) {
    refreshWeatherData();
  }
}

void drawWeatherApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  uint16_t weatherBlue = RGB565(80, 180, 255);
  
  // Header - taller for 410x502
  int headerH = 55;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, weatherBlue);
  }
  gfx->setTextColor(weatherBlue);
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 63, 14);
  gfx->print("WEATHER");
  
  // Auto-refresh: try loading cached data from wifi_sync, or fetch live
  if (!cached_weather.valid) {
    refreshWeatherData();
  }
  
  if (!cached_weather.valid) {
    gfx->fillRect(30, 180, LCD_WIDTH - 60, 100, RGB565(12, 14, 20));
    gfx->drawRect(30, 180, LCD_WIDTH - 60, 100, RGB565(40, 45, 60));
    gfx->setTextColor(RGB565(100, 105, 120));
    gfx->setTextSize(3);
    gfx->setCursor(LCD_WIDTH/2 - 80, 200);
    if (isWiFiConnected()) {
      gfx->print("Loading...");
    } else {
      gfx->print("No Data");
      gfx->setTextSize(2);
      gfx->setCursor(LCD_WIDTH/2 - 100, 245);
      gfx->print("Use WiFi Connect first");
    }
  } else {
    // Location - larger
    gfx->setTextColor(RGB565(130, 135, 150));
    gfx->setTextSize(2);
    gfx->setCursor(LCD_WIDTH/2 - 40, headerH + 15);
    gfx->print(cached_weather.location);
    
    // Temperature card - larger retro
    int tempCardY = headerH + 50;
    gfx->fillRect(30, tempCardY, LCD_WIDTH - 60, 140, RGB565(12, 14, 20));
    gfx->drawRect(30, tempCardY, LCD_WIDTH - 60, 140, RGB565(40, 45, 60));
    gfx->fillRect(30, tempCardY, 8, 8, weatherBlue);
    gfx->fillRect(LCD_WIDTH - 38, tempCardY, 8, 8, weatherBlue);
    // CRT lines in temp area
    for (int sy = tempCardY + 2; sy < tempCardY + 138; sy += 4) {
      gfx->drawFastHLine(32, sy, LCD_WIDTH - 64, RGB565(6, 6, 10));
    }
    gfx->setTextColor(RGB565(220, 225, 240));
    gfx->setTextSize(9);
    gfx->setCursor(60, tempCardY + 25);
    gfx->printf("%.0f", cached_weather.temperature);
    gfx->setTextSize(3);
    gfx->setTextColor(theme->accent);
    gfx->print(" C");
    
    // Description - larger
    gfx->setTextColor(RGB565(150, 155, 170));
    gfx->setTextSize(3);
    int descLen = cached_weather.description.length() * 18;
    gfx->setCursor((LCD_WIDTH - descLen) / 2, tempCardY + 155);
    gfx->print(cached_weather.description);
    
    // Details cards - larger retro
    int detailY = tempCardY + 200;
    int halfW = (LCD_WIDTH - 50) / 2;
    int detailH = 70;
    
    gfx->fillRect(20, detailY, halfW, detailH, RGB565(12, 14, 20));
    gfx->drawRect(20, detailY, halfW, detailH, RGB565(40, 45, 60));
    gfx->fillRect(20, detailY, 5, 5, weatherBlue);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(35, detailY + 12);
    gfx->print("Humidity");
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(3);
    gfx->setCursor(35, detailY + 38);
    gfx->printf("%.0f%%", cached_weather.humidity);
    
    gfx->fillRect(LCD_WIDTH/2 + 5, detailY, halfW, detailH, RGB565(12, 14, 20));
    gfx->drawRect(LCD_WIDTH/2 + 5, detailY, halfW, detailH, RGB565(40, 45, 60));
    gfx->fillRect(LCD_WIDTH/2 + 5, detailY, 5, 5, weatherBlue);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(LCD_WIDTH/2 + 20, detailY + 12);
    gfx->print("Wind");
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(3);
    gfx->setCursor(LCD_WIDTH/2 + 20, detailY + 38);
    gfx->printf("%.1f", cached_weather.wind_speed);
  }
  
  // Refresh button - larger and positioned for taller display
  int btnY = LCD_HEIGHT - 70;
  gfx->fillRect(LCD_WIDTH/2 - 70, btnY, 140, 55, theme->primary);
  gfx->drawRect(LCD_WIDTH/2 - 70, btnY, 140, 55, COLOR_WHITE);
  gfx->fillRect(LCD_WIDTH/2 - 70, btnY, 5, 5, COLOR_WHITE);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(LCD_WIDTH/2 - 54, btnY + 15);
  gfx->print("Refresh");
  
  drawSwipeIndicator();
}

void handleWeatherTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int btnY = LCD_HEIGHT - 70;
  if (gesture.y >= btnY && gesture.y < btnY + 55) {
    refreshWeatherData();
    drawWeatherApp();
  }
}

void refreshWeatherData() {
  // If we have weather data from wifi_sync (fetched during manual connect), use it
  if (isWeatherValid()) {
    cached_weather.location = String(getDetectedCity());
    cached_weather.description = String(getWeatherDescription());
    cached_weather.icon = String(getWeatherIcon());
    cached_weather.temperature = getWeatherTemp();
    cached_weather.humidity = getWeatherHumidity();
    cached_weather.wind_speed = getWeatherWindSpeed();
    cached_weather.pressure = getWeatherPressure();
    cached_weather.valid = true;
    Serial.println("[Weather] Loaded real weather from wifi_sync cache");
    return;
  }
  
  // Fallback: try fetching live if WiFi connected
  if (!isWiFiConnected()) return;
  cached_weather = fetchWeatherData("");
}

WeatherData fetchWeatherData(const String& location) {
  // Check if we have real weather data from wifi_sync (Open-Meteo)
  if (isWeatherValid()) {
    WeatherData data;
    data.location = String(getDetectedCity());
    data.description = String(getWeatherDescription());
    data.icon = String(getWeatherIcon());
    data.last_update = "";
    data.temperature = getWeatherTemp();
    data.humidity = getWeatherHumidity();
    data.wind_speed = getWeatherWindSpeed();
    data.pressure = getWeatherPressure();
    data.valid = true;
    Serial.println("[Weather] Using real Open-Meteo weather data");
    return data;
  }
  
  // Fallback: try live fetch if connected
  if (isWiFiConnected()) {
    // Trigger a live fetch through wifi_sync
    extern bool fetchWeather();
    if (fetchWeather()) {
      WeatherData data;
      data.location = String(getDetectedCity());
      data.description = String(getWeatherDescription());
      data.icon = String(getWeatherIcon());
      data.last_update = "";
      data.temperature = getWeatherTemp();
      data.humidity = getWeatherHumidity();
      data.wind_speed = getWeatherWindSpeed();
      data.pressure = getWeatherPressure();
      data.valid = true;
      Serial.println("[Weather] Fetched live weather via Open-Meteo");
      return data;
    }
  }
  
  // No weather available
  WeatherData data = {"Unknown", "No data", "01d", "", 0.0f, 0.0f, 0.0f, 0, false};
  Serial.println("[Weather] No weather data available");
  return data;
}

// =============================================================================
// NEWS APP
// =============================================================================

static int current_article = 0;

void initNewsApp() {
  system_state.current_screen = SCREEN_NEWS_APP;
  if (isWiFiConnected()) {
    refreshNewsData();
  }
}

void drawNewsApp() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, theme->primary);
  }
  gfx->setTextColor(theme->primary);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 24, 14);
  gfx->print("NEWS");
  
  if (news_count == 0) {
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(100, 200);
    gfx->print(isWiFiConnected() ? "No news available" : "WiFi not connected");
  } else {
    NewsArticle& article = cached_news[current_article];
    
    // Article card - retro
    gfx->fillRect(15, 60, LCD_WIDTH - 30, 280, RGB565(12, 14, 20));
    gfx->drawRect(15, 60, LCD_WIDTH - 30, 280, RGB565(40, 45, 60));
    gfx->fillRect(15, 60, 5, 5, theme->primary);
    gfx->fillRect(LCD_WIDTH - 20, 60, 5, 5, theme->primary);
    
    gfx->setTextColor(RGB565(200, 205, 220));
    gfx->setTextSize(2);
    gfx->setCursor(25, 75);
    String title = article.title;
    if (title.length() > 25) title = title.substring(0, 22) + "...";
    gfx->print(title);
    
    gfx->setTextSize(1);
    gfx->setTextColor(theme->accent);
    gfx->setCursor(25, 105);
    gfx->print(article.source);
    
    gfx->setTextColor(RGB565(130, 135, 150));
    gfx->setCursor(25, 130);
    String summaryText = article.summary;
    if (summaryText.length() > 150) summaryText = summaryText.substring(0, 150);
    gfx->print(summaryText);
    
    // Navigation counter
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setCursor(150, 310);
    gfx->printf("%d/%d", current_article + 1, news_count);
  }
  
  drawThemeButton(30, 360, 80, 40, "Prev", false);
  drawSwipeIndicator();
  drawThemeButton(250, 360, 80, 40, "Next", false);
}

void handleNewsTouch(TouchGesture& gesture) {
  if (gesture.event != TOUCH_TAP) return;
  
  int x = gesture.x, y = gesture.y;
  
  if (y >= 360 && y < 400) {
    if (x < 110) {
      if (current_article > 0) current_article--;
    } else if (x < 220) {
      system_state.current_screen = SCREEN_APP_GRID;
    } else {
      if (current_article < news_count - 1) current_article++;
    }
  }
}

void refreshNewsData() {
  news_count = fetchNewsHeadlines(cached_news, 5);
}

int fetchNewsHeadlines(NewsArticle* articles, int max_count) {
  // FIXED: Initialize NewsArticle structs properly using member assignment
  // NewsArticle has: title, summary, source, url, time_ago, timestamp
  
  articles[0].title = "Tech Innovation Summit 2025";
  articles[0].summary = "Major announcements expected at annual tech conference";
  articles[0].source = "Tech News";
  articles[0].url = "";
  articles[0].time_ago = "2h ago";
  articles[0].timestamp = 0;
  
  articles[1].title = "Climate Action Progress";
  articles[1].summary = "Global leaders discuss environmental initiatives";
  articles[1].source = "World News";
  articles[1].url = "";
  articles[1].time_ago = "4h ago";
  articles[1].timestamp = 0;
  
  articles[2].title = "Sports Championship Finals";
  articles[2].summary = "Exciting match scheduled for this weekend";
  articles[2].source = "Sports";
  articles[2].url = "";
  articles[2].time_ago = "6h ago";
  articles[2].timestamp = 0;
  
  Serial.println("[News] Fetched news (demo)");
  return 3;
}

// =============================================================================
// UTILITIES
// =============================================================================

bool syncTimeWithNTP() {
  if (!isWiFiConnected()) return false;
  configTime(0, 0, "pool.ntp.org");
  return true;
}

bool checkInternetConnectivity() {
  return pingHost("google.com");
}

bool pingHost(const String& host) {
  if (!isWiFiConnected()) return false;
  // Simplified ping check
  return true;
}

void updateInternetApps() {
  updateWiFiManager();
}

void showNetworkTools() {
  drawNetworkDiagnostics();
}

void drawNetworkDiagnostics() {
  gfx->fillScreen(RGB565(2, 2, 5));
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  gfx->fillRect(0, 0, LCD_WIDTH, 48, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, 46, 6, 3, getCurrentTheme()->primary);
  }
  gfx->setTextColor(getCurrentTheme()->primary);
  gfx->setTextSize(2);
  gfx->setCursor(50, 14);
  gfx->print("NETWORK TOOLS");
  
  gfx->fillRect(20, 70, LCD_WIDTH - 40, 60, RGB565(12, 14, 20));
  gfx->drawRect(20, 70, LCD_WIDTH - 40, 60, RGB565(40, 45, 60));
  gfx->fillRect(20, 70, 4, 4, getCurrentTheme()->primary);
  
  gfx->setTextColor(RGB565(0, 200, 80));
  gfx->setTextSize(1);
  gfx->setCursor(30, 82);
  gfx->printf("> WiFi: %s", isWiFiConnected() ? "Connected" : "Disconnected");
  gfx->setCursor(30, 102);
  gfx->printf("> Internet: %s", checkInternetConnectivity() ? "Available" : "Unavailable");
}

void runSpeedTest() {
  Serial.println("[Network] Running speed test...");
  // Would download test file and measure speed
}


// =============================================================================
// MANUAL WIFI CONNECT - Try all networks (hardcoded + SD) with scrollable status
// =============================================================================

// External references from wifi_sync.cpp
extern WiFiSyncState wifi_sync_state;
extern void initWiFiSync();
extern int loadWiFiNetworks();
extern bool connectToWiFi(const char* ssid, const char* password, int timeout_ms);
extern bool fetchLocationFromIP();
extern bool syncNTPTime();
extern bool fetchWeather();
extern const char* getDetectedCity();
extern const char* getDetectedCountry();
extern int getTimezoneOffsetHours();
extern int getTimezoneOffsetMinutes();
extern void feedWatchdog();
extern bool saveTimeBackup();
extern void deleteTimeBackup();

// Draw helper: single network row on the manual connect screen
static void drawManualNetworkRow(int index, int drawY) {
  if (index < 0 || index >= manualNetworkCount) return;
  
  ManualNetworkEntry& net = manualNetworks[index];
  
  int rowH = 58;
  int rowW = LCD_WIDTH - 40;
  int rowX = 20;
  
  // Background based on status
  uint16_t bgColor = RGB565(12, 14, 20);     // default
  uint16_t borderColor = RGB565(40, 45, 60);  // default
  uint16_t statusColor = RGB565(100, 105, 120);
  const char* statusText = "Waiting";
  
  switch (net.status) {
    case MWIFI_WAITING:
      bgColor = RGB565(12, 14, 20);
      borderColor = RGB565(40, 45, 60);
      statusColor = RGB565(100, 105, 120);
      statusText = "Waiting";
      break;
    case MWIFI_TRYING:
      bgColor = RGB565(20, 20, 35);
      borderColor = RGB565(80, 130, 255);
      statusColor = RGB565(80, 180, 255);
      statusText = ".....";
      break;
    case MWIFI_CONNECTED:
      bgColor = RGB565(15, 35, 15);
      borderColor = RGB565(0, 200, 80);
      statusColor = RGB565(0, 255, 100);
      statusText = "Connected";
      break;
    case MWIFI_FAILED:
      bgColor = RGB565(30, 15, 15);
      borderColor = RGB565(200, 60, 60);
      statusColor = RGB565(255, 80, 80);
      statusText = "Cannot connect";
      break;
  }
  
  // Row background
  gfx->fillRect(rowX, drawY, rowW, rowH, bgColor);
  gfx->drawRect(rowX, drawY, rowW, rowH, borderColor);
  
  // Left accent bar
  gfx->fillRect(rowX, drawY + 2, 4, rowH - 4, borderColor);
  
  // Corner pixel accents
  gfx->fillRect(rowX, drawY, 5, 5, borderColor);
  gfx->fillRect(rowX + rowW - 5, drawY, 5, 5, borderColor);
  
  // Network number
  gfx->setTextColor(RGB565(80, 85, 100));
  gfx->setTextSize(1);
  gfx->setCursor(rowX + 12, drawY + 8);
  gfx->printf("#%d", index + 1);
  
  // Source tag (HC / SD)
  uint16_t tagColor = net.isFromSD ? RGB565(80, 180, 255) : RGB565(255, 180, 50);
  gfx->setTextColor(tagColor);
  gfx->setCursor(rowX + 38, drawY + 8);
  gfx->print(net.isFromSD ? "[SD]" : "[HC]");
  
  // SSID name - larger
  gfx->setTextColor(RGB565(220, 225, 240));
  gfx->setTextSize(2);
  // Truncate long SSIDs
  char truncSSID[20];
  strncpy(truncSSID, net.ssid, 18);
  truncSSID[18] = '\0';
  if (strlen(net.ssid) > 18) {
    truncSSID[17] = '.';
    truncSSID[16] = '.';
  }
  gfx->setCursor(rowX + 12, drawY + 24);
  gfx->print(truncSSID);
  
  // Status text - right aligned
  gfx->setTextColor(statusColor);
  gfx->setTextSize(2);
  int statusLen = strlen(statusText) * 12;
  gfx->setCursor(rowX + rowW - statusLen - 10, drawY + 24);
  gfx->print(statusText);
}

// Draw the full manual connect screen
void drawManualConnectScreen() {
  gfx->fillScreen(RGB565(2, 2, 5));
  // CRT scanlines
  for (int y = 0; y < LCD_HEIGHT; y += 4) {
    gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(4, 4, 7));
  }
  
  ThemeColors* theme = getCurrentTheme();
  uint16_t wifiBlue = RGB565(80, 180, 255);
  
  // Header
  int headerH = 50;
  gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
  for (int x = 0; x < LCD_WIDTH; x += 8) {
    gfx->fillRect(x, headerH - 3, 6, 3, wifiBlue);
  }
  gfx->setTextColor(wifiBlue);
  gfx->setTextSize(2);
  gfx->setCursor(LCD_WIDTH/2 - 72, 8);
  gfx->print("WiFi Connect");
  
  // Network count subtitle
  gfx->setTextColor(RGB565(100, 105, 120));
  gfx->setTextSize(1);
  gfx->setCursor(LCD_WIDTH/2 - 50, 30);
  gfx->printf("%d networks loaded", manualNetworkCount);
  
  // Network list area
  int listStartY = headerH + 8;
  int rowH = 58;
  int rowGap = 6;
  int maxVisible = 6;  // How many rows fit on screen
  
  for (int i = 0; i < maxVisible; i++) {
    int netIdx = i + manualScrollOffset;
    if (netIdx >= manualNetworkCount) break;
    
    int drawY = listStartY + i * (rowH + rowGap);
    drawManualNetworkRow(netIdx, drawY);
  }
  
  // Scroll indicators
  if (manualNetworkCount > maxVisible) {
    gfx->setTextColor(RGB565(60, 65, 80));
    gfx->setTextSize(1);
    if (manualScrollOffset > 0) {
      gfx->setCursor(LCD_WIDTH/2 - 10, listStartY - 6);
      gfx->print("^^");
    }
    int bottomY = listStartY + maxVisible * (rowH + rowGap);
    if (manualScrollOffset + maxVisible < manualNetworkCount) {
      gfx->setCursor(LCD_WIDTH/2 - 10, bottomY);
      gfx->print("vv");
    }
  }
  
  // Bottom status area - sync results (only shown when done)
  if (manualConnectDone) {
    int statusY = LCD_HEIGHT - 95;
    gfx->fillRect(15, statusY, LCD_WIDTH - 30, 80, RGB565(10, 12, 18));
    gfx->drawRect(15, statusY, LCD_WIDTH - 30, 80, RGB565(40, 45, 60));
    
    gfx->setTextSize(1);
    
    if (manualConnectedIndex >= 0) {
      // Show sync results
      gfx->setTextColor(RGB565(0, 200, 80));
      gfx->setCursor(25, statusY + 8);
      gfx->printf("Connected: %s", manualNetworks[manualConnectedIndex].ssid);
      
      gfx->setTextColor(manualTimezoneFetched ? RGB565(0, 200, 80) : RGB565(200, 60, 60));
      gfx->setCursor(25, statusY + 24);
      gfx->printf("Timezone: %s", manualTimezoneFetched ? "Synced" : "Failed");
      if (manualTimezoneFetched) {
        gfx->printf(" (GMT%+d)", getTimezoneOffsetHours());
      }
      
      gfx->setTextColor(manualTimeSynced ? RGB565(0, 200, 80) : RGB565(200, 60, 60));
      gfx->setCursor(25, statusY + 40);
      gfx->printf("Time:     %s", manualTimeSynced ? "Synced" : "Failed");
      
      gfx->setTextColor(manualWeatherFetched ? RGB565(0, 200, 80) : RGB565(200, 60, 60));
      gfx->setCursor(25, statusY + 56);
      gfx->printf("Weather:  %s", manualWeatherFetched ? "Fetched" : "Failed");
      if (manualTimezoneFetched) {
        gfx->printf(" (%s, %s)", getDetectedCity(), getDetectedCountry());
      }
    } else {
      // No connection
      gfx->setTextColor(RGB565(200, 60, 60));
      gfx->setCursor(25, statusY + 15);
      gfx->print("No network connected");
      gfx->setTextColor(RGB565(130, 135, 150));
      gfx->setCursor(25, statusY + 35);
      gfx->print("Check WiFi credentials & signal");
      gfx->setCursor(25, statusY + 55);
      gfx->print("Edit SD:/WATCH/wifi/config.txt");
    }
  }
  
  // Back button at very bottom
  if (manualConnectDone || !manualConnectRunning) {
    int btnY = LCD_HEIGHT - 12;
    gfx->setTextColor(RGB565(80, 85, 100));
    gfx->setTextSize(1);
    gfx->setCursor(LCD_WIDTH/2 - 30, btnY);
    gfx->print("< BACK >");
  }
}

// Run the manual WiFi connect process (blocking with display updates)
void runManualWiFiConnect() {
  Serial.println("[ManualWiFi] === MANUAL WIFI CONNECT START ===");
  
  // Reset state
  manualNetworkCount = 0;
  manualScrollOffset = 0;
  manualConnectDone = false;
  manualConnectRunning = true;
  manualConnectedIndex = -1;
  manualTimeSynced = false;
  manualWeatherFetched = false;
  manualTimezoneFetched = false;
  
  // Load networks from wifi_sync system (hardcoded + SD card)
  // This calls loadWiFiNetworks() which loads hardcoded as slot 0, then SD card networks
  initWiFiSync();
  
  // Copy networks into our local tracking array
  for (int i = 0; i < wifi_sync_state.networks_loaded && i < MANUAL_MAX_NETWORKS; i++) {
    if (!wifi_sync_state.networks[i].valid) continue;
    
    ManualNetworkEntry& entry = manualNetworks[manualNetworkCount];
    strncpy(entry.ssid, wifi_sync_state.networks[i].ssid, 63);
    entry.ssid[63] = '\0';
    strncpy(entry.password, wifi_sync_state.networks[i].password, 63);
    entry.password[63] = '\0';
    entry.isFromSD = wifi_sync_state.networks[i].isFromSD;
    entry.valid = true;
    entry.status = MWIFI_WAITING;
    manualNetworkCount++;
  }
  
  Serial.printf("[ManualWiFi] Loaded %d networks (hardcoded + SD)\n", manualNetworkCount);
  
  if (manualNetworkCount == 0) {
    Serial.println("[ManualWiFi] No networks available!");
    manualConnectDone = true;
    manualConnectRunning = false;
    drawManualConnectScreen();
    return;
  }
  
  // Draw initial screen with all networks showing "Waiting"
  drawManualConnectScreen();
  delay(500);
  feedWatchdog();
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  feedWatchdog();
  
  // Try each network once
  bool connected = false;
  
  for (int i = 0; i < manualNetworkCount; i++) {
    if (connected) break;
    
    ManualNetworkEntry& net = manualNetworks[i];
    
    // Auto-scroll to show current network being tried
    int maxVisible = 6;
    if (i >= manualScrollOffset + maxVisible) {
      manualScrollOffset = i - maxVisible + 1;
    }
    if (i < manualScrollOffset) {
      manualScrollOffset = i;
    }
    
    // Update status to TRYING
    net.status = MWIFI_TRYING;
    drawManualConnectScreen();
    feedWatchdog();
    
    Serial.printf("[ManualWiFi] Trying [%d/%d]: %s (%s)\n", 
      i + 1, manualNetworkCount, net.ssid, net.isFromSD ? "SD" : "HC");
    
    // Attempt connection (4 second timeout per network)
    feedWatchdog();
    WiFi.disconnect();
    delay(50);
    feedWatchdog();
    WiFi.begin(net.ssid, net.password);
    feedWatchdog();
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) {
      delay(200);
      feedWatchdog();
    }
    feedWatchdog();
    
    if (WiFi.status() == WL_CONNECTED) {
      // SUCCESS
      net.status = MWIFI_CONNECTED;
      manualConnectedIndex = i;
      connected = true;
      
      Serial.printf("[ManualWiFi] CONNECTED to: %s (IP: %s)\n", 
        net.ssid, WiFi.localIP().toString().c_str());
      
      // Update system state
      wifi_state = WIFI_CONNECTED;
      system_state.wifi_connected = true;
      system_state.wifi_ssid = String(net.ssid);
      system_state.wifi_signal_strength = WiFi.RSSI();
      
      // Redraw to show connected
      drawManualConnectScreen();
      delay(300);
      feedWatchdog();
      
    } else {
      // FAILED
      net.status = MWIFI_FAILED;
      
      Serial.printf("[ManualWiFi] FAILED: %s\n", net.ssid);
      
      WiFi.disconnect();
      feedWatchdog();
      
      // Redraw to show failure
      drawManualConnectScreen();
      delay(200);
      feedWatchdog();
    }
  }
  
  // === POST-CONNECTION: Sync data if connected ===
  if (connected) {
    Serial.println("[ManualWiFi] === SYNCING DATA ===");
    
    // Step 1: Fetch timezone from IP geolocation
    Serial.println("[ManualWiFi] Step 1: Detecting timezone...");
    feedWatchdog();
    manualTimezoneFetched = fetchLocationFromIP();
    feedWatchdog();
    drawManualConnectScreen();  // Show progress
    
    // Step 2: Sync NTP time (uses detected timezone)
    Serial.println("[ManualWiFi] Step 2: Syncing NTP time...");
    feedWatchdog();
    manualTimeSynced = syncNTPTime();
    feedWatchdog();
    drawManualConnectScreen();  // Show progress
    
    // Step 3: Fetch weather
    Serial.println("[ManualWiFi] Step 3: Fetching weather...");
    feedWatchdog();
    manualWeatherFetched = fetchWeather();
    feedWatchdog();
    
    // Save time backup if NTP synced
    if (manualTimeSynced) {
      deleteTimeBackup();
      saveTimeBackup();
    }
    
    Serial.println("[ManualWiFi] === SYNC COMPLETE ===");
    Serial.printf("[ManualWiFi] Timezone: %s, Time: %s, Weather: %s\n",
      manualTimezoneFetched ? "OK" : "FAIL",
      manualTimeSynced ? "OK" : "FAIL",
      manualWeatherFetched ? "OK" : "FAIL");
    
    // Disconnect to save power
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifi_state = WIFI_DISCONNECTED;
    system_state.wifi_connected = false;
    Serial.println("[ManualWiFi] WiFi disconnected (power save)");
    
  } else {
    Serial.println("[ManualWiFi] === NO CONNECTION - ALL NETWORKS FAILED ===");
  }
  
  // Mark as done
  manualConnectDone = true;
  manualConnectRunning = false;
  
  // Final draw showing all results
  drawManualConnectScreen();
  
  Serial.println("[ManualWiFi] === MANUAL WIFI CONNECT END ===");
}

// Handle touch on the manual connect screen (scrolling + back)
void handleManualConnectTouch(TouchGesture& gesture) {
  // Only handle when connect process is done
  if (manualConnectRunning) return;
  
  if (gesture.event == TOUCH_SWIPE_UP) {
    int maxVisible = 6;
    if (manualScrollOffset + maxVisible < manualNetworkCount) {
      manualScrollOffset++;
      drawManualConnectScreen();
    }
    return;
  }
  
  if (gesture.event == TOUCH_SWIPE_DOWN) {
    if (manualScrollOffset > 0) {
      manualScrollOffset--;
      drawManualConnectScreen();
    }
    return;
  }
  
  // Any tap when done goes back to WiFi manager
  if (gesture.event == TOUCH_TAP && manualConnectDone) {
    system_state.current_screen = SCREEN_WIFI_MANAGER;
    drawNetworkListScreen();
    return;
  }
}
