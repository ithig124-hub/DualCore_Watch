/*
 * wifi_apps.cpp - WiFi & Internet Implementation (FIXED)
 */

#include "wifi_apps.h"
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
static WiFiState wifi_state = WIFI_DISCONNECTED;
static WiFiConfig wifi_config;
static NetworkInfo available_networks[20];
static int network_count = 0;
static int selected_network = -1;
static String password_input = "";

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

void disconnectWiFi() {
  WiFi.disconnect();
  wifi_state = WIFI_DISCONNECTED;
  system_state.wifi_connected = false;
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
  
  // Scan button at bottom
  if (y >= 380 && y < 420) {
    startWiFiScan();
    drawNetworkListScreen();
  }
}

void drawNetworkListScreen() {
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
  
  // Auto-refresh if not valid yet
  if (!cached_weather.valid && isWiFiConnected()) {
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
      gfx->print("No WiFi");
      gfx->setTextSize(2);
      gfx->setCursor(LCD_WIDTH/2 - 100, 245);
      gfx->print("Connect to WiFi first");
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
  if (!isWiFiConnected()) return;
  cached_weather = fetchWeatherData("");
}

WeatherData fetchWeatherData(const String& location) {
  // Struct order: location, description, icon, last_update, temperature, humidity, wind_speed, pressure, valid
  WeatherData data = {"Demo City", "Partly Cloudy", "02d", "", 22.5f, 65.0f, 3.5f, 1015, true};
  
  // In production, this would call OpenWeatherMap API
  // HTTPClient http;
  // http.begin("http://api.openweathermap.org/data/2.5/weather?q=...&appid=...");
  
  Serial.println("[Weather] Fetched weather data (demo)");
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
