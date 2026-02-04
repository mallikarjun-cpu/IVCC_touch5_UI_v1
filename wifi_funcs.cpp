#include "wifi_funcs.h"

static Preferences preferences;

// Initialize preferences
void init_wifi_preferences() {
    preferences.begin(PREF_NAMESPACE, false);  // false = read-write mode
    Serial.println("[WIFI_PREFS] Preferences initialized");
}

// Save WiFi credentials to preferences
bool save_wifi_credentials(const char* ssid, const char* password) {
    if (ssid == nullptr || password == nullptr) {
        Serial.println("[WIFI_PREFS] ERROR: Cannot save null credentials");
        return false;
    }
    
    if (strlen(ssid) == 0) {
        Serial.println("[WIFI_PREFS] ERROR: SSID is empty");
        return false;
    }
    
    bool success = true;
    success = preferences.putString(PREF_KEY_SSID, ssid) && success;
    success = preferences.putString(PREF_KEY_PASSWORD, password) && success;
    
    if (success) {
        Serial.println("[WIFI_PREFS] Credentials saved successfully");
        Serial.printf("[WIFI_PREFS] SSID: %s\n", ssid);
        Serial.printf("[WIFI_PREFS] Password length: %d\n", strlen(password));
    } else {
        Serial.println("[WIFI_PREFS] ERROR: Failed to save credentials");
    }
    
    return success;
}

// Load WiFi credentials from preferences
bool load_wifi_credentials(String& ssid, String& password) {
    ssid = preferences.getString(PREF_KEY_SSID, "");
    password = preferences.getString(PREF_KEY_PASSWORD, "");
    
    if (ssid.length() > 0) {
        Serial.println("[WIFI_PREFS] Credentials loaded from preferences");
        Serial.printf("[WIFI_PREFS] SSID: %s\n", ssid.c_str());
        Serial.printf("[WIFI_PREFS] Password length: %d\n", password.length());
        return true;
    } else {
        Serial.println("[WIFI_PREFS] No credentials found in preferences");
        return false;
    }
}

// Check if WiFi credentials exist in preferences
bool has_wifi_credentials() {
    String ssid = preferences.getString(PREF_KEY_SSID, "");
    return (ssid.length() > 0);
}

// Clear WiFi credentials from preferences
void clear_wifi_credentials() {
    preferences.remove(PREF_KEY_SSID);
    preferences.remove(PREF_KEY_PASSWORD);
    Serial.println("[WIFI_PREFS] Credentials cleared from preferences");
}

// Connect to WiFi using credentials from preferences
bool connect_to_wifi() {
    String ssid, password;
    
    if (!load_wifi_credentials(ssid, password)) {
        Serial.println("[WIFI] No credentials in preferences, skipping WiFi connection");
        return false;
    }
    
    Serial.println("[WIFI] Connecting to WiFi from preferences...");
    Serial.printf("[WIFI] SSID: %s\n", ssid.c_str());
    
    // Set WiFi mode to STA
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Begin connection
    WiFi.begin(ssid.c_str(), password.c_str());
    
    // Wait for connection with timeout (15 seconds)
    int attempts = 0;
    const int maxAttempts = 30; // 30 * 500ms = 15 seconds
    
    Serial.print("[WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    // Check connection status
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WIFI] ✓ CONNECTION SUCCESSFUL");
        Serial.print("[WIFI] IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WIFI] RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        return true;
    } else {
        Serial.println("[WIFI] ✗ CONNECTION FAILED");
        Serial.print("[WIFI] Status code: ");
        Serial.println(WiFi.status());
        return false;
    }
}

// Connect to WiFi asynchronously (non-blocking, for use in loop)
void connect_to_wifi_async(const char* ssid, const char* password) {
    if (ssid == nullptr || password == nullptr) {
        Serial.println("[WIFI] ERROR: Cannot connect with null credentials");
        return;
    }
    
    Serial.println("[WIFI] Starting asynchronous WiFi connection");
    Serial.printf("[WIFI] SSID: %s\n", ssid);
    
    // Set WiFi mode to STA
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Begin connection (non-blocking)
    WiFi.begin(ssid, password);
    Serial.println("[WIFI] WiFi.begin() called, connection in progress...");
}

