#include "wifi_funcs.h"

static Preferences preferences;

// SoftAP and OTA server instances
static WebServer* ota_server = nullptr;
static bool softap_active = false;
static bool ota_server_active = false;
static uint8_t ota_progress = 0;

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
    const int maxAttempts = 16; // 16 * 500ms = 8 seconds
    
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

// ============================================================================
// SoftAP Functions
// ============================================================================

// Start SoftAP mode for OTA updates
bool start_softap() {
    if (softap_active) {
        Serial.println("[SOFTAP] SoftAP already running");
        return true;
    }
    
    Serial.println("[SOFTAP] Starting SoftAP...");
    Serial.printf("[SOFTAP] SSID: %s\n", SOFTAP_SSID);
    Serial.printf("[SOFTAP] Password: %s\n", SOFTAP_PASSWORD);
    
    // Disconnect from any existing WiFi connection
    WiFi.disconnect();
    delay(100);
    
    // Set WiFi mode to AP (Access Point)
    if (!WiFi.mode(WIFI_AP)) {
        Serial.println("[SOFTAP] ERROR: Failed to set WiFi mode to AP");
        return false;
    }
    
    // Configure SoftAP IP address
    if (!WiFi.softAPConfig(SOFTAP_IP, SOFTAP_GATEWAY, SOFTAP_SUBNET)) {
        Serial.println("[SOFTAP] ERROR: Failed to configure SoftAP IP");
        return false;
    }
    
    // Start SoftAP
    if (!WiFi.softAP(SOFTAP_SSID, SOFTAP_PASSWORD)) {
        Serial.println("[SOFTAP] ERROR: Failed to start SoftAP");
        return false;
    }
    
    softap_active = true;
    
    Serial.println("[SOFTAP] ✓ SoftAP started successfully");
    Serial.print("[SOFTAP] IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("[SOFTAP] MAC Address: ");
    Serial.println(WiFi.softAPmacAddress());
    
    return true;
}

// Stop SoftAP mode
void stop_softap() {
    if (!softap_active) {
        return;
    }
    
    Serial.println("[SOFTAP] Stopping SoftAP...");
    
    // Stop OTA server first if running
    if (ota_server_active) {
        stop_ota_server();
    }
    
    // Stop SoftAP
    WiFi.softAPdisconnect(true);
    delay(100);
    
    softap_active = false;
    Serial.println("[SOFTAP] SoftAP stopped");
}

// Check if SoftAP is running
bool is_softap_running() {
    return softap_active && (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA);
}

// Get SoftAP IP address
IPAddress get_softap_ip() {
    if (softap_active) {
        return WiFi.softAPIP();
    }
    return IPAddress(0, 0, 0, 0);
}

// ============================================================================
// OTA Server Functions
// ============================================================================

// Handle root request - show OTA status page
void handle_ota_root() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    html += "<title>GVOLTA OTA Update</title>";
    html += "<style>body{font-family:Arial;text-align:center;margin:50px;}";
    html += "h1{color:#4A90E2;} .status{color:#00AA00;font-size:18px;margin:20px;}";
    html += ".progress{width:300px;height:30px;border:2px solid #333;margin:20px auto;}";
    html += ".progress-bar{height:100%;background:#4A90E2;width:" + String(ota_progress) + "%;}";
    html += "</style></head><body>";
    html += "<h1>GVOLTA 3kW Charger</h1>";
    html += "<h2>OTA Firmware Update</h2>";
    html += "<div class='status'>Status: Ready for update</div>";
    html += "<div class='progress'><div class='progress-bar'></div></div>";
    html += "<p>Progress: " + String(ota_progress) + "%</p>";
    html += "<p>Send POST request to /update with firmware binary</p>";
    html += "</body></html>";
    
    ota_server->send(200, "text/html", html);
}

// Handle OTA update request
void handle_ota_update() {
    HTTPUpload& upload = ota_server->upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Update start: %s\n", upload.filename.c_str());
        
        // Check if we have enough space
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        
        if (!Update.begin(maxSketchSpace, U_FLASH)) {
            Update.printError(Serial);
            ota_server->send(500, "text/plain", "Not enough space");
            return;
        }
        
        ota_progress = 0;
        Serial.printf("[OTA] Max sketch space: %u bytes\n", maxSketchSpace);
        
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        // Write received data to flash
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
            ota_server->send(500, "text/plain", "Write failed");
            return;
        }
        
        // Calculate and update progress
        ota_progress = (upload.totalSize > 0) ? (uint8_t)((upload.currentSize * 100) / upload.totalSize) : 0;
        
        Serial.printf("[OTA] Progress: %u%% (%u / %u bytes)\n", 
                     ota_progress, upload.currentSize, upload.totalSize);
        
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("[OTA] Update Success: %u bytes\nRebooting...\n", upload.totalSize);
            ota_server->send(200, "text/plain", "Update successful! Device will reboot.");
            delay(1000);
            ESP.restart();
        } else {
            Update.printError(Serial);
            ota_server->send(500, "text/plain", "Update failed");
        }
        
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        Serial.println("[OTA] Update aborted");
        ota_server->send(500, "text/plain", "Update aborted");
        ota_progress = 0;
    }
}

// Handle OTA update finish
void handle_ota_finish() {
    if (Update.end(true)) {
        Serial.println("[OTA] Update finished successfully");
        ota_server->send(200, "text/plain", "Update successful! Device will reboot.");
        delay(1000);
        ESP.restart();
    } else {
        Update.printError(Serial);
        ota_server->send(500, "text/plain", "Update failed");
    }
}

// Start OTA update server
bool start_ota_server() {
    if (ota_server_active && ota_server != nullptr) {
        Serial.println("[OTA] OTA server already running");
        return true;
    }
    
    // Ensure SoftAP is running first
    if (!softap_active) {
        Serial.println("[OTA] Starting SoftAP for OTA...");
        if (!start_softap()) {
            Serial.println("[OTA] ERROR: Failed to start SoftAP");
            return false;
        }
    }
    
    Serial.println("[OTA] Starting OTA server...");
    
    // Create web server instance
    if (ota_server == nullptr) {
        ota_server = new WebServer(OTA_PORT);
    }
    
    // Setup routes
    ota_server->on("/", HTTP_GET, handle_ota_root);
    ota_server->on(OTA_PATH, HTTP_POST, handle_ota_finish, handle_ota_update);
    
    // Handle 404
    ota_server->onNotFound([]() {
        ota_server->send(404, "text/plain", "Not found");
    });
    
    // Start server
    ota_server->begin();
    ota_server_active = true;
    
    Serial.println("[OTA] ✓ OTA server started successfully");
    Serial.printf("[OTA] Server URL: http://%s%s\n", WiFi.softAPIP().toString().c_str(), OTA_PATH);
    Serial.println("[OTA] Ready to receive firmware updates");
    
    return true;
}

// Stop OTA update server
void stop_ota_server() {
    if (!ota_server_active) {
        return;
    }
    
    Serial.println("[OTA] Stopping OTA server...");
    
    if (ota_server != nullptr) {
        ota_server->stop();
        delete ota_server;
        ota_server = nullptr;
    }
    
    ota_server_active = false;
    ota_progress = 0;
    Serial.println("[OTA] OTA server stopped");
}

// Handle OTA server requests (call this in loop)
void handle_ota_requests() {
    if (ota_server_active && ota_server != nullptr) {
        ota_server->handleClient();
    }
}

// Check if OTA server is running
bool is_ota_server_running() {
    return ota_server_active && ota_server != nullptr;
}

// Get OTA update progress (0-100)
uint8_t get_ota_progress() {
    return ota_progress;
}

// Trigger OTA mode: disconnect WiFi, wait 1s, start SoftAP and OTA server
void trigger_ota_mode() {
    Serial.println("[OTA_TRIGGER] ========================================");
    Serial.println("[OTA_TRIGGER] Starting OTA mode sequence...");
    
    // Step 1: Disconnect WiFi if connected
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[OTA_TRIGGER] Disconnecting from WiFi...");
        WiFi.disconnect();
        delay(100);
        Serial.println("[OTA_TRIGGER] WiFi disconnected");
    } else {
        Serial.println("[OTA_TRIGGER] WiFi already disconnected");
    }
    
    // Step 2: Wait 1 second as requested
    Serial.println("[OTA_TRIGGER] Waiting 1 second...");
    delay(1000);
    
    // Step 3: Start SoftAP
    Serial.println("[OTA_TRIGGER] Starting SoftAP...");
    if (start_softap()) {
        Serial.println("[OTA_TRIGGER] SoftAP started successfully");
        
        // Step 4: Start OTA server
        Serial.println("[OTA_TRIGGER] Starting OTA server...");
        if (start_ota_server()) {
            Serial.println("[OTA_TRIGGER] ✓ OTA mode activated successfully!");
            Serial.printf("[OTA_TRIGGER] Connect to: %s\n", SOFTAP_SSID);
            Serial.printf("[OTA_TRIGGER] Password: %s\n", SOFTAP_PASSWORD);
            Serial.printf("[OTA_TRIGGER] Upload URL: http://%s%s\n", 
                         WiFi.softAPIP().toString().c_str(), OTA_PATH);
        } else {
            Serial.println("[OTA_TRIGGER] ✗ Failed to start OTA server");
        }
    } else {
        Serial.println("[OTA_TRIGGER] ✗ Failed to start SoftAP");
    }
    
    Serial.println("[OTA_TRIGGER] ========================================");
}

