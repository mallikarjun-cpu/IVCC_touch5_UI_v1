#include "ble.h"
#include "wifi_funcs.h"
#include <string.h>

// Global BLE credentials storage
String ble_ssid = "";
String ble_key = "";

// Server Callbacks Implementation
void BLEManager::MyServerCallbacks::onConnect(BLEServer* pServer) {
    manager->deviceConnected = true;
    Serial.println("[BLE] Client connected");
}

void BLEManager::MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    manager->deviceConnected = false;
    Serial.println("[BLE] Client disconnected");
}

// Network List Characteristic Callbacks Implementation
void BLEManager::NetworkListCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    Serial.print("[BLE] Write received on Network List: ");
    Serial.println(value);
    if (value.length() > 0 && value[0] == '1') {
        manager->scanRequested = true;
        Serial.println("[WiFi] Scan requested");
    } else {
        Serial.println("[BLE] WARNING: Network credentials should be sent to Network Info characteristic!");
    }
}

// Network Info Characteristic Callbacks Implementation
void BLEManager::NetworkInfoCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    Serial.println("[NETWORK_INFO] ========================================");
    Serial.print("[NETWORK_INFO] Received data: '");
    Serial.print(value);
    Serial.println("'");
    Serial.print("[NETWORK_INFO] Data length: ");
    Serial.print(value.length());
    Serial.println(" bytes");
    
    // Parse format: "SSID|password|CONNECT" (handles spaces around delimiters)
    int firstDelimiter = value.indexOf('|');
    if (firstDelimiter > 0) {
        String ssid = value.substring(0, firstDelimiter);
        ssid.trim(); // Remove leading/trailing spaces
        Serial.print("[NETWORK_INFO] Parsed SSID: '");
        Serial.print(ssid);
        Serial.println("'");
        
        int secondDelimiter = value.indexOf('|', firstDelimiter + 1);
        if (secondDelimiter > firstDelimiter) {
            String password = value.substring(firstDelimiter + 1, secondDelimiter);
            password.trim(); // Remove leading/trailing spaces
            Serial.print("[NETWORK_INFO] Parsed Password length: ");
            Serial.println(password.length());
            
            String command = value.substring(secondDelimiter + 1);
            command.trim(); // Remove leading/trailing spaces
            Serial.print("[NETWORK_INFO] Parsed Command: '");
            Serial.print(command);
            Serial.println("'");
            
            // Save SSID and password to global variables
            ble_ssid = ssid;
            ble_key = password;
            
            // Save to preferences for persistent storage
            if (save_wifi_credentials(ssid.c_str(), password.c_str())) {
                Serial.println("[NETWORK_INFO] Credentials saved to preferences");
            } else {
                Serial.println("[NETWORK_INFO] WARNING: Failed to save credentials to preferences");
            }
            
            // Print saved credentials
            Serial.println("[NETWORK_INFO] ========================================");
            Serial.println("[NETWORK_INFO] Credentials saved successfully!");
            Serial.print("[NETWORK_INFO] SSID: '");
            Serial.print(ble_ssid);
            Serial.println("'");
            Serial.print("[NETWORK_INFO] Password: '");
            Serial.print(ble_key);
            Serial.println("'");
            Serial.println("[NETWORK_INFO] ========================================");
        } else {
            Serial.println("[NETWORK_INFO] ERROR: Missing second delimiter. Expected format: 'SSID|password|CONNECT'");
        }
    } else {
        Serial.println("[NETWORK_INFO] ERROR: Missing delimiter. Expected format: 'SSID|password|CONNECT'");
    }
    
    Serial.println("[NETWORK_INFO] ========================================");
}

// BLEManager Constructor
BLEManager::BLEManager() 
    : pServer(NULL), 
      pNetworkListCharacteristic(NULL), 
      pNetworkInfoCharacteristic(NULL),
      deviceConnected(false), 
      oldDeviceConnected(false), 
      scanRequested(false) {
    memset(buffer, 0, BUFFER_SIZE);
}

// Initialize BLE
void BLEManager::init() {
    Serial.println("\n[BLE] Starting initialization...");
    
    // Check if WiFi is already connected before modifying it
    bool wifi_was_connected = (WiFi.status() == WL_CONNECTED);
    
    Serial.println("[WiFi] Setting mode to STA");
    WiFi.mode(WIFI_STA);
    
    // Only disconnect if WiFi wasn't already connected
    // This preserves auto-connections from preferences
    if (!wifi_was_connected) {
        WiFi.disconnect();
        delay(100);
        Serial.println("[WiFi] WiFi initialized (disconnected)");
    } else {
        Serial.println("[WiFi] WiFi initialized (preserving existing connection)");
    }

    Serial.println("[BLE] Initializing BLE device");
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(64);
    Serial.println("[BLE] MTU set to 64 bytes");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));
    Serial.println("[BLE] Server created");

    Serial.println("[BLE] Creating service and characteristics");
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // WiFi scan request/result characteristic
    pNetworkListCharacteristic = pService->createCharacteristic(
                      NETWORK_LIST_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
    pNetworkListCharacteristic->setCallbacks(new NetworkListCallbacks(this));
    pNetworkListCharacteristic->addDescriptor(new BLE2902());
    Serial.println("[BLE] WiFi scan characteristic created");
    
    // Network info reception characteristic
    pNetworkInfoCharacteristic = pService->createCharacteristic(
                                 NETWORK_INFO_CHAR_UUID,
                                 BLECharacteristic::PROPERTY_WRITE
                               );
    pNetworkInfoCharacteristic->setCallbacks(new NetworkInfoCallbacks(this));
    Serial.println("[BLE] Network info characteristic created");
    
    pService->start();
    Serial.println("[BLE] Service started");

    Serial.println("[BLE] Starting advertising");
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true); // add device name

    BLEDevice::startAdvertising();
    Serial.println("[BLE] Initialization complete. Waiting for BLE connection...");
}

// Process BLE events (call in loop)
void BLEManager::process() {
    // Handle disconnection
    if (!deviceConnected && oldDeviceConnected) {
        Serial.println("[BLE] Restarting advertising");
        delay(500);
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
    }
    
    // Handle connection
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Handle scan request
    if (scanRequested && deviceConnected) {
        scanRequested = false;
        performWiFiScan();
    }
}

// Perform WiFi scan
void BLEManager::performWiFiScan() {
    Serial.println("[WiFi] Starting network scan...");
    int n = WiFi.scanNetworks(false, true);
    Serial.print("[WiFi] Scan complete. Found ");
    Serial.print(n);
    Serial.println(" networks");
    
    if (n == 0) {
        Serial.println("[BLE] Sending empty result");
        const char* msg = "[]";
        pNetworkListCharacteristic->setValue((uint8_t*)msg, 2);
        pNetworkListCharacteristic->notify();
    } else {
        sendScanResults(n);
    }
    
    WiFi.scanDelete();
    Serial.println("[WiFi] Scan results cleared");
}

// Send scan results in chunks
void BLEManager::sendScanResults(int n) {
    int pos = 0;
    int maxNet = (n > MAX_NETWORKS) ? MAX_NETWORKS : n;
    Serial.print("[WiFi] Processing ");
    Serial.print(maxNet);
    Serial.println(" networks");
    
    pos = snprintf(buffer, BUFFER_SIZE, "[");
    
    for (int i = 0; i < maxNet && pos < BUFFER_SIZE - 50; ++i) {
        if (i > 0) pos += snprintf(buffer + pos, BUFFER_SIZE - pos, ",");
        
        const char* enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "o" : "e";
        String ssid = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);

        if(ssid == ""){
            continue;
        }
        
        Serial.print("[WiFi] ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(ssid);
        Serial.print(" (");
        Serial.print(rssi);
        Serial.print(" dBm, ");
        Serial.print(enc);
        Serial.println(")");
        
        pos += snprintf(buffer + pos, BUFFER_SIZE - pos, 
                       "\"%s\",\"%d\"", 
                       ssid.c_str(), rssi);
    }
    snprintf(buffer + pos, BUFFER_SIZE - pos, "]");
    
    int len = strlen(buffer);
    Serial.print("[BLE] Sending data (");
    Serial.print(len);
    Serial.println(" bytes)");
    
    int chunks = (len + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    Serial.print("[BLE] Sending in ");
    Serial.print(chunks);
    Serial.println(" chunks");

    Serial.println(buffer);
    
    for (int i = 0; i < len; i += MAX_CHUNK_SIZE) {
        int chunkLen = (len - i < MAX_CHUNK_SIZE) ? (len - i) : MAX_CHUNK_SIZE;
        pNetworkListCharacteristic->setValue((uint8_t*)(buffer + i), chunkLen);
        pNetworkListCharacteristic->notify();
        delay(50);
    }
    Serial.println("[BLE] Data sent successfully");
}

// Connect to WiFi network (removed - not required per user request)
void BLEManager::connectToWiFi(const char* ssid, const char* password) {
    // WiFi connection functionality removed - credentials are saved but not used
    Serial.println("[WiFi] Note: WiFi connection not implemented - credentials saved only");
}

