#ifndef BLE_H
#define BLE_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Arduino.h>

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define NETWORK_LIST_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define NETWORK_INFO_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// BLE Configuration
#define MAX_CHUNK_SIZE 20
#define MAX_NETWORKS 10
#define BUFFER_SIZE 128

// BLE Device Name
#define BLE_DEVICE_NAME "GVOLTA-Charger"

// Global BLE credentials (set by BLE callback)
extern String ble_ssid;
extern String ble_key;

class BLEManager {
private:
    BLEServer* pServer;
    BLECharacteristic* pNetworkListCharacteristic;
    BLECharacteristic* pNetworkInfoCharacteristic;
    bool deviceConnected;
    bool oldDeviceConnected;
    bool scanRequested;
    char buffer[BUFFER_SIZE];

    // Internal callback classes
    class MyServerCallbacks : public BLEServerCallbacks {
        BLEManager* manager;
    public:
        MyServerCallbacks(BLEManager* m) : manager(m) {}
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    };

    class NetworkListCallbacks : public BLECharacteristicCallbacks {
        BLEManager* manager;
    public:
        NetworkListCallbacks(BLEManager* m) : manager(m) {}
        void onWrite(BLECharacteristic *pCharacteristic);
    };

    class NetworkInfoCallbacks : public BLECharacteristicCallbacks {
        BLEManager* manager;
    public:
        NetworkInfoCallbacks(BLEManager* m) : manager(m) {}
        void onWrite(BLECharacteristic *pCharacteristic);
    };

    // Internal methods
    void performWiFiScan();
    void sendScanResults(int n);
    void connectToWiFi(const char* ssid, const char* password); // Not used - kept for compatibility

public:
    BLEManager();
    void init();
    void process();  // Call this in loop() for non-blocking processing
    bool isConnected() { return deviceConnected; }
};

#endif // BLE_H

