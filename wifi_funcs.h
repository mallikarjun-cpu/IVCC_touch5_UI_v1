#ifndef WIFI_FUNCS_H
#define WIFI_FUNCS_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <WebServer.h>
#include <Update.h>

// WiFi credentials storage keys
#define PREF_NAMESPACE "wifi_config"
#define PREF_KEY_SSID "ssid"
#define PREF_KEY_PASSWORD "password"

// SoftAP Configuration
#define SOFTAP_SSID "GVOLTA_3kw_SoftAP"
#define SOFTAP_PASSWORD "tiger123"
#define SOFTAP_IP IPAddress(192, 168, 4, 1)
#define SOFTAP_GATEWAY IPAddress(192, 168, 4, 1)
#define SOFTAP_SUBNET IPAddress(255, 255, 255, 0)

// OTA Configuration
#define OTA_PORT 80
#define OTA_PATH "/update"

// Function declarations
void init_wifi_preferences();
bool save_wifi_credentials(const char* ssid, const char* password);
bool load_wifi_credentials(String& ssid, String& password);
bool has_wifi_credentials();
void clear_wifi_credentials();
bool connect_to_wifi();
void connect_to_wifi_async(const char* ssid, const char* password);

// SoftAP Functions
bool start_softap();
void stop_softap();
bool is_softap_running();
IPAddress get_softap_ip();

// OTA Functions
bool start_ota_server();
void stop_ota_server();
void handle_ota_requests();
bool is_ota_server_running();
uint8_t get_ota_progress();
void trigger_ota_mode(); // Trigger OTA mode: disconnect WiFi, start SoftAP

#endif // WIFI_FUNCS_H

