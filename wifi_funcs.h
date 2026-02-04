#ifndef WIFI_FUNCS_H
#define WIFI_FUNCS_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

// WiFi credentials storage keys
#define PREF_NAMESPACE "wifi_config"
#define PREF_KEY_SSID "ssid"
#define PREF_KEY_PASSWORD "password"

// Function declarations
void init_wifi_preferences();
bool save_wifi_credentials(const char* ssid, const char* password);
bool load_wifi_credentials(String& ssid, String& password);
bool has_wifi_credentials();
void clear_wifi_credentials();
bool connect_to_wifi();
void connect_to_wifi_async(const char* ssid, const char* password);

#endif // WIFI_FUNCS_H

