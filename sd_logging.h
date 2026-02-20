#ifndef SD_LOGGING_H
#define SD_LOGGING_H

#include <Arduino.h>
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include "battery_types.h"
#include "screen_definitions.h"

//SPI sd card pins
#define SD_MOSI 11    // SD card master output slave input pin
#define SD_CLK  12    // SD card clock pin
#define SD_MISO 13    // SD card master input slave output pin
#define SD_SS -1      // SD card select pin (not used)
#define SD_CS 4       // SD card select pin (IO expander pin)

// Global SD logging initialization flag
extern bool sd_logging_initialized;

// ============================================================================
// Charge Logging Functions
// ============================================================================

// Initialize charge logging (check/create charge_log.dat file)
// Returns true if successful, false otherwise
bool initChargeLogging();

// Get next serial number from charge_log.dat file
// Returns next serial number (starts at 1 if file is empty)
uint32_t getNextSerialNumber();

// Log charge start event
// Writes: serial_num,timestamp,type,v,ah,tc,tv (no newline)
// Returns true if successful, false otherwise
bool logChargeStart(uint32_t serial, BatteryType* profile);

// Log charge complete/stop event
// Appends: ,max_voltage,max_current,total_time_ms,ah_final,charge_stop_reason\n
// Returns true if successful, false otherwise
bool logChargeComplete(float max_volt, float max_curr, unsigned long total_time, float ah, charge_stop_reason_t stop_reason);

// ============================================================================
// Screen Logging Functions (Simple logging without NTP/RTC)
// ============================================================================

// Simple screen logging function for screen 2
class ScreenLogger {
private:
    static const int MAX_LOG_ENTRIES = 50;
    String logEntries[MAX_LOG_ENTRIES];
    int currentIndex = 0;
    int entryCount = 0;

public:
    // Add a log entry with timestamp (using millis for simplicity)
    void log(String message) {
        unsigned long timestamp = millis() / 1000; // seconds since boot
        String logLine = "[" + String(timestamp) + "s] " + message;

        logEntries[currentIndex] = logLine;
        currentIndex = (currentIndex + 1) % MAX_LOG_ENTRIES;
        if (entryCount < MAX_LOG_ENTRIES) {
            entryCount++;
        }
    }

    // Get log entry at index (0 = newest, higher numbers = older)
    String getLogEntry(int index) {
        if (index >= entryCount) return "";
        int actualIndex = (currentIndex - 1 - index + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
        return logEntries[actualIndex];
    }

    // Get total number of entries
    int getEntryCount() {
        return entryCount;
    }

    // Clear all log entries
    void clear() {
        currentIndex = 0;
        entryCount = 0;
        for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
            logEntries[i] = "";
        }
    }
};

// Global screen logger instance (extern declaration)
extern ScreenLogger screenLogger;

#endif // SD_LOGGING_H
