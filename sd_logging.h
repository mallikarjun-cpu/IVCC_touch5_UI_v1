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
// Charge log record (single charge cycle, filled at start and completion)
// ============================================================================
// Battery name is UTF-8 (e.g. Japanese). Log viewer: open file with encoding='utf-8'.
#define CHARGE_LOG_NAME_MAX 96  // UTF-8 bytes

// Log format (one line per charge, UTF-8): serial, start_ts, start_volt, start_temp3, "battery_name", v, ah, tc, tv, end_ts, end_volt, max_volt, max_curr, total_time_ms, ah_final, stop_reason, max_t1, max_t2, complete_flag (1=second part written; if power lost before complete, line has no newline and startup appends \n to close it)

typedef struct charge_log_record {
    uint32_t serial;
    float start_volt;
    float start_temp3_celsius;  // temp3 (e.g. room) at charge start
    char battery_name[CHARGE_LOG_NAME_MAX];
    uint16_t v;
    uint16_t ah;
    float tc;
    float tv;
    float end_volt;
    float max_volt;
    float max_curr;
    float max_t1_celsius;
    float max_t2_celsius;
    unsigned long total_time_ms;
    float ah_final;
    charge_stop_reason_t stop_reason;
} charge_log_record_t;

// Initialize charge logging (check/create charge_log.dat file)
// Returns true if successful, false otherwise
bool initChargeLogging();

// Get next serial number from charge_log.dat file
// Returns next serial number (starts at 1 if file is empty)
uint32_t getNextSerialNumber();

// Log charge start event (writes first part of line, no newline)
// Record must have: serial, start_volt, start_temp3_celsius, battery_name, v, ah, tc, tv set.
// CSV: serial,start_ts,start_volt,start_temp3,"battery_name",v,ah,tc,tv
bool logChargeStart(const charge_log_record_t* record);

// Log charge complete/stop event (appends rest of line + newline)
// Record must have: end_volt, max_volt, max_curr, max_t1_celsius, max_t2_celsius,
// total_time_ms, ah_final, stop_reason set. end_ts is taken at write time.
// Appends: ,end_ts,end_volt,max_volt,max_curr,total_time_ms,ah_final,stop_reason,max_t1,max_t2,complete_flag\n
// complete_flag=1 when second part written; on startup, if file doesn't end with \n, last line is repaired (append \n) so next log starts on new line.
bool logChargeComplete(const charge_log_record_t* record);

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
