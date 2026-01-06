
#ifndef SD_LOGGING_H
#define SD_LOGGING_H

#include <Arduino.h>
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <time.h>

//SPI sd card pins
#define SD_MOSI 11    // SD card master output slave input pin
#define SD_CLK  12    // SD card clock pin
#define SD_MISO 13    // SD card master input slave output pin
#define SD_SS -1      // SD card select pin (not used)
#define SD_CS 4       // SD card select pin (IO expander pin)

// ============================================================================
// SD Card Logging Class
// ============================================================================
class sd_logging {
private:
    String baseFolder = "/esp32s3_touchLCD";
    String currentDateFolder = "";
    String currentChargeFile = "";
    bool isInitialized = false;

    // COMMENTED OUT: NTP/RTC functionality not implemented
    /*
    // Get current date and time from NTP (placeholder for RTC future implementation)
    String getCurrentDateTime() {
        time_t now = time(nullptr);
        if (now < 8 * 3600 * 2) {  // Check if NTP time is valid (> 1970)
            return "0000-00-00 00:00:00";  // Invalid time
        }

        // Add 9 hours for JST (Japan Standard Time = UTC + 9)
        now += (9 * 3600);

        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);

        char buffer[20];
        sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        return String(buffer);
    }
    */

    // COMMENTED OUT: NTP/RTC functionality not implemented
    /*
    // Get current date (YYYY-MM-DD format) in JST
    String getCurrentDate() {
        time_t now = time(nullptr);
        if (now < 8 * 3600 * 2) {  // Check if NTP time is valid (> 1970)
            return "0000-00-00";  // Invalid time
        }

        // Add 9 hours for JST (Japan Standard Time = UTC + 9)
        now += (9 * 3600);

        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);

        char buffer[11];
        sprintf(buffer, "%04d-%02d-%02d",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

        return String(buffer);
    }
    */

    // COMMENTED OUT: NTP/RTC functionality not implemented
    /*
    // Get current time (HH:MM:SS format) in JST
    String getCurrentTime() {
        time_t now = time(nullptr);
        if (now < 8 * 3600 * 2) {  // Check if NTP time is valid (> 1970)
            return "00:00:00";  // Invalid time
        }

        // Add 9 hours for JST (Japan Standard Time = UTC + 9)
        now += (9 * 3600);

        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);

        char buffer[9];
        sprintf(buffer, "%02d:%02d:%02d",
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        return String(buffer);
    }
    */

    // COMMENTED OUT: NTP/RTC functionality not implemented
    /*
    // Get timestamp for filename (YYYYMMDD_HHMMSS format) in JST
    String getTimestampForFilename() {
        time_t now = time(nullptr);
        if (now < 8 * 3600 * 2) {  // Check if NTP time is valid (> 1970)
            return "00000000_000000";  // Invalid time
        }

        // Add 9 hours for JST (Japan Standard Time = UTC + 9)
        now += (9 * 3600);

        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);

        char buffer[16];
        sprintf(buffer, "%04d%02d%02d_%02d%02d%02d",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        return String(buffer);
    }
    */

    // Create directory if it doesn't exist
    bool createDirectory(String path) {
        if (!SD.exists(path)) {
            if (SD.mkdir(path)) {
                Serial.println("Created directory: " + path);
                return true;
            } else {
                Serial.println("Failed to create directory: " + path);
                return false;
            }
        }
        return true;
    }

    // Create new charge file for current date with battery profile info (simplified without NTP/RTC)
    String createNewChargeFile(String batteryProfileInfo) {
        // Use simple date placeholder since NTP/RTC not implemented
        String currentDate = "0000-00-00";
        String dateFolder = baseFolder + "/" + currentDate;
        currentDateFolder = dateFolder;

        // Create date folder if needed
        if (!createDirectory(dateFolder)) {
            return "";
        }

        // Use millis timestamp for filename (simplified)
        unsigned long timestamp_ms = millis();
        String timestamp = String(timestamp_ms);
        String chargeFile = dateFolder + "/log_" + timestamp + ".csv";

        // Check if file already exists (unlikely but possible)
        int counter = 1;
        while (SD.exists(chargeFile) && counter < 100) {
            chargeFile = dateFolder + "/log_" + timestamp + "_" + String(counter) + ".csv";
            counter++;
        }

        currentChargeFile = chargeFile;

        // Create file with battery profile info and header
        File file = SD.open(chargeFile, FILE_WRITE);
        if (file) {
            // First row: Battery profile confirmation
            file.println("Confirmed Battery Profile: " + batteryProfileInfo);

            // Second row: Empty line for spacing
            file.println("");

            // Third row: CSV header
            file.println("entry_no,day,time,volt,cur,rpm,temp1");

            file.close();
            Serial.println("Created charge file: " + chargeFile);
            return chargeFile;
        } else {
            Serial.println("Failed to create charge file: " + chargeFile);
            return "";
        }
    }

public:
    // Constructor
    sd_logging() {
        isInitialized = false;
        currentDateFolder = "";
        currentChargeFile = "";
    }

    // Initialize SD card and base folder
    bool initialize() {
        if (!SD.exists(baseFolder)) {
            if (!SD.mkdir(baseFolder)) {
                Serial.println("Failed to create base folder: " + baseFolder);
                return false;
            }
            Serial.println("Created base folder: " + baseFolder);
        }

        isInitialized = true;
        Serial.println("SD logging initialized successfully");
        return true;
    }

    // Start new charge session (creates new CSV file with battery profile)
    bool startNewCharge(String batteryProfileInfo) {
        if (!isInitialized) {
            Serial.println("SD logging not initialized");
            return false;
        }

        currentChargeFile = createNewChargeFile(batteryProfileInfo);
        return (currentChargeFile != "");
    }

    // Get current charge filename (for debugging/logging)
    String getChargeFileName() {
        return currentChargeFile;
    }

    // Log data entry to current charge file
    bool logData(int entry_no, float volt, float cur, float freq, float temp1) {
        if (!isInitialized || currentChargeFile == "") {
            Serial.println("SD logging not ready - not initialized or no charge file");
            return false;
        }

        // Calculate RPM from frequency (freq * 20)
        int rpm = (int)(freq * 20.0f);

        // Use simple timestamp since NTP/RTC not implemented
        String currentDate = "0000-00-00";
        String currentTime = "00:00:00";

        // Prepare log line
        String logLine = String(entry_no) + "," +
                       currentDate + "," +
                       currentTime + "," +
                       String(volt, 2) + "," +
                       String(cur, 2) + "," +
                       String(rpm) + "," +
                       String(temp1, 1);

        // Append to file
        File file = SD.open(currentChargeFile, FILE_APPEND);
        if (file) {
            file.println(logLine);
            file.close();
            return true;
        } else {
            Serial.println("Failed to open file for logging: " + currentChargeFile);
            return false;
        }
    }

    // Get current status
    void getStatus() {
        Serial.println("SD Logging Status:");
        Serial.println("  Initialized: " + String(isInitialized ? "Yes" : "No"));
        Serial.println("  Base Folder: " + baseFolder);
        Serial.println("  Current Date Folder: " + currentDateFolder);
        Serial.println("  Current Charge File: " + currentChargeFile);
    }
};

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

// Global SD logger instance (extern declaration)
extern sd_logging sdLogger;

#endif // SD_LOGGING_H
