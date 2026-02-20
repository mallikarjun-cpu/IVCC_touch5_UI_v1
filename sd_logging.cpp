#include "sd_logging.h"
#include <SD.h>
#include "battery_types.h"
#include "screen_definitions.h"

// External time data from M2
extern struct time_from_m2 {
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t day_of_week;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} m2Time;

// Global SD logging initialization flag
bool sd_logging_initialized = false;

// Global screen logger instance
ScreenLogger screenLogger;

// Charge log file name (in root directory since mkdir() doesn't work with ESP32 SD library)
// Using shorter name to avoid FAT32 filename issues
// Note: Leading slash required for ESP32 SD library root directory files
#define CHARGE_LOG_FILE "/chglog.dat"

// Initialize charge logging (check/create charge_log.dat file)
bool initChargeLogging() {
    // Check if SD card is available (cardType should not be CARD_NONE)
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD_LOG] SD card not available, cannot initialize charge logging");
        return false;
    }

    Serial.printf("[SD_LOG] SD card detected: type=%d, size=%lluMB\n", cardType, SD.cardSize() / (1024 * 1024));

    // Try to open root directory to verify SD is accessible
    File root = SD.open("/");
    if (!root) {
        Serial.println("[SD_LOG] ERROR: Cannot open root directory");
        return false;
    }
    if (!root.isDirectory()) {
        Serial.println("[SD_LOG] ERROR: Root is not a directory");
        root.close();
        return false;
    }
    root.close();
    Serial.println("[SD_LOG] Root directory accessible");

    // Test if we can write to root directory at all
    Serial.println("[SD_LOG] Testing write capability to root directory...");
    File testRootFile = SD.open("/_test_write.tmp", FILE_WRITE);
    if (testRootFile) {
        testRootFile.print("test");
        testRootFile.close();
        if (SD.exists("/_test_write.tmp")) {
            Serial.println("[SD_LOG] Write test successful - can write to root");
            SD.remove("/_test_write.tmp");
        } else {
            Serial.println("[SD_LOG] WARNING: File created but doesn't exist after close");
        }
    } else {
        Serial.println("[SD_LOG] ERROR: Cannot write to root directory - SD card may be read-only");
        return false;
    }

    // Note: ESP32 SD library mkdir() doesn't work reliably, so we'll create file in root
    // File name includes directory name for organization: logs_3kw_p1_charge_log.dat
    Serial.println("[SD_LOG] Creating log file in root directory (mkdir not supported)");

    // Check if file exists
    bool fileExists = SD.exists(CHARGE_LOG_FILE);
    Serial.printf("[SD_LOG] File %s exists: %s\n", CHARGE_LOG_FILE, fileExists ? "YES" : "NO");

    // If file doesn't exist, create it
    if (!fileExists) {
        Serial.printf("[SD_LOG] Creating file %s...\n", CHARGE_LOG_FILE);
        
        // Try different approaches - maybe filename length or format is an issue
        File file = SD.open(CHARGE_LOG_FILE, FILE_WRITE);
        if (!file) {
            Serial.printf("[SD_LOG] ERROR: SD.open() failed for %s\n", CHARGE_LOG_FILE);
            
            // Try with simpler filename to test
            Serial.println("[SD_LOG] Testing with simpler filename...");
            File testFile = SD.open("/charge_log.dat", FILE_WRITE);
            if (testFile) {
                testFile.close();
                SD.remove("/charge_log.dat");
                Serial.println("[SD_LOG] Simple filename works - issue may be with filename format/length");
                Serial.printf("[SD_LOG] Original filename length: %d chars\n", strlen(CHARGE_LOG_FILE));
            } else {
                Serial.println("[SD_LOG] Even simple filename failed");
            }
            
            return false;
        }
        
        // File opened successfully, write empty string to ensure it's created
        size_t bytesWritten = file.print("");
        file.flush();
        file.close();
        
        Serial.printf("[SD_LOG] File created successfully (wrote %d bytes)\n", bytesWritten);
        
        // Verify file was created
        if (SD.exists(CHARGE_LOG_FILE)) {
            Serial.println("[SD_LOG] File creation verified");
            return true;
        } else {
            Serial.println("[SD_LOG] ERROR: File creation failed - file does not exist after close");
            return false;
        }
    }

    Serial.println("[SD_LOG] charge_log.dat file exists");
    return true;
}

// Get next serial number from charge_log.dat file
uint32_t getNextSerialNumber() {
    if (!sd_logging_initialized) {
        Serial.println("[SD_LOG] SD logging not initialized, returning serial 1");
        return 1;
    }

    File file = SD.open(CHARGE_LOG_FILE, FILE_READ);
    if (!file) {
        Serial.println("[SD_LOG] Failed to open charge_log.dat for reading, returning serial 1");
        return 1;
    }

    uint32_t maxSerial = 0;
    String line = "";
    
    // Read file line by line to find last complete line
    while (file.available()) {
        char c = file.read();
        if (c == '\n') {
            if (line.length() > 0) {
                // Try to extract serial number from line (first field before comma)
                int firstComma = line.indexOf(',');
                if (firstComma > 0) {
                    String serialStr = line.substring(0, firstComma);
                    uint32_t serial = serialStr.toInt();
                    if (serial > maxSerial) {
                        maxSerial = serial;
                    }
                }
            }
            line = "";
        } else if (c != '\r') {
            line += c;
        }
    }
    
    // Check last line if file doesn't end with newline
    if (line.length() > 0) {
        int firstComma = line.indexOf(',');
        if (firstComma > 0) {
            String serialStr = line.substring(0, firstComma);
            uint32_t serial = serialStr.toInt();
            if (serial > maxSerial) {
                maxSerial = serial;
            }
        }
    }

    file.close();
    
    uint32_t nextSerial = maxSerial + 1;
    Serial.printf("[SD_LOG] Last serial: %lu, Next serial: %lu\n", maxSerial, nextSerial);
    return nextSerial;
}

// Get timestamp string from m2Time struct
String getTimestampString() {
    char timestamp[20];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
            m2Time.year, m2Time.month, m2Time.date,
            m2Time.hour, m2Time.minute, m2Time.second);
    return String(timestamp);
}

// Get battery chemistry string
String getBatteryTypeString(BatteryType* profile) {
    if (!profile) return "UNK";
    
    BatteryChemistry chem = profile->getChemistry();
    if (chem == LITHIUM) return "Li";
    if (chem == LEAD_ACID) return "LA";
    if (chem == LIFEPO4) return "LFP";
    return "UNK";
}

// Get charge stop reason string
String getChargeStopReasonString(charge_stop_reason_t reason) {
    switch (reason) {
        case CHARGE_STOP_COMPLETE:
            return "COMPLETE";
        case CHARGE_STOP_EMERGENCY:
            return "EMERGENCY";
        case CHARGE_STOP_VOLTAGE_SATURATION:
            return "VOLT_SAT";
        case CHARGE_STOP_VOLTAGE_LIMIT_PRECHARGE:
            return "VOLT_LIMIT";
        case CHARGE_STOP_HIGH_TEMP:
            return "HIGH_TEMP";
        case CHARGE_STOP_NONE:
        default:
            return "UNKNOWN";
    }
}

// Log charge start event
bool logChargeStart(uint32_t serial, BatteryType* profile) {
    if (!sd_logging_initialized) {
        Serial.println("[SD_LOG] SD logging not initialized, cannot log charge start");
        return false;
    }

    if (!profile) {
        Serial.println("[SD_LOG] Battery profile is null, cannot log charge start");
        return false;
    }

    File file = SD.open(CHARGE_LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("[SD_LOG] Failed to open charge_log.dat for appending");
        return false;
    }

    // Format: serial_num,timestamp,type,v,ah,tc,tv
    String timestamp = getTimestampString();
    String type = getBatteryTypeString(profile);
    uint16_t v = profile->getRatedVoltage();
    uint16_t ah = profile->getRatedAh();
    float tc = profile->getConstCurrent();
    float tv = profile->getCutoffVoltage();

    // Write start data (no newline, no closing comma)
    file.print(serial);
    file.print(",");
    file.print(timestamp);
    file.print(",");
    file.print(type);
    file.print(",");
    file.print(v);
    file.print(",");
    file.print(ah);
    file.print(",");
    file.print(tc, 1);
    file.print(",");
    file.print(tv, 1);

    file.close();
    Serial.printf("[SD_LOG] Charge start logged: serial=%lu, type=%s, v=%d, ah=%d, tc=%.1f, tv=%.1f\n",
                  serial, type.c_str(), v, ah, tc, tv);
    return true;
}

// Log charge complete/stop event
bool logChargeComplete(float max_volt, float max_curr, unsigned long total_time, float ah, charge_stop_reason_t stop_reason) {
    if (!sd_logging_initialized) {
        Serial.println("[SD_LOG] SD logging not initialized, cannot log charge complete");
        return false;
    }

    File file = SD.open(CHARGE_LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("[SD_LOG] Failed to open charge_log.dat for appending");
        return false;
    }

    // Format: ,max_voltage,max_current,total_time_ms,ah_final,charge_stop_reason\n
    file.print(",");
    file.print(max_volt, 1);
    file.print(",");
    file.print(max_curr, 1);
    file.print(",");
    file.print(total_time);
    file.print(",");
    file.print(ah, 1);
    file.print(",");
    file.print(getChargeStopReasonString(stop_reason));
    file.print("\n");

    file.close();
    Serial.printf("[SD_LOG] Charge complete logged: max_volt=%.1f, max_curr=%.1f, time=%lu, ah=%.1f, reason=%s\n",
                  max_volt, max_curr, total_time, ah, getChargeStopReasonString(stop_reason).c_str());
    return true;
}

