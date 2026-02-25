#include "can_twai.h"
#include "screen_definitions.h"
#include <esp_log.h>

// Forward declarations for global structs
extern struct sensor_data {
    float volt;
    float curr;
    int32_t temp1, temp2, temp3, temp4;
} sensorData;

extern struct time_from_m2 {
    uint16_t year;
    uint8_t month, date, day_of_week, hour, minute, second;
} m2Time;

// Forward declaration for battery detection flag
extern bool battery_detected;

// Big-endian conversion functions
uint16_t bigEndianToUint16(uint8_t* data) {
    return (data[0] << 8) | data[1];
}

int16_t bigEndianToInt16(uint8_t* data) {
    uint16_t val = bigEndianToUint16(data);
    return (int16_t)val;
}

// Convert m2Time to "time of month" in seconds (day 1-31 + time of day). Used for M2 heartbeat.
uint32_t calc_timeofmonth(void) {
    uint32_t d = (uint32_t)(m2Time.date >= 1 && m2Time.date <= 31 ? m2Time.date : 1);
    uint32_t h = (uint32_t)(m2Time.hour <= 23 ? m2Time.hour : 0);
    uint32_t m = (uint32_t)(m2Time.minute <= 59 ? m2Time.minute : 0);
    uint32_t s = (uint32_t)(m2Time.second <= 59 ? m2Time.second : 0);
    return (d * 86400UL) + (h * 3600UL) + (m * 60UL) + s;
}

// CAN configuration
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

// Manual timing config for 500kbps (ESP32-S3 compatible)
static twai_timing_config_t t_config = {
    .brp = 8,          // Baud rate prescaler
    .tseg_1 = 15,      // Time segment 1
    .tseg_2 = 4,       // Time segment 2
    .sjw = 1,          // Sync jump width
    .triple_sampling = false
};

static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// CAN status variables
static bool can_initialized = false;
static uint32_t can_rx_count = 0;
static uint32_t can_tx_count = 0;

// Initialize CAN/TWAI driver
bool init_can_twai(void) {
    esp_err_t result;

    // Install TWAI driver
    result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("Failed to install TWAI driver: %s\n", esp_err_to_name(result));
        return false;
    }

    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("Failed to start TWAI driver: %s\n", esp_err_to_name(result));
        twai_driver_uninstall();
        return false;
    }

    can_initialized = true;
    Serial.println("CAN/TWAI initialized successfully");

    // Send startup frame
    uint8_t startup_data[8] = {0xAA, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x99, 0x99};
    if (send_can_frame(STARTUP_FRAME_ID, startup_data, 8)) {
        Serial.println("Startup CAN frame sent successfully");
    } else {
        Serial.println("Failed to send startup CAN frame");
    }

    return true;
}

// Send CAN frame
bool send_can_frame(uint32_t id, uint8_t* data, uint8_t length) {
    if (!can_initialized || length > 8) {
        return false;
    }

    twai_message_t message;
    message.identifier = id;
    message.extd = 0; // Standard frame
    message.data_length_code = length;
    memcpy(message.data, data, length);

    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (result == ESP_OK) {
        can_tx_count++;
        #if CAN_DEBUG_LEVEL == 1
        Serial.printf("CAN TX: ID=0x%03X, Data=", id);
        for (int i = 0; i < length; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
        #endif
        return true;
    } else {
        #if CAN_DEBUG_LEVEL == 1
        Serial.printf("CAN TX failed: %s\n", esp_err_to_name(result));
        #endif
        return false;
    }
}

// Receive CAN frame (non-blocking)
bool receive_can_frame(twai_message_t* message) {
    if (!can_initialized) {
        return false;
    }

    esp_err_t result = twai_receive(message, pdMS_TO_TICKS(10)); // 10ms timeout
    if (result == ESP_OK) {
        can_rx_count++;
        #if CAN_DEBUG_LEVEL == 1
        Serial.printf("CAN RX: ID=0x%03X, DLC=%d, Data=",
                     message->identifier,
                     message->data_length_code);
        for (int i = 0; i < message->data_length_code; i++) {
            Serial.printf("%02X ", message->data[i]);
        }
        Serial.println();
        #endif
        return true;
    }
    return false;
}

// CAN monitoring task (can run in RTOS)
void can_task(void* parameter) {
    twai_message_t rx_message;

    while (true) {
        // Check for received messages
        if (receive_can_frame(&rx_message)) {
#if CAN_RTC_DEBUG
            // Update CAN debug screen with received frame
            update_can_debug_display(rx_message.identifier, rx_message.data, rx_message.data_length_code);
#endif // CAN_RTC_DEBUG

            // Process received frame based on ID
            switch (rx_message.identifier) {
                case SENSOR_DATA_1_ID:
                    // Process Voltage/Current data (0x101)
                    if (rx_message.data_length_code >= 4) {
                        uint16_t voltage_raw = bigEndianToUint16(&rx_message.data[0]);
                        uint16_t current_raw = bigEndianToUint16(&rx_message.data[2]);

                        sensorData.volt = voltage_raw / 100.0f;  // Convert to volts
                        sensorData.curr = current_raw / 100.0f;  // Convert to amps

                        // Update battery detection state (same logic as UART command)
                        battery_detected = (sensorData.volt >= 9.0f);

                        #if CAN_DEBUG_LEVEL == 1
                        Serial.printf("Sensor Data 1: Volt=%.2fV, Curr=%.2fA, Battery_detected=%d\n", 
                                     sensorData.volt, sensorData.curr, battery_detected);
                        #endif
                    }
                    break;

                case SENSOR_DATA_2_ID:
                    // Process Temperature data (0x102)
                    if (rx_message.data_length_code >= 8) {
                        sensorData.temp1 = (int32_t)bigEndianToInt16(&rx_message.data[0]);
                        sensorData.temp2 = (int32_t)bigEndianToInt16(&rx_message.data[2]);
                        sensorData.temp3 = (int32_t)bigEndianToInt16(&rx_message.data[4]);
                        sensorData.temp4 = (int32_t)bigEndianToInt16(&rx_message.data[6]);

                        #if CAN_DEBUG_LEVEL == 1
                        Serial.printf("Sensor Data 2: Temp1=%d, Temp2=%d, Temp3=%d, Temp4=%d\n",
                                    sensorData.temp1, sensorData.temp2, sensorData.temp3, sensorData.temp4);
                        #endif
                    }
                    break;

                case SENSOR_DATA_3_ID:
                    // Process RTC Date/Time data (0x103)
                    if (rx_message.data_length_code >= 7) {
                        m2Time.year = bigEndianToUint16(&rx_message.data[0]);
                        m2Time.month = rx_message.data[2];
                        m2Time.date = rx_message.data[3];
                        m2Time.day_of_week = rx_message.data[4];
                        m2Time.hour = rx_message.data[5];
                        m2Time.minute = rx_message.data[6];
                        m2Time.second = rx_message.data[7];

                        #if CAN_DEBUG_LEVEL == 1
                        Serial.printf("Sensor Data 3: %04d-%02d-%02d %02d:%02d:%02d (Day %d)\n",
                                    m2Time.year, m2Time.month, m2Time.date,
                                    m2Time.hour, m2Time.minute, m2Time.second, m2Time.day_of_week);
                        #endif
                    }
                    break;

                default:
                    #if CAN_DEBUG_LEVEL == 1
                    Serial.printf("Unknown CAN ID: 0x%03X\n", rx_message.identifier);
                    #endif
                    break;
            }
        }

        // Small delay to prevent hogging CPU
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Get CAN statistics
void get_can_stats(uint32_t* rx_count, uint32_t* tx_count) {
    *rx_count = can_rx_count;
    *tx_count = can_tx_count;
}

// Check if CAN is initialized
bool is_can_initialized(void) {
    return can_initialized;
}

// Send contactor control command
// CAN ID: 0x105
// Data[0]: 0x01 (M1 Node ID - ESP32 LCD)
// Data[1]: 0x4C (ON/close) or 0x8B (OFF/open)
// Data[2-7]: 0x00 (Reserved)
bool send_contactor_control(uint8_t command) {
    if (!can_initialized) {
        Serial.println("[CONTACTOR] CAN not initialized, cannot send contactor control");
        return false;
    }
    
    uint8_t data[8] = {0};
    data[0] = M1_NODE_ID;  // 0x01 - M1 Node ID (ESP32 LCD)
    data[1] = command;      // 0x4C (close) or 0x8B (open)
    // Data[2-7] remain 0x00 (Reserved)
    
    bool result = send_can_frame(CONTACTOR_CONTROL_ID, data, 8);
    
    if (result) {
        const char* cmd_str = (command == CONTACTOR_CLOSE) ? "CLOSE" : "OPEN";
        Serial.printf("[CONTACTOR] Sent %s command (0x%02X) to M2 via CAN ID 0x%03X\n", 
                     cmd_str, command, CONTACTOR_CONTROL_ID);
    } else {
        Serial.printf("[CONTACTOR] Failed to send contactor control command (0x%02X)\n", command);
    }
    
    return result;
}
