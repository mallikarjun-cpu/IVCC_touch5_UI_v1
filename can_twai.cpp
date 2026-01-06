#include "can_twai.h"
#include <esp_log.h>

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
        Serial.printf("CAN TX: ID=0x%03X, Data=", id);
        for (int i = 0; i < length; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
        return true;
    } else {
        Serial.printf("CAN TX failed: %s\n", esp_err_to_name(result));
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
        Serial.printf("CAN RX: ID=0x%03X, DLC=%d, Data=",
                     message->identifier,
                     message->data_length_code);
        for (int i = 0; i < message->data_length_code; i++) {
            Serial.printf("%02X ", message->data[i]);
        }
        Serial.println();
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
            // Process received frame based on ID
            switch (rx_message.identifier) {
                case SENSOR_FRAME_ID:
                    // Handle sensor data
                    Serial.println("Sensor data received");
                    break;
                default:
                    Serial.printf("Unknown CAN ID: 0x%03X\n", rx_message.identifier);
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
