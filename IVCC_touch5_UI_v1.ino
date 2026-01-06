#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <lvgl.h> //ensure 8.4.0 version
#include "lvgl_v8_port.h"
#include "screen_definitions.h"
#include "can_twai.h"
#include "sd_logging.h"
#include "rs485_vfdComs.h"

// Forward declarations for screen management functions
extern void initialize_all_screens();
extern void update_screen_based_on_state();
extern void update_table_values();
extern void update_wifi_connection_status();
unsigned long last_table_update;

// Forward declarations for screen management variables
extern bool battery_detected;

// Global sensor data struct
struct sensor_data {
    float volt = -3.0f;
    float curr = -3.0f;
    uint temp1 = 0;
} sensorData;

// Global instances definitions
ScreenLogger screenLogger;
sd_logging sdLogger;

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// Global board object for IO expander access
Board *board = nullptr;

/**
 * To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 */
 // #include <demos/lv_demos.h>
 // #include <examples/lv_examples.h>

void setup()
{
    String title = "LVGL porting example";

    Serial.begin(115200);
    Serial.setTimeout(10); // Prevent serial blocking
    delay(100); // Small delay for serial stabilization

    Serial.println("Initializing board, code is on github");
    board = new Board();
    board->init();

//#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
/*
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    
    // * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
    // * "bounce buffer" functionality to enhance the RGB data bandwidth.
    // * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
     
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
*/
    assert(board->begin());

    Serial.println("Initializing LVGL");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) { //line that starts lvgl
        Serial.println("ERROR: LVGL initialization failed!");
        while(1) delay(1000); // Halt on failure
    }
    Serial.println("LVGL initialized successfully");

    Serial.println("Initializing CAN/TWAI");
    if (!init_can_twai()) {
        Serial.println("CAN initialization failed!");
    }

    // Create CAN monitoring task (RTOS)
    xTaskCreatePinnedToCore(can_task, "CAN_Task", 4096, NULL, 1, NULL, 1);

    //initialise rs485 coms, with uart2 at pin 44,43 as tx,rx at 9600 baud.
    rs485_init();
    delay(100); // Small delay for serial stabilization

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    // * Create all UI screens at startup
    Serial.println("Initializing all screens...");
    initialize_all_screens();
    Serial.println("All screens initialized");

    /* Release the mutex */
    lvgl_port_unlock();
    Serial.println("LVGL mutex unlocked");

    initializeSDCard();

    // Initialize battery profiles after SD card setup
    initializeBatteryProfiles();

    Serial.println("End of setup, setup success! ------------  \n -----");
}

void loop()
{
    //Serial.println("IDLE loop");
    //check serial rx buffer. (for input cmds. ) and print .
    process_serial_cmd();

    // Periodic screen state check (non-blocking)
    update_screen_based_on_state();

    // Update current screen content
    update_current_screen();

    // Periodic table updates (every 1 second)
    if (millis() - last_table_update >= 1000) {
        update_table_values();
        last_table_update = millis();
    }

    delay(200);
}

// Process serial commands - mainly for voltage input
void process_serial_cmd() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        // Print the received command
        Serial.print("Received command: '");
        Serial.print(cmd);
        Serial.println("'");

        // Check if command ends with 'v' or 'V' (voltage command)
        if (cmd.length() > 0 && (cmd.charAt(cmd.length() - 1) == 'v' || cmd.charAt(cmd.length() - 1) == 'V')) {
            // Remove the 'v' and parse as float
            String voltStr = cmd.substring(0, cmd.length() - 1);
            float voltValue = voltStr.toFloat();

            // Basic validation - voltage should be reasonable (0-100V range)
            if (voltValue > 0 && voltValue <= 100) {
                sensorData.volt = voltValue;
                Serial.print("Voltage updated to: ");
                Serial.print(sensorData.volt);
                Serial.println("V");

                // Update battery detection state
                battery_detected = (sensorData.volt >= 9.0f);

                // Update table immediately after data change
                update_table_values();

                // Check if screen needs to change based on new state
                update_screen_based_on_state();
            } else {
                Serial.println("Invalid voltage value. Range: 0.1-100V");
            }
        } else {
            // Invalid command - show help
            Serial.println("Invalid command!");
            Serial.println("Valid commands:");
            Serial.println("  12.3v  - Set voltage to 12.3V");
            Serial.println("  45v    - Set voltage to 45V");
            Serial.println("  (voltage must be 0.1-100V)");
        }
    }
}

//initialize SD card, call at end of setup. 
void initializeSDCard() {
    // Use extended GPIO for SD card, defined in waveshare_lcd_port.h
    board->getIO_Expander()->getBase()->digitalWrite(SD_CS, LOW); // SD card select pin

    // Initialize SPI
    SPI.setHwCs(false);
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_SS);
    if (!SD.begin(SD_SS)) {
        Serial.println("Card Mount Failed"); // SD card mounting failed
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached"); // No SD card connected
        return;
    }

    Serial.print("SD Card Type: "); // SD card type
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN"); // Unknown Type
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize); // SD card size

    // Initialize SD logging
    if (!sdLogger.initialize()) {
        Serial.println("SD logging initialization failed");
    }
}