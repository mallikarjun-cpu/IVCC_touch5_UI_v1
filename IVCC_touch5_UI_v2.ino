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
extern void check_m2_heartbeat(void);
extern volatile unsigned long can101_rx_timestamp;
unsigned long last_table_update;
static unsigned long last_heartbeat_check_ms = 0;
static const unsigned long HEARTBEAT_CHECK_INTERVAL_MS = 1000;

// Forward declarations for screen management variables
extern bool battery_detected;

// Global sensor data struct
struct sensor_data {
    float volt = -3.0f;
    float curr = -3.0f;
    int32_t temp1 = 0;
    int32_t temp2 = 0;
    int32_t temp3 = 0;
    int32_t temp4 = 0;
} sensorData;

// Time data from M2 struct
struct time_from_m2 {
    uint16_t year = 2010;
    uint8_t month = 1;
    uint8_t date = 1;
    uint8_t day_of_week = 1; // 1=Sunday
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
} m2Time;

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
//    String title = "LVGL porting example";

    Serial.begin(115200);
    Serial.setTimeout(10); // Prevent serial blocking
    delay(100); // Small delay for serial stabilization

    Serial.println("Initializing board, code is on github, changed again 2");
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

    /* Initialize SD card before screens so screen 1 can show entry number from charge_log */
    initializeSDCard();

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    // * Create all UI screens at startup (screen 1 uses getNextSerialNumber for Entry if SD ready)
    Serial.println("Initializing all screens...");
    initialize_all_screens();
    Serial.println("All screens initialized");

    /* Release the mutex */
    lvgl_port_unlock();
    Serial.println("LVGL mutex unlocked");

    // Initialize battery profiles after SD card setup
    initializeBatteryProfiles();

    //give 200ms delay and send a contactor open cmd over can bus.
    delay(200);
    send_contactor_control(CONTACTOR_OPEN);
    
    Serial.println("End of setup, setup success! ------ v4.1------  \n -----");
}

void loop()
{
    //Serial.println("IDLE loop");
    //check serial rx buffer. (for input cmds. ) and print .
    //process_serial_cmd(); //remove in production

    // Periodic screen state check (non-blocking)
    update_screen_based_on_state();

    // Update current screen content
    update_current_screen();

    // Periodic table updates (every 1 second)
    if (millis() - last_table_update >= 1000) {
        update_table_values();
        last_table_update = millis();
    }

    // M2 heartbeat: every 1s, 6s startup grace; if (now - can101_rx_timestamp) > 2100 ms -> M2 lost (screen 18)
    if (millis() - last_heartbeat_check_ms >= HEARTBEAT_CHECK_INTERVAL_MS) {
        check_m2_heartbeat();
        last_heartbeat_check_ms = millis();
    }

    delay(100); // 10Hz loop frequency (100ms = 10 times per second)
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

    // Initialize charge logging first
    if (initChargeLogging()) {
        // Only set flag to true if charge logging initialization succeeds
        sd_logging_initialized = true;
        Serial.println("SD card initialized successfully, logging enabled");
    } else {
        // Set flag to false if charge logging initialization fails
        sd_logging_initialized = false;
        Serial.println("Charge logging initialization failed, logging disabled");
    }
}