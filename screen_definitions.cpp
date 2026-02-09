#include "screen_definitions.h"
#include "can_twai.h"
#include "sd_logging.h"
#include "esp_panel_board_custom_conf.h"
#include "lvgl_v8_port.h"
#include "rs485_vfdComs.h"
#include "ble.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <WiFi.h>

// External sensor data
extern struct sensor_data {
    float volt;
    float curr;
    int32_t temp1;
} sensorData;

// External time data from M2
extern struct time_from_m2 {
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t day_of_week;  // Note: 1=Sunday in M2, will convert to 1=Monday for display
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} m2Time;

// External BLE manager and credentials
extern BLEManager bleManager;
extern String ble_ssid;
extern String ble_key;

// ============================================================================
// WiFi Configuration (hardcoded for testing)
// ============================================================================

// WiFi credentials for testing - UPDATE THESE WITH YOUR ACTUAL NETWORK DETAILS
const char* TEST_WIFI_SSID = "test567";
const char* TEST_WIFI_PASSWORD = "12345678";

// WiFi connection state management
static bool wifi_connection_in_progress = false;
static unsigned long wifi_connection_start_time = 0;
static int wifi_connection_attempts = 0;
static const unsigned long WIFI_CONNECTION_TIMEOUT = 10000; // 10 seconds
static const int WIFI_MAX_ATTEMPTS = 20; // 20 attempts * 500ms = 10 seconds

// ============================================================================
// Global UI Variables
// ============================================================================

// Screen objects
lv_obj_t* screen_1 = nullptr; //screen 1 - default screen
lv_obj_t* screen_2 = nullptr; //screen 2 - battery detected, charge ready page
lv_obj_t* screen_3 = nullptr; //screen 3 - charging start (waiting for 1A)
lv_obj_t* screen_4 = nullptr; //screen 4 - Constant Current (CC) mode
lv_obj_t* screen_5 = nullptr; //screen 5 - Constant Voltage (CV) mode
lv_obj_t* screen_6 = nullptr; //screen 6 - Charging complete
lv_obj_t* screen_7 = nullptr; //screen 7 - Emergency stop
lv_obj_t* screen_13 = nullptr; //screen 13 - CAN debug screen
lv_obj_t* screen_16 = nullptr; //screen 16 - Time debug screen
lv_obj_t* screen_17 = nullptr; //screen 17 - BLE debug screen

// Shared UI objects (reused between screens)
static lv_obj_t* status_label = nullptr;
static lv_obj_t* data_table = nullptr;  // Shared table shown on all screens
// Battery container references for screens that need profiles
static lv_obj_t* screen2_battery_container = nullptr;
static lv_obj_t* screen2_button_container = nullptr;

// Selected battery profile for screen 3, 4, 5 display
static BatteryType* selected_battery_profile = nullptr;
static lv_obj_t* screen3_battery_details_label = nullptr;
static lv_obj_t* screen4_battery_details_label = nullptr;
static lv_obj_t* screen5_battery_details_label = nullptr;

// Timer variables for charging screens
static unsigned long charging_start_time = 0;  // When charging started (screen 3)
static unsigned long cv_start_time = 0;        // When CV state started (screen 5)
static unsigned long cc_state_duration_for_timer = 0;  // CC state duration for remaining time calc
static unsigned long final_charging_time_ms = 0;  // Final charging time when complete (stops updating)
static unsigned long final_remaining_time_ms = 0;  // Final remaining time when complete (stops updating)
static bool charging_complete = false;  // Flag to stop timer updates after completion
static bool pending_stop_command = false;  // Flag to send stop command after screen loads

// Timer table objects for screens 3, 4, 5, 6, 7
static lv_obj_t* screen3_timer_table = nullptr;
static lv_obj_t* screen4_timer_table = nullptr;
static lv_obj_t* screen5_timer_table = nullptr;
static lv_obj_t* screen6_timer_table = nullptr;
static lv_obj_t* screen7_timer_table = nullptr;

// screen 2 confirmation popup
static lv_obj_t* screen2_confirm_popup = nullptr;
static lv_obj_t* screen2_confirm_title_label = nullptr;
static lv_obj_t* screen2_confirm_voltage_label = nullptr;
static lv_obj_t* screen2_confirm_capacity_label = nullptr;
static lv_obj_t* screen2_confirm_current_label = nullptr;
static lv_obj_t* screen2_confirm_type_label = nullptr;
static lv_obj_t* screen2_confirm_agree_btn = nullptr;
static lv_obj_t* screen2_confirm_change_btn = nullptr;

// screen 1 WiFi status display
static lv_obj_t* screen1_wifi_status_label = nullptr;

// screen 13 CAN frame display
static lv_obj_t* screen13_can_frame_label = nullptr;
#define CAN_DEBUG_MAX_LINES 5
static char can_debug_lines[CAN_DEBUG_MAX_LINES][200];
static int can_debug_current_line = 0;

// screen 16 time display
static lv_obj_t* screen16_time_label = nullptr;

// screen 17 BLE debug display
static lv_obj_t* screen17_wifi_status_label = nullptr;
static lv_obj_t* screen17_ble_status_label = nullptr;
static lv_obj_t* screen17_ble_info_label = nullptr;

// Reboot countdown variables
static bool reboot_countdown_active = false;
static unsigned long reboot_countdown_start_time = 0;
static const unsigned long REBOOT_COUNTDOWN_DURATION = 5000; // 5 seconds

// ============================================================================
// Screen Management Globals
// ============================================================================

// Current screen and state tracking
screen_id_t current_screen_id = SCREEN_HOME;
app_state_t current_app_state = STATE_HOME;
charge_stop_reason_t charge_stop_reason = CHARGE_STOP_NONE;

// State-based screen switching triggers
bool battery_detected = false;

// ============================================================================
// Charging Control Globals
// ============================================================================
static uint16_t current_frequency = 0;                // Current motor frequency in 0.01Hz units (0 = stopped)
static unsigned long last_control_update = 0;         // Rate limiting for charging control
static unsigned long cc_state_start_time = 0;         // Time when CC state started (millis)
static unsigned long cv_state_start_time = 0;         // Time when CV state started (millis)
static unsigned long cc_state_duration = 0;           // Duration spent in CC state (millis)

// ============================================================================
// M2 State Management Globals
// ============================================================================

// M2 State Configuration Array (2 states only)
static const m2_state_config_t m2_state_configs[] = {
    {M2_STATE_STANDBY, "M2State1", lv_color_hex(0x0000FF), lv_color_hex(0x0000FF), "M2 State 1"},
    {M2_STATE_INIT, "M2State2", lv_color_hex(0x00FF00), lv_color_hex(0x00FF00), "M2 State 2"},
};

// M2 Status Manager
static m2_status_manager_t m2_manager = {
    .state_box = nullptr,
    .state_label = nullptr,
    .current_state = M2_STATE_STANDBY,
    .last_update_time = 0,
    .is_connected = false
};

// Legacy M2 global variables (for backward compatibility)
static lv_obj_t* m2_state_box = nullptr;
static lv_obj_t* m2_state_label = nullptr;

// M2 state box arrays for multiple screens (legacy support)
#define MAX_SCREENS 13
static lv_obj_t* m2_state_box_array[MAX_SCREENS] = {nullptr};
static lv_obj_t* m2_state_label_array[MAX_SCREENS] = {nullptr};

void createM2StateBox(lv_obj_t *parent_screen, const char* screen_name)
{
    // Get the current state configuration
    const m2_state_config_t* config = getM2StateConfig(m2_manager.current_state);

    // Container box (positioned right of M1 box: x=170)
    lv_obj_t *box = lv_obj_create(parent_screen);
    lv_obj_set_size(box, 160, 50);
    lv_obj_align(box, LV_ALIGN_TOP_LEFT, 10, 10);  // Top left position
    lv_obj_set_style_bg_color(box, config->bg_color, LV_PART_MAIN);
    lv_obj_set_style_border_color(box, config->border_color, LV_PART_MAIN);
    lv_obj_set_style_border_width(box, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(box, 5, LV_PART_MAIN);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);  // Disable scrolling

    // Status label
    lv_obj_t *label = lv_label_create(box);
    lv_label_set_text(label, config->label_text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_center(label);

    // Update the manager
    m2_manager.state_box = box;
    m2_manager.state_label = label;
    m2_manager.last_update_time = millis();

    // Store in arrays for multi-screen support (map screen names to indices)
    int screen_index = 0; // default
    if (strcmp(screen_name, "screen_1") == 0) screen_index = 1;
    else if (strcmp(screen_name, "screen_3") == 0) screen_index = 3;
    else if (strcmp(screen_name, "screen_4") == 0) screen_index = 4;
    else if (strcmp(screen_name, "screen_5") == 0) screen_index = 5;

    if (screen_index >= 0 && screen_index < MAX_SCREENS) {
        m2_state_box_array[screen_index] = box;
        m2_state_label_array[screen_index] = label;
    }

    // Also store in legacy pointers (last one created)
    m2_state_box = box;
    m2_state_label = label;

    Serial.printf("[CREATE M2 BOX] Screen %s (index %d): box=%p, label=%p, state=%d (%s)\n",
                  screen_name, screen_index, box, label, m2_manager.current_state, config->label_text);
}

// Get M2 state configuration by state enum (only 2 states supported)
const m2_state_config_t* getM2StateConfig(m2_state_t state) {
    if (state == M2_STATE_STANDBY) {
        return &m2_state_configs[0]; // M2State1
    } else if (state == M2_STATE_INIT) {
        return &m2_state_configs[1]; // M2State2
    }
    // Default to M2State1 if invalid state
    return &m2_state_configs[0];
}

// Update M2 state and UI
void updateM2State(m2_state_t new_state) {
    if (new_state == m2_manager.current_state) {
        return; // No change needed
    }

    // Validate state range (only allow 2 states for now)
    if (new_state != M2_STATE_STANDBY && new_state != M2_STATE_INIT) {
        Serial.printf("[M2 ERROR] Invalid state: %d (only M2State1 and M2State2 allowed)\n", new_state);
        return;
    }

    // Update state
    m2_manager.current_state = new_state;
    m2_manager.last_update_time = millis();

    // Update UI if objects exist
    if (m2_manager.state_box && m2_manager.state_label) {
        const m2_state_config_t* config = getM2StateConfig(new_state);

        // Update box colors
        lv_obj_set_style_bg_color(m2_manager.state_box, config->bg_color, LV_PART_MAIN);
        lv_obj_set_style_border_color(m2_manager.state_box, config->border_color, LV_PART_MAIN);

        // Update label text
        lv_label_set_text(m2_manager.state_label, config->label_text);

        Serial.printf("[M2 STATE UPDATE] State: %d (%s), Box: %p, Label: %p\n",
                      new_state, config->label_text, m2_manager.state_box, m2_manager.state_label);
    } else {
        Serial.printf("[M2 STATE UPDATE] State changed to %d but UI objects not initialized\n", new_state);
    }
}

// Update M2 connection status
void updateM2ConnectionStatus(bool connected) {
    m2_manager.is_connected = connected;
    m2_manager.last_update_time = millis();

    if (!connected) {
        // Force disconnected state when connection is lost
        updateM2State(M2_STATE_DISCONNECTED);
    }

    Serial.printf("[M2 CONNECTION] Status: %s\n", connected ? "CONNECTED" : "DISCONNECTED");
}

// Forward declarations for battery profile functions
void displayMatchingBatteryProfiles(float detectedVoltage, lv_obj_t* container);

// Forward declaration for emergency stop handler
void emergency_stop_event_handler(lv_event_t * e);
// Forward declaration for home button handler
void home_button_event_handler(lv_event_t * e);
// Forward declaration for WiFi status update
void update_screen1_wifi_status(void);

// ============================================================================
// Screen Management Functions
// ============================================================================

// Initialize all screens at startup
void initialize_all_screens() {
    Serial.println("[SCREEN] Creating all screens...");

    create_screen_1();   // Home screen
    create_screen_2();   // Battery detected screen
    create_screen_3();   // Charging start screen
    create_screen_4();   // CC mode screen
    create_screen_5();   // CV mode screen
    create_screen_6();   // Charging complete screen
    create_screen_7();   // Emergency stop screen
    create_screen_13();  // CAN debug screen
    create_screen_16();  // Time debug screen
    create_screen_17();  // BLE debug screen

    // Start with home screen
    switch_to_screen(SCREEN_HOME);

    Serial.println("[SCREEN] All screens initialized");
}

// Switch to a specific screen
void switch_to_screen(screen_id_t screen_id) {
    lv_obj_t* target_screen = nullptr;

    // Determine which screen object to load
    switch (screen_id) {
        case SCREEN_HOME:
            target_screen = screen_1;
            break;
        case SCREEN_BATTERY_DETECTED:
            target_screen = screen_2;
            break;
        case SCREEN_CHARGING_STARTED:
            target_screen = screen_3;
            break;
        case SCREEN_CHARGING_CC:
            target_screen = screen_4;
            break;
        case SCREEN_CHARGING_CV:
            target_screen = screen_5;
            break;
        case SCREEN_CHARGING_COMPLETE:
            target_screen = screen_6;
            break;
        case SCREEN_EMERGENCY_STOP:
            target_screen = screen_7;
            break;
        case SCREEN_CAN_DEBUG:
            target_screen = screen_13;
            break;
        case SCREEN_TIME_DEBUG:
            target_screen = screen_16;
            break;
        case SCREEN_BLE_DEBUG:
            target_screen = screen_17;
            break;
        default:
            Serial.printf("[SCREEN] ERROR: Invalid screen ID %d\n", screen_id);
            return;
    }

    if (target_screen != nullptr) {
        // Update current screen tracking
        current_screen_id = screen_id;

        // Move shared UI elements to the new screen (except screen 17)
        if (data_table != nullptr && screen_id != SCREEN_BLE_DEBUG) {
            lv_obj_set_parent(data_table, target_screen);
            lv_obj_set_pos(data_table, 12, 110);
            lv_obj_clear_flag(data_table, LV_OBJ_FLAG_HIDDEN); // Make sure table is visible
        } else if (screen_id == SCREEN_BLE_DEBUG && data_table != nullptr) {
            // Hide table on BLE debug screen
            lv_obj_add_flag(data_table, LV_OBJ_FLAG_HIDDEN);
        }
        
        // Send stop command after screen 6 or 7 loads (if pending)
        if (pending_stop_command && (screen_id == SCREEN_CHARGING_COMPLETE || screen_id == SCREEN_EMERGENCY_STOP)) {
            delay(50);  // Give screen time to fully load
            Serial.println("[CHARGING] Sending STOP command to VFD after screen load...");
            rs485_sendStopCommand();
            Serial.println("[CHARGING] Stop command sent to VFD - Motor should now be stopped");
            pending_stop_command = false;  // Clear flag
        }

        // Manage battery container visibility (only for screens that need profiles)
        if (screen2_battery_container != nullptr) {
            if (screen_id == SCREEN_BATTERY_DETECTED) {
                lv_obj_clear_flag(screen2_battery_container, LV_OBJ_FLAG_HIDDEN);
                // Refresh profiles display when switching to screen 2
                if (sensorData.volt > 0) {
                    displayMatchingBatteryProfiles(sensorData.volt, screen2_battery_container);
                } else {
                    // Show message if no voltage detected
                    lv_obj_clean(screen2_battery_container);
                    lv_obj_clear_flag(screen2_battery_container, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_t* no_volt_label = lv_label_create(screen2_battery_container);
                    lv_label_set_text(no_volt_label, "No voltage detected. Send voltage command first (e.g. '12.3v')");
                    lv_obj_set_style_text_font(no_volt_label, &lv_font_montserrat_20, LV_PART_MAIN);
                    lv_obj_set_style_text_color(no_volt_label, lv_color_hex(0xFF0000), LV_PART_MAIN);
                    lv_obj_center(no_volt_label);
                }
            } else {
                lv_obj_add_flag(screen2_battery_container, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // Update M2 state box for current screen
        update_m2_state_display();

        // Load the screen
        lv_scr_load(target_screen);

        Serial.printf("[SCREEN] Switched to screen %d\n", screen_id);
    } else {
        Serial.printf("[SCREEN] ERROR: Screen %d not initialized\n", screen_id);
    }
}

// Forward declaration for WiFi status update function
void update_wifi_connection_status();

// ============================================================================
// Charging Control Function
// ============================================================================
void update_charging_control() {
    // Only run in charging states
    if (current_app_state != STATE_CHARGING_START && 
        current_app_state != STATE_CHARGING_CC && 
        current_app_state != STATE_CHARGING_CV) {
        return;
    }
    
    // Check if battery profile is selected
    if (selected_battery_profile == nullptr) {
        return;
    }
    
    // Rate limiting: update once per second
    const unsigned long CONTROL_UPDATE_INTERVAL = 1000; // 1 second
    if (millis() - last_control_update < CONTROL_UPDATE_INTERVAL) {
        return;
    }
    last_control_update = millis();
    
    // Get target values from battery profile
    float target_current = selected_battery_profile->getConstCurrent(); // Amps
    float target_voltage = selected_battery_profile->getCutoffVoltage(); // Volts
    
    // Validate and convert sensor data (handle negative values)
    // Clamp negative values to 0 to prevent unsigned wrap-around
    float safe_actual_current = (sensorData.curr < 0.0f) ? 0.0f : sensorData.curr;
    float safe_actual_voltage = (sensorData.volt < 0.0f) ? 0.0f : sensorData.volt;
    
    // Convert to 0.01 units (as required by RS485 functions)
    uint16_t target_current_0_01A = (uint16_t)(target_current * 100);
    uint16_t target_voltage_0_01V = (uint16_t)(target_voltage * 100);
    uint16_t actual_current_0_01A = (uint16_t)(safe_actual_current * 100);
    uint16_t actual_voltage_0_01V = (uint16_t)(safe_actual_voltage * 100);
    
    uint16_t new_frequency = current_frequency;
    
    // State-specific charging logic
    if (current_app_state == STATE_CHARGING_START) {
        // STATE_CHARGING_START: Wait until 1A current flows
        // Use CC logic to increase RPM
        new_frequency = rs485_CalcFrequencyFor_CC(
            current_frequency, 
            target_current_0_01A, 
            actual_current_0_01A
        );
        
        // Debug logging
        int32_t current_error = (int32_t)actual_current_0_01A - (int32_t)target_current_0_01A;
        Serial.printf("[CHARGING_START] Target: %.2fA, Actual: %.2fA, Error: %d (0.01A), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            target_current, safe_actual_current, current_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Check if current >= 1A, then transition to CC state
        if (safe_actual_current >= 1.0f) {
            Serial.println("[CHARGING] Current reached 1A, transitioning to CC mode");
            current_app_state = STATE_CHARGING_CC;
            cc_state_start_time = millis();  // Record CC state start time
            Serial.println("[CHARGING] CC state timing started");
            // Screen switch will happen in determine_screen_from_state()
        }
    }
    else if (current_app_state == STATE_CHARGING_CC) {
        // STATE_CHARGING_CC: Constant Current mode until target voltage reached
        new_frequency = rs485_CalcFrequencyFor_CC(
            current_frequency, 
            target_current_0_01A, 
            actual_current_0_01A
        );
        
        // Debug logging
        int32_t current_error = (int32_t)actual_current_0_01A - (int32_t)target_current_0_01A;
        Serial.printf("[CHARGING_CC] Target: %.2fA, Actual: %.2fA, Error: %d (0.01A), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            target_current, safe_actual_current, current_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Check if voltage >= target voltage, then transition to CV state
        if (safe_actual_voltage >= target_voltage) {
            Serial.println("[CHARGING] Voltage reached target, transitioning to CV mode");
            // Calculate CC state duration before transitioning
            if (cc_state_start_time > 0) {
                cc_state_duration = millis() - cc_state_start_time;
                cc_state_duration_for_timer = cc_state_duration;  // Store for remaining time calculation
                Serial.printf("[CHARGING] CC state duration: %lu ms (%.2f minutes)\n", 
                    cc_state_duration, cc_state_duration / 60000.0f);
            }
            current_app_state = STATE_CHARGING_CV;
            cv_state_start_time = millis();  // Record CV state start time for charging control
            cv_start_time = millis();  // Record CV state start time for timer display
            Serial.println("[CHARGING] CV state timing started");
            
            // Calculate initial remaining time for immediate display
            if (cc_state_duration_for_timer > 0) {
                unsigned long cv_33_percent_time = cc_state_duration_for_timer / 3;  // 33% of CC time
                unsigned long cv_30_min = 30 * 60 * 1000;  // 30 minutes in ms
                unsigned long target_cv_time = (cv_33_percent_time < cv_30_min) ? cv_33_percent_time : cv_30_min;
                unsigned long rem_seconds = target_cv_time / 1000;
                unsigned long rem_minutes = rem_seconds / 60;
                rem_seconds = rem_seconds % 60;
                Serial.printf("[CHARGING] Initial remaining time: %02lu:%02lu (target CV time: %lu ms)\n", 
                             rem_minutes, rem_seconds, target_cv_time);
            }
            // Screen switch will happen in determine_screen_from_state()
        }
    }
    else if (current_app_state == STATE_CHARGING_CV) {
        // STATE_CHARGING_CV: Constant Voltage mode
        new_frequency = rs485_CalcFrequencyFor_CV(
            current_frequency, 
            target_voltage_0_01V, 
            actual_voltage_0_01V
        );
        
        // Debug logging
        int32_t voltage_error = (int32_t)actual_voltage_0_01V - (int32_t)target_voltage_0_01V;
        Serial.printf("[CHARGING_CV] Target: %.2fV, Actual: %.2fV, Error: %d (0.01V), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            target_voltage, safe_actual_voltage, voltage_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Termination condition: Check if charging is complete
        // Complete if: 30 minutes in CV mode OR 33% of CC state time has elapsed
        unsigned long current_time = millis();
        unsigned long cv_duration = 0;
        if (cv_state_start_time > 0) {
            cv_duration = current_time - cv_state_start_time;
        }
        
        const unsigned long CV_COMPLETE_TIME_MS = 30 * 60 * 1000;  // 30 minutes in milliseconds
        bool cv_time_complete = (cv_duration >= CV_COMPLETE_TIME_MS);
        
        // 33% of CC state time (if CC duration was recorded)
        bool cc_time_complete = false;
        if (cc_state_duration > 0) {
            unsigned long cc_33_percent_time = cc_state_duration / 3;  // 33% = 1/3
            cc_time_complete = (cv_duration >= cc_33_percent_time);
        }
        
        if (cv_time_complete || cc_time_complete) {
            Serial.println("[CHARGING] Charging complete condition met!");
            
            // STEP 1: Send 0 RPM command IMMEDIATELY when condition is met
            Serial.println("[CHARGING] Sending 0 RPM command immediately...");
            rs485_sendFrequencyCommand(0);  // Send 0 Hz
            current_frequency = 0;
            delay(10);  // Give RS485 time to send the command
            
            // STEP 2: Now do other things (logging, calculations, etc.)
            if (cv_time_complete) {
                Serial.printf("[CHARGING] CV mode duration: %lu ms (%.2f minutes) >= 30 minutes\n",
                    cv_duration, cv_duration / 60000.0f);
            }
            if (cc_time_complete && cc_state_duration > 0) {
                Serial.printf("[CHARGING] CV duration (%lu ms) >= 33%% of CC duration (%lu ms)\n",
                    cv_duration, cc_state_duration);
            }
            
            // Store final charging time and remaining time before transitioning
            if (charging_start_time > 0) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
                Serial.printf("[CHARGING] Final charging time: %lu ms (%.2f minutes)\n", 
                             final_charging_time_ms, final_charging_time_ms / 60000.0f);
                
                // Calculate and store final remaining time
                if (cv_start_time > 0 && cc_state_duration_for_timer > 0) {
                    unsigned long cv_elapsed = millis() - cv_start_time;
                    unsigned long cv_33_percent_time = cc_state_duration_for_timer / 3;  // 33% of CC time
                    unsigned long cv_30_min = 30 * 60 * 1000;  // 30 minutes in ms
                    unsigned long target_cv_time = (cv_33_percent_time < cv_30_min) ? cv_33_percent_time : cv_30_min;
                    final_remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                    Serial.printf("[CHARGING] Final remaining time: %lu ms (%.2f minutes)\n", 
                                 final_remaining_time_ms, final_remaining_time_ms / 60000.0f);
                }
            }
            
            // Open contactor via CAN bus (just before moving out of screen 5)
            Serial.println("[CONTACTOR] Opening contactor before completing charge...");
            send_contactor_control(CONTACTOR_OPEN);
            
            // Set stop reason and transition to complete state
            charge_stop_reason = CHARGE_STOP_COMPLETE;
            current_app_state = STATE_CHARGING_COMPLETE;
            
            // Set flag to send stop command after screen 6 loads
            pending_stop_command = true;
            
            // STEP 3: Move to screen 6
            switch_to_screen(SCREEN_CHARGING_COMPLETE);
        }
    }
}

// Update current screen content (screen-specific updates)
void update_current_screen() {
    // WiFi status display updates on screen 1 (home screen)
    if (current_screen_id == SCREEN_HOME) {
        update_screen1_wifi_status();
    }
    
    // Time debug display updates only when on time debug screen
    if (current_screen_id == SCREEN_TIME_DEBUG) {
        update_time_debug_display();
    }
    
    // BLE debug display updates only when on BLE debug screen
    if (current_screen_id == SCREEN_BLE_DEBUG) {
        update_ble_debug_display();
    }
    
    // Charging control (runs only in charging states, once per second)
    update_charging_control();
    
    // Update battery details on screens 3, 4, 5
    if (screen3_battery_details_label != nullptr && current_screen_id == SCREEN_CHARGING_STARTED) {
        if (selected_battery_profile != nullptr) {
            char details_text[100];
            sprintf(details_text, "Selected Battery: %s (%dV, %dAh, %s)",
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getRatedVoltage(),
                    selected_battery_profile->getRatedAh(),
                    getBatteryChemistryName(selected_battery_profile));
            lv_label_set_text(screen3_battery_details_label, details_text);
        } else {
            lv_label_set_text(screen3_battery_details_label, "Selected Battery: None");
        }
    }
    if (screen4_battery_details_label != nullptr && current_screen_id == SCREEN_CHARGING_CC) {
        if (selected_battery_profile != nullptr) {
            char details_text[100];
            sprintf(details_text, "Selected Battery: %s (%dV, %dAh, %s)",
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getRatedVoltage(),
                    selected_battery_profile->getRatedAh(),
                    getBatteryChemistryName(selected_battery_profile));
            lv_label_set_text(screen4_battery_details_label, details_text);
        } else {
            lv_label_set_text(screen4_battery_details_label, "Selected Battery: None");
        }
    }
    if (screen5_battery_details_label != nullptr && current_screen_id == SCREEN_CHARGING_CV) {
        if (selected_battery_profile != nullptr) {
            char details_text[100];
            sprintf(details_text, "Selected Battery: %s (%dV, %dAh, %s)",
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getRatedVoltage(),
                    selected_battery_profile->getRatedAh(),
                    getBatteryChemistryName(selected_battery_profile));
            lv_label_set_text(screen5_battery_details_label, details_text);
        } else {
            lv_label_set_text(screen5_battery_details_label, "Selected Battery: None");
        }
    }
    
    // Update timer displays on screens 3, 4, 5, 6, 7
    lvgl_port_lock(-1);  // Lock LVGL for thread safety
    
    unsigned long total_elapsed = 0;
    if (charging_complete && final_charging_time_ms > 0) {
        // Use final time if charging is complete (stops updating)
        total_elapsed = final_charging_time_ms;
    } else if (charging_start_time > 0) {
        // Calculate elapsed time if charging is in progress
        total_elapsed = millis() - charging_start_time;
    }
    
    if (total_elapsed > 0) {
        unsigned long total_seconds = total_elapsed / 1000;
        unsigned long total_minutes = total_seconds / 60;
        unsigned long total_hours = total_minutes / 60;
        total_seconds = total_seconds % 60;
        total_minutes = total_minutes % 60;
        
        char time_str[20];
        sprintf(time_str, "%02lu:%02lu:%02lu", total_hours, total_minutes, total_seconds);
        
        // Update total time on screens 3, 4, 5 (only if charging in progress)
        if (!charging_complete) {
            if (screen3_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_STARTED) {
                lv_table_set_cell_value(screen3_timer_table, 1, 0, time_str);
            }
            if (screen4_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_CC) {
                lv_table_set_cell_value(screen4_timer_table, 1, 0, time_str);
            }
            if (screen5_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_CV) {
                lv_table_set_cell_value(screen5_timer_table, 1, 0, time_str);
                
                // Also update remaining time on screen 5 (CV state)
                if (cv_start_time > 0 && cc_state_duration_for_timer > 0) {
                    unsigned long cv_elapsed = millis() - cv_start_time;
                    unsigned long cv_33_percent_time = cc_state_duration_for_timer / 3;  // 33% of CC time
                    unsigned long cv_30_min = 30 * 60 * 1000;  // 30 minutes in ms
                    unsigned long target_cv_time = (cv_33_percent_time < cv_30_min) ? cv_33_percent_time : cv_30_min;
                    unsigned long remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                    
                    unsigned long rem_seconds = remaining_time_ms / 1000;
                    unsigned long rem_minutes = rem_seconds / 60;
                    rem_seconds = rem_seconds % 60;
                    sprintf(time_str, "%02lu:%02lu", rem_minutes, rem_seconds);
                    lv_table_set_cell_value(screen5_timer_table, 1, 1, time_str);
                } else {
                    // If cv_start_time or cc_state_duration not set yet, show default
                    if (cv_start_time == 0) {
                        lv_table_set_cell_value(screen5_timer_table, 1, 1, "--:--");
                    } else {
                        lv_table_set_cell_value(screen5_timer_table, 1, 1, "00:00");
                    }
                }
            }
        }
        
        // Update final time on screens 6 and 7 (always show final time when complete)
        if (screen6_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_COMPLETE) {
            lv_table_set_cell_value(screen6_timer_table, 1, 0, time_str);
            
            // Also display final remaining time on screen 6
            if (final_remaining_time_ms > 0) {
                unsigned long rem_seconds = final_remaining_time_ms / 1000;
                unsigned long rem_minutes = rem_seconds / 60;
                rem_seconds = rem_seconds % 60;
                sprintf(time_str, "%02lu:%02lu", rem_minutes, rem_seconds);
                lv_table_set_cell_value(screen6_timer_table, 1, 1, time_str);
            } else {
                lv_table_set_cell_value(screen6_timer_table, 1, 1, "00:00");
            }
        }
        if (screen7_timer_table != nullptr && current_screen_id == SCREEN_EMERGENCY_STOP) {
            lv_table_set_cell_value(screen7_timer_table, 1, 0, time_str);
        }
    }
    
    lvgl_port_unlock();  // Unlock LVGL
}

// Determine which screen should be shown based on current state
screen_id_t determine_screen_from_state() {
    // Priority-based screen selection
    if (current_app_state == STATE_CHARGING_START) {
        return SCREEN_CHARGING_STARTED;
    }
    if (current_app_state == STATE_CHARGING_CC) {
        return SCREEN_CHARGING_CC;
    }
    if (current_app_state == STATE_CHARGING_CV) {
        return SCREEN_CHARGING_CV;
    }
    if (current_app_state == STATE_CHARGING_COMPLETE) {
        return SCREEN_CHARGING_COMPLETE;
    }
    if (current_app_state == STATE_EMERGENCY_STOP) {
        return SCREEN_EMERGENCY_STOP;
    }
    // Don't switch away from CAN debug screen based on voltage
    if (current_screen_id == SCREEN_CAN_DEBUG) {
        return SCREEN_CAN_DEBUG;
    }
    // Don't switch away from time debug screen based on voltage
    if (current_screen_id == SCREEN_TIME_DEBUG) {
        return SCREEN_TIME_DEBUG;
    }
    // Don't switch away from BLE debug screen based on voltage
    if (current_screen_id == SCREEN_BLE_DEBUG) {
        return SCREEN_BLE_DEBUG;
    }
    if (battery_detected && sensorData.volt >= 9.0f) {
        return SCREEN_BATTERY_DETECTED;
    }

    return SCREEN_HOME; // Default fallback
}

// Check state and switch screens if needed
void update_screen_based_on_state() {
    screen_id_t target_screen = determine_screen_from_state();

    if (target_screen != current_screen_id) {
        switch_to_screen(target_screen);
    }
}

// ============================================================================
// Battery Profile Display Functions
// ============================================================================

// Forward declarations for event handlers
void screen2_profile_selected_event_handler(lv_event_t * e);

// Display battery profiles matching the detected voltage range
void displayMatchingBatteryProfiles(float detectedVoltage, lv_obj_t* container) {
    if (!container) {
        Serial.println("[ERROR] Battery list container not provided");
        return;
    }

    // Clear any existing content
    lv_obj_clean(container);

    // Make container visible
    lv_obj_clear_flag(container, LV_OBJ_FLAG_HIDDEN);

    // Get matching profiles
    BatteryType* matches[50]; // Max 50 matches
    int matchCount = 0;
    batteryProfiles.getMatchingProfiles(detectedVoltage, matches, matchCount);

    Serial.printf("[BATTERY] Detected voltage: %.1fV, found %d matching profiles\n", detectedVoltage, matchCount);

    if (matchCount == 0) {
        // No matching profiles - show message
        lv_obj_t* no_match_label = lv_label_create(container);
        lv_label_set_text(no_match_label, "No battery profiles match the detected voltage range");
        lv_obj_set_style_text_font(no_match_label, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(no_match_label, lv_color_hex(0xFF0000), LV_PART_MAIN);
        lv_obj_center(no_match_label);
        return;
    }

    // Create a scrollable list of matching battery profiles
    int button_y = 10; // Start Y position for first button
    const int button_height = 60;
    const int button_spacing = 5;

    for (int i = 0; i < matchCount; i++) {
        BatteryType* profile = matches[i];
        if (!profile) continue;

        // Create a button for each battery profile
        lv_obj_t* profile_btn = lv_btn_create(container);
        lv_obj_set_size(profile_btn, 900, button_height);
        lv_obj_set_pos(profile_btn, 10, button_y); // Position with X=10, increasing Y
        lv_obj_set_style_bg_color(profile_btn, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
        lv_obj_set_style_border_width(profile_btn, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(profile_btn, lv_color_hex(0x808080), LV_PART_MAIN);

        // Add event handler for profile selection
        lv_obj_add_event_cb(profile_btn, screen2_profile_selected_event_handler, LV_EVENT_CLICKED, (void*)profile);

        // Create label with battery info
        lv_obj_t* profile_label = lv_label_create(profile_btn);
        lv_label_set_text(profile_label, profile->getDisplayName().c_str());
        lv_obj_set_style_text_font(profile_label, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(profile_label, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_center(profile_label);

        Serial.printf("[BATTERY] Added matching profile at Y=%d: %s\n", button_y, profile->getDisplayName().c_str());

        // Move to next button position
        button_y += button_height + button_spacing;
    }
}

// ============================================================================
// UI Update Functions
// ============================================================================

// Update table values (thread-safe)
void update_table_values() {
    // Lock LVGL before updating UI
    lvgl_port_lock(-1);

    if (data_table != nullptr) {
        // Update voltage (column 0)
        lv_table_set_cell_value(data_table, 1, 0,
            String(sensorData.volt, 1).c_str());

        // Update current (column 1)
        lv_table_set_cell_value(data_table, 1, 1,
            String(sensorData.curr, 2).c_str());

        // Update temperature (column 2) - temp1 from CAN data (0.01°C resolution)
        float temp1_celsius = sensorData.temp1 / 100.0f;
        lv_table_set_cell_value(data_table, 1, 2,
            String(temp1_celsius, 1).c_str());

        // Update frequency (column 3) - show current RPM
        float freq_hz = current_frequency / 100.0f;
        float rpm = VFD_FREQ_TO_RPM(freq_hz);
        lv_table_set_cell_value(data_table, 1, 3, String((int)rpm).c_str());

        // Update entry counter (column 4)
        static int entry_counter = 0;
        lv_table_set_cell_value(data_table, 1, 4,
            String(entry_counter).c_str());
    }

    // Unlock LVGL
    lvgl_port_unlock();
}

// Update M2 state display on current screen
void update_m2_state_display() {
    lvgl_port_lock(-1);

    // Update M2 state box on current screen
    if (m2_manager.state_box && m2_manager.state_label) {
        const m2_state_config_t* config = getM2StateConfig(m2_manager.current_state);

        lv_obj_set_style_bg_color(m2_manager.state_box, config->bg_color, LV_PART_MAIN);
        lv_obj_set_style_border_color(m2_manager.state_box, config->border_color, LV_PART_MAIN);
        lv_label_set_text(m2_manager.state_label, config->label_text);
    }

    lvgl_port_unlock();
}

// Function to check WiFi connection status and update UI
void update_wifi_connection_status() {
    if (!wifi_connection_in_progress) {
        return; // No connection in progress
    }

    unsigned long current_time = millis();
    wl_status_t wifi_status = WiFi.status();

    // Check if connection succeeded
    if (wifi_status == WL_CONNECTED) {
        Serial.println("[WIFI] Connected successfully!");
        Serial.printf("[WIFI] Connected to: %s\n", TEST_WIFI_SSID);
        Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());

        // WiFi connected successfully (no UI update needed since screen 12 is removed)

        wifi_connection_in_progress = false;
        wifi_connection_attempts = 0;
        return;
    }

    // Check if connection timed out
    if (current_time - wifi_connection_start_time >= WIFI_CONNECTION_TIMEOUT) {
        Serial.println("[WIFI] Connection failed!");
        Serial.printf("[WIFI] WiFi status: %d\n", wifi_status);

        // WiFi connection failed (no UI update needed since screen 12 is removed)

        wifi_connection_in_progress = false;
        wifi_connection_attempts = 0;
        return;
    }

    // Update progress every 500ms (attempt)
    static unsigned long last_attempt_time = 0;
    if (current_time - last_attempt_time >= 500) {
        wifi_connection_attempts++;
        last_attempt_time = current_time;

        // Print serial progress
        Serial.printf("[WIFI] Connection attempt %d/%d...\n", wifi_connection_attempts, WIFI_MAX_ATTEMPTS);

        // WiFi connection progress (no UI update needed since screen 12 is removed)
    }
}

// Update CAN debug screen with received frame
void update_can_debug_display(uint32_t id, uint8_t* data, uint8_t length) {
    if (screen13_can_frame_label != nullptr && current_screen_id == SCREEN_CAN_DEBUG) {
        lvgl_port_lock(-1);

        // Format frame text
        char frame_text[200];
        sprintf(frame_text, "0x%03X : ", id);

        // Add data bytes to the string
        for (int i = 0; i < length && i < 8; i++) {
            sprintf(frame_text + strlen(frame_text), "%02X ", data[i]);
        }

        // Update current line and move to next
        strcpy(can_debug_lines[can_debug_current_line], frame_text);
        can_debug_current_line = (can_debug_current_line + 1) % CAN_DEBUG_MAX_LINES;

        // Build display text from all lines
        char display_text[1200] = "";
        for (int i = 0; i < CAN_DEBUG_MAX_LINES; i++) {
            if (strlen(can_debug_lines[i]) > 0) {
                if (strlen(display_text) > 0) {
                    strcat(display_text, "\n");
                }
                strcat(display_text, can_debug_lines[i]);
            }
        }

        lv_label_set_text(screen13_can_frame_label, display_text);

        lvgl_port_unlock();
    }
}

// Update time debug screen with M2 RTC time
void update_time_debug_display() {
    if (screen16_time_label != nullptr && current_screen_id == SCREEN_TIME_DEBUG) {
        lvgl_port_lock(-1);

        // Convert day_of_week from M2 format (1=Sunday) to display format (1=Monday)
        // M2: 1=Sunday, 2=Monday, 3=Tuesday, 4=Wednesday, 5=Thursday, 6=Friday, 7=Saturday
        // Display: 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday, 7=Sunday
        uint8_t display_day = m2Time.day_of_week;
        if (display_day == 1) {
            display_day = 7;  // Sunday becomes 7
        } else {
            display_day = display_day - 1;  // Monday=2 becomes 1, etc.
        }

        // Day names for display (1=Monday)
        const char* day_names[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

        // Format time string: 24hr format, day name, date
        char time_text[200];
        sprintf(time_text, "Time: %02d:%02d:%02d\nDay: %d (%s)\nDate: %04d-%02d-%02d",
            m2Time.hour, m2Time.minute, m2Time.second,
            display_day, (display_day >= 1 && display_day <= 7) ? day_names[display_day] : "Unknown",
            m2Time.year, m2Time.month, m2Time.date);

        lv_label_set_text(screen16_time_label, time_text);

        lvgl_port_unlock();
    }
}

// Update WiFi status display on screen 1
void update_screen1_wifi_status() {
    if (screen1_wifi_status_label != nullptr && current_screen_id == SCREEN_HOME) {
        lvgl_port_lock(-1);
        
        bool wifi_connected = (WiFi.status() == WL_CONNECTED);
        
        if (wifi_connected) {
            lv_label_set_text(screen1_wifi_status_label, "WiFi");
            lv_obj_set_style_text_color(screen1_wifi_status_label, lv_color_hex(0x00FF00), LV_PART_MAIN);  // Green
            lv_obj_set_style_bg_color(screen1_wifi_status_label, lv_color_hex(0xE8F5E9), LV_PART_MAIN);  // Light green background
        } else {
            lv_label_set_text(screen1_wifi_status_label, "WiFi");
            lv_obj_set_style_text_color(screen1_wifi_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
            lv_obj_set_style_bg_color(screen1_wifi_status_label, lv_color_hex(0xFFEBEE), LV_PART_MAIN);  // Light red background
        }
        
        lvgl_port_unlock();
    }
}

// Update BLE debug display on screen 17
void update_ble_debug_display() {
    if (screen17_wifi_status_label != nullptr && screen17_ble_status_label != nullptr && screen17_ble_info_label != nullptr && current_screen_id == SCREEN_BLE_DEBUG) {
        lvgl_port_lock(-1);

        // Check WiFi connection status
        bool wifi_connected = (WiFi.status() == WL_CONNECTED);
        
        // Check BLE connection status
        bool ble_connected = bleManager.isConnected();

        // Update WiFi status label with color
        if (wifi_connected) {
            lv_label_set_text(screen17_wifi_status_label, "WiFi Connected");
            lv_obj_set_style_text_color(screen17_wifi_status_label, lv_color_hex(0x00FF00), LV_PART_MAIN);  // Green
        } else {
            lv_label_set_text(screen17_wifi_status_label, "WiFi Disconnected");
            lv_obj_set_style_text_color(screen17_wifi_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
        }

        // Update BLE status label with color
        if (ble_connected) {
            lv_label_set_text(screen17_ble_status_label, "BLE Connected");
            lv_obj_set_style_text_color(screen17_ble_status_label, lv_color_hex(0x00FF00), LV_PART_MAIN);  // Green
        } else {
            lv_label_set_text(screen17_ble_status_label, "BLE Disconnected");
            lv_obj_set_style_text_color(screen17_ble_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
        }

        // Update BLE credentials display
        char info_text[300];
        if (reboot_countdown_active) {
            // Show countdown message
            unsigned long elapsed = millis() - reboot_countdown_start_time;
            unsigned long remaining = (REBOOT_COUNTDOWN_DURATION - elapsed) / 1000;
            if (remaining > 5) remaining = 5;
            if (remaining < 0) remaining = 0;
            unsigned long current_second = 5 - remaining + 1; // 1/5, 2/5, etc.
            
            sprintf(info_text, "SSID: %s\nPassword: %s\n\nCredentials saved!\n\nDevice will reboot to connect to WiFi\nin %lu/6 secs", 
                    ble_ssid.c_str(), ble_key.c_str(), current_second);
        } else if (ble_ssid.length() > 0 || ble_key.length() > 0) {
            sprintf(info_text, "SSID: %s\nPassword: %s\n\nCredentials received via BLE", 
                    ble_ssid.c_str(), ble_key.c_str());
        } else {
            sprintf(info_text, "No credentials received yet.\n\nWaiting for Android app to send:\nSSID|password|CONNECT");
        }
        lv_label_set_text(screen17_ble_info_label, info_text);

        lvgl_port_unlock();
    }
}

// Start reboot countdown (called after WiFi credentials are saved)
void start_reboot_countdown(void) {
    reboot_countdown_active = true;
    reboot_countdown_start_time = millis();
    Serial.println("[REBOOT] Starting 5-second countdown before reboot...");
}

// Process reboot countdown and reboot if time elapsed
void process_reboot_countdown(void) {
    if (reboot_countdown_active) {
        unsigned long elapsed = millis() - reboot_countdown_start_time;
        
        if (elapsed >= REBOOT_COUNTDOWN_DURATION) {
            Serial.println("[REBOOT] Countdown complete, rebooting device...");
            delay(100); // Small delay to ensure serial message is sent
            ESP.restart();
        } else {
            // Update display every second to show countdown
            static unsigned long last_countdown_update = 0;
            if (millis() - last_countdown_update >= 1000) {
                last_countdown_update = millis();
                update_ble_debug_display(); // Update screen 17 with countdown
            }
        }
    }
}

// ============================================================================
// Event Handlers
// ============================================================================

// Screen 1 CAN Debug button event handler
void screen1_can_debug_btnhandler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] Switching to CAN debug screen");
        // Switch to CAN debug screen (no state change needed)
        switch_to_screen(SCREEN_CAN_DEBUG);
    }
}

// Screen 1 Time Debug button event handler
void screen1_time_debug_btnhandler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] Switching to time debug screen");
        // Switch to time debug screen (no state change needed)
        switch_to_screen(SCREEN_TIME_DEBUG);
    }
}

// Screen 1 BLE Debug button event handler
void screen1_ble_debug_btnhandler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] Switching to BLE debug screen");
        // Switch to BLE debug screen (no state change needed)
        switch_to_screen(SCREEN_BLE_DEBUG);
    }
}

void screen2_confirm_agree_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN2] CONFIRM AGREE button pressed - battery confirmed for charging");

        // Send CAN test frame with ID 0x100 (256 decimal)
        uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        if (send_can_frame(HANDSHAKE_FRAME_ID, test_data, 8)) {
            Serial.println("[CAN] Battery confirmation frame sent successfully");
        } else {
            Serial.println("[CAN] Failed to send battery confirmation frame");
        }

        // Show START/STOP buttons, hide the confirmation popup and battery profiles
        lv_obj_clear_flag(screen2_button_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(screen2_confirm_popup, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(screen2_battery_container, LV_OBJ_FLAG_HIDDEN);
    }
}

void screen2_confirm_change_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN2] CONFIRM CHANGE button pressed - returning to battery selection");
        // Hide the confirmation popup (user wants to change battery selection)
        lv_obj_add_flag(screen2_confirm_popup, LV_OBJ_FLAG_HIDDEN);
    }
}

void screen2_profile_selected_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    Serial.printf("[SCREEN2] Profile selection event handler called, code: %d\n", code);

    if(code == LV_EVENT_CLICKED) {
        BatteryType* selected_profile = (BatteryType*)lv_event_get_user_data(e);
        if (selected_profile) {
            Serial.printf("[SCREEN2] Profile selected: %s\n", selected_profile->getDisplayName().c_str());

            // Store selected profile globally for screen 3 display
            selected_battery_profile = selected_profile;

            // Update confirmation popup with selected profile details
            lv_label_set_text_fmt(screen2_confirm_voltage_label, "Voltage: %d V", selected_profile->getRatedVoltage());
            lv_label_set_text_fmt(screen2_confirm_capacity_label, "Capacity: %d Ah", selected_profile->getRatedAh());
            lv_label_set_text_fmt(screen2_confirm_current_label, "Current: %.1f A", selected_profile->getConstCurrent());
            lv_label_set_text_fmt(screen2_confirm_type_label, "Type: %s", getBatteryChemistryName(selected_profile));

            // Show confirmation popup and bring it to foreground
            lv_obj_clear_flag(screen2_confirm_popup, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(screen2_confirm_popup);
        }
    }
}

void screen2_start_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN2] START button pressed - switching to charging screen");

        // Close contactor via CAN bus (M1 -> M2)
        Serial.println("[CONTACTOR] Closing contactor before starting charge...");
        send_contactor_control(CONTACTOR_CLOSE);
        
        // Send RS485 start command to VFD
        rs485_sendStartCommand();
        Serial.println("Start cmd sent to vfd, going to scrn3 now.");

        // Initialize charging control variables
        current_frequency = 0;  // Start at 0 (stopped)
        last_control_update = 0;  // Reset rate limiting
        cc_state_start_time = 0;  // Reset CC timing
        cv_state_start_time = 0;  // Reset CV timing
        cc_state_duration = 0;    // Reset CC duration
        cc_state_duration_for_timer = 0;  // Reset for timer calculation
        
        // Initialize charging start time and reset completion flag
        charging_start_time = millis();
        charging_complete = false;
        final_charging_time_ms = 0;
        final_remaining_time_ms = 0;
        pending_stop_command = false;  // Reset stop command flag

        // Switch to charging start state
        current_app_state = STATE_CHARGING_START;
        switch_to_screen(SCREEN_CHARGING_STARTED);
    }
}

void screen2_reselect_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN2] RE-SELECT button pressed - returning to battery selection");

        // Hide START and RE-SELECT buttons
        lv_obj_add_flag(screen2_button_container, LV_OBJ_FLAG_HIDDEN);

        // Show battery list again with current voltage
        if (sensorData.volt > 0) {
            displayMatchingBatteryProfiles(sensorData.volt, screen2_battery_container);
        } else {
            // Show default 0V profile if no voltage detected
            displayMatchingBatteryProfiles(0.0f, screen2_battery_container);
        }
    }
}

// Generic back button event handler (used on screens 13, 16, 17)
void generic_back_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] BACK button pressed - returning to home screen");
        // Reset state and return to home
        current_app_state = STATE_HOME;
        switch_to_screen(SCREEN_HOME);
    }
}

// Emergency stop event handler (used on screens 3, 4, 5)
void emergency_stop_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[EMERGENCY] Emergency stop button pressed!");
        
        // STEP 1: Send 0 RPM command IMMEDIATELY in button callback (100% triggers)
        Serial.println("[EMERGENCY] Sending 0 RPM command immediately in button callback...");
        rs485_sendFrequencyCommand(0);  // Send 0 Hz
        current_frequency = 0;
        delay(10);  // Give RS485 time to send the command
        
        // Open contactor via CAN bus IMMEDIATELY on emergency stop
        Serial.println("[CONTACTOR] Opening contactor immediately on emergency stop...");
        send_contactor_control(CONTACTOR_OPEN);
        
        // Store final charging time and remaining time before transitioning (if charging was in progress)
        if (charging_start_time > 0 && !charging_complete) {
            final_charging_time_ms = millis() - charging_start_time;
            charging_complete = true;
            Serial.printf("[EMERGENCY] Final charging time: %lu ms (%.2f minutes)\n", 
                         final_charging_time_ms, final_charging_time_ms / 60000.0f);
            
            // Calculate and store final remaining time (if in CV state)
            if (cv_start_time > 0 && cc_state_duration_for_timer > 0) {
                unsigned long cv_elapsed = millis() - cv_start_time;
                unsigned long cv_33_percent_time = cc_state_duration_for_timer / 3;  // 33% of CC time
                unsigned long cv_30_min = 30 * 60 * 1000;  // 30 minutes in ms
                unsigned long target_cv_time = (cv_33_percent_time < cv_30_min) ? cv_33_percent_time : cv_30_min;
                final_remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                Serial.printf("[EMERGENCY] Final remaining time: %lu ms (%.2f minutes)\n", 
                             final_remaining_time_ms, final_remaining_time_ms / 60000.0f);
            }
        }
        
        // Set stop reason
        charge_stop_reason = CHARGE_STOP_EMERGENCY;
        
        // Set flag to send stop command after screen 7 loads
        pending_stop_command = true;
        
        // STEP 2: Switch to emergency stop state and screen
        current_app_state = STATE_EMERGENCY_STOP;
        switch_to_screen(SCREEN_EMERGENCY_STOP);
    }
}

// Home button event handler (used on screens 6, 7)
void home_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[HOME] Home button pressed");
        
        // Reset stop reason
        charge_stop_reason = CHARGE_STOP_NONE;
        
        // Reset timer variables
        charging_start_time = 0;
        cv_start_time = 0;
        cc_state_duration_for_timer = 0;
        pending_stop_command = false;  // Reset stop command flag
        
        // Check if battery is still connected
        if (battery_detected && sensorData.volt >= 9.0f) {
            // Battery still connected - go to battery detected screen
            Serial.println("[HOME] Battery still connected, switching to battery detected screen");
            current_app_state = STATE_BATTERY_DETECTED;
            switch_to_screen(SCREEN_BATTERY_DETECTED);
        } else {
            // Battery removed - go to home screen
            Serial.println("[HOME] Battery removed, switching to home screen");
            current_app_state = STATE_HOME;
            battery_detected = false;
            switch_to_screen(SCREEN_HOME);
        }
    }
}

// ============================================================================
// Screen Creation Functions
// ============================================================================
//screen 1, default screen.
void create_screen_1()
{
    screen_1 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_1, lv_color_hex(0xADD8E6), LV_PART_MAIN);  // Light blue background
    lv_obj_set_style_bg_opa(screen_1, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_1, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_1, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_1);
    lv_label_set_text(title, "GCU 3kW Charger v1.0");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (battery detected, etc)
    status_label = lv_label_create(screen_1);
    lv_label_set_text(status_label, "Connect Battery");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Create or reposition shared data table
    if (data_table == nullptr) {
        // Create table if it doesn't exist
        data_table = lv_table_create(lv_scr_act());  // Create on active screen initially

        // Table properties
        lv_table_set_col_cnt(data_table, 5);
        lv_table_set_row_cnt(data_table, 2);  // Exactly 2 rows

        // Set column widths
        lv_table_set_col_width(data_table, 0, 198);  // Volt
        lv_table_set_col_width(data_table, 1, 198);  // Curr
        lv_table_set_col_width(data_table, 2, 198);  // Temp1
        lv_table_set_col_width(data_table, 3, 198);  // Rpm
        lv_table_set_col_width(data_table, 4, 198);  // Entry

        // Headers (Row 0)
        lv_table_set_cell_value(data_table, 0, 0, "Volt");
        lv_table_set_cell_value(data_table, 0, 1, "Curr");
        lv_table_set_cell_value(data_table, 0, 2, "Temp1");
        lv_table_set_cell_value(data_table, 0, 3, "Rpm");
        lv_table_set_cell_value(data_table, 0, 4, "Entry");

        // Values (Row 1)
        lv_table_set_cell_value(data_table, 1, 0, "--");
        lv_table_set_cell_value(data_table, 1, 1, "--");
        lv_table_set_cell_value(data_table, 1, 2, "--");
        lv_table_set_cell_value(data_table, 1, 3, "--");
        lv_table_set_cell_value(data_table, 1, 4, "-1");

        // Style table - Blue headers, white text
        lv_obj_set_style_bg_color(data_table, lv_color_hex(0x1E88E5), LV_PART_ITEMS);
        lv_obj_set_style_text_color(data_table, lv_color_hex(0xFFFFFF), LV_PART_ITEMS);
        lv_obj_set_style_text_font(data_table, &lv_font_montserrat_28, LV_PART_ITEMS);
        lv_obj_set_style_border_width(data_table, 2, LV_PART_MAIN);
        lv_obj_set_style_border_width(data_table, 1, LV_PART_ITEMS);
        lv_obj_set_style_pad_all(data_table, 10, LV_PART_ITEMS);  // More padding

        // CRITICAL: Disable scrolling on table
        lv_obj_clear_flag(data_table, LV_OBJ_FLAG_SCROLLABLE);

        // Auto-size table based on content
        lv_obj_set_width(data_table, LV_SIZE_CONTENT);
        lv_obj_set_height(data_table, LV_SIZE_CONTENT);
    }

    // Move table to screen_1
    lv_obj_set_parent(data_table, screen_1);
    lv_obj_set_pos(data_table, 12, 110);

    // Battery profiles not applicable for screen 1 - removed container creation

    // Button container for bottom buttons
    lv_obj_t* screen1_button_container = lv_obj_create(screen_1);
    lv_obj_set_size(screen1_button_container, ESP_PANEL_BOARD_WIDTH, 100);
    lv_obj_align(screen1_button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(screen1_button_container, lv_color_hex(0x87CEEB), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen1_button_container, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(screen1_button_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(screen1_button_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(screen1_button_container, LV_OBJ_FLAG_SCROLLABLE);

    // CAN Debug button
    lv_obj_t* screen1_can_debug_btn = lv_btn_create(screen1_button_container);
    lv_obj_set_size(screen1_can_debug_btn, 300, 80);
    lv_obj_set_style_bg_color(screen1_can_debug_btn, lv_color_hex(0xFFA500), LV_PART_MAIN);  // Orange button
    lv_obj_add_event_cb(screen1_can_debug_btn, screen1_can_debug_btnhandler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen1_can_debug_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* screen1_can_debug_label = lv_label_create(screen1_can_debug_btn);
    lv_label_set_text(screen1_can_debug_label, "CAN Debug");
    lv_obj_set_style_text_font(screen1_can_debug_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_can_debug_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen1_can_debug_label);

    // Time Debug button (next to CAN Debug button)
    lv_obj_t* screen1_time_debug_btn = lv_btn_create(screen1_button_container);
    lv_obj_set_size(screen1_time_debug_btn, 300, 80);
    lv_obj_set_style_bg_color(screen1_time_debug_btn, lv_color_hex(0x9370DB), LV_PART_MAIN);  // Medium purple button
    lv_obj_add_event_cb(screen1_time_debug_btn, screen1_time_debug_btnhandler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen1_time_debug_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* screen1_time_debug_label = lv_label_create(screen1_time_debug_btn);
    lv_label_set_text(screen1_time_debug_label, "Time Debug");
    lv_obj_set_style_text_font(screen1_time_debug_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_time_debug_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen1_time_debug_label);

    // BLE Debug button (next to Time Debug button)
    lv_obj_t* screen1_ble_debug_btn = lv_btn_create(screen1_button_container);
    lv_obj_set_size(screen1_ble_debug_btn, 300, 80);
    lv_obj_set_style_bg_color(screen1_ble_debug_btn, lv_color_hex(0x4169E1), LV_PART_MAIN);  // Royal blue button
    lv_obj_add_event_cb(screen1_ble_debug_btn, screen1_ble_debug_btnhandler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen1_ble_debug_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* screen1_ble_debug_label = lv_label_create(screen1_ble_debug_btn);
    lv_label_set_text(screen1_ble_debug_label, "BLE Debug");
    lv_obj_set_style_text_font(screen1_ble_debug_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_ble_debug_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen1_ble_debug_label);

    // v4.50: Add M2 state status box (top-left corner) - Screen 1
    createM2StateBox(screen_1, "screen_1");

    // WiFi status display box (top-right corner) - Screen 1
    screen1_wifi_status_label = lv_label_create(screen_1);
    lv_label_set_text(screen1_wifi_status_label, "WiFi");
    lv_obj_set_style_text_font(screen1_wifi_status_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_wifi_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red initially
    lv_obj_set_style_bg_color(screen1_wifi_status_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White background
    lv_obj_set_style_bg_opa(screen1_wifi_status_label, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen1_wifi_status_label, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen1_wifi_status_label, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(screen1_wifi_status_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen1_wifi_status_label, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_width(screen1_wifi_status_label, 100);
    lv_obj_set_style_text_align(screen1_wifi_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_1); // Removed - handled by screen manager
    Serial.println("[SCREEN] Screen 1 created successfully");
}


//screen 2 - battery detected, charge ready page
void create_screen_2(void) {
    // Create screen 2 (battery detected screen with filtered battery list)
    screen_2 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_2, lv_color_hex(0xDDA0DD), LV_PART_MAIN);  // Light purple background
    lv_obj_set_style_bg_opa(screen_2, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_2, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_2, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_2);
    lv_label_set_text(title, "GCU 3kW Charger v1.0");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (battery detected, charge ready)
    lv_obj_t *status_label_2 = lv_label_create(screen_2);
    lv_label_set_text(status_label_2, "Battery detected - charge ready");
    lv_obj_set_style_text_color(status_label_2, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label_2, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_2, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_2
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_2);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // v4.08: Battery list container (shown with matching profiles on screen 2)
    screen2_battery_container = lv_obj_create(screen_2);
    lv_obj_set_size(screen2_battery_container, 950, 280);
    lv_obj_set_pos(screen2_battery_container, 37, 260);  // Below table
    lv_obj_set_style_bg_color(screen2_battery_container, lv_color_hex(0xF0F0F0), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen2_battery_container, 2, LV_PART_MAIN);
    lv_obj_set_scroll_dir(screen2_battery_container, LV_DIR_VER);  // Vertical scroll

    // v4.08: Button container (hidden by default, shown after profile selection)
    screen2_button_container = lv_obj_create(screen_2);
    lv_obj_set_size(screen2_button_container, ESP_PANEL_BOARD_WIDTH, 100);
    lv_obj_align(screen2_button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(screen2_button_container, lv_color_hex(0x87CEEB), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen2_button_container, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(screen2_button_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(screen2_button_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(screen2_button_container, LV_OBJ_FLAG_HIDDEN);  // Hidden initially
    lv_obj_clear_flag(screen2_button_container, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling

    // RE-SELECT button (in button_container)
    lv_obj_t *screen2_reselect_button = lv_btn_create(screen2_button_container);
    lv_obj_set_size(screen2_reselect_button, 300, 80);
    lv_obj_set_style_bg_color(screen2_reselect_button, lv_color_hex(0xFF6600), LV_PART_MAIN);  // Orange
    lv_obj_add_event_cb(screen2_reselect_button, screen2_reselect_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen2_reselect_button, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling
    lv_obj_t *screen2_reselect_label = lv_label_create(screen2_reselect_button);
    lv_label_set_text(screen2_reselect_label, "RE-SELECT");
    lv_obj_set_style_text_font(screen2_reselect_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen2_reselect_label);

    // START button (in button_container)
    lv_obj_t *screen2_start_button = lv_btn_create(screen2_button_container);
    lv_obj_set_size(screen2_start_button, 300, 80);
    lv_obj_set_style_bg_color(screen2_start_button, lv_color_hex(0x00AA00), LV_PART_MAIN);
    lv_obj_add_event_cb(screen2_start_button, screen2_start_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen2_start_button, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling
    lv_obj_t *screen2_start_label = lv_label_create(screen2_start_button);
    lv_label_set_text(screen2_start_label, "START");
    lv_obj_set_style_text_font(screen2_start_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen2_start_label);

    // Add M2 state status box - screen 2
    createM2StateBox(screen_2, "screen_2");

    // Display 0V battery profiles by default during screen creation
    // Will be refreshed with actual voltage when screen is navigated to
    displayMatchingBatteryProfiles(0.0f, screen2_battery_container);

    // v4.60: Create confirmation popup (light gray background, multiple labels for different fonts)
    screen2_confirm_popup = lv_obj_create(screen_2);
    lv_obj_set_size(screen2_confirm_popup, 750, 500);
    lv_obj_center(screen2_confirm_popup);
    lv_obj_set_style_bg_color(screen2_confirm_popup, lv_color_hex(0xD3D3D3), LV_PART_MAIN);  // Light gray background
    lv_obj_set_style_border_width(screen2_confirm_popup, 4, LV_PART_MAIN);
    lv_obj_set_style_border_color(screen2_confirm_popup, lv_color_hex(0x000000), LV_PART_MAIN);  // Black border
    lv_obj_set_style_radius(screen2_confirm_popup, 15, LV_PART_MAIN);
    lv_obj_add_flag(screen2_confirm_popup, LV_OBJ_FLAG_HIDDEN);  // Hidden by default

    // Title label
    screen2_confirm_title_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_title_label, "Confirm Battery:");
    lv_obj_set_style_text_font(screen2_confirm_title_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_title_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_title_label, LV_ALIGN_TOP_MID, 0, 20);

    // Voltage label (30pt - large and bold-looking)
    screen2_confirm_voltage_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_voltage_label, "Voltage: -- V");
    lv_obj_set_style_text_font(screen2_confirm_voltage_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_voltage_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_voltage_label, LV_ALIGN_TOP_MID, 0, 80);

    // Capacity label (30pt - large and bold-looking)
    screen2_confirm_capacity_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_capacity_label, "Capacity: -- Ah");
    lv_obj_set_style_text_font(screen2_confirm_capacity_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_capacity_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_capacity_label, LV_ALIGN_TOP_MID, 0, 140);

    // Current label (26pt)
    screen2_confirm_current_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_current_label, "Current: -- A");
    lv_obj_set_style_text_font(screen2_confirm_current_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_current_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_current_label, LV_ALIGN_TOP_MID, 0, 210);

    // Type label (26pt)
    screen2_confirm_type_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_type_label, "Type: --");
    lv_obj_set_style_text_font(screen2_confirm_type_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_type_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_type_label, LV_ALIGN_TOP_MID, 0, 270);

    // AGREE button
    screen2_confirm_agree_btn = lv_btn_create(screen2_confirm_popup);
    lv_obj_set_size(screen2_confirm_agree_btn, 300, 90);  // Bigger buttons
    lv_obj_align(screen2_confirm_agree_btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    lv_obj_set_style_bg_color(screen2_confirm_agree_btn, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green
    lv_obj_add_event_cb(screen2_confirm_agree_btn, screen2_confirm_agree_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t *screen2_agree_label = lv_label_create(screen2_confirm_agree_btn);
    lv_label_set_text(screen2_agree_label, "AGREE");
    lv_obj_set_style_text_font(screen2_agree_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen2_agree_label);

    // CHANGE button
    screen2_confirm_change_btn = lv_btn_create(screen2_confirm_popup);
    lv_obj_set_size(screen2_confirm_change_btn, 300, 90);  // Bigger buttons
    lv_obj_align(screen2_confirm_change_btn, LV_ALIGN_BOTTOM_RIGHT, -30, -30);
    lv_obj_set_style_bg_color(screen2_confirm_change_btn, lv_color_hex(0xFF6600), LV_PART_MAIN);  // Orange
    lv_obj_add_event_cb(screen2_confirm_change_btn, screen2_confirm_change_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t *screen2_change_label = lv_label_create(screen2_confirm_change_btn);
    lv_label_set_text(screen2_change_label, "CHANGE");
    lv_obj_set_style_text_font(screen2_change_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen2_change_label);

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_2); // Removed - handled by screen manager
    Serial.println("[SCREEN] Screen 2 created successfully");
}

void create_screen_3(void) {
    // Create screen 3 (charging started screen - similar layout to screen 2)
    screen_3 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_3, lv_color_hex(0x90EE90), LV_PART_MAIN);  // Light green background
    lv_obj_set_style_bg_opa(screen_3, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_3, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_3, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_3);
    lv_label_set_text(title, "Charge Started!");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (charging start - waiting for 1A)
    lv_obj_t *status_label_3 = lv_label_create(screen_3);
    lv_label_set_text(status_label_3, "Starting charge... Waiting for 1A");
    lv_obj_set_style_text_color(status_label_3, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_3, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_3, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_3
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_3);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 50px)
    screen3_battery_details_label = lv_label_create(screen_3);
    lv_label_set_text(screen3_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen3_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen3_battery_details_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(screen3_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 300);  // Moved up 50px (350 -> 300)
    
    // Timer table (2x2) for screen 3 - below battery label, left aligned
    screen3_timer_table = lv_table_create(screen_3);
    lv_table_set_col_cnt(screen3_timer_table, 2);
    lv_table_set_row_cnt(screen3_timer_table, 2);
    lv_table_set_col_width(screen3_timer_table, 0, 200);
    lv_table_set_col_width(screen3_timer_table, 1, 200);
    lv_table_set_cell_value(screen3_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen3_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen3_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen3_timer_table, 1, 1, "");
    lv_obj_set_style_bg_color(screen3_timer_table, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
    lv_obj_set_style_text_font(screen3_timer_table, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(screen3_timer_table, LV_ALIGN_TOP_LEFT, 12, 360);  // Below battery label, left aligned
    lv_obj_clear_flag(screen3_timer_table, LV_OBJ_FLAG_SCROLLABLE);

    // Add M2 state status box - screen 3
    createM2StateBox(screen_3, "screen_3");

    // Emergency Stop button (bottom mid, large and visible)
    lv_obj_t* screen3_emergency_stop_btn = lv_btn_create(screen_3);
    lv_obj_set_size(screen3_emergency_stop_btn, 200, 80);
    lv_obj_align(screen3_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen3_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen3_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen3_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen3_emergency_stop_label = lv_label_create(screen3_emergency_stop_btn);
    lv_label_set_text(screen3_emergency_stop_label, "EMERGENCY\nSTOP");
    lv_obj_set_style_text_font(screen3_emergency_stop_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen3_emergency_stop_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen3_emergency_stop_label);

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_3); // Removed - handled by screen manager
    Serial.println("[SCREEN] Screen 3 created successfully");
}

//screen 4 - Constant Current (CC) mode
void create_screen_4(void) {
    screen_4 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_4, lv_color_hex(0x90EE90), LV_PART_MAIN);  // Light green background
    lv_obj_set_style_bg_opa(screen_4, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_4, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_4, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_4);
    lv_label_set_text(title, "Constant Current Mode");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (CC charging in progress)
    lv_obj_t *status_label_4 = lv_label_create(screen_4);
    lv_label_set_text(status_label_4, "CC Charging in progress...");
    lv_obj_set_style_text_color(status_label_4, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_4, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_4, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_4
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_4);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 50px)
    screen4_battery_details_label = lv_label_create(screen_4);
    lv_label_set_text(screen4_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen4_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen4_battery_details_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(screen4_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 300);  // Moved up 50px (350 -> 300)
    
    // Timer table (2x2) for screen 4 - below battery label, left aligned
    screen4_timer_table = lv_table_create(screen_4);
    lv_table_set_col_cnt(screen4_timer_table, 2);
    lv_table_set_row_cnt(screen4_timer_table, 2);
    lv_table_set_col_width(screen4_timer_table, 0, 200);
    lv_table_set_col_width(screen4_timer_table, 1, 200);
    lv_table_set_cell_value(screen4_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen4_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen4_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen4_timer_table, 1, 1, "");
    lv_obj_set_style_bg_color(screen4_timer_table, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
    lv_obj_set_style_text_font(screen4_timer_table, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(screen4_timer_table, LV_ALIGN_TOP_LEFT, 12, 360);  // Below battery label, left aligned
    lv_obj_clear_flag(screen4_timer_table, LV_OBJ_FLAG_SCROLLABLE);

    // Add M2 state status box - screen 4
    createM2StateBox(screen_4, "screen_4");

    // Emergency Stop button (bottom mid, large and visible)
    lv_obj_t* screen4_emergency_stop_btn = lv_btn_create(screen_4);
    lv_obj_set_size(screen4_emergency_stop_btn, 200, 80);
    lv_obj_align(screen4_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen4_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen4_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen4_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen4_emergency_stop_label = lv_label_create(screen4_emergency_stop_btn);
    lv_label_set_text(screen4_emergency_stop_label, "EMERGENCY\nSTOP");
    lv_obj_set_style_text_font(screen4_emergency_stop_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen4_emergency_stop_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen4_emergency_stop_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 4 (CC Mode) created successfully");
}

//screen 5 - Constant Voltage (CV) mode
void create_screen_5(void) {
    screen_5 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_5, lv_color_hex(0x90EE90), LV_PART_MAIN);  // Light green background
    lv_obj_set_style_bg_opa(screen_5, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_5, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_5, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_5);
    lv_label_set_text(title, "Constant Voltage Mode");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (CV charging in progress)
    lv_obj_t *status_label_5 = lv_label_create(screen_5);
    lv_label_set_text(status_label_5, "CV Charging in progress...");
    lv_obj_set_style_text_color(status_label_5, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_5, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_5, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_5
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_5);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 50px)
    screen5_battery_details_label = lv_label_create(screen_5);
    lv_label_set_text(screen5_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen5_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen5_battery_details_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(screen5_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 300);  // Moved up 50px (350 -> 300)
    
    // Timer table (2x2) for screen 5 - shows total time and remaining time, below battery label, left aligned
    screen5_timer_table = lv_table_create(screen_5);
    lv_table_set_col_cnt(screen5_timer_table, 2);
    lv_table_set_row_cnt(screen5_timer_table, 2);
    lv_table_set_col_width(screen5_timer_table, 0, 200);
    lv_table_set_col_width(screen5_timer_table, 1, 200);
    lv_table_set_cell_value(screen5_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen5_timer_table, 0, 1, "Remaining");
    lv_table_set_cell_value(screen5_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen5_timer_table, 1, 1, "00:00");
    lv_obj_set_style_bg_color(screen5_timer_table, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
    lv_obj_set_style_text_font(screen5_timer_table, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(screen5_timer_table, LV_ALIGN_TOP_LEFT, 12, 360);  // Below battery label, left aligned
    lv_obj_clear_flag(screen5_timer_table, LV_OBJ_FLAG_SCROLLABLE);

    // Add M2 state status box - screen 5
    createM2StateBox(screen_5, "screen_5");

    // Emergency Stop button (bottom mid, large and visible)
    lv_obj_t* screen5_emergency_stop_btn = lv_btn_create(screen_5);
    lv_obj_set_size(screen5_emergency_stop_btn, 200, 80);
    lv_obj_align(screen5_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen5_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen5_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen5_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen5_emergency_stop_label = lv_label_create(screen5_emergency_stop_btn);
    lv_label_set_text(screen5_emergency_stop_label, "EMERGENCY\nSTOP");
    lv_obj_set_style_text_font(screen5_emergency_stop_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen5_emergency_stop_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen5_emergency_stop_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 5 (CV Mode) created successfully");
}

//screen 6 - Charging complete
void create_screen_6(void) {
    screen_6 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_6, lv_color_hex(0x90EE90), LV_PART_MAIN);  // Light green background
    lv_obj_set_style_bg_opa(screen_6, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_6, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_6, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_6);
    lv_label_set_text(title, "Charging Complete!");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label
    lv_obj_t *status_label_6 = lv_label_create(screen_6);
    lv_label_set_text(status_label_6, "Battery charging completed successfully");
    lv_obj_set_style_text_color(status_label_6, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_6, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_6, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_6
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_6);
        lv_obj_set_pos(data_table, 12, 110);
    }
    
    // Timer table (2x2) for screen 6 - shows total time and remaining time, left aligned
    screen6_timer_table = lv_table_create(screen_6);
    lv_table_set_col_cnt(screen6_timer_table, 2);
    lv_table_set_row_cnt(screen6_timer_table, 2);
    lv_table_set_col_width(screen6_timer_table, 0, 200);
    lv_table_set_col_width(screen6_timer_table, 1, 200);
    lv_table_set_cell_value(screen6_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen6_timer_table, 0, 1, "Remaining");
    lv_table_set_cell_value(screen6_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen6_timer_table, 1, 1, "00:00");
    lv_obj_set_style_bg_color(screen6_timer_table, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
    lv_obj_set_style_text_font(screen6_timer_table, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(screen6_timer_table, LV_ALIGN_TOP_LEFT, 12, 300);  // Left aligned, moved up
    lv_obj_clear_flag(screen6_timer_table, LV_OBJ_FLAG_SCROLLABLE);

    // Add M2 state status box - screen 6
    createM2StateBox(screen_6, "screen_6");

    // Home button (bottom mid)
    lv_obj_t* screen6_home_btn = lv_btn_create(screen_6);
    lv_obj_set_size(screen6_home_btn, 200, 80);
    lv_obj_align(screen6_home_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen6_home_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN);  // Blue
    lv_obj_add_event_cb(screen6_home_btn, home_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen6_home_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen6_home_label = lv_label_create(screen6_home_btn);
    lv_label_set_text(screen6_home_label, "Home");
    lv_obj_set_style_text_font(screen6_home_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen6_home_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen6_home_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 6 (Charging Complete) created successfully");
}

//screen 7 - Emergency stop
void create_screen_7(void) {
    screen_7 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_7, lv_color_hex(0xFF6B6B), LV_PART_MAIN);  // Light red background
    lv_obj_set_style_bg_opa(screen_7, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_7, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_7, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_7);
    lv_label_set_text(title, "EMERGENCY STOP");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label
    lv_obj_t *status_label_7 = lv_label_create(screen_7);
    lv_label_set_text(status_label_7, "Charging stopped by user");
    lv_obj_set_style_text_color(status_label_7, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
    lv_obj_set_style_text_font(status_label_7, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_7, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_7
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_7);
        lv_obj_set_pos(data_table, 12, 110);
    }
    
    // Timer table (2x2) for screen 7 - shows total time, left aligned
    screen7_timer_table = lv_table_create(screen_7);
    lv_table_set_col_cnt(screen7_timer_table, 2);
    lv_table_set_row_cnt(screen7_timer_table, 2);
    lv_table_set_col_width(screen7_timer_table, 0, 200);
    lv_table_set_col_width(screen7_timer_table, 1, 200);
    lv_table_set_cell_value(screen7_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen7_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen7_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen7_timer_table, 1, 1, "");
    lv_obj_set_style_bg_color(screen7_timer_table, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
    lv_obj_set_style_text_font(screen7_timer_table, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(screen7_timer_table, LV_ALIGN_TOP_LEFT, 12, 300);  // Left aligned, moved up
    lv_obj_clear_flag(screen7_timer_table, LV_OBJ_FLAG_SCROLLABLE);

    // Add M2 state status box - screen 7
    createM2StateBox(screen_7, "screen_7");

    // Home button (bottom mid)
    lv_obj_t* screen7_home_btn = lv_btn_create(screen_7);
    lv_obj_set_size(screen7_home_btn, 200, 80);
    lv_obj_align(screen7_home_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen7_home_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN);  // Blue
    lv_obj_add_event_cb(screen7_home_btn, home_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen7_home_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen7_home_label = lv_label_create(screen7_home_btn);
    lv_label_set_text(screen7_home_label, "Home");
    lv_obj_set_style_text_font(screen7_home_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen7_home_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen7_home_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 7 (Emergency Stop) created successfully");
}



//screen 13 - CAN debug screen
void create_screen_13(void) {
    screen_13 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_13, lv_color_hex(0x90EE90), LV_PART_MAIN);  // Light green background
    lv_obj_set_style_bg_opa(screen_13, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_13, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_13, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_13);
    lv_label_set_text(title, "CAN Debug");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label
    lv_obj_t *status_label = lv_label_create(screen_13);
    lv_label_set_text(status_label, "Received CAN Frames");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x000000), LV_PART_MAIN);  // Black for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 60);

    // CAN frames display area (scrollable container)
    lv_obj_t* can_frames_container = lv_obj_create(screen_13);
    lv_obj_set_size(can_frames_container, 990, 340);
    lv_obj_set_pos(can_frames_container, 12, 240);  // Below table, aligned with table X position
    lv_obj_set_style_bg_color(can_frames_container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(can_frames_container, 2, LV_PART_MAIN);
    lv_obj_set_scroll_dir(can_frames_container, LV_DIR_VER);  // Vertical scroll

    // CAN frame display label (will be updated dynamically)
    screen13_can_frame_label = lv_label_create(can_frames_container);
    lv_label_set_text(screen13_can_frame_label, "No CAN frames received yet...\nWaiting for CAN data...");
    lv_obj_set_style_text_font(screen13_can_frame_label, &lv_font_montserrat_28, LV_PART_MAIN);  // Font 28 as requested
    lv_obj_set_style_text_color(screen13_can_frame_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen13_can_frame_label, LV_ALIGN_TOP_LEFT, 10, 10);

    // Initialize CAN debug lines
    memset(can_debug_lines, 0, sizeof(can_debug_lines));
    can_debug_current_line = 0;

    // Back button (top right)
    lv_obj_t *screen13_back_btn = lv_btn_create(screen_13);
    lv_obj_set_size(screen13_back_btn, 100, 50);
    lv_obj_align(screen13_back_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(screen13_back_btn, lv_color_hex(0xFF4444), LV_PART_MAIN);  // Red back button
    lv_obj_add_event_cb(screen13_back_btn, generic_back_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(screen13_back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);

    // Add M2 state status box (same position as screen 1)
    createM2StateBox(screen_13, "screen_13");

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_13); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 13 (CAN Debug) created successfully");
}

//screen 16 - Time debug screen
void create_screen_16(void) {
    screen_16 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_16, lv_color_hex(0xADD8E6), LV_PART_MAIN);  // Light blue background
    lv_obj_set_style_bg_opa(screen_16, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_16, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_16, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_16);
    lv_label_set_text(title, "Time Debug");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label
    lv_obj_t *status_label = lv_label_create(screen_16);
    lv_label_set_text(status_label, "M2 RTC Time Display");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x000000), LV_PART_MAIN);  // Black for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Time display label (will be updated dynamically)
    screen16_time_label = lv_label_create(screen_16);
    lv_label_set_text(screen16_time_label, "Waiting for time data from M2...");
    lv_obj_set_style_text_font(screen16_time_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen16_time_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen16_time_label, LV_ALIGN_CENTER, 0, 0);

    // Back button (top right)
    lv_obj_t *screen16_back_btn = lv_btn_create(screen_16);
    lv_obj_set_size(screen16_back_btn, 100, 50);
    lv_obj_align(screen16_back_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(screen16_back_btn, lv_color_hex(0xFF4444), LV_PART_MAIN);  // Red back button
    lv_obj_add_event_cb(screen16_back_btn, generic_back_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(screen16_back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);

    // Add M2 state status box (same position as screen 1)
    createM2StateBox(screen_16, "screen_16");

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_16); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 16 (Time Debug) created successfully");
}

//screen 17 - BLE debug screen
void create_screen_17(void) {
    screen_17 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_17, lv_color_hex(0xADD8E6), LV_PART_MAIN);  // Light blue background
    lv_obj_set_style_bg_opa(screen_17, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_17, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_17, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_17);
    lv_label_set_text(title, "BLE Debug");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);  

    // WiFi connection status label (first line, left side)
    screen17_wifi_status_label = lv_label_create(screen_17);
    lv_label_set_text(screen17_wifi_status_label, "WiFi Disconnected");
    lv_obj_set_style_text_font(screen17_wifi_status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen17_wifi_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red initially
    lv_obj_align(screen17_wifi_status_label, LV_ALIGN_TOP_LEFT, 20, 90);  // Moved down by 30px (60 -> 90)

    // BLE connection status label (first line, center aligned)
    screen17_ble_status_label = lv_label_create(screen_17);
    lv_label_set_text(screen17_ble_status_label, "BLE Disconnected");
    lv_obj_set_style_text_font(screen17_ble_status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen17_ble_status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red initially
    lv_obj_align(screen17_ble_status_label, LV_ALIGN_TOP_MID, 0, 90);  // Moved down by 30px (60 -> 90), center aligned

    // BLE info display label (SSID and password)
    screen17_ble_info_label = lv_label_create(screen_17);
    lv_label_set_text(screen17_ble_info_label, "No credentials received yet...");
    lv_obj_set_style_text_font(screen17_ble_info_label, &lv_font_montserrat_26, LV_PART_MAIN);  // Changed from 20 to 26
    lv_obj_set_style_text_color(screen17_ble_info_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen17_ble_info_label, LV_ALIGN_TOP_LEFT, 20, 150);  // Moved down by 30px (120 -> 150)
    lv_obj_set_width(screen17_ble_info_label, 960);  // Set width for text wrapping
    lv_label_set_long_mode(screen17_ble_info_label, LV_LABEL_LONG_WRAP);

    // Back button (top right)
    lv_obj_t *screen17_back_btn = lv_btn_create(screen_17);
    lv_obj_set_size(screen17_back_btn, 100, 50);
    lv_obj_align(screen17_back_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(screen17_back_btn, lv_color_hex(0xFF4444), LV_PART_MAIN);  // Red back button
    lv_obj_add_event_cb(screen17_back_btn, generic_back_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(screen17_back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);

    // Add M2 state status box (same position as screen 1)
    createM2StateBox(screen_17, "screen_17");

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_17); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 17 (BLE Debug) created successfully");
}
