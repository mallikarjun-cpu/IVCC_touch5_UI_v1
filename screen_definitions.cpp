#include "screen_definitions.h"
#include "can_twai.h"
#include "sd_logging.h"
#include "esp_panel_board_custom_conf.h"
#include "lvgl_v8_port.h"
#include "rs485_vfdComs.h"
#include <Arduino.h>
#include <esp_heap_caps.h>

// External sensor data
extern struct sensor_data {
    float volt;
    float curr;
    int32_t temp1;
    int32_t temp2;
    int32_t temp3;
    int32_t temp4;
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
lv_obj_t* screen_8 = nullptr; //screen 8 - Voltage saturation detected
lv_obj_t* screen_13 = nullptr; //screen 13 - CAN debug screen
lv_obj_t* screen_16 = nullptr; //screen 16 - Time debug screen
lv_obj_t* screen_18 = nullptr; //screen 18 - M2 connection failed or lost

// Shared UI objects (reused between screens)
static lv_obj_t* status_label = nullptr;
static lv_obj_t* data_table = nullptr;  // Shared table shown on all screens
// Battery container references for screens that need profiles
static lv_obj_t* screen2_battery_container = nullptr;
static lv_obj_t* screen2_button_container = nullptr;

// Selected battery profile for screen 3, 4, 5, 8 display
static BatteryType* selected_battery_profile = nullptr;
static lv_obj_t* screen3_battery_details_label = nullptr;
static lv_obj_t* screen4_battery_details_label = nullptr;
static lv_obj_t* screen5_battery_details_label = nullptr;
static lv_obj_t* screen3_temp_label = nullptr;
static lv_obj_t* screen4_temp_label = nullptr;
static lv_obj_t* screen5_temp_label = nullptr;
static lv_obj_t* screen8_battery_details_label = nullptr;
static lv_obj_t* screen8_temp_label = nullptr;

// Timer variables for charging screens
static unsigned long charging_start_time = 0;  // When charging started (screen 3)
static unsigned long cv_start_time = 0;        // When CV state started (screen 5)
static unsigned long cc_state_duration_for_timer = 0;  // CC state duration for remaining time calc
static unsigned long precharge_duration_for_timer = 0;  // Precharge duration (set at precharge->CC transition) for CV time calc
static unsigned long final_charging_time_ms = 0;  // Final charging time when complete (stops updating)
static unsigned long final_remaining_time_ms = 0;  // Final remaining time when complete (stops updating)
static bool charging_complete = false;  // Flag to stop timer updates after completion
static bool pending_stop_command = false;  // Flag to send stop command after screen loads
static bool current_flow_start = false;  // True after current >= 1.1 A in step 1; used for disconnect and timeout

// Ah calculation variables
static float accumulated_ah = 0.0f;  // Accumulated Ah (default 0.0)
static unsigned long last_ah_update_time = 0;  // Last time Ah was updated (for rate limiting)
static unsigned long last_rtc_update_time = 0;  // Last time RTC time was updated (for rate limiting, 2Hz = 500ms)

// Max value tracking during charge
static float max_current_during_charge = 0.0f;
static float max_voltage_during_charge = 0.0f;
static uint32_t current_charge_serial = 0;  // Serial number for current charge cycle

// Log number for SD card display
static int32_t log_num_sdhc = -1;  // Latest complete log number (default -1)

// Timer table objects for screens 3, 4, 5, 6, 7, 8
static lv_obj_t* screen3_timer_table = nullptr;
static lv_obj_t* screen4_timer_table = nullptr;
static lv_obj_t* screen5_timer_table = nullptr;
static lv_obj_t* screen6_timer_table = nullptr;
static lv_obj_t* screen6_battery_details_label = nullptr;
static lv_obj_t* screen7_timer_table = nullptr;
static lv_obj_t* screen7_battery_details_label = nullptr;
static lv_obj_t* screen8_timer_table = nullptr;

// screen 2 confirmation popup
static lv_obj_t* screen2_confirm_popup = nullptr;
static lv_obj_t* screen2_confirm_title_label = nullptr;
static lv_obj_t* screen2_confirm_battery_info_label = nullptr;  // Battery info (voltage, Ah)
static lv_obj_t* screen2_confirm_voltage_label = nullptr;  // TV label
static lv_obj_t* screen2_confirm_capacity_label = nullptr;  // TC label
static lv_obj_t* screen2_confirm_current_label = nullptr;
static lv_obj_t* screen2_confirm_type_label = nullptr;
static lv_obj_t* screen2_confirm_agree_btn = nullptr;
static lv_obj_t* screen2_confirm_change_btn = nullptr;

// screen 2 confirmed battery display label (below table)
static lv_obj_t* screen2_confirmed_battery_label = nullptr;

// screen 6 and 7 remove battery popup
static lv_obj_t* screen6_status_label = nullptr;  // Status label for screen 6 (dynamic based on stop reason)
static lv_obj_t* screen6_remove_battery_popup = nullptr;
static lv_obj_t* screen6_remove_battery_label = nullptr;
static lv_obj_t* screen7_status_label = nullptr;  // Status label for screen 7 (dynamic based on stop reason)
static lv_obj_t* screen7_remove_battery_popup = nullptr;
static lv_obj_t* screen7_remove_battery_label = nullptr;

// screen 1 RTC time display
static lv_obj_t* screen1_rtc_time_label = nullptr;

// screen 13 CAN frame display
static lv_obj_t* screen13_can_frame_label = nullptr;
#define CAN_DEBUG_MAX_LINES 7
static char can_debug_lines[CAN_DEBUG_MAX_LINES][200];
static int can_debug_current_line = 0;

// screen 16 time display
static lv_obj_t* screen16_time_label = nullptr;

// screen 18 M2 lost - RTC time display (same position/rate as screen 1)
static lv_obj_t* screen18_rtc_time_label = nullptr;

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

// Voltage saturation detection variables (3kW)
static float base_volt_satu_ref = 0.0f;              // Base voltage reference when entering CC stage
static float present_volt_satu_check = 0.0f;         // Present voltage for saturation check
static unsigned long last_voltage_saturation_check_time = 0;  // Last time voltage saturation was checked
static float voltage_saturation_detected_voltage = 0.0f;      // Voltage when saturation was detected
static unsigned long voltage_saturation_cv_start_time = 0;   // CV start time for screen 8 state

// M2 heartbeat: frame 101 based (check every 1s after 6s grace; if no 101 for 2100ms -> lost)
static bool m2_connection_lost = false;

// Forward declarations for battery profile functions
void displayMatchingBatteryProfiles(float detectedVoltage, lv_obj_t* container);

// Forward declaration for emergency stop handler
void emergency_stop_event_handler(lv_event_t * e);
// Forward declaration for home button handler
void home_button_event_handler(lv_event_t * e);
// Forward declaration for Ah update function
void update_accumulated_ah(void);

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
    create_screen_8();   // Voltage saturation detected screen
    create_screen_18();  // M2 connection failed or lost screen
#if CAN_RTC_DEBUG
    create_screen_13();  // CAN debug screen
    create_screen_16();  // Time debug screen
#endif // CAN_RTC_DEBUG

    // Start with home screen
    switch_to_screen(SCREEN_HOME);

    Serial.println("[SCREEN] All screens initialized");
}

// Switch to a specific screen
void switch_to_screen(screen_id_t screen_id) {
    // Once on screen 18 (M2 lost), no navigation away - restart only
    if (current_screen_id == SCREEN_M2_LOST && screen_id != SCREEN_M2_LOST) {
        return;
    }

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
        case SCREEN_VOLTAGE_SATURATION:
            target_screen = screen_8;
            break;
        case SCREEN_M2_LOST:
            target_screen = screen_18;
            break;
#if CAN_RTC_DEBUG
        case SCREEN_CAN_DEBUG:
            target_screen = screen_13;
            break;
        case SCREEN_TIME_DEBUG:
            target_screen = screen_16;
            break;
#endif // CAN_RTC_DEBUG
        default:
            Serial.printf("[SCREEN] ERROR: Invalid screen ID %d\n", screen_id);
            return;
    }

    if (target_screen != nullptr) {
        // On first entry to M2 lost screen only: 0 rpm, stop motor, open contactor (once)
        if (screen_id == SCREEN_M2_LOST && current_screen_id != SCREEN_M2_LOST) {
            rs485_sendFrequencyCommand(0);
            delay(5);
            Serial.println("[M2] 0 rpm sent");
            rs485_sendStopCommand();
            delay(5);
            Serial.println("[M2] Stop motor sent");
            send_contactor_control(CONTACTOR_OPEN);
            Serial.println("[M2] Contactor open sent");
        }

        // Lock LVGL for entire switch (move table + load screen). Prevents first-boot overlay
        // where screen 18 labels drew over screen 1 background/table when update_m2_state_display was removed.
        lvgl_port_lock(-1);

        // Update current screen tracking
        current_screen_id = screen_id;

        // Move shared UI elements to the new screen
        if (data_table != nullptr) {
            lv_obj_set_parent(data_table, target_screen);
            lv_obj_set_pos(data_table, 12, 110);
            lv_obj_clear_flag(data_table, LV_OBJ_FLAG_HIDDEN); // Make sure table is visible
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
                // Reset screen 2 UI state when switching to it - ensure clean state
                // Hide buttons and popup, show battery list
                if (screen2_button_container != nullptr) {
                    lv_obj_add_flag(screen2_button_container, LV_OBJ_FLAG_HIDDEN);
                }
                if (screen2_confirm_popup != nullptr) {
                    lv_obj_add_flag(screen2_confirm_popup, LV_OBJ_FLAG_HIDDEN);
                }
                // Hide confirmed battery label when switching to screen 2 (will show again if battery is confirmed)
                if (screen2_confirmed_battery_label != nullptr) {
                    lv_obj_add_flag(screen2_confirmed_battery_label, LV_OBJ_FLAG_HIDDEN);
                }
                
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

        // Set battery details label once on entry to screens 3, 4, 5 (won't change during charge)
        if (screen_id == SCREEN_CHARGING_STARTED && screen3_battery_details_label != nullptr) {
            if (selected_battery_profile != nullptr) {
                char details_text[180];
                sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                        selected_battery_profile->getBatteryName().c_str(),
                        selected_battery_profile->getDisplayName().c_str(),
                        selected_battery_profile->getCutoffVoltage(),
                        selected_battery_profile->getConstCurrent());
                lv_label_set_text(screen3_battery_details_label, details_text);
            } else {
                lv_label_set_text(screen3_battery_details_label, "Selected Battery: None");
            }
        }
        if (screen_id == SCREEN_CHARGING_CC && screen4_battery_details_label != nullptr) {
            if (selected_battery_profile != nullptr) {
                char details_text[180];
                sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                        selected_battery_profile->getBatteryName().c_str(),
                        selected_battery_profile->getDisplayName().c_str(),
                        selected_battery_profile->getCutoffVoltage(),
                        selected_battery_profile->getConstCurrent());
                lv_label_set_text(screen4_battery_details_label, details_text);
            } else {
                lv_label_set_text(screen4_battery_details_label, "Selected Battery: None");
            }
        }
        if (screen_id == SCREEN_CHARGING_CV && screen5_battery_details_label != nullptr) {
            if (selected_battery_profile != nullptr) {
                char details_text[180];
                sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                        selected_battery_profile->getBatteryName().c_str(),
                        selected_battery_profile->getDisplayName().c_str(),
                        selected_battery_profile->getCutoffVoltage(),
                        selected_battery_profile->getConstCurrent());
                lv_label_set_text(screen5_battery_details_label, details_text);
            } else {
                lv_label_set_text(screen5_battery_details_label, "Selected Battery: None");
            }
        }
        // Set battery details label on entry to screens 6, 7
        if (screen_id == SCREEN_CHARGING_COMPLETE && screen6_battery_details_label != nullptr) {
            if (selected_battery_profile != nullptr) {
                char details_text[180];
                sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                        selected_battery_profile->getBatteryName().c_str(),
                        selected_battery_profile->getDisplayName().c_str(),
                        selected_battery_profile->getCutoffVoltage(),
                        selected_battery_profile->getConstCurrent());
                lv_label_set_text(screen6_battery_details_label, details_text);
            } else {
                lv_label_set_text(screen6_battery_details_label, "Selected Battery: None");
            }
        }
        if (screen_id == SCREEN_EMERGENCY_STOP && screen7_battery_details_label != nullptr) {
            if (selected_battery_profile != nullptr) {
                char details_text[180];
                sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                        selected_battery_profile->getBatteryName().c_str(),
                        selected_battery_profile->getDisplayName().c_str(),
                        selected_battery_profile->getCutoffVoltage(),
                        selected_battery_profile->getConstCurrent());
                lv_label_set_text(screen7_battery_details_label, details_text);
            } else {
                lv_label_set_text(screen7_battery_details_label, "Selected Battery: None");
            }
        }

        // Load the screen
        lv_scr_load(target_screen);
        // Force full redraw of new screen so previous screen does not linger (first USB boot fix).
        lv_obj_invalidate(target_screen);

        lvgl_port_unlock();

        Serial.printf("[SCREEN] Switched to screen %d\n", screen_id);
    } else {
        Serial.printf("[SCREEN] ERROR: Screen %d not initialized\n", screen_id);
    }
}






// ============================================================================
// Charging Control Function start
// ============================================================================
void update_charging_control() {
    // Only run in charging states
    if (current_app_state != STATE_CHARGING_START && 
        current_app_state != STATE_CHARGING_CC && 
        current_app_state != STATE_CHARGING_CV &&
        current_app_state != STATE_CHARGING_VOLTAGE_SATURATION) { return; }
    
    // Check if battery profile is selected
    if (selected_battery_profile == nullptr) { return; }
    
    // Rate limiting: update once per second
    const unsigned long CONTROL_UPDATE_INTERVAL = 1000; // 1 second
    if (millis() - last_control_update < CONTROL_UPDATE_INTERVAL) {
        return;
    }
    last_control_update = millis();
    
    // Temperature check: Monitor temp1 and temp2 during charging states
    // Convert from 0.01°C units to Celsius
    float temp1_celsius = sensorData.temp1 / 100.0f;
    float temp2_celsius = sensorData.temp2 / 100.0f;
    
    // Check if either temperature exceeds threshold
    if (temp1_celsius > MAX_TEMP_THRESHOLD || temp2_celsius > MAX_TEMP_THRESHOLD) {
        Serial.printf("[TEMP] High temperature detected! Temp1=%.2f°C, Temp2=%.2f°C, Threshold=%.1f°C\n",
                     temp1_celsius, temp2_celsius, MAX_TEMP_THRESHOLD);
        Serial.println("[TEMP] Triggering emergency stop due to high temperature");
        
        // STEP 1: Send 0 RPM command IMMEDIATELY
        Serial.println("[TEMP] Sending 0 RPM command immediately...");
        rs485_sendFrequencyCommand(0);  // Send 0 Hz
        current_frequency = 0;
        delay(10);  // Give RS485 time to send the command
       
        // STEP 2: Open contactor via CAN bus IMMEDIATELY
        Serial.println("[TEMP] Opening contactor immediately on high temperature...");
        send_contactor_control(CONTACTOR_OPEN);
        
        // STEP 3: Store final charging time before transitioning (if charging was in progress)
        if (charging_start_time > 0 && !charging_complete) {
            final_charging_time_ms = millis() - charging_start_time;
            charging_complete = true;
            Serial.printf("[TEMP] Final charging time: %lu ms (%.2f minutes)\n", 
                         final_charging_time_ms, final_charging_time_ms / 60000.0f);
            
            // Calculate and store final remaining time (if in CV state)
            if (cv_start_time > 0 && cc_state_duration_for_timer > 0) {
                unsigned long cv_elapsed = millis() - cv_start_time;
                unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
                final_remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                Serial.printf("[TEMP] Final remaining time: %lu ms (%.2f minutes)\n", 
                             final_remaining_time_ms, final_remaining_time_ms / 60000.0f);
            }
        }
        
        // STEP 4: Set stop reason to high temperature
        charge_stop_reason = CHARGE_STOP_HIGH_TEMP;
        current_flow_start = false;
        
        // Log charge complete
        if (sd_logging_initialized) {
            logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                             final_charging_time_ms, accumulated_ah, charge_stop_reason);
        }
        
        // STEP 5: Set flag to send stop command after screen 7 loads
        pending_stop_command = true;
        
        // STEP 6: Switch to emergency stop state and screen
        current_app_state = STATE_EMERGENCY_STOP;
        switch_to_screen(SCREEN_EMERGENCY_STOP);
        
        // Exit early - don't continue with normal charging control
        return;
    }
    
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
    // ============================================================================
    // [1] STATE_CHARGING_START: Maintain PRECHARGE_AMPS (2A) for PRECHARGE_TIME (3 minutes), then transition to CC
    // ============================================================================
    if (current_app_state == STATE_CHARGING_START) {
        // Set current_flow_start once current >= 1.5 A (allows 0 A before that; 1.0 A below = disconnect after flow)
        if (safe_actual_current >= 1.5f) {
            current_flow_start = true;
        }
        // Battery disconnected: current had flowed but now dropped below 1.0 A
        if (current_flow_start && safe_actual_current < 1.0f) {
            Serial.println("[CHARGING] Battery disconnected (current < 1.0 A after flow), emergency stop");
            send_contactor_control(CONTACTOR_OPEN);
            delay(10);
            rs485_sendFrequencyCommand(0);
            current_frequency = 0;
            delay(10);
            if (charging_start_time > 0 && !charging_complete) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
            }
            charge_stop_reason = CHARGE_STOP_BATTERY_DISCONNECTED;
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge,
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            current_flow_start = false;
            pending_stop_command = true;
            current_app_state = STATE_EMERGENCY_STOP;
            switch_to_screen(SCREEN_EMERGENCY_STOP);
            return;
        }
        // Step 1 safety: no current flow within timeout, or RPM over limit
        unsigned long step1_elapsed = (charging_start_time > 0) ? (millis() - charging_start_time) : 0;
        float step1_rpm = VFD_FREQ_TO_RPM(current_frequency / 100.0f);

        if ((step1_elapsed >= PRECHARGE_CURRENT_FLOW_TIMEOUT_MS && !current_flow_start) ||
        ((step1_rpm > (float)PRECHARGE_RPM_LIMIT) && !current_flow_start)) {
            Serial.println("[CHARGING] Volt or current error (no flow in time or RPM > limit), emergency stop");
            send_contactor_control(CONTACTOR_OPEN);
            delay(10);
            rs485_sendFrequencyCommand(0);
            current_frequency = 0;
            delay(10);
            if (charging_start_time > 0 && !charging_complete) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
            }
            charge_stop_reason = CHARGE_STOP_VOLT_OR_CURRENT_ERROR;
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge,
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            current_flow_start = false;
            pending_stop_command = true;
            current_app_state = STATE_EMERGENCY_STOP;
            switch_to_screen(SCREEN_EMERGENCY_STOP);
            return;
        }
        // Use CC logic to maintain current at PRECHARGE_AMPS (2A) - like CC but fixed at precharge level
        uint16_t precharge_target_0_01A = (uint16_t)(PRECHARGE_AMPS * 100);  // Convert 2A to 0.01A units
        new_frequency = rs485_CalcFrequencyFor_CC(current_frequency, precharge_target_0_01A, actual_current_0_01A);
 
        // Debug logging
        #if ACTUAL_TARGET_CC_CV_debug
        int32_t current_error = (int32_t)actual_current_0_01A - (int32_t)precharge_target_0_01A;
        Serial.printf("[CHARGING_START] Target: %.2fA (Precharge), Actual: %.2fA, Error: %d (0.01A), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            PRECHARGE_AMPS, safe_actual_current, current_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        #endif
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Check if voltage reached target voltage before 3 minutes - override and go to complete
        if (safe_actual_voltage >= target_voltage) {
            Serial.println("[CHARGING] Voltage limit reached during precharge, transitioning to complete");
            
            // STEP 1: Send 0 RPM command IMMEDIATELY when condition is met
            Serial.println("[CHARGING] Sending 0 RPM command immediately...");
            rs485_sendFrequencyCommand(0);  // Send 0 Hz
            current_frequency = 0;
            delay(10);  // Give RS485 time to send the command
            
            // STEP 2: Calculate final charging time
            unsigned long current_time = millis();
            if (charging_start_time > 0) {
                final_charging_time_ms = current_time - charging_start_time;
                charging_complete = true;
                Serial.printf("[CHARGING] Final charging time: %lu ms (%.2f minutes)\n", 
                             final_charging_time_ms, final_charging_time_ms / 60000.0f);
            }
            
            // Open contactor via CAN bus
            Serial.println("[CONTACTOR] Opening contactor on voltage limit reached during precharge...");
            send_contactor_control(CONTACTOR_OPEN);
            
            // Set stop reason to voltage limit reached during precharge
            charge_stop_reason = CHARGE_STOP_VOLTAGE_LIMIT_PRECHARGE;
            
            // Log charge complete
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            
// Transition to complete state
            current_app_state = STATE_CHARGING_COMPLETE;
            current_flow_start = false;
            Serial.println("[CHARGING] Transitioned to charging complete state (voltage limit during precharge)");

            // Set flag to send stop command after screen 6 loads
            pending_stop_command = true;

            // STEP 3: Move to screen 6
            switch_to_screen(SCREEN_CHARGING_COMPLETE);
        }
        // Check if precharge time elapsed (3 minutes), then transition to CC state
        else {
            unsigned long current_time = millis();
            unsigned long precharge_elapsed = 0;
            if (charging_start_time > 0) {
                precharge_elapsed = current_time - charging_start_time;
            }
            
            if (precharge_elapsed >= PRECHARGE_TIME_MS) {
                Serial.println("[CHARGING] Precharge complete (x min elapsed), transitioning to CC mode");
                precharge_duration_for_timer = precharge_elapsed;  // For CV time: precharge + 50% CC, max 33 min
                current_app_state = STATE_CHARGING_CC; //update the fsm state to charging cc
                cc_state_start_time = millis();  // Record CC state start time
                Serial.println("[CHARGING] CC state timing started");
                
                // Initialize voltage saturation tracking on CC entry
                base_volt_satu_ref = safe_actual_voltage;  // Record base voltage reference
                present_volt_satu_check = 0.0f;  // Reset present voltage check
                last_voltage_saturation_check_time = millis();  // Initialize check time
                Serial.printf("[VOLT_SAT] CC entry: base_volt_satu_ref = %.2fV\n", base_volt_satu_ref);
                
                // Screen switch will happen in determine_screen_from_state()
            }
        }
    }

    // ============================================================================
    // [2] STATE_CHARGING_CC: Constant Current mode until target voltage reached
    // ============================================================================
    else if (current_app_state == STATE_CHARGING_CC) {
        // Battery disconnected in step 2
        if (current_flow_start && safe_actual_current < 1.0f) {
            Serial.println("[CHARGING_CC] Battery disconnected (current < 1.0 A), emergency stop");
            send_contactor_control(CONTACTOR_OPEN);
            delay(10);
            rs485_sendFrequencyCommand(0);
            current_frequency = 0;
            delay(10);
            if (charging_start_time > 0 && !charging_complete) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
            }
            charge_stop_reason = CHARGE_STOP_BATTERY_DISCONNECTED;
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge,
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            current_flow_start = false;
            pending_stop_command = true;
            current_app_state = STATE_EMERGENCY_STOP;
            switch_to_screen(SCREEN_EMERGENCY_STOP);
            return;
        }
        new_frequency = rs485_CalcFrequencyFor_CC(current_frequency, target_current_0_01A, actual_current_0_01A); 
        
        // Debug logging
        #if ACTUAL_TARGET_CC_CV_debug
        int32_t current_error = (int32_t)actual_current_0_01A - (int32_t)target_current_0_01A;
        Serial.printf("[CHARGING_CC] Target: %.2fA, Actual: %.2fA, Error: %d (0.01A), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            target_current, safe_actual_current, current_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        #endif
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Check if 110% capacity reached: Stop charging if accumulated_ah >= 110% of selected_battery_profile Ah
        if (selected_battery_profile != nullptr) {
            float battery_profile_ah = (float)selected_battery_profile->getRatedAh();
            float capacity_threshold = battery_profile_ah * 1.1f;  // 110% of rated capacity
            
            if (accumulated_ah >= capacity_threshold) {
                Serial.printf("[CHARGING_CC] 110%% capacity reached! Accumulated: %.2f Ah, Threshold: %.2f Ah (%.0f Ah * 1.1)\n",
                             accumulated_ah, capacity_threshold, battery_profile_ah);
                
                // Send 0 RPM command immediately
                Serial.println("[CHARGING_CC] Sending 0 RPM command immediately...");
                rs485_sendFrequencyCommand(0);  // Send 0 Hz
                current_frequency = 0;
                delay(10);  // Give RS485 time to send the command
                
                // Open contactor via CAN bus IMMEDIATELY
                Serial.println("[CONTACTOR] Opening contactor immediately on 110% capacity reached...");
                send_contactor_control(CONTACTOR_OPEN);
                
                // Store final charging time before transitioning (if charging was in progress)
                if (charging_start_time > 0 && !charging_complete) {
                    final_charging_time_ms = millis() - charging_start_time;
                    charging_complete = true;
                    Serial.printf("[CHARGING_CC] Final charging time: %lu ms (%.2f minutes)\n", 
                                 final_charging_time_ms, final_charging_time_ms / 60000.0f);
                    
                    // Calculate and store final remaining time (if in CV state)
                    if (cv_start_time > 0 && cc_state_duration_for_timer > 0) {
                        unsigned long cv_elapsed = millis() - cv_start_time;
                        unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                        unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                        unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
                        final_remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                        Serial.printf("[CHARGING_CC] Final remaining time: %lu ms (%.2f minutes)\n", 
                                     final_remaining_time_ms, final_remaining_time_ms / 60000.0f);
                    }
                }
                
                // Set stop reason to 110% capacity reached
                charge_stop_reason = CHARGE_STOP_110_PERCENT_CAPACITY;
                current_flow_start = false;
                
                // Log charge complete
                if (sd_logging_initialized) {
                    logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                                     final_charging_time_ms, accumulated_ah, charge_stop_reason);
                }
                
                // Set flag to send stop command after screen 7 loads
                pending_stop_command = true;
                
                // Switch to emergency stop state and screen
                current_app_state = STATE_EMERGENCY_STOP;
                switch_to_screen(SCREEN_EMERGENCY_STOP);
                
                // Exit early - don't continue with normal charging control
                return;
            }
        }
        
        // Voltage saturation check: Check every xx1 minutes (VOLTAGE_SATURATION_CHECK_INTERVAL_MS)
        unsigned long current_time = millis();
        if (last_voltage_saturation_check_time > 0 && 
            (current_time - last_voltage_saturation_check_time) >= VOLTAGE_SATURATION_CHECK_INTERVAL_MS) {
            
            // Time to check for voltage saturation
            present_volt_satu_check = safe_actual_voltage;  // Record present voltage
            float voltage_difference = present_volt_satu_check - base_volt_satu_ref;
            
            Serial.printf("[VOLT_SAT] Check: base=%.2fV, present=%.2fV, diff=%.2fV\n", 
                         base_volt_satu_ref, present_volt_satu_check, voltage_difference);
            
            if (voltage_difference > VOLTAGE_SATURATION_THRESHOLD_V) {
                // Voltage increased by more than 0.5V - no saturation, continue CC
                Serial.println("[VOLT_SAT] Voltage increased > 0.5V, no saturation detected. Continuing CC stage.");
                base_volt_satu_ref = present_volt_satu_check;  // Update base reference
                present_volt_satu_check = 0.0f;  // Reset present check
                last_voltage_saturation_check_time = current_time;  // Reset check timer
            } else {
                // Voltage increase <= 0.5V (or negative) - saturation detected
                Serial.printf("[VOLT_SAT] Saturation detected! Voltage diff=%.2fV <= %.2fV\n", 
                             voltage_difference, VOLTAGE_SATURATION_THRESHOLD_V);
                Serial.printf("[VOLT_SAT] Recording saturation voltage: %.2fV\n", present_volt_satu_check);
                
                // Record the saturation voltage
                voltage_saturation_detected_voltage = present_volt_satu_check;
                
                // Transition to voltage saturation state (Screen 8)
                current_app_state = STATE_CHARGING_VOLTAGE_SATURATION;
                voltage_saturation_cv_start_time = millis();  // Record CV start time for screen 8
                current_flow_start = false;  // Reset on saturate entry
                Serial.println("[VOLT_SAT] Transitioning to voltage saturation state (Screen 8)");
                
                // Immediately calculate and send CV frequency command using saturation voltage
                uint16_t saturation_voltage_0_01V = (uint16_t)(voltage_saturation_detected_voltage * 100);
                uint16_t sat_cv_frequency = rs485_CalcFrequencyFor_CV(
                    current_frequency, 
                    saturation_voltage_0_01V, 
                    actual_voltage_0_01V
                );
                rs485_sendFrequencyCommand(sat_cv_frequency);
                current_frequency = sat_cv_frequency;
                Serial.println("[VOLT_SAT] CV frequency command sent immediately on saturation transition");
                
                // Screen switch will happen in determine_screen_from_state()
                return;  // Exit early, don't check normal voltage transition
            }
        }
        
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
            
            // CRITICAL: Immediately calculate and send CV frequency command to prevent motor freeze
            // Don't wait for next update_charging_control() call - ensure continuous control
            uint16_t cv_frequency = rs485_CalcFrequencyFor_CV(
                current_frequency, 
                target_voltage_0_01V, 
                actual_voltage_0_01V
            );
            rs485_sendFrequencyCommand(cv_frequency);
            current_frequency = cv_frequency;
            Serial.println("[CHARGING] CV frequency command sent immediately on transition");
            
// Calculate initial remaining time for immediate display
            if (cc_state_duration_for_timer > 0) {
                unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
                unsigned long rem_seconds = target_cv_time / 1000;
                unsigned long rem_minutes = rem_seconds / 60;
                rem_seconds = rem_seconds % 60;
                Serial.printf("[CHARGING] Initial remaining time: %02lu:%02lu (target CV time: %lu ms)\n",
                             rem_minutes, rem_seconds, target_cv_time);
            }
            // Screen switch will happen in determine_screen_from_state()
        }
    }

    // [3] STATE_CHARGING_CV: Constant Voltage mode
    else if (current_app_state == STATE_CHARGING_CV) {
        // Battery disconnected in step 3
        if (current_flow_start && safe_actual_current < 1.0f) {
            Serial.println("[CHARGING_CV] Battery disconnected (current < 1.0 A), emergency stop");
            send_contactor_control(CONTACTOR_OPEN);
            delay(10);
            rs485_sendFrequencyCommand(0);
            current_frequency = 0;
            delay(10);
            if (charging_start_time > 0 && !charging_complete) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
            }
            charge_stop_reason = CHARGE_STOP_BATTERY_DISCONNECTED;
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge,
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            current_flow_start = false;
            pending_stop_command = true;
            current_app_state = STATE_EMERGENCY_STOP;
            switch_to_screen(SCREEN_EMERGENCY_STOP);
            return;
        }
        new_frequency = rs485_CalcFrequencyFor_CV(current_frequency, target_voltage_0_01V, actual_voltage_0_01V);
        
        // Debug logging
        #if ACTUAL_TARGET_CC_CV_debug
        int32_t voltage_error = (int32_t)actual_voltage_0_01V - (int32_t)target_voltage_0_01V;
        Serial.printf("[CHARGING_CV] Target: %.2fV, Actual: %.2fV, Error: %d (0.01V), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            target_voltage, safe_actual_voltage, voltage_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        #endif
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Termination condition: Check if charging is complete
        // Complete if: 33 minutes in CV mode OR (precharge time + 50% of CC time) has elapsed
        unsigned long current_time = millis();
        unsigned long cv_duration = 0;
        if (cv_state_start_time > 0) {
            cv_duration = current_time - cv_state_start_time;
        }
        
        const unsigned long CV_COMPLETE_TIME_MS = 33 * 60 * 1000;  // 33 minutes in milliseconds
        bool cv_time_complete = (cv_duration >= CV_COMPLETE_TIME_MS);
        
        // Precharge + 50% of CC state time (if CC duration was recorded)
        bool cc_time_complete = false;
        if (cc_state_duration > 0) {
            unsigned long cv_target_time = precharge_duration_for_timer + (cc_state_duration / 2);
            unsigned long cv_33_min = 33 * 60 * 1000;
            unsigned long cv_target_capped = (cv_target_time < cv_33_min) ? cv_target_time : cv_33_min;
            cc_time_complete = (cv_duration >= cv_target_capped);
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
                Serial.printf("[CHARGING] CV mode duration: %lu ms (%.2f minutes) >= 33 minutes\n",
                    cv_duration, cv_duration / 60000.0f);
            }
            if (cc_time_complete && cc_state_duration > 0) {
                Serial.printf("[CHARGING] CV duration (%lu ms) >= (precharge + 50%% CC) target\n",
                    cv_duration);
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
                    unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                    unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                    unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
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
            current_flow_start = false;
            
            // Log charge complete
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            
            current_app_state = STATE_CHARGING_COMPLETE;
            
            // Set flag to send stop command after screen 6 loads
            pending_stop_command = true;
            
            // STEP 3: Move to screen 6
            switch_to_screen(SCREEN_CHARGING_COMPLETE);
        }
    }
        
    // [4] STATE_CHARGING_VOLTAGE_SATURATION: Constant Voltage mode using saturation voltage as target
    else if (current_app_state == STATE_CHARGING_VOLTAGE_SATURATION) {
        // Battery disconnected in voltage saturation
        if (current_flow_start && safe_actual_current < 1.0f) {
            Serial.println("[CHARGING_VOLT_SAT] Battery disconnected (current < 1.0 A), emergency stop");
            send_contactor_control(CONTACTOR_OPEN);
            delay(10);
            rs485_sendFrequencyCommand(0);
            current_frequency = 0;
            delay(10);
            if (charging_start_time > 0 && !charging_complete) {
                final_charging_time_ms = millis() - charging_start_time;
                charging_complete = true;
            }
            charge_stop_reason = CHARGE_STOP_BATTERY_DISCONNECTED;
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge,
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            current_flow_start = false;
            pending_stop_command = true;
            current_app_state = STATE_EMERGENCY_STOP;
            switch_to_screen(SCREEN_EMERGENCY_STOP);
            return;
        }
        // Use the recorded saturation voltage as target instead of battery profile cutoff voltage
        uint16_t saturation_voltage_0_01V = (uint16_t)(voltage_saturation_detected_voltage * 100);
        
        new_frequency = rs485_CalcFrequencyFor_CV(current_frequency, saturation_voltage_0_01V, actual_voltage_0_01V);
        
        // Debug logging
        #if ACTUAL_TARGET_CC_CV_debug
        int32_t voltage_error = (int32_t)actual_voltage_0_01V - (int32_t)saturation_voltage_0_01V;
        Serial.printf("[CHARGING_VOLT_SAT] Target: %.2fV, Actual: %.2fV, Error: %d (0.01V), Freq: %d -> %d (%.2f Hz -> %.2f Hz)\n",
            voltage_saturation_detected_voltage, safe_actual_voltage, voltage_error,
            current_frequency, new_frequency,
            current_frequency / 100.0f, new_frequency / 100.0f);
        #endif
        
        // Send frequency command
        rs485_sendFrequencyCommand(new_frequency);
        current_frequency = new_frequency;
        
        // Termination condition: Check if CV duration has reached xx2 minutes (VOLTAGE_SATURATION_CV_DURATION_MS)
        unsigned long current_time = millis();
        unsigned long sat_cv_duration = 0;
        if (voltage_saturation_cv_start_time > 0) {
            sat_cv_duration = current_time - voltage_saturation_cv_start_time;
        }
        
        bool sat_cv_time_complete = (sat_cv_duration >= VOLTAGE_SATURATION_CV_DURATION_MS);
        
        if (sat_cv_time_complete) {
            Serial.println("[CHARGING] Voltage saturation CV charging complete (xx2 minutes elapsed)!");
            
            // STEP 1: Send 0 RPM command IMMEDIATELY when condition is met
            Serial.println("[CHARGING] Sending 0 RPM command immediately...");
            rs485_sendFrequencyCommand(0);  // Send 0 Hz
            current_frequency = 0;
            delay(10);  // Give RS485 time to send the command
            
            // STEP 2: Now do other things (logging, calculations, etc.)
            // Calculate total charging time
            if (charging_start_time > 0) {
                final_charging_time_ms = current_time - charging_start_time;
                charging_complete = true;
                Serial.printf("[CHARGING] Final charging time: %lu ms (%.2f minutes)\n", 
                             final_charging_time_ms, final_charging_time_ms / 60000.0f);
            }
            
            // Open contactor via CAN bus
            Serial.println("[CONTACTOR] Opening contactor on charging complete...");
            send_contactor_control(CONTACTOR_OPEN);
            
            // Set stop reason to voltage saturation
            charge_stop_reason = CHARGE_STOP_VOLTAGE_SATURATION;
            current_flow_start = false;
            
            // Log charge complete
            if (sd_logging_initialized) {
                logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                                 final_charging_time_ms, accumulated_ah, charge_stop_reason);
            }
            
            // Transition to complete state
            current_app_state = STATE_CHARGING_COMPLETE;
            Serial.println("[CHARGING] Transitioned to charging complete state (voltage saturation)");
            
            // Set flag to send stop command after screen 6 loads
            pending_stop_command = true;
            
            // STEP 3: Move to screen 6
            switch_to_screen(SCREEN_CHARGING_COMPLETE);
        }
    }
}
// ============================================================================
//main charging control funciton end 
// ============================================================================






// Update current screen content (screen-specific updates)
void update_current_screen() {
    // Screen 1 (home): Update M2 RTC time label only when battery_detected is false (rate limited to 2Hz = 500ms)
    if (current_screen_id == SCREEN_HOME) {
        if (!battery_detected && screen1_rtc_time_label != nullptr) {
            unsigned long current_time = millis();
            const unsigned long RTC_UPDATE_INTERVAL = 500; // 2Hz = 500ms
            
            if (current_time - last_rtc_update_time >= RTC_UPDATE_INTERVAL) {
                char rtc_text[50];
                sprintf(rtc_text, "M2 rtc time: %04d-%02d-%02d %02d:%02d:%02d",
                        m2Time.year, m2Time.month, m2Time.date,
                        m2Time.hour, m2Time.minute, m2Time.second);
                lv_label_set_text(screen1_rtc_time_label, rtc_text);
                lv_obj_clear_flag(screen1_rtc_time_label, LV_OBJ_FLAG_HIDDEN);
                last_rtc_update_time = current_time;
            }
        } else if (battery_detected && screen1_rtc_time_label != nullptr) {
            // Hide label when battery is detected
            lv_obj_add_flag(screen1_rtc_time_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
    // Screen 18: update M2 RTC date/time (same position and 500ms rate as screen 1)
    if (current_screen_id == SCREEN_M2_LOST && screen18_rtc_time_label != nullptr) {
        unsigned long current_time = millis();
        const unsigned long RTC_UPDATE_INTERVAL = 500;  // 2Hz = 500ms
        if (current_time - last_rtc_update_time >= RTC_UPDATE_INTERVAL) {
            char rtc_text_18[50];
            sprintf(rtc_text_18, "M2 rtc time: %04d-%02d-%02d %02d:%02d:%02d",
                    m2Time.year, m2Time.month, m2Time.date,
                    m2Time.hour, m2Time.minute, m2Time.second);
            lv_label_set_text(screen18_rtc_time_label, rtc_text_18);
            last_rtc_update_time = current_time;
        }
    }
    
#if CAN_RTC_DEBUG
    // Time debug display updates only when on time debug screen
    if (current_screen_id == SCREEN_TIME_DEBUG) {
        update_time_debug_display();
    }
#endif // CAN_RTC_DEBUG

    // Charging control (runs only in charging states, once per second)
    update_charging_control();
    
    // Update Ah calculation (runs only in charging states)
    update_accumulated_ah();
    
    // Check battery removal on screens 6 and 7 - auto-dismiss popup and navigate when battery removed
    if ((current_screen_id == SCREEN_CHARGING_COMPLETE || current_screen_id == SCREEN_EMERGENCY_STOP)) {
        // Check if battery was removed while popup is showing
        if (!battery_detected || sensorData.volt < 9.0f) {
            // Battery removed - hide popup if visible
            lvgl_port_lock(-1);  // Lock LVGL for thread safety
            bool should_navigate = false;
            
            if (screen6_remove_battery_popup != nullptr && current_screen_id == SCREEN_CHARGING_COMPLETE) {
                if (!lv_obj_has_flag(screen6_remove_battery_popup, LV_OBJ_FLAG_HIDDEN)) {
                    Serial.println("[HOME] Battery removed, auto-navigating to home");
                    lv_obj_add_flag(screen6_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
                    should_navigate = true;
                }
            } else if (screen7_remove_battery_popup != nullptr && current_screen_id == SCREEN_EMERGENCY_STOP) {
                if (!lv_obj_has_flag(screen7_remove_battery_popup, LV_OBJ_FLAG_HIDDEN)) {
                    Serial.println("[HOME] Battery removed, auto-navigating to home");
                    lv_obj_add_flag(screen7_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
                    should_navigate = true;
                }
            }
            
            if (should_navigate) {
                // Reset variables
                charge_stop_reason = CHARGE_STOP_NONE;
                charging_start_time = 0;
                cv_start_time = 0;
                    cc_state_duration_for_timer = 0;
                    precharge_duration_for_timer = 0;
                    pending_stop_command = false;
                    current_flow_start = false;
                    battery_detected = false;
                    
                    // Clear selected battery profile - user must select again from scratch
                    selected_battery_profile = nullptr;
                    
                    // Navigate to home
                    current_app_state = STATE_HOME;
                    lvgl_port_unlock();  // Unlock before calling switch_to_screen (which may lock internally)
                    switch_to_screen(SCREEN_HOME);
            } else {
                lvgl_port_unlock();  // Unlock if no navigation needed
            }
        }
    }
    
    // Update temperature label on screen 3
    if (screen3_temp_label != nullptr && current_screen_id == SCREEN_CHARGING_STARTED) {
        float temp1_celsius = sensorData.temp1 / 100.0f;
        float temp2_celsius = sensorData.temp2 / 100.0f;
        char temp_text[80];
        sprintf(temp_text, "Motor temp : %.1f , Gcu temp : %.1f", temp1_celsius, temp2_celsius);
        lv_label_set_text(screen3_temp_label, temp_text);
    }
    if (screen4_battery_details_label != nullptr && current_screen_id == SCREEN_CHARGING_CC) {
        if (selected_battery_profile != nullptr) {
            char details_text[180];
            sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)",
                    selected_battery_profile->getBatteryName().c_str(),
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getCutoffVoltage(),
                    selected_battery_profile->getConstCurrent());
            lv_label_set_text(screen4_battery_details_label, details_text);
        } else {
            lv_label_set_text(screen4_battery_details_label, "Selected Battery: None");
        }
    }
    // Update temperature label on screen 4
    if (screen4_temp_label != nullptr && current_screen_id == SCREEN_CHARGING_CC) {
        float temp1_celsius = sensorData.temp1 / 100.0f;
        float temp2_celsius = sensorData.temp2 / 100.0f;
        char temp_text[80];
        sprintf(temp_text, "Motor temp : %.1f , Gcu temp : %.1f", temp1_celsius, temp2_celsius);
        lv_label_set_text(screen4_temp_label, temp_text);
    }
    // Update temperature label on screen 5
    if (screen5_temp_label != nullptr && current_screen_id == SCREEN_CHARGING_CV) {
        float temp1_celsius = sensorData.temp1 / 100.0f;
        float temp2_celsius = sensorData.temp2 / 100.0f;
        char temp_text[80];
        sprintf(temp_text, "Motor temp : %.1f , Gcu temp : %.1f", temp1_celsius, temp2_celsius);
        lv_label_set_text(screen5_temp_label, temp_text);
    }
    // Update temperature label on screen 8 (same format as screens 3,4,5)
    if (screen8_temp_label != nullptr && current_screen_id == SCREEN_VOLTAGE_SATURATION) {
        float temp1_celsius = sensorData.temp1 / 100.0f;
        float temp2_celsius = sensorData.temp2 / 100.0f;
        char temp_text[80];
        sprintf(temp_text, "Motor temp : %.1f , Gcu temp : %.1f", temp1_celsius, temp2_celsius);
        lv_label_set_text(screen8_temp_label, temp_text);
    }
    if (screen8_battery_details_label != nullptr && current_screen_id == SCREEN_VOLTAGE_SATURATION) {
        if (selected_battery_profile != nullptr) {
            char details_text[220];
            sprintf(details_text, "Selected Battery: %s , %s (TV: %.1f V, TC: %.1f A)\nSaturation Voltage: %.2f V",
                    selected_battery_profile->getBatteryName().c_str(),
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getCutoffVoltage(),
                    selected_battery_profile->getConstCurrent(),
                    voltage_saturation_detected_voltage);
            lv_label_set_text(screen8_battery_details_label, details_text);
        } else {
            lv_label_set_text(screen8_battery_details_label, "Selected Battery: None");
        }
    }
    
    // Update screen 6 status label based on charge stop reason
    if (screen6_status_label != nullptr && current_screen_id == SCREEN_CHARGING_COMPLETE) {
        if (charge_stop_reason == CHARGE_STOP_VOLTAGE_LIMIT_PRECHARGE) {
            lv_label_set_text(screen6_status_label, "Voltage limit reached during precharge");
            lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green (info, not error)
        } else if (charge_stop_reason == CHARGE_STOP_VOLTAGE_SATURATION) {
            lv_label_set_text(screen6_status_label, "Charge stopped due to voltage saturate!");
            lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else if (charge_stop_reason == CHARGE_STOP_COMPLETE) {
            lv_label_set_text(screen6_status_label, "Battery charging completed successfully");
            lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
        } else if (charge_stop_reason == CHARGE_STOP_EMERGENCY) {
            // This shouldn't happen on screen 6, but handle it just in case
            lv_label_set_text(screen6_status_label, "Charging stopped by user");
            lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else {
            // Default message
            lv_label_set_text(screen6_status_label, "Battery charging completed successfully");
            lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
        }
    }
    
    // Update screen 7 status label based on charge stop reason
    if (screen7_status_label != nullptr && current_screen_id == SCREEN_EMERGENCY_STOP) {
        if (charge_stop_reason == CHARGE_STOP_HIGH_TEMP) {
            lv_label_set_text(screen7_status_label, "High temp detected");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else if (charge_stop_reason == CHARGE_STOP_EMERGENCY) {
            lv_label_set_text(screen7_status_label, "Charging stopped by user");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else if (charge_stop_reason == CHARGE_STOP_110_PERCENT_CAPACITY) {
            lv_label_set_text(screen7_status_label, "110% capacity reached");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else if (charge_stop_reason == CHARGE_STOP_BATTERY_DISCONNECTED) {
            lv_label_set_text(screen7_status_label, "Battery disconnected error");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else if (charge_stop_reason == CHARGE_STOP_VOLT_OR_CURRENT_ERROR) {
            lv_label_set_text(screen7_status_label, "Volt or current error");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        } else {
            // Default message
            lv_label_set_text(screen7_status_label, "Charging stopped by user");
            lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
        }
    }
    
    // Update timer displays on screens 3, 4, 5, 6, 7, 8
    lvgl_port_lock(-1);  // Lock LVGL for thread safety
    
    unsigned long total_elapsed = 0;
    if (charging_complete && final_charging_time_ms > 0) {
        // Use final time if charging is complete (stops updating)
        total_elapsed = final_charging_time_ms;
    } else if (charging_start_time > 0) {
        // Calculate elapsed time if charging is in progress
        total_elapsed = millis() - charging_start_time;
    }
    
    // Format Ah value (always update, even if time is 0)
    char ah_str[20];
    sprintf(ah_str, "%.1f", accumulated_ah);
    
    // Update Ah display on screens 3, 4, 5, 8 (always, not just when time > 0)
    if (!charging_complete) {
        if (screen3_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_STARTED) {
            lv_table_set_cell_value(screen3_timer_table, 1, 2, ah_str);
        }
        if (screen4_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_CC) {
            lv_table_set_cell_value(screen4_timer_table, 1, 2, ah_str);
        }
        if (screen5_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_CV) {
            lv_table_set_cell_value(screen5_timer_table, 1, 2, ah_str);
        }
        if (screen8_timer_table != nullptr && current_screen_id == SCREEN_VOLTAGE_SATURATION) {
            lv_table_set_cell_value(screen8_timer_table, 1, 2, ah_str);
        }
    }
    
    if (total_elapsed > 0) {
        unsigned long total_seconds = total_elapsed / 1000;
        unsigned long total_minutes = total_seconds / 60;
        unsigned long total_hours = total_minutes / 60;
        total_seconds = total_seconds % 60;
        total_minutes = total_minutes % 60;
        
        char time_str[20];
        sprintf(time_str, "%02lu:%02lu:%02lu", total_hours, total_minutes, total_seconds);
        
        // Update total time on screens 3, 4, 5, 8 (only if charging in progress)
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
                    unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                    unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                    unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
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
            if (screen8_timer_table != nullptr && current_screen_id == SCREEN_VOLTAGE_SATURATION) {
                lv_table_set_cell_value(screen8_timer_table, 1, 0, time_str);
                
                // Also update remaining time on screen 8 (voltage saturation CV state)
                if (voltage_saturation_cv_start_time > 0) {
                    unsigned long sat_cv_elapsed = millis() - voltage_saturation_cv_start_time;
                    unsigned long remaining_time_ms = (VOLTAGE_SATURATION_CV_DURATION_MS > sat_cv_elapsed) ? 
                                                      (VOLTAGE_SATURATION_CV_DURATION_MS - sat_cv_elapsed) : 0;
                    
                    unsigned long rem_seconds = remaining_time_ms / 1000;
                    unsigned long rem_minutes = rem_seconds / 60;
                    rem_seconds = rem_seconds % 60;
                    sprintf(time_str, "%02lu:%02lu", rem_minutes, rem_seconds);
                    lv_table_set_cell_value(screen8_timer_table, 1, 1, time_str);
                } else {
                    // If voltage_saturation_cv_start_time not set yet, show default
                    lv_table_set_cell_value(screen8_timer_table, 1, 1, "--:--");
                }
            }
        }
        
        // Update final time on screens 6 and 7 (always show final time when complete)
        if (screen6_timer_table != nullptr && current_screen_id == SCREEN_CHARGING_COMPLETE) {
            lv_table_set_cell_value(screen6_timer_table, 1, 0, time_str);
            lv_table_set_cell_value(screen6_timer_table, 1, 2, ah_str);
            
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
            lv_table_set_cell_value(screen7_timer_table, 1, 2, ah_str);
        }
    }
    
    lvgl_port_unlock();  // Unlock LVGL
}

// Determine which screen should be shown based on current state
screen_id_t determine_screen_from_state() {
    // Once on screen 18, stay there - only restart can leave (handled elsewhere)
    if (current_screen_id == SCREEN_M2_LOST) {
        return SCREEN_M2_LOST;
    }
    // M2 connection lost overrides all other states
    if (m2_connection_lost) {
        return SCREEN_M2_LOST;
    }
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
    if (current_app_state == STATE_CHARGING_VOLTAGE_SATURATION) {
        return SCREEN_VOLTAGE_SATURATION;
    }
    if (current_app_state == STATE_CHARGING_COMPLETE) {
        return SCREEN_CHARGING_COMPLETE;
    }
    if (current_app_state == STATE_EMERGENCY_STOP) {
        return SCREEN_EMERGENCY_STOP;
    }
#if CAN_RTC_DEBUG
    // Don't switch away from CAN debug screen based on voltage
    if (current_screen_id == SCREEN_CAN_DEBUG) {
        return SCREEN_CAN_DEBUG;
    }
    // Don't switch away from time debug screen based on voltage
    if (current_screen_id == SCREEN_TIME_DEBUG) {
        return SCREEN_TIME_DEBUG;
    }
#endif // CAN_RTC_DEBUG
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

// M2 heartbeat: run every 1s from loop. 6s grace for first frame 101 to arrive.
// After grace: if no frame ever (can101_rx_timestamp == 0) or last frame > 2100 ms ago -> M2 lost (screen 18).
void check_m2_heartbeat(void) {
    const unsigned long GRACE_MS = 6000;
    const unsigned long LOST_THRESHOLD_MS = 2100;
    unsigned long now = (unsigned long) millis();
    if (now < GRACE_MS) {
        return;
    }
    unsigned long last_101 = (unsigned long) can101_rx_timestamp;
    if (last_101 == 0 || (now - last_101) > LOST_THRESHOLD_MS) {
        m2_connection_lost = true;
        if (current_screen_id != SCREEN_M2_LOST) {
            switch_to_screen(SCREEN_M2_LOST);
        }
    } else {
        m2_connection_lost = false;
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

        // Create label with battery info: "batteryName , displayName"
        lv_obj_t* profile_label = lv_label_create(profile_btn);
        String labelText = profile->getBatteryName() + " , " + profile->getDisplayName();
        lv_label_set_text(profile_label, labelText.c_str());
        lv_obj_set_style_text_font(profile_label, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(profile_label, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_center(profile_label);

        Serial.printf("[BATTERY] Added matching profile at Y=%d: %s\n", button_y, labelText.c_str());

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

    // Update max values during charge (non-blocking)
    if (charging_start_time > 0 && !charging_complete) {
        if (sensorData.curr > max_current_during_charge) {
            max_current_during_charge = sensorData.curr;
        }
        if (sensorData.volt > max_voltage_during_charge) {
            max_voltage_during_charge = sensorData.volt;
        }
    }

    if (data_table != nullptr) {
        // Update voltage (column 0)
        lv_table_set_cell_value(data_table, 1, 0,
            String(sensorData.volt, 1).c_str());

        // Update current (column 1)
        lv_table_set_cell_value(data_table, 1, 1,
            String(sensorData.curr, 2).c_str());

        // Update temperature (column 2) - temp3 from CAN data (room temp, 0.01°C resolution)
        float temp3_celsius = sensorData.temp3 / 100.0f;
        lv_table_set_cell_value(data_table, 1, 2,
            String(temp3_celsius, 1).c_str());

        // Update frequency (column 3) - show current RPM
        float freq_hz = current_frequency / 100.0f;
        float rpm = VFD_FREQ_TO_RPM(freq_hz);
        lv_table_set_cell_value(data_table, 1, 3, String((int)rpm).c_str());

        // Update log number (column 4)
        lv_table_set_cell_value(data_table, 1, 4,
            String(log_num_sdhc).c_str());
    }

    // Unlock LVGL
    lvgl_port_unlock();
}

// Update accumulated Ah based on current flow
void update_accumulated_ah(void) {
    // Only run in charging states
    if (current_app_state != STATE_CHARGING_START && 
        current_app_state != STATE_CHARGING_CC && 
        current_app_state != STATE_CHARGING_CV) {
        return;
    }
    
    // Rate limiting: update every 1 second
    const unsigned long AH_UPDATE_INTERVAL = 1000; // 1 second
    unsigned long current_time = millis();
    
    // Initialize on first call or if reset
    if (last_ah_update_time == 0) {
        last_ah_update_time = current_time;
        Serial.println("[AH] Ah calculation initialized");
        return;
    }
    
    // Check if enough time has passed
    if (current_time - last_ah_update_time < AH_UPDATE_INTERVAL) {
        return;
    }
    
    // Calculate time delta in hours
    unsigned long time_delta_ms = current_time - last_ah_update_time;
    float time_delta_hours = time_delta_ms / 3600000.0f; // Convert ms to hours
    
    // Get current (handle negative values)
    float safe_current = (sensorData.curr < 0.0f) ? 0.0f : sensorData.curr;
    
    // Ah = current (A) * time (h)
    float ah_increment = safe_current * time_delta_hours;
    accumulated_ah += ah_increment;
    
    #if Ah_CALCULATION_DEBUG
        // Debug logging
        Serial.printf("[AH] Current: %.2fA, Time: %.3fh, Increment: %.4fAh, Total: %.2fAh\n",
                    safe_current, time_delta_hours, ah_increment, accumulated_ah);
    #endif
    
    last_ah_update_time = current_time;
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

// Update time debug screen with M2 RTC time (rate limited to 2Hz = 500ms)
void update_time_debug_display() {
    if (screen16_time_label != nullptr && current_screen_id == SCREEN_TIME_DEBUG) {
        unsigned long current_time = millis();
        const unsigned long RTC_UPDATE_INTERVAL = 500; // 2Hz = 500ms
        
        // Rate limit to 2Hz
        if (current_time - last_rtc_update_time >= RTC_UPDATE_INTERVAL) {
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
            last_rtc_update_time = current_time;

            lvgl_port_unlock();
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
        
        // Show confirmed battery info below the table
        if (screen2_confirmed_battery_label != nullptr && selected_battery_profile != nullptr) {
            char confirmed_str[200];
            sprintf(confirmed_str, "Confirmed: %s , %s (TV: %.1f V, TC: %.1f A)",
                    selected_battery_profile->getBatteryName().c_str(),
                    selected_battery_profile->getDisplayName().c_str(),
                    selected_battery_profile->getCutoffVoltage(),
                    selected_battery_profile->getConstCurrent());
            lv_label_set_text(screen2_confirmed_battery_label, confirmed_str);
            lv_obj_clear_flag(screen2_confirmed_battery_label, LV_OBJ_FLAG_HIDDEN);
        }
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

            // Update confirmation popup: displayName then batteryName (font 28), then TV/TC
            char tv_str[50];
            char tc_str[50];
            char battery_info_str[120];
            snprintf(battery_info_str, sizeof(battery_info_str), "%s\n%s",
                    selected_profile->getDisplayName().c_str(),
                    selected_profile->getBatteryName().c_str());

            sprintf(tv_str, "Target Voltage: %.1f V", selected_profile->getCutoffVoltage());
            sprintf(tc_str, "Target Current: %.1f A", selected_profile->getConstCurrent());

            lv_label_set_text(screen2_confirm_battery_info_label, battery_info_str);
            lv_label_set_text(screen2_confirm_voltage_label, tv_str);
            lv_label_set_text(screen2_confirm_capacity_label, tc_str);
            lv_label_set_text(screen2_confirm_current_label, "");  // Hide unused label
            lv_label_set_text(screen2_confirm_type_label, "");  // Hide unused label

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
        precharge_duration_for_timer = 0;
        
        // Reset voltage saturation tracking variables
        base_volt_satu_ref = 0.0f;
        present_volt_satu_check = 0.0f;
        last_voltage_saturation_check_time = 0;
        voltage_saturation_detected_voltage = 0.0f;
        voltage_saturation_cv_start_time = 0;
        
        // Initialize charging start time and reset completion flag
        charging_start_time = millis();
        charging_complete = false;
        final_charging_time_ms = 0;
        final_remaining_time_ms = 0;
        pending_stop_command = false;  // Reset stop command flag
        current_flow_start = false;
        
        // Reset Ah calculation
        accumulated_ah = 0.0f;
        last_ah_update_time = millis();
        
        // Reset max tracking variables
        max_current_during_charge = 0.0f;
        max_voltage_during_charge = 0.0f;
        
        // Log charge start
        if (sd_logging_initialized && selected_battery_profile != nullptr) {
            current_charge_serial = getNextSerialNumber();
            if (logChargeStart(current_charge_serial, selected_battery_profile)) {
                // Update log_num_sdhc after successful log write
                log_num_sdhc = (int32_t)current_charge_serial;
                // Update table display immediately
                if (data_table != nullptr) {
                    lv_table_set_cell_value(data_table, 1, 4, String(log_num_sdhc).c_str());
                }
            }
        }

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
                unsigned long cv_target_base = precharge_duration_for_timer + (cc_state_duration_for_timer / 2);  // precharge + 50% CC
                unsigned long cv_33_min = 33 * 60 * 1000;  // 33 minutes in ms
                unsigned long target_cv_time = (cv_target_base < cv_33_min) ? cv_target_base : cv_33_min;
                final_remaining_time_ms = (target_cv_time > cv_elapsed) ? (target_cv_time - cv_elapsed) : 0;
                Serial.printf("[EMERGENCY] Final remaining time: %lu ms (%.2f minutes)\n", 
                             final_remaining_time_ms, final_remaining_time_ms / 60000.0f);
            }
        }
        
        // Set stop reason
        charge_stop_reason = CHARGE_STOP_EMERGENCY;
        current_flow_start = false;
        
        // Log charge complete
        if (sd_logging_initialized) {
            logChargeComplete(max_voltage_during_charge, max_current_during_charge, 
                             final_charging_time_ms, accumulated_ah, charge_stop_reason);
        }
        
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
        
        // Check if battery is still connected
        if (battery_detected && sensorData.volt >= 9.0f) {
            // Battery still connected - show message to remove battery
            Serial.println("[HOME] Battery still connected, showing remove battery message");
            
            // Show popup on current screen (6 or 7)
            if (current_screen_id == SCREEN_CHARGING_COMPLETE && screen6_remove_battery_popup != nullptr) {
                lv_obj_clear_flag(screen6_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
                lv_obj_move_foreground(screen6_remove_battery_popup);
            } else if (current_screen_id == SCREEN_EMERGENCY_STOP && screen7_remove_battery_popup != nullptr) {
                lv_obj_clear_flag(screen7_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
                lv_obj_move_foreground(screen7_remove_battery_popup);
            }
            // Don't navigate - stay on current screen until battery is removed
        } else {
            // Battery removed - proceed with navigation to home
            Serial.println("[HOME] Battery removed, switching to home screen");
            
            // Hide popups if visible
            if (screen6_remove_battery_popup != nullptr) {
                lv_obj_add_flag(screen6_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
            }
            if (screen7_remove_battery_popup != nullptr) {
                lv_obj_add_flag(screen7_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);
            }
            
            // Reset stop reason
            charge_stop_reason = CHARGE_STOP_NONE;
            
            // Reset timer variables
            charging_start_time = 0;
            cv_start_time = 0;
            cc_state_duration_for_timer = 0;
            precharge_duration_for_timer = 0;
            pending_stop_command = false;  // Reset stop command flag
            current_flow_start = false;
            
            // Ensure battery_detected is false
            battery_detected = false;
            
            // Clear selected battery profile - user must select again from scratch
            selected_battery_profile = nullptr;
            
            // Navigate to home screen
            current_app_state = STATE_HOME;
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
    lv_label_set_text(title, "GCU 3kW Charger v4.1");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_30, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Status label (battery detected, etc)
    status_label = lv_label_create(screen_1);
    lv_label_set_text(status_label, "Connect Battery");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_28, LV_PART_MAIN);
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
        lv_table_set_col_width(data_table, 2, 198);  // Temp3
        lv_table_set_col_width(data_table, 3, 198);  // Rpm
        lv_table_set_col_width(data_table, 4, 198);  // Entry

        // Headers (Row 0)
        lv_table_set_cell_value(data_table, 0, 0, "Voltage");
        lv_table_set_cell_value(data_table, 0, 1, "Current");
        lv_table_set_cell_value(data_table, 0, 2, "Temp");
        lv_table_set_cell_value(data_table, 0, 3, "RPM");
        lv_table_set_cell_value(data_table, 0, 4, "LOG_num");

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

        // Initialize log_num_sdhc from SD card (latest complete log number)
        if (sd_logging_initialized) {
            uint32_t nextSerial = getNextSerialNumber();
            if (nextSerial > 1) {
                log_num_sdhc = (int32_t)(nextSerial - 1);  // Latest complete log = next - 1
            } else {
                log_num_sdhc = -1;  // No logs yet
            }
            // Update table display with initial value
            lv_table_set_cell_value(data_table, 1, 4, String(log_num_sdhc).c_str());
        }
    }

    // Move table to screen_1
    lv_obj_set_parent(data_table, screen_1);
    lv_obj_set_pos(data_table, 12, 110);

    // M2 RTC Time label (20px below table)
    screen1_rtc_time_label = lv_label_create(screen_1);
    lv_label_set_text(screen1_rtc_time_label, "M2 rtc time: -- --");
    lv_obj_set_style_text_font(screen1_rtc_time_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_rtc_time_label, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    //lv_obj_align(screen1_rtc_time_label, LV_ALIGN_TOP_LEFT, 12, 230);  // 20px below table (110 + ~100 table height + 20)
    lv_obj_align(screen1_rtc_time_label, LV_ALIGN_TOP_MID, 0, 280);  // screen center below table

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

#if CAN_RTC_DEBUG
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
#endif // CAN_RTC_DEBUG

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
    lv_label_set_text(title, "GCU 3kW Charger v4.1");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (battery detected, charge ready)
    lv_obj_t *status_label_2 = lv_label_create(screen_2);
    lv_label_set_text(status_label_2, "Battery detected - charge ready");
    lv_obj_set_style_text_color(status_label_2, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label_2, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(status_label_2, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_2
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_2);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // v4.08: Battery list container (shown with matching profiles on screen 2)
    screen2_battery_container = lv_obj_create(screen_2);
    lv_obj_set_size(screen2_battery_container, 950, 300);
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


    // Confirmed battery label (below table, above buttons) - hidden by default
    screen2_confirmed_battery_label = lv_label_create(screen_2);
    lv_label_set_text(screen2_confirmed_battery_label, "");
    lv_obj_set_style_text_font(screen2_confirmed_battery_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirmed_battery_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirmed_battery_label, LV_ALIGN_TOP_LEFT, 12, 240);  // Below table (table is at y=110, ~100px tall)
    lv_obj_add_flag(screen2_confirmed_battery_label, LV_OBJ_FLAG_HIDDEN);  // Hidden by default

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
    lv_label_set_text(screen2_confirm_title_label, "Confirm");
    lv_obj_set_style_text_font(screen2_confirm_title_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_title_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_title_label, LV_ALIGN_TOP_MID, 0, 20);

    // Battery info: displayName on first line, batteryName on second line (font 28)
    screen2_confirm_battery_info_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_battery_info_label, "--\n--");
    lv_obj_set_style_text_font(screen2_confirm_battery_info_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_battery_info_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_battery_info_label, LV_ALIGN_TOP_MID, 0, 55);

    // Target Voltage label - below battery name block
    screen2_confirm_voltage_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_voltage_label, "Target Voltage: -- V");
    lv_obj_set_style_text_font(screen2_confirm_voltage_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_voltage_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_voltage_label, LV_ALIGN_TOP_MID, 0, 130);

    // Target Current label
    screen2_confirm_capacity_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_capacity_label, "Target Current: -- A");
    lv_obj_set_style_text_font(screen2_confirm_capacity_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_capacity_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_capacity_label, LV_ALIGN_TOP_MID, 0, 180);

    // Current label (unused, hidden)
    screen2_confirm_current_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_current_label, "");
    lv_obj_set_style_text_font(screen2_confirm_current_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen2_confirm_current_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen2_confirm_current_label, LV_ALIGN_TOP_MID, 0, 210);

    // Type label (unused, hidden)
    screen2_confirm_type_label = lv_label_create(screen2_confirm_popup);
    lv_label_set_text(screen2_confirm_type_label, "");
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
    lv_obj_set_style_bg_color(screen_3, lv_color_hex(0xB8E6B8), LV_PART_MAIN);  // Lighter green background
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
    char precharge_text[60];
    sprintf(precharge_text, "Step 1: Precharge, upto %.1f amps.", PRECHARGE_AMPS);
    lv_label_set_text(status_label_3, precharge_text);
    lv_obj_set_style_text_color(status_label_3, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_3, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_3, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_3
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_3);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 30px)
    screen3_battery_details_label = lv_label_create(screen_3);
    lv_label_set_text(screen3_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen3_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen3_battery_details_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen3_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);  // Moved up 30px (300 -> 270)
    
    // Temperature label (below battery details label)
    screen3_temp_label = lv_label_create(screen_3);
    lv_label_set_text(screen3_temp_label, "Motor temp : -- , Gcu temp : --");
    lv_obj_set_style_text_color(screen3_temp_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen3_temp_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen3_temp_label, LV_ALIGN_TOP_LEFT, 12, 270);  // Below battery details label
    
    // Timer table (3x2) for screen 3 - below battery label, left aligned
    screen3_timer_table = lv_table_create(screen_3);
    lv_table_set_col_cnt(screen3_timer_table, 3);
    lv_table_set_row_cnt(screen3_timer_table, 2);
    lv_table_set_col_width(screen3_timer_table, 0, 200);
    lv_table_set_col_width(screen3_timer_table, 1, 200);
    lv_table_set_col_width(screen3_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen3_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen3_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen3_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen3_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen3_timer_table, 1, 1, "");
    lv_table_set_cell_value(screen3_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen3_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen3_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen3_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen3_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen3_timer_table, LV_ALIGN_TOP_MID, 0, 330);  // Centered horizontally, same vertical position
    lv_obj_clear_flag(screen3_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Emergency Stop button (bottom mid, large and visible, one-line label)
    lv_obj_t* screen3_emergency_stop_btn = lv_btn_create(screen_3);
    lv_obj_set_size(screen3_emergency_stop_btn, 360, 80);
    lv_obj_align(screen3_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen3_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen3_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen3_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen3_emergency_stop_label = lv_label_create(screen3_emergency_stop_btn);
    lv_label_set_text(screen3_emergency_stop_label, "EMERGENCY STOP");
    lv_obj_set_style_text_font(screen3_emergency_stop_label, &lv_font_montserrat_26, LV_PART_MAIN);
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
    lv_label_set_text(status_label_4, "Step 2, constant current charge");
    lv_obj_set_style_text_color(status_label_4, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_4, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_4, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_4
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_4);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 30px)
    screen4_battery_details_label = lv_label_create(screen_4);
    lv_label_set_text(screen4_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen4_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen4_battery_details_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen4_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);  // Moved up 30px (300 -> 270)
    
    // Temperature label (below battery details label)
    screen4_temp_label = lv_label_create(screen_4);
    lv_label_set_text(screen4_temp_label, "Motor temp : -- , Gcu temp : --");
    lv_obj_set_style_text_color(screen4_temp_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen4_temp_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen4_temp_label, LV_ALIGN_TOP_LEFT, 12, 270);  // Below battery details label
    
    // Timer table (3x2) for screen 4 - below battery label, left aligned
    screen4_timer_table = lv_table_create(screen_4);
    lv_table_set_col_cnt(screen4_timer_table, 3);
    lv_table_set_row_cnt(screen4_timer_table, 2);
    lv_table_set_col_width(screen4_timer_table, 0, 200);
    lv_table_set_col_width(screen4_timer_table, 1, 200);
    lv_table_set_col_width(screen4_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen4_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen4_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen4_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen4_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen4_timer_table, 1, 1, "");
    lv_table_set_cell_value(screen4_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen4_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen4_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen4_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen4_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen4_timer_table, LV_ALIGN_TOP_MID, 0, 330);  // Centered horizontally, same vertical position
    lv_obj_clear_flag(screen4_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Emergency Stop button (bottom mid, large and visible, one-line label)
    lv_obj_t* screen4_emergency_stop_btn = lv_btn_create(screen_4);
    lv_obj_set_size(screen4_emergency_stop_btn, 360, 80);
    lv_obj_align(screen4_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen4_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen4_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen4_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen4_emergency_stop_label = lv_label_create(screen4_emergency_stop_btn);
    lv_label_set_text(screen4_emergency_stop_label, "EMERGENCY STOP");
    lv_obj_set_style_text_font(screen4_emergency_stop_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen4_emergency_stop_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen4_emergency_stop_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 4 (CC Mode) created successfully");
}

//screen 5 - Constant Voltage (CV) mode
void create_screen_5(void) {
    screen_5 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_5, lv_color_hex(0x6BC96B), LV_PART_MAIN);  // Darker green background
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
    lv_label_set_text(status_label_5, "Step 3, Constant voltage charge");
    lv_obj_set_style_text_color(status_label_5, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_5, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_5, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_5
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_5);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, moved up 30px)
    screen5_battery_details_label = lv_label_create(screen_5);
    lv_label_set_text(screen5_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen5_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen5_battery_details_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen5_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);  // Moved up 30px (300 -> 270)
    
    // Temperature label (below battery details label)
    screen5_temp_label = lv_label_create(screen_5);
    lv_label_set_text(screen5_temp_label, "Motor temp : -- , Gcu temp : --");
    lv_obj_set_style_text_color(screen5_temp_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen5_temp_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen5_temp_label, LV_ALIGN_TOP_LEFT, 12, 270);  // Below battery details label
    
    // Timer table (3x2) for screen 5 - shows total time, remaining time, and Ah, below battery label, left aligned
    screen5_timer_table = lv_table_create(screen_5);
    lv_table_set_col_cnt(screen5_timer_table, 3);
    lv_table_set_row_cnt(screen5_timer_table, 2);
    lv_table_set_col_width(screen5_timer_table, 0, 200);
    lv_table_set_col_width(screen5_timer_table, 1, 200);
    lv_table_set_col_width(screen5_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen5_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen5_timer_table, 0, 1, "Remaining");
    lv_table_set_cell_value(screen5_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen5_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen5_timer_table, 1, 1, "00:00");
    lv_table_set_cell_value(screen5_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen5_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen5_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen5_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen5_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen5_timer_table, LV_ALIGN_TOP_MID, 0, 330);  // Centered horizontally, same vertical position
    lv_obj_clear_flag(screen5_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Emergency Stop button (bottom mid, large and visible, one-line label)
    lv_obj_t* screen5_emergency_stop_btn = lv_btn_create(screen_5);
    lv_obj_set_size(screen5_emergency_stop_btn, 360, 80);
    lv_obj_align(screen5_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen5_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen5_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen5_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen5_emergency_stop_label = lv_label_create(screen5_emergency_stop_btn);
    lv_label_set_text(screen5_emergency_stop_label, "EMERGENCY STOP");
    lv_obj_set_style_text_font(screen5_emergency_stop_label, &lv_font_montserrat_26, LV_PART_MAIN);
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

    // Status label (will be updated dynamically based on charge_stop_reason)
    screen6_status_label = lv_label_create(screen_6);
    lv_label_set_text(screen6_status_label, "Battery charging completed successfully");
    lv_obj_set_style_text_color(screen6_status_label, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(screen6_status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen6_status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Selected battery details label (below table)
    screen6_battery_details_label = lv_label_create(screen_6);
    lv_label_set_text(screen6_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen6_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen6_battery_details_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen6_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);

    // Move shared data table to screen_6
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_6);
        lv_obj_set_pos(data_table, 12, 110);
    }
    
    // Timer table (3x2) for screen 6 - shows total time, remaining time, and Ah, left aligned
    screen6_timer_table = lv_table_create(screen_6);
    lv_table_set_col_cnt(screen6_timer_table, 3);
    lv_table_set_row_cnt(screen6_timer_table, 2);
    lv_table_set_col_width(screen6_timer_table, 0, 200);
    lv_table_set_col_width(screen6_timer_table, 1, 200);
    lv_table_set_col_width(screen6_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen6_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen6_timer_table, 0, 1, "Remaining");
    lv_table_set_cell_value(screen6_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen6_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen6_timer_table, 1, 1, "00:00");
    lv_table_set_cell_value(screen6_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen6_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen6_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen6_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen6_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen6_timer_table, LV_ALIGN_TOP_MID, 0, 270);  // Centered horizontally, same vertical position
    lv_obj_clear_flag(screen6_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Home button (bottom mid)
    lv_obj_t* screen6_home_btn = lv_btn_create(screen_6);
    lv_obj_set_size(screen6_home_btn, 200, 80);
    lv_obj_align(screen6_home_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen6_home_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN);  // Blue
    lv_obj_add_event_cb(screen6_home_btn, home_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen6_home_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen6_home_label = lv_label_create(screen6_home_btn);
    lv_label_set_text(screen6_home_label, "Home");
    lv_obj_set_style_text_font(screen6_home_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen6_home_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen6_home_label);

    // Create remove battery popup for screen 6
    screen6_remove_battery_popup = lv_obj_create(screen_6);
    lv_obj_set_size(screen6_remove_battery_popup, 700, 300);
    lv_obj_center(screen6_remove_battery_popup);
    lv_obj_set_style_bg_color(screen6_remove_battery_popup, lv_color_hex(0xFFE4B5), LV_PART_MAIN);  // Moccasin background
    lv_obj_set_style_border_width(screen6_remove_battery_popup, 4, LV_PART_MAIN);
    lv_obj_set_style_border_color(screen6_remove_battery_popup, lv_color_hex(0xFF6600), LV_PART_MAIN);  // Orange border
    lv_obj_set_style_radius(screen6_remove_battery_popup, 15, LV_PART_MAIN);
    lv_obj_add_flag(screen6_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);  // Hidden by default
    
    screen6_remove_battery_label = lv_label_create(screen6_remove_battery_popup);
    lv_label_set_text(screen6_remove_battery_label, "Please remove the battery \nbefore returning to home..");
    lv_obj_set_style_text_font(screen6_remove_battery_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen6_remove_battery_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red text
    lv_obj_set_style_text_align(screen6_remove_battery_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(screen6_remove_battery_label);

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

    // Status label (will be updated dynamically based on stop reason)
    screen7_status_label = lv_label_create(screen_7);
    lv_label_set_text(screen7_status_label, "Charging stopped by user");
    lv_obj_set_style_text_color(screen7_status_label, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
    lv_obj_set_style_text_font(screen7_status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen7_status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Selected battery details label (below table)
    screen7_battery_details_label = lv_label_create(screen_7);
    lv_label_set_text(screen7_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen7_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen7_battery_details_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen7_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);

    // Move shared data table to screen_7
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_7);
        lv_obj_set_pos(data_table, 12, 110);
    }
    
    // Timer table (3x2) for screen 7 - shows total time and Ah, left aligned
    screen7_timer_table = lv_table_create(screen_7);
    lv_table_set_col_cnt(screen7_timer_table, 3);
    lv_table_set_row_cnt(screen7_timer_table, 2);
    lv_table_set_col_width(screen7_timer_table, 0, 200);
    lv_table_set_col_width(screen7_timer_table, 1, 200);
    lv_table_set_col_width(screen7_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen7_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen7_timer_table, 0, 1, "");
    lv_table_set_cell_value(screen7_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen7_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen7_timer_table, 1, 1, "");
    lv_table_set_cell_value(screen7_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen7_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen7_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen7_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen7_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen7_timer_table, LV_ALIGN_TOP_MID, 0, 270);  // Centered horizontally, same vertical position
    lv_obj_clear_flag(screen7_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Home button (bottom mid)
    lv_obj_t* screen7_home_btn = lv_btn_create(screen_7);
    lv_obj_set_size(screen7_home_btn, 200, 80);
    lv_obj_align(screen7_home_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen7_home_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN);  // Blue
    lv_obj_add_event_cb(screen7_home_btn, home_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen7_home_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen7_home_label = lv_label_create(screen7_home_btn);
    lv_label_set_text(screen7_home_label, "Home");
    lv_obj_set_style_text_font(screen7_home_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen7_home_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen7_home_label);

    // Create remove battery popup for screen 7
    screen7_remove_battery_popup = lv_obj_create(screen_7);
    lv_obj_set_size(screen7_remove_battery_popup, 700, 300);
    lv_obj_center(screen7_remove_battery_popup);
    lv_obj_set_style_bg_color(screen7_remove_battery_popup, lv_color_hex(0xFFE4B5), LV_PART_MAIN);  // Moccasin background
    lv_obj_set_style_border_width(screen7_remove_battery_popup, 4, LV_PART_MAIN);
    lv_obj_set_style_border_color(screen7_remove_battery_popup, lv_color_hex(0xFF6600), LV_PART_MAIN);  // Orange border
    lv_obj_set_style_radius(screen7_remove_battery_popup, 15, LV_PART_MAIN);
    lv_obj_add_flag(screen7_remove_battery_popup, LV_OBJ_FLAG_HIDDEN);  // Hidden by default
    
    screen7_remove_battery_label = lv_label_create(screen7_remove_battery_popup);
    lv_label_set_text(screen7_remove_battery_label, "Please remove the battery \nbefore returning to home..");
    lv_obj_set_style_text_font(screen7_remove_battery_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen7_remove_battery_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red text
    lv_obj_set_style_text_align(screen7_remove_battery_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(screen7_remove_battery_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 7 (Emergency Stop) created successfully");
}

//screen 8 - Voltage saturation detected
void create_screen_8(void) {
    screen_8 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_8, lv_color_hex(0xD3D3D3), LV_PART_MAIN);  // Light gray background
    lv_obj_set_style_bg_opa(screen_8, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_8, LV_OPA_COVER, LV_PART_MAIN);

    // Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_8, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_8);
    lv_label_set_text(title, "Voltage Saturation Detected");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (CV charging in progress with saturation voltage)
    lv_obj_t *status_label_8 = lv_label_create(screen_8);
    lv_label_set_text(status_label_8, "CV Charging at saturation voltage...");
    lv_obj_set_style_text_color(status_label_8, lv_color_hex(0x8B0000), LV_PART_MAIN);  // Dark red
    lv_obj_set_style_text_font(status_label_8, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_8, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_8
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_8);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table, same position as screens 3,4,5)
    screen8_battery_details_label = lv_label_create(screen_8);
    lv_label_set_text(screen8_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen8_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen8_battery_details_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(screen8_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 230);
    
    // Temperature label (below battery details, same position and size as screens 3,4,5)
    screen8_temp_label = lv_label_create(screen_8);
    lv_label_set_text(screen8_temp_label, "Motor temp : -- , Gcu temp : --");
    lv_obj_set_style_text_color(screen8_temp_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen8_temp_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(screen8_temp_label, LV_ALIGN_TOP_LEFT, 12, 270);
    
    // Timer table (3x2) for screen 8 - below temp label, same position as screens 3,4,5
    screen8_timer_table = lv_table_create(screen_8);
    lv_table_set_col_cnt(screen8_timer_table, 3);
    lv_table_set_row_cnt(screen8_timer_table, 2);
    lv_table_set_col_width(screen8_timer_table, 0, 200);
    lv_table_set_col_width(screen8_timer_table, 1, 200);
    lv_table_set_col_width(screen8_timer_table, 2, 240);  // Wider so "Charged(Ah)" doesn't wrap
    lv_table_set_cell_value(screen8_timer_table, 0, 0, "Total Time");
    lv_table_set_cell_value(screen8_timer_table, 0, 1, "Remaining");
    lv_table_set_cell_value(screen8_timer_table, 0, 2, "Charged(Ah)");
    lv_table_set_cell_value(screen8_timer_table, 1, 0, "00:00:00");
    lv_table_set_cell_value(screen8_timer_table, 1, 1, "00:00");
    lv_table_set_cell_value(screen8_timer_table, 1, 2, "0.0");
    lv_obj_set_style_bg_color(screen8_timer_table, lv_color_hex(0xDDA0DD), LV_PART_ITEMS);  // Light purple
    lv_obj_set_style_border_color(screen8_timer_table, lv_color_hex(0x000000), LV_PART_ITEMS);  // Black border
    lv_obj_set_style_border_width(screen8_timer_table, 3, LV_PART_ITEMS);  // Thick black border
    lv_obj_set_style_text_font(screen8_timer_table, &lv_font_montserrat_26, LV_PART_ITEMS);
    lv_obj_align(screen8_timer_table, LV_ALIGN_TOP_MID, 0, 330);  // Same position as screens 3,4,5
    lv_obj_clear_flag(screen8_timer_table, LV_OBJ_FLAG_SCROLLABLE);


    // Emergency Stop button (bottom mid, large and visible, one-line label) - ALWAYS ENABLED
    lv_obj_t* screen8_emergency_stop_btn = lv_btn_create(screen_8);
    lv_obj_set_size(screen8_emergency_stop_btn, 360, 80);
    lv_obj_align(screen8_emergency_stop_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(screen8_emergency_stop_btn, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red
    lv_obj_add_event_cb(screen8_emergency_stop_btn, emergency_stop_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen8_emergency_stop_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* screen8_emergency_stop_label = lv_label_create(screen8_emergency_stop_btn);
    lv_label_set_text(screen8_emergency_stop_label, "EMERGENCY STOP");
    lv_obj_set_style_text_font(screen8_emergency_stop_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen8_emergency_stop_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen8_emergency_stop_label);

    // Note: Screen loading is handled by switch_to_screen()
    Serial.println("[SCREEN] Screen 8 (Voltage Saturation) created successfully");
}

// Screen 13 - CAN debug screen
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


    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_16); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 16 (Time Debug) created successfully");
}

// Screen 18 - M2 connection failed or lost (no buttons; restart only option)
void create_screen_18(void) {
    screen_18 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_18, lv_color_hex(0xFF6B61), LV_PART_MAIN);  // Light red background
    lv_obj_set_style_bg_opa(screen_18, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_18, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_scroll_dir(screen_18, LV_DIR_NONE);

    // Labels above table (table at y=110, ~100px tall)
    lv_obj_t* title = lv_label_create(screen_18);
    lv_label_set_text(title, "Connection failed or lost with M2 V4.1"); //update ver number here too
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t* msg = lv_label_create(screen_18);
    lv_label_set_text(msg, "Contactor open, motor stopped.");
    lv_obj_set_style_text_color(msg, lv_color_hex(0x8B0000), LV_PART_MAIN);
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 235);

    lv_obj_t* msg2 = lv_label_create(screen_18);
    lv_label_set_text(msg2, "Check with M2 and Restart device.");
    lv_obj_set_style_text_color(msg2, lv_color_hex(0x8B0000), LV_PART_MAIN);
    lv_obj_set_style_text_font(msg2, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_align(msg2, LV_ALIGN_TOP_MID, 0, 280);

    // Table same position as screen 1
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_18);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // M2 RTC time label 20px below table (same position and style as screen 1)
    screen18_rtc_time_label = lv_label_create(screen_18);
    lv_label_set_text(screen18_rtc_time_label, "M2 rtc time: -- --");
    lv_obj_set_style_text_font(screen18_rtc_time_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen18_rtc_time_label, lv_color_hex(0x000000), LV_PART_MAIN);
    //lv_obj_align(screen18_rtc_time_label, LV_ALIGN_TOP_LEFT, 12, 330);  // 20px below table (110 + ~100 + 20)
    lv_obj_align(screen18_rtc_time_label, LV_ALIGN_TOP_MID, 0, 350);  // screen center below table

    Serial.println("[SCREEN] Screen 18 (M2 connection lost) created successfully");
}
