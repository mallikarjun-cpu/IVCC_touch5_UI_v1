#include "screen_definitions.h"
#include "can_twai.h"
#include "sd_logging.h"
#include "esp_panel_board_custom_conf.h"
#include "lvgl_v8_port.h"
#include "rs485_vfdComs.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <WiFi.h>

// External sensor data
extern struct sensor_data {
    float volt;
    float curr;
    uint temp1;
} sensorData;

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
lv_obj_t* screen_1 = nullptr;
lv_obj_t* screen_2 = nullptr;
lv_obj_t* screen_3 = nullptr;
lv_obj_t* screen_11 = nullptr;
lv_obj_t* screen_12 = nullptr;

// Shared UI objects (reused between screens)
static lv_obj_t* status_label = nullptr;
static lv_obj_t* data_table = nullptr;  // Shared table shown on all screens
// Battery container references for screens that need profiles
static lv_obj_t* screen2_battery_container = nullptr;
static lv_obj_t* screen2_button_container = nullptr;
static lv_obj_t* screen11_battery_container = nullptr;

// Selected battery profile for screen 3 display
static BatteryType* selected_battery_profile = nullptr;
static lv_obj_t* screen3_battery_details_label = nullptr;

// screen 2 confirmation popup
static lv_obj_t* screen2_confirm_popup = nullptr;
static lv_obj_t* screen2_confirm_title_label = nullptr;
static lv_obj_t* screen2_confirm_voltage_label = nullptr;
static lv_obj_t* screen2_confirm_capacity_label = nullptr;
static lv_obj_t* screen2_confirm_current_label = nullptr;
static lv_obj_t* screen2_confirm_type_label = nullptr;
static lv_obj_t* screen2_confirm_agree_btn = nullptr;
static lv_obj_t* screen2_confirm_change_btn = nullptr;

// screen 11 button objects
static lv_obj_t* screen11_button_container = nullptr;
static lv_obj_t* screen11_stop_button = nullptr;
static lv_obj_t* screen11_start_button = nullptr;

// screen 11 confirmation popup objects
static lv_obj_t* screen11_confirm_popup = nullptr;
static lv_obj_t* screen11_confirm_title_label = nullptr;
static lv_obj_t* screen11_confirm_voltage_label = nullptr;
static lv_obj_t* screen11_confirm_capacity_label = nullptr;
static lv_obj_t* screen11_confirm_current_label = nullptr;
static lv_obj_t* screen11_confirm_type_label = nullptr;
static lv_obj_t* screen11_confirm_agree_btn = nullptr;
static lv_obj_t* screen11_confirm_change_btn = nullptr;

// screen 11 back button
static lv_obj_t* screen11_back_btn = nullptr;

// screen 12 connection status label
static lv_obj_t* screen12_connection_status = nullptr;

// ============================================================================
// Screen Management Globals
// ============================================================================

// Current screen and state tracking
screen_id_t current_screen_id = SCREEN_HOME;
app_state_t current_app_state = STATE_HOME;

// State-based screen switching triggers
bool battery_detected = false;
bool wifi_config_requested = false;

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
    else if (strcmp(screen_name, "screen_11") == 0) screen_index = 11;
    else if (strcmp(screen_name, "screen_12") == 0) screen_index = 12;

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
void displayAllBatteryProfiles(lv_obj_t* container);
void displayMatchingBatteryProfiles(float detectedVoltage, lv_obj_t* container);

// ============================================================================
// Screen Management Functions
// ============================================================================

// Initialize all screens at startup
void initialize_all_screens() {
    Serial.println("[SCREEN] Creating all screens...");

    create_screen_1();   // Home screen
    create_screen_2();   // Battery detected screen
    create_screen_3();   // Charging started screen
    create_screen_11();  // Battery profiles screen
    create_screen_12();  // WiFi config screen

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
        case SCREEN_BATTERY_PROFILES:
            target_screen = screen_11;
            break;
        case SCREEN_WIFI_CONFIG:
            target_screen = screen_12;
            break;
        default:
            Serial.printf("[SCREEN] ERROR: Invalid screen ID %d\n", screen_id);
            return;
    }

    if (target_screen != nullptr) {
        // Update current screen tracking
        current_screen_id = screen_id;

        // Move shared UI elements to the new screen (except screen 12)
        if (data_table != nullptr && screen_id != SCREEN_WIFI_CONFIG) {
            lv_obj_set_parent(data_table, target_screen);
            lv_obj_set_pos(data_table, 12, 110);
            lv_obj_clear_flag(data_table, LV_OBJ_FLAG_HIDDEN); // Make sure table is visible
        } else if (screen_id == SCREEN_WIFI_CONFIG && data_table != nullptr) {
            // Hide table on WiFi config screen
            lv_obj_add_flag(data_table, LV_OBJ_FLAG_HIDDEN);
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
        if (screen11_battery_container != nullptr) {
            if (screen_id == SCREEN_BATTERY_PROFILES) {
                lv_obj_clear_flag(screen11_battery_container, LV_OBJ_FLAG_HIDDEN);
                // Refresh profiles display when switching to screen 11
                displayAllBatteryProfiles(screen11_battery_container);
            } else {
                lv_obj_add_flag(screen11_battery_container, LV_OBJ_FLAG_HIDDEN);
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

// Update current screen content (screen-specific updates)
void update_current_screen() {
    // WiFi connection status updates only when on WiFi config screen
    if (current_screen_id == SCREEN_WIFI_CONFIG) {
        update_wifi_connection_status();
    }

    // Update battery details on screen 3
    if (current_screen_id == SCREEN_CHARGING_STARTED && screen3_battery_details_label != nullptr) {
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
}

// Determine which screen should be shown based on current state
screen_id_t determine_screen_from_state() {
    // Priority-based screen selection
    if (wifi_config_requested) {
        return SCREEN_WIFI_CONFIG;
    }
    if (current_app_state == STATE_CHARGING_STARTED) {
        return SCREEN_CHARGING_STARTED;
    }
    if (current_app_state == STATE_BATTERY_PROFILES) {
        return SCREEN_BATTERY_PROFILES;
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
void screen11_profile_selected_event_handler(lv_event_t * e);
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

        // Add event handler for profile selection - use different handler for screen2
        if (container == screen2_battery_container) {
            lv_obj_add_event_cb(profile_btn, screen2_profile_selected_event_handler, LV_EVENT_CLICKED, (void*)profile);
        } else {
            lv_obj_add_event_cb(profile_btn, screen11_profile_selected_event_handler, LV_EVENT_CLICKED, (void*)profile);
        }

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

// Display all available battery profiles in the specified container
void displayAllBatteryProfiles(lv_obj_t* container) {
    Serial.println("[BATTERY] displayAllBatteryProfiles() called");

    if (!container) {
        Serial.println("[ERROR] Battery list container not provided");
        return;
    }

    // Clear any existing content
    lv_obj_clean(container);

    // Make container visible
    lv_obj_clear_flag(container, LV_OBJ_FLAG_HIDDEN);

    int profileCount = batteryProfiles.getProfileCount();
    Serial.printf("[BATTERY] Displaying all %d profiles\n", profileCount);

    if (profileCount == 0) {
        Serial.println("[BATTERY] No profiles found! Check if battery profiles are loaded.");
        lv_obj_t* no_profiles_label = lv_label_create(container);
        lv_label_set_text(no_profiles_label, "No battery profiles loaded!");
        lv_obj_set_style_text_font(no_profiles_label, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(no_profiles_label, lv_color_hex(0xFF0000), LV_PART_MAIN);
        lv_obj_center(no_profiles_label);
        return;
    }

    // Create a scrollable list of all battery profiles
    int button_y = 10; // Start Y position for first button
    const int button_height = 60;
    const int button_spacing = 5;

    for (int i = 0; i < batteryProfiles.getProfileCount(); i++) {
        BatteryType* profile = batteryProfiles.getProfile(i);
        if (!profile) continue;

        // Create a button for each battery profile
        lv_obj_t* profile_btn = lv_btn_create(container);
        lv_obj_set_size(profile_btn, 900, button_height);
        lv_obj_set_pos(profile_btn, 10, button_y); // Position with X=10, increasing Y
        lv_obj_set_style_bg_color(profile_btn, lv_color_hex(0xE0E0E0), LV_PART_MAIN);
        lv_obj_set_style_border_width(profile_btn, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(profile_btn, lv_color_hex(0x808080), LV_PART_MAIN);

        // Add event handler for profile selection
        lv_obj_add_event_cb(profile_btn, screen11_profile_selected_event_handler, LV_EVENT_CLICKED, (void*)profile);

        // Create label with battery info
        lv_obj_t* profile_label = lv_label_create(profile_btn);
        lv_label_set_text(profile_label, profile->getDisplayName().c_str());
        lv_obj_set_style_text_font(profile_label, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(profile_label, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_center(profile_label);

        Serial.printf("[BATTERY] Added profile at Y=%d: %s\n", button_y, profile->getDisplayName().c_str());

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

        // Update temperature (column 2)
        lv_table_set_cell_value(data_table, 1, 2,
            String(sensorData.temp1).c_str());

        // Update frequency (column 3) - placeholder for now
        lv_table_set_cell_value(data_table, 1, 3, "--");

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

        // Update UI to show connected status
        if (screen12_connection_status != nullptr) {
            lv_label_set_text(screen12_connection_status, "WiFi Connected Successfully!");
            lv_obj_set_style_text_color(screen12_connection_status, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green for success
        }

        wifi_connection_in_progress = false;
        wifi_connection_attempts = 0;
        return;
    }

    // Check if connection timed out
    if (current_time - wifi_connection_start_time >= WIFI_CONNECTION_TIMEOUT) {
        Serial.println("[WIFI] Connection failed!");
        Serial.printf("[WIFI] WiFi status: %d\n", wifi_status);

        // Update UI to show failure status
        if (screen12_connection_status != nullptr) {
            lv_label_set_text(screen12_connection_status, "WiFi Connection Failed!");
            lv_obj_set_style_text_color(screen12_connection_status, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for failure
        }

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

        // Update LVGL label with progress
        if (screen12_connection_status != nullptr) {
            char status_text[60];
            sprintf(status_text, "Connecting to WiFi (%d/%d)", wifi_connection_attempts, WIFI_MAX_ATTEMPTS);
            lv_label_set_text(screen12_connection_status, status_text);
            lv_obj_set_style_text_color(screen12_connection_status, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green
        }
    }
}

// ============================================================================
// Event Handlers
// ============================================================================

// Screen 1 battery profiles button event handler
void screen1_bat_profiles_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] Switching to battery profiles screen");
        // Update state and switch screen
        current_app_state = STATE_BATTERY_PROFILES;
        wifi_config_requested = false; // Clear other state flags
        switch_to_screen(SCREEN_BATTERY_PROFILES);
    }
}

// Screen 1 WiFi button event handler
void screen1_wifi_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] Switching to WiFi config screen");
        // Update state and switch screen
        current_app_state = STATE_WIFI_CONFIG;
        wifi_config_requested = true;
        switch_to_screen(SCREEN_WIFI_CONFIG);
    }
}

void screen2_confirm_agree_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN2] CONFIRM AGREE button pressed - battery confirmed for charging");

        // Send CAN test frame with ID 0x100 (256 decimal)
        uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        if (send_can_frame(SENSOR_FRAME_ID, test_data, 8)) {
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

        // Send RS485 start command to VFD
        rs485_sendStartCommand();
        Serial.println("Start cmd sent to vfd, going to scrn3 now.");

        // Switch to charging started state
        current_app_state = STATE_CHARGING_STARTED;
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

// screen 11 event handlers
void screen11_profile_selected_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    Serial.printf("[SCREEN11] Event handler called, code: %d\n", code);

    if(code == LV_EVENT_CLICKED) {
        BatteryType* selected_profile = (BatteryType*)lv_event_get_user_data(e);
        if (selected_profile) {
            Serial.printf("[SCREEN11] Profile selected: %s\n", selected_profile->getDisplayName().c_str());

            // Update confirmation popup with selected profile details
            lv_label_set_text_fmt(screen11_confirm_voltage_label, "Voltage: %d V", selected_profile->getRatedVoltage());
            lv_label_set_text_fmt(screen11_confirm_capacity_label, "Capacity: %d Ah", selected_profile->getRatedAh());
            lv_label_set_text_fmt(screen11_confirm_current_label, "Current: %.1f A", selected_profile->getConstCurrent());
            lv_label_set_text_fmt(screen11_confirm_type_label, "Type: %s", getBatteryChemistryName(selected_profile));

            // Show confirmation popup and bring it to foreground
            lv_obj_clear_flag(screen11_confirm_popup, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(screen11_confirm_popup);
        }
    }
}

void screen11_stop_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN11] STOP button pressed");
        // TODO: Implement stop functionality
    }
}

void screen11_start_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN11] START button pressed");
        // TODO: Implement start functionality
    }
}

void screen11_confirm_agree_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN11] CONFIRM AGREE button pressed - sending CAN test frame");

        // Send CAN test frame with ID 0x100 (256 decimal)
        uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        if (send_can_frame(SENSOR_FRAME_ID, test_data, 8)) {
            Serial.println("[CAN] Test frame sent successfully");
        } else {
            Serial.println("[CAN] Failed to send test frame");
        }

        // Hide the confirmation popup
        lv_obj_add_flag(screen11_confirm_popup, LV_OBJ_FLAG_HIDDEN);
    }
}

void screen11_confirm_change_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN11] CONFIRM CHANGE button pressed - testing only");
        // Hide the confirmation popup (testing functionality)
        lv_obj_add_flag(screen11_confirm_popup, LV_OBJ_FLAG_HIDDEN);
    }
}

// screen 11 back button event handler (battery profiles)
void screen11_back_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] BACK button pressed - returning to home screen");
        // Reset state and return to home
        current_app_state = STATE_HOME;
        wifi_config_requested = false;
        switch_to_screen(SCREEN_HOME);
    }
}

// Screen 12 Connect button event handler
void screen12_connect_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[WIFI] Connect button pressed - starting WiFi connection");
        Serial.printf("[WIFI] Attempting to connect to SSID: %s\n", TEST_WIFI_SSID);

        // Start asynchronous WiFi connection
        WiFi.begin(TEST_WIFI_SSID, TEST_WIFI_PASSWORD);

        // Set connection state
        wifi_connection_in_progress = true;
        wifi_connection_start_time = millis();
        wifi_connection_attempts = 0;

        // Initial UI update
        if (screen12_connection_status != nullptr) {
            lv_label_set_text(screen12_connection_status, "Connecting to WiFi...");
            lv_obj_set_style_text_color(screen12_connection_status, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green
        }
    }
}

// screen 12 back button event handler (WiFi config)
void screen12_back_button_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        Serial.println("[SCREEN] BACK button pressed - returning to home screen");
        // Reset state and return to home
        current_app_state = STATE_HOME;
        wifi_config_requested = false;
        switch_to_screen(SCREEN_HOME);
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
        lv_table_set_col_width(data_table, 3, 198);  // Freq
        lv_table_set_col_width(data_table, 4, 198);  // Entry

        // Headers (Row 0)
        lv_table_set_cell_value(data_table, 0, 0, "Volt");
        lv_table_set_cell_value(data_table, 0, 1, "Curr");
        lv_table_set_cell_value(data_table, 0, 2, "Temp1");
        lv_table_set_cell_value(data_table, 0, 3, "Freq");
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

    // Battery Profiles button
    lv_obj_t* screen1_bat_profiles_btn = lv_btn_create(screen1_button_container);
    lv_obj_set_size(screen1_bat_profiles_btn, 300, 80);
    lv_obj_set_style_bg_color(screen1_bat_profiles_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN);  // Blue button
    lv_obj_add_event_cb(screen1_bat_profiles_btn, screen1_bat_profiles_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen1_bat_profiles_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* screen1_bat_profiles_label = lv_label_create(screen1_bat_profiles_btn);
    lv_label_set_text(screen1_bat_profiles_label, "Bat Profiles");
    lv_obj_set_style_text_font(screen1_bat_profiles_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_bat_profiles_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen1_bat_profiles_label);

    // WiFi button (next to battery profiles button)
    lv_obj_t* screen1_wifi_btn = lv_btn_create(screen1_button_container);
    lv_obj_set_size(screen1_wifi_btn, 300, 80);
    lv_obj_set_style_bg_color(screen1_wifi_btn, lv_color_hex(0x228B22), LV_PART_MAIN);  // Green button
    lv_obj_add_event_cb(screen1_wifi_btn, screen1_wifi_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen1_wifi_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* screen1_wifi_label = lv_label_create(screen1_wifi_btn);
    lv_label_set_text(screen1_wifi_label, "WiFi Config");
    lv_obj_set_style_text_font(screen1_wifi_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen1_wifi_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(screen1_wifi_label);

    // v4.50: Add M2 state status box (top-left corner) - Screen 1
    createM2StateBox(screen_1, "screen_1");

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_1); // Removed - handled by screen manager
    Serial.println("[SCREEN] Screen 1 created successfully");
}


//screen 2 - battery detected, charge ready page
void create_screen_2(void) {
    // Create screen 2 (similar to screen 11 but with filtered battery list)
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

    // Status label (charging in progress)
    lv_obj_t *status_label_3 = lv_label_create(screen_3);
    lv_label_set_text(status_label_3, "Charging in progress...");
    lv_obj_set_style_text_color(status_label_3, lv_color_hex(0x006400), LV_PART_MAIN);  // Dark green
    lv_obj_set_style_text_font(status_label_3, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label_3, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_3
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_3);
        lv_obj_set_pos(data_table, 12, 110);
    }

    // Battery profile details label (below table)
    screen3_battery_details_label = lv_label_create(screen_3);
    lv_label_set_text(screen3_battery_details_label, "Selected Battery: --");
    lv_obj_set_style_text_color(screen3_battery_details_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(screen3_battery_details_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(screen3_battery_details_label, LV_ALIGN_TOP_LEFT, 12, 350);  // Below table

    // Add M2 state status box - screen 3
    createM2StateBox(screen_3, "screen_3");

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_3); // Removed - handled by screen manager
    Serial.println("[SCREEN] Screen 3 created successfully");
}


//screen 11 , confirmation etc
void create_screen_11(void) {
    screen_11 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_11, lv_color_hex(0xADD8E6), LV_PART_MAIN);  // Light blue background
    lv_obj_set_style_bg_opa(screen_11, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_11, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_11, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_11);
    lv_label_set_text(title, "GCU 3kW Charger v1.0");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label (battery detected, etc)
    status_label = lv_label_create(screen_11);
    lv_label_set_text(status_label, "Available profiles.");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 60);

    // Move shared data table to screen_11
    if (data_table != nullptr) {
        lv_obj_set_parent(data_table, screen_11);
        lv_obj_set_pos(data_table, 12, 110);
    }
    
    // v4.08: Battery list container (shown with all profiles on screen 11)
    screen11_battery_container = lv_obj_create(screen_11);
    lv_obj_set_size(screen11_battery_container, 950, 280);
    lv_obj_set_pos(screen11_battery_container, 37, 260);  // Below table
    lv_obj_set_style_bg_color(screen11_battery_container, lv_color_hex(0xF0F0F0), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen11_battery_container, 2, LV_PART_MAIN);
    lv_obj_set_scroll_dir(screen11_battery_container, LV_DIR_VER);  // Vertical scroll
    lv_obj_add_flag(screen11_battery_container, LV_OBJ_FLAG_HIDDEN);  // Hidden initially, shown by screen manager

    // v4.08: Button container (hidden by default, shown after profile selection)
    screen11_button_container = lv_obj_create(screen_11);
    lv_obj_set_size(screen11_button_container, ESP_PANEL_BOARD_WIDTH, 100);
    lv_obj_align(screen11_button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(screen11_button_container, lv_color_hex(0x87CEEB), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen11_button_container, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(screen11_button_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(screen11_button_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(screen11_button_container, LV_OBJ_FLAG_HIDDEN);  // Hidden initially
    lv_obj_clear_flag(screen11_button_container, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling

    // STOP button (in button_container)
    screen11_stop_button = lv_btn_create(screen11_button_container);
    lv_obj_set_size(screen11_stop_button, 300, 80);
    lv_obj_set_style_bg_color(screen11_stop_button, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_add_event_cb(screen11_stop_button, screen11_stop_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen11_stop_button, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling
    lv_obj_t *screen11_stop_label = lv_label_create(screen11_stop_button);
    lv_label_set_text(screen11_stop_label, "STOP");
    lv_obj_set_style_text_font(screen11_stop_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen11_stop_label);

    // START button (in button_container)
    screen11_start_button = lv_btn_create(screen11_button_container);
    lv_obj_set_size(screen11_start_button, 300, 80);
    lv_obj_set_style_bg_color(screen11_start_button, lv_color_hex(0x00AA00), LV_PART_MAIN);
    lv_obj_add_event_cb(screen11_start_button, screen11_start_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(screen11_start_button, LV_OBJ_FLAG_SCROLLABLE);  // v4.26: Disable scrolling
    lv_obj_t *screen11_start_label = lv_label_create(screen11_start_button);
    lv_label_set_text(screen11_start_label, "START");
    lv_obj_set_style_text_font(screen11_start_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen11_start_label);

    // Add M2 state status box - screen 11
    createM2StateBox(screen_11, "screen_11");

    // Add back button (top right)
    screen11_back_btn = lv_btn_create(screen_11);
    lv_obj_set_size(screen11_back_btn, 100, 50);
    lv_obj_align(screen11_back_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(screen11_back_btn, lv_color_hex(0xFF4444), LV_PART_MAIN);  // Red back button
    lv_obj_add_event_cb(screen11_back_btn, screen11_back_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(screen11_back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);

    // v4.60: Create confirmation popup (light gray background, multiple labels for different fonts)
    screen11_confirm_popup = lv_obj_create(screen_11);
    lv_obj_set_size(screen11_confirm_popup, 750, 500);
    lv_obj_center(screen11_confirm_popup);
    lv_obj_set_style_bg_color(screen11_confirm_popup, lv_color_hex(0xD3D3D3), LV_PART_MAIN);  // Light gray background
    lv_obj_set_style_border_width(screen11_confirm_popup, 4, LV_PART_MAIN);
    lv_obj_set_style_border_color(screen11_confirm_popup, lv_color_hex(0x000000), LV_PART_MAIN);  // Black border
    lv_obj_set_style_radius(screen11_confirm_popup, 15, LV_PART_MAIN);
    lv_obj_add_flag(screen11_confirm_popup, LV_OBJ_FLAG_HIDDEN);  // Hidden by default

    // Title label
    screen11_confirm_title_label = lv_label_create(screen11_confirm_popup);
    lv_label_set_text(screen11_confirm_title_label, "Confirm Battery:");
    lv_obj_set_style_text_font(screen11_confirm_title_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen11_confirm_title_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen11_confirm_title_label, LV_ALIGN_TOP_MID, 0, 20);

    // Voltage label (30pt - large and bold-looking)
    screen11_confirm_voltage_label = lv_label_create(screen11_confirm_popup);
    lv_label_set_text(screen11_confirm_voltage_label, "Voltage: -- V");
    lv_obj_set_style_text_font(screen11_confirm_voltage_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen11_confirm_voltage_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen11_confirm_voltage_label, LV_ALIGN_TOP_MID, 0, 80);

    // Capacity label (30pt - large and bold-looking)
    screen11_confirm_capacity_label = lv_label_create(screen11_confirm_popup);
    lv_label_set_text(screen11_confirm_capacity_label, "Capacity: -- Ah");
    lv_obj_set_style_text_font(screen11_confirm_capacity_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen11_confirm_capacity_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen11_confirm_capacity_label, LV_ALIGN_TOP_MID, 0, 140);

    // Current label (26pt)
    screen11_confirm_current_label = lv_label_create(screen11_confirm_popup);
    lv_label_set_text(screen11_confirm_current_label, "Current: -- A");
    lv_obj_set_style_text_font(screen11_confirm_current_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen11_confirm_current_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen11_confirm_current_label, LV_ALIGN_TOP_MID, 0, 210);

    // Type label (26pt)
    screen11_confirm_type_label = lv_label_create(screen11_confirm_popup);
    lv_label_set_text(screen11_confirm_type_label, "Type: --");
    lv_obj_set_style_text_font(screen11_confirm_type_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(screen11_confirm_type_label, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_align(screen11_confirm_type_label, LV_ALIGN_TOP_MID, 0, 270);

    // AGREE button
    screen11_confirm_agree_btn = lv_btn_create(screen11_confirm_popup);
    lv_obj_set_size(screen11_confirm_agree_btn, 300, 90);  // Bigger buttons
    lv_obj_align(screen11_confirm_agree_btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    lv_obj_set_style_bg_color(screen11_confirm_agree_btn, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green
    lv_obj_add_event_cb(screen11_confirm_agree_btn, screen11_confirm_agree_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t *screen11_agree_label = lv_label_create(screen11_confirm_agree_btn);
    lv_label_set_text(screen11_agree_label, "AGREE");
    lv_obj_set_style_text_font(screen11_agree_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen11_agree_label);

    // CHANGE button
    screen11_confirm_change_btn = lv_btn_create(screen11_confirm_popup);
    lv_obj_set_size(screen11_confirm_change_btn, 300, 90);  // Bigger buttons
    lv_obj_align(screen11_confirm_change_btn, LV_ALIGN_BOTTOM_RIGHT, -30, -30);
    lv_obj_set_style_bg_color(screen11_confirm_change_btn, lv_color_hex(0xFF6600), LV_PART_MAIN);  // Orange
    lv_obj_add_event_cb(screen11_confirm_change_btn, screen11_confirm_change_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t *screen11_change_label = lv_label_create(screen11_confirm_change_btn);
    lv_label_set_text(screen11_change_label, "CHANGE");
    lv_obj_set_style_text_font(screen11_change_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_center(screen11_change_label);

    // Display all available battery profiles
    displayAllBatteryProfiles(screen11_battery_container);

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_11); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 11 created successfully");
}

//screen 12 - wifi configs
void create_screen_12(void) {
    screen_12 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_12, lv_color_hex(0xADD8E6), LV_PART_MAIN);  // Light blue background
    lv_obj_set_style_bg_opa(screen_12, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_12, LV_OPA_COVER, LV_PART_MAIN);

    // v4.09: Screen NOT scrollable (fixed layout)
    lv_obj_set_scroll_dir(screen_12, LV_DIR_NONE);  // No scrolling

    // Title
    lv_obj_t *title = lv_label_create(screen_12);
    lv_label_set_text(title, "WiFi Configs");
    lv_obj_set_style_text_color(title, lv_color_hex(0x000000), LV_PART_MAIN);  // Black text
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);  // Use available font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Status label
    lv_obj_t *status_label = lv_label_create(screen_12);
    lv_label_set_text(status_label, "Test WiFi Connection");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), LV_PART_MAIN);  // Red for visibility
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 60);

    // WiFi info display
    lv_obj_t *wifi_info = lv_label_create(screen_12);
    lv_label_set_text_fmt(wifi_info, "SSID: %s\nPassword: %s", TEST_WIFI_SSID, TEST_WIFI_PASSWORD);
    lv_obj_set_style_text_color(wifi_info, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_font(wifi_info, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(wifi_info, LV_ALIGN_TOP_MID, 0, 120);

    // Connection status label
    screen12_connection_status = lv_label_create(screen_12);
    lv_label_set_text(screen12_connection_status, "Not Connected");
    lv_obj_set_style_text_color(screen12_connection_status, lv_color_hex(0x666666), LV_PART_MAIN);  // Gray
    lv_obj_set_style_text_font(screen12_connection_status, &lv_font_montserrat_22, LV_PART_MAIN);
    lv_obj_align(screen12_connection_status, LV_ALIGN_TOP_MID, 0, 200);

    // Connect button
    lv_obj_t *connect_btn = lv_btn_create(screen_12);
    lv_obj_set_size(connect_btn, 300, 80);
    lv_obj_align(connect_btn, LV_ALIGN_TOP_MID, 0, 250);
    lv_obj_set_style_bg_color(connect_btn, lv_color_hex(0x00AA00), LV_PART_MAIN);  // Green button
    lv_obj_add_event_cb(connect_btn, screen12_connect_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_clear_flag(connect_btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "CONNECT");
    lv_obj_set_style_text_font(connect_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(connect_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);  // White text
    lv_obj_center(connect_label);

    // Add M2 state status box (same position as screen 1)
    createM2StateBox(screen_12, "screen_12");

    // Back button (top right)
    lv_obj_t *screen12_back_btn = lv_btn_create(screen_12);
    lv_obj_set_size(screen12_back_btn, 100, 50);
    lv_obj_align(screen12_back_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(screen12_back_btn, lv_color_hex(0xFF4444), LV_PART_MAIN);  // Red back button
    lv_obj_add_event_cb(screen12_back_btn, screen12_back_button_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_label = lv_label_create(screen12_back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);

    // Note: Screen loading is handled by switch_to_screen()
    // lv_scr_load(screen_12); // Removed - handled by screen manager

    Serial.println("[SCREEN] Screen 12 created successfully");
}
