#ifndef SCREEN_DEFINITIONS_H
#define SCREEN_DEFINITIONS_H

#include <lvgl.h>
#include "battery_types.h"

// Global screen objects
extern lv_obj_t* screen_1;
extern lv_obj_t* screen_11;
extern lv_obj_t* screen_2;
extern lv_obj_t* screen_3;
extern lv_obj_t* screen_4;
extern lv_obj_t* screen_5;
extern lv_obj_t* screen_6;
extern lv_obj_t* screen_7;
extern lv_obj_t* screen_12;
extern lv_obj_t* screen_13;

// Screen management enums and variables
typedef enum {
    SCREEN_HOME = 1,           // Screen 1 - Default/Home screen
    SCREEN_BATTERY_DETECTED,   // Screen 2 - Battery detected/charging
    SCREEN_CHARGING_STARTED,   // Screen 3 - Charging start (waiting for 1A)
    SCREEN_CHARGING_CC,        // Screen 4 - Constant Current mode
    SCREEN_CHARGING_CV,        // Screen 5 - Constant Voltage mode
    SCREEN_CHARGING_COMPLETE,  // Screen 6 - Charging complete
    SCREEN_EMERGENCY_STOP,     // Screen 7 - Emergency stop
    SCREEN_BATTERY_PROFILES,   // Screen 11 - Battery profiles
    SCREEN_WIFI_CONFIG,        // Screen 12 - WiFi configuration
    SCREEN_CAN_DEBUG           // Screen 13 - CAN debug screen
} screen_id_t;

typedef enum {
    STATE_HOME = 0,               // Home state
    STATE_BATTERY_DETECTED,       // Battery detected
    STATE_CHARGING_START,         // Charging start (waiting for 1A current)
    STATE_CHARGING_CC,            // Constant Current mode
    STATE_CHARGING_CV,            // Constant Voltage mode
    STATE_CHARGING_COMPLETE,      // Charging complete
    STATE_EMERGENCY_STOP,         // Emergency stop
    STATE_BATTERY_PROFILES = 11,  // Battery profiles menu
    STATE_WIFI_CONFIG = 12        // WiFi configuration
} app_state_t;

// Charge stop reason enum
typedef enum {
    CHARGE_STOP_NONE = 0,              // No stop reason
    CHARGE_STOP_COMPLETE = 1,          // Charging complete (normal termination)
    CHARGE_STOP_EMERGENCY = 2,         // Emergency stop (user initiated)
    // Future reasons can be added here
} charge_stop_reason_t;

extern screen_id_t current_screen_id;
extern app_state_t current_app_state;
extern charge_stop_reason_t charge_stop_reason;

// M2 Status Manager Struct
typedef struct {
    lv_obj_t* state_box;
    lv_obj_t* state_label;
    m2_state_t current_state;
    uint32_t last_update_time;
    bool is_connected;
} m2_status_manager_t;

// Function declarations for creating custom UI screens
void create_screen_1(void); //screen 1 - default screen
void create_screen_11(void); //screen 11 - all battery profiles
void create_screen_2(void); //screen 2 - battery detected, charge ready page
void create_screen_3(void); //screen 3 - charging start (waiting for 1A)
void create_screen_4(void); //screen 4 - Constant Current (CC) mode
void create_screen_5(void); //screen 5 - Constant Voltage (CV) mode
void create_screen_6(void); //screen 6 - Charging complete
void create_screen_7(void); //screen 7 - Emergency stop
void create_screen_12(void); //screen 12 - wifi configs
void create_screen_13(void); //screen 13 - CAN debug screen

// Screen management functions
void initialize_all_screens(void);
void switch_to_screen(screen_id_t screen_id);
void update_current_screen(void);
screen_id_t determine_screen_from_state(void);
void update_screen_based_on_state(void);

// Table and UI update functions
void update_table_values(void);
void update_m2_state_display(void);
void update_can_debug_display(uint32_t id, uint8_t* data, uint8_t length);
void update_charging_control(void); // Charging control logic (CC/CV)

// M2 State Management Functions
void updateM2State(m2_state_t new_state);
void updateM2ConnectionStatus(bool connected);
const m2_state_config_t* getM2StateConfig(m2_state_t state);

#endif // SCREEN_DEFINITIONS_H
