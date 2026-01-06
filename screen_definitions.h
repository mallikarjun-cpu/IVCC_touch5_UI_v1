#ifndef SCREEN_DEFINITIONS_H
#define SCREEN_DEFINITIONS_H

#include <lvgl.h>
#include "battery_types.h"

// Global screen objects
extern lv_obj_t* screen_1;
extern lv_obj_t* screen_11;
extern lv_obj_t* screen_2;
extern lv_obj_t* screen_3;
extern lv_obj_t* screen_12;

// Screen management enums and variables
typedef enum {
    SCREEN_HOME = 1,           // Screen 1 - Default/Home screen
    SCREEN_BATTERY_DETECTED,   // Screen 2 - Battery detected/charging
    SCREEN_CHARGING_STARTED,   // Screen 3 - Charging started
    SCREEN_BATTERY_PROFILES,   // Screen 11 - Battery profiles
    SCREEN_WIFI_CONFIG         // Screen 12 - WiFi configuration
} screen_id_t;

typedef enum {
    STATE_HOME = 0,               // Home state
    STATE_BATTERY_DETECTED,       // Battery detected
    STATE_CHARGING_STARTED,       // Charging started
    STATE_BATTERY_PROFILES = 11,       // Battery profiles menu
    STATE_WIFI_CONFIG = 12             // WiFi configuration
} app_state_t;

extern screen_id_t current_screen_id;
extern app_state_t current_app_state;

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
void create_screen_3(void); //screen 3 - charging started
void create_screen_12(void); //screen 12 - wifi configs

// Screen management functions
void initialize_all_screens(void);
void switch_to_screen(screen_id_t screen_id);
void update_current_screen(void);
screen_id_t determine_screen_from_state(void);
void update_screen_based_on_state(void);

// Table and UI update functions
void update_table_values(void);
void update_m2_state_display(void);

// M2 State Management Functions
void updateM2State(m2_state_t new_state);
void updateM2ConnectionStatus(bool connected);
const m2_state_config_t* getM2StateConfig(m2_state_t state);

#endif // SCREEN_DEFINITIONS_H
