#include "battery_types.h"
#include <Arduino.h>
#include <lvgl.h>

// ============================================================================
// Helper Functions Implementation
// ============================================================================

// v4.60: Helper function to get battery chemistry name from profile
const char* getBatteryChemistryName(BatteryType *profile) {
    if (!profile) return "Unknown";

    switch(profile->getChemistry()) {
        case LITHIUM:   return "Lithium";
        case LEAD_ACID: return "Lead Acid";
        case LIFEPO4:   return "LiFePO4";
        default:        return "Unknown";
    }
}

// ============================================================================
// Global Battery Profile Manager Instance
// ============================================================================

BatteryProfileManager batteryProfiles;

// ============================================================================
// Initialize Battery Profiles Implementation
// ============================================================================

void initializeBatteryProfiles() {
    Serial.println("Initializing battery profiles...");
    //batteryProfiles.addProfile(new BatteryType(chemistry, voltage, ah, cutoff, current));
    //default battery for screen2 , 0volt
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 0, 0, 0.0, 0.0));
    // 12V Lead Acid batteries (cutoff 16V)
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 10, 16.0, 6.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 20, 15.0, 12.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 35, 15.0, 21.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 65, 16.0, 30.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 100, 16.2, 60.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 100, 16.2, 90.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 120, 16.2, 1200.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 150, 16.2, 1500.0));

    // 24V Lead Acid batteries (cutoff 33.2V) - also used for 28.8V rated hoist batteries
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 24, 10, 33.2, 6.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 24, 20, 33.2, 12.0));

    // 36V Lead Acid batteries (cutoff 48V)
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 36, 10, 48.0, 6.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 36, 20, 48.0, 12.0));

    // 48V Lead Acid batteries (cutoff 53.9V)
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 48, 280, 66, 6.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 48, 565, 66, 6.0));

    // 18V Lead Acid batteries (cutoff 21V) - Note: using LEAD_ACID chemistry for 18V
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 18, 10, 21.0, 6.0));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 18, 20, 21.0, 12.0));

    // 28V Lithium batteries
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 10, 31.0, 5.0));
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 20, 33.0, 9.0)); //ronin 2c
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 40, 33.0, 18.0)); //ronin 4c

    // 48V Lithium batteries (cutoff 53.9V)
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 5, 52, 3.0));
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 10, 53, 6.0)); //ronin 2c 
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 20, 53, 12.0)); //ronin 4c

    // 51V LiFePO4 batteries
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 51, 280, 58.4, 12.0));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 51, 565, 58.4, 12.0));

    Serial.print("Battery profiles initialized: ");
    Serial.print(batteryProfiles.getProfileCount());
    Serial.println(" profiles loaded");
}
