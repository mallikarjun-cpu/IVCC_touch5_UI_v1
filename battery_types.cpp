#include "battery_types.h"
#include <Arduino.h>
#include <lvgl.h>

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
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 0, 0, 0.0, 0.0, "default"));
    // 12V Lead Acid batteries (cutoff 16V)
/*
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 12, 280, 14.5, 150.0, "1] Lifpo nishiarai 0.5c"));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 12, 280, 15.1, 150.0, "2] Lifpo nishiarai 0.5c"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 20, 16.0, 12.0, "Old Nishiarai 2f"));
*/
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 75, 16, 45.0, "[A] 8T 油圧ショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 80, 16, 48.0, "[B] 14T 油圧ショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 115, 16, 70.0, "[C] 20T 油圧ショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 75, 16, 45.0, "[D] 3.5T タイヤショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 160, 16, 96.0, "[E] 11T タイヤショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 195, 16, 117.0, "[F] 18T タイヤショベル"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 80, 16, 48.0, "[G] 4T コンベインドローラー"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 72, 16, 43.0, "[H] 10t タイヤローラー"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 72, 16, 43.0, "[I] 60KVA 発電機"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 80, 16, 48.0, "[J] 150KVA 発電機"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 130, 16, 78.0, "[K] 220KVA 発電機"));
/*

    // 24V Lead Acid batteries (cutoff 33.2V) - also used for 28.8V rated hoist batteries
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 24, 10, 31.2, 6.0, "big Nishiarai 2f"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 24, 20, 31.2, 12.0, "big2 Nishiarai 2f"));

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
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 4.5, 31.0, 5.0, "Ronin_OLD"));
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 4.5, 33.0, 9.0, "Ronin_OLD 2C")); //ronin 2c
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 28, 4.5, 33.0, 18.0, "Ronin_OLD 4C")); //ronin 4c

    // 48V Lithium batteries (cutoff 53.9V)
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 5, 52, 3.0));
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 10, 53, 6.0)); //ronin 2c 
    batteryProfiles.addProfile(new BatteryType(LITHIUM, 48, 20, 53, 12.0)); //ronin 4c

    // 51V LiFePO4 batteries
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 51, 280, 58.4, 12.0));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 51, 565, 58.4, 12.0));

*/

    Serial.print("Battery profiles initialized: ");
    Serial.print(batteryProfiles.getProfileCount());
    Serial.println(" profiles loaded");
}


/*
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 10, 16.0, 6.0, "Old Nishiarai 2f"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 20, 13.0, 12.0, "Low cutoff test"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 280, 15, 150.0, "Battery 10"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 20, 16.0, 12.0, "Battery 4"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 35, 16.0, 21.0, "Battery 5"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 65, 16.0, 30.0, "Battery 6"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 100, 16.2, 60.0, "Battery 7"));
    batteryProfiles.addProfile(new BatteryType(LEAD_ACID, 12, 280, 15, 100.0, "Battery 7"));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 12, 280, 14.0, 90.0, "LifPo nishiarai 0.4c"));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 12, 280, 14.0, 120.0, "LifPo nishiarai 0.4c"));
    batteryProfiles.addProfile(new BatteryType(LIFEPO4, 12, 280, 14.0, 150.0, "Lifpo nishiarai 0.5c"));
*/