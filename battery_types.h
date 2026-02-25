#ifndef BATTERY_TYPES_H
#define BATTERY_TYPES_H

#include <Arduino.h>
#include <lvgl.h>

// ============================================================0================
// Battery Type Class Definition
// ============================================================================
/*
This class represents different battery configurations for the charger.
- Battery types: Lithium, Lead Acid, LiFePO4
- Rated voltages: 12V, 18V, 24V, 28V, 36V, 48V, 51V, etc (up to 420V)
- Capacity: 2Ah to 565Ah
- Cutoff voltage: 16V to 480V (configurable per battery)
- Constant current: Configurable per battery
*/

enum BatteryChemistry {
    LITHIUM,
    LEAD_ACID,
    LIFEPO4
};

class BatteryType {
private:
    BatteryChemistry chemistry;
    uint16_t ratedVoltage;      // Rated voltage (12, 18, 24, 28, 36, 48, 51, etc)
    uint16_t ratedAh;           // Rated capacity in Ah (2-565)
    float cutoffVoltage;        // Upper cutoff voltage for CV mode
    float constCurrent;         // Target charging current in CC mode
    String displayName;         // Display name for UI

public:
    // Constructor
    BatteryType(BatteryChemistry chem, uint16_t voltage, uint16_t ah,
                float cutoff, float current)
        : chemistry(chem), ratedVoltage(voltage), ratedAh(ah),
          cutoffVoltage(cutoff), constCurrent(current) {
        
        // Generate display name
        String chemStr = (chemistry == LITHIUM) ? "Li" : 
                        (chemistry == LEAD_ACID) ? "LA" : "LFP";
        displayName = String(voltage) + "V " + chemStr + " " + String(ah) + "Ah";
    }

    // Getters
    BatteryChemistry getChemistry() const { return chemistry; }
    uint16_t getRatedVoltage() const { return ratedVoltage; }
    uint16_t getRatedAh() const { return ratedAh; }
    float getCutoffVoltage() const { return cutoffVoltage; }
    float getConstCurrent() const { return constCurrent; }
    String getDisplayName() const { return displayName; }
    
    // Setters (for manual configuration)
    void setCutoffVoltage(float voltage) { cutoffVoltage = voltage; }
    void setConstCurrent(float current) { constCurrent = current; }
};

// v4.60: Helper function to get battery chemistry name from profile
const char* getBatteryChemistryName(BatteryType *profile);

// ============================================================================
// Battery Profile Manager
// ============================================================================
class BatteryProfileManager {
private:
    static const int MAX_PROFILES = 50;
    BatteryType* profiles[MAX_PROFILES];
    int profileCount;

public:
    BatteryProfileManager() : profileCount(0) {}
    
    ~BatteryProfileManager() {
        for (int i = 0; i < profileCount; i++) {
            delete profiles[i];
        }
    }
    
    void addProfile(BatteryType* profile) {
        if (profileCount < MAX_PROFILES) {
            profiles[profileCount++] = profile;
        }
    }
    
    // Get profiles matching detected voltage range
    void getMatchingProfiles(float detectedVoltage, BatteryType** matches, int& matchCount) {
        matchCount = 0;
        
        // Voltage detection ranges
        if (detectedVoltage >= 9 && detectedVoltage < 16) {
            // Show 12V Lead Acid and 18V Lithium options
            for (int i = 0; i < profileCount; i++) {
                if ((profiles[i]->getRatedVoltage() == 12 && profiles[i]->getChemistry() == LEAD_ACID) ||
                    (profiles[i]->getRatedVoltage() == 18 && profiles[i]->getChemistry() == LITHIUM)) {
                    matches[matchCount++] = profiles[i];
                }
            }
        }
        else if (detectedVoltage >= 16 && detectedVoltage < 21) {
            // Show 18V Lithium options
            for (int i = 0; i < profileCount; i++) {
                if (profiles[i]->getRatedVoltage() == 18) {
                    matches[matchCount++] = profiles[i];
                }
            }
        }
        else if (detectedVoltage >= 21 && detectedVoltage < 33) {
            // Show 24V Lead Acid and 28V Lithium options
            for (int i = 0; i < profileCount; i++) {
                if ((profiles[i]->getRatedVoltage() == 24 && profiles[i]->getChemistry() == LEAD_ACID) ||
                    (profiles[i]->getRatedVoltage() == 28 && profiles[i]->getChemistry() == LITHIUM)) {
                    matches[matchCount++] = profiles[i];
                }
            }
        }
        else if (detectedVoltage >= 33 && detectedVoltage < 42) {
            // Show 36V options
            for (int i = 0; i < profileCount; i++) {
                if (profiles[i]->getRatedVoltage() == 36) {
                    matches[matchCount++] = profiles[i];
                }
            }
        }
        else if (detectedVoltage >= 42 && detectedVoltage <= 66) {
            // Show 48V Lead Acid and 51V LiFePO4 options
            for (int i = 0; i < profileCount; i++) {
                if ((profiles[i]->getRatedVoltage() == 48) ||
                    (profiles[i]->getRatedVoltage() == 51 && profiles[i]->getChemistry() == LIFEPO4)) {
                    matches[matchCount++] = profiles[i];
                }
            }
        }
        else {
            //return the 0 volt battery profile
            matches[matchCount++] = profiles[0];
        }
    }
    
    int getProfileCount() const { return profileCount; }
    BatteryType* getProfile(int index) const {
        if (index >= 0 && index < profileCount) {
            return profiles[index];
        }
        return nullptr;
    }
};

// ============================================================================
// Global Battery Profile Manager Declaration
// ============================================================================

extern BatteryProfileManager batteryProfiles;

// ============================================================================
// Initialize Battery Profiles Declaration
// ============================================================================

void initializeBatteryProfiles();

// ============================================================================
// M2 Sensor Node State Management
// ============================================================================

// M2 State Enum (10 states as requested)
typedef enum {
    M2_STATE_STANDBY = 0,      // Default standby state
    M2_STATE_INIT,             // Initializing
    // M2_STATE_READY,            // Ready for operation - commented out, not required
    // M2_STATE_CHARGING,         // Charging active - commented out, not required
    // M2_STATE_FULL,             // Battery full - commented out, not required
    // M2_STATE_ERROR,            // Error state - commented out, not required
    M2_STATE_DISCONNECTED,     // Disconnected
    // M2_STATE_FAULT,            // Fault detected - commented out, not required
    // M2_STATE_CALIBRATING,      // Calibrating - commented out, not required
    // M2_STATE_MAINTENANCE       // Maintenance mode - commented out, not required
} m2_state_t;

// M2 State Configuration Struct
typedef struct {
    m2_state_t state;
    const char* label_text;
    lv_color_t bg_color;
    lv_color_t border_color;
    const char* description;
} m2_state_config_t;

#endif // BATTERY_TYPES_H