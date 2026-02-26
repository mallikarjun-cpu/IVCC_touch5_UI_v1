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
    String batteryName;         // User given name for UI; max 32 chars recommended
    String displayName;        // Display name for UI (e.g. "24V LA 20Ah")

public:
    // Constructor; name optional — if blank/null, batteryName becomes "<un_named warning>"
    BatteryType(BatteryChemistry chem, uint16_t voltage, uint16_t ah,
                float cutoff, float current, const char* name = nullptr)
        : chemistry(chem), ratedVoltage(voltage), ratedAh(ah),
          cutoffVoltage(cutoff), constCurrent(current) {
        if (name != nullptr && name[0] != '\0') {
            batteryName = String(name);
        } else {
            batteryName = "<un_named warning>";
        }
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
    String getBatteryName() const { return batteryName; }

    // Setters (for manual configuration)
    void setCutoffVoltage(float voltage) { cutoffVoltage = voltage; }
    void setConstCurrent(float current) { constCurrent = current; }
    // Set battery name; blank/empty is stored as "<un_named warning>". Max 32 chars recommended.
    void setBatteryName(const String& name) {
        String t = name;
        t.trim();
        batteryName = (t.length() == 0) ? "<un_named warning>" : name;
    }
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
            String bn = profile->getBatteryName();
            bn.trim();
            if (bn.length() == 0) {
                profile->setBatteryName("<un_named warning>");
            }
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

#endif // BATTERY_TYPES_H