
#include "rs485_vfdComs.h"
#include <HardwareSerial.h>
#include <cstring>

// Use Serial2 for RS485 communication (pins 44=TX, 43=RX)
extern HardwareSerial Serial2;

/**
 * @brief  CRC calculation for VFD Modbus RTU
 * @param  buffer: Pointer to data buffer
 * @param  length: Length of data (excluding CRC bytes)
 * @retval 16-bit CRC value
 */
uint16_t rs485_calculate_crc(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t carryFlag;

    for (int i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            carryFlag = crc & 0x0001;
            crc >>= 1;
            if (carryFlag) {
                crc ^= 0xA001;
            }
        }
    }
    return crc;
}

void rs485_sendModbusCommand(const uint8_t* packet, int length, const char* description) {
    // Transmit packet using Arduino Serial2 (hardware handles DE/RE automatically)
    Serial2.write((uint8_t*)packet, length);
    Serial2.flush();  // Ensure transmission completes
}

/**
 * @brief  Send VFD Start command
 * @retval None
 */
void rs485_sendStartCommand(void) {
    // Start command packet: {VFD_ADDRESS, 0x06, 0xC7, 0x38, 0x00, 0x01, 0xF4, 0xB3}
    std::array<uint8_t, 8> controlPacket = {VFD_ADDRESS, 0x06, 0xC7, 0x38, 0x00, 0x01, 0xF4, 0xB3};
    rs485_sendModbusCommand(controlPacket.data(), 8, "Start command");
}

/**
 * @brief  Send VFD Stop command
 * @retval None
 */
void rs485_sendStopCommand(void) {
    // Stop command packet: {VFD_ADDRESS, 0x06, 0xC7, 0x38, 0x00, 0x05, 0xF5, 0x70}
    std::array<uint8_t, 8> controlPacket = {VFD_ADDRESS, 0x06, 0xC7, 0x38, 0x00, 0x05, 0xF5, 0x70};
    rs485_sendModbusCommand(controlPacket.data(), 8, "Stop command");
}

/**
 * @brief  Send VFD Frequency command
 * @param  frequency_0_01hz: Target frequency in 0.01Hz units (e.g., 1000 = 10.00 Hz, 1050 = 10.50 Hz)
 * @retval None
 * @note   VFD register 51001 (0xC739) expects frequency in 0.01Hz units
 *         This provides fine-grained control (0.01Hz resolution)
 */
void rs485_sendFrequencyCommand(uint16_t frequency_0_01hz) {
    // VFD expects frequency in 0.01Hz units directly
    // Extract frequency bytes (big-endian)
    uint8_t F_UP_BYTE = (frequency_0_01hz >> 8) & 0xFF;
    uint8_t F_LOW_BYTE = frequency_0_01hz & 0xFF;

    // Build base packet (without CRC)
    // Register address: 0xC739 (51001 decimal) - VFD frequency command register
    std::array<uint8_t, 6> basePacket = {VFD_ADDRESS, 0x06, 0xC7, 0x39, F_UP_BYTE, F_LOW_BYTE};

    // Calculate CRC for base packet
    uint16_t crc = rs485_calculate_crc(basePacket.data(), 6);

    // Build complete packet with CRC using memcpy for efficiency
    std::array<uint8_t, 8> turnPacket;
    memcpy(turnPacket.data(), basePacket.data(), 6);
    turnPacket[6] = crc & 0xFF;        // CRC low byte
    turnPacket[7] = (crc >> 8) & 0xFF; // CRC high byte

    // Send command
    rs485_sendModbusCommand(turnPacket.data(), 8, "Frequency command");

    // Debug print - show frequency in Hz using String for better C++ style
    float frequency_hz = frequency_0_01hz / 100.0f;
    String msg = "Frequency command sent: " + String(frequency_0_01hz) +
                 " (0.01Hz units) = " + String(frequency_hz, 2) + " Hz";
    Serial.println(msg);
}

/**
 * @brief  Calculate frequency for Constant Current (CC) mode
 * @param  current_frequency: Current frequency in 0.01Hz units
 * @param  target_current: Target current in 0.01A units (e.g., 1000 = 10.00A)
 * @param  actual_current: Actual measured current in 0.01A units
 * @return New frequency in 0.01Hz units
 * @note   Proportional control with adaptive step sizes based on error magnitude
 */
uint16_t rs485_CalcFrequencyFor_CC(uint16_t current_frequency, uint16_t target_current, uint16_t actual_current) {
    int32_t current_error = (int32_t)actual_current - (int32_t)target_current;
    uint32_t abs_error = RS485_ABS(current_error);

    int16_t frequency_offset = 0;
    int16_t new_frequency = (int16_t)current_frequency;

    // Calculate error percentage (0-100%)
    // Error % = (abs_error / target_current) * 100
    // Handle division by zero and very small targets
    uint32_t error_percent = 0;
    if (target_current > 0) {
        // Calculate percentage: (abs_error * 100) / target_current
        error_percent = (abs_error * 100) / target_current;
    } else {
        // If target is 0, treat as 100% error
        error_percent = 100;
    }
    
    // Clamp to 100% max
    if (error_percent > 100) {
        error_percent = 100;
    }

    // Current is too low - need to increase frequency
    if (current_error < 0) {
        if (error_percent >= 90 || actual_current == 0) {
            frequency_offset = RS485_CALC_FREQ_COND05;  // Very large step: 2.00 Hz (90-100% error)
        }
        else if (error_percent >= 70) {
            frequency_offset = RS485_CALC_FREQ_COND04;  // Large step: 1.00 Hz (70-90% error)
        }
        else if (error_percent >= 50) {
            frequency_offset = RS485_CALC_FREQ_COND01;  // Medium-large step: 0.50 Hz (50-70% error)
        }
        else if (error_percent >= 30) {
            frequency_offset = RS485_CALC_FREQ_COND02;  // Medium step: 0.30 Hz (30-50% error)
        }
        else if (error_percent >= 10) {
            frequency_offset = RS485_CALC_FREQ_COND03;  // Small step: 0.10 Hz (10-30% error)
        }
        else {
            frequency_offset = RS485_CALC_FREQ_COND00;  // Tiny step: 0.01 Hz (fine control, <10% error)
        }
    }
    // Current is too high - need to decrease frequency
    else if (current_error > 0) {
        if (error_percent >= 90) {
            frequency_offset = RS485_CALC_FREQ_COND05;  // Very large step: 2.00 Hz (90-100% error)
        }
        else if (error_percent >= 70) {
            frequency_offset = RS485_CALC_FREQ_COND04;  // Large step: 1.00 Hz (70-90% error)
        }
        else if (error_percent >= 50) {
            frequency_offset = RS485_CALC_FREQ_COND01;  // Medium-large step: 0.50 Hz (50-70% error)
        }
        else if (error_percent >= 30) {
            frequency_offset = RS485_CALC_FREQ_COND02;  // Medium step: 0.30 Hz (30-50% error)
        }
        else if (error_percent >= 10) {
            frequency_offset = RS485_CALC_FREQ_COND03;  // Small step: 0.10 Hz (10-30% error)
        }
        else {
            frequency_offset = RS485_CALC_FREQ_COND00;  // Tiny step: 0.01 Hz (fine control, <10% error)
        }
        frequency_offset = -frequency_offset;  // Make negative (decrease frequency)
    }

    // Apply frequency offset
    new_frequency += frequency_offset;

    // Clamp frequency between MIN and MAX limits
    new_frequency = RS485_MAX(new_frequency, RS485_FREQ_MIN);
    new_frequency = RS485_MIN(new_frequency, RS485_FREQ_MAX);

    return (uint16_t)new_frequency;
}

/**
 * @brief  Calculate frequency for Constant Voltage (CV) mode
 * @param  current_frequency: Current frequency in 0.01Hz units
 * @param  target_voltage: Target voltage in 0.01V units (e.g., 1200 = 12.00V)
 * @param  actual_voltage: Actual measured voltage in 0.01V units
 * @return New frequency in 0.01Hz units
 * @note   Proportional control with adaptive step sizes based on error magnitude
 */
uint16_t rs485_CalcFrequencyFor_CV(uint16_t current_frequency, uint16_t target_voltage, uint16_t actual_voltage) {
    int32_t voltage_error = (int32_t)actual_voltage - (int32_t)target_voltage;
    uint32_t abs_error = RS485_ABS(voltage_error);

    int16_t frequency_offset = 0;
    int16_t new_frequency = (int16_t)current_frequency;

    // Voltage is too low - need to increase frequency (forward control)
    if (voltage_error < 0) {
        if (abs_error >= RS485_ERROR_VOLTAGE_LARGE) {
            frequency_offset = RS485_CALC_FREQ_COND01;  // Large step: 0.50 Hz
        }
        else if (abs_error >= RS485_ERROR_VOLTAGE_MID) {
            frequency_offset = RS485_CALC_FREQ_COND02;  // Medium step: 0.25 Hz
        }
        else if (abs_error >= RS485_ERROR_VOLTAGE_SMALL) {
            frequency_offset = RS485_CALC_FREQ_COND03;  // Small step: 0.10 Hz
        }
        else {
            frequency_offset = RS485_CALC_FREQ_COND00;  // Tiny step: 0.01 Hz (fine control)
        }
    }
    // Voltage is too high - need to decrease frequency (reverse control)
    else if (voltage_error > 0) {
        if (abs_error >= RS485_ERROR_VOLTAGE_REV) {
            frequency_offset = RS485_CALC_FREQ_COND11;  // Large step: 0.50 Hz
        }
        else if (new_frequency > RS485_CALC_FREQ_COND12) {
            frequency_offset = RS485_CALC_FREQ_COND12;  // Medium step: 0.20 Hz
        }
        else {
            frequency_offset = RS485_CALC_FREQ_COND13;  // Tiny step: 0.01 Hz (fine control)
        }
        frequency_offset = -frequency_offset;  // Make negative (decrease frequency)
    }

    // Apply frequency offset
    new_frequency += frequency_offset;

    // Clamp frequency between MIN and MAX limits
    new_frequency = RS485_MAX(new_frequency, RS485_FREQ_MIN);
    new_frequency = RS485_MIN(new_frequency, RS485_FREQ_MAX);

    return (uint16_t)new_frequency;
}

/**
 * @brief  Initialize RS485 interface for Arduino (TX only, hardware DE/RE)
 * @retval None
 */
void rs485_init(void) {
    // Initialize Serial2 for RS485 communication
    Serial2.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX, RS485_TX);

    // Small delay for initialization
    delay(100);

    // Send startup message on RS485 bus
    const char* startup_msg = "hello from touchUI 5 inch";
    Serial2.print(startup_msg);
    Serial2.flush();

    Serial.println("RS485 interface initialized (TX only)");
    Serial.print("TX Pin: ");
    Serial.print(RS485_TX);
    Serial.print(", RX Pin: ");
    Serial.print(RS485_RX);
    Serial.print(", Baud Rate: ");
    Serial.println(RS485_BAUD_RATE);
}
