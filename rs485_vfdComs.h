
#ifndef RS485_VFDCOMS_H
#define RS485_VFDCOMS_H

#include <Arduino.h>
#include <stdint.h>
#include <array>

// Arduino-specific RS485 pin configuration
#define RS485_TX 44
#define RS485_RX 43

/* RS485 Modbus Configuration */
#define VFD_ADDRESS 0x01
#define RS485_BAUD_RATE 9600

/* Frequency to RPM conversion (adjust for different VFDs) */
/* 1Hz = 20 RPM for your VFD (change this value for other VFDs: 15, 25, 30, etc.) */
#define VFD_FREQ_TO_RPM_RATIO 20  // 1Hz = 20 RPM
#define VFD_FREQ_TO_RPM(freq_hz) ((freq_hz) * VFD_FREQ_TO_RPM_RATIO)  // Convert Hz to RPM
#define VFD_RPM_TO_FREQ(rpm) ((rpm) / (float)VFD_FREQ_TO_RPM_RATIO)   // Convert RPM to Hz

/* Frequency calculation constants - 3kW VFD Configuration */
/* Frequency step sizes (in 0.01Hz units) */
#define RS485_CALC_FREQ_COND05    (200)    // 2.00Hz - Very large error step (20A+)
#define RS485_CALC_FREQ_COND04    (100)    // 1.00Hz - Large error step (15-20A)
#define RS485_CALC_FREQ_COND01    (50)     // 0.50Hz - Medium-large error step (7-15A)
#define RS485_CALC_FREQ_COND02    (30)     // 0.30Hz - Medium error step (5-7A)
#define RS485_CALC_FREQ_COND03    (10)     // 0.10Hz - Small error step (2-5A)
#define RS485_CALC_FREQ_COND00    (1)      // 0.01Hz - Tiny error step (fine control, <2A)
#define RS485_CALC_FREQ_COND11    (50)     // 0.50Hz - CV reverse large
#define RS485_CALC_FREQ_COND12    (20)     // 0.20Hz - CV reverse medium
#define RS485_CALC_FREQ_COND13    (1)      // 0.01Hz - CV reverse small

/* Frequency limits (in 0.01Hz units) - 3kW: max 300Hz, min 30Hz */
#define RS485_FREQ_MAX            (30000)  // 300.00 Hz
//#define RS485_FREQ_MIN            (3000)   // 30.00 Hz
#define RS485_FREQ_MIN            (4500)   // 45.00 Hz

/* Error thresholds (in 0.01A or 0.01V units) - adjust based on your sensor scaling */
#define RS485_ERROR_CURRENT_VERY_LARGE (2000)  // 20.00A difference -> 200Hz step
#define RS485_ERROR_CURRENT_LARGE (1500)      // 15.00A difference -> 100Hz step
#define RS485_ERROR_CURRENT_MID   (700)       // 7.00A difference -> 50Hz step
#define RS485_ERROR_CURRENT_SMALL (500)       // 5.00A difference -> 30Hz step
#define RS485_ERROR_CURRENT_TINY  (200)       // 2.00A difference -> 10Hz step
#define RS485_ERROR_VOLTAGE_LARGE (400)    // 4.00V difference
#define RS485_ERROR_VOLTAGE_MID   (200)    // 2.00V difference
#define RS485_ERROR_VOLTAGE_SMALL (100)    // 1.00V difference
#define RS485_ERROR_VOLTAGE_REV   (600)    // 6.00V difference (CV reverse)

/* Helper macros for frequency calculation */
#define RS485_ABS(x) ((x) < 0 ? -(x) : (x))
#define RS485_MAX(a, b) ((a) > (b) ? (a) : (b))
#define RS485_MIN(a, b) ((a) < (b) ? (a) : (b))

/* Function declarations */
uint16_t rs485_calculate_crc(uint8_t *buffer, uint16_t length);
void rs485_sendModbusCommand(const uint8_t* packet, int length, const char* description);
void rs485_sendStartCommand(void);
void rs485_sendStopCommand(void);
void rs485_sendFrequencyCommand(uint16_t frequency_0_01hz);  // frequency in 0.01Hz units
void rs485_init(void);

/* Frequency calculation functions (PID-like proportional control) */
uint16_t rs485_CalcFrequencyFor_CC(uint16_t current_frequency, uint16_t target_current, uint16_t actual_current);
uint16_t rs485_CalcFrequencyFor_CV(uint16_t current_frequency, uint16_t target_voltage, uint16_t actual_voltage);

#endif /* RS485_VFDCOMS_H */
