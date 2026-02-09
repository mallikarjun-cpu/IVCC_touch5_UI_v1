#ifndef CAN_TWAI_H
#define CAN_TWAI_H

#include <Arduino.h>
#include <driver/twai.h>

// CAN Debug Level Control
// Set to 1 to enable all CAN debug prints, 0 to disable
#define CAN_DEBUG_LEVEL 0

// CAN Configuration
#define CAN_TX_PIN      GPIO_NUM_15  // ESP32-S3 CAN TX pin
#define CAN_RX_PIN      GPIO_NUM_16  // ESP32-S3 CAN RX pin

// CAN Frame IDs
#define STARTUP_FRAME_ID        0x901
#define HANDSHAKE_FRAME_ID      0x100  // Handshake/Heartbeat
#define SENSOR_DATA_1_ID        0x101  // Voltage/Current
#define SENSOR_DATA_2_ID        0x102  // Temperature Channels
#define SENSOR_DATA_3_ID        0x103  // RTC Date/Time
#define CONTACTOR_CONTROL_ID    0x105  // Contactor control (M1 -> M2)

// CAN Node IDs
#define M1_NODE_ID              0x01    // ESP32 Touch LCD
#define M2_NODE_ID              0x02    // STM32 Sensor Node

// CAN Message Types
#define MSG_HANDSHAKE_REQ       0x00
#define MSG_HANDSHAKE_ACK       0x01
#define MSG_HEARTBEAT           0x02

// Contactor control commands
#define CONTACTOR_CLOSE         0x4C  // Close contactor (ON)
#define CONTACTOR_OPEN          0x8B  // Open contactor (OFF)

// Function declarations
bool init_can_twai(void);
bool send_can_frame(uint32_t id, uint8_t* data, uint8_t length);
bool receive_can_frame(twai_message_t* message);
void can_task(void* parameter);
bool send_contactor_control(uint8_t command);  // Send contactor control command (0x4C = close, 0x8B = open)

#endif // CAN_TWAI_H
