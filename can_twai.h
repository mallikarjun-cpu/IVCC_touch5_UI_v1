#ifndef CAN_TWAI_H
#define CAN_TWAI_H

#include <Arduino.h>
#include <driver/twai.h>

// CAN Configuration
#define CAN_TX_PIN      GPIO_NUM_15  // ESP32-S3 CAN TX pin
#define CAN_RX_PIN      GPIO_NUM_16  // ESP32-S3 CAN RX pin

// CAN Frame IDs
#define STARTUP_FRAME_ID        0x901
#define HANDSHAKE_FRAME_ID      0x100  // Handshake/Heartbeat
#define SENSOR_DATA_1_ID        0x101  // Voltage/Current
#define SENSOR_DATA_2_ID        0x102  // Temperature Channels
#define SENSOR_DATA_3_ID        0x103  // RTC Date/Time

// CAN Node IDs
#define M1_NODE_ID              0x01    // ESP32 Touch LCD
#define M2_NODE_ID              0x02    // STM32 Sensor Node

// CAN Message Types
#define MSG_HANDSHAKE_REQ       0x00
#define MSG_HANDSHAKE_ACK       0x01
#define MSG_HEARTBEAT           0x02

// Function declarations
bool init_can_twai(void);
bool send_can_frame(uint32_t id, uint8_t* data, uint8_t length);
bool receive_can_frame(twai_message_t* message);
void can_task(void* parameter);

#endif // CAN_TWAI_H
