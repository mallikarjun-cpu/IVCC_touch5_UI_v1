#ifndef CAN_TWAI_H
#define CAN_TWAI_H

#include <Arduino.h>
#include <driver/twai.h>

// CAN Configuration
#define CAN_TX_PIN      GPIO_NUM_15  // ESP32-S3 CAN TX pin
#define CAN_RX_PIN      GPIO_NUM_16  // ESP32-S3 CAN RX pin

// CAN Frame IDs
#define STARTUP_FRAME_ID    0x901
#define SENSOR_FRAME_ID     0x100  // Example sensor frame ID

// Function declarations
bool init_can_twai(void);
bool send_can_frame(uint32_t id, uint8_t* data, uint8_t length);
bool receive_can_frame(twai_message_t* message);
void can_task(void* parameter);

#endif // CAN_TWAI_H
