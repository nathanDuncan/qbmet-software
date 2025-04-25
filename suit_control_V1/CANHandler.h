#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <ACAN_ESP32.h>

/*
 * CANHandler — lightweight wrapper around ACAN_ESP32
 * --------------------------------------------------
 * ‣ Call setupCAN(txPin, rxPin) from your sketch’s setup().
 *   If no pins are passed, TX defaults to GPIO 22 and RX to GPIO 21.
 * ‣ DESIRED_BIT_RATE is set to 1 Mbit s⁻¹; adjust if needed.
 * ‣ latestFrame[0‑3] holds the most recent frame for motor IDs 1‑4.
 */

class CANHandler {
public:
    CANHandler() = default;

    // Initialise CAN controller
    void setupCAN(uint8_t txPin = 22, uint8_t rxPin = 21);

    // Transmit a message
    void sendCANMessage(const CANMessage& message);

    // Poll CAN hardware; call this each loop()
    void update();

    // Convenience helpers
    bool       getIsOnline(uint8_t id);
    CANMessage getLatestFrame(uint8_t id);
    CANMessage getLatestMessage(uint32_t targetId);
    bool       isMessageOnline(uint32_t targetId, uint32_t timeout) const;

    // Last frame received for IDs 0‑3 (motor1‑motor4)
    CANMessage latestFrame[4];

private:
    static const uint32_t DESIRED_BIT_RATE = 1'000'000UL;  // 1 Mbps

    // Timestamp (ms) when each ID was last seen
    uint32_t lastRecievedTime[4] = {0, 0, 0, 0};

    // How long (ms) before we consider a device “offline”
    int recieveTimeout = 600;
};

#endif  // CAN_HANDLER_H
