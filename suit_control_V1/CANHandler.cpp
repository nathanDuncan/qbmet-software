#include <Arduino.h>
#include "CANHandler.h"

// ─────────────────────────────────────────────────────────────
// NOTE:  Constructor definition no longer needed because the
//        header uses  = default;
//        Remove the old body you had here.
// ─────────────────────────────────────────────────────────────


// -------------------------------------------------------------
// CAN initialisation
// -------------------------------------------------------------
void CANHandler::setupCAN (uint8_t txPin, uint8_t rxPin) {

    ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);

    // Cast uint8_t → gpio_num_t
    settings.mTxPin = static_cast<gpio_num_t>(txPin);  // GPIO 22
    settings.mRxPin = static_cast<gpio_num_t>(rxPin);  // GPIO 21

    settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;

    const uint32_t errorCode = ACAN_ESP32::can.begin (settings);

    if (errorCode == 0) {
        Serial.println("CAN initialised successfully.");
    } else {
        Serial.printf ("CAN init failed, error code: %lu\n", errorCode);
    }
}

// -------------------------------------------------------------
// Message transmit
// -------------------------------------------------------------
void CANHandler::sendCANMessage (const CANMessage &message) {
    if (!ACAN_ESP32::can.tryToSend (message)) {
        Serial.println("CAN‑TX failed (buffer full?).");
    }
}

// -------------------------------------------------------------
// Poll CAN controller, cache most‑recent frames
// -------------------------------------------------------------
void CANHandler::update () {
    CANMessage message;
    uint8_t    count = 0;

    while (ACAN_ESP32::can.receive (message) && count < 4) {
        latestFrame[count]      = message;
        lastRecievedTime[count] = millis();

        // Optional debug print
        // Serial.printf("RX ‑ ID: 0x%lX  Data:", message.id);
        for (uint8_t i = 0; i < message.len; ++i) {
            // Serial.printf(" %02X", message.data[i]);
        }
        // Serial.println();

        ++count;
    }
}

// -------------------------------------------------------------
// Convenience helpers
// -------------------------------------------------------------
bool CANHandler::getIsOnline (uint8_t id) {
    return (millis() - lastRecievedTime[id - 1]) <= recieveTimeout;
}

CANMessage CANHandler::getLatestFrame (uint8_t id) {
    return latestFrame[id - 1];
}

CANMessage CANHandler::getLatestMessage (uint32_t targetId) {
    for (int i = 0; i < 4; ++i) {
        if (latestFrame[i].id == targetId) {
            return latestFrame[i];
        }
    }
    return CANMessage{};   // Empty/zeroed struct if not found
}

bool CANHandler::isMessageOnline (uint32_t targetId,
                                  uint32_t timeout) const {
    for (int i = 0; i < 4; ++i) {
        if (latestFrame[i].id == targetId &&
            (millis() - lastRecievedTime[i] <= timeout)) {
            return true;
        }
    }
    return false;
}
