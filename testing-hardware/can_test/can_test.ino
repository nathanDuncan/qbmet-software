#include <Arduino.h>
#include "CANHandler.h"
#include "RemoteDebug.h"

// Define CAN TX/RX pins
#define CAN_TX_PIN 22
#define CAN_RX_PIN 21

// Global CAN handler instance
CANHandler canHandler;

void setup() {
  Serial.begin(115200);
  delay(5000);

  Debug.begin("CANReceiveTest");

  Serial.println("Initializing CAN...");
  canHandler.setupCAN(CAN_TX_PIN, CAN_RX_PIN);
  Serial.println("CAN initialized. Waiting for motor messages...");
}

void loop() {
  canHandler.update();

  for (uint8_t motorID = 1; motorID <= 4; motorID++) {
    if (canHandler.getIsOnline(motorID)) {
      CANMessage msg = canHandler.getLatestFrame(motorID - 1);

      Serial.print("Motor ");
      Serial.print(motorID);
      Serial.print(" | ID: 0x");
      Serial.print(msg.id, HEX);
      Serial.print(" | Data: ");
      for (int i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  delay(100);  // Slow down output for readability
}
