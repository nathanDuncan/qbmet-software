#include "CANHandler.h"

CANHandler::CANHandler()
{
}

void CANHandler::setupCAN()
{
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
  // Make sure these match your actual wiring (adjust as needed)
  settings.mTxPin = GPIO_NUM_14;  // For example, if you are using D5 for TX
  settings.mRxPin = GPIO_NUM_27;  // And D4 for RX
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode == 0)
  {
    Serial.println("CAN initialized successfully.");
  }
  else
  {
    Serial.printf("CAN init error, code: %lu\n", errorCode);
  }
}

void CANHandler::sendCANMessage(const CANMessage &message)
{
  if (ACAN_ESP32::can.tryToSend(message))
  {
    // Successfully sent the message
  }
  else
  {
    Serial.println("CAN message send failed.");
  }
}

bool CANHandler::getIsOnline(uint8_t id)
{
  if (millis() - lastRecievedTime[id - 1] > recieveTimeout)
  {
    return false;
  }
  return true;
}

void CANHandler::update()
{
  CANMessage message;
  uint8_t count = 0;
  while (ACAN_ESP32::can.receive(message) && count < 4)
  {
    latestFrame[count] = message;
    lastRecievedTime[count] = millis();
    
    // Debug print each received message.
    Serial.printf("Received CAN message - ID: %lu, Data: ", message.id);
    for (uint8_t i = 0; i < message.len; i++) {
      Serial.printf("%02X ", message.data[i]);
    }
    Serial.println();
    
    count++;
  }
}


// NEW: Searches the latestFrame array for a message with the specified targetId.
CANMessage CANHandler::getLatestMessage(uint32_t targetId)
{
  CANMessage result;
  result.id = 0;  // Default to 0 if not found.
  for (int i = 0; i < 4; i++) {
    if (latestFrame[i].id == targetId) {
      result = latestFrame[i];
      break;
    }
  }
  return result;
}

// NEW: Returns true if a message with the specified targetId was received within the given timeout.
bool CANHandler::isMessageOnline(uint32_t targetId, uint32_t timeout) const {
    for (int i = 0; i < 4; i++) {
       if (latestFrame[i].id == targetId && (millis() - lastRecievedTime[i] <= timeout)) {
           return true;
       }
    }
    return false;
}

CANMessage CANHandler::getLatestFrame(uint8_t id)
{
  return latestFrame[id - 1];
}
