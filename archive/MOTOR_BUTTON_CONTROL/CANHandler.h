#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <ACAN_ESP32.h>

class CANHandler
{
public:
    CANHandler();
    void setupCAN();
    void sendCANMessage(const CANMessage &message);
    void update();
    bool getIsOnline(uint8_t id);
    CANMessage latestFrame[4];
    CANMessage getLatestFrame(uint8_t id);

    CANMessage getLatestMessage(uint32_t targetId);

    bool isMessageOnline(uint32_t targetId, uint32_t timeout) const;

private:
    static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL;
    uint32_t lastRecievedTime[4] = {0, 0, 0, 0};
    int recieveTimeout = 600; // ms
};

#endif // CAN_HANDLER_H
