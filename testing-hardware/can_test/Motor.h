#ifndef MOTOR_H
#define MOTOR_H

#include "ACAN_ESP32.h"
#include "CANHandler.h"
#include "RemoteDebug.h"

// Define control mode IDs if not already defined:
typedef enum {
  CAN_PACKET_SET_DUTY = 0,           // Duty Cycle Mode
  CAN_PACKET_SET_CURRENT,            // Current Loop Mode
  CAN_PACKET_SET_CURRENT_BRAKE,      // Current Brake Mode
  CAN_PACKET_SET_RPM,                // Speed Mode
  CAN_PACKET_SET_POS,                // Position Mode
  CAN_PACKET_SET_ORIGIN_HERE,        // Set Origin Mode
  CAN_PACKET_SET_POS_SPD             // Position-Speed Loop Mode
} CAN_PACKET_ID;

class Motor
{
public:
  Motor(uint16_t ID, CANHandler &canHandler, RemoteDebug &Debug);

  void start();
  void stop();
  void setPosition(float pos, float kp, float kd);
  void setVelocity(float vel, float kd);
  void setTorque(float torque);
  void reZero();

  // This sends a command using the existing conversion functions.
  void sendCommand(float p_des, float v_des, float kp, float kd, float t_ff);

  // NEW: Command brake current (Current Brake Mode)
  // The value (in A) will be scaled: 1 A = 1000 counts.
  void setCurrentBrake(float current);

  float getPosition() const;
  float getVelocity() const;
  float getTorque() const;
  int getTemperature() const;  // Changed to signed int so that subtraction works correctly.
  uint8_t getErrorCode() const;
  bool isOnline() const;

  void update();

private:
  uint16_t canID;
  float P_MIN = -40;
  float P_MAX = 40;
  float V_MIN = -50;
  float V_MAX = 50;
  float T_MIN = -25;
  float T_MAX = 25;
  float Kp_MIN = 0;
  float Kp_MAX = 500;
  float Kd_MIN = 0;
  float Kd_MAX = 5;

  float pOut = 0;
  float vOut = 0;
  float iOut = 0;
  int temperature = 0;         // Use a signed type for proper subtraction (e.g., int8_t or int)
  uint8_t errorCode = 0;
  bool isStopped;

  int floatToUInt(float x, float x_min, float x_max, unsigned int bits);
  float uintToFloat(int x_int, float x_min, float x_max, int bits);
  void packCommand(CANMessage &msg, float p_des, float v_des, float kp, float kd, float t_ff);
  void unpackCommand(const CANMessage &msg);

  CANMessage latestFrame;  // Used for the outgoing command frame
  CANHandler &canHandler;
  uint32_t lastSendTime = 0;
  int sendInterval = 1; // ms
  RemoteDebug &Debug;
};

#endif // MOTOR_H
