#include "Motor.h"

Motor::Motor(uint16_t ID, CANHandler &canHandler, RemoteDebug &Debug)
    : canID(ID), canHandler(canHandler), Debug(Debug)
{
  // Initialize the outgoing command frame.
  latestFrame.id = canID;
  latestFrame.ext = false;
  latestFrame.rtr = false;
  latestFrame.len = 8;
  for (int i = 0; i < 8; i++) {
      latestFrame.data[i] = 0xFF;
  }
  // Default to the "start MIT mode" command on creation
  latestFrame.data[7] = 0xFC;
}

void Motor::start()
{
  isStopped = false;
  CANMessage startFrame;
  startFrame.id = canID;
  startFrame.ext = false;
  startFrame.rtr = false;
  startFrame.len = 8;
  for (int i = 0; i < 7; i++) {
    startFrame.data[i] = 0xFF;
  }
  startFrame.data[7] = 0xFC; // Command to enter MIT mode

  if (ACAN_ESP32::can.tryToSend(startFrame))
  {
    debugI("MOTOR: Started MIT mode on ID: %d", canID);
  }
}

void Motor::stop()
{
  CANMessage endFrame;
  endFrame.id = canID;
  endFrame.ext = false;
  endFrame.rtr = false;
  endFrame.len = 8;
  for (int i = 0; i < 7; i++) {
    endFrame.data[i] = 0xFF;
  }
  endFrame.data[7] = 0xFD; // Command to stop motor

  if (ACAN_ESP32::can.tryToSend(endFrame))
  {
    debugI("MOTOR: Motor with ID %d stopped", canID);
  }
  isStopped = true;
}

void Motor::setPosition(float pos, float kp, float kd)
{
  sendCommand(pos, 0, kp, kd, 0);
}

void Motor::setVelocity(float vel, float kd)
{
  // Use the current measured position as the position setpoint to maintain continuity
  float currentPos = getPosition();
  sendCommand(currentPos, vel, 0, kd, 0);
}

void Motor::setTorque(float torque)
{
  sendCommand(0, 0, 0, 0, torque);
}

void Motor::sendCommand(float p_des, float v_des, float kp, float kd, float t_ff)
{
  packCommand(latestFrame, p_des, v_des, kp, kd, t_ff);
}

void Motor::reZero()
{
  CANMessage resetFrame;
  resetFrame.id = canID;
  resetFrame.ext = false;
  resetFrame.rtr = false;
  resetFrame.len = 8;
  for (int i = 0; i < 7; i++) {
    resetFrame.data[i] = 0xFF;
  }
  resetFrame.data[7] = 0xFE; // Command to re-zero motor

  if (ACAN_ESP32::can.tryToSend(resetFrame))
  {
    debugI("MOTOR: Motor with ID %d re-zeroed", canID);
  }
  else {
    debugI("MOTOR: Motor with ID %d unsuccessfully re-zeroed", canID);
  }
}

// ------------------ Helper Conversions ------------------

int Motor::floatToUInt(float x, float x_min, float x_max, unsigned int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int result = 0;

  if (bits == 12) {
    result = static_cast<unsigned int>((x - offset) * 4095.0 / span);
  }
  else if (bits == 16) {
    result = static_cast<unsigned int>((x - offset) * 65535.0 / span);
  }
  return result;
}

float Motor::uintToFloat(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  float result = 0.0;

  if (bits == 12) {
    result = static_cast<float>(x_int) * span / 4095.0 + offset;
  }
  else if (bits == 16) {
    result = static_cast<float>(x_int) * span / 65535.0 + offset;
  }
  return result;
}

// ------------------ Command Packing ------------------

void Motor::packCommand(CANMessage &msg, float p_des, float v_des, float kp, float kd, float t_ff)
{
  // Constrain values to the specified ranges.
  p_des = constrain(p_des, P_MIN, P_MAX);
  v_des = constrain(v_des, V_MIN, V_MAX);
  kp    = constrain(kp,    Kp_MIN, Kp_MAX);
  kd    = constrain(kd,    Kd_MIN, Kd_MAX);
  t_ff  = constrain(t_ff,  T_MIN,  T_MAX);

  // Convert floats to unsigned integers using the conversion functions.
  unsigned int p_int  = floatToUInt(p_des, P_MIN,   P_MAX, 16);
  unsigned int v_int  = floatToUInt(v_des, V_MIN,   V_MAX, 12);
  unsigned int kp_int = floatToUInt(kp,    Kp_MIN,  Kp_MAX, 12);
  unsigned int kd_int = floatToUInt(kd,    Kd_MIN,  Kd_MAX, 12);
  unsigned int t_int  = floatToUInt(t_ff,  T_MIN,   T_MAX, 12);

  // Pack integers into the CAN message data array:
  msg.data[0] = p_int >> 8;              // High 8 bits of position
  msg.data[1] = p_int & 0xFF;            // Low 8 bits of position
  msg.data[2] = v_int >> 4;              // High 8 bits of velocity
  msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  msg.data[4] = kp_int & 0xFF;           // Low 8 bits of kp
  msg.data[5] = kd_int >> 4;             // High 8 bits of kd
  msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  msg.data[7] = t_int & 0xFF;            // Low 8 bits of torque
}

// ------------------ Command Unpacking ------------------

void Motor::unpackCommand(const CANMessage &msg)
{
  // Verify that we received an 8-byte message.
  if (msg.len < 8) {
    Serial.println("Error: Received CAN message is too short.");
    return;
  }

  // Layout for T-Motor/MIT-style protocol (works for any motor ID using the same data structure):
  // Bytes 0-1: Position (16 bits)
  // Bytes 2-3: Velocity (12 bits: upper 8 bits in data[2], high nibble in data[3])
  // Bytes 3-4: KP (12 bits)
  // Bytes 5-6: KD (12 bits)
  // Bytes 6-7: Torque/Current (12 bits)

  unsigned int positionInt = (msg.data[0] << 8) | msg.data[1];
  unsigned int velocityInt = (msg.data[2] << 4) | (msg.data[3] >> 4);

  unsigned int kp_int = ((msg.data[3] & 0x0F) << 8) | msg.data[4];
  unsigned int kd_int = (msg.data[5] << 4) | (msg.data[6] >> 4);
  unsigned int t_int  = ((msg.data[6] & 0x0F) << 8) | msg.data[7];

  // Debug prints to verify correct parsing
  // Serial.print("Received CAN message: ");
  for (uint8_t i = 0; i < msg.len; i++) {
    // Serial.printf("%02X ", msg.data[i]);
  }
  // Serial.println();

  // Serial.printf("Raw ints: Position=%u, Velocity=%u, KP=%u, KD=%u, Torque=%u\n",
                // positionInt, velocityInt, kp_int, kd_int, t_int);

  // Convert to Floats
  pOut = uintToFloat(positionInt, P_MIN, P_MAX, 16);
  vOut = uintToFloat(velocityInt, V_MIN, V_MAX, 12);
  iOut = uintToFloat(t_int,       T_MIN, T_MAX, 12);

  // Serial.printf("Converted: pOut=%f, vOut=%f, iOut=%f\n", pOut, vOut, iOut);
}

// ------------------ Update & Send ------------------

void Motor::update()
{
  if (isStopped) {
    return;
  }
  // Retrieve the latest frame for this motor's ID
  CANMessage feedbackMsg = canHandler.getLatestMessage(canID);
  if (feedbackMsg.id != 0) {
      unpackCommand(feedbackMsg);
  }

  // Periodically re-send the latest command frame
  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();
    if (!ACAN_ESP32::can.tryToSend(latestFrame)) {
      Debug.printf("MOTOR: CAN command failed, ID: %d\n", canID);
    }
  }
}

// ------------------ Getters ------------------

float Motor::getPosition() const   { return pOut; }
float Motor::getVelocity() const   { return vOut; }
float Motor::getTorque()   const   { return iOut; }
int   Motor::getTemperature() const { return temperature; }
uint8_t Motor::getErrorCode() const { return errorCode; }

bool Motor::isOnline() const {
  // Mark "online" if we receive a message for this motor ID within the last 600 ms.
  return canHandler.isMessageOnline(canID, 600);
}
