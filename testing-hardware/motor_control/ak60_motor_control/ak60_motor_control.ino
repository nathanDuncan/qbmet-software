/*
  Control Cubemars AK Series Motor via CAN Bus using Raspberry Pi Pico W and MCP2515

  Hardware Connections:

  Raspberry Pi Pico W      MCP2515         CAN Transceiver (e.g., TJA1050)
  ------------------------------------------------------------------------
  3.3V ------------------- VCC
  GND ------------------- GND
  GPIO 2 (SPI SCK) -------- SCK
  GPIO 3 (SPI MOSI) ------ MOSI
  GPIO 4 (SPI MISO) ------ MISO
  GPIO 5 ----------------- CS
  GPIO 6 ----------------- INT

  CANH and CANL connected to the motor's CAN bus

  Note: Ensure that you have proper CAN bus termination (120 Ohm resistors) at both ends if necessary.
*/

#include <SPI.h>
#include <mcp2515_can.h>
#include <MsTimer2.h> // Optional: Use if periodic tasks are needed

/* SAMD core */
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// CAN Speed Configuration
#define CAN_SPEED CAN_1MHZ

// Value Limits based on Motor Specifications
#define P_MIN   -95.5f
#define P_MAX    95.5f
#define V_MIN   -30.0f
#define V_MAX    30.0f
#define KP_MIN   0.0f
#define KP_MAX  500.0f
#define KD_MIN   0.0f
#define KD_MAX    5.0f
#define T_MIN   -18.0f
#define T_MAX    18.0f

// Control Parameters
float p_in = 0.0f;     // Desired Position
float v_in = 0.0f;     // Desired Velocity
float kp_in = 100.0f;  // Proportional Gain
float kd_in = 1.0f;    // Derivative Gain
float current = 50.0f; // Desired Current

// Measured Values - Responses from the Motor
float p_out = 0.0f;  // Actual Position
float v_out = 0.0f;  // Actual Velocity
float t_out = 0.0f;  // Actual Torque

// MCP2515 CAN Controller Configuration
const int SPI_CS_PIN = 5;    // Chip Select pin connected to MCP2515 CS
const int CAN_INT_PIN = 6;   // Interrupt pin connected to MCP2515 INT

// Initialize MCP_CAN object
mcp2515_can CANbus(SPI_CS_PIN);

// Drive ID Configuration
const uint8_t DRIVE_ID = 0x01;               // Example Drive ID (Change as per your motor)
const uint32_t CAN_ID = 0x00 + DRIVE_ID;      // Full CAN ID: 0x00 + Drive ID = 0x01

void setup() {
  // Initialize Serial for debugging
  SERIAL.begin(115200);
  while (!SERIAL) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  SERIAL.println("Initializing CAN bus...");

  // Initialize MCP2515 CAN controller
  // Parameters:
  // CAN_SPEED: 1 MHz
  // MCP_8MHZ: set MCP2515 oscillator frequency (change to MCP_16MHZ if using 16MHz crystal)
  if (CANbus.begin(MCP_ANY, CAN_SPEED, MCP_8MHZ) == CAN_OK) {
    SERIAL.println("MCP2515 Initialized Successfully!");
  } else {
    SERIAL.println("Error Initializing MCP2515...");
    while (1); // Stop execution
  }

  // Set MCP2515 to normal mode
  CANbus.setMode(MCP_NORMAL);

  SERIAL.println("CAN bus initialized. Ready to send messages.");

  // Initialize Motor Modes
  Zero();
  EnterMode();

  // Optional: Initialize Timer for periodic tasks
  // MsTimer2::set(1000, periodicTask); // Set to 1 second
  // MsTimer2::start();
}

void loop() {
  // Handle Serial Input for User Commands
  if (SERIAL.available() > 0) {
    char rc = SERIAL.read();
    SERIAL.println(rc);
    handleUserInput(rc);
  }

  // Send CAN Command
  sendCANCommand();

  // Receive CAN Messages
  receiveCANMessages();

  // Print Data to Serial Monitor
  SERIAL.print("Desired Pos: ");
  SERIAL.print(p_in);
  SERIAL.print(" | Actual Pos: ");
  SERIAL.print(p_out);
  SERIAL.print(" | Desired Vel: ");
  SERIAL.print(v_in);
  SERIAL.print(" | Actual Vel: ");
  SERIAL.print(v_out);
  SERIAL.print(" | Current: ");
  SERIAL.println(t_out);

  delay(1000); // Wait for 1 second before next loop
}

// Function to handle user input from Serial
void handleUserInput(char cmd) {
  switch (cmd) {
    case 'u':
      v_in += 3.0f;
      v_in = constrain(v_in, V_MIN, V_MAX);
      SERIAL.println("Increased velocity by 3.0");
      break;
    case 'd':
      v_in -= 3.0f;
      v_in = constrain(v_in, V_MIN, V_MAX);
      SERIAL.println("Decreased velocity by 3.0");
      break;
    case 's':
      EnterMode();
      SERIAL.println("Entered Operation Mode");
      break;
    case 'e':
      ExitMode();
      SERIAL.println("Exited Operation Mode");
      break;
    default:
      SERIAL.println("Unknown command");
      break;
  }
}

// Function to send CAN Command
void sendCANCommand() {
  byte motorCommand[8];
  pack_cmd(motorCommand, p_in, v_in, kp_in, kd_in, current);

  // Send the CAN message
  byte sendStatus = CANbus.sendMsgBuf(CAN_ID, 0, 8, motorCommand);

  if (sendStatus == CAN_OK) {
    SERIAL.print("Sent CAN message to ID 0x");
    SERIAL.println(CAN_ID, HEX);
  } else {
    SERIAL.print("Error Sending CAN message. Error Code: ");
    SERIAL.println(sendStatus);
  }
}

// Function to receive CAN Messages
void receiveCANMessages() {
  if (CANbus.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    byte len = 0;
    byte rxBuf[8];

    // Read the message
    CANbus.readMsgBuf(&rxId, &len, rxBuf);

    SERIAL.print("Received CAN message from ID 0x");
    SERIAL.println(rxId, HEX);
    SERIAL.print("Data: ");
    for(int i = 0; i < len; i++) {
      SERIAL.print(rxBuf[i], HEX);
      SERIAL.print(" ");
    }
    SERIAL.println();

    // Unpack the received data if it's from the motor's response
    if (rxId == CAN_ID) {
      unpack_reply(rxBuf);
    }
  }
}

// Function to enter operation mode
void EnterMode() {
  byte buf[8];
  memset(buf, 0xFF, 7);
  buf[7] = 0xFC;
  CANbus.sendMsgBuf(CAN_ID, 0, 8, buf);
}

// Function to exit operation mode
void ExitMode() {
  byte buf[8];
  memset(buf, 0xFF, 7);
  buf[7] = 0xFD;
  CANbus.sendMsgBuf(CAN_ID, 0, 8, buf);
}

// Function to zero the motor position
void Zero() {
  byte buf[8];
  memset(buf, 0xFF, 7);
  buf[7] = 0xFE;
  CANbus.sendMsgBuf(CAN_ID, 0, 8, buf);
}

// Function to convert float to unsigned integer based on range and bits
unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if(bits == 12){
    pgg = (unsigned int) ((x - offset) * 4095.0 / span);
  }
  if(bits == 16){
    pgg = (unsigned int) ((x - offset) * 65535.0 / span);
  }
  return pgg;
}

// Function to convert unsigned integer to float based on range and bits
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// Converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float) x_int) * span / 4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float) x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

// Function to pack command data into CAN message
void pack_cmd(byte *data, float p_des, float v_des, float kp, float kd, float t_ff){
  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -95.5 and 95.5
  /// 12 bit velocity command, between -30 and +30 rad/s
  /// 12 bit KP, between 0 and 500 N-m/rad
  /// 12 bit KD, between 0 and 5 N-m*s/rad
  /// 12 bit current, between -18 and 18 A
  /// CAN packet is 8 8-bit words
  /// Formatted as follows. For each quantity, bit i0 is LSB
  /// DATA[0]: [position[15-8]]
  /// DATA[1]: [position[7-0]]
  /// DATA[2]: [velocity[11-4]]
  /// DATA[3]: [velocity[3-0], KP[11-8]]
  /// DATA[4]: [KP[7-0]]
  /// DATA[5]: [KD[11-4]]
  /// DATA[6]: [KD[3-0], current[11-8]]
  /// DATA[7]: [current[7-0]]

  /// Limit data to be within bounds ///
  p_des = constrain(p_des, P_MIN, P_MAX);
  v_des = constrain(v_des, V_MIN, V_MAX);
  kp = constrain(kp, KP_MIN, KP_MAX);
  kd = constrain(kd, KD_MIN, KD_MAX);
  t_ff = constrain(t_ff, T_MIN, T_MAX);

  /// Convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// Pack ints into the CAN buffer ///
  data[0] = (p_int >> 8) & 0xFF;                                   // DATA[0]: Motor Position High Byte
  data[1] = p_int & 0xFF;                                          // DATA[1]: Motor Position Low Byte
  data[2] = (v_int >> 4) & 0xFF;                                   // DATA[2]: Motor Speed High Byte
  data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);       // DATA[3]: Motor Speed Low 4 bits + KP High 4 bits
  data[4] = kp_int & 0xFF;                                         // DATA[4]: KP Low Byte
  data[5] = (kd_int >> 4) & 0xFF;                                  // DATA[5]: KD High Byte
  data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);       // DATA[6]: KD Low 4 bits + Current High 4 bits
  data[7] = t_int & 0xFF;                                          // DATA[7]: Current Low Byte
}

// Function to unpack CAN reply
void unpack_reply(byte *buf) {
  /// CAN Reply Object Structure ///
  /// 16 bit position, between -95.5 and 95.5
  /// 12 bit velocity, between -30 and +30 rad/s
  /// 12 bit current, between -18 and 18 A
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows. For each quantity, bit 0 is LSB
  /// DATA[0]: [position[15-8]]
  /// DATA[1]: [position[7-0]]
  /// DATA[2]: [velocity[11-4]]
  /// DATA[3]: [velocity[3-0], current[11-8]]
  /// DATA[4]: [current[7-0]]
  /// DATA[5] to DATA[7]: Reserved or additional data

  // Unpack ints from CAN buffer
  unsigned int p_int = (buf[0] << 8) | buf[1];
  unsigned int v_int = (buf[2] << 4) | (buf[3] >> 4);
  unsigned int i_int = ((buf[3] & 0x0F) << 8) | buf[4];

  // Convert uints to floats
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
}
