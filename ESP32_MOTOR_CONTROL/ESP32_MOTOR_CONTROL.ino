#include <Arduino.h>
#include <Wire.h>  
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"

// -------------------------
// CAN‑bus TX / RX pins
// -------------------------
static const int CAN_TX_PIN = 22;   // D22  ➜ TX
static const int CAN_RX_PIN = 21;   // D21  ➜ RX

static const int I2C_SDA_PIN = 32;   // pick any free GPIOs
static const int I2C_SCL_PIN = 33;
// -------------------------

// Global CAN handler
CANHandler canHandler;

// Motors
Motor motor1(0x01, canHandler, Debug);
Motor motor2(0x03, canHandler, Debug);

// Cycle timing (milliseconds)
const unsigned long FORWARD_TIME = 1000;
const unsigned long REVERSE_TIME = 1000;
const unsigned long WAIT_TIME    = 5000;

// Define states for a simple state machine
enum MotorState { FORWARD, REVERSE, WAIT };

MotorState   currentState   = FORWARD;   // Start in FORWARD
unsigned long stateStartTime = 0;        // Timestamp when we switched states

void setup() {
  Serial.begin(115200);
  delay(10000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // ---------- CAN BUS INITIALISATION ----------
  // Use the new pin definitions
  canHandler.setupCAN(CAN_TX_PIN, CAN_RX_PIN);
  Serial.println("CAN bus initialized.");
  // --------------------------------------------

  // Initialise motors
  motor1.start();
  motor2.start();
  motor1.reZero();
  motor2.reZero();
  Serial.println("Motors re‑zeroed.");

  // Record the time we enter the first state
  stateStartTime = millis();
}

void loop() {
  // Update CAN and motor state
  canHandler.update();
  motor1.update();
  motor2.update();

  // Determine how long we've been in our current state
  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {
    case FORWARD:
      // Command motors forward
      motor1.sendCommand(0.0, 0.0, 0.0, 0.6,  3.0);
      motor2.sendCommand(0.0, 0.0, 0.0, 0.6,  3.0);

      if (elapsed >= FORWARD_TIME) {
        currentState   = REVERSE;
        stateStartTime = millis();
      }
      break;

    case REVERSE:
      // Command motors in reverse
      motor1.sendCommand(0.0, 0.0, 0.0, 0.6, -3.0);
      motor2.sendCommand(0.0, 0.0, 0.0, 0.6, -3.0);

      if (elapsed >= REVERSE_TIME) {
        currentState   = WAIT;
        stateStartTime = millis();
      }
      break;

    case WAIT:
      // Stop the motors during WAIT
      motor1.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
      motor2.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);

      if (elapsed >= WAIT_TIME) {
        currentState   = FORWARD;
        stateStartTime = millis();
      }
      break;
  }

  // Debug telemetry
  Debug.println("==== Motor 1 Telemetry ====");
  Debug.printf("Position   : %f\n", motor1.getPosition());
  Debug.printf("Velocity   : %f\n", motor1.getVelocity());
  Debug.printf("Torque     : %f\n", motor1.getTorque());
  Debug.printf("Temperature: %d\n", motor1.getTemperature());
  Debug.printf("Error Code : %d\n", motor1.getErrorCode());
  Debug.printf("Online     : %s\n", motor1.isOnline() ? "Yes" : "No");

  Debug.println("==== Motor 2 Telemetry ====");
  Debug.printf("Position   : %f\n", motor2.getPosition());
  Debug.printf("Velocity   : %f\n", motor2.getVelocity());
  Debug.printf("Torque     : %f\n", motor2.getTorque());
  Debug.printf("Temperature: %d\n", motor2.getTemperature());
  Debug.printf("Error Code : %d\n", motor2.getErrorCode());
  Debug.printf("Online     : %s\n", motor2.isOnline() ? "Yes" : "No");
  Debug.println("============================");

  // Short delay to keep loop timing manageable
  delay(1000);
}
