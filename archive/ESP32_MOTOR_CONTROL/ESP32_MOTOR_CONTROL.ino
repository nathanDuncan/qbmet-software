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

static const int I2C_SDA_PIN = 32;
static const int I2C_SCL_PIN = 33;
// -------------------------

// Global CAN handler
CANHandler canHandler;

// Motor
Motor motor1(0x02, canHandler, Debug);  // Make sure this is the correct ID!

// State machine timing
const unsigned long FORWARD_TIME = 1000;
const unsigned long REVERSE_TIME = 1000;
const unsigned long WAIT_TIME    = 5000;

enum MotorState { FORWARD, REVERSE, WAIT };
MotorState   currentState   = FORWARD;
unsigned long stateStartTime = 0;

// Re-send neutral command every second for debug
unsigned long lastNeutralPing = 0;
const unsigned long PING_INTERVAL = 1000;

// Track online state transition
bool wasOnline = false;

void setup() {
  Serial.begin(115200);
  delay(10000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Serial.println("Initializing CAN...");
  canHandler.setupCAN(CAN_TX_PIN, CAN_RX_PIN);
  Serial.println("CAN initialized.");

  motor1.start();
  Serial.println("MOTOR: Started MIT mode on ID: 2");
  motor1.reZero();
  Serial.println("MOTOR: Motor with ID 2 re-zeroed");
  Serial.println("Motors re‑zeroed.");

  stateStartTime = millis();
}

void loop() {
  // Update CAN and motor
  canHandler.update();
  motor1.update();

  unsigned long now = millis();

  // Debug: re-send neutral command every second
  if (now - lastNeutralPing >= PING_INTERVAL) {
    motor1.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
    Serial.println("↪ Sent neutral command to wake motor...");
    lastNeutralPing = now;
  }

  // State machine
  unsigned long elapsed = now - stateStartTime;

  switch (currentState) {
    case FORWARD:
      motor1.sendCommand(0.0, 0.0, 0.0, 0.6,  3.0);
      if (elapsed >= FORWARD_TIME) {
        currentState = REVERSE;
        stateStartTime = now;
      }
      break;

    case REVERSE:
      motor1.sendCommand(0.0, 0.0, 0.0, 0.6, -3.0);
      if (elapsed >= REVERSE_TIME) {
        currentState = WAIT;
        stateStartTime = now;
      }
      break;

    case WAIT:
      motor1.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
      if (elapsed >= WAIT_TIME) {
        currentState = FORWARD;
        stateStartTime = now;
      }
      break;
  }

  // Check online status and log changes
  bool isNowOnline = motor1.isOnline();
  if (isNowOnline && !wasOnline) {
    Serial.println("✅ Motor 1 just came online!");
  } else if (!isNowOnline && wasOnline) {
    Serial.println("⚠️ Motor 1 went offline!");
  }
  wasOnline = isNowOnline;

  // Debug telemetry
  Debug.println("==== Motor 1 Telemetry ====");
  Debug.printf("Position   : %f\n", motor1.getPosition());
  Debug.printf("Velocity   : %f\n", motor1.getVelocity());
  Debug.printf("Torque     : %f\n", motor1.getTorque());
  Debug.printf("Temperature: %d\n", motor1.getTemperature());
  Debug.printf("Error Code : %d\n", motor1.getErrorCode());
  Debug.printf("Online     : %s\n", isNowOnline ? "Yes" : "No");
  Debug.println("============================");

  delay(5000);  // Keep loop fast but stable
}
