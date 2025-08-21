#include <Arduino.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"

// Button pin definitions
const int buttonPin1 = 18;
const int buttonPin2 = 19;

// Global CAN handler
CANHandler canHandler;

// Motors
Motor motor1(0x01, canHandler, Debug);
Motor motor2(0x02, canHandler, Debug);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  canHandler.setupCAN();
  Serial.println("CAN bus initialized.");

  motor1.start();
  motor2.start();
  motor1.reZero();
  motor2.reZero();
  Serial.println("Motors re-zeroed.");

  // Use INPUT if buttons are wired to VCC (HIGH when pressed)
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  delay(100);
  Serial.println("Ready to read button inputs...");
}

void loop() {
  canHandler.update();

  // Read button states
  bool pressed1 = digitalRead(buttonPin1) == HIGH;
  bool pressed2 = digitalRead(buttonPin2) == HIGH;

  if (pressed1) {
    Serial.println("Button 1 Pressed: REVERSE");
    motor1.sendCommand(0.0, 0.0, 0.0, 0.6, -4.0);
    motor2.sendCommand(0.0, 0.0, 0.0, 0.6, -4.0);
  } else if (pressed2) {
    Serial.println("Button 2 Pressed: FORWARD");
    motor1.sendCommand(0.0, 0.0, 0.0, 0.6, 4.0);
    motor2.sendCommand(0.0, 0.0, 0.0, 0.6, 4.0);
  } else {
    motor1.stop()
    motor2.stop()
  }

  motor1.update();
  motor2.update();

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

  delay(20);  // small debounce and loop delay
}