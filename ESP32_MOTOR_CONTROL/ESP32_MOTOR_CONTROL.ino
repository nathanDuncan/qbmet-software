#include <Arduino.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"

// Create global instances (using the correct CAN IDs)
CANHandler canHandler;
Motor motor(0x01, canHandler, Debug);  // Use the feedback ID if that’s what your motor expects

void setup() {
  Serial.begin(115200);
  delay(10000);
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  canHandler.setupCAN();
  Serial.println("CAN bus initialized.");

  // Start the motor in MIT mode (enter motor control mode)
  motor.start();
  Serial.println("Motor started.");

  // Set the current motor position as zero
  motor.reZero();
  delay(100);

  Serial.println("Sending fixed command: velocity=0, torque=1.0, kp=0, kd=0.3, position=0");
}

void loop() {
  // Update CAN bus communications
  canHandler.update();

  // Send the fixed motor command with the specified parameters
  motor.sendCommand(0.0, 0.0, 0.0, 0.4, 2.0);

  // Update the motor command transmission
  motor.update();

  // Print telemetry for debugging
  Debug.printf("Position   : %f\n", motor.getPosition());
  Debug.printf("Velocity   : %f\n", motor.getVelocity());
  Debug.printf("Torque     : %f\n", motor.getTorque());
  Debug.printf("Temperature: %d\n", motor.getTemperature());
  Debug.printf("Error Code : %d\n", motor.getErrorCode());
  Debug.printf("Online     : %s\n", motor.isOnline() ? "Yes" : "No");

  delay(2500);  // Adjust the delay as needed
}
