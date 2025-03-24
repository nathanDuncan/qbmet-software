#include <Arduino.h>
#include "CANHandler.h"
#include "Motor.h"
#include "RemoteDebug.h"

// Create a global CAN handler
CANHandler canHandler;

// Create two Motor objects, one for ID=1 and one for ID=2
Motor motor1(0x01, canHandler, Debug);
Motor motor2(0x02, canHandler, Debug);

void setup() {
  Serial.begin(115200);
  delay(10000); // Give time to open Serial Monitor
  Serial.println("Beginning program...");

  Debug.begin("ESP32_Motor_Controller");

  // Initialize the CAN bus
  canHandler.setupCAN();
  Serial.println("CAN bus initialized.");

  // Start motor1 in MIT mode
  motor1.start();
  Serial.println("Motor 1 started.");

  // Start motor2 in MIT mode
  motor2.start();
  Serial.println("Motor 2 started.");

  // Re-zero motors
  motor1.reZero();
  motor2.reZero();
  Serial.println("Motors re-zeroed.");
  
  delay(100);
  Serial.println("Sending fixed commands to both motors...");
}

void loop() {
  // Update incoming CAN messages
  canHandler.update();

  // Send the same command to both motors
  // (position=0.0, velocity=0.0, kp=0.4, kd=0.0, t_ff=2.0)
  motor1.sendCommand(0.0, 0.0, 0.0, 0.6, 4.0);
  motor2.sendCommand(0.0, 0.0, 0.0, 0.6, 4.0);

  // Update each motor's outgoing messages
  motor1.update();
  motor2.update();

  // Print telemetry for Motor 1
  Debug.println("==== Motor 1 Telemetry ====");
  Debug.printf("Position   : %f\n", motor1.getPosition());
  Debug.printf("Velocity   : %f\n", motor1.getVelocity());
  Debug.printf("Torque     : %f\n", motor1.getTorque());
  Debug.printf("Temperature: %d\n", motor1.getTemperature());
  Debug.printf("Error Code : %d\n", motor1.getErrorCode());
  Debug.printf("Online     : %s\n", motor1.isOnline() ? "Yes" : "No");

  // Print telemetry for Motor 2
  Debug.println("==== Motor 2 Telemetry ====");
  Debug.printf("Position   : %f\n", motor2.getPosition());
  Debug.printf("Velocity   : %f\n", motor2.getVelocity());
  Debug.printf("Torque     : %f\n", motor2.getTorque());
  Debug.printf("Temperature: %d\n", motor2.getTemperature());
  Debug.printf("Error Code : %d\n", motor2.getErrorCode());
  Debug.printf("Online     : %s\n", motor2.isOnline() ? "Yes" : "No");
  Debug.println("============================");

  delay(2500);  // Adjust delay to suit your application
}
